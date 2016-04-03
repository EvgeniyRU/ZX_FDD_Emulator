/// ----------------------------------------------------------------------------------------------------------------
/// SD CARD MODULE Based on Petit FatFs
/// ----------------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <avr/io.h>
#include "SDCardModule.h"

static uint8_t CardType;
static uint8_t resp_buf[4];

/// receive SPI byte
static uint8_t spiRead()
{
  SPDR = 0xFF;
  loop_until_bit_is_set(SPSR, SPIF);
  return SPDR;
}
/// send SPI byte
static void spiSend(uint8_t d)
{
  SPDR = d;
  loop_until_bit_is_set(SPSR, SPIF);
}

// Send a command to CARD
static uint8_t send_cmd(uint8_t cmd, uint32_t param, uint8_t cnt)
{
  uint8_t i, res;

  DESELECT();
  if (cmd & 0x80)
  { // ACMD{X} is the command sequense of CMD55 + CMD{X}
    cmd &= 0x7F;
    res = send_cmd(CMD55, 0, 0);
    if (res > 1) return res;
  }

  /* Select the card */
  DESELECT();
  spiRead();
  SELECT();
  spiRead();


  spiSend(cmd);
  spiSend((uint8_t)(param >> 24));
  spiSend((uint8_t)(param >> 16));
  spiSend((uint8_t)(param >> 8));
  spiSend((uint8_t)param);

  i = 0x01;             /* Dummy CRC + Stop */
  if (cmd == CMD0) i = 0x95;      /* Valid CRC for CMD0(0) */
  if (cmd == CMD8) i = 0x87;      /* Valid CRC for CMD8(0x1AA) */
  spiSend(i);

  for( i = 0; i < 128; i++) if((res = spiRead()) != 0xFF) break;  // wait for response and get it
  for( i = 0; i < cnt; i++) resp_buf[i] = spiRead();    // read more response if not R1 command

  return res;             // Return with the response value
}


/// CARD Initialization
CSTATUS card_initialize (void)
{
  uint8_t cmd, n;

  CardType = 0;

  DESELECT();
  for (n = 10; n; n--) spiRead(); // wait some time

  if (send_cmd(CMD0, 0, 0) == 1)
  { // Entered Idle state
    if (send_cmd(CMD8, 0x1AA, 4) == 1)
    { // SDv2
      if (resp_buf[2] == 0x01 && resp_buf[3] == 0xAA)
      { // Supports voltage 2.7-3.6V        
        // Wait for leaving Idle state (ACMD41 with HCS bit)
        while( (n = send_cmd(ACMD41, 1UL << 30, 0)) == 1);
              if( n > 1 ) return STA_NOINIT;

        if (send_cmd(CMD58, 0, 4) == 0) // Check for SDHC type (if CCS bit set in the OCR)
          CardType = (resp_buf[0] & 0x40) ? CT_SD2 | CT_SDHC : CT_SD2;
      }
    }
    else
    { // SDv1 or MMC
      if (send_cmd(ACMD41, 0, 0) <= 1)
      { // SDv1
        CardType = CT_SD1;
        cmd = ACMD41;
      }
      else
      { // MMCv3
        CardType = CT_MMC;
        cmd = CMD1;
      }
      
      while( (n = send_cmd(cmd, 0, 0)) == 1);
      if( n > 1 ) return STA_NOINIT;

      if (send_cmd(CMD16, 512, 0) != 0) CardType = 0; // Set R/W block length to 512        
    }
  }

  DESELECT();
  spiRead();

  return CardType ? 0 : STA_NOINIT;
}

/// Read partial sector
CRESULT card_readp (
  void *dest,     // Pointer to the destination object to put data
  uint32_t lba,     // Start sector number (LBA)
  uint16_t ofs,     // Byte offset in the sector (0..511)
  uint16_t cnt      // Byte count (1..512), b15:destination flag
)
{
  CRESULT res;
  uint8_t *p, rc;
  uint16_t cf;

  if (!cnt || ofs + cnt > 512) return RES_PARERR; // wrong parameters

  if (!(CardType & CT_SDHC)) lba *= 512;    // SDHC - LBA = block number (512 bytes), other - LBA = byte offset

  res = RES_ERROR;

  if (send_cmd(CMD17, lba, 0) == 0)
  { // READ_SINGLE_BLOCK

    while( (rc = spiRead() ) == 0xFF ); // wait for CARD is ready to transmit block of data

    if (rc == 0xFE)
    { // receive block data
      cf = 512 + 2 - ofs - cnt;

      while (ofs--) 
      { // Skip bytes before offset
        SPDR = 0xFF;
        loop_until_bit_is_set(SPSR, SPIF);
      }

      p = (uint8_t*)dest;
      do
      {
        SPDR = 0xFF;
        loop_until_bit_is_set(SPSR, SPIF);
        *p++ = SPDR;
      } while (--cnt); // read data

      do {
        SPDR = 0xFF;
        loop_until_bit_is_set(SPSR, SPIF);
      } while (--cf);   // Skip rest of data and CRC      

      res = cnt ? RES_STRERR : RES_OK;
    }
  }

  DESELECT();
  SPDR = 0xFF;
  loop_until_bit_is_set(SPSR, SPIF);

  return cnt ? RES_ERROR : RES_OK;
}
  

/// Read full sector
void card_read_sector (
  void *dest,     // Pointer to the destination object to put data
  uint32_t lba     // Start sector number (LBA)
)
{
  uint8_t *p , rc, i;

  if (!(CardType & CT_SDHC)) lba *= 512;    // SDHC - LBA = block number (512 bytes), other - LBA = byte offset
  
  if (send_cmd(CMD17, lba, 0) == 0)
  { // READ_SINGLE_BLOCK
    while( (rc = spiRead() ) == 0xFF ); // wait for CARD is ready to transmit block of data
    
    if (rc == 0xFE)
    { // receive block data
      p = (uint8_t*)dest;
      i = 0, rc = 2;
      do { // read 512 bytes
        SPDR = 0xFF;
        loop_until_bit_is_set(SPSR, SPIF);
        *p++ = SPDR;
        i++;
        if(i == 0) rc--;
      } while(rc != 0);      
      // Skip CRC
      SPDR = 0xFF;
      loop_until_bit_is_set(SPSR, SPIF);
      SPDR = 0xFF;
      loop_until_bit_is_set(SPSR, SPIF);
    }
  }

  DESELECT();
  SPDR = 0xFF;
  loop_until_bit_is_set(SPSR, SPIF);
}
/// ----------------------------------------------------------------------------------------------------------------
