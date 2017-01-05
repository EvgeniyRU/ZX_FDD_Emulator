/// ----------------------------------------------------------------------------------------------------------------
/// SD CARD MODULE Based on Petit FatFs
/// ----------------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <avr/io.h>
#include "SDCardModule.h"

static uint8_t CardType;
static uint8_t resp_buf[4];
static cid_t CID;

uint8_t getCardType()
{
    return CardType;
}

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
uint8_t send_cmd(uint8_t cmd, uint32_t param, uint8_t cnt)
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

    i = 0xFF;                       // Dummy CRC + Stop bit
    if (cmd == CMD0) i = 0x95;      // Valid CRC for CMD0(0)
    if (cmd == CMD8) i = 0x87;      // Valid CRC for CMD8(0x1AA)
    spiSend(i);

    for( i = 0; i < 128; i++) if((res = spiRead()) != 0xFF) break;  // wait for response and get it
    for( i = 0; i < cnt; i++) resp_buf[i] = spiRead();    // read more response if not R1 command

    return res;             // Return with the response value
}


/// CARD Initialization
CSTATUS card_initialize (void)
{
  uint8_t cmd, n;
  uint16_t tmr;

  CardType = 0;

  DESELECT();
  for (n = 20; n; n--) spiRead(); // wait some time

  if (send_cmd(CMD0, 0, 0) == 1)
  { // Entered Idle state
    if (send_cmd(CMD8, 0x1AA, 4) == 1)
    { // SDv2
      if (resp_buf[2] == 0x01 && resp_buf[3] == 0xAA)
      { // Supports voltage 2.7-3.6V        
        // Wait for leaving Idle state (ACMD41 with HCS bit)
        for (tmr = 25000; tmr && send_cmd(ACMD41, 1UL << 30, 0); tmr--) ;  /* Wait for leaving idle state (ACMD41 with HCS bit) */
        if (tmr && send_cmd(CMD58, 0, 4) == 0)    /* Check CCS bit in the OCR */        
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

      for (tmr = 25000; tmr && send_cmd(cmd, 0, 0); tmr--) ;  /* Wait for leaving idle state */

      if (!tmr || send_cmd(CMD16, 512, 0) != 0) CardType = 0; // Set R/W block length to 512        
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
  uint16_t bc;

  // commented as it is too much useless code
  //if (!cnt || ofs + cnt > 512) return RES_PARERR; // wrong parameters

  if (!(CardType & CT_SDHC)) lba *= 512;    // SDHC - LBA = block number (512 bytes), other - LBA = byte offset

  res = RES_ERROR;

  if (send_cmd(CMD17, lba, 0) == 0)
  { // READ_SINGLE_BLOCK

      bc = 40000;
      do { rc = spiRead(); } while (rc == 0xFF && --bc); // wait for CARD is ready to transmit block of data


      if (rc == 0xFE)
      { // receive data block 512 bytes + CRC
          p = (uint8_t*)dest;
          for(uint16_t i = 0; i < 514; i++)
          {
              SPDR = 0xFF;
              loop_until_bit_is_set(SPSR, SPIF);
              if( i >= ofs && cnt > 0)
              {
                  *p++ = SPDR;
                  cnt--;
              }
          }
          DESELECT();
          SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF);
          return cnt ? RES_ERROR : RES_OK;
      }
  }
  return RES_ERROR;
}
  

/// Read full sector
CRESULT card_read_sector (
  void *dest,     // Pointer to the destination object to put data
  uint32_t lba     // Start sector number (LBA)
)
{
  uint8_t *p , rc;

  if (!(CardType & CT_SDHC)) lba *= 512;    // SDHC - LBA = block number (512 bytes), other - LBA = byte offset
  
  if (send_cmd(CMD17, lba, 0) == 0)
  { // READ_SINGLE_BLOCK
    while( (rc = spiRead() ) == 0xFF ); // wait for CARD is ready to transmit block of data
    
    if (rc == 0xFE)
    { // receive data block + CRC
      p = (uint8_t*)dest;
      for(uint8_t i=0 ;; i++) { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); *p++=SPDR; if(i==0xFF) break;}
      for(uint8_t i=0 ;; i++) { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); *p++=SPDR; if(i==0xFF) break;}
      SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF);
      SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF);
    }
    DESELECT();
    SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF);
    return RES_OK;
  }
  return RES_ERROR;
}

/// Write sector
CRESULT card_writep (
  const uint8_t *buff, // Pointer to the bytes to be written (NULL:Initiate/Finalize sector write)
  uint32_t sa      // Number of bytes to send, Sector number (LBA) or zero
)
{
  CRESULT res;
  uint16_t bc;
  static uint16_t wc;

  res = RES_ERROR;

  if (buff) {   // Send data bytes
    bc = (uint16_t)sa;
    while (bc && wc) {    // Send data bytes to the card
      spiSend(*buff++);
      wc--; bc--;
    }
    res = RES_OK;
  } else {
    if (sa) { // Initiate sector write process 
      if (!(CardType & CT_SDHC)) sa *= 512;  // Convert to byte address if needed
      if (send_cmd(CMD24, sa,0) == 0) {     // WRITE_SINGLE_BLOCK
        spiSend(0xFF); spiSend(0xFE);   // Data block header
        wc = 512;             // Set byte counter
        res = RES_OK;
      }
    } else {  // Finalize sector write process
      bc = wc + 2;
      while (bc--) spiSend(0); // Fill left bytes and CRC with zeros
      if ((spiRead() & 0x1F) == 0x05) { // Receive data resp and wait for end of write process in timeout of 500ms
        for (bc = 5000; spiRead() != 0xFF && bc; bc--) ;//dly_100us(); // Wait ready
        if (bc) res = RES_OK;
      }
      DESELECT();
      spiRead();
    }
  }

  return res;
}

/// Read CID
uint32_t card_read_serial()
{
  uint8_t* ptr = (uint8_t*)&CID;
  uint8_t res = 0;
  if (send_cmd(CMD10, 0, 0) == 0)
  { // READ_CID
      for(uint8_t i = 0; i < 16; i++) { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); *ptr++=SPDR; }
      DESELECT();
      res = CID.psn;
  }
  return res;
}

/// ----------------------------------------------------------------------------------------------------------------
