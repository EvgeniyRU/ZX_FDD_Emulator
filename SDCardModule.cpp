/// ----------------------------------------------------------------------------------------------------------------
/// SD CARD MODULE Based on Petit FatFs
/// ----------------------------------------------------------------------------------------------------------------
#include <stdint.h>
#include <avr/io.h>
#include <Arduino.h>
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

const uint8_t crc7_table[256] PROGMEM = {
  0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f, 0x48, 0x41, 0x5a, 0x53, 0x6c, 0x65, 0x7e, 0x77, 0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
  0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e, 0x32, 0x3b, 0x20, 0x29, 0x16, 0x1f, 0x04, 0x0d, 0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
  0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14, 0x63, 0x6a, 0x71, 0x78, 0x47, 0x4e, 0x55, 0x5c, 0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
  0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13, 0x7d, 0x74, 0x6f, 0x66, 0x59, 0x50, 0x4b, 0x42, 0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
  0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69, 0x1e, 0x17, 0x0c, 0x05, 0x3a, 0x33, 0x28, 0x21, 0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
  0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38, 0x41, 0x48, 0x53, 0x5a, 0x65, 0x6c, 0x77, 0x7e, 0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
  0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67, 0x10, 0x19, 0x02, 0x0b, 0x34, 0x3d, 0x26, 0x2f, 0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
  0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04, 0x6a, 0x63, 0x78, 0x71, 0x4e, 0x47, 0x5c, 0x55, 0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
  0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a, 0x6d, 0x64, 0x7f, 0x76, 0x49, 0x40, 0x5b, 0x52, 0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
  0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b, 0x17, 0x1e, 0x05, 0x0c, 0x33, 0x3a, 0x21, 0x28, 0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
  0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31,
  0x46, 0x4f, 0x54, 0x5d, 0x62, 0x6b, 0x70, 0x79
};

// Send a command to CARD
uint8_t send_cmd(uint8_t cmd, uint32_t param, uint8_t cnt)
{
    uint8_t send_buf[6];
    uint8_t i, res;

    DESELECT();
    if (cmd & 0x80)
    { // ACMD{X} is the command sequense of CMD55 + CMD{X}
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0, 0);
        if (res > 1) return res;
    }

    send_buf[0] = cmd;    // Start + Command index
    send_buf[1] = param >> 24;  // bits [31..24]
    send_buf[2] = param >> 16;  // bits [23..16]
    send_buf[3] = param >> 8; // bits [15..08]
    send_buf[4] = param;  // bits [07..00]

    // CRC7 calculatuin
    uint8_t crc = 0;
    for(i = 0; i < 5; i++ )
        crc = pgm_read_byte_near(crc7_table + ((crc << 1) ^ send_buf[i]));

    send_buf[5] = (crc << 1) | 1; // CRC + Parity

    /* Select the card */
    DESELECT();
    spiRead();
    SELECT();
    spiRead();

    for( i = 0; i < 6 ; i++) spiSend(send_buf[i]);      // send command
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
  for (n = 10; n; n--) spiRead(); // wait some time

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

  // commented as it is too much useless code
  //if (!cnt || ofs + cnt > 512) return RES_PARERR; // wrong parameters

  if (!(CardType & CT_SDHC)) lba *= 512;    // SDHC - LBA = block number (512 bytes), other - LBA = byte offset

  res = RES_ERROR;

  if (send_cmd(CMD17, lba, 0) == 0)
  { // READ_SINGLE_BLOCK

      while( (rc = spiRead() ) == 0xFF ); // wait for CARD is ready to transmit block of data

      if (rc == 0xFE)
      { // receive data block 512 bytes + CRC
          p = (uint8_t*)dest;
          for(uint16_t i = 0; i < 514; i++)
          {
              SPDR = 0xFF;
              loop_until_bit_is_set(SPSR, SPIF);
              if( i >= ofs )
              {
                  *p++ = SPDR;
                  if(!--cnt)
                  {
                      send_cmd(CMD12,0,0); // stop transmission from SD card if partial sector read;
                      DESELECT();
                      SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF);
                      return cnt ? RES_ERROR : RES_OK;
                  }
              }
          }
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
