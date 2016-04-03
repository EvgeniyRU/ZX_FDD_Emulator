#ifndef SDCARD_MODULE_H
#define SDCARD_MODULE_H



/* SD card attached to SPI bus as follows: MOSI - pin 11, MISO - pin 12, CLK(SCK) - pin 13, CS - pin 10 */
#define SPI_DDR   DDRB
#define SPI_PORT  PORTB
#define SPI_CS    PB2   // pin 10
#define SPI_MOSI  PB3   // pin 11
#define SPI_MISO  PB4   // pin 12
#define SPI_SCK   PB5   // pin 13

// CS operation
#define SELECT()  SPI_PORT &= ~_BV(SPI_CS) /* CS = L */
#define DESELECT() SPI_PORT |=  _BV(SPI_CS) /* CS = H */


typedef uint8_t  CSTATUS;

/// Results of CARD Functions
typedef enum {
  RES_OK = 0,   /* 0: Function succeeded */
  RES_ERROR,    /* 1: Disk error */
  RES_STRERR,   /* 2: Seream error */
  RES_NOTRDY,   /* 3: Not ready */
  RES_PARERR    /* 4: Invalid parameter */
} CRESULT;


#define STA_NOINIT    0x01  // Drive not initialized
#define STA_NODISK    0x02  // No medium in the drive

/// Card type flags (CardType)
#define CT_MMC        0x01  // MMC ver 3
#define CT_SD1        0x02  // SD ver 1
#define CT_SD2        0x04  // SD ver 2
#define CT_SDHC       0x08  // SDHC

/// MMC/SD cards commands
#define CMD0  (0x40+0)  // GO_IDLE_STATE
#define CMD1  (0x40+1)  // SEND_OP_COND (MMC)
#define ACMD41  (0xC0+41) // SEND_OP_COND (SD)
#define CMD8  (0x40+8)  // SEND_IF_COND
#define CMD9  (0x40+9)  // SEND_CSD
#define CMD10 (0x40+10) // SEND_CID
#define CMD12 (0x40+12) // STOP_TRANSMISSION
#define ACMD13  (0xC0+13) // SD_STATUS (SD)
#define CMD16 (0x40+16) // SET_BLOCKLEN
#define CMD17 (0x40+17) // READ_SINGLE_BLOCK
#define CMD18 (0x40+18) // READ_MULTIPLE_BLOCK
#define CMD23 (0x40+23) // SET_BLOCK_COUNT (MMC)
#define ACMD23  (0xC0+23) // SET_WR_BLK_ERASE_COUNT (SD)
#define CMD24 (0x40+24) // WRITE_BLOCK
#define CMD25 (0x40+25) // WRITE_MULTIPLE_BLOCK
#define CMD55 (0x40+55) // APP_CMD
#define CMD58 (0x40+58) // READ_OCR

/// receive SPI byte
static uint8_t spiRead();

/// send SPI byte
static void spiSend(uint8_t d);

// Send a command to CARD
static uint8_t send_cmd(uint8_t, uint32_t, uint8_t);


/// CARD Initialization
CSTATUS card_initialize (void);

/// Read partial sector
CRESULT card_readp (void *, uint32_t, uint16_t, uint16_t);

/// Read full sector
void card_read_sector (void *, uint32_t);
/// ----------------------------------------------------------------------------------------------------------------



#endif /* SDCARD_MODULE_H */
