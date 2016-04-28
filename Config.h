#ifndef FDDCONFIG_H
#define FDDCONFIG_H

#define MAX_CYL 82          /// maximal cylinder supported by FDD
#define MAX_DIR_LEVEL 10     /// maximal subfolders support, the higher value the more memory used.

/// Floppy pinout configuration -----------------------------------------------------------------------------

#define SIDE_PIN  PIND  // PORT pin at which side pin is located
#define SIDE_SEL  PD0   // pin 0,  SIDE SELECT                                  (INPUT)
#define READ_DATA PD1   // pin 1,  READ_DATA                                    (OUTPUT) /// defined in USART
#define WP        PD2   // pin 2,  WRITE PROTECT                                (OUTPUT)
#define TRK00     PD3   // pin 3,  TRACK 00                                     (OUTPUT)
#define STEP      PD4   // pin 4,  STEP                                         (INPUT)
#define DIR_SEL   PD5   // pin 5,  DIRECTION SELECT                             (INPUT)
#define MOTOR_ON  PD6   // pin 6,  MOTOR ON                                     (INPUT)
#define DRIVE_SEL PD7   // pin 7,  DRIVE SELECT CONNECT DS0-DS3 using jumper    (INPUT)

#define INDEX     PB0   // pin 8,  INDEX                                        (OUTPUT)

#define ENC_A     PC2
#define ENC_B     PC3
#define BTN       PC1

/// SD Card pinout configuration -----------------------------------------------------------------------------

/* SD card attached to SPI bus as follows: MOSI - pin 11, MISO - pin 12, CLK(SCK) - pin 13, CS - pin 10 */
#define SPI_DDR   DDRB
#define SPI_PORT  PORTB
#define SPI_CS    PB2   // pin 10
#define SPI_MOSI  PB3   // pin 11
#define SPI_MISO  PB4   // pin 12
#define SPI_SCK   PB5   // pin 13

/// PIN Definitions for Arduion IDE 1.0 support --------------------------------------------------------------

#if !defined(PB0)
#define  PB0 0
#endif

#if !defined(PB1)
#define PB1 1
#endif

#if !defined(PB2)
#define PB2 2
#endif

#if !defined(PB3)
#define PB3 3
#endif

#if !defined(PB4)
#define PB4 4
#endif

#if !defined(PB5)
#define PB5 5
#endif

#if !defined(PB6)
#define PB6 6
#endif

#if !defined(PB7)
#define PB7 7
#endif


#if !defined(PC0)
#define PC0 0
#endif

#if !defined(PC1)
#define PC1 1
#endif

#if !defined(PC2)
#define PC2 2
#endif

#if !defined(PC3)
#define PC3 3
#endif

#if !defined(PC4)
#define PC4 4
#endif

#if !defined(PC5)
#define PC5 5
#endif

#if !defined(PC6)
#define PC6 6
#endif

#if !defined(PC7)
#define PC7 7
#endif


#if !defined(PD0)
#define PD0 0
#endif

#if !defined(PD1)
#define PD1 1
#endif

#if !defined(PD2)
#define PD2 2
#endif

#if !defined(PD3)
#define PD3 3
#endif

#if !defined(PD4)
#define PD4 4
#endif

#if !defined(PD5)
#define PD5 5
#endif

#if !defined(PD6)
#define PD6 6
#endif

#if !defined(PD7)
#define PD7 7
#endif


#endif /* FDDCONFIG_H */
