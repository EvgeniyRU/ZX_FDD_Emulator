// ZX-Spectrum FDD Emulator
//
// Current version opens file "default.trd" from SD Card root folder and use it to emulate floppy disk
// SD CARD: Supported only FAT32 file system, cards SDC/SDHC/MMC, all cluster sizes from 512 bytes to 64k
//
#include <avr/io.h>
#include <util/atomic.h>
#include "PinDefs.h"


#define SIDE_SEL  PD0   // pin 0,  SIDE SELECT                                  (INPUT)
#define READ_DATA PD1   // pin 1,  READ_DATA                                    (OUTPUT) /// defined in USART
#define WP        PD2   // pin 2,  WRITE PROTECT                                (OUTPUT)
#define TRK00     PD3   // pin 3,  TRACK 00                                     (OUTPUT)
#define STEP      PD4   // pin 4,  STEP                                         (INPUT)
#define DIR_SEL   PD5   // pin 5,  DIRECTION SELECT                             (INPUT)
#define MOTOR_ON  PD6   // pin 6,  MOTOR ON                                     (INPUT)
#define DRIVE_SEL PD7   // pin 7,  DRIVE SELECT CONNECT DS0-DS3 using jumper    (INPUT)

#define INDEX     PB0   // pin 8,  INDEX                                        (OUTPUT)


#include "SDCardModule.h"
#include "Fat32Module.h"
#include "LCDModule.h"
FATFS fat;
DIR dir;
FILINFO fnfo;

/// EMULATOR START -------------------------------------------------------------------------------------------------

#define MAX_CYL 82          /// maximal cylinder supported by FDD

uint8_t prev_byte, cylinder_changed, max_cylinder, cylinder;
uint8_t sector_data[256]; // sector data
uint32_t clust_table[MAX_CYL]; // Cluster table
uint32_t sector_table[32]; // Cluster table for sectors in cylinder

// inverted MFM table for fast converting
uint8_t MFM_tab_inv_true_0[32] = {0x77,0x77,0x77,0x77,0x7D,0x7D,0x7D,0x7D,0xDF,0xDF,0xDF,0xDF,0xDD,0xDD,0xDD,0xDD,0xF7,0xF7,0xF7,0xF7,0xFD,0xFD,0xFD,0xFD,0xDF,0xDF,0xDF,0xDF,0xDD,0xDD,0xDD,0xDD};
uint8_t MFM_tab_inv_true_1[32] = {0x77,0x7D,0xDF,0xDD,0xF7,0xFD,0xDF,0xDD,0x77,0x7D,0xDF,0xDD,0xF7,0xFD,0xDF,0xDD,0x77,0x7D,0xDF,0xDD,0xF7,0xFD,0xDF,0xDD,0x77,0x7D,0xDF,0xDD,0xF7,0xFD,0xDF,0xDD};

const uint16_t Crc16Table[256] PROGMEM = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};


///
/// Interrupts enable/disable functions
///////////////////////////////////////////
void inline USART_enable() { UCSR0B = 0x08; }
void inline USART_disable() { UCSR0B = 0x00; }
void inline PCINT2_enable()
{
    PCIFR  |= _BV(PCIE2); // reset interrupt flag
    PCICR  |= _BV(PCIE2);
}
void inline PCINT2_disable() { PCICR &= ~_BV(PCIE2); }

///
/// STEP pin interrupt
///////////////////////////////////////////
ISR(PCINT2_vect)
{
    asm volatile(
      "sbic %0,%1"      "\n\t" // check for falling edge
      "rjmp PCINT_END"  "\n\t"
      "sts 0xC1,r1"     "\n\t" // disable USART
      :: "I" _SFR_IO_ADDR(PIND), "I" (STEP)
    );

    if(PIND & _BV(DIR_SEL))
    {
        if(cylinder != 0) cylinder--;
        if(cylinder == 0) DDRD |= _BV(TRK00);
    }
    else
    {
        if(cylinder + 1 < max_cylinder) cylinder++;
        DDRD &= ~_BV(TRK00); // Set TRK00 - LOW or HIGH
    }
    cylinder_changed = 1;

    asm("PCINT_END:");
}


/// FDD Emulator initialization
///////////////////////////////////////////
void emu_init()
{
    cli(); // DISABLE GLOBAL INTERRUPTS

    // Setup USART in MasterSPI mode 500000bps
    UBRR0H = 0x00;
    UBRR0L = 0x07; // 1000 kbps
    UCSR0C = 0xC0;
    UCSR0A = 0x00;
    UCSR0B = 0x00; // disabled

    PCMSK2 |= _BV(PCINT20); // SET PCINT2 interrupt on PD4 (STEP pin)

    // INIT pins and ports
    PORTD |= _BV(STEP) | _BV(MOTOR_ON) | _BV(DRIVE_SEL) | _BV(DIR_SEL) | _BV(SIDE_SEL); // set pull-up
    
    DDRB &= ~_BV(INDEX); // SET INDEX HIGH
    DDRD &= ~(_BV(WP) | _BV(TRK00)); // Set RD, WP,TRK00 as input // | _BV(READ_DATA)
 
    // Init SPI for SD Card
    SPI_DDR = _BV(SPI_MOSI) | _BV(SPI_SCK) | _BV(SPI_CS); //set output mode for MOSI, SCK ! move SS to GND
    SPCR = _BV(MSTR) | _BV(SPE);   // Master mode, SPI enable, clock speed MCU_XTAL/4, LSB first
    SPSR = _BV(SPI2X);             // double speed

    sei();   // ENABLE GLOBAL INTERRUPTS
}

/// Send byte in MFM encoding as 4 bytes at speed 1000000bps
////////////////////////////////////////////////////////////////
void send_byte(uint8_t sector_byte)
{
    uint8_t tmp = MFM_tab_inv_true_0[sector_byte >> 4]; // get first MFM byte from table (first 4 bits)
    if((prev_byte & 1) && !(sector_byte & 0x80)) tmp |= 0x80; // check previous last bit and correct first clock bit of a new byte
    loop_until_bit_is_set(UCSR0A,UDRE0);
    UDR0 = tmp;

    loop_until_bit_is_set(UCSR0A,UDRE0);
    UDR0 = MFM_tab_inv_true_1[sector_byte >> 4]; // get first MFM byte from table (second 4 bits)

    loop_until_bit_is_set(UCSR0A,UDRE0);
    UDR0 = MFM_tab_inv_true_0[sector_byte & 0x1f]; // get second MFM byte from table (first 4 bits)

    loop_until_bit_is_set(UCSR0A,UDRE0);
    UDR0 = MFM_tab_inv_true_1[sector_byte & 0x1f]; // get second MFM byte from table (second 4 bits)

    prev_byte = sector_byte;
}

///
/// MAIN Routine
///////////////////////////////////////////
uint8_t s_cylinder, side, sector_byte, sector, cnt, tmpc;
union { uint16_t val; struct { byte low; byte high; } bytes; } CRC_H, CRC_D;

int main()
{
    init(); // init arduino libraries

    LCD_init();

    emu_init(); // initialize FDD emulator


    while(1)
    { // MAIN LOOP START
        /// MAIN LOOP USED FOR SELECT and INIT SD CARD and other

     MOUNT:        
        //>>>>>> print "NO CARD PRESENT" on LCD
        pf_mount(0);
        LCD_clear();
     NO_FILES:
        LCD_print("NO CARD INSERTED");
        while(pf_mount(&fat) != FR_OK);
        LCD_clear();
        LCD_print(" CARD MOUNT OK.");
        //>>>>>> print "CARD INFO etc..."

        //uint32_t serial = card_read_serial();
        

        //>>>>>> SELECT TRD IMAGE HERE

        pf_opendir(&dir,"/");

        uint16_t index = 0;
        
        for(;;)
        {
            if(pf_readdir(&dir,&fnfo) != FR_OK) goto MOUNT;
            if(strcasestr(fnfo.fname,".trd"))
                if ((fnfo.fattrib & AM_DIR) == 0 && fnfo.fname[0] != 0) { index++; break; }
            if(pf_dirnext(&dir) != FR_OK) break;
        }

        if(index == 0)
        {
            LCD_clear();
            LCD_print("- NO TRD FILES -");
            delay(1000);
            goto NO_FILES;
        }


        /////////////////////////////////////////////////////////////////
        // MOUNT TRD IMAGE and init Track Cluster table
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // --------------------------------------------------------------------------------------------------------------------------------
        //if(pf_open("default.trd") != FR_OK) goto MOUNT; // if unable to open file, usually if SD card is removed
        if(pf_open(fnfo.fname) != FR_OK) goto MOUNT; // if unable to open file, usually if SD card is removed
        
        LCD_clear();
        LCD_print("default.trd     ");
        LCD_print(0,1,"CYL: 00 HEAD: 00" );

        max_cylinder = fat.fsize / 8192 + ((fat.fsize % 8192) ? 1 : 0); // calculate maximal cylinder
        if( max_cylinder > MAX_CYL ) max_cylinder = MAX_CYL;

        /// FAST create cluster table for cylinders ---------------------------------------------------------------------------------------
        uint32_t cur_fat = fat.org_clust, cur_fat_sector = cur_fat / 64;
        if(card_readp(sector_data, fat.fatbase + cur_fat_sector/2, (cur_fat_sector%2)*256, 256) != RES_OK) goto MOUNT;
        clust_table[0] = cur_fat;
        for(uint16_t i = 1; i < max_cylinder*16; i++)
        { /// 16 SD sectors per cylinder
            if( i % fat.csize == 0) // cluster boundary
            {
                if( (cur_fat / 64) != cur_fat_sector )
                {
                     cur_fat_sector = cur_fat / 64;
                     card_readp(sector_data, fat.fatbase + cur_fat_sector/2, (cur_fat_sector%2)*256, 256); // read data_block with current cluster number
                }
                cur_fat = (uint32_t)(*(uint32_t*)(sector_data + (uint8_t)((uint8_t)cur_fat << 2)));
            }
            if(i % 16 == 0) clust_table[i/16] = cur_fat;
        } // --------------------------------------------------------------------------------------------------------------------------------
        ///////////////////////////////////////////////////////////////////////////////////////////////////

        cylinder = 0;
        s_cylinder = 255;
        cylinder_changed = 0;

        while (1)
        { /// DRIVE SELECT LOOP

            while ( PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)) ); // wait drive select && motor_on

            /// DEVICE ENABLED ==========================================================================================================================

            if(cylinder == 0) DDRD |= _BV(TRK00);
            PCINT2_enable(); // ENABLE INDERRUPT (STEP pin)

            DDRD |= _BV(WP); // set WRITE PROTECT

            //check SD Card is present and same card as was mounted
            //if(serial != card_read_serial()) break; // exit from loop if card is not present or another card.

            uint8_t read_error = 0;

            do { // READ DATA LOOP (send data from FDD to FDD controller)  
            //-------------------------------------------------------------------------------------------                

                for(volatile uint8_t sector = 0; sector < 16; sector++)
                { // transmit each sector of the track

                    if( sector == 0 ) // initialize track data for next round
                    {
                        prev_byte = 0;
                        for(volatile uint16_t tmpcn = 0; tmpcn < 1000; tmpcn++) tmpc++; // wait for cylinder change detect

                        if(s_cylinder != cylinder)
                        {
                            while( cylinder_changed )
                            { // wait while cylinder changing
                                ATOMIC_BLOCK(ATOMIC_FORCEON)cylinder_changed = 0;
                                for(volatile uint16_t tmpcn = 0; tmpcn < 10000; tmpcn++) tmpc++;
                            }
                          
                            s_cylinder = cylinder;

                            // FAST create cluster table for all 32 cylinder sectors
                            cur_fat = clust_table[s_cylinder];
                            cur_fat_sector = cur_fat / 64;
                            sector_table[0] = sector_table[1] = cur_fat; // sector 1 offset on SD card
                            if(card_readp(sector_data, fat.fatbase + cur_fat_sector/2, (cur_fat_sector%2)*256, 256) != RES_OK) { read_error = 1; break; }
                            for(uint8_t i = 1; i < 16; i++) // 2 - 32 sectors
                            {
                                if((i % fat.csize == 0))
                                {
                                    if( (cur_fat / 64) != cur_fat_sector )
                                    {
                                        cur_fat_sector = cur_fat / 64;
                                        card_readp(sector_data, fat.fatbase + cur_fat_sector/2, (cur_fat_sector%2)*256, 256); // read data_block with current cluster number
                                    }
                                    cur_fat = (uint32_t)(*(uint32_t*)(sector_data + (uint8_t)((uint8_t)cur_fat << 2)));
                                }
                                sector_table[i*2] = sector_table[i*2+1] = cur_fat;
                            }
                        }
                        else
                            cylinder_changed = 0;

                    }

                    side = (~PIND) & 1;
                    // read sector data from SD card
                    if(card_readp(sector_data,fat.database + (sector_table[side*16 + sector] - 2) * fat.csize + ((s_cylinder*2 + side)*8 + sector/2) % fat.csize,(sector%2)*256,256) != RES_OK) { read_error = 1; break; }

                    CRC_H.val = 0xB230;
                    CRC_H.val = (CRC_H.bytes.low * 256) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ s_cylinder));
                    CRC_H.val = (CRC_H.bytes.low * 256) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ side));
                    CRC_H.val = (CRC_H.bytes.low * 256) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ sector+1));
                    CRC_H.val = (CRC_H.bytes.low * 256) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ 1));

                    CRC_D.val = 0xE295;
                    for(uint8_t i = 0; ; i++)
                    {
                        CRC_D.val = (CRC_D.bytes.low << 8) ^ pgm_read_word_near(Crc16Table + (CRC_D.bytes.high ^ sector_data[i]));
                        if(i==255) break;
                    }

                    if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // if cylinder is changed or FDD is disabled

                    // NOW WE'RE READY TO SEND SECTOR

                    if(sector == 0)
                    { // Send TRACK GAP4A ----------------------------------------------------
                        USART_enable();
                        for(cnt = 0; cnt < 10; cnt++)
                          send_byte(0x4E);
                        DDRB |= _BV(INDEX); // SET INDEX LOW
                    }

                    // Send sector Address Field + start data field --------------------------
                    for(cnt = 0; cnt < 60; cnt++)
                    {
                        switch(cnt)
                        {
                            case 0: sector_byte = 0; break;
                            case 12: sector_byte = 0xA1; MFM_tab_inv_true_0[1] = 0x7F; break;
                            case 15: sector_byte = 0xFE; MFM_tab_inv_true_0[1] = 0x77; break;
                            case 16: sector_byte = s_cylinder; break;
                            case 17: sector_byte = side; break;
                            case 18: sector_byte = sector + 1; break;
                            case 19: sector_byte = 1; break;
                            case 20: sector_byte = CRC_H.bytes.high; break;
                            case 21: sector_byte = CRC_H.bytes.low; break;
                            case 22: sector_byte = 0x4E; break; // 22 in TR-DOS
                            case 44: sector_byte = 0x00; break;
                            // data field header
                            case 56: sector_byte = 0xA1;
                                     MFM_tab_inv_true_0[1] = 0x7F;
                                     break;
                            case 59: sector_byte = 0xFB;  MFM_tab_inv_true_0[1] = 0x77; break;
                        }
                        send_byte(sector_byte);
                    }

                    if(!sector) DDRB &= ~_BV(INDEX); // SET INDEX HIGH if sector = 0

                    if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // if cylinder is changed or FDD is disabled

                    // Send sector data -----------------------------------------------------
                    for(cnt = 0; ; cnt++)
                    {
                        send_byte(sector_data[cnt]);
                        if(cnt == 255) break;
                    }

                    // Send CRC
                    send_byte(CRC_D.bytes.high);
                    send_byte(CRC_D.bytes.low);
                    // Send sector GAP ------------------------------------------------------
                    for(cnt = 0; cnt < 54; cnt++)
                        send_byte(0x4E);

                    if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // if cylinder is changed or FDD is disabled

                    if(sector == 0)
                    {
                         // print "CYLINDER, HEAD INFO" or track number on LCD                         
                         LCD_print(5,1,cylinder / 10);
                         LCD_print(cylinder % 10);
                         LCD_print(15,1,side);
                    }

                }
                if(read_error) break;

            } while( !(PIND & ( _BV(MOTOR_ON) | _BV(DRIVE_SEL) )) ); // READ DATA SEND LOOP END
            //-------------------------------------------------------------------------------------------
            PCINT2_disable(); // DISABLE INDERRUPT (STEP pin)
            USART_disable(); // disable interrupt after sending track
            DDRD &= ~(_BV(WP) | _BV(TRK00)); // Set WP,TRK00 as input
            if(read_error) break;

            /// DEVICE DISABLED =========================================================================================================================
            
        } /// DRIVE SELECT LOOP END

    } // MAIN LOOP END

} // END MAIN



