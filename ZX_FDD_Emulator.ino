// ZX-Spectrum FDD Emulator
//
// Current version opens file "default.trd" from SD Card root folder and use it to emulate floppy disk
// SD CARD: Supported only FAT32 file system, cards SDC/SDHC/MMC, all cluster sizes from 512 bytes to 64k
//
#include <avr/io.h>
#include <util/atomic.h>
#include "PinDefs.h"

#define SIDE_SEL  PC0   // pin A0, SIDE SELECT                                  (INPUT)
#define DRIVE_SEL PC1   // pin A1, DRIVE SELECT CONNECT DS0-DS3 using jumper    (INPUT)

#define DIR_SEL   PB0   // pin 8,  DIRECTION SELECT                             (INPUT)
  
//#define WRIT_DATA PD0   // pin 0,  WRITE DATA                                   (INPUT) /// not used yet
#define READ_DATA PD1   // pin 1,  READ_DATA                                    (OUTPUT) /// defined in USART
#define STEP      PD2   // pin 2,  STEP                                         (INPUT)
//#define WRT_GATE  PD3   // pin 3,  WRITE GATE                                   (INPUT) /// not used yet
#define MOTOR_ON  PD4   // pin 4,  MOTOR ON                                     (INPUT) 
#define INDEX     PD5   // pin 5,  INDEX                                        (OUTPUT)
#define TRK00     PD6   // pin 6,  TRACK 00                                     (OUTPUT)
#define WP        PD7   // pin 7,  WRITE PROTECT                                (OUTPUT)

#include "SDCardModule.h"
#include "Fat32Module.h"
FATFS fat;

/// EMULATOR START -------------------------------------------------------------------------------------------------

#define MAX_CYL 84          /// maximal cylinder supported by FDD
#define MAX_TRACK MAX_CYL*2 /// maximal track

uint8_t sector_header[64]; // sector header
uint8_t sector_data[2][256]; // sector data
uint32_t clust_table[MAX_TRACK]; // Cluster table
uint32_t cluster_chain[8]; // max cluster chain 8 is for 512 bytes cluster or less if higher

// STATE variable values
// 0-2    - TRACK HEADER
// 4-9    - SECTOR
// 10-11  - TRACK FOOTER
uint8_t state, max_track, sector, tmp, max_cylinder, cylinder, cylinder_changed;
volatile uint8_t data_sent; // this is important!!!
uint16_t CRC_tmp;
union { uint16_t val; struct { byte low; byte high; } bytes; } CRC;
register volatile uint8_t sector_byte asm("r2");
register volatile uint8_t b_index asm("r3");

// inverted table for fast converting
uint8_t MFM_tab_inv[16] = {0x55,0x76,0x59,0x5A,0x65,0x66,0x69,0x6A,0x95,0x96,0x99,0x9A,0xA5,0xA6,0xA9,0xAA};

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
void inline USART_enable() { UCSR0B = 0x28; }
void inline USART_disable() { UCSR0B = 0x00; }
void inline INT0_enable() { EIMSK |= 0x01; }
void inline INT0_disable() { EIMSK &= ~0x01; }

uint8_t t_millis = 0;
uint8_t easy_millis()
{
    uint8_t millis_ret;
    ATOMIC_BLOCK(ATOMIC_FORCEON)millis_ret = t_millis;
    return millis_ret;
}

ISR (TIMER0_OVF_vect)    // timer0 interrupt service routine
{
    t_millis++;
}


///
/// STEP pin interrupt
///////////////////////////////////////////
ISR(INT0_vect)
{
  // don't do step if write_gate active (LOW)
    if(PINB & _BV(DIR_SEL))
    {
        if(cylinder > 0) cylinder--;
    }
    else
        if(cylinder < max_cylinder) cylinder++;
    USART_disable();
    PORTD |= _BV(INDEX); // Set INDEX - HIGH
    cylinder_changed = 1;
    data_sent = 2; // set flag indicates end of track, for reinitialize track data and read new sectors
}


///
/// READ DATA interrupt
///////////////////////////////////////////
ISR(USART_UDRE_vect)
{ 
    if (!tmp)
    { // Send first MFM byte
        UDR0 = MFM_tab_inv[sector_byte >> 4];  // put byte to send buffer
        tmp = 1;
    }
    else
    { // Send second MFM byte
        UDR0 = MFM_tab_inv[sector_byte & 0x0f];
        tmp = 0; // this is important!

        // GET NEXT DATA BYTE (REAL DATA NOT MFM)
        switch (state)
        {
          case 0: // start track header -----------------------------------------
            PORTD &= ~_BV(INDEX); // Set INDEX - LOW
            state = 1;
            b_index = 0;
            break;
      
          case 1: // send track header ------------------------------------------
            if (++b_index != 5) break; // 80 in FDD
            state = 2;
            b_index = 0;            
            PORTD |= _BV(INDEX); // Set INDEX - HIGH
            break;
            
          case 2: // SECTOR START (ADDERSS FIELD) ------------------------------
          {
            switch(b_index)
            {
              // Address field CRC Calculation
              case 0: CRC.val = 0xB230; break;              
              // WARNING!!! SIDE_SEL pin used!!! THE BEST SOLUTION!!! If side is changed during track tranmition we set wrong CRC for that sector.
              case 1: if ((PINC & 1) == sector_header[17]) CRC.bytes.low = 0; break;
              case 2: CRC_tmp = pgm_read_word_near(Crc16Table + (CRC.bytes.high ^ sector_header[16])); break;             
              case 3: CRC.val = (CRC.bytes.low * 256) ^ CRC_tmp; break;
              case 4: CRC_tmp = pgm_read_word_near(Crc16Table + (CRC.bytes.high ^ sector_header[17])); break;
              case 5: CRC.val = (CRC.bytes.low * 256) ^ CRC_tmp; break;
              case 6: sector_header[18] = sector + 1; break;
              case 7: CRC_tmp = pgm_read_word_near(Crc16Table + (CRC.bytes.high ^ sector_header[18])); break;
              case 8: CRC.val = (CRC.bytes.low * 256) ^ CRC_tmp; break;
              case 9: CRC_tmp = pgm_read_word_near(Crc16Table + (CRC.bytes.high ^ 1)); break;
              case 10: CRC.val = (CRC.bytes.low * 256) ^ CRC_tmp; break;
              case 11: sector_header[20] = CRC.bytes.high; break;
              case 12: sector_header[21] = CRC.bytes.low; break;
            }
            // send sector bytes before data
            sector_byte = sector_header[b_index];   // pre-get new byte from buffer
            
            if (++b_index != 60) break;
            CRC.val = 0xE295; // START GENERATING CRC HERE, PRE-CALC value for A1,A1,A1,FB = 0xE295 next CRC value
            b_index = 0;
            state = 3;
            data_sent = 0;
            break;
          }
          case 3: // DATA FIELD ----------------------------------------------------
            // get sector data values
            sector_byte = sector_data[sector % 2][b_index++];   // pre-get new byte from buffer
            CRC.val = (CRC.bytes.low * 256) ^ pgm_read_word_near(Crc16Table + (CRC.bytes.high ^ sector_byte));
            if (b_index != 0) break;
            if(++sector < 15) data_sent = 1;
            state = 4;
            break;
        
          case 4: // SEND SECTOR CRC -----------------------------------------------
            // increase sector, set flag indicates end of 2 sectors, for reading new sectors
            sector_byte = CRC.bytes.high;
            state = 5;
            break;

          case 5:
            sector_byte = CRC.bytes.low;
            state = 6;
            break;
      
          case 6: // SECTOR FOOTER -------------------------------------------------            
            sector_byte = 0x4E;
            state = 7;
            break;      

          case 7:
            if (++b_index != 56) break; // 56 in FDD, increased for stability!
            if (sector != 16)
            {
                state = 2;
                b_index = 0;
                break;
            }
            // TRACK END -------------------------------------------------
            USART_disable();
            data_sent = 2; // set flag indicates end of track, for reinitialize track data and read new sectors
            break;
        } // GET DATA BYTE END

    }

    ISR_END:;
}


/// FDD Emulator initialization
///////////////////////////////////////////
void emu_init()
{
    cli(); // DISABLE GLOBAL INTERRUPTS

    // sector header generation
    memset(&sector_header[0x00],0x00,60);   // 0x00(0)-0x0B(11) sync field    
    memset(&sector_header[0x0C],0xA1,3);    // 0x0C(12)-0x0E(14) 3x0xA1
    sector_header[0x0F] = 0xFE;             // 0x0F(15) 0xFE
    sector_header[0x13] = 0x01;             // 0x13(19) sector len (256 bytes)
    memset(&sector_header[0x16],0x4E,22);   // 0x16(22)-0x2B
    memset(&sector_header[0x38],0xA1,3);    // 0x38(56)-0x3A
    sector_header[0x3B] = 0xFB;             // 0x3B(59)

    // Setup USART in MasterSPI mode 500000bps
    UBRR0H = 0x00;
    UBRR0L = 0x0F; // 500 kbps
    UCSR0C = 0xC0;
    UCSR0A = 0x00;
    UCSR0B = 0x00; // disabled

    //INIT INT0 interrupt
    EICRA = 0x03; // INT0 (rising edge=0x03) RISING EDGE IS IMPORTANT!!!

    // INIT pins and ports
    // ALL OUTPUT PINS WORK IN HI-Z MODE!!! ALL INPUT PINS SHOULD BE WITH PULL UP!!!
    PORTC |= _BV(DRIVE_SEL) | _BV(SIDE_SEL) ; // set pull_up
    PORTD |= _BV(STEP) | _BV(MOTOR_ON) | _BV(INDEX); // set pull-up // | _BV(WRT_GATE)
    PORTB |= _BV(DIR_SEL); // set pull-up
    PORTD &= ~(_BV(WP) | _BV(TRK00)); // disable pullup for HI-Z mode on these pins
  
    // Init SPI for SD Card
    SPI_DDR = _BV(SPI_MOSI) | _BV(SPI_SCK) | _BV(SPI_CS); //set output mode for MOSI, SCK ! move SS to GND
    SPCR = _BV(MSTR) | _BV(SPE);   // Master mode, SPI enable, clock speed MCU_XTAL/4, LSB first
    SPSR = _BV(SPI2X);             // double speed

    sei();   // ENABLE GLOBAL INTERRUPTS
}

///
/// MAIN Routine
///////////////////////////////////////////
int main() {  
  
    //init(); // init arduino libraries

    emu_init(); // initialize FDD emulator

    while(1)
    { // MAIN LOOP START
        /// MAIN LOOP USED FOR SELECT and INIT SD CARD and other
      
        //>>>>>> print "NO CARD PRESENT" on LCD
        while(pf_mount(&fat) != FR_OK);
        //>>>>>> print "CARD INFO etc..."

        //>>>>>> SELECT TRD IMAGE HERE

        byte sdhc = getCardType() & CT_SDHC;
        uint8_t chained = fat.csize < 8;
        uint8_t clusters_per_track = 8 / fat.csize;
        uint8_t tracks_per_cluster = fat.csize / 8;        

        while (1)
        { /// DRIVE SELECT LOOP

            uint8_t chain_index, track, track_sect;

            while ( (PIND & _BV(MOTOR_ON)) || (PINC & _BV(DRIVE_SEL))); // wait drive select && motor_on

            /// DEVICE ENABLED ==========================================================================================================================

            PORTD |= _BV(INDEX); // set INDEX HIGH
            DDRD |= _BV(WP) | _BV(TRK00) | _BV(INDEX); // Set WP and TRK00 output and LOW, INDEX - HIGH
        
            
            /////////////////////////////////////////////////////////////////
            // INIT after drive selected
            
            if(pf_open("default.trd") != FR_OK) break; // if unable to open file, usually if SD card is removed

            max_track = (fat.fsize/4096 < MAX_TRACK)?fat.fsize/4096:MAX_TRACK; // calculate maximal cylinder
            max_cylinder = max_track / 2; // calculate maximal cylinder

            /// FAST create cluster table for tracks ----------------------------------------------------------------------------------------
            clust_table[0] = fat.org_clust;
            uint32_t cur_fat, cur_fat_sector = fat.org_clust / 128;
            card_read_sector(sector_data, fat.fatbase + cur_fat_sector); // read data_block with start cluster number            
            for(uint8_t i = 1; i < max_track; i++) 
            {
                cur_fat = clust_table[i-1];
                                
                if(tracks_per_cluster == 0)
                {
                    for(uint8_t k = 0; k < clusters_per_track; k++)
                    {
                        if( (cur_fat >> 7) != cur_fat_sector )
                        {
                            cur_fat_sector = cur_fat / 128;
                            card_read_sector(sector_data, fat.fatbase + cur_fat_sector); // read data_block with current cluster number
                        }
                        cur_fat = (uint32_t)(*(uint32_t*)(&sector_data[0][0] + ((uint8_t)cur_fat % 128) * 4));
                    }
                }
                else
                {
                    if( i % tracks_per_cluster == 0)
                    {
                        if( (cur_fat >> 7) != cur_fat_sector )
                        {
                            cur_fat_sector = cur_fat / 128;
                            card_read_sector(sector_data, fat.fatbase + cur_fat_sector); // read data_block with current cluster number
                        }
                        cur_fat = (uint32_t)(*(uint32_t*)(&sector_data[0][0] + ((uint8_t)cur_fat % 128) * 4));
                    }
                }   
                clust_table[i] = cur_fat;
            } // --------------------------------------------------------------------------------------------------------------------------------
                        
            INT0_enable(); // ENABLE INDERRUPT (STEP pin)

            // update values for current track
            data_sent = 2;
            cylinder = cylinder_changed = 0;
        
            do { // READ DATA SEND LOOP (send data from FDD to FDD controller)  
            //-------------------------------------------------------------------------------------------
                while (data_sent == 0); // wait until sector data of track is not completely sent
                if( data_sent == 2 ) // initialize track data for next round
                {
                    if(cylinder_changed)
                    {
                        send_cmd(CMD12,0,0); // stop transmission from SD card if partial sector read;
                        DESELECT();
                        t_millis = 0;
                        TCCR0A = 0;
                        TCCR0B = 3;    // 3 = 1024mcs overflow ~ 1ms
                        TIMSK0 = 1;   // enable timer interrupt
                        while(easy_millis() < 15);
                        TIMSK0 = 0;
                        do {
                            cylinder_changed = 0;
                            t_millis = 0;
                            TCCR0A = 0;
                            TCCR0B = 3;    // 3 = 1024mcs overflow ~ 1ms
                            TIMSK0 = 1;   // enable timer interrupt
                            while(easy_millis() < 4);
                            TIMSK0 = 0;
                        } while(cylinder_changed);                      
                    }
                    // set initial values for current track
                    sector_header[16] = cylinder;  // current Floppy cylinder
                    if(sector_header[16] == 0) DDRD |= _BV(TRK00); else DDRD &= ~_BV(TRK00); // Set TRK00 Low or HI-Z
                    sector_header[17] = PINC & _BV(SIDE_SEL) ? 0 : 1;
                    track = sector_header[16] * 2 + sector_header[17]; // track number
                    track_sect = track << 3;
                    fat.dsect = fat.database + (clust_table[track] - 2) * fat.csize + (track_sect % fat.csize); // track start LBA number on SD card
                    track_sect += 2;

                    if(chained)
                    { // prepare cluster chain if cluster is less 4K
                        cluster_chain[0] = get_fat(clust_table[track]);
                        for(chain_index = 1; chain_index < clusters_per_track; chain_index++) cluster_chain[chain_index] = get_fat(cluster_chain[chain_index-1]);
                        chain_index = 0;
                    }
                    
                    //>>>>>> print "CYLINDER, HEAD INFO" or track number on LCD

                    for(uint8_t i=0; i < 255; i++) { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); }
                    card_read_sector(sector_data,fat.dsect++); // read 2 floppy sectors from SD card (1 SD card sector) and increase LBA
                    
                    sector_byte = 0x4E;
                    state = b_index = sector = tmp = data_sent = 0;
                    
                    USART_enable(); // Enable DATA transmit interrupt                    
                }
                else
                {
                    // on cluster boundary, get next cluster number and calculate LBA  ( only if cluster on SD card is less than 4k !!! )
                    if((sector % 2) == 1)
                    { // read SC card first half of the sector (odd floppy sector)
                        if( chained )
                         if ((++track_sect % fat.csize) == 0 )
                            fat.dsect = fat.database + (cluster_chain[chain_index++]-2) * fat.csize;
                    
                        // FAST SD card sector loading (read 2 floppy sectors and increase LBA)
                        uint32_t lba = fat.dsect++;
                        if (!sdhc) lba *= 512;    // SDHC - LBA = block number (512 bytes), other - LBA = byte offset
                        send_cmd(CMD17, lba, 0);
                        do { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); } while(SPDR == 0xFF);
                        for(uint8_t i=0 ;; i++) { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); sector_data[0][i]=SPDR; if(i==0xFF) break;}
                    }
                    else
                    { // read SC card first second half of the sector (even floppy sector)
                        for(uint8_t i=0 ;; i++) { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); sector_data[1][i]=SPDR; if(i==0xFF) break;}
                        DESELECT();
                    }
                }                                        
                
                while(data_sent == 1); // wait whule data is start transmitting
        
            } while (!(PIND & _BV(MOTOR_ON)) && !(PINC & _BV(DRIVE_SEL))); // READ DATA SEND LOOP END
            //-------------------------------------------------------------------------------------------
    
            USART_disable(); // disable interrupt after sending track
            INT0_disable(); // DISABLE INDERRUPT (STEP pin)
            DDRD &= ~(_BV(WP) | _BV(TRK00) | _BV(INDEX)); // Set WP and TRK00 as input and HI-Z to not affect other floppies
            PORTD |= _BV(INDEX); // set pullup on index

            /// DEVICE DISABLED =========================================================================================================================

        } /// DRIVE SELECT LOOP END
      
    } // MAIN LOOP END
  
} // END MAIN


