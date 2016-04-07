// ZX-Spectrum FDD Emulator
//
#define SIDE_SEL  PC0   // pin A0, SIDE SELECT                                  (INPUT)
#define DRIVE_SEL PC1   // pin A1, DRIVE SELECT CONNECT DS0-DS3 using jumper    (INPUT)

#define DIR_SEL   PB0   // pin 8,  DIRECTION SELECT                             (INPUT)
  
#define WRIT_DATA PD0   // pin 0,  WRITE DATA                                   (INPUT) /// not used yet
#define READ_DATA PD1   // pin 1,  READ_DATA                                    (OUTPUT) /// defined in USART
#define STEP      PD2   // pin 2,  STEP                                         (INPUT)
#define WRT_GATE  PD3   // pin 3,  WRITE GATE                                   (INPUT) /// not used yet
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
uint32_t cluster_chain[4]; // max cluster chain 4 is for 1k cluster or less if higher

// STATE variable values
// 0-2    - TRACK HEADER
// 4-9    - SECTOR
// 10-11  - TRACK FOOTER
uint8_t state, max_cylinder, max_track, second_byte, sector_even_odd, s_side, s_cylinder, sector_byte, tmp, b_index, cylinder, track_changed, sector;
volatile uint8_t data_sent; // this is important!!!
uint16_t CRC, CRC_tmp;

// inverted table for fast converting
uint8_t MFM_tab_inv[16] = {0x55,0x56,0x59,0x5A,0x65,0x66,0x69,0x6A,0x95,0x96,0x99,0x9A,0xA5,0xA6,0xA9,0xAA};

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
void inline USART_enable_TX() { UCSR0B |= 0x28; }
void inline USART_enable_RX() { UCSR0B |= 0x30; }
void inline USART_disable() { UCSR0B = 0x00; }
void inline INT0_enable() { EIMSK |= 0x01; }
void inline INT0_disable() { EIMSK &= ~0x01; }


///
/// STEP pin interrupt
///////////////////////////////////////////
ISR(INT0_vect)
{
    if(PINB & _BV(DIR_SEL))
    {
        if(cylinder > 0) cylinder--;
    }
    else
        if(cylinder < max_cylinder) cylinder++;

    USART_disable();
    PORTD |= _BV(INDEX); // Set INDEX - HIGH it may be LOW in USART interrupt
    track_changed = 1;
    data_sent = 2;
}


///
/// READ DATA interrupt
///////////////////////////////////////////
ISR(USART_UDRE_vect)
{ 
    if (tmp == 0)
    { // Send first MFM byte
        UDR0 = MFM_tab_inv[sector_byte >> 4];  // put byte to send buffer
        tmp = 1;
    }
    else
    { // Send second MFM byte
        UDR0 = second_byte ? second_byte : MFM_tab_inv[sector_byte & 0x0f];
        tmp = 0; // this is important!

        // GET NEXT DATA BYTE (REAL DATA NOT MFM)
        switch (state)
        {
          case 0: // start track header -----------------------------------------
            PORTD &= ~_BV(INDEX); // Set INDEX - LOW
            sector_byte = 0x4E;
            state = 1;
            b_index = 1;
            break;
      
          case 1: // send track header ------------------------------------------
            if (b_index++ != 80) break; // 80 in FDD
            state = 2;
            b_index = 0;            
            break;

          case 2: // track C2 field ---------------------------------------------
            // this field is important for stable work!!! but may be skipped :)
            switch(b_index++)
            {
                case 0: sector_byte = 0x00; break;
            
                case 12: sector_byte = 0xC2; // translate this value!
                case 13:
                case 14:
                  second_byte = 0xDB; // inverted 0x24;
                  break;            
                
                case 15: second_byte = 0; sector_byte = 0xFC; break;            
                
                case 16: sector_byte = 0x4E;
                  b_index = 1;
                  state = 3;
                  break;
            }
            break;
            
          case 3:
            if (++b_index != 50) break; // 50 in FDD
            PORTD |= _BV(INDEX); // Set INDEX - HIGH
            b_index = 0;
            state = 4;
            break;
            
          case 4: // SECTOR START (ADDERSS FIELD) ------------------------------
          {
            switch(b_index)
            {
              case 0:
                // THE BEST SOLUTION!!! :)
                sector_header[17] = s_side;
                if ((PINC & 1) == sector_header[17]) // WARNING!!! SIDE_SEL pin used!!!
                {
                    USART_disable();
                    data_sent = 2; // set flag indicates end of track, for reinitialize track data and read new sectors
                    goto ISR_END;
                }
                break;             

              case 1: CRC = 0xB230; sector_header[16] = s_cylinder; break;
              case 2: CRC_tmp = pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ sector_header[16])); break;
              case 3: CRC = (CRC << 8) ^ CRC_tmp; break;
              case 4: CRC_tmp = pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ sector_header[17])); break;
              case 5: CRC = (CRC << 8) ^ CRC_tmp; break;
              case 6: sector_header[18] = sector + 1; break;
              case 7: CRC_tmp = pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ (sector + 1))); break;
              case 8: CRC = (CRC << 8) ^ CRC_tmp; break;
              case 9: CRC_tmp = pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ 1)); break;
              case 10: CRC = (CRC << 8) ^ CRC_tmp; break;
                                                        
              case 12:
              case 13:
              case 14:
                second_byte = 0x76; //inverted 0x89;
                break;

              case 15: second_byte = 0; break;

              case 20: sector_header[20] = (byte)(CRC >> 8); break;
              case 21: sector_header[21] = (byte)CRC; break;

              case 22: data_sent = 0; break; /// very important!!!

              case 41: sector_even_odd = sector & 1; break;
              case 42: CRC = 0xE295; break; // START GENERATING CRC HERE, PRE-CALC value for A1,A1,A1,FB = 0xE295 next CRC value

              case 56:
              case 57:
              case 58:
                second_byte = 0x76; //inverted 0x89;
                break;

              case 59: second_byte = 0; break;
            }
            // send sector bytes before data
            sector_byte = sector_header[b_index];   // pre-get new byte from buffer
            
            if (++b_index != 60) break;
            b_index = 0;
            state = 5;            
            break;
          }
          case 5: // DATA FIELD ----------------------------------------------------        
            // get sector data values
            sector_byte = sector_data[sector_even_odd][b_index++];   // pre-get new byte from buffer
            CRC = (CRC << 8) ^ pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ sector_byte));
            if (b_index != 0) break;
            state = 6;
            break;
        
          case 6: // SEND SECTOR CRC -----------------------------------------------
            if(++sector != 16) data_sent = 1;  // increase sector, reset flag indicates end of sector, for reading new sectors
            sector_byte = (uint8_t)(CRC >> 8);
            state = 7;
            break;

          case 7:
            sector_byte = (uint8_t)CRC;
            state = 8;
            break;
      
          case 8: // SECTOR FOOTER -------------------------------------------------            
            sector_byte = 0x4E;
            state = 9;
            break;      

          case 9:
            if (++b_index != 56) break; // 56 in FDD, increased for stability!!!
            if (sector != 16)
            {
                state = 4;
                b_index = 0;
                break;
            }
            state = 10;
            break;
            
          case 10: // TRACK FOOTER -------------------------------------------------           
            state = 11;
            b_index = 0;
            break;

          case 11:
            if (++b_index != 200) break; // > 500 in FDD
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
    EICRA = 0x03; // INT0 (falling edge=0x02, rising edge=0x03)

    // INIT pins and ports
    // ALL OUTPUT PINS WORK IN HI-Z MODE!!! ALL INPUT PINS SHOULD BE WITH PULL UP!!!
    PORTC |= _BV(DRIVE_SEL) | _BV(SIDE_SEL) ; // set pull_up
    PORTD |= _BV(STEP) | _BV(MOTOR_ON) | _BV(WRT_GATE) | _BV(INDEX); // set pull-up
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
      
        //>>>>>> print "NO CARD PRESENT"
        while(pf_mount(&fat) != FR_OK);
        //>>>>>> print "CARD INFO, TRD IMAGE NAME"

        byte sdhc = getCardType() & CT_SDHC;
        uint8_t chained = fat.csize < 8;
        uint8_t clusters_per_track = 8 / fat.csize;
        uint8_t tracks_per_cluster = fat.csize / 8;        

        while (1)
        { /// DRIVE SELECT LOOP

            uint8_t chain_index, track, csize_mod = fat.csize-1;
            uint16_t track_sect;            

            while ( (PIND & _BV(MOTOR_ON)) || (PINC & _BV(DRIVE_SEL))); // wait drive select && motor_on

            PORTD |= _BV(INDEX); // set INDEX HIGH
            DDRD |= _BV(WP) | _BV(TRK00) | _BV(INDEX); // Set WP and TRK00 output and LOW, INDEX - HIGH
        
            
            /////////////////////////////////////////////////////////////////
            // INIT after drive selected
            
            if(pf_open("default.trd") != FR_OK) break; // if unable to open file, usually if SD card is removed

            max_track = (fat.fsize/4096 < MAX_TRACK)?fat.fsize/4096:MAX_TRACK; // calculate maximal cylinder
            max_cylinder = max_track / 2; // calculate maximal cylinder

            /// FAST create cluster table for tracks ----------------------------------------------------------------------------------------
            clust_table[0] = fat.org_clust;
            uint32_t cur_fat, cur_fat_sector = fat.org_clust >> 7;
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
                            cur_fat_sector = cur_fat >> 7;
                            card_read_sector(sector_data, fat.fatbase + cur_fat_sector); // read data_block with current cluster number
                        }
                        cur_fat = (uint32_t)(*(uint32_t*)(&sector_data[0][0] + ((uint8_t)cur_fat & 127) * 4));
                    }
                }
                else
                {
                    if( i % tracks_per_cluster == 0)
                    {
                        if( (cur_fat >> 7) != cur_fat_sector )
                        {
                            cur_fat_sector = cur_fat >> 7;
                            card_read_sector(sector_data, fat.fatbase + cur_fat_sector); // read data_block with current cluster number
                        }
                        cur_fat = (uint32_t)(*(uint32_t*)(&sector_data[0][0] + ((uint8_t)cur_fat & 127) * 4));
                    }
                }   
                clust_table[i] = cur_fat;
            } // --------------------------------------------------------------------------------------------------------------------------------
                        
            INT0_enable(); // ENABLE INDERRUPT (STEP pin)

            // update values for current track
            s_cylinder = cylinder = track_changed = 0;
            data_sent = 2;
        
            do { // READ DATA SEND LOOP (send data from FDD to FDD controller)  
            //-------------------------------------------------------------------------------------------
            REPEAT:
                while (data_sent == 0); // wait until sector data of track is not completely sent

                if( data_sent == 2 ) // initialize track data for next round
                {
                    // set initial values for current track
                    s_cylinder = cylinder; // current Floppy cylinder
                    if(s_cylinder == 0) DDRD |= _BV(TRK00); else DDRD &= ~_BV(TRK00); // Set TRK00 Low or HI-Z
                    s_side = PINC & _BV(SIDE_SEL) ? 0 : 1;
                    track = s_cylinder * 2 + s_side; // track number
                    track_sect = track * 8;
                    fat.dsect = fat.database + (clust_table[track] - 2) * fat.csize + (track_sect & csize_mod); // track start LBA number on SD card

                    if(chained)
                    { // prepare cluster chain if cluster is less 4K
                        cluster_chain[0] = get_fat(clust_table[track]);
                        for(chain_index = 1; chain_index < clusters_per_track; chain_index++) cluster_chain[chain_index] = get_fat(cluster_chain[chain_index-1]);
                        chain_index = 0;
                    }
                    //>>>>>> print "CYLINDER, HEAD INFO" or track number

                    card_read_sector(sector_data,fat.dsect++); // read 2 floppy sectors from SD card (1 SD card sector) and increase LBA
                    
                    state = b_index = sector = second_byte = tmp = 0;

                    if(track_changed) { track_changed = 0; goto REPEAT; }
                    
                    sector_byte = 0x4E;
                    data_sent = 0;
                    USART_enable_TX(); // Enable DATA transmit interrupt                    
                }
                else
                {
                    if((sector & 1) == 0)
                    {
                        if( chained && ((track_sect + sector + 1) & csize_mod) == 0 ) // on cluster boundary, get next cluster number and calculate LBA  ( only if cluster on SD card is less than 4k !!! )
                            fat.dsect = fat.database + (cluster_chain[chain_index++]-2) * fat.csize;

                        // FAST SD card sector loading (read 2 floppy sectors and increase LBA)
                        uint32_t lba = fat.dsect++;
                        if (!sdhc) lba <<= 9;    // SDHC - LBA = block number (512 bytes), other - LBA = byte offset
                        send_cmd(CMD17, lba, 0);
                        do {
                            SPDR = 0xFF;
                            loop_until_bit_is_set(SPSR, SPIF);                          
                        } while(SPDR == 0xFF);
                        uint8_t i = 0;
                        do { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); sector_data[0][i++]=SPDR; } while(i!=0);
                        do { SPDR = 0xFF; loop_until_bit_is_set(SPSR, SPIF); sector_data[1][i++]=SPDR; } while(i!=0);
                        DESELECT();
                    }
                }                        
            
                while(data_sent == 1); // wait until sector data is start transmitting
        
            } while (!(PIND & _BV(MOTOR_ON)) && !(PINC & _BV(DRIVE_SEL))); // READ DATA SEND LOOP END
            //-------------------------------------------------------------------------------------------
    
            USART_disable(); // disable interrupt after sending track
            INT0_disable(); // DISABLE INDERRUPT (STEP pin)
            DDRD &= ~(_BV(WP) | _BV(TRK00) | _BV(INDEX)); // Set WP and TRK00 as input and HI-Z to not affect other floppies
            PORTD |= _BV(INDEX); // set pullup on index

        } /// DRIVE SELECT LOOP END
      
    } // MAIN LOOP END
  
} // END MAIN


