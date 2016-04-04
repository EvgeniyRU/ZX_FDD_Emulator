// ZX-Spectrum FDD Emulator
//
#define SIDE_SEL  PC0   // pin A0, SIDE SELECT                                  (INPUT)
#define DRIVE_SEL PC1   // pin A1, DRIVE SELECT CONNECT DS0-DS3 using jumper    (INPUT) /// not used yet

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

#define MAX_CYL 86            /// maximal cylinder supported by FDD
#define MAX_TRACK MAX_CYL*2  /// maximal track

uint8_t sector_header[64]; // sector header
uint8_t sector_data[2][256]; // sector data
uint32_t clust_table[MAX_TRACK]; // Cluster table
uint32_t cluster_chain[4]; // max cluster chain 4 is for 1k cluster or less if higher

// STATE variable values
// 0-2    - TRACK HEADER
// 4-9    - SECTOR
// 10-11  - TRACK FOOTER
uint8_t state, max_cylinder, prev_byte, second_byte, sector_even_odd, s_side, s_cylinder, sector_byte, sector, tmp, b_index;
volatile uint8_t data_sent, cylinder;
uint16_t CRC;

uint8_t MFM_tab[32] = { 0xAA,0xA9,0xA4,0xA5,0x92,0x91,0x94,0x95,0x4A,0x49,0x44,0x45,0x52,0x51,0x54,0x55,0x2A,0x29,0x24,0x25,0x12,0x11,0x14,0x15,0x4A,0x49,0x44,0x45,0x52,0x51,0x54,0x55 };

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
void inline USART_enable() { UCSR0B |= 0x20; }
void inline USART_disable() { UCSR0B &= ~0x20; }
void inline INT0_enable() { EIMSK |= 0x01; }
void inline INT0_disable() { EIMSK &= ~0x01; }

///
/// STEP pin interrupt
///////////////////////////////////////////
ISR(INT0_vect)
{
  if(PINB & _BV(DIR_SEL)) {
    if(cylinder > 0) cylinder--;
  }
  else
    if(cylinder < max_cylinder) cylinder++;
}

///
/// READ DATA interrupt
///////////////////////////////////////////
ISR(USART_UDRE_vect)
{ 
  if (!tmp) // it is ok, as MFM_tab doesn't have zero values
  { 
    // GET NEXT DATA BYTE (REAL DATA NOT MFM)
    switch (state)
    {
      case 0: // start track header -----------------------------------------
            if(s_cylinder == 0) DDRD |= _BV(TRK00); else DDRD &= ~_BV(TRK00); // Set TRK00 Low or HI-Z
            sector_byte = 0x4E;
            state = 1;            
            break;
      case 1: // start track header -----------------------------------------
            DDRD |= _BV(INDEX); // Set INDEX - LOW
            state = 2;            
            break;
      case 2: // send track header ------------------------------------------
            if (++b_index != 80) break; // 80 in FDD
            state = 4;
            b_index = 0;
            DDRD &= ~_BV(INDEX); // Set INDEX - HI-Z
            break;            

      case 4: // SECTOR START (ADDERSS FIELD) ------------------------------
      {
            uint8_t send_immed = 1;
            switch(b_index)
            {
              case 0: CRC = 0xB230; break;          

              case 1: sector_header[16] = s_cylinder; break;
              case 2: CRC = (CRC << 8) ^ pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ s_cylinder)); break;
              case 3: sector_header[17] = ((~(PINC & _BV(SIDE_SEL))) & 1)==s_side?s_side:2; break; // THE BEST SOLUTION!!! :)
              case 4: CRC = (CRC << 8) ^ pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ s_side)); break;
              case 5: sector_header[18] = sector + 1; break;
              case 6: CRC = (CRC << 8) ^ pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ (sector + 1))); break;
              case 7: CRC = (CRC << 8) ^ pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ 1)); break;
              case 8: sector_header[20] = (byte)(CRC >> 8); break;
              case 9: sector_header[21] = (byte)CRC; break;
                                                        
              case 12:
              case 13:
              case 14:
                second_byte = 0x89;
                break;

              case 56:
              case 57:
              case 58:
                second_byte = 0x89;
                break;
            }
            // send sector bytes before data
            if(send_immed) sector_byte = sector_header[b_index];   // pre-get new byte from buffer
            
            if (++b_index != 60) break;
            b_index = 0;
            state = 5;
            // START GENERATING CRC HERE, PRE-CALC value for A1,A1,A1,FB = 0xE295        
            CRC = 0xE295; // next CRC value
            data_sent = 0;
            sector_even_odd = sector & 1;
            break;
      }
      case 5: // DATA FIELD -------------------------------------------------        
            // get sector data values
            sector_byte = sector_data[sector_even_odd][b_index++];   // pre-get new byte from buffer
            CRC = (CRC << 8) ^ pgm_read_word_near(Crc16Table + ((CRC >> 8) ^ sector_byte));
            if (b_index != 0) break;
            state = 6;
            if (sector <= 14)
            {
              sector++; // increase sector
              data_sent = 1;  // reset flag indicates end of sector, for reading new sectors
            }            
            break;
        
      case 6: // SECTOR CRC ----------------------------------------------------
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
            b_index++;
            if (b_index != 80) break; // 56 in FDD, increased for stability!!!
            if (data_sent)
            {
                state = 4;
                b_index = 0;
                break;
            }
            state = 10;
            break;
            
      case 10: // TRACK FOOTER -------------------------------------------------           
            sector = 0;
            data_sent = 0;  // reset flag indicates end of track, for reinitialize track data and read new sectors            
            state = 11;
            b_index = 0;
            break;

      case 11:
            if (++b_index != 100) break; // > 500 in FDD
            state = 0;
            b_index = 0;
            data_sent = 2;
            USART_disable(); // disable interrupt after sending track
            goto ISR_END;
            break;
    } // GET DATA BYTE END

    tmp = MFM_tab[sector_byte >> 4]; // get first MFM byte from table    
    if(prev_byte && !(sector_byte & 0x80)) tmp &= 0x7F;
    UDR0 = ~tmp;  // put byte to send buffer
  }
  else
  { // Send second MFM byte
    prev_byte = sector_byte & 1;

    if (second_byte == 0)
      tmp = MFM_tab[sector_byte & 0x1f]; // get second MFM byte from table to "tmp"
    else 
    {
      tmp = second_byte;
      second_byte = 0;
    }    

    UDR0 = ~tmp;  // put byte to send buffer
    
    tmp = 0; // this is important!
  }
ISR_END:;
}

///
/// Prepare sector header template
///////////////////////////////////////////
void prepare_sector_header()
{
  // Address field
  byte i;
  for(i=0; i <= 11; i++) sector_header[i] = 0x00; // 0x00(0)-0x0B(11) sync field
  for(i=12; i <= 14; i++) sector_header[i] = 0xA1; // 0x0C(12)-0x0E(14) 3x0xA1
  sector_header[15] = 0xFE;         // 0x0F(15) 0xFE
  // cylinder, SIDE, SECTOR
  sector_header[16] = 0x00;         // 0x10(16) cylinder
  sector_header[17] = 0x00;         // 0x11(17) side
  sector_header[18] = 0x01;         // 0x12(18) sector
  sector_header[19] = 0x01;         // 0x13(19) sector len (256 bytes)
  sector_header[20] = 0xFA;         // 0x14(20) CRC1 for trk=0, side=0, sector=1
  sector_header[21] = 0x0C;         // 0x15(21) CRC2
  // GAP 2
  for(i=22; i <= 43; i++) sector_header[i] = 0x4E; // 0x16(22)-0x2B
  // DATA field
  for(i=44; i <= 55; i++) sector_header[i] = 0x00; // 0x2C(44)-0x37 sync field
  for(i=56; i <= 58; i++) sector_header[i] = 0xA1; // 0x38(56)-0x3A
  sector_header[59] = 0xFB;         // 0x3B(59)
}

/// FDD Emulator initialization
///////////////////////////////////////////
void emu_init()
{
  cli(); // DISABLE GLOBAL INTERRUPTS

  prepare_sector_header();

  // Setup USART in MasterSPI mode 500000bps
  UBRR0H = 0x00;
  UBRR0L = 0x0F; // 500 kbps
  UCSR0C = 0xC0;
  UCSR0A = 0x00;
  UCSR0B = 0x08; // disabled

  //INIT INT0 interrupt
  EICRA = 0x03; // INT0 (falling edge=0x02, rising edge=0x03)

  // INIT pins and ports
  // ALL OUTPUT PINS WORK IN HI-Z MODE!!! ALL INPUT PINS SHOULD BE WITH PULL UP!!!
  PORTC |= _BV(DRIVE_SEL) | _BV(WRT_GATE) ; // set pull_up
  PORTD |= _BV(STEP) | _BV(MOTOR_ON) | _BV(SIDE_SEL); // set pull-up
  PORTB |= _BV(DIR_SEL); // set pull-up
  PORTD &= ~(_BV(WP) | _BV(TRK00) | _BV(INDEX)); // disable pullup for HI-Z mode on these pins
  
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
  
      while (1)
      { /// DRIVE SELECT LOOP

        uint8_t chain_index, chain_size, chain_gen_index, track, csize_mod = fat.csize-1;

        while ( (PIND & _BV(MOTOR_ON)) || (PINC & _BV(DRIVE_SEL))); // wait drive select && motor_on

        DDRD |= _BV(WP) | _BV(TRK00); // Set WP and TRK00 output and LOW
        
        /////////////////////////////////////////////////////////////////
        // INIT after drive selected
            
        if(pf_open("default.trd") != FR_OK) break;

        chain_size = 8 / (fat.csize+1);

        max_cylinder = (fat.fsize/8192 <= MAX_CYL)?fat.fsize/8192:MAX_CYL; // calculate maximal cylinder

        ///create cluster table for tracks
        clust_table[0] = fat.org_clust;
        uint8_t fats_per_track = 8 / fat.csize;
        uint8_t tracks_per_cluster = fat.csize / 8;
        for(uint8_t i = 1; i < MAX_TRACK; i++) 
        {
          uint32_t cur_fat = clust_table[i-1];
          if(tracks_per_cluster == 0)
          {
              for(uint8_t k = 0; k < fats_per_track; k++) cur_fat = get_fat(cur_fat);
          }
          else
              if( i % tracks_per_cluster == 0) cur_fat = get_fat(cur_fat);
          clust_table[i] = cur_fat;
        }

        INT0_enable(); // ENABLE INDERRUPT (STEP pin)

        // update values for current track
        state = s_cylinder = cylinder = prev_byte = second_byte = sector = tmp = b_index = 0;
        
        data_sent = 2;        
        
        do { // READ DATA SEND LOOP (send data from FDD to FDD controller)  
        //-------------------------------------------------------------------------------------------

            while (!data_sent); // wait until sector data of track is not completely sent

            if( data_sent == 2 ) // initialize track data for next round
            {
                // set initial values for current track
                s_cylinder = cylinder; // current Floppy cylinder
                s_side = (~(PINC & _BV(SIDE_SEL))) & 1; // current Floppy HEAD (side)
                
                //>>>>>> print "CYLINDER, HEAD INFO"
                
                track = s_cylinder * 2 + s_side; // track number
                fat.curr_clust = clust_table[track]; // track first cluster in FAT32
                fat.dsect = fat.database + (fat.curr_clust - 2) * fat.csize + ((track << 3) & csize_mod); // track start LBA number on SD card
                chain_index = chain_gen_index = 0;
                card_read_sector(sector_data,fat.dsect++); // read 2 floppy sectors from SD card (1 SD card sector) and increase LBA
                USART_enable(); // Enable DATA transmit interrupt
            }
            else
            {
                if((sector & 1) == 0)
                {
                    if( (((track << 3) + sector + 1) & csize_mod) == 0 )
                    { // on cluster boundary, get next cluster number and calculate LBA  ( only if cluster on SD card is less than 4k !!! )
                        fat.curr_clust = cluster_chain[chain_index++];
                        fat.dsect = fat.database + (fat.curr_clust - 2) * fat.csize;
                    }
                    card_read_sector(sector_data,fat.dsect++); // read 2 floppy sectors from SD card (1 SD card sector) and increase LBA
                }
                else if(chain_size > (sector >> 1))
                { // create cluster chain table when not reading sectors (for FAT32 clusters less then 4k)
                    cluster_chain[chain_gen_index] = (chain_gen_index == 0) ? get_fat(fat.curr_clust) : get_fat(cluster_chain[chain_gen_index-1]);
                    chain_gen_index++;
                }
            }
            //if(!data_sent) {cli(); while(1);} // temporary check for speed errors
                        
            while(data_sent); // wait until sector data is start transmitting
        
        } while (!(PIND & _BV(MOTOR_ON)) && !(PINC & _BV(DRIVE_SEL))); // READ DATA SEND LOOP END
        //-------------------------------------------------------------------------------------------
    
        
        INT0_disable(); // DISABLE INDERRUPT (STEP pin)

        DDRD &= ~(_BV(WP) | _BV(TRK00)); // Set WP and TRK00 as input and HI-Z to not affect other floppies

      } /// DRIVE SELECT LOOP END
      
  } // MAIN LOOP END
  
} // END MAIN
