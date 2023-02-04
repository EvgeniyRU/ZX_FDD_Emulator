// ZX-Spectrum FDD Emulator
//
// It use TRD file from SD Card to emulate floppy disk. File is selected by encoder or buttons (set in Config.h).
// SD CARD: Supported only FAT32 file system, cards SDC/SDHC/MMC, all cluster sizes from 512 bytes to 64k
// Read-only mode, write mode is not supported!
//

#include "Config.h"
#include "SDCardModule.h"
#include "Fat32Module.h"
#include "LCDModule.h"

/// EMULATOR START -------------------------------------------------------------------------------------------------

/// Global variables
uint8_t cylinder_changed, max_cylinder, cylinder, prev_byte, A1_mark = 0;

///////
/// Interrupts enable/disable functions
///////////////////////////////////////////
void inline USART_enable()
{
    cli();
    /* Set MSPI mode of operation and SPI data mode 0. */
    UCSR0C = _BV(UMSEL01) | _BV(UMSEL00);    
    UCSR0B |= _BV(TXEN0);
    sei();
}
void inline USART_disable()
{
    cli();
    UCSR0C &= ~(_BV(UMSEL01) | _BV(UMSEL00));
    UCSR0B &= ~_BV(TXEN0);
    sei();
}
void inline PCINT1_enable() { PCIFR  |= _BV(PCIE1); PCICR  |= _BV(PCIE1); }
void inline PCINT1_disable() { PCICR &= ~_BV(PCIE1); }
void inline PCINT2_enable() { PCIFR  |= _BV(PCIE2); PCICR  |= _BV(PCIE2); }
void inline PCINT2_disable() { PCICR &= ~_BV(PCIE2); }


#if (USE_ENCODER == 1)  // if encoder selected in config
///////
/// ENCODER interrupt
volatile int8_t encoder_val = 0; // this is important!
uint8_t prev_pc = 0;
///////////////////////////////////////////
ISR(PCINT1_vect)
{
    uint8_t pc_val = PINC & (_BV(ENC_A) | _BV(ENC_B)), A=0, B=0;
    if(prev_pc == (_BV(ENC_A) | _BV(ENC_B)) && pc_val != 0)
    {        
	for(uint8_t i = 0; i < 10; i++)
        {
            if(PINC & _BV(ENC_A)) A++;
            if(PINC & _BV(ENC_B)) B++;
        }
	if(A > 8) encoder_val++; else if(B > 8) encoder_val--;
    }
    prev_pc = pc_val;
}
#endif    // end - if encoder selected in config

///////
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

///////
/// Send byte in MFM as 4 bytes at speed 1000000bps
////////////////////////////////////////////////////////////////
void send_byte(uint8_t sector_byte)
{
    /// inverted, very small MFM table for fast converting
    static uint8_t MFM_tab[8] = { 0x77,0x7D,0xDF,0xDD,0xF7,0xFD,0xDF,0xDD };

    uint8_t tmp = sector_byte >> 6; // get first MFM byte from table (first 4 bits)
    if((prev_byte & 1) && !(sector_byte & 0x80)) tmp |= 0x04; // check previous last bit and correct first clock bit of a new byte

    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
    UDR0 = MFM_tab[tmp];

    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
    UDR0 = MFM_tab[(sector_byte >> 4) & 0x07]; // get first MFM byte from table (second 4 bits)

    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
    UDR0 = A1_mark ? 0x7F : MFM_tab[(sector_byte >> 2)& 0x07]; // get second MFM byte from table (first 4 bits)

    prev_byte = sector_byte;

    loop_until_bit_is_set(UCSR0A,UDRE0); // wait USART buffer is ready for the next byte
    UDR0 = MFM_tab[sector_byte & 0x07]; // get second MFM byte from table (second 4 bits)

}

///////
/// Print 2 files on LCD and file pointer
////////////////////////////////////////////////////////////////
FILINFO disp_files[2], fnfo;
DIR dir, first_dir;
void print_files(uint8_t index)
{
    LCD_print_char(0,index,0);
    for(uint8_t i = 0; i < 2; i++)
    {
        if((disp_files[i].fattrib & AM_DIR) != 0) LCD_print_char(1,i,1); // display folder icon
        LCD_print(2,i,disp_files[i].fname); // display file name
    }
}

///////
/// f_array_ind - LCD display line number
/// dire - direction 0 - forward, 1 - backward
/// READ DIRECTORY ENTRY (1 file name) and put it to array (disp_files) for print on LCD
/////////////////////////////////////////////////////
int8_t readdir(uint8_t f_array_ind, uint8_t dire)
{
    for(;;)
    {
        if(dire)
        {
            if(!memcmp(&dir,&first_dir,sizeof(dir))) return -2;
            if(pf_dirprev(&dir) != FR_OK) return -2;
        }
        else
        {
            if(pf_dirnext(&dir) != FR_OK) return -2;
        }

        if(pf_readdir(&dir, &fnfo, dire) != FR_OK) return -1;   // read directory entry
        
        //if(fnfo.fname[0] != 0 && ( (  (strcasestr(fnfo.fname,".trd") || strcasestr(fnfo.fname,".scl") ) && (fnfo.fattrib & AM_DIR) == 0) || (fnfo.fattrib & AM_DIR) != 0) )
        if(fnfo.fname[0] != 0 && ( (  strcasestr(fnfo.fname,".trd") && (fnfo.fattrib & AM_DIR) == 0) || (fnfo.fattrib & AM_DIR) != 0) )
        {
            if( dire && !memcmp(&disp_files[0],&disp_files[1],sizeof(fnfo)-13) && !strncmp(disp_files[0].fname,disp_files[1].fname,12) )
		return 0;
          
            if(f_array_ind == 0) memcpy(&disp_files[1],&disp_files[0],sizeof(fnfo));
            if(f_array_ind == 1) memcpy(&disp_files[0],&disp_files[1],sizeof(fnfo));
            memcpy(&disp_files[f_array_ind%2],&fnfo,sizeof(fnfo));
            return 0;
        }
    }
    return -3;
}


uint8_t sector_data[256]; // sector data
//uint8_t SCL_buf[512]; // buffer for SCL files

uint32_t clust_table[MAX_CYL], sector_table[32]; // Cluster table, Cluster table for sectors in cylinder
union { uint16_t val; struct { byte low; byte high; } bytes; } CRC_H, CRC_D;
char dirs[MAX_DIR_LEVEL][13];
char *path;
FATFS fat;

///
/// MAIN Routine
///////////////////////////////////////////
int main()
{    
    //init(); // init arduino libraries

    LCD_init();

    /// INIT emulator --------------------------------------------------
    
    cli(); // DISABLE GLOBAL INTERRUPTS

    // Setup USART in MasterSPI mode 1000000bps
    UBRR0H = 0x00;

    UBRR0L = 0x07; // 1000 kbps for 16MHz external oscillator
    
    UCSR0A = 0x00;
    UCSR0B = 0x00; // disabled

    PCMSK2 |= _BV(PCINT20); // SET PCINT2 interrupt on PD4 (STEP pin)

#if (USE_ENCODER == 1)
    PCMSK1 |= _BV(PCINT10) | _BV(PCINT11); // SET PCINT1 (PC2, PC3) Encoder
#endif

    // INIT pins and ports
    PORTD |= _BV(STEP) | _BV(MOTOR_ON) | _BV(DRIVE_SEL) | _BV(DIR_SEL) | _BV(SIDE_SEL); // set pull-up
    PORTC |= _BV(ENC_A) | _BV(ENC_B) | _BV(BTN); // set pull-up
    
    DDRB &= ~_BV(INDEX); // SET INDEX HIGH
    DDRD &= ~(_BV(WP) | _BV(TRK00)); // Set WP,TRK00 as input
 
    // Init SPI for SD Card
    SPI_DDR = _BV(SPI_MOSI) | _BV(SPI_SCK) | _BV(SPI_CS); //set output mode for MOSI, SCK, CS(SS)
    SPCR = _BV(MSTR) | _BV(SPE);   // Master mode, SPI enable, clock rate f_osc/4, LSB first
    SPSR |= _BV(SPI2X);           // set double speed

    sei();   // ENABLE GLOBAL INTERRUPTS

    path = (char*)(sector_data + 32); // use as temporary buffer for path generation
    fat.buf = sector_data;

    /// ---------------------------------------------------------------
    
    uint8_t sector_byte, disp_index, f_index, btn_cnt, dir_level, first, pind;

    while(1)
    { // MAIN LOOP START
        /// MAIN LOOP USED FOR SELECT and INIT SD CARD and other

     MOUNT:
        PCINT1_disable();
        LCD_clear();
        LCD_print(F("NO CARD INSERTED"));
     NO_FILES:
        uint8_t eeprom_file = 0;
        memset(disp_files,0,sizeof(fnfo)*2);
        pf_mount(0);
        while(pf_mount(&fat) != FR_OK);

        LCD_clear();
        LCD_print(F(" CARD MOUNT OK."));
        
        uint32_t serial = card_read_serial();

        DESELECT(); // set SD card inactive
        eeprom_read_block((void*)path,(const void*)4,224); // read saved block with trd filename from eeprom
        
        if(path[0] != 0)
        {
            eeprom_file = 1;
            uint32_t serial2;
            eeprom_read_block((void*)&serial2,(const void*)0,4); // read saved card serial number from eeprom
            if(serial2 != serial) // compare saved serial number with current card serial number
            {
                eeprom_write_block((const void*)&serial, (void*)0, 4); // if not equal write sd card serial to eeprom
                eeprom_write_byte((uint8_t*)4, 0); // write zero value to eeprom for reset saved filename on next loop
                goto NO_FILES;
            }
            goto OPEN_FILE; // if serials equal jump to open file (file name read from eeprom)
        }

        /// SELECT TRD IMAGE HERE ----------------------------------------------------------------------------
        
        pf_opendir(&dir,"/");

        dir_level = 0;

        
DIRECTORY_LIST:
        LCD_clear();
        disp_index = 0, f_index = 0;

        first = 1;

        if(readdir(2,0) != 0) 
        {
            if(readdir(2,0) == 0) 
            {
                memcpy(&first_dir,&dir,sizeof(dir));
                f_index++;
            }
        }
        else
        {
             memcpy(&first_dir,&dir,sizeof(dir));
             f_index++;          
        }

        if(readdir(3,0) == 0) f_index++;
        
        if(!f_index)
        {
            if(!dir_level)
            {
                LCD_print(F(" -- NO FILES -- "));
                _delay_ms(3000);
                goto NO_FILES;
            }
            else
            {
                memset(disp_files,0,sizeof(fnfo)*2);
                f_index = 1;
                disp_index = 0;
                disp_files[disp_index].fname[0] = '.';
                disp_files[disp_index].fname[1] = '.';
                disp_files[disp_index].fattrib |= AM_DIR;
            }
        }
        
    FILE_LIST:
        LCD_clear();
        print_files(disp_index);


#if (USE_ENCODER == 1)
    // Encoder processing -----------------------------------------
        encoder_val = 0;
        prev_pc = 0;
        PCINT1_enable();
        while(PINC & _BV(BTN))
        {
            while(encoder_val == 0)
            {
              if(!(PINC & _BV(BTN))) break;
            }
            if( serial != card_read_serial() ) goto MOUNT;
            
            if(encoder_val > 0)
            { // read next directory entry
                cli();
                if(f_index > 1)
                {
                    if(disp_index == 0)
                    { // only move pointer
                        if(first) first = 0; else if(readdir(3,0) == -1) goto MOUNT;
                        disp_index=1;
                        LCD_print_char(0,1,0);
                        LCD_print_char(0,0,32);
                    }
                    else
                    { // load next entry
                        int8_t res = readdir(1,0);
                        if(res == 0)
                        {
                            first = 0;
                            LCD_clear();
                            print_files(disp_index);
                        }
                        else if(res == -1) goto MOUNT;
                    }
                }
                encoder_val = 0;
                sei();
            }
            else if(encoder_val < 0)
            { // read previous directory entry
                cli();
                if(f_index > 1)
                {
                    if(disp_index == 1)
                    { // only move pointer
                        if(readdir(2,1) == -1) goto MOUNT;
                        disp_index=0;                        
                        LCD_print_char(0,0,0);
                        LCD_print_char(0,1,32);
                    }
                    else if(!first)
                    { // load previous entry
                        int8_t res = readdir(0,1);
                        if(res == 0)
                        {
                            LCD_clear();
                            print_files(disp_index);
                        }
                        else if(res == -1) goto MOUNT;
                    }
                }
                encoder_val = 0;
                sei();
            }
        }
#else
    // Buttons processing -----------------------------------------
        while(PINC & _BV(BTN))
        {
              while((PINC & _BV(ENC_A)) && (PINC & _BV(ENC_B)))
              {
                  if(!(PINC & _BV(BTN))) break;
              }
              if( serial != card_read_serial() ) goto MOUNT;
              
              if(! (PINC & _BV(ENC_A) ))
              { // button A pushed
                PRESS_A_AGAIN:
                  if(f_index > 1)
                  {
                      if(disp_index == 0)
                      { // only move pointer
                          if(first) first = 0; else if(readdir(3,0) == -1) goto MOUNT;
                          disp_index=1;
                          LCD_print_char(0,1,0);
                          LCD_print_char(0,0,32);
                      }
                      else
                      { // load next entry
                          int8_t res = readdir(1,0);
                          if(res == 0)
                          {
                              first = 0;
                              LCD_clear();
                              print_files(disp_index);
                          }
                          else if(res == -1) goto MOUNT;
                      }
                  }
                  // wait button released
                  uint8_t wait = 0;
                  while(! (PINC & _BV(ENC_A)) )
                  {
                    _delay_ms(100);
                    if(++wait == 3) goto PRESS_A_AGAIN;
                  }
              }

              if(! (PINC & _BV(ENC_B)) )
              { // button B pushed
                PRESS_B_AGAIN:
                  if(f_index > 1)
                  {
                      if(disp_index == 1)
                      { // only move pointer
                          if(readdir(2,1) == -1) goto MOUNT;
                          disp_index=0;                        
                          LCD_print_char(0,0,0);
                          LCD_print_char(0,1,32);
                      }
                      else if(!first)
                      { // load previous entry
                          int8_t res = readdir(0,1);
                          if(res == 0)
                          {
                              LCD_clear();
                              print_files(disp_index);
                          }
                          else if(res == -1) goto MOUNT;
                      }
                  }
                  // wait button released
                  uint8_t wait = 0;
                  while(! (PINC & _BV(ENC_B)) )
                  {
                    _delay_ms(100);
                    if(++wait == 3) goto PRESS_B_AGAIN;
                  }
              }
        }
#endif

        btn_cnt = 0;
        while(!(PINC & _BV(BTN)))
        {
            // wait button is released
            btn_cnt++;
            _delay_ms(100);
        }

        _delay_ms(300);
        
        pind = 1;
        path[0] = '/';

        // if directory selected
        if( disp_files[disp_index].fattrib & AM_DIR )
        {
            if(!memcmp(disp_files[disp_index].fname,"..",2))
            {
                if(dir_level > 0) dir_level--;
            }
            else
            if(dir_level < MAX_DIR_LEVEL)
                memcpy(&dirs[dir_level++],disp_files[disp_index].fname,13);
            
            if(dir_level == 0) pf_opendir(&dir,"/"); else
            {
                for(uint8_t i = 0; i < dir_level; i++)
                {
                    memcpy(&path[pind],&dirs[i],strlen(dirs[i]));
                    pind += strlen(dirs[i]);
                    path[pind++]='/';
                }
                path[--pind] = 0;
                pf_opendir(&dir,path);
            }
            memset(disp_files,0,sizeof(fnfo)*2);
            PCINT1_disable();
            goto DIRECTORY_LIST;
        }

        PCINT1_disable();

        /// /END SELECT TRD IMAGE ------------------------------------------------------------------------------


        /////////////////////////////////////////////////////////////////
        // MOUNT TRD IMAGE and init Track Cluster table
        // disp_files[disp_index].fname contain short name (8.3) of selected TRD image
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // --------------------------------------------------------------------------------------------------------------------------------
        
        for(uint8_t i = 0; i < dir_level; i++)
        {
            memcpy(&path[pind],&dirs[i],strlen(dirs[i]));
            pind += strlen(dirs[i]);
            path[pind++]='/';
        }
        memcpy(path + pind, disp_files[disp_index].fname,strlen(disp_files[disp_index].fname));
        path[pind + strlen(disp_files[disp_index].fname)]=0;

OPEN_FILE:        

//        uint8_t SCL = 0;

//        if(path[strlen(path)] == 'L' || path[strlen(path)] == 'l') SCL = 1;

        if(pf_open(path) != FR_OK) { // if unable to open file, usually if SD card is removed
            if(eeprom_file == 1) {
                DESELECT();
                eeprom_write_byte((uint8_t*)4, 0);
            }
            goto MOUNT;
        }

        if(eeprom_file == 0 && btn_cnt > 20) ///////////////////////////////////////////////////
        {
            DESELECT();
            eeprom_write_block((const void*)&serial,(void*)0, 4);
            eeprom_write_block((const void*)path, (void*)4, strlen(path)+1);
        }
        
        LCD_clear();
        LCD_print_char(0);
        LCD_print_char(32);
        if(eeprom_file == 0)
            LCD_print(disp_files[disp_index].fname);
        else
        {
            uint8_t ptr = strlen(path);
            while(path[ptr] != '/') ptr--;
            LCD_print((char*)&path[ptr+1]);
        }

        LCD_print(0,1, F("CYL: 00  HEAD: 0") );

        max_cylinder = fat.fsize / 8192 + ((fat.fsize % 8192) ? 1 : 0); // calculate maximal cylinder
        if( max_cylinder > MAX_CYL ) max_cylinder = MAX_CYL; // if TRD image size too big set limit to MAX_CYL

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




        /// Emulator loop --------------
        uint8_t s_cylinder = 255;
        cylinder = 0;
        cylinder_changed = 0;

        _delay_ms(1500);

        while (1)
        { /// DRIVE SELECT LOOP

            LCD_light_off();

            while ( ( PIND & (_BV(MOTOR_ON) | _BV(DRIVE_SEL)) ) != 0 )  // wait drive select && motor_on
            {
                if(!(PINC & _BV(BTN)))
                { // if button pressed                    
                    while(!(PINC & _BV(BTN))); // wait button is released

                    DESELECT();
                    
                    LCD_light_on();
                    
                    if(eeprom_file == 1)
                    { // if filename from eeprom, reset eeprom data
                        eeprom_write_byte((uint8_t*)4, 0);
                        goto MOUNT;
                    }
                    goto FILE_LIST;
                }
            }

            
            /// DEVICE ENABLED =========================================================================================================================|

            if(cylinder == 0) DDRD |= _BV(TRK00);
            PCINT2_enable(); // ENABLE INDERRUPT (STEP pin)

            DDRD |= _BV(WP); // set WRITE PROTECT

            //check SD Card is present and same card as was mounted
            //if(serial != card_read_serial()) break; // exit from loop if card is not present or another card.

            uint8_t read_error = 0;            

            do { // READ DATA LOOP (send data from FDD to FDD controller)
            //=================================================================================================================================]
                uint8_t tmpc;

                for(volatile uint8_t sector = 0; sector < 16; sector++)
                { // transmit each sector of the track                    

                    if( sector == 0 ) // initialize track data for next round
                    {
                        prev_byte = 0;
                        for(volatile uint16_t tmpcn = 0; tmpcn < 1000; tmpcn++) tmpc++; // wait for cylinder change detect

                        if(s_cylinder != cylinder)
                        { // if cylinder is changed we need to calculate sector table for current cylinder (for fast sectors read)
                            while( cylinder_changed )
                            { // wait while cylinder changing
                                ATOMIC_BLOCK(ATOMIC_FORCEON)cylinder_changed = 0;
                                for(volatile uint16_t tmpcn = 0; tmpcn < 10000; tmpcn++) tmpc++;
                            }
                          
                            s_cylinder = cylinder;

                            // FAST create cluster table for 32 cylinder sectors (sector_table) --------------
                            cur_fat = clust_table[s_cylinder];
                            cur_fat_sector = cur_fat / 64;
                            sector_table[0] = sector_table[1] = cur_fat; // sector 1 offset on SD card
                            // process cluster chain
                            if(card_readp(sector_data, fat.fatbase + cur_fat_sector/2, (cur_fat_sector%2)*256, 256) != RES_OK) { read_error = 1; break; }
                            for(uint8_t i = 1; i < 16; i++) // TRD sector 256 bytes, SD card sector 512 bytes, so 32 TRD sectors = 16 sectors on SD
                            {
                                if((i % fat.csize == 0))
                                { // if cluster is changed
                                    if( (cur_fat / 64) != cur_fat_sector )
                                    {
                                        cur_fat_sector = cur_fat / 64;
                                        card_readp(sector_data, fat.fatbase + cur_fat_sector/2, (cur_fat_sector%2)*256, 256); // read data_block with current cluster number
                                    }
                                    cur_fat = (uint32_t)(*(uint32_t*)(sector_data + (uint8_t)((uint8_t)cur_fat << 2)));
                                }
                                sector_table[i*2] = sector_table[i*2+1] = cur_fat;  // 2 TRD sectors in same cluster
                            }
                            // --------------------------------------------------------------------------------
                        }
                    }

                    uint8_t side = ((~SIDE_PIN) & _BV(SIDE_SEL)) >> SIDE_SEL; // side detect

/////////////////////
//                    if(SCL && cylinder == 0 && side == 0) ; //////////////////////
/////////////////////

                    // READ SECTOR DATA from SD card
                    // fat.database - start cluster of FAT32 filesystem
                    // fat.csize    - cluster size
                    if(card_readp(sector_data,fat.database + (sector_table[side*16 + sector] - 2) * fat.csize + ((s_cylinder*2 + side)*8 + sector/2) % fat.csize,(sector%2)*256,256) != RES_OK) { read_error = 1; break; }

                    // Calculate CRC for sector header --------------------------------------
                    CRC_H.val = 0xB230;  // initial polynom value for sector header
                    CRC_H.val = (CRC_H.bytes.low << 8) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ s_cylinder));
                    CRC_H.val = (CRC_H.bytes.low << 8) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ side));
                    CRC_H.val = (CRC_H.bytes.low << 8) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ sector+1));
                    CRC_H.val = (CRC_H.bytes.low << 8) ^ pgm_read_word_near(Crc16Table + (CRC_H.bytes.high ^ 1));
                    // ----------------------------------------------------------------------

                    // Calculate CRC for current sector data --------------------------------
                    CRC_D.val = 0xE295; // initial polynom value for data
                    for(uint8_t i = 0; ;)
                    {
                        CRC_D.val = (CRC_D.bytes.low << 8) ^ pgm_read_word_near(Crc16Table + (CRC_D.bytes.high ^ sector_data[i]));
                        if(i++ == 255) break;
                    }
                    // ----------------------------------------------------------------------

                    if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // Stop sending track if cylinder is changed or FDD is disabled

                    if(!LCD_check_light()) LCD_light_on(); // Enable LCD Light if FDD is active

                    // NOW WE'RE READY TO SEND SECTOR ===============================================================================>

                    if(!sector)
                    { // if sector = 0
                        // print CYLINDER, HEAD INFO or track number on LCD
                        LCD_print(5,1,cylinder / 10);
                        LCD_print(cylinder % 10);
                        LCD_print(15,1,side);
                        
                        // Send TRACK GAP4A --------------------------------------------------
                        USART_enable();
                        for(uint8_t cnt = 0; cnt < 10; cnt++) send_byte(0x4E);
                        DDRB |= _BV(INDEX); // SET INDEX LOW if sector = 0 (Start Index pulse at start of the track)
                        // -------------------------------------------------------------------
                    }

                    // Send sector Address Field + start data field --------------------------
                    for(uint8_t cnt = 0; cnt < 60; cnt++)
                    {
                        switch(cnt)
                        {
                            case 0: sector_byte = 0; break;
                            case 12: sector_byte = 0xA1; A1_mark = 1; break;
                            case 15: sector_byte = 0xFE; A1_mark = 0; break;
                            case 16: sector_byte = s_cylinder; break;
                            case 17: sector_byte = side; break;
                            case 18: sector_byte = sector + 1; break;
                            case 19: sector_byte = 1; break;
                            case 20: sector_byte = CRC_H.bytes.high; break;
                            case 21: sector_byte = CRC_H.bytes.low; break;
                            case 22: sector_byte = 0x4E; break;
                            case 44: sector_byte = 0x00; break;
                            // data field header
                            case 56: sector_byte = 0xA1; A1_mark = 1; break;
                            case 59: sector_byte = 0xFB; A1_mark = 0; break;
                        }
                        send_byte(sector_byte);
                    }
                    // ----------------------------------------------------------------------

                    if(!sector) DDRB &= ~_BV(INDEX); // SET INDEX HIGH if sector = 0 (End Index pulse at start of the track)

                    if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // Stop sending track if cylinder is changed or FDD is not active

                    // Send sector data (256 bytes) -----------------------------------------
                    for(uint8_t cnt = 0; ;)
                    {
                        send_byte(sector_data[cnt]);
                        if(cnt++ == 255) break;
                    }
                    // ----------------------------------------------------------------------

                    // Send sector data CRC -------------------------------------------------
                    send_byte(CRC_D.bytes.high);
                    send_byte(CRC_D.bytes.low);
                    // ----------------------------------------------------------------------

                    // Send sector GAP ------------------------------------------------------
                    for(uint8_t cnt = 0; cnt < 54; cnt++) send_byte(0x4E);
                    // ----------------------------------------------------------------------

                    if(cylinder_changed || (PIND & _BV(MOTOR_ON))) break; // Stop sending track if cylinder is changed or FDD is disabled

                    // END SEND SECTOR ==============================================================================================>
                }
                
                if(read_error) break;

            } while(  (PIND & ( _BV(MOTOR_ON) | _BV(DRIVE_SEL) )) == 0 ); // READ DATA SEND LOOP END
            //=================================================================================================================================]
            USART_disable(); // DISABLE USART INDERRUPT after sending track
            PCINT2_disable(); // DISABLE PCINT INDERRUPT (STEP pin)

            LCD_light_off(); // Disable LCD Light if FDD is not active

            DDRB &= ~_BV(INDEX); // SET INDEX HIGH
            DDRD &= ~(_BV(WP) | _BV(TRK00)); // Set WP,TRK00 as input

            DESELECT(); // disconnect SD Card

            if(read_error) break;

            /// DEVICE DISABLED ========================================================================================================================|
            
        } /// DRIVE SELECT LOOP END

    } // MAIN LOOP END

} // END MAIN

