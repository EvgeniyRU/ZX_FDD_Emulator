#ifndef LCD_MODULE_H
#define LCD_MODULE_H

#include "Config.h"


///                         En Rw Rs D0 D1 D2 D3 L
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// I2C Extender settings
#define LCDEX_ADDR  0x4E  // Extender address, 0x27 for standard chinese module
#define LCDEX_RS    0     // RS pin on extender
#define LCDEX_RW    1     // RW pin on extender
#define LCDEX_EN    2     // EN pin on extender
#define LCDEX_LIGHT 3     // LIGHT pin on extender
#define LCDEX_D4    4     // D4 pin on extender
#define LCDEX_D5    5     // D5 pin on extender
#define LCDEX_D6    6     // D6 pin on extender
#define LCDEX_D7    7     // D7 pin on extender

#define LCD_DELAY_ENABLE_PULSE 1      // enable signal pulse width in micro seconds

#define SCL_CLOCK  100000L

////////////////////////////////////////////////////////////////////////////////
// LCD Commands definition
 
// Clear Display -------------- 0b00000001
#define LCD_CLEAR_DISPLAY       0x01
 
// Cursor Home ---------------- 0b0000001x
#define LCD_CURSOR_HOME         0x02
 
// Set Entry Mode ------------- 0b000001xx
#define LCD_SET_ENTRY           0x04
 
#define LCD_ENTRY_DECREASE      0x00
#define LCD_ENTRY_INCREASE      0x02
#define LCD_ENTRY_NOSHIFT       0x00
#define LCD_ENTRY_SHIFT         0x01
 
// Set Display ---------------- 0b00001xxx
#define LCD_SET_DISPLAY         0x08
 
#define LCD_DISPLAY_OFF         0x00
#define LCD_DISPLAY_ON          0x04
#define LCD_CURSOR_OFF          0x00
#define LCD_CURSOR_ON           0x02
#define LCD_BLINKING_OFF        0x00
#define LCD_BLINKING_ON         0x01
 
// Set Shift ------------------ 0b0001xxxx
#define LCD_SET_SHIFT           0x10
 
#define LCD_CURSOR_MOVE         0x00
#define LCD_DISPLAY_SHIFT       0x08
#define LCD_SHIFT_LEFT          0x00
#define LCD_SHIFT_RIGHT         0x04
 
// Set Function --------------- 0b001xxxxx
#define LCD_SET_FUNCTION        0x20
 
#define LCD_FUNCTION_4BIT       0x00
#define LCD_FUNCTION_8BIT       0x10
#define LCD_FUNCTION_1LINE      0x00
#define LCD_FUNCTION_2LINE      0x08
#define LCD_FUNCTION_5X7        0x00
#define LCD_FUNCTION_5X10       0x04
 
#define LCD_SOFT_RESET          0x30
 
// Set CG RAM Address --------- 0b01xxxxxx  (Character Generator RAM)
#define LCD_SET_CGADR           0x40
 
#define LCD_GC_CHAR0            0
#define LCD_GC_CHAR1            1
#define LCD_GC_CHAR2            2
#define LCD_GC_CHAR3            3
#define LCD_GC_CHAR4            4
#define LCD_GC_CHAR5            5
#define LCD_GC_CHAR6            6
#define LCD_GC_CHAR7            7
 
// Set DD RAM Address --------- 0b1xxxxxxx  (Display Data RAM)
#define LCD_SET_DDADR           0x80

#define LCD_DDADR_LINE1         0x00
#define LCD_DDADR_LINE2         0x40
#define LCD_DDADR_LINE3         0x10
#define LCD_DDADR_LINE4         0x50


#define lcd_e_delay()   _delay_us(LCD_DELAY_ENABLE_PULSE)


void LCD_init();
void LCD_clear();
void LCD_print(const char* txt);
void LCD_print(uint8_t x, uint8_t y, const char* txt);
void LCD_print(uint8_t num);
void LCD_print(uint8_t x, uint8_t y, uint8_t num);


#endif /* LCD_MODULE_H */
