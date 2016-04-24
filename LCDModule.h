#include "PinDefs.h"


void LCD_init();
void LCD_clear();
void LCD_print(const char* txt);
void LCD_print(uint8_t x, uint8_t y, const char* txt);
void LCD_print(int num);
void LCD_print(uint8_t x, uint8_t y, int num);
