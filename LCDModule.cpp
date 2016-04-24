#include <Wire.h>
//// This module use NewliquidCrystal library from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
//// Later it'll be rewritten without using libraries
#include <LiquidCrystal_I2C.h>
#include "LCDModule.h"

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

void LCD_init()
{
  Wire.begin();
  lcd.begin(16,2);               // initialize the lcd
  lcd.clear();  
  lcd.home ();                   // go home
}

void LCD_clear()
{
  lcd.clear();
}

void LCD_print(const char* txt)
{
  lcd.print(txt);
}

void LCD_print(int num)
{
  lcd.print(num);
}

void LCD_print(uint8_t x, uint8_t y, const char* txt)
{
  lcd.setCursor(x,y);
  lcd.print(txt);
}

void LCD_print(uint8_t x, uint8_t y, int num)
{
  lcd.setCursor(x,y);
  lcd.print(num);
}

