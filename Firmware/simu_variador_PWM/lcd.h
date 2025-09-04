// lcd.h
#ifndef LCD_H
#define LCD_H

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI

#include <SPI.h>

#endif

#define CLK 18
#define MOSI 23
#define CS 5
#define RESET 255 // ningun pin

extern U8G2_ST7920_128X64_F_SW_SPI u8g2; //  DECLARACIÓN visible para otros archivos


void u8g2_MainMenu(uint8_t v, uint8_t vh, uint8_t vl,uint8_t a, uint8_t d, const char* mode);
void u8g2_ControlMenu(int select);
void u8g2_ProgramMenu(int);
void u8g2_prepare(void);
void u8g2_ParamChange(uint8_t value, const char* valueName, const char* unit);


#endif