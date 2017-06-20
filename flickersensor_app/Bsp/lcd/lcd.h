/**
* @file         lcd.h
* @author       Weyne Chen
* @version      V01
* @date         2017.05.17
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#ifndef __LCD_H
#define __LCD_H
#include "main.h"
#include "stm32f1xx_hal.h"
#include "sys.h"
#define LCD_XSIZE 128
#define LCD_YSIZE 128

#define SPI_SDA PDout(9)   /*< SPI SDO */
#define SPI_SCK PBout(14)   /*< SPI CLK */
#define SPI_CS PBout(12)    
#define SPI_RS PBout(15)    
typedef enum {
    SPI_COMMAND,
    SPI_DATA,
} SPIDataCommandTypeDef;

#define MCU 1
#define SPI 0

#define INTERFACE MCU

void LCD_WriteFull(uint16_t color);
void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p,uint8_t font);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_Init(void);
void LCD_Fill(uint16_t ybegin, uint16_t yend);

extern uint16_t FontColor;
extern uint16_t BackColor;
#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
