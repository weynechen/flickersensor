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
#define LCD_XSIZE 128
#define LCD_YSIZE 128

void LCDWriteFull(uint16_t color);
void LCDInit(void);
#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
