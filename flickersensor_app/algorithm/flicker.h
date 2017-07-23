/**
* @file         flicker.h
* @author       Weyne Chen
* @version      V01
* @date         2017.05.17
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#ifndef __FLICKER_H
#define __FLICKER_H
#include "stm32f1xx_hal.h"


float GetFlickerValue(uint16_t *data , uint16_t len);

#define OVERFLOW_LIMIT 2600
#define DC_LOW_LIMIT 1000
#define DC_UP_LIMIT 2400

typedef enum
{
    DC_ERROR = -1,
    OVERFLOW_ERROR = -2,
}FlickerErrorTypeDef;

#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
