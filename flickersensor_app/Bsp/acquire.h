/**
 * @file         acquire.h
 * @author       Weyne Chen
 * @version      V01
 * @date         2017.05.17
 * @brief
 * @note
 * @attention    COPYRIGHT WEYNE
 **/

#ifndef __ACQUIRE_H
#define __ACQUIRE_H
#include "stm32f1xx_hal.h"

void SelChannel(uint8_t channel);
void AcquireStart(void);

#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
