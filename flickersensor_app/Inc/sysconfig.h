/**
 * @file         sysconfig.h
 * @author       Weyne Chen
 * @version      V01
 * @date         2017.05.17
 * @brief
 * @note
 * @attention    COPYRIGHT WEYNE
 **/

#ifndef __SYSCONFIG_H
#define __SYSCONFIG_H
#include "stm32f1xx_hal.h"

#define N 5

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern TIM_HandleTypeDef htim1;
extern uint8_t DataReady;
extern uint16_t Buffer0[N*10];

#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
