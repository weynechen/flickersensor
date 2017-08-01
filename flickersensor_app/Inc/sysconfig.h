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
#define BUFFER_SIZE 128

typedef enum
{
    FLICKER_VALUE,
    VCOM_VALUE,
    ID_VALUE,
	TASK_NULL = 0xff,
}TaskIDTypeDef;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;
extern uint8_t DataReady;
extern uint16_t Buffer0[N*10];
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t RXTemp[BUFFER_SIZE];
extern uint8_t DataTemp[BUFFER_SIZE];
extern TaskIDTypeDef TaskID;
extern uint16_t DataLen;
extern uint16_t VCOM;
extern uint16_t ID;


#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
