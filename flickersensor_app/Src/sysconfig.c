/**
* @file         sysconfig.c
* @author       Weyne Chen
* @version      V01
* @date         2017.05.17
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#include "sysconfig.h"


uint16_t Buffer0[N*10];
uint8_t RXTemp[BUFFER_SIZE];
TaskIDTypeDef TaskID = TASK_NULL;
uint16_t DataLen = 0;
uint8_t DataTemp[BUFFER_SIZE];
uint16_t VCOM = 0;
uint16_t ID = 0;

/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
