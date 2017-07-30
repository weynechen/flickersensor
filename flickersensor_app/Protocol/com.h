/**
* @file         com.h
* @author       Weyne Chen
* @version      V01
* @date         2017.06.24
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#ifndef __COM_H
#define __COM_H
#include "sysconfig.h"
__packed typedef struct
{
    uint8_t DeviceID;
    uint8_t PackageID;
    uint16_t DataLength;
    uint16_t Data;
    uint8_t Crc8;
} PackageTypeDef;

typedef enum {
    TP = 0,
    FLICKER_SENSOR,
    U_DISK
} DeviceIDTypeDef;

typedef enum {
    FLICKER_VALUE,

} FlickerSensorTypeDef;

void SendFlicker(uint16_t flicker);
void ParsePack(void);
void UART1_RestartDMA(void);
#endif
/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
