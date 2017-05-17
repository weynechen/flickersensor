/**
 * @file         acquire.c
 * @author       Weyne Chen
 * @version      V01
 * @date         2017.05.17
 * @brief
 * @note
 * @attention    COPYRIGHT WEYNE
 **/

#include "acquire.h"
#include "sysconfig.h"
#include "main.h"

uint8_t DataReady = 0;


void AcquireStart(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Buffer0, N * 10);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    DataReady = 1;
}


void SelChannel(uint8_t channel)
{
    uint8_t a = channel & 0x01;
    uint8_t b = channel & 0x02;
    uint8_t c = channel & 0x04;

    if (a == 0x01)
    {
        HAL_GPIO_WritePin(GPIOB, A_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, A_Pin, GPIO_PIN_RESET);
    }

    if (b == 0x02)
    {
        HAL_GPIO_WritePin(GPIOB, B_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, B_Pin, GPIO_PIN_RESET);
    }


    if (c == 0x04)
    {
        HAL_GPIO_WritePin(GPIOB, C_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, C_Pin, GPIO_PIN_RESET);
    }
}


/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
