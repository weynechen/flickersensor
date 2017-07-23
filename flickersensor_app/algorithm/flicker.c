/**
 * @file         flicker.c
 * @author       Weyne Chen
 * @version      V01
 * @date         2017.05.17
 * @brief
 * @note
 * @attention    COPYRIGHT WEYNE
 **/

#include "flicker.h"
#include "sysconfig.h"

uint32_t max = 0, min = 0, avg = 0;

static void Sort(uint16_t *data, uint16_t len)
{
    uint16_t tmp, i, j;

    for (i = 0; i < len; i++)
    {
        for (j = 0; j < len - i - 1; j++)
        {
            if (data[j] > data[j + 1])
            {
                tmp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = tmp;
            }
        }
    }
}

float GetFlickerValue(uint16_t *data, uint16_t len)
{
    float flicker_value;
    uint16_t i = 0;
    uint16_t counter = 0;
	
		max = 0;
		min = 0;
		avg = 0;
	
    Sort(data, len);

    for (i = 1; i < len - 1; i++)
    {
        avg += data[i];
        counter++;
    }

    avg /= counter;
    if (avg < DC_LOW_LIMIT)
        return DC_ERROR;

    if(avg > DC_UP_LIMIT)
        return OVERFLOW_ERROR;

    counter = 0;
    for (i = 1; i < N; i++)
    {
        min += data[i];
        counter++;
    }

    min /= counter;

    counter = 0;
    for (i = len - 1 - N; i < len - 1; i++)
    {
        max += data[i];
        counter++;
    }

    max /= counter;

    if (max > OVERFLOW_LIMIT)
        return OVERFLOW_ERROR;

    if (min > max)
    {
        return 0;
    }

    flicker_value = ((float)max - (float)min) / (float)avg;

    return flicker_value;
}

/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
