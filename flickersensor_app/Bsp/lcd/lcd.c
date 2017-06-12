/**
* @file         lcd.c
* @author       Weyne Chen
* @version      V01
* @date         2017.05.17
* @brief         
* @note          
* @attention    COPYRIGHT WEYNE
**/

#include "lcd.h"

typedef struct
{
	uint16_t begin;
	uint16_t end;
} CoordinateTypeDef;

/********************************************************************************************************
Driver
*********************************************************************************************************/
static GPIO_TypeDef *const DataPort[] = {GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOC};
const static uint16_t DataPin[] = {D0_Pin, D1_Pin, D2_Pin, D3_Pin, D4_Pin, D5_Pin, D6_Pin, D7_Pin};

#define DELAY(x)        \
	\
do               \
	{                   \
		uint16_t i = x; \
		while (i--)     \
			;           \
	\
}                \
	\
while(0)

#define WR_TO_LCD                                                \
	do                                                           \
	{                                                            \
		HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_RESET); \
		DELAY(72);                                               \
		HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_SET);   \
	\
} while (0)

static void ParseData(uint8_t data)
{
	uint8_t shift = 0x01;
	uint8_t index = 0;

	while (shift != 0x00)
	{
		if ((data & shift) == shift)
		{
			HAL_GPIO_WritePin(DataPort[index], DataPin[index], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(DataPort[index], DataPin[index], GPIO_PIN_RESET);
		}

		shift = shift << 1;
		index++;
	}
}

static void WriteCmd(uint8_t cmd)
{

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	ParseData(cmd);
	WR_TO_LCD;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

static void WriteData(uint8_t data)
{
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	ParseData(data);
	WR_TO_LCD;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

static void SetAddress(CoordinateTypeDef x, CoordinateTypeDef y)
{
	WriteCmd(0x2A);
	WriteData(x.begin >> 8);
	WriteData(x.begin);
	WriteData(x.end >> 8);
	WriteData(x.end);

	WriteCmd(0x2B);
	WriteData(y.begin >> 8);
	WriteData(y.begin);
	WriteData(y.end >> 8);
	WriteData(y.end);

	WriteCmd(0x2c);
}

/**********************************************************************************************
API
***********************************************************************************************/
void LCDInit(void)
{
	HAL_GPIO_WritePin(S_P_GPIO_Port, S_P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI_LANE_SEL_GPIO_Port, SPI_LANE_SEL_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(120);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	WriteCmd(0x11); //Sleep out
	HAL_Delay(120); //delay 120ms
	WriteCmd(0xB1);
	WriteData(0x01);
	WriteData(0x08);
	WriteData(0x05);
	WriteCmd(0xB2);
	WriteData(0x01);
	WriteData(0x08);
	WriteData(0x05);
	WriteCmd(0xB3);
	WriteData(0x01);
	WriteData(0x08);
	WriteData(0x05);
	WriteData(0x05);
	WriteData(0x08);
	WriteData(0x05);
	WriteCmd(0xB4); //Dot inversion
	WriteData(0x00);
	/*
//column address set(2AH)
		WriteCmd(0x2A);     
		WriteData(0X00);   
		WriteData(0X00);   
		WriteData(0X00); 
		WriteData(0X7f); 
//Row address set(2BH)
		WriteCmd(0x2B);     
		WriteData(0X00);   
		WriteData(0X20);   
		WriteData(0X00); 
		WriteData(0Xa0); 
		*/
	WriteCmd(0x2A);
	WriteData(0X00);
	WriteData(0X02);
	WriteData(0X00);
	WriteData(0X81);
	//Row address set(2BH)
	WriteCmd(0x2B);
	WriteData(0X00);
	WriteData(0X21);
	WriteData(0X00);
	WriteData(0Xa1);

	WriteCmd(0xC0);
	WriteData(0x28);
	WriteData(0x08);
	WriteData(0x04);
	WriteCmd(0xC1);
	WriteData(0xC0);

	WriteCmd(0xC2);
	WriteData(0x0D);
	WriteData(0x00);

	WriteCmd(0xC3);
	WriteData(0x8D);
	WriteData(0x2A);

	WriteCmd(0xC4);
	WriteData(0x8D);
	WriteData(0xEE);

	WriteCmd(0xC5); //VCOM
	WriteData(0x12);

	WriteCmd(0x36); //MX, MY, RGB mode
	WriteData(0x08);

	WriteCmd(0xE0);
	WriteData(0x03);
	WriteData(0x1B);
	WriteData(0x12);
	WriteData(0x11);
	WriteData(0x3F);
	WriteData(0x3A);
	WriteData(0x32);
	WriteData(0x34);
	WriteData(0x2F);
	WriteData(0x2B);
	WriteData(0x30);
	WriteData(0x3A);
	WriteData(0x00);
	WriteData(0x01);
	WriteData(0x02);
	WriteData(0x05);
	WriteCmd(0xE1);
	WriteData(0x03);
	WriteData(0x1B);
	WriteData(0x12);
	WriteData(0x11);
	WriteData(0x32);
	WriteData(0x2F);
	WriteData(0x2A);
	WriteData(0x2F);
	WriteData(0x2E);
	WriteData(0x2C);
	WriteData(0x35);
	WriteData(0x3F);
	WriteData(0x00);
	WriteData(0x00);
	WriteData(0x01);
	WriteData(0x05);

	WriteCmd(0x3A); //65k mode
	WriteData(0x05);
	WriteCmd(0x29); //Display on
}

void LCDWriteFull(uint16_t color)
{
	CoordinateTypeDef x = {0x0, LCD_XSIZE};
	CoordinateTypeDef y = {0x0, LCD_YSIZE};
	uint16_t i = 0;
	uint16_t j = 0;
	
	SetAddress(x, y);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	for (; i < LCD_YSIZE; i++)
	{
		for (; j < LCD_XSIZE; j++)
		{
			ParseData(color >> 8);
			WR_TO_LCD;
			ParseData(color);
			WR_TO_LCD;
		}
	}
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
