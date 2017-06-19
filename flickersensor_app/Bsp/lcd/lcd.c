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
#include "font.h"
uint16_t FontColor = 0xffff;
uint16_t BackColor = 0x00;

/********************************************************************************************************
Driver
*********************************************************************************************************/
#if (INTERFACE == MCU)
static GPIO_TypeDef *const DataPort[] = {D0_GPIO_Port, D1_GPIO_Port, D2_GPIO_Port, D3_GPIO_Port, D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port};
const static uint16_t DataPin[] = {D0_Pin, D1_Pin, D2_Pin, D3_Pin, D4_Pin, D5_Pin, D6_Pin, D7_Pin};
#endif

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

#define WR_RISING_EDGE                                           \
	do                                                           \
	{                                                            \
		HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_RESET); \
		DELAY(7);                                               \
		HAL_GPIO_WritePin(WR_GPIO_Port, WR_Pin, GPIO_PIN_SET);   \
		DELAY(7);                                               \
	\
} while (0)

#if (INTERFACE == MCU)

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
	DELAY(72);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
	DELAY(72);
	ParseData(cmd);
	WR_RISING_EDGE;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

static void WriteData(uint8_t data)
{
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	DELAY(72);
	HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
	DELAY(72);
	ParseData(data);
	WR_RISING_EDGE;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}
#else
static void SPIWrite8Bit(uint8_t data, SPIDataCommandTypeDef rs)
{
	uint8_t i = 0;
	SPI_CS = 0;
	SPI_RS = rs;
	SPI_SCK = 0;
	DELAY(7);

	for (i = 0; i < 8; i++)
	{
		SPI_SDA = data >> (7 - i);

		DELAY(7);
		SPI_SCK = 1;
		DELAY(7);
		SPI_SCK = 0;
		DELAY(7);
	}

	SPI_CS = 1;
}

static void WriteCmd(uint8_t cmd)
{
	SPIWrite8Bit(cmd, SPI_COMMAND);
}

static void WriteData(uint8_t data)
{
	SPIWrite8Bit(data, SPI_DATA);
}

static void WriteColor(uint16_t color)
{
	unsigned char k;
	SPI_SDA = 0; //SDA=0
	SPI_SCK = 0; //SCL=0
	SPI_RS = 1;  //A0=1
	SPI_CS = 0;  //CS=0
	for (k = 0; k < 16; k++)
	{
		SPI_SCK = 0; //SCL=0
		if (color & 0x8000)
			SPI_SDA = 1; //SDA=1
		else
			SPI_SDA = 0; //SDA=0
		color = color << 1;
		DELAY(7);
		SPI_SCK = 1; //SCL=1
		DELAY(7);
	}
	SPI_CS = 1; //CS=1
}
#endif
static void SetBeginAddress(uint16_t x, uint16_t y)
{
	WriteCmd(0x2A);
	WriteData(x >> 8);
	WriteData(x);
	WriteData(LCD_XSIZE >> 8);
	WriteData(LCD_XSIZE);

	WriteCmd(0x2B);
	WriteData(y >> 8);
	WriteData(y);
	WriteData(LCD_YSIZE >> 8);
	WriteData(LCD_YSIZE);

	WriteCmd(0x2c);
}

/**********************************************************************************************
API
***********************************************************************************************/
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
	SetBeginAddress(x, y);
	WriteData(color);
}

void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size)
{
	uint8_t temp;
	uint8_t pos, t;
	num = num - ' ';

	for (pos = 0; pos < size; pos++)
	{
		if (size == 12)
			temp = asc2_1206[num][pos];
		else
			temp = asc2_1608[num][pos];
		for (t = 0; t < size / 2; t++)
		{
			if (temp & 0x01)
				LCD_DrawPoint(x, y, FontColor);
			else
				LCD_DrawPoint(x, y, BackColor);

			temp >>= 1;
			x++;
		}
		y++;
	}
}

void LCD_ShowString(uint16_t x, uint16_t y, const uint8_t *p)
{
	while (*p != '\0')
	{
		LCD_ShowChar(x, y, *p, 16);
		x += 8;
		p++;
	}
}

void LCD_WriteFull(uint16_t color)
{
	uint16_t i = 0;
	uint16_t j = 0;

	SetBeginAddress(0, 0);

	for (i = 0; i < LCD_XSIZE; i++)
	{
		for (j = 0; j < LCD_YSIZE; j++)
		{
#if (INTERFACE == SPI)
			WriteColor(color);
#else
			WriteData(color >> 8);
			WriteData(color);
#endif
		}
	}
}

void LCD_Init(void)
{
#if (INTERFACE == SPI)
	HAL_GPIO_WritePin(S_P_GPIO_Port, S_P_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI_LANE_SEL_GPIO_Port, SPI_LANE_SEL_Pin, GPIO_PIN_SET);
#else
	HAL_GPIO_WritePin(S_P_GPIO_Port, S_P_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI_LANE_SEL_GPIO_Port, SPI_LANE_SEL_Pin, GPIO_PIN_RESET);
#endif

	HAL_GPIO_WritePin(RD_GPIO_Port, RD_Pin, GPIO_PIN_SET);
	DELAY(120);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	DELAY(10);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
	DELAY(120);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	DELAY(100);
	WriteCmd(0x11); //Sleep out
	DELAY(120); //DELAY 120ms
	//------------------------------------ST7735S Frame rate-----------------------------------------//
	WriteCmd(0xB1); //Frame rate 80Hz
	WriteData(0x02);
	WriteData(0x35);
	WriteData(0x36);

	WriteCmd(0xB2); //Frame rate 80Hz
	WriteData(0x02);
	WriteData(0x35);
	WriteData(0x36);

	WriteCmd(0xB3); //Frame rate 80Hz
	WriteData(0x02);
	WriteData(0x35);
	WriteData(0x36);
	WriteData(0x02);
	WriteData(0x35);
	WriteData(0x36);
	//------------------------------------End ST7735S Frame rate-----------------------------------------//
	WriteCmd(0xB4);  //Dot inversion
	WriteData(0x03); //ST7735S

	//------------------------------------ST7735S Power Sequence-----------------------------------------//
	WriteCmd(0xC0);
	WriteData(0xA2);
	WriteData(0x02);
	WriteData(0x84);
	WriteCmd(0xC1);
	WriteData(0xC5);
	WriteCmd(0xC2);
	WriteData(0x0D);
	WriteData(0x00);
	WriteCmd(0xC3);
	WriteData(0x8D);
	WriteData(0x2A);
	WriteCmd(0xC4);
	WriteData(0x8D);
	WriteData(0xEE);
	//---------------------------------End ST7735S Power Sequence-------------------------------------//
	WriteCmd(0xC5); //VCOM
	WriteData(0x03);

	WriteCmd(0x36); //MX, MY, RGB mode
	WriteData(0x08);
	//------------------------------------ST7735S Gamma Sequence-----------------------------------------//
	WriteCmd(0xE0);
	WriteData(0x12);
	WriteData(0x1C);
	WriteData(0x10);
	WriteData(0x18);
	WriteData(0x33);
	WriteData(0x2C);
	WriteData(0x25);
	WriteData(0x28);
	WriteData(0x28);
	WriteData(0x27);
	WriteData(0x2F);
	WriteData(0x3C);
	WriteData(0x00);
	WriteData(0x03);
	WriteData(0x03);
	WriteData(0x10);

	WriteCmd(0xE1);
	WriteData(0x12);
	WriteData(0x1C);
	WriteData(0x10);
	WriteData(0x18);
	WriteData(0x2D);
	WriteData(0x28);
	WriteData(0x23);
	WriteData(0x28);
	WriteData(0x28);
	WriteData(0x26);
	WriteData(0x2F);
	WriteData(0x3B);
	WriteData(0x00);
	WriteData(0x03);
	WriteData(0x03);
	WriteData(0x10);
	//----------------End ST7735S Gamma Sequence-------------//
	WriteCmd(0x3A); //65k mode
	WriteData(0x05);

	WriteCmd(0x2A); //Set Column Address
	WriteData(0x00);
	WriteData(0x00 + 2);
	WriteData(0x00);
	WriteData(0x7F + 2);

	WriteCmd(0x2B); //Set Page Address
	WriteData(0x00);
	WriteData(0x00);
	WriteData(0x00);
	WriteData(0x7F);

	WriteCmd(0x29); //Display on
	DELAY(20);
	WriteCmd(0x2C);
	DELAY(50);
}

/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
