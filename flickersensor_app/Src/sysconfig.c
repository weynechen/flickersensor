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
#include "flash_if.h"

#define KEY_STORE_ADDRESS ADDR_FLASH_PAGE_65
#define APP_END_ADDRESS   ADDR_FLASH_PAGE_64
const uint32_t __attribute__((at(KEY_STORE_ADDRESS))) KeyStore = 0x5A2F5D82;


uint16_t Buffer0[SAMPLE_SIZE];
uint8_t RXTemp[BUFFER_SIZE];
TaskIDTypeDef TaskID = TASK_NULL;
uint16_t DataLen = 0;
uint8_t DataTemp[BUFFER_SIZE];
uint16_t VCOM = 0;
uint16_t ID = 0;

static uint32_t CalSecurityCode(void)
{
  uint32_t i;

  typedef struct
  {
    uint32_t id1;
    uint32_t id2;
    uint32_t id3;
  } UIDTypeDef;

  UIDTypeDef id = *(volatile UIDTypeDef *)(0x1FFF0000 + 0xF7E8);
  uint32_t result = 0;

  for (i = 0x08000000; i < APP_END_ADDRESS; i += 4)
  {
    result ^= *((volatile uint32_t *)i);
  }

  result ^= 0x12345678;
  result ^= id.id1;
  result ^= id.id2;
  result ^= id.id3;

  return result;
}

bool CheckSecurity(void)
{
  uint32_t key_store = (uint32_t) * (volatile uint32_t *)(KEY_STORE_ADDRESS);
  uint32_t key_cal = CalSecurityCode();

  if (key_store == 0x5A2F5D82)
  {
    FLASH_If_Erase(KEY_STORE_ADDRESS, 1);
    FLASH_If_Write(KEY_STORE_ADDRESS, &key_cal, 1);
  }
  else
  {
    if (key_cal == key_store)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  return true;
}

/********************* (C) COPYRIGHT WEYNE CHEN *******END OF FILE ********/
