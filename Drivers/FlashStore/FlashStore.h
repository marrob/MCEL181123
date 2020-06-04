/**
  ******************************************************************************
  * @file    FlashStore.h
  * @author  Margit R�bert
  * @date    2019.01.28
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported macro ------------------------------------------------------------*/
#define FLASH_STORE_OK                    0
#define FLASH_STORE_ERASE_ERROR           1
#define FLASH_STORE_WRITE_ERROR           2
#define FLASH_STORE_CRC_ERROR             3
#define FLASH_STORE_SIZE_TOO_BIG_ERROR    4

/* Exported functions --------------------------------------------------------*/
uint8_t FlashStoreWrite(uint32_t page, const void *data, size_t size);
uint8_t FlashStoreRead(uint32_t page, void *data, size_t size);

/************************ (C) COPYRIGHT Konvol�ci� **********END OF FILE****/
