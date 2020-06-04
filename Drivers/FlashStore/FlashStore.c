/**
  ******************************************************************************
  * @file    FlashStore.c
  * @author  Margit R�bert
  * @date    2019.01.28
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "FlashStore.h"

/* Private functions ---------------------------------------------------------*/
static uint16_t CalcCrc16Ansi(uint16_t initValue, const void* data, size_t size);

/**
  * @brief  Flash Save Data
  * @param page Address of the page example: 0x0807F800
  * @param size max size = FLASH_PAGE_SIZE - 2
  * @retval status
  */
uint8_t FlashStoreWrite(uint32_t page, const void *data, size_t size)
{
  if(size - 2 > FLASH_PAGE_SIZE)
    return FLASH_STORE_SIZE_TOO_BIG_ERROR;

  /*--- destination page earase ---*/
  FLASH_EraseInitTypeDef erase;
  erase.NbPages = 1;
  erase.PageAddress = page;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  uint32_t erase_error_code;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&erase, &erase_error_code);
  if(erase_error_code != 0xFFFFFFFF)
    return FLASH_STORE_ERASE_ERROR;

  uint8_t *bytePtr = (uint8_t*)data;
  int numOfhalfWords = size / 2;
  int numOfBytes = size % 2;
  uint32_t srcIndex = 0;
  uint16_t dataU16;
  
  /*  []{0x01, 0x02, 0x03, 0x04} ==> 0x0102, 0x0304 ==> 0x3040102...*/ 
  /*--- Write half words ---*/
  for(int i = 0; i < numOfhalfWords ; i++)
  {
    dataU16 = bytePtr[srcIndex]<<8;
    dataU16 |= bytePtr[srcIndex + 1];
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page + srcIndex, dataU16) == HAL_OK)
      srcIndex += 2;
    else
      return FLASH_STORE_WRITE_ERROR;
  }
  /*--- Write 1 extra half word if we need ---*/
  if (numOfBytes!= 0)
  {
    dataU16 = bytePtr[srcIndex]<<8;
    dataU16 |= 0xAA;
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page + srcIndex, dataU16) == HAL_OK)
      srcIndex += 2;
    else
      return FLASH_STORE_WRITE_ERROR;
  }
  /*--- Calculate CRC-16-ANSI  ---*/
  uint16_t crc = CalcCrc16Ansi(0x00, data, size);
  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, page + srcIndex, crc ) == HAL_OK)
    srcIndex += 2;
  else
    return FLASH_STORE_WRITE_ERROR;

  HAL_FLASH_Lock();
  return FLASH_STORE_OK;
}

/**
  * @brief Flahs Read Data
  * @param page Address of the page example: 0x0807F800
  * @param data Area where will be read
  * @param size max size = FLASH_PAGE_SIZE - 2
  * @retval status
  */
uint8_t FlashStoreRead(uint32_t page, void *data, size_t size)
{
  if(size - 2 > FLASH_PAGE_SIZE)
    return FLASH_STORE_SIZE_TOO_BIG_ERROR;

  uint8_t *bytePtr = (uint8_t*)data;
  int numOfHalfWords = size / 2;
  int numOfBytes = size % 2;
  uint32_t srcIndex, destIndex = 0;
  uint16_t dataU16;
  /*--- Read half words ---*/
  for(srcIndex = 0; srcIndex < numOfHalfWords; srcIndex++)
  { 
    dataU16 = ((uint16_t*)page)[srcIndex];
    bytePtr[destIndex] = dataU16 >> 8;
    bytePtr[destIndex + 1] = dataU16;
    destIndex += 2;
  }
  /*--- Read bytes ---*/
  if(numOfBytes != 0)
  {
    dataU16 = ((uint16_t*)page)[srcIndex];
    bytePtr[destIndex] = dataU16 >> 8;
    destIndex += 2;
    srcIndex++;
  }
  /*--- CRC chceck ---*/
  uint16_t calcCrc = CalcCrc16Ansi(0x00, data, size);
  uint16_t readCrc = ((uint16_t*)page)[srcIndex];

  if(calcCrc != readCrc)
    return FLASH_STORE_CRC_ERROR;

return FLASH_STORE_OK;
}

/**
  * @brief  CRC-16-ANSI
  * @retval value
  * CRC-16-ANSI
  * Least significant bit first (little-endian)
  * x16 + x15 + x2 + 1 => 0x8005
  * 0x48,0x65,0x6C,0x6C,0x6F,0x20,0x57,0x6F,0x72,0x6C,0x64 -> CRC:0x70C3
  * uint8_t data[]= "Hello World";
  * uint16_t crc = CalcCrc16(0, data, sizeof(data) - 1);
  * if(crc == 0x70C3)
  *    printf("PASSED.");
  */
static uint16_t CalcCrc16Ansi(uint16_t initValue, const void* data, size_t size)
{
  uint16_t remainder = initValue;
  uint16_t polynomial = 0x8005;
  uint8_t *byte_ptr= (uint8_t*)data;
  
  for (size_t i = 0; i < size; ++i)
  {
    remainder ^= (byte_ptr[i] << 8);
    for (uint8_t bit = 8; bit > 0; --bit)
    {
      if (remainder & 0x8000)
        remainder = (remainder << 1) ^ polynomial;
      else
        remainder = (remainder << 1);
    }
  }
  return (remainder);
}
/************************ (C) COPYRIGHT Konvol�ci� **********END OF FILE****/
