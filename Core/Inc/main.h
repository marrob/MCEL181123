/* USER CODE BEGIN Header */
/*
 * main.h
 *
 *  Created on: 2020. márc. 6.
 *      Author: Margit Robert
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "Led.h"
#include "cannet.h"
#include "cannetdb.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct _Calibration_Type
{
  char DateTime[14];
  float SET01;
  float SET02;
  float SET03;
  float SET04;
  float SET05;
  float SET06;
  float SET07;
  float SET08;
  float SET09;
  float SET10;
  float SET11;
  float SET12;


}Calibration_Type;

typedef struct _CanBusSpeedType
{
  uint32_t Baud;
  uint8_t Pre;
  uint32_t BS1;
  uint32_t BS2;
  uint32_t SJW;
}CanBusSpeedTypeDef;

typedef enum _CtrlStates_Type
{
  SDEV_START,                   //0
  SDEV_IDLE,                    //1
  SDEV_DBG1,                    //2
  SDEV_DBG2,                    //3
  SDEV_MEAS_START,              //4
  SDEV_MEAS_BUSY,
  SDEV_MEAS_CPLT,               //6

}CtrlStates_Type;

typedef enum _CalibState_Type
{
  SCALIB_MAKE_HARDFAULT,      //
  SCALIB_HARD_RESET,          //
  SDBG_LAST                   //
}DeviceCalibState_Type;

typedef struct
{
  uint8_t RxErrCnt;
  uint8_t TxErrCnt;
  uint16_t RxTestCnt;
}CanHandle_Type;

typedef  struct _PowerCtrl_Type
{
  uint8_t Enabled;
  float CVprog;
  float CCprog;
  float CVmeas;
  float CCmeas;
  float CCoffset;

  float FallDown;
  uint8_t Range;
  uint8_t CC;
  uint8_t CV;
  uint8_t OE;
  uint8_t GD;
  uint8_t RSense;
  uint8_t Trigger;
  struct
  {
    float UpSp; /*V/s*/
    float DownSp; /*V/s*/
    int32_t Timestamp ;
    uint8_t IsCplt;
    uint32_t Iterations;
    uint32_t TotalIter;
    float Step;
    float OutVolt;
  }Speed;

  struct
  {
    CtrlStates_Type Next;
    CtrlStates_Type Curr;
    CtrlStates_Type Pre;
  }State;

}PowerCtrl_Type;

typedef enum _DebugState_Type
{
  SDBG_IDLE,                       /*!< 00 Itt várakozik                         */
  SDBG_MAKE_HARDFAULT,             /*!< 01 Durva hibát csinál a WDT teszteléséhez*/
  SDBG_HARD_RESET,                 /*!< 02 Eszköz újraindítása                   */
}DeviceDebugState_Type;


typedef struct _AppTypeDef
{
 // CAN_HandleTypeDef Handle;
  PowerCtrl_Type PowerCtrl;
  CanNet_Type CanNet;
  ADC_HandleTypeDef Adc;
  UART_HandleTypeDef Uart;
  CanHandle_Type Can;
  DeviceDebugState_Type    DebugState;
  uint8_t SwitchesState;
  uint64_t Version;
  uint8_t ThisNode;
  uint64_t UpTimeSec;
  float ControllerTemp;
  float TransistorTemp;
  uint32_t  ServiceCode;
  Calibration_Type Calib;

  struct _status
  {
    uint32_t MainCycleTime;
  }Status;

}DeviceTypeDef;

extern I2C_HandleTypeDef hi2c2;
//extern SPI_HandleTypeDef hspi2;
extern LedHandle_Type    hLed;
extern DeviceTypeDef     Device;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define HMI_LED_GREEN 0
#define HMI_LED_RED 1

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#ifdef DEBUG
  #define DEVICE_DEBUG_LEVEL    3
#else
  #define DEVICE_DEBUG_LEVEL    2
#endif

#if (DEVICE_DEBUG_LEVEL > 0)
#define  DeviceUsrLog(...)  {printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define DeviceUsrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 1)

#define  DeviceErrLog(...)  {printf(VT100_ATTR_RED);\
                             printf("ERROR.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf(VT100_ATTR_RESET);\
                             printf("\r\n");}
#else
#define DeviceErrLog(...)
#endif

#if (DEVICE_DEBUG_LEVEL > 2)
#define  DeviceDbgLog(...)  {printf(VT100_ATTR_YELLOW);\
                             printf("DEBUG.DEVICE:") ;\
                             printf(__VA_ARGS__);\
                             printf(VT100_ATTR_RESET);\
                             printf("\r\n");}
#else
#define DeviceDbgLog(...)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void ConsoleWrite(char *str);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PS_SYNC_Pin GPIO_PIN_1
#define PS_SYNC_GPIO_Port GPIOA
#define PS_OE_Pin GPIO_PIN_2
#define PS_OE_GPIO_Port GPIOA
#define LIVE_LED_Pin GPIO_PIN_3
#define LIVE_LED_GPIO_Port GPIOA
#define RA3_Pin GPIO_PIN_4
#define RA3_GPIO_Port GPIOA
#define RA2_Pin GPIO_PIN_5
#define RA2_GPIO_Port GPIOA
#define RA1_Pin GPIO_PIN_6
#define RA1_GPIO_Port GPIOA
#define RA0_Pin GPIO_PIN_7
#define RA0_GPIO_Port GPIOA
#define MA2_Pin GPIO_PIN_4
#define MA2_GPIO_Port GPIOC
#define MA0_Pin GPIO_PIN_5
#define MA0_GPIO_Port GPIOC
#define MA1_Pin GPIO_PIN_0
#define MA1_GPIO_Port GPIOB
#define MA3_Pin GPIO_PIN_1
#define MA3_GPIO_Port GPIOB
#define PS2ON_Pin GPIO_PIN_2
#define PS2ON_GPIO_Port GPIOB
#define ADC_CS_Pin GPIO_PIN_12
#define ADC_CS_GPIO_Port GPIOB
#define CPRG_EN_Pin GPIO_PIN_6
#define CPRG_EN_GPIO_Port GPIOC
#define CV_MD_Pin GPIO_PIN_7
#define CV_MD_GPIO_Port GPIOC
#define OV_Pin GPIO_PIN_8
#define OV_GPIO_Port GPIOC
#define VPRG_LD_Pin GPIO_PIN_9
#define VPRG_LD_GPIO_Port GPIOC
#define VPRG_EN_Pin GPIO_PIN_8
#define VPRG_EN_GPIO_Port GPIOA
#define CC_MD_Pin GPIO_PIN_11
#define CC_MD_GPIO_Port GPIOC
#define CPRG_LD_Pin GPIO_PIN_12
#define CPRG_LD_GPIO_Port GPIOC
#define ADS_LD_Pin GPIO_PIN_2
#define ADS_LD_GPIO_Port GPIOD
#define LED_G_Pin GPIO_PIN_4
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_5
#define LED_R_GPIO_Port GPIOB
#define PS1ON_Pin GPIO_PIN_7
#define PS1ON_GPIO_Port GPIOB
#define TP9_Pin GPIO_PIN_8
#define TP9_GPIO_Port GPIOB
#define TP10_Pin GPIO_PIN_9
#define TP10_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */



/* Generic  -------------------------------------------------------------------*/

#define DEVICE_NAME             "MCEL181123"
#define DEVICE_FW               "190930_1217"
#define DEVICE_FW_HEX           0x1909301217
#define DEVICE_PCB              "00"
#define DEVICE_MNF              "KONVOLUCIO"

#define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
#define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
#define DEVICE_FW_SIZE          sizeof(DEVICE_FW)
#define DEVICE_SN_SIZE          sizeof(DEVICE_SN)
#define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)

#define DEVICE_OK           0
#define DEVICE_FAIL         1

#define DEVICE_FAIL_LED     0
#define DEVICE_STR_SIZE     80

#define FAIL_LED_MEM_LOAD   1
#define FAIL_LED_MEM_TEST   2
#define FAIL_LED_RLY_DRV    3
#define FAIL_LED_MEM_WRITE  4


#define FLASH_STORE_CALIB_PAGE_ADDR         0x0807F800
#define BSP_VREF 5.0000

#define BSP_RY1_MASK     0x01
#define BSP_RY2_MASK     0x02
#define BSP_RY4RY5_MASK  0x04
#define BSP_GUARD        0x08
#define BSP_U11B_MASK    0x10
#define BSP_U11D_MASK    0x20
#define BSP_RY6_MASK     0x40
#define BSP_RY3_MASK     0x80

#define MEAS_MODE_CV         0
#define MEAS_MODE_CC         1
#define MEAS_MODE_CC_OFFSET  2
#define MEAS_MODE_CC_ST_R79  3
#define MEAS_MODE_CC_ST_R80  4

#define MCEL_OE_TYPE_OFF  0
#define MCEL_OE_TYPE_ON   1
#define MCEL_OE_TYPE_UP   2
#define MCEL_OE_TYPE_DOWN 3


/* VT100 ---------------------------------------------------------------------*/
/*
 * https://www.csie.ntu.edu.tw/~r92094/c++/VT100.html
 * http://www.termsys.demon.co.uk/vtansi.htm
 */
#define VT100_CLEARSCREEN         "\033[2J"
#define VT100_CURSORHOME          "\033[H"
#define VT100_ATTR_RESET          "\033[0m"
#define VT100_ATTR_RED            "\033[31m"
#define VT100_ATTR_GREEN          "\033[32m"
#define VT100_ATTR_YELLOW         "\033[33m"
#define VT100_CUP(__v__,__h__)    ("\033["__v__";"__h__"H") /*Cursor Position*/



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
