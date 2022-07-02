/* USER CODE BEGIN Header */
/*
 * main.c
 *
 *  Created on: 2020. márc. 6.
 *      Author: Margit Robert
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "LiveLed.h"
#include "memory.h"
#include "FlashStore.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DeviceTypeDef Device;
LiveLED_HnadleTypeDef hLiveLed;
LedHandle_Type        hLed;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void LiveLedOff(void);
void LiveLedOn(void);
void LedGreenOn(void);
void LedGreenOff(void);
void LedRedOn(void);
void LedRedOff(void);

void DebugTask(void);
uint8_t GetAddress(void);

void SwitchesSet(uint8_t mask);
void SwitchesClear(uint8_t mask);
uint8_t SelectorGet(void);
void SwitchesUpdate(uint8_t value);
uint32_t AdcGetValue(void);
double AdcGetVoltGen(void);
double AdcGetVolt(void);
uint32_t VDacSetVolt(float volt);
void VDacSet(uint16_t data);
uint32_t CDacSetVolt(float volt);
void CDacSet(uint16_t data);

void Ps1On(void);
void Ps1Off(void);
void Ps2On(void);
void Ps2Off(void);

float Mcp9800Meas(I2C_HandleTypeDef *hi2c);
void Mcp9800Config (I2C_HandleTypeDef *hi2c);

float AdCCmeasControllerTemp(ADC_HandleTypeDef *hadc);

void CanDiagTask(CanHandle_Type *context);

uint8_t CanNetLowLevelTxMailboxIsEmpty(void);

LedItem_Type LedList[] = {
  { HMI_LED_GREEN,    &LedGreenOn,     &LedGreenOff,    },
  { HMI_LED_RED,      &LedRedOn,       &LedRedOff,      },
};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TIME_RESOLTUIN_MS 5

void CalibrationOff(void)
{
    Device.Calib.SET01 = 0;
    Device.Calib.SET02 = 0;
    Device.Calib.SET03 = 0;
    Device.Calib.SET04 = 0;
    Device.Calib.SET05 = 0;
    Device.Calib.SET06 = 0;
    Device.Calib.SET07 = 0;
    Device.Calib.SET08 = 0;
    Device.Calib.SET09 = 1;
    Device.Calib.SET10 = 1;
    Device.Calib.SET11 = 1;
    Device.Calib.SET12 = 1;
}

void CalibrationDefault(void)
{
    Device.Calib.SET01 = 0;        /*ReadCvOffsetErr_SET0001*/
    Device.Calib.SET02 = 0;        /*ProgCvOffsetErr_SET0002*/
    Device.Calib.SET03 = 0;        /*ReadCvGainErr_SET0003*/
    Device.Calib.SET04 = 0;        /*ProgCvGainErr_SET0004*/
    Device.Calib.SET05 = 0.008;    /*ReadCcOffsetErrRng0_SET0005*/
    Device.Calib.SET06 = 0.002;    /*ReadCcOffsetErrRng1_SET0006*/
    Device.Calib.SET07 = 0;        /*ProgCcOffsetErrRng0_SET0007*/
    Device.Calib.SET08 = 0;        /*ProgCcOffsetErrRng1_SET0008*/
    Device.Calib.SET09 = 0.9933;   /*ReadCcGainErrRng0_SET0009*/
    Device.Calib.SET10 = 1;        /*ProgCctGainErrRng0_SET0010*/
    Device.Calib.SET11 = 1.0030;   /*ReadCcGainErrRng1_SET0011*/
    Device.Calib.SET12 = 1;        /*ProgCcGainErrRng1_SET0012*/
}

void CalibrationSave(void)
{
  if (FlashStoreWrite(FLASH_STORE_CALIB_PAGE_ADDR, &Device.Calib, sizeof(Device.Calib)) != FLASH_STORE_OK)
  {
    DeviceDbgLog("CalibrationSave: Success");
  }
  else
  {
    DeviceDbgLog("CalibrationSave: Fail");
  }
}

void UpdateStatus(void)
{
  CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_RANGE,  &Device.PowerCtrl.Range); /*0x03*/
  CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_TEMP_TR,&Device.TransistorTemp);  /*0x05*/
  CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_TEMP_UC,&Device.ControllerTemp);  /*0x06*/
  CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_STA_CC, &Device.PowerCtrl.CC);    /*0x07*/
  CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_STA_CV, &Device.PowerCtrl.CV);    /*0x08*/
  CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_STA_OE, &Device.PowerCtrl.OE);    /*0x09*/
  CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_VER,    &Device.Version);         /*0xFE*/
}

/* Events  -------------------------------------------------------------------*/
void RxTestMsgIncomeEvent(void)
{
  Device.Can.RxTestCnt++;
}

void ChangedEventCVprog(uint8_t node)
{
  uint32_t code = VDacSetVolt(Device.PowerCtrl.CVprog);
  DeviceDbgLog("ChangedEventCVprog: %f -> 0x%05lX", Device.PowerCtrl.CVprog, code);
}

void ChangedEventCCprog(uint8_t node)
{
  if(Device.PowerCtrl.Range == 0)
  {/*mivel a 100mA-es méréshatárnál a shunt 5OHM és igy 5V => 100mA*/
    /*** 0 ... 99.999***/
    Device.PowerCtrl.CCprog /= 20;
  }
  else if(Device.PowerCtrl.Range == 1)
  { /*** 0 ... 49.999 ***/
    Device.PowerCtrl.CCprog /= 10;
  }
  uint32_t code = CDacSetVolt(Device.PowerCtrl.CCprog);
  DeviceDbgLog("ChangedEventCCprog: %f -> 0x%05lX", Device.PowerCtrl.CCprog, code);
}

void ChangedEventCrng(uint8_t node)
{
  if(Device.PowerCtrl.Range == 0)
  {
    SwitchesClear(BSP_RY2_MASK);
    SwitchesSet(BSP_RY1_MASK);
  }
  else if(Device.PowerCtrl.Range == 1)
  {
    SwitchesClear(BSP_RY1_MASK);
    SwitchesSet(BSP_RY2_MASK);
  }
  DeviceDbgLog("ChangedEventCrng: %d", Device.PowerCtrl.Range);
}

void ChangedEventRSense(uint8_t node)
{
  if(Device.PowerCtrl.RSense)
  {
    SwitchesSet(BSP_RY4RY5_MASK);
    DeviceDbgLog("ChangedEventRSense: ON");
  }
  else
  {
    SwitchesClear(BSP_RY4RY5_MASK);
    DeviceDbgLog("ChangedEventRSense: OFF");
  }
}

void ChangedEventOE(uint8_t node)
{
  if(Device.PowerCtrl.OE == MCEL_OE_TYPE_ON)
  {
    /*** previoius sates set back */
    ChangedEventRSense(0);
    ChangedEventCVprog(0);
    /*** 12V ON ***/
    Ps2On();
    DeviceDbgLog("ChangedEventOE: ON");
  }
  else if (Device.PowerCtrl.OE == MCEL_OE_TYPE_OFF)
  {
    VDacSetVolt(0);
    /*** 12V OFF ***/
    Ps2Off();
    /*** SNESE OFF ***/
    SwitchesClear(BSP_RY4RY5_MASK);
    DeviceDbgLog("ChangedEventOE: OFF");
  }
  else if (Device.PowerCtrl.OE == MCEL_OE_TYPE_UP)
  {
    ChangedEventRSense(0);
    /*** 12V ON ***/
    Ps2On();
    PowerCtrl_Type *ctrl = &Device.PowerCtrl;
    ctrl->Speed.Iterations = 0;
    ctrl->Speed.IsCplt = 0;
    ctrl->Speed.Timestamp = HAL_GetTick();
    ctrl->Speed.Step =  (ctrl->Speed.UpSp/1000) * TIME_RESOLTUIN_MS;
    ctrl->Speed.TotalIter = ctrl->CVprog / ctrl->Speed.Step;
    ctrl->Speed.OutVolt = 0;
    DeviceDbgLog("ChangedEventOE: RiseUp");
  }
  else if (Device.PowerCtrl.OE == MCEL_OE_TYPE_DOWN)
  {
    ChangedEventRSense(0);
    /*** 12V ON ***/
    Ps2On();
    PowerCtrl_Type *ctrl = &Device.PowerCtrl;
    ctrl->Speed.Iterations = 0;
    ctrl->Speed.IsCplt = 0;
    ctrl->Speed.Timestamp = HAL_GetTick();
    ctrl->Speed.Step =  (ctrl->Speed.DownSp/1000) * TIME_RESOLTUIN_MS;
    ctrl->Speed.TotalIter = ctrl->CVprog / ctrl->Speed.Step;
    ctrl->Speed.OutVolt = ctrl->CVprog;
    DeviceDbgLog("ChangedEventOE: RiseDown");
  }
}

void ChangedEventGD(uint8_t node)
{
  if (Device.PowerCtrl.GD == 0)
  {
    SwitchesSet(BSP_GUARD);
    DeviceDbgLog("ChangedEventGD: OFF");
  }
  else
  {
    SwitchesClear(BSP_GUARD);
    DeviceDbgLog("ChangedEventGD: ON");
  }
}

void ChangedEventTrigger(uint8_t node)
{
  if(Device.PowerCtrl.Trigger == MEAS_MODE_CV)
  {/*** Trigger meas output volt***/
      SwitchesClear(BSP_U11B_MASK);
      SwitchesSet(BSP_U11D_MASK);
  }
  else if (Device.PowerCtrl.Trigger == MEAS_MODE_CC)
  {/*** Trigger meas output current ***/
      SwitchesClear(BSP_U11D_MASK);
      SwitchesSet(BSP_U11B_MASK);
  }
  else if(Device.PowerCtrl.Trigger == MEAS_MODE_CC_OFFSET)
  {
    /*** Trigger meas output current offset ***/
      SwitchesClear(BSP_U11D_MASK | BSP_RY1_MASK | BSP_RY2_MASK);
      SwitchesSet(BSP_U11B_MASK);
  }
  else if(Device.PowerCtrl.Trigger == MEAS_MODE_CC_ST_R79)
  {   /*** Trigger measure current on  MEAS_MODE_CC_ST_R79 ***/
      SwitchesClear(BSP_U11D_MASK |
                       BSP_RY1_MASK |
                       BSP_RY4RY5_MASK |
                       BSP_RY3_MASK);
      SwitchesSet(  BSP_U11B_MASK |
                       BSP_RY2_MASK |
                       BSP_RY6_MASK);
  }
  else if(Device.PowerCtrl.Trigger == MEAS_MODE_CC_ST_R80)
  {  /*** Trigger measure current on  MEAS_MODE_CC_ST_R80 ***/
      SwitchesClear(BSP_U11D_MASK |
                       BSP_RY2_MASK |
                       BSP_RY4RY5_MASK|
                       BSP_RY6_MASK);
      SwitchesSet(  BSP_U11B_MASK |
                       BSP_RY1_MASK |
                       BSP_RY3_MASK);
  }

  DeviceDbgLog("ChangedEventTrigger: %d", Device.PowerCtrl.Trigger);
  if( Device.PowerCtrl.State.Curr == SDEV_IDLE)
  {
    Device.PowerCtrl.State.Next = SDEV_MEAS_START;
  }
  else
  {
    DeviceDbgLog("Trigger NotValid.");
  }
}

void ChangedEventServices(uint8_t node)
{
  DeviceDbgLog("ChangedEventServices: %ld", Device.ServiceCode);
  switch(Device.ServiceCode)
  {
    case SRV_RESTART:
    {
      NVIC_SystemReset();
      DeviceDbgLog("ChangedEventServices: NVIC_SystemReset");
      break;
    }
    case SRV_UPDATE_STATUS:
    {
      UpdateStatus();
      break;
    }
    case SRV_CALIB_SAVE:
    {
      CalibrationSave();
      break;
    }
    case SRV_CALIB_OFF:
    {
      CalibrationOff();
      break;
    }
    case SRV_CALIB_DEFAULT:
    {
      CalibrationDefault();
      break;
    }
  }
}

/* Power Control -------------------------------------------------------------*/
void PowerControlRiseUpTask(PowerCtrl_Type *ctrl)
{
  if((ctrl->OE == MCEL_OE_TYPE_UP || ctrl->OE == MCEL_OE_TYPE_DOWN) && !ctrl->Speed.IsCplt)
  {
    if(HAL_GetTick() - ctrl->Speed.Timestamp > TIME_RESOLTUIN_MS)
    {
      ctrl->Speed.Timestamp = HAL_GetTick();
      if(ctrl->Speed.Iterations > ctrl->Speed.TotalIter - 1)
      {
        ctrl->Speed.IsCplt = 1;
      }
      else
      {
        if(ctrl->OE == MCEL_OE_TYPE_UP)
        {
          ctrl->Speed.OutVolt += ctrl->Speed.Step;
          ctrl->Speed.Iterations++;
          VDacSetVolt(ctrl->Speed.OutVolt);
        }
        else if(ctrl->OE == MCEL_OE_TYPE_DOWN)
        {
          ctrl->Speed.OutVolt -= ctrl->Speed.Step;
          ctrl->Speed.Iterations++;
          VDacSetVolt(ctrl->Speed.OutVolt);
        }
      }
    }
  }
}

void PowerControlTask(PowerCtrl_Type *ctrl)
{
  static int32_t timestamp = 0;
  static double adcvalue = 0;
  char str[10];
  switch (ctrl->State.Curr)
  {
    case SDEV_START:
    {
      if(HAL_GetTick() - timestamp > 1000)
      {
        Ps1On(); /*+15V, -15V, 5.5V ON*/
        ctrl->CVprog = 0.0;
        if(HAL_GetTick() - timestamp > 2000)
        {
          ctrl->State.Next = SDEV_IDLE;
          DeviceDbgLog("PowerControlTask: SDEV_IDLE -> SDEV_OUTPUT_OFF");
        }
      }
      break;
    }

    case SDEV_DBG1:
    {
      SwitchesSet(BSP_RY1_MASK); /*** RANGE 5OHM ***/
      VDacSetVolt(2.5);
      CDacSetVolt(2.5);
      Ps2On(); /*** 12V ON ***/
      ctrl->State.Next = SDEV_IDLE;
      DeviceDbgLog("PowerControlTask: SDEV_DBG1 -> SDEV_IDLE");
      break;
    }
    case SDEV_DBG2:
    {
      Device.PowerCtrl.CVprog = 2.5;
      ChangedEventCVprog(0);

      Device.PowerCtrl.Range = 0;
      ChangedEventCrng(0);

      Device.PowerCtrl.CCprog = 99;
      ChangedEventCCprog(0);

      Device.PowerCtrl.OE = 1;
      ChangedEventOE(0);

      Device.PowerCtrl.Trigger = MEAS_MODE_CC_OFFSET;
      ChangedEventTrigger(0);
      DeviceDbgLog("PowerControlTask: SDEV_DBG2 -> SDEV_IDLE");
      break;
    }
    case SDEV_IDLE:
    {
      if(ctrl->State.Pre != ctrl->State.Curr)
      {
        DeviceDbgLog("PowerControlTask: SDEV_IDLE");
        LedGreenOn();
        LedRedOff();
      }
      break;
    }
    case SDEV_MEAS_START:
    {
      if(ctrl->State.Pre != ctrl->State.Curr)
      {
        timestamp = HAL_GetTick();
        LedRedOn();
        LedGreenOff();
      }
      if(HAL_GetTick() - timestamp > 20)
      {
        adcvalue = AdcGetVoltGen();
        ctrl->State.Next = SDEV_MEAS_BUSY;
        timestamp = HAL_GetTick();
        DeviceDbgLog("PowerControlTask: SDEV_MEAS_START -> SDEV_MEAS_BUSY");
      }
      break;
    }
    case SDEV_MEAS_BUSY:
    {
      if(HAL_GetTick() - timestamp > 250)
      {
        adcvalue = AdcGetVoltGen();
        if( adcvalue != -2)
        {
           ctrl->State.Next = SDEV_MEAS_CPLT;
           DeviceDbgLog("PowerControlTask: SDEV_MEAS_BUSY -> SDEV_MEAS_CPLT");
        }
      }
      break;
    }

    case SDEV_MEAS_CPLT:
    {
      if(ctrl->Trigger == MEAS_MODE_CV)
      {
        /*** Fromating 0 ... 4.9999 -> 4 3/4 ***/
        sprintf(str, "%0.5f", adcvalue);
        sscanf(str, "%6f", &ctrl->CVmeas);
        CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_V_MEAS, &ctrl->CVmeas);
      }
      else if (ctrl->Trigger == MEAS_MODE_CC || ctrl->Trigger == MEAS_MODE_CC_ST_R79 || ctrl->Trigger == MEAS_MODE_CC_ST_R80 )
      {
          double ccmeas = adcvalue;
          if(ctrl->Range == 0 )
          { /*** 0.000 .. 99.999mA ***/
            ccmeas *= 20;
            ccmeas -= Device.Calib.SET05;
            ccmeas *= Device.Calib.SET09;
          }
          else if(ctrl->Range == 1)
          {/*** 0.000 .. 49.999uA ***/
            ccmeas *= 10;
            ccmeas -= Device.Calib.SET06;
            ccmeas *= Device.Calib.SET11;
          }
          /*** Formatting  0 ... XX.XXX ***/
          sprintf(str, "%06.3f", ccmeas);
          sscanf(str, "%6f", &ctrl->CCmeas);
          CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_C_MEAS, &ctrl->CCmeas);
       }
       else if(ctrl->Trigger == MEAS_MODE_CC_OFFSET)
       {
          ctrl->CCoffset = adcvalue;
          CanNetSendSignal(&Device.CanNet, Device.ThisNode, 0, MSG_MCEL_OFFSET,  &ctrl->CCoffset);
       }
      ctrl->State.Next = SDEV_IDLE;
      DeviceDbgLog("PowerControlTask: SDEV_MEAS_CPLT -> SDEV_IDLE");
      break;
    }
  }
  ctrl->State.Pre = ctrl->State.Curr;
  ctrl->State.Curr = ctrl->State.Next;
}

/* Common Tasks --------------------------------------------------------------------*/
void UpTimeIncrementTask(void)
{
  static int32_t timestamp = 0;
//  char string[80];

  if((HAL_GetTick() - timestamp) >= 1000)
  {
    timestamp = HAL_GetTick();
    Device.UpTimeSec++;

//    SwitchesSet(BSP_RY1_MASK); /*** RANGE 5OHM ***/
//    VDacSetVolt(2.5);
//    CDacSetVolt(2.5);
//
//    uint32_t temp = AdcGetValue();
//    sprintf(string, "0x%lX\r\n", temp);
//    ConsoleWrite(string);
//
//    sprintf(string, "%f\r\n", AdcGetVolt());
//    ConsoleWrite(string);
  }
}

void TempsMeasTask(void)
{
  static int32_t timestamp = 0;

  if((HAL_GetTick() - timestamp) >= 1000)
  {
    timestamp = HAL_GetTick();
    Device.ControllerTemp = AdCCmeasControllerTemp(&Device.Adc);
    Device.TransistorTemp = Mcp9800Meas(&hi2c2);
  }
}

float AdCCmeasControllerTemp(ADC_HandleTypeDef *hadc)
{
  uint32_t lsbs = HAL_ADC_GetValue(hadc);
  return (lsbs * 0.0141);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  printf(VT100_CLEARSCREEN);
  printf(VT100_CURSORHOME);
  printf(VT100_ATTR_RESET);

#ifdef DEBUG
  printf(VT100_ATTR_RED);
    DeviceUsrLog("This is a DEBUG version.");
  printf(VT100_ATTR_RESET);
#endif

  DeviceUsrLog("Manufacturer:%s, Name:%s, Version:%s", DEVICE_MNF, DEVICE_NAME, DEVICE_FW);
  /*** Leds ***/
  hLed.pLedTable = LedList;
  hLed.Records = sizeof(LedList)/sizeof(LedItem_Type);
  LedInit(&hLed);
  LedOff(&hLed, DEVICE_FAIL_LED);

  CanNetInit(&Device.CanNet, NODE_MCEL, Device.ThisNode, Messages, CanNetMessagesCount());

  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);

  Device.PowerCtrl.OE = MCEL_OE_TYPE_ON;

  CalibrationOff();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t timestamp = HAL_GetTick();

    LiveLedTask(&hLiveLed);
    LedTask(&hLed);

    CanNetTask(&Device.CanNet);
    UpTimeIncrementTask();
    CanDiagTask(&Device.Can);
    TempsMeasTask();
  //Device.TransistorTemp = 3.51;
  //Device.ControllerTemp = 3.51;
    PowerControlTask(&Device.PowerCtrl);
    PowerControlRiseUpTask(&Device.PowerCtrl);

    Device.Status.MainCycleTime = HAL_GetTick() - timestamp;


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef  sFilterConfig;

  /* Configure the CAN Filter */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  sFilterConfig.FilterIdHigh = 0x00;
  sFilterConfig.FilterIdLow = 0x00;

  sFilterConfig.FilterMaskIdHigh = 0x00;
  sFilterConfig.FilterMaskIdLow = 0x00;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    DeviceErrLog("HAL_CAN_ConfigFilter");


  if (HAL_CAN_Start(&hcan) != HAL_OK)
    DeviceErrLog("HAL_CAN_Start");

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PS2ON_Pin|LED_G_Pin|LED_R_Pin|PS1ON_Pin 
                          |TP9_Pin|TP10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CPRG_EN_Pin|VPRG_LD_Pin|CPRG_LD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CV_MD_GPIO_Port, CV_MD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VPRG_EN_GPIO_Port, VPRG_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADS_LD_GPIO_Port, ADS_LD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PS_SYNC_Pin PS_OE_Pin */
  GPIO_InitStruct.Pin = PS_SYNC_Pin|PS_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LIVE_LED_Pin */
  GPIO_InitStruct.Pin = LIVE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIVE_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RA3_Pin RA2_Pin RA1_Pin RA0_Pin */
  GPIO_InitStruct.Pin = RA3_Pin|RA2_Pin|RA1_Pin|RA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MA2_Pin MA0_Pin */
  GPIO_InitStruct.Pin = MA2_Pin|MA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MA1_Pin MA3_Pin */
  GPIO_InitStruct.Pin = MA1_Pin|MA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PS2ON_Pin LED_G_Pin LED_R_Pin PS1ON_Pin 
                           TP9_Pin TP10_Pin */
  GPIO_InitStruct.Pin = PS2ON_Pin|LED_G_Pin|LED_R_Pin|PS1ON_Pin 
                          |TP9_Pin|TP10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_CS_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CPRG_EN_Pin VPRG_LD_Pin CPRG_LD_Pin */
  GPIO_InitStruct.Pin = CPRG_EN_Pin|VPRG_LD_Pin|CPRG_LD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CV_MD_Pin */
  GPIO_InitStruct.Pin = CV_MD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CV_MD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OV_Pin CC_MD_Pin */
  GPIO_InitStruct.Pin = OV_Pin|CC_MD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VPRG_EN_Pin */
  GPIO_InitStruct.Pin = VPRG_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(VPRG_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADS_LD_Pin */
  GPIO_InitStruct.Pin = ADS_LD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADS_LD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* Can  ---------------------------------------------------------------------*/
/**
  * @brief  Rx Fifo 0 message pending callback
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t data[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK)
  {
    DeviceErrLog("HAL_CAN_RxCpltCallback.HAL_CAN_Receive_IT");
  }

//  StringPlusDataToHexaString(data, StringBuffer, 8);
//  DeviceDbgLog("0x%08X %s",rxHeader.ExtId, StringBuffer);

  if(Device.PowerCtrl.State.Curr == SDEV_IDLE)
  {
    if(GetNodeAddress(rxHeader.ExtId) == Device.ThisNode)
      CanNetReadMsgLowLevel(&Device.CanNet,rxHeader.ExtId, data, rxHeader.DLC);
    else if(IsBroadcasMsg(rxHeader.ExtId))
      CanNetReadMsgLowLevel(&Device.CanNet,rxHeader.ExtId, data, rxHeader.DLC);
  }
}

uint8_t CanNetLowLevelTxMailboxIsEmpty(void)
{
  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) != 0)
    return CANNET_EMPTY;
  else
    return CANNET_FULL;
}

uint8_t CanNetWriteMsgLowLevel(uint32_t arbId, uint8_t *data, size_t size)
{
  CAN_TxHeaderTypeDef txHeader;
  txHeader.StdId = 0x000;
  txHeader.ExtId = arbId;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.IDE = CAN_ID_EXT;
  txHeader.DLC = size;
  txHeader.TransmitGlobalTime = DISABLE;
  uint32_t txMailbox;

  if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) != HAL_OK)
    return DEVICE_FAIL;

return CANNET_OK;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  DeviceErrLog("HAL_CAN_ErrorCallback ErrorCode: 0x%02lX" ,hcan->ErrorCode);
}

void CanDiagTask(CanHandle_Type *context)
{
  static int32_t timestamp;
  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    context->RxErrCnt = (hcan.Instance->ESR & CAN_ESR_REC) >> 24;
    context->TxErrCnt = (hcan.Instance->ESR & CAN_ESR_TEC) >> 16;
  }
}




void SwitchesSet(uint8_t mask)
{
  Device.SwitchesState |= mask;
  SwitchesUpdate(Device.SwitchesState);
}

void SwitchesClear(uint8_t mask)
{
  Device.SwitchesState &= ~mask;
  SwitchesUpdate(Device.SwitchesState);
}

uint8_t SelectorGet(void)
{
  return Device.SwitchesState;
}





double AdcGetVoltGen(void)
{
  static double meas = 0;
  uint32_t adcval = AdcGetValue();
  if((adcval & 0x80000000) != 0x80000000)
  {/*** End Of Conversion ***/

    if((adcval & 0xF0000000) == 0x20000000)
    {/*** 0 < Vin < VREF ***/
      /*** Status bits not need ***/
      adcval &=0x0FFFFFFF;
      double nanovolt = adcval * 18.626; //(nV)
      meas = nanovolt * 1E-9;
    }
    else if((adcval & 0xF0000000) == 0x30000000)
    {
      /*** Vin > VREF ***/
      meas = 5.0;
    }
    else if ((adcval & 0xF0000000) == 0x10000000)
    {
      /*** Vin < 0 ***/
      meas = -0;
    }
  }
  else
  {
    /*** Conversion In progress ***/
     meas = -2;
  }
  return meas;
}

double AdcGetVolt(void)
{
  static double CVmeas = 0;
  uint32_t adcval = AdcGetValue();
  DelayMs(300); /*** 160msec még kevés ***/
  adcval = AdcGetValue();
  if((adcval & 0x80000000) != 0x80000000)
  {/*** End Of Conversion ***/

    if((adcval & 0xF0000000) == 0x20000000)
    {/*** 0 < Vin < VREF ***/
      /*** Status bits not need ***/
      adcval &=0x0FFFFFFF;
      double nanovolt = adcval * 18.626; //(nV)
      CVmeas = nanovolt * 1E-9;
    }
    else if((adcval & 0xF0000000) == 0x30000000)
    {
      /*** Vin > VREF ***/
      CVmeas = 5.0;
    }
    else if ((adcval & 0xF0000000) == 0x10000000)
    {
      /*** Vin < 0 ***/
      CVmeas = -0;
    }
  }
  else
  {
    /*** Conversion In progress ***/
     CVmeas = -2;
  }

  return CVmeas;
}


uint32_t VDacSetVolt(float volt)
{
  uint32_t code = volt /(BSP_VREF/65536);
  if(volt < 0)
    code = 0;
  if(volt> 4.99999)
    code = 0xFFFF;
  VDacSet(code);
return code;
}

uint32_t CDacSetVolt(float volt)
{
  uint32_t code = volt /(BSP_VREF/65536);
  if(volt < 0)
    code = 0;
  if(volt> 4.99999)
    code = 0xFFFF;
  CDacSet(code);
return code;
}

void SwitchesUpdate(uint8_t data)
{
  uint8_t dummy[sizeof(data)];
  memset(dummy,0x00,sizeof(data));
  HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&data, dummy, sizeof(data), 100);
  HAL_GPIO_WritePin(ADS_LD_GPIO_Port, ADS_LD_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ADS_LD_GPIO_Port, ADS_LD_Pin, GPIO_PIN_RESET);
}

uint32_t AdcGetValue(void)
{
  uint32_t retval = 0;
  uint8_t i,j;
  uint8_t dummy[sizeof(retval)];
  uint8_t temp[sizeof(retval)], reverse[sizeof(retval)];
  memset(dummy,0x00,sizeof(retval));

  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, dummy, temp, sizeof(retval), 100);
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  j = sizeof(temp)-1;
  for(i=0; i  <sizeof(temp); i++)
    reverse[j--] = *(temp + i);

  return *((uint32_t*)&reverse);
}

void VDacSet(uint16_t data)
{

/*
 * VREF x (1/65,536) = 0xFFFF
 * VREF x (1/65,536) = 0x8000 => 0.5*VREF
 * VREF x (1/65,536)
*/
  uint8_t i,j;
  uint8_t reverse[sizeof(data)];
  uint8_t dummy[sizeof(data)];
  memset(dummy,0x00,sizeof(data));

  uint8_t  *ptr = (uint8_t *)&data;
  j = sizeof(data)-1;
  for(i=0; i<sizeof(data); i++)
    reverse[j--] = *(ptr + i);

  /*VPROG_EN# -> Select */
  HAL_GPIO_WritePin(VPRG_EN_GPIO_Port, VPRG_EN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, reverse, dummy, sizeof(data), 100);
  /*VPROG_EN# -> Select */
  HAL_GPIO_WritePin(VPRG_EN_GPIO_Port, VPRG_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(VPRG_LD_GPIO_Port, VPRG_LD_Pin, GPIO_PIN_RESET);
  /* DAC Load  H->L->H*/
  HAL_GPIO_WritePin(VPRG_LD_GPIO_Port, VPRG_LD_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VPRG_LD_GPIO_Port, VPRG_LD_Pin, GPIO_PIN_SET);
}

void CDacSet(uint16_t data)
{
  uint8_t i,j;
  uint8_t reverse[sizeof(data)];
  uint8_t dummy[sizeof(data)];
  memset(dummy,0x00,sizeof(data));

  uint8_t  *ptr = (uint8_t*)&data;
  j = sizeof(data)-1;
  for(i=0; i<sizeof(data); i++)
    reverse[j--] = *(ptr + i);

  /*CPROG_EN# -> Select */
  HAL_GPIO_WritePin(CPRG_EN_GPIO_Port, CPRG_EN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi2, reverse, dummy, sizeof(data), 100);
  /*CPROG_EN# -> Select */
  HAL_GPIO_WritePin(CPRG_EN_GPIO_Port, CPRG_EN_Pin, GPIO_PIN_SET);
  /* DAC Load  H->L->H*/
  HAL_GPIO_WritePin(CPRG_LD_GPIO_Port, CPRG_LD_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CPRG_LD_GPIO_Port, CPRG_LD_Pin, GPIO_PIN_SET);
}

/* printf -------------------------------------------------------------------*/
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
  return len;
}

void ConsoleWrite(char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}


/* LEDs ---------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(LIVE_LED_GPIO_Port, LIVE_LED_Pin, GPIO_PIN_RESET);
}

void LedGreenOn(void)
{
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void LedGreenOff(void)
{
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}

void LedRedOn(void)
{
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}

void LedRedOff(void)
{
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}

/* PS ---------------------------------------------------------------------*/
void Ps1On(void)
{
  HAL_GPIO_WritePin(PS1ON_GPIO_Port, PS1ON_Pin, GPIO_PIN_SET);
}

void Ps1Off(void)
{
  HAL_GPIO_WritePin(PS1ON_GPIO_Port, PS1ON_Pin, GPIO_PIN_RESET);
}

void Ps2On(void)
{
  HAL_GPIO_WritePin(PS2ON_GPIO_Port, PS2ON_Pin, GPIO_PIN_SET);
}

void Ps2Off(void)
{
  HAL_GPIO_WritePin(PS2ON_GPIO_Port, PS2ON_Pin, GPIO_PIN_RESET);
}

/* MCP9800 -----------------------------------------------------------*/
const uint8_t MCP9800_DEV_ADDR = 0x48 << 1; /*** A0 ***/

void Mcp9800Config (I2C_HandleTypeDef *hi2c)
{
  uint8_t REG_CONF = 0x00; /*** ADC:12bit  ***/
  const uint8_t REG_CONF_ADDR = 0x01 ;
  uint8_t error = 0;

  while(HAL_I2C_Mem_Write(hi2c, MCP9800_DEV_ADDR, REG_CONF_ADDR, I2C_MEMADD_SIZE_8BIT, &REG_CONF, sizeof(REG_CONF), 100)!= HAL_OK)
  {
    if (( error = HAL_I2C_GetError(hi2c)) != HAL_I2C_ERROR_AF)
    {
      DeviceErrLog("Mcp9800Meas.HAL_I2C_Mem_Read Error Code:%d", error);
      break;
    }
  }
}

float Mcp9800Meas(I2C_HandleTypeDef *hi2c)
{
  const uint8_t REG_TA_ADDR = 0x00 ;
  float ta = 0.0;
  uint8_t data[2] = {0 , 0};
  uint8_t error = 0;
  // microchip sample value 0x194 = 25.25C (LSB:0.0625)
  while(HAL_I2C_Mem_Read(hi2c, MCP9800_DEV_ADDR, REG_TA_ADDR, I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 100)!= HAL_OK)
  {
    if ((error = HAL_I2C_GetError(hi2c)) != HAL_I2C_ERROR_AF)
    {
      DeviceErrLog("Mcp9800Meas.HAL_I2C_Mem_Read Error Code:%d", error);
      ta = -1;
      break;
    }
  }
  if(ta != -1)
    ta = (((data[0] << 4) | (data[1] >> 4)) & ~0x800) * 0.0625;
  return ta;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
