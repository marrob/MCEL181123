#ifndef _CANDB__H_
#define _CANDB__H_ 1
/* Includes ------------------------------------------------------------------*/
#include "cannet.h"

/* Public define -------------------------------------------------------------*/
#define NODE_PC            0x01
#define NODE_MCEL          0x05

#define MSG_MCEL_V_MEAS    0x01
#define MSG_MCEL_C_MEAS    0x02
#define MSG_MCEL_RANGE     0x03
#define MSG_MCEL_STATUS    0x04
#define MSG_MCEL_TEMP_TR   0x05
#define MSG_MCEL_TEMP_UC   0x06
#define MSG_MCEL_STA_CC    0x07
#define MSG_MCEL_STA_CV    0x08
#define MSG_MCEL_STA_OE    0x09
#define MSG_MCEL_OFFSET    0x10
#define MSG_MCEL_VER       0xFE
#define MSG_MCEL_LIVE      0xFF
#define MSG_CLB_GET_01     0x21
#define MSG_CLB_GET_02     0x22
#define MSG_CLB_GET_03     0x23
#define MSG_CLB_GET_04     0x24
#define MSG_CLB_GET_05     0x25
#define MSG_CLB_GET_06     0x26
#define MSG_CLB_GET_07     0x27
#define MSG_CLB_GET_08     0x28
#define MSG_CLB_GET_09     0x29
#define MSG_CLB_GET_10     0x30
#define MSG_CLB_GET_11     0x31
#define MSG_CLB_GET_12     0x32
/*--*/
#define MSG_PC_CV          0x01
#define MSG_PC_CC          0x02
#define MSG_PC_RANGE       0x03
#define MSG_PC_OE          0x04
#define MSG_PC_RSENSE      0x05
#define MSG_PC_GD          0x06
#define MSG_PC_TRIG        0x07
#define MSG_PC_UP_SPEED    0x08
#define MSG_PC_DOWN_SPEED  0x09
#define MSG_PC_SERVICES    0x10
#define MSG_PC_CLB_SET01   0x21
#define MSG_PC_CLB_SET02   0x22
#define MSG_PC_CLB_SET03   0x23
#define MSG_PC_CLB_SET04   0x24
#define MSG_PC_CLB_SET05   0x25
#define MSG_PC_CLB_SET06   0x26
#define MSG_PC_CLB_SET07   0x27
#define MSG_PC_CLB_SET08   0x28
#define MSG_PC_CLB_SET09   0x29
#define MSG_PC_CLB_SET10   0x30
#define MSG_PC_CLB_SET11   0x31
#define MSG_PC_CLB_SET12   0x32

/* Public typedef ------------------------------------------------------------*/
typedef enum _Services
{
  SRV_RESTART,
  SRV_UPDATE_STATUS,
  SRV_CALIB_SAVE,
  SRV_CALIB_OFF,
  SRV_CALIB_DEFAULT,
}Services_Type;

/* Public variables ----------------------------------------------------------*/
extern CanNetMessageItem_Type Messages[];


/* Exported functions --------------------------------------------------------*/
uint8_t CanNetMessagesCount(void);

void ChangedEventCVprog(uint8_t node);
void ChangedEventCCprog(uint8_t node);
void ChangedEventCrng(uint8_t node);
void ChangedEventRSense(uint8_t node);
void ChangedEventOE(uint8_t node);
void ChangedEventGD(uint8_t node);
void ChangedEventTrigger(uint8_t node);
void ChangedEventServices(uint8_t node);

#endif
/************************ (C) COPYRIGHT Konvolúció **********END OF FILE****/
