/**
  ******************************************************************************
  * @file    cannet.h
  * @author  Margit R�bert
  * @version 0.0.0
  * @date    2019.05.30
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Konvol�ci�</center></h2>
  *******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _CANNET__H_
#define _CANNET__H_ 1
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define CANNET_DEBUG_LEVEL     0

/* Public define -------------------------------------------------------------*/

#define CANNET_OK               0
#define CANNET_FAIL             1
#define CANNET_NOT_IMPLEMENTED  2
#define CANNET_MSG_TX_FAIL      3
#define CANNET_SIG_NOT_FOUND    4
#define CANNET_UNKNOWN_TYPE     5
#define CANNET_TXFIFO_FULL      6


#define CANNET_EMPTY            0
#define CANNET_FULL             1


#define CANNET_SIGNAL_NAME_SIZE 32
#define CANNET_BROADCAST_MSG  0x01
#define CANNET_CAN_MSG_SIZE     8
#define CANNET_CAN_TX_FIFO_SIZE 16


#define GetNodeType(__arbId__)     ((0x0F000000 & (__arbId__)) >> 24)
#define GetNodeAddress(__arbId__)  ((0x00FF0000 & (__arbId__)) >> 16)
#define GetMsgId(__arbId__)         (0x0000FF00 & (__arbId__)) >> 8
#define IsBroadcasMsg(__arbId__)    (0x00000001 & (__arbId__))

/* Debug Macros --------------------------------------------------------------*/
#if (CANNET_DEBUG_LEVEL > 0)
#define  CanNetUsrLog(...)  {printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define CanNetUsrLog(...)
#endif

#if (CANNET_DEBUG_LEVEL > 1)

#define  CanNetErrLog(...)  {printf("ERROR.CANNET:") ;\
                             printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define CanNetErrLog(...)
#endif

#if (CANNET_DEBUG_LEVEL > 2)
#define  CanNetDbgLog(...)  {printf("DEBUG.CANNET:") ;\
                             printf(__VA_ARGS__);\
                             printf("\r\n");}
#else
#define CanNetDbgLog(...)
#endif

/* Public typedef ------------------------------------------------------------*/
typedef enum _CanNetDataType
{
  CANNET_BOOL,
  CANNET_U8,
  CANNET_U16,
  CANNET_U32,
  CANNET_U64,
  CANNET_FLOAT,
}CanNetDataType_Type;

typedef struct _CanNetMessageItem
{
  char Name[CANNET_SIGNAL_NAME_SIZE];
  uint8_t Id;
  uint8_t NodeType;
  uint8_t IsPeriod;
  uint32_t PeriodTime;
  void (*pInComeNotify)(uint8_t node);
  void (*pValue);
  CanNetDataType_Type Type;
  int32_t LastTimesamp;
}CanNetMessageItem_Type;

typedef struct _CanNetFifoStoreItem
{
  uint32_t ArbId;
  uint8_t  Data[CANNET_CAN_MSG_SIZE];
  size_t Size;
}CanNetFifoStoreItem_Type;

typedef struct _CanNetFifoHandler
{
  uint16_t WritePos;
  uint16_t ReadPos;
  uint16_t Item;
  CanNetFifoStoreItem_Type Storage[CANNET_CAN_TX_FIFO_SIZE];
}CanNetFifoHandler_Type;


typedef struct _CanNet_Type
{
  uint8_t NodeType;
  CanNetMessageItem_Type *pMsg;
  uint8_t MsgCount;
  uint8_t Node;      /*Az �tj�r� eszk�z eset�ben nem kell param�terzeni mert neki nincs c�me.*/
  CanNetFifoHandler_Type TxFifo;
  struct
  {
    uint8_t RxFrame;
    uint8_t ItIsNotForMeFrame;
    uint8_t TxFrame;
  }Diag;
}CanNet_Type;


/* Exported functions --------------------------------------------------------*/
extern uint32_t HAL_GetTick(void);
uint8_t CanNetInit(CanNet_Type *context, uint8_t nodeTypeId, uint8_t node, CanNetMessageItem_Type *msg, uint8_t msgCount);
uint8_t CanNetTask(CanNet_Type *context);
uint8_t CanNetReadMsgLowLevel(CanNet_Type *context, uint32_t arbId, uint8_t *data, size_t size);
uint8_t CanNetSendSignal(CanNet_Type *context, uint8_t node, uint8_t broadcast, uint8_t msgId, void *pValue);
__weak uint8_t CanNetWriteMsgLowLevel(uint32_t arbId, uint8_t *data, size_t size);
__weak uint8_t CanNetLowLevelTxMailboxIsEmpty(void);

#endif //
/************************ (C) COPYRIGHT Konvol�ci� **********END OF FILE****/
