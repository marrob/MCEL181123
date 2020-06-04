/**
  ******************************************************************************
  * @file    can_net.c
  * @author  Margit R�bert
  * @version 0.0.1
  * @date    2019.05.30

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "cannet.h"
#include <stdio.h>

/* Private macro -------------------------------------------------------------*/
inline static void Insert(CanNetFifoHandler_Type *context, CanNetFifoStoreItem_Type *item)
{
  memcpy(&(context->Storage[context->WritePos]), item, sizeof(CanNetFifoStoreItem_Type));
  context->WritePos++;
  context->WritePos &= CANNET_CAN_TX_FIFO_SIZE - 1;
  context->Item++;
}

inline static void Extract(CanNetFifoHandler_Type *context, CanNetFifoStoreItem_Type *item)
{
  memcpy(item, &(context->Storage[context->ReadPos]),sizeof(CanNetFifoStoreItem_Type));
  context->ReadPos++;
  context->ReadPos &= CANNET_CAN_TX_FIFO_SIZE - 1;
  context->Item--;
}

inline static void Clear(CanNetFifoHandler_Type *handler)
{
  handler->WritePos=0;
  handler->ReadPos=0;
  handler->Item=0;
}

/* Private function prototypes -----------------------------------------------*/
uint8_t CanNetVar2Msg(void *pValue, char *name, CanNetDataType_Type type, uint8_t *msg, size_t *size);
uint8_t CanNetMsg2Var(void *pValue, char *name, CanNetDataType_Type type, uint8_t *msg, size_t size);
static uint32_t GetArbId(uint8_t nodeType, uint8_t node, uint8_t broadcast, uint8_t msgId);


/**@brief Eszk�z incializ�l�sa
  *@retval status
  */
uint8_t CanNetInit(CanNet_Type *context, uint8_t nodeType, uint8_t node,  CanNetMessageItem_Type *msg, uint8_t msgCount)
{
  context->NodeType = nodeType;
  context->pMsg = msg;
  context->MsgCount = msgCount;
  context->Node = node;
  context->TxFifo.Item = 100;
  Clear(&context->TxFifo);
  return CANNET_OK;
}
/**@brief Task
  *@retval status
  */
uint8_t CanNetTask(CanNet_Type *context)
{
  uint8_t status = CANNET_OK;
  uint8_t data[CANNET_CAN_MSG_SIZE];
  size_t size;
  CanNetFifoStoreItem_Type bufItem;

  for(uint8_t i = 0; i < context->MsgCount; i++)
  {
    CanNetMessageItem_Type *msg = &((CanNetMessageItem_Type*)(context->pMsg))[i];
    if(msg->IsPeriod && context->NodeType == msg->NodeType)
    {
      int32_t now = HAL_GetTick();
      if((now - msg->LastTimesamp) >= msg->PeriodTime )
      {
        CanNetVar2Msg(msg->pValue, msg->Name, msg->Type, data, &size);
        msg->LastTimesamp = HAL_GetTick();
        bufItem.ArbId = GetArbId(context->NodeType, context->Node,0 ,msg->Id);
        memcpy(bufItem.Data, data, CANNET_CAN_MSG_SIZE);
        bufItem.Size = size;
        if (context->TxFifo.Item < CANNET_CAN_TX_FIFO_SIZE)
          Insert(&context->TxFifo, &bufItem);
        else
          return CANNET_TXFIFO_FULL;
       break;
      }
    }
  }

  if(context->TxFifo.Item != 0)
  {
    if(CanNetLowLevelTxMailboxIsEmpty() == CANNET_EMPTY)
    {
      context->Diag.TxFrame++;
      CanNetFifoStoreItem_Type bufItem;
      Extract(&context->TxFifo, &bufItem);
      status = CanNetWriteMsgLowLevel(bufItem.ArbId, bufItem.Data, bufItem.Size);
    }
  }
return status;
}

//__weak uint8_t CanNetWriteMsgLowLevel(uint32_t arbId, uint8_t *data, size_t size)
//{
//  return CANNET_NOT_IMPLEMENTED;
//}
//
//__weak uint8_t CanNetLowLevelTxMailboxIsEmpty(void)
//{
//  return CANNET_NOT_IMPLEMENTED;
//}

uint8_t CanNetReadMsgLowLevel(CanNet_Type *context, uint32_t arbId, uint8_t *data, size_t size)
{
  for(uint8_t i=0; i<context->MsgCount; i++)
  {
    CanNetMessageItem_Type *msg;
    msg = &((CanNetMessageItem_Type*)(context->pMsg))[i];

    if((msg->NodeType == GetNodeType(arbId)) &&  (msg->Id == GetMsgId(arbId)))
    {
      context->Diag.RxFrame++;
      CanNetMsg2Var(msg->pValue, msg->Name, msg->Type, data, size);
      if(msg->pInComeNotify != NULL)
        msg->pInComeNotify(GetNodeAddress(arbId));
    }
    else
    {
      context->Diag.ItIsNotForMeFrame++;
    }
  }
  return CANNET_OK;
}


/**
  * @brief CanNetSendSignal
  * You can send message behalf any node with the node paramter
  */
uint8_t CanNetSendSignal(CanNet_Type *context, uint8_t node, uint8_t broadcast, uint8_t msgId, void *pValue)
{
  uint8_t isFoundSignal;
  uint8_t status = CANNET_OK;

  uint8_t data[CANNET_CAN_MSG_SIZE];
  size_t size;

    for(uint8_t i = 0; i < context->MsgCount; i++)
    {
      CanNetMessageItem_Type *msg = &((CanNetMessageItem_Type*)(context->pMsg))[i];
      if(msgId == msg->Id && msg->NodeType == context->NodeType)
      {
        isFoundSignal = 1;
        CanNetVar2Msg(pValue,msg->Name,msg->Type,data, &size);
#if (CANNET_DEBUG_LEVEL > 0)
        CanNetDbgLog("TX: %s %s %d", msg->Name, sig->Name, sig->Cnt++);
#endif
        CanNetFifoStoreItem_Type bufItem;
        bufItem.ArbId = GetArbId(context->NodeType, node, broadcast, msg->Id);
        memcpy(bufItem.Data, data, CANNET_CAN_MSG_SIZE);
        bufItem.Size = size;
        Insert(&context->TxFifo, &bufItem);
        break;
      }
    }
  if (isFoundSignal)
    return CANNET_OK;
  else
    return CANNET_SIG_NOT_FOUND;
}

static uint32_t GetArbId(uint8_t nodeType, uint8_t node, uint8_t broadcast, uint8_t msgId)
{
  uint32_t arbId = 0;
  arbId |= nodeType << 24;
  if(!broadcast)
    arbId |= node << 16;
  arbId |= msgId << 8;
  if(broadcast)
  arbId |= 1;
  return arbId;
}

uint8_t CanNetVar2Msg(void *pValue, char *name, CanNetDataType_Type type, uint8_t *msg, size_t *size)
{
  uint8_t bytes = 0;
  memset(msg, 0, CANNET_CAN_MSG_SIZE);
  switch(type)
  {
    case CANNET_U8:
    case CANNET_BOOL:
    {
      *size = bytes = sizeof(uint8_t);
      msg[0] = *(uint8_t*)pValue;
      break;
    }
     case CANNET_U16:
    {
     *size = bytes = sizeof(uint16_t);
      for(int i = 0; bytes--; i++)
        msg[bytes] = *((uint8_t*)(pValue)+i);
      break;
    }
    case CANNET_U32:
    {
     *size = bytes = sizeof(uint32_t);
      for(int i = 0; bytes--; i++)
        msg[bytes] = *((uint8_t*)(pValue)+i);
      break;
    }
    case CANNET_U64:
    {
     *size = bytes = sizeof(uint64_t);
      for(int i = 0; bytes--; i++)
        msg[bytes] = *((uint8_t*)(pValue)+i);
      break;
    }
    case CANNET_FLOAT:
    {

     *size = bytes = sizeof(float);
      for(int i = 0;  bytes--; i++)
      msg[bytes] = *((uint8_t*)(pValue)+i);
      #if (CANNET_DEBUG_LEVEL > 0)
        CanNetDbgLog("TX: %s %f", name, *(float*)pValue);
      #endif
      break;
    }
    default:
    {
      return CANNET_UNKNOWN_TYPE;
    }
  }
  return CANNET_OK;
}

uint8_t CanNetMsg2Var(void *pValue, char *name, CanNetDataType_Type type, uint8_t *msg, size_t size)
{
  uint8_t bytes = 0;
  switch(type)
  {
    case CANNET_U8:
    case CANNET_BOOL:
    {
      bytes = sizeof(uint8_t);
      *(uint8_t*)pValue = msg[0];
      break;
    }
    case CANNET_U16:
    {
      bytes = sizeof(uint16_t);
      for(int i = 0; bytes--; i++)
       *((uint8_t*)(pValue)+i) = msg[bytes];
      break;
    }
    case CANNET_U32:
    {

      bytes = sizeof(uint32_t);
      for(int i = 0; bytes--; i++)
      *((uint8_t*)(pValue)+i) = msg[bytes];
      break;
    }
    case CANNET_U64:
    {
      bytes = sizeof(uint64_t);
      for(int i = 0; bytes--; i++)
      *((uint8_t*)(pValue)+i) = msg[bytes];
      break;
    }

    case CANNET_FLOAT:
    {
      bytes = sizeof(float);
      for(int i = 0;  bytes--; i++)
     *((uint8_t*)(pValue) + i) = msg[bytes] ;
    }
    default:
    {
      return CANNET_UNKNOWN_TYPE;
    }
  }
  return CANNET_OK;
}


/************************ (C) COPYRIGHT Konvol�ci� **********END OF FILE****/

