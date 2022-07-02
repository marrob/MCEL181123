/* Includes ------------------------------------------------------------------*/
#include "cannetdb.h"
#include "main.h"

CanNetMessageItem_Type Messages[] = 
{
  /*Name                   Id,               NodeTypeI,     IsPeriod, PeriodTime,  pInComeNotify, pValue,                      Type          */
  /*01234567890123456                                                                                                                          */
  { "MSG_MCEL_V_MEAS",     MSG_MCEL_V_MEAS,   NODE_MCEL,    0,        0,           NULL,                 &Device.PowerCtrl.CVmeas,       CANNET_FLOAT        },
  { "MSG_MCEL_C_MEAS",     MSG_MCEL_C_MEAS,   NODE_MCEL,    0,        0,           NULL,                 &Device.PowerCtrl.CCmeas,       CANNET_FLOAT        },
  { "MSG_MCEL_RANGE",      MSG_MCEL_RANGE,    NODE_MCEL,    1,        500,           NULL,               &Device.PowerCtrl.Range,        CANNET_U8           },
  { "MSG_MCEL_OFFSET",     MSG_MCEL_OFFSET,   NODE_MCEL,    0,        0,           NULL,                 &Device.PowerCtrl.CCoffset,     CANNET_FLOAT        },
  { "MSG_MCEL_TEMP_TR",    MSG_MCEL_TEMP_TR,  NODE_MCEL,    0,        0,           NULL,                 &Device.TransistorTemp,         CANNET_FLOAT        },
  { "MSG_MCEL_TEMP_UC",    MSG_MCEL_TEMP_UC,  NODE_MCEL,    0,        0,           NULL,                 &Device.ControllerTemp,         CANNET_U32,         },
  { "MSG_MCEL_STA_CC",     MSG_MCEL_STA_CC,   NODE_MCEL,    0,        0,           NULL,                 &Device.PowerCtrl.CC,           CANNET_BOOL,        },
  { "MSG_MCEL_STA_CV",     MSG_MCEL_STA_CV,   NODE_MCEL,    0,        0,           NULL,                 &Device.PowerCtrl.CV,           CANNET_BOOL,        },
  { "MSG_MCEL_STA_OE",     MSG_MCEL_STA_OE,   NODE_MCEL,    0,        0,           NULL,                 &Device.PowerCtrl.OE,           CANNET_BOOL,        },
  { "MSG_MCEL_VER",        MSG_MCEL_VER,      NODE_MCEL,    0,        0,           NULL,                 &Device.Version,                CANNET_U64,          },
  { "MSG_MCEL_LIVE",       MSG_MCEL_LIVE,     NODE_MCEL,    1,        500,         NULL,                 &Device.UpTimeSec,              CANNET_U8,          },

  { "MSG_PC_CV",           MSG_PC_CV,         NODE_PC,      0,        0,           &ChangedEventCVprog,  &Device.PowerCtrl.CVprog,       CANNET_FLOAT        },
  { "MSG_PC_CC",           MSG_PC_CC,         NODE_PC,      0,        0,           &ChangedEventCCprog,  &Device.PowerCtrl.CCprog,       CANNET_FLOAT        },
  { "MSG_PC_RANGE",        MSG_PC_RANGE,      NODE_PC,      0,        0,           &ChangedEventCrng,    &Device.PowerCtrl.Range,        CANNET_U8,          },
  { "MSG_PC_OE",           MSG_PC_OE,         NODE_PC,      0,        0,           &ChangedEventOE,      &Device.PowerCtrl.OE,           CANNET_U8,          },
  { "MSG_PC_RSENSE",       MSG_PC_RSENSE,     NODE_PC,      0,        0,           &ChangedEventRSense,  &Device.PowerCtrl.RSense,       CANNET_BOOL,        },
  { "MSG_PC_GD",           MSG_PC_GD,         NODE_PC,      0,        0,           &ChangedEventGD,      &Device.PowerCtrl.GD,           CANNET_BOOL,        },
  { "MSG_PC_TRIG",         MSG_PC_TRIG,       NODE_PC,      0,        0,           &ChangedEventTrigger, &Device.PowerCtrl.Trigger,      CANNET_U8,          },
  { "MSG_PC_UP_SPEED",     MSG_PC_UP_SPEED,   NODE_PC,      0,        0,           NULL,                 &Device.PowerCtrl.Speed.UpSp,   CANNET_FLOAT,       },
  { "MSG_PC_DOWN_SPEED",   MSG_PC_DOWN_SPEED, NODE_PC,      0,        0,           NULL,                 &Device.PowerCtrl.Speed.DownSp, CANNET_FLOAT,       },
  { "MSG_PC_SERVICES",     MSG_PC_SERVICES,   NODE_PC,      0,        0,           &ChangedEventServices,&Device.ServiceCode,            CANNET_U32,         },
  { "MSG_PC_CLB_SET01",    MSG_PC_CLB_SET01,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET01,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET02",    MSG_PC_CLB_SET02,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET02,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET03",    MSG_PC_CLB_SET03,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET03,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET04",    MSG_PC_CLB_SET04,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET04,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET05",    MSG_PC_CLB_SET05,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET05,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET06",    MSG_PC_CLB_SET06,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET06,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET07",    MSG_PC_CLB_SET07,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET07,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET08",    MSG_PC_CLB_SET08,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET08,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET09",    MSG_PC_CLB_SET09,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET09,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET10",    MSG_PC_CLB_SET10,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET10,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET11",    MSG_PC_CLB_SET11,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET11,            CANNET_FLOAT,       },
  { "MSG_PC_CLB_SET12",    MSG_PC_CLB_SET12,  NODE_PC,      0,        0,           NULL,                 &Device.Calib.SET12,            CANNET_FLOAT,       },
};


uint8_t CanNetMessagesCount(void)
{
  return sizeof(Messages)/sizeof(CanNetMessageItem_Type);
}
/************************ (C) COPYRIGHT Konvol�ci� **********END OF FILE****/
