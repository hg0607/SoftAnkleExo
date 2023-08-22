#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include "stm32f4xx_hal.h"
#include "ringbuffer.h"

#define CAN_MSG_DEBUG 0

#define SINGLE_MSG_SIZE 128 //size of single msg

typedef void (*pSendMsgFunc)(const char *data, uint32_t size);

typedef union 
{
  struct 
  {
    pSendMsgFunc pSendFunc;
    volatile uint32_t len;
  }info;
  char data[8];
}DebugInfoBufTypeDef;

typedef struct
{
  ring_buffer_t infoBuf;
  ring_buffer_t dataBuf;
  volatile uint8_t lock;
  uint32_t lockCnt;
  uint32_t fullCnt;
  uint32_t sendCnt;
}DebugTypeDef;

/* DebugCanRxMsg ****************************/
typedef struct
{
  CAN_RxHeaderTypeDef header;
  char canData[8];
  unsigned int timeStamp;
}CanRxMsg;

typedef union
{
  CanRxMsg msg;
  char data[sizeof(CanRxMsg)];
}DebugCanRxMsg;

/* DebugCanTxMsg ****************************/
typedef struct
{
  CAN_TxHeaderTypeDef header;
  char canData[8];
  unsigned int timeStamp;
}CanTxMsg;

typedef union
{
  CanTxMsg msg;
  char data[sizeof(CanTxMsg)];
}DebugCanTxMsg;

/* DebugAdMsg ****************************/
typedef struct 
{
  unsigned char header_1;
  unsigned char header_2;
  unsigned int tick;
  unsigned int cnt;
  unsigned short int adValue[16];
}AdValueMsg;

typedef union
{
  AdValueMsg msg;
  char data[sizeof(AdValueMsg)];
}DebugAdValueMsg;


/* Debug Buffer Function ****************************/
void initDebugBuf(void);
void addDebugMsg(DebugTypeDef *debug, const char *data, uint32_t len, pSendMsgFunc sendMsg);
void sendOneDebugMsg(DebugTypeDef *debug);
void sendAllDebugMsg(DebugTypeDef *debug);

/* Send Msg Function ****************************/
void debugPrintf(const char *fmt, ...);
void debugPrintPending(const char *fmt, ...);
void debugSendPending(const char *data, uint32_t len);
void sendPrintMsgByUart(const char *data, uint32_t size);
void sendCanTxMsgByUart(const char *data, uint32_t size);
void sendCanRxMsgByUart(const char *data, uint32_t size);
void sendAdValueMsgByUart(const char *data, uint32_t size);

/* extern variable ****************************/
extern DebugTypeDef uartDebug;
extern DebugCanTxMsg debugCanTxMsg;
extern DebugCanRxMsg debugCanRxMsg;
extern DebugAdValueMsg adValueMsg;

#endif
