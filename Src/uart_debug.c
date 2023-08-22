#include "uart_debug.h"
#include "stdio.h"
#include "stdarg.h"
#include "stm32f4xx_hal_uart.h"

/* Uart DMA **************************************************************************/
extern UART_HandleTypeDef huart1;
static char uartDebugDmaBuf[SINGLE_MSG_SIZE] = {0};
static char tempBuf[SINGLE_MSG_SIZE] = {0};
static char dmaSendErrorCnt = 0;

static void Error_Handler(void)
{
  dmaSendErrorCnt++;
}
/* AD value Message *********************************************************************/
DebugAdValueMsg adValueMsg;

void sendAdValueMsgByUart(const char *data, uint32_t size)
{
  uint8_t len = 0;
  uint32_t cnt = 0;
  AdValueMsg *adValueMsg = (AdValueMsg *)data;

  // adValueMsg->header_1 = 0x55;
  // adValueMsg->header_2 = 0xaa;

  // for(cnt = 0;cnt < size;cnt++)
  // {
  //   uartDebugDmaBuf[cnt] = data[cnt];
  // }

  len = sprintf(&uartDebugDmaBuf[0], 
                "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                adValueMsg->adValue[0], 
                adValueMsg->adValue[1], 
                adValueMsg->adValue[2], 
                adValueMsg->adValue[3],
                adValueMsg->adValue[4], 
                adValueMsg->adValue[5], 
                adValueMsg->adValue[6], 
                adValueMsg->adValue[7],
                adValueMsg->adValue[8], 
                adValueMsg->adValue[9], 
                adValueMsg->adValue[10], 
                adValueMsg->adValue[11],
                adValueMsg->adValue[12], 
                adValueMsg->adValue[13], 
                adValueMsg->adValue[14], 
                adValueMsg->adValue[15],
                adValueMsg->tick,
                adValueMsg->cnt);

  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uartDebugDmaBuf, len)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();  
  }
}

/* Print Message ************************************************************************/
void debugPrintf(const char *fmt, ...) 
{
  va_list argp;
  uint32_t n = 0;

  va_start(argp, fmt);
  n = vsprintf ((char*)tempBuf, fmt, argp);
  va_end(argp);

  addDebugMsg(&uartDebug, tempBuf, n, sendPrintMsgByUart);
}

void sendPrintMsgByUart(const char *data, uint32_t size)
{
  uint32_t cnt = 0;

  for(cnt = 0;cnt < size;cnt++)
  {
    uartDebugDmaBuf[cnt] = data[cnt];
  }
  
  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uartDebugDmaBuf, size)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();  
  }
}

void debugPrintPending(const char *fmt, ...)
{
  va_list argp;
  uint32_t n = 0;

  va_start(argp, fmt);
  n = vsprintf ((char*)tempBuf, fmt, argp);
  va_end(argp);

  if(HAL_UART_Transmit(&huart1, (uint8_t*)tempBuf, n, 1000)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();  
  }
}

void debugSendPending(const char *data, uint32_t len)
{
  if(HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();  
  }
}

/* CAN Message **************************************************************************/
DebugCanTxMsg debugCanTxMsg;
DebugCanRxMsg debugCanRxMsg;

void sendCanTxMsgByUart(const char *data, uint32_t size)
{
  uint8_t len = 0;
  CanTxMsg *canMsg = (CanTxMsg *)data;

  #if CAN_MSG_DEBUG == 0
  return;
  #endif

  len = sprintf(&uartDebugDmaBuf[0], 
                "T- %d Id:%#5x L:%d Data:%#4x %#4x %#4x %#4x %#4x %#4x %#4x %#4x\r\n", 
                canMsg->timeStamp,
                canMsg->header.StdId, 
                canMsg->header.DLC, 
                canMsg->canData[0], 
                canMsg->canData[1], 
                canMsg->canData[2], 
                canMsg->canData[3], 
                canMsg->canData[4], 
                canMsg->canData[5], 
                canMsg->canData[6], 
                canMsg->canData[7]);

  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uartDebugDmaBuf, len)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();  
  }
}

void sendCanRxMsgByUart(const char *data, uint32_t size)
{
  uint8_t len = 0;
  CanRxMsg *canMsg = (CanRxMsg *)data;

  #if CAN_MSG_DEBUG == 0
  return;
  #endif

  len = sprintf(&uartDebugDmaBuf[0], 
                "R- %d Id:%#5x L:%d Data:%#4x %#4x %#4x %#4x %#4x %#4x %#4x %#4x\r\n", 
                canMsg->timeStamp,
                canMsg->header.StdId, 
                canMsg->header.DLC, 
                canMsg->canData[0], 
                canMsg->canData[1], 
                canMsg->canData[2], 
                canMsg->canData[3], 
                canMsg->canData[4], 
                canMsg->canData[5], 
                canMsg->canData[6], 
                canMsg->canData[7]);

  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*)uartDebugDmaBuf, len)!= HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();  
  }
}

/************************************************************************************/
DebugTypeDef uartDebug;
static char uartDebugInfoBuf[RING_BUFFER_SIZE_L] = {0};
static char uartDebugDataBuf[RING_BUFFER_SIZE_XL] = {0};

void initDebugBuf(void)
{
  ring_buffer_init(&uartDebug.infoBuf, &uartDebugInfoBuf[0], RING_BUFFER_SIZE_L);
  ring_buffer_init(&uartDebug.dataBuf, &uartDebugDataBuf[0], RING_BUFFER_SIZE_XL);
  uartDebug.lock = 0;
  uartDebug.lockCnt = 0;
  uartDebug.fullCnt = 0;
  uartDebug.sendCnt = 0;
}

void addDebugMsg(DebugTypeDef *debug, const char *data, uint32_t len, pSendMsgFunc sendMsg)
{
  DebugInfoBufTypeDef msg;
  
  if(debug->lock == 0)
  {  
    debug->lock = 1;

    if(ring_buffer_num_items(&debug->dataBuf) + len > debug->dataBuf.size)
    {
      debug->fullCnt++;
    }
    else
    {
      msg.info.pSendFunc = sendMsg;
      msg.info.len = len;

      ring_buffer_queue_arr(&debug->infoBuf, msg.data, sizeof(DebugInfoBufTypeDef));
      ring_buffer_queue_arr(&debug->dataBuf, data, len);
    }

    debug->lock = 0;
  }
  else
  {
    debug->lockCnt++;
  }
}
void sendOneDebugMsg(DebugTypeDef *debug)
{
  DebugInfoBufTypeDef msg;

  if(ring_buffer_is_empty(&debug->infoBuf))
    return;

  if(debug->lock == 0)
  {  
    debug->lock = 1;

    ring_buffer_dequeue_arr(&debug->infoBuf, msg.data, sizeof(DebugInfoBufTypeDef));
    ring_buffer_dequeue_arr(&debug->dataBuf, tempBuf, msg.info.len);

    if(msg.info.pSendFunc != 0)
    {
      (msg.info.pSendFunc)(tempBuf, msg.info.len);
    }
    debug->sendCnt++;

    debug->lock = 0;
  }
  else
  {
    debug->lockCnt++;
  }
}

void sendAllDebugMsg(DebugTypeDef *debug)
{
  while(0 == ring_buffer_is_empty(&debug->infoBuf))
  {
    if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
    {
      sendOneDebugMsg(debug);
    }
  }
}

