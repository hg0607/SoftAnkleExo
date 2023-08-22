/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "ringbuffer.h"
/* USER CODE END Includes */

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define SINGLE_MSG_SIZE 256 //size of single msg
#define BUFARR_SIZE     1
#define MSG_LEN         16
/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
typedef struct
{
  ring_buffer_t infoBuf;
  ring_buffer_t dataBuf;
  volatile uint8_t lock;
  uint32_t lockCnt;
  uint32_t fullCnt;
  uint32_t sendCnt;
}DebugTypeDef;
typedef void (*pSendMsgFunc)(const char *data, uint16_t size);
typedef union 
{
  struct 
  {
    pSendMsgFunc pSendFunc;
    volatile uint32_t len;
  }info;
  char data[8];
}DebugInfoBufTypeDef;

void uartPrintf(const char *fmt, ...); 
//void usbPrintf(const char *fmt, ...); 
void initDebugBuf(void);
void debugPrintf(const char *fmt, ...);
void sendAllDebugMsg(DebugTypeDef *debug);
void uart_start_receive();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

