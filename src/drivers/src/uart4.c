/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * uart3.c - uart3 driver for Hardware UART1
 */


#include <string.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "queue.h"

/*ST includes */
#include "stm32fxxx.h"

#include "config.h"
#include "uart4.h"
#include "nvic.h"
#include "cfassert.h"
#include "config.h"
#include "nvicconf.h"

#define DEBUG_MODULE  "UART4"

xQueueHandle uart4queue;

static bool isInit = false;

void uart4Init(const uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(USART4_GPIO_PERIF, ENABLE);
  ENABLE_USART4_RCC(USART4_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = USART4_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(USART4_GPIO_PORT, &GPIO_InitStructure);



  //Map uart to alternate functions
  GPIO_PinAFConfig(USART4_GPIO_PORT, USART4_GPIO_AF_RX_PIN, USART4_GPIO_AF_RX);

  USART_InitStructure.USART_BaudRate            = baudrate;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx;
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART4_TYPE, &USART_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART4_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  uart4queue = xQueueCreate(64, sizeof(uint8_t));

  USART_ITConfig(USART4_TYPE, USART_IT_RXNE, ENABLE);

  //Enable UART
  USART_Cmd(USART4_TYPE, ENABLE);

  isInit = true;
}

bool uart4Test(void)
{
  return isInit;
}

void uart4SendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
    while (!(USART4_TYPE->SR & USART_FLAG_TXE));
    USART4_TYPE->DR = (data[i] & 0x00FF);
  }
}

int uart4Putchar(int ch)
{
    uart4SendData(1, (uint8_t *)&ch);

    return (unsigned char)ch;
}


void __attribute__((used)) UART4_IRQHandler(void)		//Hardware USART
{
  uint8_t ubxrxData;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  if (USART_GetITStatus(USART4_TYPE, USART_IT_RXNE))
  {
    ubxrxData = USART_ReceiveData(USART4_TYPE) & 0x00FF;
    xQueueSendFromISR(uart4queue, &ubxrxData, &xHigherPriorityTaskWoken);
  }
}
