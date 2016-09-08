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
 * usart3.h - usart3 driver for deck port, on Hardware UART1
 */
#ifndef UART4_H_
#define UART4_H_

#include "crtp.h"
#include <stdbool.h>
#include "eprintf.h"

//								Hardware Definition

#define USART4_TYPE            	UART4
#define USART4_PERIF            RCC_APB1Periph_UART4
#define ENABLE_USART4_RCC       RCC_APB1PeriphClockCmd
#define USART4_IRQ              UART4_IRQn

#define USART4_GPIO_PERIF       RCC_AHB1Periph_GPIOC
#define USART4_GPIO_PORT        GPIOC
#define USART4_GPIO_TX_PIN      GPIO_Pin_10		//Only receiver is used for SBUS RX Communication
#define USART4_GPIO_RX_PIN      GPIO_Pin_11
#define USART4_GPIO_AF_TX_PIN   GPIO_PinSource10	// Only receiver is used for SBUS RX Communication
#define USART4_GPIO_AF_RX_PIN   GPIO_PinSource11
#define USART4_GPIO_AF_TX       GPIO_AF_UART4
#define USART4_GPIO_AF_RX       GPIO_AF_UART4

/**
 * Initialize the UART.
 */
void uart4Init(const uint32_t baudrate);

/**
 * Test the UART status.
 *
 * @return true if the UART is initialized
 */
bool uart4Test(void);

/**
 * Sends raw data using a lock. Should be used from
 * exception functions and for debugging when a lot of data
 * should be transfered.
 * @param[in] size  Number of bytes to send
 * @param[in] data  Pointer to data
 */
void uart4SendData(uint32_t size, uint8_t* data);

/**
 * Send a single character to the serial port using the uartSendData function.
 * @param[in] ch Character to print. Only the 8 LSB are used.
 *
 * @return Character printed
 */
int uart4Putchar(int ch);

// xQueueHandle uart1queue;
/**
 * Uart printf macro that uses eprintf
 * @param[in] FMT String format
 * @param[in] ... Parameters to print
 *
 * @note If UART Crtp link is activated this function does nothing
 */
#define uart4Printf(FMT, ...) eprintf(uart3Putchar, FMT, ## __VA_ARGS__)

#endif /* UART4_H_ */

