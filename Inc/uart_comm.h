/**
 ******************************************************************************
 * @file    uart_comm.h
 * @brief   UART command interface via USART6
 * @note    USART6 uses Arduino D0/D1 pins (PG9=RX, PG14=TX) @ 115200 baud
 ******************************************************************************
 */

#ifndef UART_COMM_H
#define UART_COMM_H

#include "FreeRTOS.h"
#include "queue.h"

/* External queue handle - defined in main.c */
extern QueueHandle_t xCmdQueue;

/**
 * @brief  Initialize USART6 for command interface (115200 baud, 8N1)
 * @note   USART6: PG14 = TX (D1), PG9 = RX (D0) - Arduino headers
 * @retval None
 */
void UART_Comm_Init(void);

/**
 * @brief  Send one character via USART6 (polling)
 * @param  c: character to send
 * @retval None
 */
void UART_Comm_SendChar(char c);

/**
 * @brief  Send string via USART6 with length limit
 * @param  str: null-terminated string
 * @retval None
 * @note   Limited to 256 characters to prevent excessive transmission time
 */
void UART_Comm_SendString(const char *str);

/**
 * @brief  USART6 interrupt handler - receives characters and queues them
 * @retval None
 */
void USART6_IRQHandler(void);

/**
 * @brief  Command processing task - handles commands from UART6
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void CommandTask(void *pvParameters);

#endif /* UART_COMM_H */
