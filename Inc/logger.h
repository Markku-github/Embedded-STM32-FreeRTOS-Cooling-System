/**
 ******************************************************************************
 * @file    logger.h
 * @brief   Logging system via USART3
 * @note    USART3 uses ST-Link VCP (PD8=TX, PD9=RX) @ 115200 baud
 ******************************************************************************
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "FreeRTOS.h"
#include "queue.h"
#include "config.h"  /* For LOG_MSG_MAX_LEN */

/* External queue handle - defined in main.c */
extern QueueHandle_t xLogQueue;

/**
 * @brief  Initialize USART3 for logging (115200 baud, 8N1)
 * @note   USART3: PD8 = TX, PD9 = RX (Nucleo-F767ZI virtual COM port)
 * @retval None
 */
void Logger_Init(void);

/**
 * @brief  Send one character via USART3 (polling)
 * @param  c: character to send
 * @retval None
 */
void Logger_SendChar(char c);

/**
 * @brief  Send string via USART3 with length limit
 * @param  str: null-terminated string
 * @retval None
 * @note   Limited to LOG_MSG_MAX_LEN characters to prevent excessive transmission time
 */
void Logger_SendString(const char *str);

/**
 * @brief  Logger task - centralized logging via UART3
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void LoggerTask(void *pvParameters);

/**
 * @brief  Queue a log message (thread-safe)
 * @param  msg: null-terminated log message
 * @retval None
 * @note   The message is copied internally before queuing. The queue uses
 *         copy-by-value semantics to avoid dangling pointer issues.
 *         Messages longer than LOG_MSG_MAX_LEN-1 will be truncated.
 */
void Log(const char *msg);

#endif /* LOGGER_H */
