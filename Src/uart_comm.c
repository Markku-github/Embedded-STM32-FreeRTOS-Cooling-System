/**
 ******************************************************************************
 * @file    uart_comm.c
 * @brief   UART command interface implementation via USART6
 * @note    USART6 uses Arduino D0/D1 pins (PG9=RX, PG14=TX) @ 115200 baud
 ******************************************************************************
 */

/* Own header */
#include "uart_comm.h"

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

/* MCU headers */
#include "stm32f7xx.h"

/* Project headers */
#include "globals.h"
#include "led_control.h"
#include "tasks.h"
#include "utils.h"
#include "logger.h"
#include "config.h"
#include "watchdog.h"

/**
 * @brief  Initialize USART6 for command interface (115200 baud, 8N1)
 * @note   USART6: PG14 = TX (D1), PG9 = RX (D0) - Arduino headers
 * @retval None
 */
void UART_Comm_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    GPIOG->MODER &= ~((3 << 28) | (3 << 18));
    GPIOG->MODER |= ((2 << 28) | (2 << 18));
    GPIOG->AFR[1] &= ~((0xF << 24) | (0xF << 4));
    GPIOG->AFR[1] |= ((8 << 24) | (8 << 4));
    
    /* Configure baud rate for 115200 baud */
    USART6->BRR = UART_BRR_115200;
    
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    
    /* Set interrupt priority to allow FreeRTOS ISR-safe API calls
     * Priority must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
     * In ARM Cortex-M: LOWER number = HIGHER priority
     * Priority 6 is LOWER priority than 5, which is safe for FreeRTOS API calls
     */
    NVIC_SetPriority(USART6_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(USART6_IRQn);
}

/**
 * @brief  Send one character via USART6 (polling)
 * @param  c: character to send
 * @retval None
 */
void UART_Comm_SendChar(char c)
{
    while (!(USART6->ISR & USART_ISR_TXE));
    USART6->TDR = c;
}

/**
 * @brief  Send string via USART6 with length limit
 * @param  str: null-terminated string
 * @retval None
 * @note   Limited to 256 characters to prevent excessive transmission time
 */
void UART_Comm_SendString(const char *str)
{
    size_t count = 0;
    while (*str && count < 256)
    {
        UART_Comm_SendChar(*str++);
        count++;
    }
}

/**
 * @brief  USART6 interrupt handler - receives characters and queues them
 * @note   ISR-safe implementation:
 *         - Uses xQueueSendFromISR (ISR-safe API)
 *         - Calls portYIELD_FROM_ISR to request context switch
 *         - Priority set to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1
 *         - Minimal processing (read char, queue it)
 * @retval None
 */
void USART6_IRQHandler(void)
{
    if (USART6->ISR & USART_ISR_RXNE)
    {
        char c = USART6->RDR;
        BaseType_t woken = pdFALSE;
        xQueueSendFromISR(xCmdQueue, &c, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/**
 * @brief  Case-insensitive string comparison (internal helper)
 * @param  s1: first string (converted to lowercase)
 * @param  s2: second string (must be lowercase)
 * @param  n: number of characters to compare
 * @retval 1 if strings match, 0 otherwise
 */
static uint8_t strncmp_lower(const char *s1, const char *s2, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        char c1 = s1[i];
        if (c1 >= 'A' && c1 <= 'Z') c1 += 32;
        if (c1 != s2[i]) return 0;
    }
    return 1;
}

/**
 * @brief  Process received command string
 * @param  cmd: command buffer
 * @param  len: command length
 * @retval None
 * @note   Supported commands: help, status, temp, emergency, reset
 */
static void processCommand(char *cmd, uint8_t len)
{
    char tempStr[8];
    if (len == 0) return;
    if (len == 4 && strncmp_lower(cmd, "help", 4))
    {
        UART_Comm_SendString("\r\n=== Commands ===\r\n");
        UART_Comm_SendString("temp <0-100> - Set temperature\r\n");
        UART_Comm_SendString("status       - Show status\r\n");
        UART_Comm_SendString("emergency    - Trigger emergency (ALARM state)\r\n");
        UART_Comm_SendString("reset        - Reset to IDLE (temp=0, clear emergency)\r\n");
        UART_Comm_SendString("help         - Show help\r\n");
    }
    else if (len == 6 && strncmp_lower(cmd, "status", 6))
    {
        float temp = 0;
        uint8_t emerg = 0;
        if (xSemaphoreTake(xTempMutex, 100) == pdTRUE) { temp = currentTemperature; xSemaphoreGive(xTempMutex); }
        if (xSemaphoreTake(xEmergencyMutex, 100) == pdTRUE) { emerg = emergencySignal; xSemaphoreGive(xEmergencyMutex); }
        UART_Comm_SendString("State: ");
        UART_Comm_SendString(SystemState_ToString(currentState));
        UART_Comm_SendString(" | Temp: ");
        simple_itoa((int)temp, tempStr);
        UART_Comm_SendString(tempStr);
        UART_Comm_SendString("C | Emerg: ");
        UART_Comm_SendChar(emerg ? '1' : '0');
        UART_Comm_SendString("\r\n");
    }
    else if (len == 5 && strncmp_lower(cmd, "reset", 5))
    {
        /* FULL SYSTEM RESET - return to IDLE state */
        /* Clear temperature */
        if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
        {
            currentTemperature = 0.0f;
            xSemaphoreGive(xTempMutex);
        }
        else
        {
            Log("[CMD] WARNING: Temp mutex timeout during reset");
        }
        
        /* Clear emergency signal */
        if (xSemaphoreTake(xEmergencyMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
        {
            emergencySignal = 0;
            xSemaphoreGive(xEmergencyMutex);
        }
        else
        {
            Log("[CMD] WARNING: Emergency mutex timeout during reset");
        }
        
        /* Log the software reset via command */
        Log("[CMD] System reset command received");
        
        /* State will be updated by ControllerTask to IDLE */
        UART_Comm_SendString("System reset to IDLE state. Temp=0C, Emergency cleared.\r\n");
    }
    else if (len >= 6 && strncmp_lower(cmd, "temp ", 5))
    {
        int16_t val = 0;
        /* Parse integer with overflow protection */
        for (uint8_t i = 5; i < len; i++)
        {
            if (cmd[i] >= '0' && cmd[i] <= '9')
            {
                /* Check for overflow before multiplication */
                if (val > 1000)
                {
                    UART_Comm_SendString("ERROR: Value too large (max 100)\r\n");
                    return;
                }
                val = val * 10 + (cmd[i] - '0');
            }
        }
        
        if (val >= 0 && val <= 100)
        {
            if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
            {
                currentTemperature = (float)val;
                xSemaphoreGive(xTempMutex);
                UART_Comm_SendString("Temp set to ");
                simple_itoa(val, tempStr);
                UART_Comm_SendString(tempStr);
                UART_Comm_SendString("C\r\n");
            }
            else
            {
                UART_Comm_SendString("ERROR: Mutex timeout\r\n");
                Log("[CMD] WARNING: Temp mutex timeout in temp command");
            }
        }
        else
        {
            UART_Comm_SendString("ERROR: Must be 0-100\r\n");
        }
    }
    else if (len == 9 && strncmp_lower(cmd, "emergency", 9))
    {
        /* Trigger emergency - set emergency signal to 1 */
        if (xSemaphoreTake(xEmergencyMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
        {
            emergencySignal = 1;
            xSemaphoreGive(xEmergencyMutex);
            UART_Comm_SendString("EMERGENCY TRIGGERED! System entering ALARM state.\r\n");
            UART_Comm_SendString("Use 'reset' command to clear.\r\n");
        }
        else
        {
            UART_Comm_SendString("ERROR: Mutex timeout\r\n");
            Log("[CMD] WARNING: Emergency mutex timeout in emergency command");
        }
    }
    else { UART_Comm_SendString("Unknown command. Type 'help'\r\n"); }
}

/**
 * @brief  Command processing task - handles commands from UART6
 * @param  pvParameters: pointer to task parameters
 * @retval None
 * @note   Command buffer is 64 bytes with overflow protection
 */
void CommandTask(void *pvParameters)
{
    (void)pvParameters;
    char rxChar, cmdBuffer[64];
    uint8_t idx = 0;
    const uint8_t CMD_BUFFER_SIZE = sizeof(cmdBuffer) - 1;  /* Reserve 1 byte for null terminator */
    
    Log("[CMD] CommandTask started");
    vTaskDelay(pdMS_TO_TICKS(100));
    UART_Comm_SendString("\r\n=====================================\r\n  STM32F767 Temperature Control v1.0\r\n  Command Interface Ready\r\n=====================================\r\nType 'help' for commands\r\n> ");
    for (;;)
    {
        /* Use timeout instead of portMAX_DELAY to allow periodic processing */
        if (xQueueReceive(xCmdQueue, &rxChar, pdMS_TO_TICKS(QUEUE_RECEIVE_TIMEOUT_MS)) == pdTRUE)
        {
            if (rxChar == '\r' || rxChar == '\n') 
            { 
                UART_Comm_SendString("\r\n"); 
                cmdBuffer[idx] = '\0'; 
                processCommand(cmdBuffer, idx); 
                idx = 0; 
                UART_Comm_SendString("> "); 
            }
            else if (rxChar >= 0x20 && rxChar <= 0x7E && idx < CMD_BUFFER_SIZE) 
            { 
                cmdBuffer[idx++] = rxChar; 
                UART_Comm_SendChar(rxChar); 
            }
            else if ((rxChar == 0x08 || rxChar == 0x7F) && idx > 0) 
            { 
                idx--; 
                UART_Comm_SendString("\b \b"); 
            }
            /* Silently ignore characters when buffer is full (no echo) */
        }
        
        /* Report heartbeat to watchdog (even on timeout) */
        Watchdog_ReportHeartbeat(WATCHDOG_TASK_COMMAND);
        /* Timeout is normal - just continue waiting for next character */
    }
}
