/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Markku Kirjava
 * @brief          : STM32F767ZI FreeRTOS Smart Cooling Controller - Phase 2
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Standard library */
#include <stdint.h>

/* Third-party */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* MCU headers */
#include "stm32f7xx.h"

/* Application modules */
#include "logger.h"
#include "led_control.h"
#include "uart_comm.h"
#include "tasks.h"
#include "config.h"
#include "globals.h"
#include "user_button.h"
#include "watchdog.h"
#include "wcet.h"

/* Global FreeRTOS queues (extern in headers) */
QueueHandle_t xTempQueue = NULL;   /* Temperature data queue */
QueueHandle_t xLogQueue = NULL;    /* Log message queue */
QueueHandle_t xCmdQueue = NULL;    /* Command character queue (from UART RX) */

/* Mutex for protecting shared variables */
SemaphoreHandle_t xTempMutex = NULL;      /* Protects currentTemperature */
SemaphoreHandle_t xEmergencyMutex = NULL; /* Protects emergencySignal */

/* Current system state (shared, protected by FreeRTOS scheduling) */
volatile SystemState_t currentState = STATE_IDLE;

/* Current temperature (shared, protected by xTempMutex) */
volatile float currentTemperature = 0.0f;

/* Emergency signal (shared, protected by xEmergencyMutex) */
volatile uint8_t emergencySignal = 0;  /* 1 = Emergency sensor active, 0 = Normal */

/**
 * @brief  Main program
 * @retval int
 */
int main(void)
{
    /* Initialize System */
    SystemInit();
    
    /* Read and store reset reason before clearing flags */
    uint32_t resetFlags = RCC->CSR;
    
    /* Initialize GPIO for LEDs */
    LED_Init();
    
    /* Initialize USER button (PC13) with interrupt */
    UserButton_Init();
    
    /* Initialize UART (USART3 for logging) */
    Logger_Init();
    
    /* Log reset reason (after logger is initialized) */
    if (resetFlags & RCC_CSR_LPWRRSTF)
    {
        Log("[SYSTEM] Reset reason: Low-power management reset");
    }
    else if (resetFlags & RCC_CSR_WWDGRSTF)
    {
        Log("[SYSTEM] Reset reason: Window watchdog reset");
    }
    else if (resetFlags & RCC_CSR_IWDGRSTF)
    {
        Log("[SYSTEM] Reset reason: Independent watchdog reset");
    }
    else if (resetFlags & RCC_CSR_SFTRSTF)
    {
        Log("[SYSTEM] Reset reason: Software reset");
    }
    else if (resetFlags & RCC_CSR_PORRSTF)
    {
        Log("[SYSTEM] Reset reason: Power-on reset (POR/PDR)");
    }
    else if (resetFlags & RCC_CSR_PINRSTF)
    {
        Log("[SYSTEM] Reset reason: External pin reset (NRST)");
    }
    else if (resetFlags & RCC_CSR_BORRSTF)
    {
        Log("[SYSTEM] Reset reason: Brown-out reset (BOR)");
    }
    else
    {
        Log("[SYSTEM] Reset reason: Unknown");
    }
    
    /* Clear reset flags */
    RCC->CSR |= RCC_CSR_RMVF;
    
    /* Initialize UART2 (USART6 on Arduino D0/D1 for command/status interface) */
    UART_Comm_Init();
    
    /* Create queues */
    xTempQueue = xQueueCreate(TEMP_QUEUE_SIZE, sizeof(TempData_t));
    xLogQueue = xQueueCreate(LOG_QUEUE_SIZE, LOG_MSG_MAX_LEN);  /* Copy-by-value: queue of char arrays */
    xCmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(char));
    
    /* Create mutexes for shared variable protection */
    xTempMutex = xSemaphoreCreateMutex();
    xEmergencyMutex = xSemaphoreCreateMutex();
    
    if (xTempQueue == NULL || xLogQueue == NULL || xCmdQueue == NULL ||
        xTempMutex == NULL || xEmergencyMutex == NULL)
    {
        /* Queue or mutex creation failed - halt */
        Logger_SendString("ERROR: Queue/Mutex creation failed!\r\n");
        for (;;);
    }
    
    /* Create WatchdogTask (monitors task heartbeats and feeds IWDG) */
    xTaskCreate(
        WatchdogTask,
        "Watchdog",
        WATCHDOG_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 4,  /* Highest priority - must run to feed watchdog */
        NULL
    );
    
    /* Create UserButtonTask (emergency sensor simulation - critical) */
    xTaskCreate(
        UserButtonTask,
        "UserButton",
        USER_BUTTON_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 3,  /* High priority - emergency signal must be immediate */
        NULL
    );
    
    /* Create ControllerTask (state machine and control logic) */
    xTaskCreate(
        ControllerTask,
        "Controller",
        CONTROLLER_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 2,  /* Control priority */
        NULL
    );
    
    /* Create FanControlTask (actuator control - green LED/fan simulation) */
    xTaskCreate(
        FanControlTask,
        "FanControl",
        FAN_CONTROL_STACK_SIZE,  /* 192 words = 768 bytes */
        NULL,
        tskIDLE_PRIORITY + 2,  /* Same as controller - real-time actuation */
        NULL
    );
    
    /* Create AnalysisTask (temperature monitoring - reads value set via UART) */
    xTaskCreate(
        AnalysisTask,
        "Analysis",
        ANALYSIS_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,  /* Normal priority - monitoring task */
        NULL
    );
    
    /* Create BlueLEDControlTask (status indication - blue LED patterns) */
    xTaskCreate(
        BlueLEDControlTask,
        "BlueLED",
        BLUE_LED_STACK_SIZE,     /* 256 words = 1024 bytes (SOS pattern needs more stack) */
        NULL,
        tskIDLE_PRIORITY + 1,  /* Normal priority - indication only */
        NULL
    );

    /* Create LoggerTask (diagnostic logging via USART3) */
    xTaskCreate(
        LoggerTask,
        "Logger",
        LOGGER_STACK_SIZE,     /* 256 words */
        NULL,
        tskIDLE_PRIORITY + 0,  /* Low priority - must not block control tasks */
        NULL
    );

    /* Create CommandTask (user command interface via USART6) */
    xTaskCreate(
        CommandTask,
        "Command",
        COMMAND_STACK_SIZE,    /* 512 words */
        NULL,
        tskIDLE_PRIORITY + 0,  /* Low priority - user interaction can wait */
        NULL
    );

    /* Initialize watchdog hardware (WatchdogTask will feed it based on heartbeats) */
    Watchdog_Init();

    /* Initialize performance tracking (WCET measurement) */
    WCET_Init(SystemCoreClock);

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;);
}
