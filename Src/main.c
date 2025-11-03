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

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f7xx.h"

// Register definitions for STM32F767ZI
#define GPIOB_BASE      0x40020400

#define RCC_AHB1ENR     (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_ODR       (*(volatile uint32_t*)(GPIOB_BASE + 0x14))

#define GPIOB_EN        (1 << 1)   // Bit 1 for GPIOB

// LED pin definitions (Nucleo-F767ZI)
#define LED_GREEN       0   // LD1 on PB0 (Fan speed indicator / COOLING state)
#define LED_BLUE        7   // LD2 on PB7 (System active / MONITORING state)
#define LED_RED         14  // LD3 on PB14 (Emergency / ALARM state)

// Temperature thresholds (°C)
#define TEMP_NORMAL_MAX     20   // Below this: MONITORING
#define TEMP_WARNING_MAX    20   // Above this: COOLING starts
#define TEMP_CRITICAL       80   // Above this: ALARM

// Queue sizes
#define TEMP_QUEUE_SIZE     10
#define LOG_QUEUE_SIZE      20

// Log message max length
#define LOG_MSG_MAX_LEN     128

/* System state machine */
typedef enum {
    STATE_IDLE = 0,      /* System idle, no monitoring */
    STATE_MONITORING,    /* Normal monitoring, temp < TEMP_WARNING_MAX */
    STATE_COOLING,       /* Active cooling, temp >= TEMP_WARNING_MAX */
    STATE_ALARM          /* Critical alarm, temp >= TEMP_CRITICAL */
} SystemState_t;

/* Temperature data structure */
typedef struct {
    float temperature;   /* Temperature in Celsius */
    uint32_t timestamp;  /* Timestamp in ms */
} TempData_t;

/* Global FreeRTOS queues */
static QueueHandle_t xTempQueue = NULL;   /* Temperature data queue */
static QueueHandle_t xLogQueue = NULL;    /* Log message queue */

/* Current system state (shared, protected by FreeRTOS scheduling) */
static volatile SystemState_t currentState = STATE_IDLE;

/* Current temperature (shared, updated by AnalysisTask) */
static volatile float currentTemperature = 0.0f;

/* Emergency signal (shared, indicates external emergency sensor active) */
static volatile uint8_t emergencySignal = 0;  /* 1 = Emergency sensor active, 0 = Normal */

/* Function prototypes */
static void ControllerTask(void *pvParameters);
static void AnalysisTask(void *pvParameters);
static void LoggerTask(void *pvParameters);
static void FanControlTask(void *pvParameters);
static void BlueLEDControlTask(void *pvParameters);
static void GPIO_Init(void);
static void UART_Init(void);
static void UART_SendChar(char c);
static void UART_SendString(const char *str);
static void LED_On(uint8_t pin);
static void LED_Off(uint8_t pin);
static void LED_Toggle(uint8_t pin);
static void LED_SetState(SystemState_t state);
static void Log(const char *message);

/**
 * @brief  Controller task - implements state machine
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
static void ControllerTask(void *pvParameters)
{
    (void)pvParameters;
    TempData_t tempData;
    SystemState_t newState;
    uint32_t idleStartTime = 0;
    const uint32_t IDLE_DURATION_MS = 5000;  /* 5 seconds in IDLE before auto-start */
    
    Log("[CTRL] ControllerTask started");
    
    /* Start in IDLE state - stays here until explicitly moved via command */
    currentState = STATE_IDLE;
    LED_SetState(currentState);
    Log("[CTRL] State: IDLE (waiting for start command)");
    idleStartTime = xTaskGetTickCount();
    
    for (;;)
    {
        /* Auto-start after IDLE_DURATION_MS for demo purposes */
        /* TODO: Remove auto-start when UART command interface is ready */
        if (currentState == STATE_IDLE)
        {
            uint32_t currentTime = xTaskGetTickCount();
            if ((currentTime - idleStartTime) >= IDLE_DURATION_MS)
            {
                currentState = STATE_MONITORING;
                LED_SetState(currentState);
                Log("[CTRL] State: MONITORING (auto-started for demo)");
            }
            /* Drain temperature queue but don't process in IDLE */
            xQueueReceive(xTempQueue, &tempData, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
        /* Wait for temperature data from AnalysisTask */
        if (xQueueReceive(xTempQueue, &tempData, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            /* Emergency signal overrides temperature-based state logic */
            if (emergencySignal)
            {
                newState = STATE_ALARM;
            }
            /* Normal temperature-based state determination */
            else if (tempData.temperature >= TEMP_CRITICAL)
            {
                newState = STATE_ALARM;
            }
            else if (tempData.temperature >= TEMP_WARNING_MAX)
            {
                newState = STATE_COOLING;
            }
            else if (tempData.temperature < TEMP_NORMAL_MAX)
            {
                newState = STATE_MONITORING;
            }
            else
            {
                /* Hysteresis: stay in current state if between thresholds */
                newState = currentState;
            }
            
            /* State transition */
            if (newState != currentState)
            {
                char logMsg[LOG_MSG_MAX_LEN];
                const char* stateNames[] = {"IDLE", "MONITORING", "COOLING", "ALARM"};
                
                /* Log different message if emergency-triggered */
                if (emergencySignal && newState == STATE_ALARM)
                {
                    snprintf(logMsg, sizeof(logMsg),
                             "[CTRL] State: %s -> %s (EMERGENCY SIGNAL!)",
                             stateNames[currentState], stateNames[newState]);
                }
                else
                {
                    snprintf(logMsg, sizeof(logMsg),
                             "[CTRL] State: %s -> %s (Temp: %.1f C)",
                             stateNames[currentState], stateNames[newState],
                             tempData.temperature);
                }
                Log(logMsg);
                
                currentState = newState;
                LED_SetState(currentState);
            }
            /* Even if state doesn't change, update LED (emergency signal might have changed) */
            else if (currentState == STATE_ALARM)
            {
                LED_SetState(currentState);
            }
        }
        
        /* Small delay */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief  Analysis task - simulates temperature sensor and processing
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
static void AnalysisTask(void *pvParameters)
{
    (void)pvParameters;
    TempData_t tempData;
    float simulatedTemp = 0.0f; /* Start at 0°C */
    uint32_t cycle = 0;
    
    Log("[ANLY] AnalysisTask started");
    
    for (;;)
    {
        /* Simulate temperature variation (sawtooth pattern) */
        /* Cycle: 0°C -> 100°C over 100 seconds, then reset to IDLE for 5s */
        if (cycle >= 100)
        {
            /* Reset cycle and return to IDLE for 5 seconds */
            cycle = 0;
            currentState = STATE_IDLE;
            LED_SetState(currentState);
            Log("[ANLY] Cycle complete, returning to IDLE for 5s");
            vTaskDelay(pdMS_TO_TICKS(5000));
            /* After IDLE, ControllerTask will auto-start to MONITORING */
        }
        
        simulatedTemp = 0.0f + (float)cycle;
        
        /* Update global current temperature for FanControlTask */
        currentTemperature = simulatedTemp;
        
        /* TODO: Remove this simulation when USER button (PC13) is implemented */
        /* Simulate emergency sensor trigger at high temperature (90-95°C) for demo */
        if (simulatedTemp >= 90.0f && simulatedTemp < 95.0f)
        {
            if (!emergencySignal)
            {
                emergencySignal = 1;
                Log("[ANLY] SIMULATION: Emergency sensor activated (90-95°C range)");
            }
        }
        else
        {
            if (emergencySignal)
            {
                emergencySignal = 0;
                Log("[ANLY] SIMULATION: Emergency sensor deactivated");
            }
        }
        
        /* Prepare temperature data */
        tempData.temperature = simulatedTemp;
        tempData.timestamp = xTaskGetTickCount();
        
        /* Send to ControllerTask */
        if (xQueueSend(xTempQueue, &tempData, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            Log("[ANLY] ERROR: TempQueue full!");
        }
        
        /* Log temperature every 5 seconds */
        if (cycle % 5 == 0)
        {
            char logMsg[LOG_MSG_MAX_LEN];
            snprintf(logMsg, sizeof(logMsg),
                     "[ANLY] Temperature: %.1f C", simulatedTemp);
            Log(logMsg);
        }
        
        cycle++;
        
        /* Wait 1 second between readings */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief  Fan control task - controls green LED based on temperature
 * @param  pvParameters: pointer to task parameters
 * @retval None
 * @note   Green LED: OFF in IDLE/MONITORING, blinks in COOLING (20-80°C proportional),
 *         solid in ALARM (≥80°C)
 */
static void FanControlTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t blinkInterval;
    uint32_t lastToggleTime = 0;
    
    const uint32_t MIN_BLINK_INTERVAL_MS = 100;  /* Fastest blink at 80°C */
    const uint32_t MAX_BLINK_INTERVAL_MS = 2000; /* Slowest blink at 20°C */
    
    Log("[FAN] FanControlTask started");
    
    for (;;)
    {
        float temp = currentTemperature;
        SystemState_t state = currentState;
        
        if (state == STATE_IDLE || state == STATE_MONITORING)
        {
            /* No cooling needed - fan off, LED off */
            LED_Off(LED_GREEN);
        }
        else if (state == STATE_COOLING)
        {
            /* Active cooling - blink rate proportional to temperature (20-80°C) */
            if (temp < TEMP_WARNING_MAX) temp = TEMP_WARNING_MAX; /* Clamp to 20°C minimum */
            if (temp > TEMP_CRITICAL) temp = TEMP_CRITICAL;       /* Clamp to 80°C maximum */
            
            /* Linear mapping: 20°C -> 2000ms, 80°C -> 100ms */
            float tempRange = TEMP_CRITICAL - TEMP_WARNING_MAX;  /* 60°C range */
            float tempOffset = temp - TEMP_WARNING_MAX;
            blinkInterval = MAX_BLINK_INTERVAL_MS -
                           (uint32_t)((tempOffset / tempRange) *
                           (MAX_BLINK_INTERVAL_MS - MIN_BLINK_INTERVAL_MS));
            
            /* Toggle LED at calculated interval */
            uint32_t currentTime = xTaskGetTickCount();
            if ((currentTime - lastToggleTime) >= blinkInterval)
            {
                LED_Toggle(LED_GREEN);
                lastToggleTime = currentTime;
            }
        }
        else if (state == STATE_ALARM)
        {
            /* Critical temperature - maximum fan speed, LED solid */
            LED_On(LED_GREEN);
        }
        
        /* Check every 50ms */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief  Logger task - centralized logging via UART
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
static void LoggerTask(void *pvParameters)
{
    (void)pvParameters;
    char logMsg[LOG_MSG_MAX_LEN];
    
    /* Send startup banner directly (before queue is active) */
    UART_SendString("\r\n=== STM32-RTcore: Smart Cooling Controller ===\r\n");
    UART_SendString("System starting... Phase 2 - State Machine\r\n\r\n");
    
    for (;;)
    {
        /* Wait for log messages from queue */
        if (xQueueReceive(xLogQueue, logMsg, portMAX_DELAY) == pdTRUE)
        {
            /* Send timestamp */
            uint32_t timestamp_ms = xTaskGetTickCount();
            char timeBuf[16];
            snprintf(timeBuf, sizeof(timeBuf), "[%lu ms] ", timestamp_ms);
            UART_SendString(timeBuf);
            
            /* Send log message */
            UART_SendString(logMsg);
            UART_SendString("\r\n");
        }
    }
}

/**
 * @brief  Blue LED control task - controls blue LED based on system state
 * @param  pvParameters: pointer to task parameters
 * @retval None
 * @note   Blue LED patterns:
 *         IDLE: Blink (1500ms interval)
 *         MONITORING: Blink (500ms interval)
 *         COOLING: Solid ON
 *         ALARM: Double-pulse pattern (OFF 1s, ON 150ms, OFF 150ms, ON 150ms, repeat)
 */
static void BlueLEDControlTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t lastTime = 0;
    uint8_t alarmPhase = 0;  /* For ALARM double-pulse pattern */
    uint8_t ledState = 0;    /* Track LED state for blinking */
    
    Log("[BLUE] BlueLEDControlTask started");
    
    /* Initialize LED to OFF */
    LED_Off(LED_BLUE);
    lastTime = xTaskGetTickCount();
    
    for (;;)
    {
        SystemState_t state = currentState;
        uint32_t currentTime = xTaskGetTickCount();
        
        switch (state)
        {
            case STATE_IDLE:
                /* Blink - 1500ms interval */
                if ((currentTime - lastTime) >= 1500)
                {
                    ledState = !ledState;
                    if (ledState)
                        LED_On(LED_BLUE);
                    else
                        LED_Off(LED_BLUE);
                    lastTime = currentTime;
                }
                alarmPhase = 0;  /* Reset alarm pattern */
                break;
                
            case STATE_MONITORING:
                /* Blink - 500ms interval */
                if ((currentTime - lastTime) >= 500)
                {
                    ledState = !ledState;
                    if (ledState)
                        LED_On(LED_BLUE);
                    else
                        LED_Off(LED_BLUE);
                    lastTime = currentTime;
                }
                alarmPhase = 0;  /* Reset alarm pattern */
                break;
                
            case STATE_COOLING:
                /* Solid ON */
                LED_On(LED_BLUE);
                ledState = 1;
                lastTime = currentTime;  /* Reset timer */
                alarmPhase = 0;  /* Reset alarm pattern */
                break;
                
            case STATE_ALARM:
                /* Double-pulse pattern: OFF 1000ms -> ON 150ms -> OFF 150ms -> ON 150ms -> repeat */
                switch (alarmPhase)
                {
                    case 0:  /* OFF for 1000ms */
                        LED_Off(LED_BLUE);
                        if ((currentTime - lastTime) >= 1000)
                        {
                            alarmPhase = 1;
                            lastTime = currentTime;
                        }
                        break;
                        
                    case 1:  /* First pulse ON for 150ms */
                        LED_On(LED_BLUE);
                        if ((currentTime - lastTime) >= 150)
                        {
                            alarmPhase = 2;
                            lastTime = currentTime;
                        }
                        break;
                        
                    case 2:  /* OFF for 150ms */
                        LED_Off(LED_BLUE);
                        if ((currentTime - lastTime) >= 150)
                        {
                            alarmPhase = 3;
                            lastTime = currentTime;
                        }
                        break;
                        
                    case 3:  /* Second pulse ON for 150ms */
                        LED_On(LED_BLUE);
                        if ((currentTime - lastTime) >= 150)
                        {
                            alarmPhase = 0;  /* Reset to start */
                            lastTime = currentTime;
                        }
                        break;
                        
                    default:
                        alarmPhase = 0;
                        break;
                }
                break;
                
            default:
                break;
        }
        
        /* Check every 50ms */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief  Turn LED on
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
static void LED_On(uint8_t pin)
{
    GPIOB_ODR |= (1 << pin);
}

/**
 * @brief  Turn LED off
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
static void LED_Off(uint8_t pin)
{
    GPIOB_ODR &= ~(1 << pin);
}

/**
 * @brief  Toggle LED
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
static void LED_Toggle(uint8_t pin)
{
    GPIOB_ODR ^= (1 << pin);
}

/**
 * @brief  Set LED states based on system state
 * @param  state: Current system state
 * @retval None
 * @note   Only controls Red LED here. Green LED controlled by FanControlTask,
 *         Blue LED controlled by BlueLEDControlTask.
 *         Red LED represents EMERGENCY SENSOR status, not alarm state!
 */
static void LED_SetState(SystemState_t state)
{
    /* Red LED: ON only when emergency signal is active (external sensor triggered) */
    if (emergencySignal)
    {
        LED_On(LED_RED);
    }
    else
    {
        LED_Off(LED_RED);
    }
}

/**
 * @brief  Send log message to LoggerTask via queue
 * @param  message: Log message string (max LOG_MSG_MAX_LEN)
 * @retval None
 */
static void Log(const char *message)
{
    char logMsg[LOG_MSG_MAX_LEN];
    
    /* Copy message to local buffer */
    strncpy(logMsg, message, LOG_MSG_MAX_LEN - 1);
    logMsg[LOG_MSG_MAX_LEN - 1] = '\0';
    
    /* Send to logger queue (non-blocking) */
    if (xLogQueue != NULL)
    {
        xQueueSend(xLogQueue, logMsg, 0);
    }
}

/**
 * @brief  Initialize GPIO for LEDs
 * @retval None
 */
static void GPIO_Init(void)
{
    /* Enable clock for GPIOB */
    RCC_AHB1ENR |= GPIOB_EN;

    /* Set PB0, PB7, PB14 as outputs (MODER: 01 = output mode) */
    GPIOB_MODER &= ~((3 << (LED_GREEN * 2)) | (3 << (LED_BLUE * 2)) | (3 << (LED_RED * 2)));
    GPIOB_MODER |= ((1 << (LED_GREEN * 2)) | (1 << (LED_BLUE * 2)) | (1 << (LED_RED * 2)));
    
    /* Turn off all LEDs initially */
    LED_Off(LED_GREEN);
    LED_Off(LED_BLUE);
    LED_Off(LED_RED);
}

/**
 * @brief  Initialize USART3 (115200 baud, 8N1)
 * @note   USART3: PD8 = TX, PD9 = RX (Nucleo-F767ZI virtual COM port)
 * @retval None
 */
static void UART_Init(void)
{
    /* Enable clocks for GPIOD and USART3 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    
    /* Configure PD8 (TX) and PD9 (RX) as alternate function */
    /* MODER: 10 = Alternate function mode */
    GPIOD->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOD->MODER |= ((2 << (8 * 2)) | (2 << (9 * 2)));
    
    /* Set output type to push-pull (default, but explicit) */
    GPIOD->OTYPER &= ~((1 << 8) | (1 << 9));
    
    /* Set high speed */
    GPIOD->OSPEEDR |= ((3 << (8 * 2)) | (3 << (9 * 2)));
    
    /* No pull-up/pull-down */
    GPIOD->PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    
    /* Set alternate function AF7 (USART3) for PD8 and PD9 */
    /* AFR[1] is for pins 8-15 */
    GPIOD->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOD->AFR[1] |= ((7 << ((8 - 8) * 4)) | (7 << ((9 - 8) * 4)));
    
    /* Configure USART3 */
    /* Baud rate = fCK / USARTDIV, fCK = 16 MHz (HSI), target = 115200 */
    /* USARTDIV = 16000000 / 115200 = 138.888... ≈ 139 (0x8B) */
    USART3->BRR = 139;
    
    /* Enable USART, transmitter */
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE;
    
    /* Wait for USART to be ready (small delay) */
    for (volatile int i = 0; i < 10000; i++);
}

/**
 * @brief  Send one character via UART (polling)
 * @param  c: character to send
 * @retval None
 */
static void UART_SendChar(char c)
{
    /* Wait until transmit data register is empty */
    while (!(USART3->ISR & USART_ISR_TXE));
    
    /* Write character to transmit data register */
    USART3->TDR = c;
}

/**
 * @brief  Send string via UART
 * @param  str: null-terminated string
 * @retval None
 */
static void UART_SendString(const char *str)
{
    while (*str)
    {
        UART_SendChar(*str++);
    }
}

/**
 * @brief  Main program
 * @retval int
 */
int main(void)
{
    /* Initialize System */
    SystemInit();
    
    /* Initialize GPIO */
    GPIO_Init();
    
    /* Initialize UART */
    UART_Init();
    
    /* Create queues */
    xTempQueue = xQueueCreate(TEMP_QUEUE_SIZE, sizeof(TempData_t));
    xLogQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(char) * LOG_MSG_MAX_LEN);
    
    if (xTempQueue == NULL || xLogQueue == NULL)
    {
        /* Queue creation failed - halt */
        UART_SendString("ERROR: Queue creation failed!\r\n");
        for (;;);
    }
    
    /* Create LoggerTask (highest priority - handles all logging) */
    xTaskCreate(
        LoggerTask,
        "Logger",
        256,                   /* Stack size (words) */
        NULL,
        tskIDLE_PRIORITY + 3,  /* High priority */
        NULL
    );
    
    /* Create ControllerTask (state machine) */
    xTaskCreate(
        ControllerTask,
        "Controller",
        256,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );
    
    /* Create AnalysisTask (temperature simulation) */
    xTaskCreate(
        AnalysisTask,
        "Analysis",
        256,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    
    /* Create FanControlTask (green LED control based on temperature) */
    xTaskCreate(
        FanControlTask,
        "FanControl",
        128,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );
    
    /* Create BlueLEDControlTask (blue LED control based on state) */
    xTaskCreate(
        BlueLEDControlTask,
        "BlueLED",
        128,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL
    );

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;);
}
