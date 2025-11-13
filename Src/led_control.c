/**
 ******************************************************************************
 * @file    led_control.c
 * @brief   LED control implementation
 ******************************************************************************
 */

/* Own header */
#include "led_control.h"

/* MCU headers */
#include "stm32f7xx.h"

/* Project headers */
#include "config.h"
#include "globals.h"
#include "logger.h"
#include "led_patterns.h"

/* External queue handle (defined in main.c) */
extern QueueHandle_t xLogQueue;

/* Register definitions */
#define GPIOB_BASE      0x40020400
#define GPIOC_BASE      0x40020800
#define RCC_AHB1ENR     (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_ODR       (*(volatile uint32_t*)(GPIOB_BASE + 0x14))
#define GPIOC_MODER     (*(volatile uint32_t*)(GPIOC_BASE + 0x00))
#define GPIOC_PUPDR     (*(volatile uint32_t*)(GPIOC_BASE + 0x0C))
#define GPIOC_IDR       (*(volatile uint32_t*)(GPIOC_BASE + 0x10))
#define GPIOB_EN        (1 << 1)   /* Bit 1 for GPIOB */
#define GPIOC_EN        (1 << 2)   /* Bit 2 for GPIOC */

/* EXTI and SYSCFG registers */
#define SYSCFG_EXTICR4  (*(volatile uint32_t*)(0x40013800 + 0x14))
#define EXTI_IMR        (*(volatile uint32_t*)(0x40013C00 + 0x00))
#define EXTI_RTSR       (*(volatile uint32_t*)(0x40013C00 + 0x08))
#define EXTI_FTSR       (*(volatile uint32_t*)(0x40013C00 + 0x0C))
#define EXTI_PR         (*(volatile uint32_t*)(0x40013C00 + 0x14))
#define RCC_APB2ENR     (*(volatile uint32_t*)(RCC_BASE + 0x44))
#define SYSCFG_EN       (1 << 14)  /* Bit 14 for SYSCFG */

/**
 * @brief  Initialize GPIO for LEDs
 * @retval None
 */
void LED_Init(void)
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

/* Button_Init() has been moved to Src/button_test.c */

// Poistettu vanha EXTI15_10 IRQ handler (#if 0 -lohko), ei käytössä. Katso git historia tarvittaessa.

/**
 * @brief  Turn LED on
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
void LED_On(uint8_t pin)
{
    GPIOB_ODR |= (1 << pin);
}

/**
 * @brief  Turn LED off
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
void LED_Off(uint8_t pin)
{
    GPIOB_ODR &= ~(1 << pin);
}

/**
 * @brief  Toggle LED
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
void LED_Toggle(uint8_t pin)
{
    GPIOB_ODR ^= (1 << pin);
}

/**
 * @brief  Set LED states based on system state
 * @param  state: Current system state
 * @retval None
 * @note   Only controls Red LED here. Green LED controlled by FanControlTask,
 *         Blue LED controlled by BlueLEDControlTask.
 *         Red LED represents EMERGENCY SENSOR status.
 */
void LED_SetState(SystemState_t state)
{
    (void)state;  /* State not used, only emergency signal */
    
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
 * @brief  Fan control task - controls green LED blink rate based on temperature
 * @param  pvParameters: pointer to task parameters
 * @retval None
 * @note   Green LED: OFF in IDLE/MONITORING, blinks in COOLING (proportional),
 *         solid in CRITICAL/ALARM
 */
void FanControlTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t lastToggleTime = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  /* 50ms period */
    
    Log("[FAN] FanControlTask started");
    
    /* Initialize xLastWakeTime with current time */
    xLastWakeTime = xTaskGetTickCount();
    
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
            /* Active cooling - blink rate proportional to temperature */
            uint32_t blinkInterval = LED_Pattern_FanBlinkInterval(temp);
            
            /* Toggle LED at calculated interval */
            uint32_t currentTime = xTaskGetTickCount();
            if ((currentTime - lastToggleTime) >= blinkInterval)
            {
                LED_Toggle(LED_GREEN);
                lastToggleTime = currentTime;
            }
        }
        else if (state == STATE_CRITICAL || state == STATE_ALARM)
        {
            /* CRITICAL or ALARM - maximum fan speed, LED solid */
            LED_On(LED_GREEN);
        }
        
        /* Wait until next cycle (50ms period, no drift) */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
 *         CRITICAL: Double-pulse pattern
 *         ALARM: SOS Morse code
 */
void BlueLEDControlTask(void *pvParameters)
{
    (void)pvParameters;
    uint32_t lastTime = 0;
    uint8_t phase = 0;       /* Pattern phase counter */
    uint8_t ledState = 0;    /* Current LED state */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  /* 50ms period */
    
    Log("[BLUE] BlueLEDControlTask started");
    
    /* Initialize LED to OFF */
    LED_Off(LED_BLUE);
    lastTime = xTaskGetTickCount();
    
    /* Initialize xLastWakeTime with current time */
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        SystemState_t state = currentState;
        uint32_t currentTime = xTaskGetTickCount();
        
        switch (state)
        {
            case STATE_IDLE:
                /* Blink - 1500ms interval */
                ledState = LED_Pattern_Blink(currentTime, &lastTime, &ledState, IDLE_BLINK_INTERVAL);
                phase = 0;  /* Reset pattern phase */
                break;
                
            case STATE_MONITORING:
                /* Blink - 500ms interval */
                ledState = LED_Pattern_Blink(currentTime, &lastTime, &ledState, MONITORING_BLINK_INTERVAL);
                phase = 0;  /* Reset pattern phase */
                break;
                
            case STATE_COOLING:
                /* Solid ON */
                ledState = 1;
                lastTime = currentTime;
                phase = 0;  /* Reset pattern phase */
                break;
                
            case STATE_CRITICAL:
                /* Double-pulse pattern */
                ledState = LED_Pattern_DoublePulse(currentTime, &lastTime, &phase);
                break;
                
            case STATE_ALARM:
                /* SOS Morse code pattern */
                ledState = LED_Pattern_SOS(currentTime, &lastTime, &phase);
                break;
                
            default:
                break;
        }
        
        /* Apply LED state */
        if (ledState)
            LED_On(LED_BLUE);
        else
            LED_Off(LED_BLUE);
        
        /* Wait until next cycle (50ms period, no drift) */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
