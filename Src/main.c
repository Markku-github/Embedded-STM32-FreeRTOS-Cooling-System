/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Markku Kirjava
 * @brief          : STM32F767ZI FreeRTOS LED Blink Demo
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
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f7xx.h"

// Register definitions for STM32F767ZI
#define GPIOB_BASE      0x40020400

#define RCC_AHB1ENR     (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_ODR       (*(volatile uint32_t*)(GPIOB_BASE + 0x14))

#define GPIOB_EN        (1 << 1)  // Bit 1 for GPIOB
#define PB7_MODER       (1 << 14)  // Pin 7, output mode

/* Function prototypes */
static void LED_Task(void *pvParameters);
static void GPIO_Init(void);

/**
 * @brief  LED blink task
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
static void LED_Task(void *pvParameters)
{
    (void)pvParameters; /* Unused parameter */

    for (;;)
    {
        /* Toggle LED on PB7 */
        GPIOB_ODR ^= (1 << 7);
        
        /* Delay for 500ms using FreeRTOS */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief  Initialize GPIO for LED
 * @retval None
 */
static void GPIO_Init(void)
{
    /* Enable clock for GPIOB */
    RCC_AHB1ENR |= GPIOB_EN;

    /* Set PB7 as output */
    GPIOB_MODER |= PB7_MODER;
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

    /* Create LED task */
    xTaskCreate(
        LED_Task,              /* Task function */
        "LED_Task",            /* Task name */
        128,                   /* Stack size (words) */
        NULL,                  /* Task parameters */
        tskIDLE_PRIORITY + 1,  /* Priority */
        NULL                   /* Task handle */
    );

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;);
}
