/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Markku Kirjava
 * @brief          : STM32F767ZI FreeRTOS Smart Cooling Controller
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

#define GPIOB_EN        (1 << 1)   // Bit 1 for GPIOB

// LED pin definitions (Nucleo-F767ZI)
#define LED_GREEN       0   // LD1 on PB0 (Fan speed indicator)
#define LED_BLUE        7   // LD2 on PB7 (Alert indicator)
#define LED_RED         14  // LD3 on PB14 (Emergency indicator)

/* Function prototypes */
static void LED_Task(void *pvParameters);
static void GPIO_Init(void);
static void LED_On(uint8_t pin);
static void LED_Off(uint8_t pin);
static void LED_Toggle(uint8_t pin);

/**
 * @brief  LED blink task (test all three LEDs)
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
static void LED_Task(void *pvParameters)
{
    (void)pvParameters; /* Unused parameter */

    for (;;)
    {
        /* Toggle all three LEDs for testing */
        LED_Toggle(LED_GREEN);
        LED_Toggle(LED_BLUE);
        LED_Toggle(LED_RED);
        
        /* Delay for 500ms using FreeRTOS */
        vTaskDelay(pdMS_TO_TICKS(500));
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
