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
#include <string.h>
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
static void UART_Init(void);
static void UART_SendChar(char c);
static void UART_SendString(const char *str);
static void LED_On(uint8_t pin);
static void LED_Off(uint8_t pin);
static void LED_Toggle(uint8_t pin);

/**
 * @brief  LED blink task (test all three LEDs with UART logging)
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
        
        /* Send log message via UART with timestamp */
        UART_SendString("[");
        
        /* Get current tick count and convert to milliseconds */
        uint32_t timestamp_ms = xTaskGetTickCount();
        
        /* Simple integer to string conversion */
        char buf[16];
        int i = 0;
        if (timestamp_ms == 0) {
            buf[i++] = '0';
        } else {
            char rev[16];
            int j = 0;
            uint32_t temp = timestamp_ms;
            while (temp > 0) {
                rev[j++] = '0' + (temp % 10);
                temp /= 10;
            }
            while (j > 0) {
                buf[i++] = rev[--j];
            }
        }
        buf[i] = '\0';
        
        UART_SendString(buf);
        UART_SendString(" ms] LEDs toggled\r\n");
        
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
    
    /* Send startup message */
    UART_SendString("\r\n=== STM32-RTcore: Smart Cooling Controller ===\r\n");
    UART_SendString("System starting...\r\n\r\n");

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
