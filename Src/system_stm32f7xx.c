/**
 ******************************************************************************
 * @file    system_stm32f7xx.c
 * @brief   CMSIS Device System Source File for STM32F7xx devices.
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f7xx.h"

/* System clock frequency (Core clock) */
uint32_t SystemCoreClock = 16000000; /* Default 16 MHz HSI */

/**
 * @brief  Update SystemCoreClock variable
 * @param  None
 * @retval None
 */
void SystemCoreClockUpdate(void)
{
    /* For now, keep default HSI frequency */
    SystemCoreClock = 16000000;
}

/**
 * @brief  Setup the microcontroller system
 * @param  None
 * @retval None
 */
void SystemInit(void)
{
    /* FPU settings - Enable FPU for Cortex-M7 */
    /* Enable CP10 and CP11 coprocessors (FPU) - required for FreeRTOS ARM_CM7 port */
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

    /* Reset the RCC clock configuration to the default reset state */
    /* Set HSION bit */
    RCC->CR |= 0x00000001;

    /* Reset CFGR register */
    RCC->CFGR = 0x00000000;
}
