/**
 ******************************************************************************
 * @file    user_button.c
 * @brief   USER button interrupt-based task (EXTI on PC13)
 * @note    Uses EXTI13 interrupt to detect USER button press
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stm32f7xx.h"
#include "logger.h"
#include "globals.h"
#include "tasks.h"
#include "config.h"
#include "wcet.h"

/* GPIO and peripheral base addresses */
#define GPIOC_BASE      0x40020800UL
#define SYSCFG_BASE     0x40013800UL
#define EXTI_BASE       0x40013C00UL

/* RCC registers */
#define RCC_AHB1ENR     (*(volatile uint32_t*)(0x40023800UL + 0x30))
#define RCC_APB2ENR     (*(volatile uint32_t*)(0x40023800UL + 0x44))
#define GPIOC_EN        (1 << 2)   /* Bit 2 for GPIOC */
#define SYSCFG_EN       (1 << 14)  /* Bit 14 for SYSCFG */

/* GPIOC registers */
#define GPIOC_MODER     (*(volatile uint32_t*)(GPIOC_BASE + 0x00))
#define GPIOC_PUPDR     (*(volatile uint32_t*)(GPIOC_BASE + 0x0C))
#define GPIOC_IDR       (*(volatile uint32_t*)(GPIOC_BASE + 0x10))

/* SYSCFG registers (for EXTI configuration) */
#define SYSCFG_EXTICR4  (*(volatile uint32_t*)(SYSCFG_BASE + 0x14))

/* EXTI registers */
#define EXTI_IMR        (*(volatile uint32_t*)(EXTI_BASE + 0x00))
#define EXTI_RTSR       (*(volatile uint32_t*)(EXTI_BASE + 0x08))
#define EXTI_FTSR       (*(volatile uint32_t*)(EXTI_BASE + 0x0C))
#define EXTI_PR         (*(volatile uint32_t*)(EXTI_BASE + 0x14))

/* USER button pin */
#define USER_BUTTON_PIN  13

/* Task handle for ISR notification */
static TaskHandle_t xButtonTaskHandle = NULL;

/**
 * @brief  Initialize USER button (PC13) with EXTI interrupt
 * @retval None
 */
void UserButton_Init(void)
{
    /* Enable clocks for GPIOC and SYSCFG */
    RCC_AHB1ENR |= GPIOC_EN;
    RCC_APB2ENR |= SYSCFG_EN;

    /* Configure PC13 as input (MODER = 00) */
    GPIOC_MODER &= ~(3 << (USER_BUTTON_PIN * 2));

    /* No pull-up/pull-down (external pull resistor on Nucleo board) */
    GPIOC_PUPDR &= ~(3 << (USER_BUTTON_PIN * 2));

    /* Connect EXTI13 to PC13 via SYSCFG_EXTICR4 (bits 7:4 = 0010 for port C) */
    SYSCFG_EXTICR4 &= ~(0xF << 4);  /* Clear bits 7:4 */
    SYSCFG_EXTICR4 |= (2 << 4);     /* Set to 0010 (GPIOC) */

    /* Configure EXTI13: rising edge trigger (button press = LOW->HIGH) */
    EXTI_RTSR |= (1 << USER_BUTTON_PIN);
    EXTI_FTSR &= ~(1 << USER_BUTTON_PIN);  /* Disable falling edge */

    /* Enable EXTI13 interrupt */
    EXTI_IMR |= (1 << USER_BUTTON_PIN);

    /* Enable EXTI15_10 interrupt in NVIC (IRQ 40) */
    NVIC_SetPriority(EXTI15_10_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief  Button interrupt-based task
 * @note   Waits for notification from EXTI ISR, then applies software debounce
 */
void UserButtonTask(void *pvParameters)
{
    (void)pvParameters;
    
    /* Store task handle for ISR notification */
    xButtonTaskHandle = xTaskGetCurrentTaskHandle();
    
    Log("[USER_BUTTON] Button task started (EXTI13 interrupt-based)");
    
    for (;;)
    {
        uint32_t wcet_start = WCET_Start();
        
        /* Wait for notification from ISR (blocks indefinitely) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        /* Software debounce: wait for signal to stabilize */
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_DELAY_MS));
        
        /* Read button state after debounce */
        uint8_t buttonState = (GPIOC_IDR >> USER_BUTTON_PIN) & 1;
        
        if (buttonState == 1)
        {
            /* Button still pressed after debounce - trigger emergency */
            Log("[USER_BUTTON] Button PRESSED (B1)! Triggering emergency...");
            
            /* Set emergency signal (protected by mutex) */
            if (xSemaphoreTake(xEmergencyMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
            {
                emergencySignal = 1;
                xSemaphoreGive(xEmergencyMutex);
            }
            else
            {
                Log("[USER_BUTTON] WARNING: Emergency mutex timeout");
            }
        }
        else
        {
            /* False trigger or too short press - ignore */
            Log("[USER_BUTTON] Button press too short or noise - ignored");
        }
        
        /* Record task execution time */
        WCET_StopAndRecord("UserButton", wcet_start);
    }
}

/**
 * @brief  EXTI15_10 interrupt handler (handles EXTI line 13 for PC13)
 * @note   ISR-safe implementation:
 *         - Uses vTaskNotifyGiveFromISR (ISR-safe API)
 *         - Calls portYIELD_FROM_ISR to request context switch
 *         - Priority set to configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1
 *         - Minimal processing (clear flag, notify task)
 * @retval None
 */
void EXTI15_10_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    /* Check if EXTI13 triggered this interrupt */
    if (EXTI_PR & (1 << USER_BUTTON_PIN))
    {
        /* Clear pending bit by writing 1 */
        EXTI_PR = (1 << USER_BUTTON_PIN);
        
        /* Notify button task from ISR (ISR-safe API) */
        if (xButtonTaskHandle != NULL)
        {
            vTaskNotifyGiveFromISR(xButtonTaskHandle, &xHigherPriorityTaskWoken);
        }
    }
    
    /* Request context switch if higher priority task was woken */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
