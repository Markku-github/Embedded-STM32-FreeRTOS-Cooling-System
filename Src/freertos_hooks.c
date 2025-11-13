/**
 ******************************************************************************
 * @file    freertos_hooks.c
 * @brief   FreeRTOS hook functions implementation
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "stm32f7xx.h"

/**
 * @brief  Application stack overflow hook (if enabled in FreeRTOSConfig.h)
 * @param  xTask: Task handle
 * @param  pcTaskName: Task name
 * @retval None
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    
    /* Stack overflow detected - halt system */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/**
 * @brief  Application malloc failed hook (if enabled in FreeRTOSConfig.h)
 * @retval None
 */
void vApplicationMallocFailedHook(void)
{
    /* Malloc failed - halt system */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

/**
 * @brief  Application idle hook (if enabled in FreeRTOSConfig.h)
 * @retval None
 * @note   Watchdog is now fed by WatchdogTask based on task heartbeats
 */
void vApplicationIdleHook(void)
{
    /* Idle hook - watchdog feeding moved to WatchdogTask for better monitoring */
}

/**
 * @brief  Application tick hook (if enabled in FreeRTOSConfig.h)
 * @retval None
 */
void vApplicationTickHook(void)
{
    /* Optional: Add periodic tasks here */
}
