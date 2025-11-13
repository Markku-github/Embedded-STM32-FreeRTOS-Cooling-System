/**
 ******************************************************************************
 * @file    watchdog.h
 * @brief   Independent Watchdog (IWDG) interface with heartbeat monitoring
 ******************************************************************************
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Task identifiers for heartbeat monitoring */
typedef enum {
    WATCHDOG_TASK_CONTROLLER = 0,  /**< Controller task */
    WATCHDOG_TASK_ANALYSIS   = 1,  /**< Analysis task */
    WATCHDOG_TASK_LOGGER     = 2,  /**< Logger task */
    WATCHDOG_TASK_COMMAND    = 3,  /**< Command task */
} WatchdogTaskId_t;

/**
 * @brief Initialize Independent Watchdog with timeout defined in config.h.
 *        Safe to call once during system initialization.
 */
void Watchdog_Init(void);

/**
 * @brief  Watchdog monitoring task - checks task heartbeats and feeds IWDG
 * @param  pvParameters: pointer to task parameters
 * @retval None
 * @note   This task monitors critical tasks and only feeds watchdog if all are alive
 */
void WatchdogTask(void *pvParameters);

/**
 * @brief  Report heartbeat from a task
 * @param  taskId: Task identifier
 * @retval None
 * @note   Thread-safe, can be called from any task
 */
void Watchdog_ReportHeartbeat(WatchdogTaskId_t taskId);

/**
 * @brief  Manually feed the watchdog (low-level)
 * @retval None
 * @note   Only use this if you know what you're doing. Prefer heartbeat model.
 */
void Watchdog_Feed(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
