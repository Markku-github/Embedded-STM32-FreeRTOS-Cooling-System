/**
 ******************************************************************************
 * @file    tasks.h
 * @brief   FreeRTOS task definitions for temperature control
 ******************************************************************************
 */

#ifndef TASKS_H
#define TASKS_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>
#include "config.h"  /* For system constants */

/* Temperature data structure */
typedef struct {
    float temperature;   /* Temperature in Celsius */
    uint32_t timestamp;  /* Timestamp in ms */
} TempData_t;

/* External queue handles - defined in main.c */
extern QueueHandle_t xTempQueue;

/* External mutex handles - defined in main.c */
extern SemaphoreHandle_t xTempMutex;
extern SemaphoreHandle_t xEmergencyMutex;

/**
 * @brief  Controller task - main state machine
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void ControllerTask(void *pvParameters);

/**
 * @brief  Analysis task - temperature monitoring
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void AnalysisTask(void *pvParameters);

#endif /* TASKS_H */
