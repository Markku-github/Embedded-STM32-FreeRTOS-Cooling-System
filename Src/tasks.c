/**
 ******************************************************************************
 * @file    tasks.c
 * @brief   FreeRTOS task implementations for temperature control
 ******************************************************************************
 */

/* Own header */
#include "tasks.h"

/* Project headers */
#include "globals.h"
#include "led_control.h"
#include "logger.h"
#include "utils.h"
#include "watchdog.h"
#include "wcet.h"

/**
 * @brief  Controller task - implements state machine
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void ControllerTask(void *pvParameters)
{
    (void)pvParameters;
    TempData_t tempData;
    SystemState_t newState;
    uint32_t loopCount = 0;
    
    Log("[CTRL] ControllerTask started");
    
    /* Start in IDLE state */
    currentState = STATE_IDLE;
    LED_SetState(currentState);
    Log("[CTRL] State: IDLE (system ready)");
    
    for (;;)
    {
        uint32_t wcet_start = WCET_Start();
        
        /* Wait for temperature data from AnalysisTask */
        BaseType_t queueResult = xQueueReceive(xTempQueue, &tempData, pdMS_TO_TICKS(100));
        
        uint8_t emergency;
        
        /* Read emergency signal with mutex protection */
        if (xSemaphoreTake(xEmergencyMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
        {
            emergency = emergencySignal;
            xSemaphoreGive(xEmergencyMutex);
        }
        else
        {
            emergency = 0;
            Log("[CTRL] WARNING: Emergency mutex timeout");
        }
        
        /* If queue receive failed (timeout), check emergency anyway */
        if (queueResult != pdTRUE)
        {
            /* No new temperature data, but check if emergency changed */
            if (emergency && currentState != STATE_ALARM)
            {
                newState = STATE_ALARM;
                
                /* Update state immediately */
                currentState = newState;
                LED_SetState(currentState);
                
                char logMsg[LOG_MSG_MAX_LEN];
                str_copy(logMsg, "[CTRL] EMERGENCY! State: ALARM", LOG_MSG_MAX_LEN);
                Log(logMsg);
            }
            else if (!emergency && currentState == STATE_ALARM)
            {
                /* Emergency cleared - return to temperature-based state */
                float temp;
                if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
                {
                    temp = currentTemperature;
                    xSemaphoreGive(xTempMutex);
                }
                else
                {
                    temp = 0.0f;
                    Log("[CTRL] WARNING: Temp mutex timeout");
                }
                
                /* Determine new state based on temperature */
                if (temp >= 80.0f)
                    newState = STATE_CRITICAL;
                else if (temp >= 20.0f)
                    newState = STATE_COOLING;
                else if (temp > 0.0f)
                    newState = STATE_MONITORING;
                else
                    newState = STATE_IDLE;
                
                /* Update state */
                currentState = newState;
                LED_SetState(currentState);
                
                char logMsg[LOG_MSG_MAX_LEN];
                str_copy(logMsg, "[CTRL] Emergency cleared. State: ", LOG_MSG_MAX_LEN);
                str_append(logMsg, SystemState_ToString(currentState), LOG_MSG_MAX_LEN);
                Log(logMsg);
            }
            
            /* No temperature data, continue to next iteration */
            continue;
        }
        
        /* We have new temperature data, proceed with normal logic */
        {
            
            /* State determination logic:
             * 1. ALARM state: Only from emergency signal (button/command)
             * 2. IDLE state: Startup or after reset command
             * 3. MONITORING/COOLING/CRITICAL: Based on temperature
             */
            
            if (emergency)
            {
                /* Emergency signal detected - enter ALARM state */
                newState = STATE_ALARM;
            }
            else
            {
                /* Normal temperature-based state transitions */
                /* Note: Temperature 0 means "not set yet", stay in IDLE */
                if (tempData.temperature >= 80.0f)
                {
                    newState = STATE_CRITICAL;
                }
                else if (tempData.temperature >= 20.0f)
                {
                    newState = STATE_COOLING;
                }
                else if (tempData.temperature > 0.0f)
                {
                    newState = STATE_MONITORING;
                }
                else
                {
                    /* Temperature is 0 (not set) - stay in IDLE */
                    newState = STATE_IDLE;
                }
            }
            
            /* State transition */
            if (newState != currentState)
            {
                char logMsg[LOG_MSG_MAX_LEN];
                char tempStr[16];
                
                /* Build log message manually without snprintf */
                str_copy(logMsg, "[CTRL] State: ", LOG_MSG_MAX_LEN);
                str_append(logMsg, SystemState_ToString(currentState), LOG_MSG_MAX_LEN);
                str_append(logMsg, " -> ", LOG_MSG_MAX_LEN);
                str_append(logMsg, SystemState_ToString(newState), LOG_MSG_MAX_LEN);
                
                /* Log different message if emergency-triggered */
                if (emergencySignal && newState == STATE_ALARM)
                {
                    str_append(logMsg, " (EMERGENCY SIGNAL!)", LOG_MSG_MAX_LEN);
                }
                else
                {
                    str_append(logMsg, " (Temp: ", LOG_MSG_MAX_LEN);
                    int_to_str((int)tempData.temperature, tempStr);
                    str_append(logMsg, tempStr, LOG_MSG_MAX_LEN);
                    str_append(logMsg, "C)", LOG_MSG_MAX_LEN);
                }
                Log(logMsg);
                
                currentState = newState;
                LED_SetState(currentState);
            }
            
            /* Always update LED to reflect current emergency signal state */
            LED_SetState(currentState);
        }  /* End of "We have new temperature data" block */
        
        /* Stack diagnostics every ~10 seconds (100ms queue timeout * 100 loops) */
        if (loopCount % 100 == 0 && loopCount > 0)
        {
            UBaseType_t stackHWM = uxTaskGetStackHighWaterMark(NULL);
            if (stackHWM < 32)  /* Less than 128 bytes free (32 words * 4 bytes) */
            {
                char logMsg[LOG_MSG_MAX_LEN];
                char stackStr[16];
                str_copy(logMsg, "[WARN] ControllerTask low stack! HWM=", LOG_MSG_MAX_LEN);
                int_to_str(stackHWM, stackStr);
                str_append(logMsg, stackStr, LOG_MSG_MAX_LEN);
                str_append(logMsg, " words", LOG_MSG_MAX_LEN);
                Log(logMsg);
            }
        }
        
        loopCount++;
        
        /* Record task execution time (before delay) */
        WCET_StopAndRecord("Controller", wcet_start);
        
        /* Report heartbeat to watchdog */
        Watchdog_ReportHeartbeat(WATCHDOG_TASK_CONTROLLER);
        
        /* Delay is not measured */
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief  Analysis task - temperature monitoring
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void AnalysisTask(void *pvParameters)
{
    (void)pvParameters;
    TempData_t tempData;
    uint32_t loopCount = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);  /* 1 second period */
    
    Log("[ANLY] AnalysisTask started");
    
    /* Initialize xLastWakeTime with current time */
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        uint32_t wcet_start = WCET_Start();
        
        /* Use current temperature set via UART command */
        /* No simulation - temperature is controlled manually via 'temp' command */
        
        /* Prepare temperature data with mutex protection */
        if (xSemaphoreTake(xTempMutex, pdMS_TO_TICKS(MUTEX_TIMEOUT_MS)) == pdTRUE)
        {
            tempData.temperature = currentTemperature;
            xSemaphoreGive(xTempMutex);
        }
        else
        {
            tempData.temperature = 0.0f;
            Log("[ANLY] WARNING: Temp mutex timeout");
        }
        tempData.timestamp = xTaskGetTickCount();
        
        /* Send to ControllerTask */
        if (xQueueSend(xTempQueue, &tempData, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            Log("[ANLY] ERROR: TempQueue full!");
        }
        
        /* Log temperature every 5 seconds */
        if (loopCount % 5 == 0)
        {
            char logMsg[LOG_MSG_MAX_LEN];
            char tempStr[16];
            
            /* Build message manually */
            str_copy(logMsg, "[ANLY] Temperature: ", LOG_MSG_MAX_LEN);
            int_to_str((int)currentTemperature, tempStr);
            str_append(logMsg, tempStr, LOG_MSG_MAX_LEN);
            str_append(logMsg, "C", LOG_MSG_MAX_LEN);
            Log(logMsg);
        }
        
        /* Stack diagnostics every 10 seconds */
        if (loopCount % 10 == 0)
        {
            UBaseType_t stackHWM = uxTaskGetStackHighWaterMark(NULL);
            if (stackHWM < 32)  /* Less than 128 bytes free (32 words * 4 bytes) */
            {
                char logMsg[LOG_MSG_MAX_LEN];
                char stackStr[16];
                str_copy(logMsg, "[WARN] AnalysisTask low stack! HWM=", LOG_MSG_MAX_LEN);
                int_to_str(stackHWM, stackStr);
                str_append(logMsg, stackStr, LOG_MSG_MAX_LEN);
                str_append(logMsg, " words", LOG_MSG_MAX_LEN);
                Log(logMsg);
            }
        }
        
        loopCount++;
        
        /* Record task execution time */
        WCET_StopAndRecord("Analysis", wcet_start);
        
        /* Report heartbeat to watchdog */
        Watchdog_ReportHeartbeat(WATCHDOG_TASK_ANALYSIS);
        
        /* Wait until next cycle (no drift accumulation) */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
