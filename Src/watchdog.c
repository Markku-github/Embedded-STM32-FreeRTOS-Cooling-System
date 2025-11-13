/**
 ******************************************************************************
 * @file    watchdog.c
 * @brief   Independent Watchdog (IWDG) implementation with heartbeat monitoring
 ******************************************************************************
 */

#include "stm32f7xx.h"
#include "config.h"
#include "watchdog.h"
#include "FreeRTOS.h"
#include "task.h"
#include "logger.h"

/* Minimal IWDG register map and base, as not defined in local stm32f7xx.h */
typedef struct
{
    volatile uint32_t KR;   /* Key Register */
    volatile uint32_t PR;   /* Prescaler Register */
    volatile uint32_t RLR;  /* Reload Register */
    volatile uint32_t SR;   /* Status Register */
    volatile uint32_t WINR; /* Window Register */
} IWDG_TypeDef;

#ifndef IWDG_BASE
#define IWDG_BASE (0x40003000UL)
#endif

#ifndef IWDG
#define IWDG ((IWDG_TypeDef *) IWDG_BASE)
#endif

#if ENABLE_WATCHDOG

/* Heartbeat tracking - timestamp of last heartbeat for each task */
static volatile TickType_t taskHeartbeats[WATCHDOG_NUM_TASKS] = {0};

/* Clamp helper */
static inline uint32_t clamp_u32(uint32_t val, uint32_t min, uint32_t max)
{
    return (val < min) ? min : (val > max) ? max : val;
}

void Watchdog_Init(void)
{
    /* Unlock IWDG registers for write */
    IWDG->KR = 0x5555U;

    /* Prescaler: divide LSI by 64 -> ~500 Hz tick (32k / 64) */
    IWDG->PR = 4U; /* 0: /4, 1: /8, 2: /16, 3: /32, 4: /64, 5: /128, 6: /256 */

    /* Compute reload to achieve ~WATCHDOG_TIMEOUT_MS */
    /* RL = timeout_s * (LSI / prescaler) - 1 */
    const uint32_t prescaler = 64U;
    uint32_t ticks = (uint32_t)((WATCHDOG_TIMEOUT_MS * (uint32_t)LSI_FREQ_HZ) / (1000U * prescaler));
    uint32_t reload = (ticks == 0U) ? 0U : (ticks - 1U);

    /* Clamp to 12-bit range */
    reload = clamp_u32(reload, 0U, 4095U);

    IWDG->RLR = reload;

    /* Reload the counter */
    IWDG->KR = 0xAAAAU;

    /* Start the watchdog */
    IWDG->KR = 0xCCCCU;
}

/**
 * @brief  Manually feed the watchdog (low-level)
 * @retval None
 */
void Watchdog_Feed(void)
{
    IWDG->KR = 0xAAAAU;  /* Reload counter */
}

/**
 * @brief  Report heartbeat from a task
 * @param  taskId: Task identifier
 * @retval None
 */
void Watchdog_ReportHeartbeat(WatchdogTaskId_t taskId)
{
    if (taskId < WATCHDOG_NUM_TASKS)
    {
        taskHeartbeats[taskId] = xTaskGetTickCount();
    }
}

/**
 * @brief  Watchdog monitoring task - checks task heartbeats and feeds IWDG
 * @param  pvParameters: pointer to task parameters
 * @retval None
 * @note   This task monitors critical tasks and only feeds watchdog if all are alive
 */
void WatchdogTask(void *pvParameters)
{
    (void)pvParameters;
    
    const TickType_t checkPeriod = pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS);
    const TickType_t heartbeatTimeout = pdMS_TO_TICKS(HEARTBEAT_TIMEOUT_MS);
    
    /* Initialize all heartbeats to current time */
    TickType_t now = xTaskGetTickCount();
    for (uint8_t i = 0; i < WATCHDOG_NUM_TASKS; i++)
    {
        taskHeartbeats[i] = now;
    }
    
    Log("[WDG] Watchdog task started - monitoring 4 critical tasks");
    
    for (;;)
    {
        vTaskDelay(checkPeriod);
        
        now = xTaskGetTickCount();
        uint8_t allTasksAlive = 1;
        
        /* Check if all tasks have reported recently */
        for (uint8_t i = 0; i < WATCHDOG_NUM_TASKS; i++)
        {
            TickType_t timeSinceHeartbeat = now - taskHeartbeats[i];
            
            if (timeSinceHeartbeat > heartbeatTimeout)
            {
                allTasksAlive = 0;
                /* Log which task is stuck (will be last message before reset) */
                const char* taskNames[] = {"Controller", "Analysis", "Logger", "Command"};
                Log("[WDG] CRITICAL: Task missing heartbeat: ");
                Log(taskNames[i]);
                break;
            }
        }
        
        if (allTasksAlive)
        {
            /* All tasks alive - feed watchdog */
            Watchdog_Feed();
        }
        else
        {
            /* One or more tasks stuck - DO NOT feed watchdog */
            /* System will reset via IWDG timeout */
            Log("[WDG] System will reset due to task failure");
            /* Let watchdog expire naturally */
        }
    }
}

#else /* ENABLE_WATCHDOG == 0 */

void Watchdog_Init(void)
{
    /* Watchdog disabled at build time */
}

void Watchdog_Feed(void)
{
    /* Watchdog disabled */
}

void Watchdog_ReportHeartbeat(WatchdogTaskId_t taskId)
{
    (void)taskId;
    /* Watchdog disabled */
}

void WatchdogTask(void *pvParameters)
{
    (void)pvParameters;
    /* Watchdog disabled - task does nothing but delay */
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif /* ENABLE_WATCHDOG */
