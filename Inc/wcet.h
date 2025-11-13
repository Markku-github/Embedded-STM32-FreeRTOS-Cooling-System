/**
 ******************************************************************************
 * @file    wcet.h
 * @brief   WCET (Worst-Case Execution Time) measurement using DWT cycle counter
 * @note    Cortex-M7 DWT->CYCCNT for cycle-accurate timing
 ******************************************************************************
 */

#ifndef WCET_H
#define WCET_H

#include <stdint.h>

/**
 * @brief  Initialize DWT cycle counter and WCET tracking
 * @param  cpu_hz: CPU frequency in Hz (for µs/ms conversion)
 * @retval None
 */
void WCET_Init(uint32_t cpu_hz);

/**
 * @brief  Start cycle counter measurement
 * @retval Current cycle count (32-bit)
 */
uint32_t WCET_Start(void);

/**
 * @brief  Stop measurement and record maximum for task
 * @param  task_name: Task name (max 24 chars)
 * @param  start_cycles: Cycle count from WCET_Start()
 * @retval None
 */
void WCET_StopAndRecord(const char *task_name, uint32_t start_cycles);

/**
 * @brief  Print WCET report to UART (cycles, µs, ms)
 * @retval None
 */
void WCET_PrintReport(void);

/**
 * @brief  Get total CPU load percentage (accumulated task time / wall clock time)
 * @retval CPU load 0-100 (approximate)
 */
uint8_t WCET_GetCPULoad(void);

#endif /* WCET_H */
