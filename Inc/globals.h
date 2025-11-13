/**
 ******************************************************************************
 * @file    globals.h
 * @brief   Global shared variables (defined in main.c)
 ******************************************************************************
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>
#include "led_control.h"  /* For SystemState_t */

/**
 * @brief  Shared global variables
 * @note   All defined in main.c
 *         Protected by FreeRTOS scheduler or mutexes
 */

/* Current system state (protected by FreeRTOS scheduler) */
extern volatile SystemState_t currentState;

/* Current temperature in Celsius (protected by xTempMutex) */
extern volatile float currentTemperature;

/* Emergency signal flag: 1 = emergency active, 0 = normal (protected by xEmergencyMutex) */
extern volatile uint8_t emergencySignal;

#endif /* GLOBALS_H */
