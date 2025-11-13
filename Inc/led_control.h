/**
 ******************************************************************************
 * @file    led_control.h
 * @brief   LED control functions and tasks
 * @note    Green LED = PB0, Blue LED = PB7, Red LED = PB14
 *          USER Button = PC13 (emergency trigger)
 ******************************************************************************
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* LED pin definitions (Nucleo-F767ZI) */
#define LED_GREEN       0   /* LD1 on PB0 (Fan speed indicator / COOLING state) */
#define LED_BLUE        7   /* LD2 on PB7 (System active / MONITORING state) */
#define LED_RED         14  /* LD3 on PB14 (Emergency / ALARM state) */

/* USER Button pin (Nucleo-F767ZI) */
#define BUTTON_USER     13  /* USER button B1 on PC13 (active HIGH, external pull-down) */

/* System state machine */
typedef enum {
    STATE_IDLE = 0,      /* System idle or after reset */
    STATE_MONITORING,    /* Normal monitoring, 0-20°C */
    STATE_COOLING,       /* Active cooling, 20-80°C */
    STATE_CRITICAL,      /* Critical temperature, 80-100°C */
    STATE_ALARM          /* Emergency alarm (button/command triggered) */
} SystemState_t;

/* LED blink intervals (milliseconds) */
#define LED_BLINK_FAST      100   /* Fast blink for ALARM state */
#define LED_BLINK_MEDIUM    250   /* Medium blink for COOLING state */
#define LED_BLINK_SLOW      500   /* Slow blink for MONITORING state */
#define LED_HEARTBEAT       1000  /* Heartbeat interval for system alive */

/* Task stack sizes (in words, 1 word = 4 bytes on ARM Cortex-M) */
#define FAN_CONTROL_STACK_SIZE   192  /* 768 bytes - fan control with temperature calculations */
#define BLUE_LED_STACK_SIZE      256  /* 1024 bytes - SOS pattern needs larger stack (18 phases) */

/**
 * @brief  Convert SystemState enum to string
 * @param  state: System state
 * @retval Pointer to state name string
 */
static inline const char* SystemState_ToString(SystemState_t state)
{
    static const char* names[] = {
        "IDLE", "MONITORING", "COOLING", "CRITICAL", "ALARM"
    };
    return (state < 5) ? names[state] : "UNKNOWN";
}

/**
 * @brief  Initialize GPIO for LEDs
 * @retval None
 */
void LED_Init(void);

/**
 * @brief  Initialize USER button (PC13) with interrupt
 * @retval None
 */
void Button_Init(void);

/**
 * @brief  Turn LED on
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
void LED_On(uint8_t pin);

/**
 * @brief  Turn LED off
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
void LED_Off(uint8_t pin);

/**
 * @brief  Toggle LED
 * @param  pin: GPIO pin number (0-15)
 * @retval None
 */
void LED_Toggle(uint8_t pin);

/**
 * @brief  Set LED states based on system state
 * @param  state: Current system state
 * @retval None
 */
void LED_SetState(SystemState_t state);

/**
 * @brief  Fan control task - controls green LED blink rate based on temperature
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void FanControlTask(void *pvParameters);

/**
 * @brief  Blue LED control task - controls blue LED based on system state
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void BlueLEDControlTask(void *pvParameters);

#endif /* LED_CONTROL_H */
