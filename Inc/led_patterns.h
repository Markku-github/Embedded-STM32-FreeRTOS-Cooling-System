/**
 ******************************************************************************
 * @file    led_patterns.h
 * @brief   LED pattern state machines (SOS, double-pulse, blink)
 ******************************************************************************
 */

#ifndef LED_PATTERNS_H
#define LED_PATTERNS_H

#include <stdint.h>

/* Pattern timing constants (milliseconds) */
/* IDLE state */
#define IDLE_BLINK_INTERVAL         1500

/* MONITORING state */
#define MONITORING_BLINK_INTERVAL   500

/* CRITICAL state - double pulse pattern */
#define CRITICAL_PAUSE_LONG         1000
#define CRITICAL_PULSE_ON           150
#define CRITICAL_PULSE_OFF          150

/* ALARM state - SOS morse code pattern */
#define SOS_DOT_ON                  200   /* Short pulse (dot) */
#define SOS_DOT_OFF                 200   /* Space between dots/dashes within letter */
#define SOS_DASH_ON                 800   /* Long pulse (dash) */
#define SOS_DASH_OFF                200   /* Space between dashes */
#define SOS_LETTER_PAUSE            400   /* Pause between S and O, O and S */
#define SOS_SEQUENCE_PAUSE          2000  /* Long pause between SOS sequences */

/* Fan control timing constants */
#define MIN_BLINK_INTERVAL_MS       50    /* Fastest blink at 80°C+ */
#define MAX_BLINK_INTERVAL_MS       800   /* Slowest blink at 20°C */

/**
 * @brief  Update simple blink pattern
 * @param  currentTime: Current tick count
 * @param  lastTime: Pointer to last toggle time
 * @param  ledState: Pointer to current LED state
 * @param  interval: Blink interval in ms
 * @retval New LED state (0=OFF, 1=ON)
 */
uint8_t LED_Pattern_Blink(uint32_t currentTime, uint32_t *lastTime, uint8_t *ledState, uint32_t interval);

/**
 * @brief  Update double-pulse pattern (CRITICAL state)
 * @param  currentTime: Current tick count
 * @param  lastTime: Pointer to last state change time
 * @param  phase: Pointer to current pattern phase (0-3)
 * @retval LED state (0=OFF, 1=ON)
 */
uint8_t LED_Pattern_DoublePulse(uint32_t currentTime, uint32_t *lastTime, uint8_t *phase);

/**
 * @brief  Update SOS morse code pattern (ALARM state)
 * @param  currentTime: Current tick count
 * @param  lastTime: Pointer to last state change time
 * @param  phase: Pointer to current pattern phase (0-17)
 * @retval LED state (0=OFF, 1=ON)
 */
uint8_t LED_Pattern_SOS(uint32_t currentTime, uint32_t *lastTime, uint8_t *phase);

/**
 * @brief  Calculate fan blink interval based on temperature
 * @param  temperature: Current temperature in Celsius
 * @retval Blink interval in milliseconds
 */
uint32_t LED_Pattern_FanBlinkInterval(float temperature);

#endif /* LED_PATTERNS_H */
