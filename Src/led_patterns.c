/**
 ******************************************************************************
 * @file    led_patterns.c
 * @brief   LED pattern state machine implementations
 ******************************************************************************
 */

/* Own header */
#include "led_patterns.h"

/* Project headers */
#include "config.h"

/**
 * @brief  Update simple blink pattern
 * @param  currentTime: Current tick count
 * @param  lastTime: Pointer to last toggle time
 * @param  ledState: Pointer to current LED state
 * @param  interval: Blink interval in ms
 * @retval New LED state (0=OFF, 1=ON)
 */
uint8_t LED_Pattern_Blink(uint32_t currentTime, uint32_t *lastTime, uint8_t *ledState, uint32_t interval)
{
    if ((currentTime - *lastTime) >= interval)
    {
        *ledState = !(*ledState);
        *lastTime = currentTime;
    }
    return *ledState;
}

/**
 * @brief  Update double-pulse pattern (CRITICAL state)
 * @param  currentTime: Current tick count
 * @param  lastTime: Pointer to last state change time
 * @param  phase: Pointer to current pattern phase (0-3)
 * @retval LED state (0=OFF, 1=ON)
 * @note   Pattern: OFF 1000ms -> ON 150ms -> OFF 150ms -> ON 150ms -> repeat
 */
uint8_t LED_Pattern_DoublePulse(uint32_t currentTime, uint32_t *lastTime, uint8_t *phase)
{
    uint8_t ledState = 0;
    
    switch (*phase)
    {
        case 0:  /* OFF for 1000ms */
            ledState = 0;
            if ((currentTime - *lastTime) >= CRITICAL_PAUSE_LONG)
            {
                *phase = 1;
                *lastTime = currentTime;
            }
            break;
            
        case 1:  /* First pulse ON for 150ms */
            ledState = 1;
            if ((currentTime - *lastTime) >= CRITICAL_PULSE_ON)
            {
                *phase = 2;
                *lastTime = currentTime;
            }
            break;
            
        case 2:  /* OFF for 150ms */
            ledState = 0;
            if ((currentTime - *lastTime) >= CRITICAL_PULSE_OFF)
            {
                *phase = 3;
                *lastTime = currentTime;
            }
            break;
            
        case 3:  /* Second pulse ON for 150ms */
            ledState = 1;
            if ((currentTime - *lastTime) >= CRITICAL_PULSE_ON)
            {
                *phase = 0;  /* Reset to start */
                *lastTime = currentTime;
            }
            break;
            
        default:
            *phase = 0;
            break;
    }
    
    return ledState;
}

/**
 * @brief  Update SOS morse code pattern (ALARM state)
 * @param  currentTime: Current tick count
 * @param  lastTime: Pointer to last state change time
 * @param  phase: Pointer to current pattern phase (0-17)
 * @retval LED state (0=OFF, 1=ON)
 * @note   Pattern: ··· --- ··· (SOS morse code with 2s pause)
 */
uint8_t LED_Pattern_SOS(uint32_t currentTime, uint32_t *lastTime, uint8_t *phase)
{
    uint8_t ledState = 0;
    
    switch (*phase)
    {
        /* First S - three dots */
        case 0:  /* Dot 1 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DOT_ON) { *phase = 1; *lastTime = currentTime; }
            break;
        case 1:  /* Dot 1 OFF */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_DOT_OFF) { *phase = 2; *lastTime = currentTime; }
            break;
        case 2:  /* Dot 2 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DOT_ON) { *phase = 3; *lastTime = currentTime; }
            break;
        case 3:  /* Dot 2 OFF */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_DOT_OFF) { *phase = 4; *lastTime = currentTime; }
            break;
        case 4:  /* Dot 3 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DOT_ON) { *phase = 5; *lastTime = currentTime; }
            break;
        case 5:  /* Dot 3 OFF, short pause before O */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_LETTER_PAUSE) { *phase = 6; *lastTime = currentTime; }
            break;
            
        /* O - three dashes */
        case 6:  /* Dash 1 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DASH_ON) { *phase = 7; *lastTime = currentTime; }
            break;
        case 7:  /* Dash 1 OFF */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_DASH_OFF) { *phase = 8; *lastTime = currentTime; }
            break;
        case 8:  /* Dash 2 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DASH_ON) { *phase = 9; *lastTime = currentTime; }
            break;
        case 9:  /* Dash 2 OFF */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_DASH_OFF) { *phase = 10; *lastTime = currentTime; }
            break;
        case 10:  /* Dash 3 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DASH_ON) { *phase = 11; *lastTime = currentTime; }
            break;
        case 11:  /* Dash 3 OFF, short pause before second S */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_LETTER_PAUSE) { *phase = 12; *lastTime = currentTime; }
            break;
            
        /* Second S - three dots */
        case 12:  /* Dot 1 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DOT_ON) { *phase = 13; *lastTime = currentTime; }
            break;
        case 13:  /* Dot 1 OFF */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_DOT_OFF) { *phase = 14; *lastTime = currentTime; }
            break;
        case 14:  /* Dot 2 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DOT_ON) { *phase = 15; *lastTime = currentTime; }
            break;
        case 15:  /* Dot 2 OFF */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_DOT_OFF) { *phase = 16; *lastTime = currentTime; }
            break;
        case 16:  /* Dot 3 ON */
            ledState = 1;
            if ((currentTime - *lastTime) >= SOS_DOT_ON) { *phase = 17; *lastTime = currentTime; }
            break;
        case 17:  /* Dot 3 OFF, then long pause */
            ledState = 0;
            if ((currentTime - *lastTime) >= SOS_SEQUENCE_PAUSE)
            {
                *phase = 0;  /* Restart SOS sequence */
                *lastTime = currentTime;
            }
            break;
            
        default:
            *phase = 0;
            break;
    }
    
    return ledState;
}

/**
 * @brief  Calculate fan blink interval based on temperature
 * @param  temperature: Current temperature in Celsius
 * @retval Blink interval in milliseconds
 * @note   Linear mapping: 20°C -> 800ms, 80°C -> 50ms
 */
uint32_t LED_Pattern_FanBlinkInterval(float temperature)
{
    float temp = temperature;
    
    /* Clamp to valid range */
    if (temp < TEMP_WARNING_MAX) temp = TEMP_WARNING_MAX; /* 20°C minimum */
    if (temp > TEMP_CRITICAL) temp = TEMP_CRITICAL;       /* 80°C maximum */
    
    /* Linear mapping: 20°C -> 800ms, 80°C -> 50ms */
    float tempRange = TEMP_CRITICAL - TEMP_WARNING_MAX;  /* 60°C range */
    float tempOffset = temp - TEMP_WARNING_MAX;
    
    uint32_t interval = MAX_BLINK_INTERVAL_MS -
                       (uint32_t)((tempOffset / tempRange) *
                       (MAX_BLINK_INTERVAL_MS - MIN_BLINK_INTERVAL_MS));
    
    return interval;
}
