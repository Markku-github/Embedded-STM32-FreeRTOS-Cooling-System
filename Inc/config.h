/**
 ******************************************************************************
 * @file    config.h
 * @brief   System configuration constants and thresholds
 * @note    Centralized configuration to avoid magic numbers and duplication
 ******************************************************************************
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ============================================================================
 * Temperature Thresholds (°C)
 * ============================================================================ */
#define TEMP_NORMAL_MAX     19   /**< Below this: MONITORING (hysteresis) */
#define TEMP_WARNING_MAX    20   /**< Above this: COOLING starts */
#define TEMP_CRITICAL       80   /**< Above this: CRITICAL state */

/* ============================================================================
 * Buffer Sizes
 * ============================================================================ */
#define LOG_MSG_MAX_LEN     128  /**< Maximum log message length */
#define TEMP_STR_MAX_LEN    16   /**< Maximum temperature string length */

/* ============================================================================
 * Command Lengths
 * ============================================================================ */
#define CMD_TEMP_MIN_LENGTH  6   /**< Minimum length for "temp X" command */
#define CMD_HELP_LENGTH      4   /**< Length of "help" command */
#define CMD_STATUS_LENGTH    6   /**< Length of "status" command */

/* ============================================================================
 * Task Stack Sizes (in words, 1 word = 4 bytes on ARM)
 * ============================================================================ */
#define LOGGER_STACK_SIZE       256  /**< Logger task stack size */
#define COMMAND_STACK_SIZE      512  /**< Command task stack size */
#define CONTROLLER_STACK_SIZE   256  /**< Controller task stack size */
#define ANALYSIS_STACK_SIZE     256  /**< Analysis task stack size */
#define FAN_CONTROL_STACK_SIZE  192  /**< Fan control task stack size */
#define BLUE_LED_STACK_SIZE     256  /**< Blue LED task stack size */
#define USER_BUTTON_STACK_SIZE  256  /**< User button task stack size */
#define WATCHDOG_STACK_SIZE     256  /**< Watchdog monitoring task stack size */

/* ============================================================================
 * Queue Sizes
 * ============================================================================ */
#define TEMP_QUEUE_SIZE     10   /**< Temperature data queue size */
#define LOG_QUEUE_SIZE      20   /**< Log message queue size */
#define CMD_QUEUE_SIZE      64   /**< Command character queue size */

/* ============================================================================
 * Watchdog Configuration
 * ============================================================================ */
/** Enable/disable Independent Watchdog (IWDG). Set to 0 to disable at build time. */
#define ENABLE_WATCHDOG         1

/** IWDG timeout in milliseconds (approximate, depends on LSI frequency tolerance). */
#define WATCHDOG_TIMEOUT_MS     2000

/** LSI nominal frequency in Hz for IWDG calculations (typ. 32 kHz, ±30%). */
#define LSI_FREQ_HZ             32000U

/** Watchdog heartbeat model configuration */
#define WATCHDOG_CHECK_PERIOD_MS    500   /**< How often watchdog task checks heartbeats */
#define HEARTBEAT_TIMEOUT_MS        1500  /**< Maximum time without heartbeat before reset */

/** Number of critical tasks that must report heartbeat */
#define WATCHDOG_NUM_TASKS          4     /**< Controller, Analysis, Logger, Command */

/* ============================================================================
 * UART Configuration
 * ============================================================================ */
/** 
 * UART baud rate register value for 115200 baud
 * Calculation: BRR = fCK / baud_rate
 * - USART3 (APB1): 16 MHz HSI / 115200 = 138.888... ≈ 139 (0x8B)
 * - USART6 (APB2): Empirically verified, same value works
 * Note: Actual APB clock may vary, this value has been tested and confirmed working
 */
#define UART_BRR_115200         139

/* ============================================================================
 * Button Debounce Configuration
 * ============================================================================ */
/**
 * Button debounce delay in milliseconds
 * Time to wait after interrupt before reading stable button state
 */
#define BUTTON_DEBOUNCE_DELAY_MS    300

/* ============================================================================
 * FreeRTOS Timeout Configuration
 * ============================================================================ */
/**
 * Timeout values for FreeRTOS queue and mutex operations
 * Using finite timeouts instead of portMAX_DELAY to prevent deadlocks
 */
#define MUTEX_TIMEOUT_MS        1000   /**< Mutex take timeout (1 second) */
#define QUEUE_SEND_TIMEOUT_MS   100    /**< Queue send timeout (100 ms) */
#define QUEUE_RECEIVE_TIMEOUT_MS 1000  /**< Queue receive timeout for blocking tasks (1 second) */

#endif /* CONFIG_H */
