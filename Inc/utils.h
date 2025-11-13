/**
 ******************************************************************************
 * @file    utils.h
 * @brief   Utility functions for string manipulation
 ******************************************************************************
 */

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

/* Maximum value for simple_itoa (3 digits) */
#define SIMPLE_ITOA_MAX_VALUE 999

/**
 * @brief  Copy string from source to destination
 * @param  dest: destination buffer
 * @param  src: source string
 * @param  maxLen: maximum length of destination buffer
 * @retval None
 */
void str_copy(char *dest, const char *src, uint16_t maxLen);

/**
 * @brief  Append string to destination
 * @param  dest: destination buffer
 * @param  src: source string to append
 * @param  maxLen: maximum length of destination buffer
 * @retval None
 */
void str_append(char *dest, const char *src, uint16_t maxLen);

/**
 * @brief  Convert signed integer to string
 * @param  value: integer value to convert
 * @param  buffer: buffer to store the string
 * @retval None
 */
void int_to_str(int32_t value, char *buffer);

/**
 * @brief  Convert unsigned integer to string (0-999)
 * @param  value: integer value to convert (0-999)
 * @param  buffer: buffer to store the string (min 4 bytes)
 * @retval 1 if successful, 0 if value out of range
 */
uint8_t simple_itoa(uint16_t value, char *buffer);

#endif /* UTILS_H */
