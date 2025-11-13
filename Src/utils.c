/**
 ******************************************************************************
 * @file    utils.c
 * @brief   Utility functions implementation
 ******************************************************************************
 */

#include "utils.h"

/**
 * @brief  Copy string from source to destination
 * @param  dest: destination buffer
 * @param  src: source string
 * @param  maxLen: maximum length of destination buffer
 * @retval None
 */
void str_copy(char *dest, const char *src, uint16_t maxLen)
{
    uint16_t i = 0;
    while (src[i] != '\0' && i < (maxLen - 1))
    {
        dest[i] = src[i];
        i++;
    }
    dest[i] = '\0';
}

/**
 * @brief  Append string to destination
 * @param  dest: destination buffer
 * @param  src: source string to append
 * @param  maxLen: maximum length of destination buffer
 * @retval None
 */
void str_append(char *dest, const char *src, uint16_t maxLen)
{
    uint16_t destLen = 0;
    
    /* Find end of destination string */
    while (dest[destLen] != '\0' && destLen < maxLen)
    {
        destLen++;
    }
    
    /* Append source string */
    uint16_t i = 0;
    while (src[i] != '\0' && (destLen + i) < (maxLen - 1))
    {
        dest[destLen + i] = src[i];
        i++;
    }
    
    /* Null terminate */
    if (destLen + i < maxLen)
    {
        dest[destLen + i] = '\0';
    }
}

/**
 * @brief  Convert signed integer to string
 * @param  value: integer value to convert
 * @param  buffer: buffer to store the string
 * @retval None
 */
void int_to_str(int32_t value, char *buffer)
{
    uint8_t i = 0;
    uint8_t isNegative = 0;
    
    /* Handle negative numbers */
    if (value < 0)
    {
        isNegative = 1;
        value = -value;
    }
    
    /* Handle zero */
    if (value == 0)
    {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }
    
    /* Convert digits in reverse order */
    char temp[12];
    while (value > 0)
    {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }
    
    /* Add negative sign if needed */
    uint8_t bufIdx = 0;
    if (isNegative)
    {
        buffer[bufIdx++] = '-';
    }
    
    /* Reverse the digits */
    while (i > 0)
    {
        buffer[bufIdx++] = temp[--i];
    }
    buffer[bufIdx] = '\0';
}

/**
 * @brief  Convert unsigned integer to string (0-999)
 * @param  value: integer value to convert (0-999)
 * @param  buffer: buffer to store the string (min 4 bytes)
 * @retval 1 if successful, 0 if value out of range
 */
uint8_t simple_itoa(uint16_t value, char *buffer)
{
    /* Input validation */
    if (value > SIMPLE_ITOA_MAX_VALUE)
    {
        buffer[0] = '\0';
        return 0;
    }
    
    uint8_t i = 0;
    
    /* Handle zero */
    if (value == 0)
    {
        buffer[0] = '0';
        buffer[1] = '\0';
        return 1;
    }
    
    /* Convert digits in reverse order */
    char temp[4];
    while (value > 0)
    {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }
    
    /* Reverse the digits into buffer */
    uint8_t bufIdx = 0;
    while (i > 0)
    {
        buffer[bufIdx++] = temp[--i];
    }
    buffer[bufIdx] = '\0';
    
    return 1;
}
