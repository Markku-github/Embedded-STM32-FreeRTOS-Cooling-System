/**
 ******************************************************************************
 * @file    wcet.c
 * @brief   Task performance measurement using FreeRTOS ticks
 ******************************************************************************
 */

#include "wcet.h"
#include <string.h>
#include "logger.h"
#include "uart_comm.h"
#include "utils.h"
#include "FreeRTOS.h"
#include "task.h"

#define WCET_MAX_ENTRIES 16

typedef struct {
    char name[24];
    uint32_t max_ticks;
    uint64_t total_ticks;
    uint32_t call_count;
} wcet_entry_t;

static wcet_entry_t wcet_table[WCET_MAX_ENTRIES];
static uint32_t wcet_start_time = 0;

void WCET_Init(uint32_t cpu_hz)
{
    (void)cpu_hz;
    wcet_start_time = xTaskGetTickCount();
    
    for (int i = 0; i < WCET_MAX_ENTRIES; i++)
    {
        wcet_table[i].name[0] = 0;
        wcet_table[i].max_ticks = 0;
        wcet_table[i].total_ticks = 0;
        wcet_table[i].call_count = 0;
    }
    
    Log("[WCET] Performance tracking initialized");
}

uint32_t WCET_Start(void)
{
    return (uint32_t)xTaskGetTickCount();
}

void WCET_StopAndRecord(const char *task_name, uint32_t start_ticks)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint32_t delta = now - start_ticks;
    
    int free_idx = -1;
    for (int i = 0; i < WCET_MAX_ENTRIES; i++)
    {
        if (wcet_table[i].name[0] != 0 &&
            strncmp(wcet_table[i].name, task_name, sizeof(wcet_table[i].name) - 1) == 0)
        {
            if (delta > wcet_table[i].max_ticks)
                wcet_table[i].max_ticks = delta;
            wcet_table[i].total_ticks += delta;
            wcet_table[i].call_count++;
            return;
        }
        if (wcet_table[i].name[0] == 0 && free_idx < 0)
            free_idx = i;
    }
    
    if (free_idx >= 0)
    {
        strncpy(wcet_table[free_idx].name, task_name, sizeof(wcet_table[free_idx].name) - 1);
        wcet_table[free_idx].name[sizeof(wcet_table[free_idx].name) - 1] = '\0';
        wcet_table[free_idx].max_ticks = delta;
        wcet_table[free_idx].total_ticks = delta;
        wcet_table[free_idx].call_count = 1;
    }
}

void WCET_PrintReport(void)
{
    uint32_t now_ticks = xTaskGetTickCount();
    uint32_t elapsed_ticks = now_ticks - wcet_start_time;
    
    UART_Comm_SendString("\r\n========== Task Load Analysis ==========\r\n");
    UART_Comm_SendString("Task             | Max (ms) | Avg (ms) | Count\r\n");
    UART_Comm_SendString("----------------------------------------\r\n");
    
    uint64_t total_task_ms = 0;
    
    for (int i = 0; i < WCET_MAX_ENTRIES; i++)
    {
        if (wcet_table[i].name[0] != 0 && wcet_table[i].call_count > 0)
        {
            uint32_t max_ms = wcet_table[i].max_ticks;
            uint32_t avg_ms = (uint32_t)(wcet_table[i].total_ticks / wcet_table[i].call_count);
            uint32_t calls = wcet_table[i].call_count;
            
            total_task_ms += wcet_table[i].total_ticks;
            
            char line[120];
            char tmp[16];
            
            str_copy(line, wcet_table[i].name, sizeof(line));
            
            /* Pad to 16 chars */
            int len = 0;
            while (line[len] != 0) len++;
            while (len < 16) { line[len] = ' '; len++; }
            line[len] = 0;
            
            str_append(line, "| ", sizeof(line));
            
            memset(tmp, 0, sizeof(tmp));
            int_to_str((int)max_ms, tmp);
            str_append(line, tmp, sizeof(line));
            if (strlen(tmp) < 3) str_append(line, "  ", sizeof(line));
            else str_append(line, " ", sizeof(line));
            str_append(line, "| ", sizeof(line));
            
            memset(tmp, 0, sizeof(tmp));
            int_to_str((int)avg_ms, tmp);
            str_append(line, tmp, sizeof(line));
            if (strlen(tmp) < 3) str_append(line, "  ", sizeof(line));
            else str_append(line, " ", sizeof(line));
            str_append(line, "| ", sizeof(line));
            
            memset(tmp, 0, sizeof(tmp));
            int_to_str((int)calls, tmp);
            str_append(line, tmp, sizeof(line));
            str_append(line, "\r\n", sizeof(line));
            
            UART_Comm_SendString(line);
        }
    }
    
    UART_Comm_SendString("----------------------------------------\r\n");
    
    double cpu_load = 0.0;
    if (elapsed_ticks > 0)
        cpu_load = (double)total_task_ms * 100.0 / (double)elapsed_ticks;
    if (cpu_load > 100.0)
        cpu_load = 100.0;
    
    char summary[120];
    str_copy(summary, "Uptime: ", sizeof(summary));
    char tmp_str[16];
    
    memset(tmp_str, 0, sizeof(tmp_str));
    int_to_str((int)elapsed_ticks, tmp_str);
    str_append(summary, tmp_str, sizeof(summary));
    str_append(summary, " ms | Total task time: ", sizeof(summary));
    
    memset(tmp_str, 0, sizeof(tmp_str));
    int_to_str((int)total_task_ms, tmp_str);
    str_append(summary, tmp_str, sizeof(summary));
    str_append(summary, " ms | CPU Load: ~", sizeof(summary));
    
    memset(tmp_str, 0, sizeof(tmp_str));
    int_to_str((int)cpu_load, tmp_str);
    str_append(summary, tmp_str, sizeof(summary));
    str_append(summary, "%\r\n", sizeof(summary));
    
    UART_Comm_SendString(summary);
    UART_Comm_SendString("========================================\r\n\r\n");
}

uint8_t WCET_GetCPULoad(void)
{
    uint32_t now_ticks = xTaskGetTickCount();
    uint32_t elapsed_ticks = now_ticks - wcet_start_time;
    
    if (elapsed_ticks == 0)
        return 0;
    
    uint64_t total_task_ms = 0;
    for (int i = 0; i < WCET_MAX_ENTRIES; i++)
    {
        if (wcet_table[i].name[0] != 0)
            total_task_ms += wcet_table[i].total_ticks;
    }
    
    double cpu_load = (double)total_task_ms * 100.0 / (double)elapsed_ticks;
    if (cpu_load > 100.0)
        cpu_load = 100.0;
    
    return (uint8_t)cpu_load;
}

