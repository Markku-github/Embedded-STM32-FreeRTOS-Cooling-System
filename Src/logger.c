/**
 ******************************************************************************
 * @file    logger.c
 * @brief   Logging system implementation via USART3 with DMA
 ******************************************************************************
 */

/* Own header */
#include "logger.h"

/* MCU headers */
#include "stm32f7xx.h"

/* Project headers */
#include "config.h"
#include "watchdog.h"

/* DMA definitions for USART3 TX */
#define DMA_BUFFER_SIZE 512

/* DMA register structures */
typedef struct {
    volatile uint32_t CR;     /* Control register */
    volatile uint32_t NDTR;   /* Number of data register */
    volatile uint32_t PAR;    /* Peripheral address register */
    volatile uint32_t M0AR;   /* Memory 0 address register */
    volatile uint32_t M1AR;   /* Memory 1 address register */
    volatile uint32_t FCR;    /* FIFO control register */
} DMA_Stream_TypeDef;

typedef struct {
    volatile uint32_t LISR;   /* Low interrupt status register */
    volatile uint32_t HISR;   /* High interrupt status register */
    volatile uint32_t LIFCR;  /* Low interrupt flag clear register */
    volatile uint32_t HIFCR;  /* High interrupt flag clear register */
} DMA_TypeDef;

#define DMA1_BASE           0x40026000UL
#define DMA1                ((DMA_TypeDef*)DMA1_BASE)
#define DMA1_Stream3_BASE   (DMA1_BASE + 0x10 + 0x18 * 3)  /* Stream 3 offset */
#define DMA1_Stream3        ((DMA_Stream_TypeDef*)DMA1_Stream3_BASE)

/* DMA1 Stream3 Channel 4 is USART3_TX */
#define DMA_SxCR_CHSEL_4    (4 << 25)  /* Channel 4 */
#define DMA_SxCR_PL_HIGH    (2 << 16)  /* Priority high */
#define DMA_SxCR_MINC       (1 << 10)  /* Memory increment */
#define DMA_SxCR_DIR_M2P    (1 << 6)   /* Memory to peripheral */
#define DMA_SxCR_TCIE       (1 << 4)   /* Transfer complete interrupt */
#define DMA_SxCR_EN         (1 << 0)   /* Enable */

#define DMA_HIFCR_CTCIF3    (1 << 27)  /* Clear transfer complete flag for stream 3 */
#define DMA_HISR_TCIF3      (1 << 27)  /* Transfer complete flag for stream 3 */

#define USART_CR3_DMAT      (1 << 7)   /* DMA transmitter enable */

/* DMA buffer and state */
static char dmaBuffer[DMA_BUFFER_SIZE];
static volatile uint8_t dmaTransferInProgress = 0;

/**
 * @brief  Simple integer to string conversion with buffer size protection
 * @param  value: integer to convert
 * @param  str: output buffer
 * @param  bufSize: size of output buffer (must be at least 2)
 * @retval None
 * @note   Buffer overflow protected - truncates if necessary
 */
static void itoa_simple(uint32_t value, char *str, size_t bufSize)
{
    if (bufSize < 2)
    {
        return;  /* Buffer too small */
    }
    
    char temp[16];
    int idx = 0;
    
    if (value == 0)
    {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    
    /* Convert digits in reverse */
    while (value > 0 && idx < 15)
    {
        temp[idx++] = '0' + (value % 10);
        value /= 10;
    }
    
    /* Reverse to correct order with bounds checking */
    int i;
    for (i = 0; i < idx && i < (int)(bufSize - 1); i++)
    {
        str[i] = temp[idx - 1 - i];
    }
    str[i] = '\0';
}

/**
 * @brief  Initialize USART3 for logging (115200 baud, 8N1) with DMA
 * @note   USART3: PD8 = TX, PD9 = RX (Nucleo-F767ZI virtual COM port)
 * @note   DMA1 Stream3 Channel 4 used for TX
 * @retval None
 */
void Logger_Init(void)
{
    /* Enable clocks for GPIOD, USART3, and DMA1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    RCC->AHB1ENR |= (1 << 21);  /* DMA1 clock enable */
    
    /* Configure PD8 (TX) and PD9 (RX) as alternate function */
    /* MODER: 10 = Alternate function mode */
    GPIOD->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOD->MODER |= ((2 << (8 * 2)) | (2 << (9 * 2)));
    
    /* Set output type to push-pull (default, but explicit) */
    GPIOD->OTYPER &= ~((1 << 8) | (1 << 9));
    
    /* Set high speed */
    GPIOD->OSPEEDR |= ((3 << (8 * 2)) | (3 << (9 * 2)));
    
    /* No pull-up/pull-down */
    GPIOD->PUPDR &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    
    /* Set alternate function AF7 (USART3) for PD8 and PD9 */
    /* AFR[1] is for pins 8-15 */
    GPIOD->AFR[1] &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4)));
    GPIOD->AFR[1] |= ((7 << ((8 - 8) * 4)) | (7 << ((9 - 8) * 4)));
    
    /* Configure USART3 baud rate for 115200 baud */
    USART3->BRR = UART_BRR_115200;
    
    /* Configure DMA1 Stream3 for USART3 TX */
    DMA1_Stream3->CR = 0;  /* Disable stream first */
    while (DMA1_Stream3->CR & DMA_SxCR_EN);  /* Wait until disabled */
    
    /* Clear all interrupt flags for stream 3 */
    DMA1->HIFCR = 0x3F << 22;  /* Clear all flags for stream 3 */
    
    /* Configure DMA stream:
     * - Channel 4 (USART3_TX)
     * - Memory to peripheral
     * - Memory increment mode
     * - High priority
     * - Transfer complete interrupt enabled
     */
    DMA1_Stream3->CR = DMA_SxCR_CHSEL_4 | 
                       DMA_SxCR_PL_HIGH | 
                       DMA_SxCR_MINC | 
                       DMA_SxCR_DIR_M2P | 
                       DMA_SxCR_TCIE;
    
    /* Set peripheral address (USART3 TDR) */
    DMA1_Stream3->PAR = (uint32_t)&USART3->TDR;
    
    /* Enable DMA1 Stream3 interrupt in NVIC */
    NVIC_SetPriority((IRQn_Type)59, 5);  /* DMA1_Stream3_IRQn = 59 */
    NVIC_EnableIRQ((IRQn_Type)59);
    
    /* Enable USART3 DMA transmitter */
    USART3->CR3 |= USART_CR3_DMAT;
    
    /* Enable USART, transmitter */
    USART3->CR1 = USART_CR1_UE | USART_CR1_TE;

    /* Wait for USART to be ready (small delay) */
    for (volatile int i = 0; i < 10000; i++);

    /* TEST: Send 'A' using polling to verify UART works before DMA */
    Logger_SendChar('A');
}

/**
 * @brief  Send one character via USART3 (polling)
 * @param  c: character to send
 * @retval None
 */
void Logger_SendChar(char c)
{
    while (!(USART3->ISR & USART_ISR_TXE));
    USART3->TDR = c;
}
// ...existing code...

/**
 * @brief  Send string via USART3 with DMA
 * @param  str: null-terminated string
 * @retval None
 * @note   Uses DMA for efficient transmission, falls back to polling if DMA busy
 */
void Logger_SendString(const char *str)
{
    if (!str) return;
    
    /* Calculate string length */
    size_t len = 0;
    while (str[len] != '\0' && len < LOG_MSG_MAX_LEN)
    {
        len++;
    }
    
    if (len == 0) return;
    
    /* If DMA transfer in progress, wait or use polling fallback */
    if (dmaTransferInProgress)
    {
        /* Fallback to polling for this message */
        for (size_t i = 0; i < len; i++)
        {
            Logger_SendChar(str[i]);
        }
        return;
    }
    
    /* Copy string to DMA buffer */
    for (size_t i = 0; i < len; i++)
    {
        dmaBuffer[i] = str[i];
    }
    
    /* Mark transfer as in progress */
    dmaTransferInProgress = 1;
    
    /* Disable DMA stream */
    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream3->CR & DMA_SxCR_EN);
    
    /* Clear transfer complete flag */
    DMA1->HIFCR = DMA_HIFCR_CTCIF3;
    
    /* Configure memory address and transfer count */
    DMA1_Stream3->M0AR = (uint32_t)dmaBuffer;
    DMA1_Stream3->NDTR = len;
    
    /* Enable DMA stream to start transfer */
    DMA1_Stream3->CR |= DMA_SxCR_EN;
}

/**
 * @brief  DMA1 Stream3 interrupt handler (USART3 TX)
 * @retval None
 * @note   Called when DMA transfer completes
 */
void DMA1_Stream3_IRQHandler(void)
{
    /* Check if transfer complete interrupt */
    if (DMA1->HISR & DMA_HISR_TCIF3)
    {
        /* Clear interrupt flag */
        DMA1->HIFCR = DMA_HIFCR_CTCIF3;
        
        /* Mark transfer as complete */
        dmaTransferInProgress = 0;
    }
}

/**
 * @brief  Logger task - centralized logging via UART3
 * @param  pvParameters: pointer to task parameters
 * @retval None
 */
void LoggerTask(void *pvParameters)
{
    (void)pvParameters;
    char logMsg[LOG_MSG_MAX_LEN];
    
    /* Send startup banner directly (before queue is active) */
    Logger_SendString("\r\n=== STM32-RTcore: Smart Cooling Controller ===\r\n");
    Logger_SendString("System starting...\r\n\r\n");
    
    for (;;)
    {
        /* Wait for log messages from queue with timeout */
        if (xQueueReceive(xLogQueue, logMsg, pdMS_TO_TICKS(QUEUE_RECEIVE_TIMEOUT_MS)) == pdTRUE)
        {
            /* Send timestamp manually without snprintf */
            uint32_t timestamp_ms = xTaskGetTickCount();
            char timeBuf[16];
            
            Logger_SendChar('[');
            itoa_simple(timestamp_ms, timeBuf, sizeof(timeBuf));
            Logger_SendString(timeBuf);
            Logger_SendString(" ms] ");
            
            /* Send log message */
            Logger_SendString(logMsg);
            Logger_SendString("\r\n");
        }
        
        /* Report heartbeat to watchdog (even on timeout) */
        Watchdog_ReportHeartbeat(WATCHDOG_TASK_LOGGER);
    }
}

/**
 * @brief  Queue a log message (thread-safe)
 * @param  msg: null-terminated log message
 * @retval None
 * @note   Message is copied to a local buffer before queuing to avoid
 *         dangling pointer issues. Queue uses copy-by-value.
 */
void Log(const char *msg)
{
    /* Send to logger queue if available */
    if (xLogQueue != NULL)
    {
        /* Copy message to local buffer (queue will copy this buffer) */
        char buf[LOG_MSG_MAX_LEN];
        size_t i;
        
        /* Manual strncpy to avoid including string.h */
        for (i = 0; i < LOG_MSG_MAX_LEN - 1 && msg[i] != '\0'; i++)
        {
            buf[i] = msg[i];
        }
        buf[i] = '\0';  /* Null-terminate */
        
        /* Send buffer (queue copies the entire buffer) */
        xQueueSend(xLogQueue, buf, 0);  /* Don't block */
    }
}
