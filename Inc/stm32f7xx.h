/**
 ******************************************************************************
 * @file    stm32f7xx.h
 * @brief   CMSIS STM32F7xx Device Peripheral Access Layer Header File
 ******************************************************************************
 */

#ifndef __STM32F7XX_H
#define __STM32F7XX_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/* Processor and Core Peripheral Section */
#define __CM7_REV                 0x0001U  /* Cortex-M7 revision r0p1 */
#define __MPU_PRESENT             1U       /* MPU present */
#define __NVIC_PRIO_BITS          4U       /* Uses 4 priority bits */
#define __Vendor_SysTickConfig    0U       /* Set to 1 if different SysTick Config */
#define __FPU_PRESENT             1U       /* FPU present */
#define __ICACHE_PRESENT          1U       /* Instruction cache present */
#define __DCACHE_PRESENT          1U       /* Data cache present */

/* Memory Base Addresses */
#define FLASH_BASE            0x08000000UL  /* FLASH base address */
#define SRAM1_BASE            0x20000000UL  /* SRAM1 base address */
#define PERIPH_BASE           0x40000000UL  /* Peripheral base address */

/* Peripheral Memory Map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

/* Peripheral Base Addresses */
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800UL)

/* CMSIS Cortex-M7 Core Peripheral Access Layer */
typedef struct
{
  volatile uint32_t CPACR;   /* Offset: 0x088 (R/W)  Coprocessor Access Control Register */
  volatile uint32_t padding[897]; /* Padding to reach VTOR offset */
  volatile uint32_t VTOR;    /* Offset: 0xD08 (R/W)  Vector Table Offset Register */
} SCB_Type;

/* RCC Register Definition */
typedef struct
{
  volatile uint32_t CR;            /* RCC clock control register,                          Address offset: 0x00 */
  volatile uint32_t PLLCFGR;       /* RCC PLL configuration register,                      Address offset: 0x04 */
  volatile uint32_t CFGR;          /* RCC clock configuration register,                    Address offset: 0x08 */
  volatile uint32_t CIR;           /* RCC clock interrupt register,                        Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;      /* RCC AHB1 peripheral reset register,                  Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;      /* RCC AHB2 peripheral reset register,                  Address offset: 0x14 */
  volatile uint32_t AHB3RSTR;      /* RCC AHB3 peripheral reset register,                  Address offset: 0x18 */
  uint32_t      RESERVED0;         /* Reserved, 0x1C                                                            */
  volatile uint32_t APB1RSTR;      /* RCC APB1 peripheral reset register,                  Address offset: 0x20 */
  volatile uint32_t APB2RSTR;      /* RCC APB2 peripheral reset register,                  Address offset: 0x24 */
  uint32_t      RESERVED1[2];      /* Reserved, 0x28-0x2C                                                       */
  volatile uint32_t AHB1ENR;       /* RCC AHB1 peripheral clock register,                  Address offset: 0x30 */
  volatile uint32_t AHB2ENR;       /* RCC AHB2 peripheral clock enable register,           Address offset: 0x34 */
  volatile uint32_t AHB3ENR;       /* RCC AHB3 peripheral clock enable register,           Address offset: 0x38 */
  uint32_t      RESERVED2;         /* Reserved, 0x3C                                                            */
  volatile uint32_t APB1ENR;       /* RCC APB1 peripheral clock enable register,           Address offset: 0x40 */
  volatile uint32_t APB2ENR;       /* RCC APB2 peripheral clock enable register,           Address offset: 0x44 */
} RCC_TypeDef;

/* GPIO Register Definition */
typedef struct
{
  volatile uint32_t MODER;         /* GPIO port mode register,                             Address offset: 0x00 */
  volatile uint32_t OTYPER;        /* GPIO port output type register,                      Address offset: 0x04 */
  volatile uint32_t OSPEEDR;       /* GPIO port output speed register,                     Address offset: 0x08 */
  volatile uint32_t PUPDR;         /* GPIO port pull-up/pull-down register,                Address offset: 0x0C */
  volatile uint32_t IDR;           /* GPIO port input data register,                       Address offset: 0x10 */
  volatile uint32_t ODR;           /* GPIO port output data register,                      Address offset: 0x14 */
  volatile uint32_t BSRR;          /* GPIO port bit set/reset register,                    Address offset: 0x18 */
  volatile uint32_t LCKR;          /* GPIO port configuration lock register,               Address offset: 0x1C */
  volatile uint32_t AFR[2];        /* GPIO alternate function registers,                   Address offset: 0x20-0x24 */
} GPIO_TypeDef;

/* USART Register Definition */
typedef struct
{
  volatile uint32_t CR1;           /* USART Control register 1,                            Address offset: 0x00 */
  volatile uint32_t CR2;           /* USART Control register 2,                            Address offset: 0x04 */
  volatile uint32_t CR3;           /* USART Control register 3,                            Address offset: 0x08 */
  volatile uint32_t BRR;           /* USART Baud rate register,                            Address offset: 0x0C */
  volatile uint32_t GTPR;          /* USART Guard time and prescaler register,             Address offset: 0x10 */
  volatile uint32_t RTOR;          /* USART Receiver timeout register,                     Address offset: 0x14 */
  volatile uint32_t RQR;           /* USART Request register,                              Address offset: 0x18 */
  volatile uint32_t ISR;           /* USART Interrupt and status register,                 Address offset: 0x1C */
  volatile uint32_t ICR;           /* USART Interrupt flag clear register,                 Address offset: 0x20 */
  volatile uint32_t RDR;           /* USART Receive data register,                         Address offset: 0x24 */
  volatile uint32_t TDR;           /* USART Transmit data register,                        Address offset: 0x28 */
} USART_TypeDef;

#define SCB_CPACR_OFFSET      0xE000ED88UL
#define SCB                   ((SCB_Type *) SCB_CPACR_OFFSET)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)
#define GPIOD                 ((GPIO_TypeDef *) GPIOD_BASE)
#define USART3                ((USART_TypeDef *) USART3_BASE)

/* RCC Register Bit Definitions */
#define RCC_AHB1ENR_GPIODEN   (1UL << 3)   /* GPIOD clock enable */
#define RCC_APB1ENR_USART3EN  (1UL << 18)  /* USART3 clock enable */

/* USART ISR Register Bit Definitions */
#define USART_ISR_TXE         (1UL << 7)   /* Transmit data register empty */
#define USART_ISR_TC          (1UL << 6)   /* Transmission complete */

/* USART CR1 Register Bit Definitions */
#define USART_CR1_UE          (1UL << 0)   /* USART enable */
#define USART_CR1_TE          (1UL << 3)   /* Transmitter enable */
#define USART_CR1_RE          (1UL << 2)   /* Receiver enable */

#ifndef VECT_TAB_OFFSET
#define VECT_TAB_OFFSET  0x00000000UL
#endif

/* System Init and Clock Update */
extern uint32_t SystemCoreClock;
void SystemInit(void);
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F7XX_H */
