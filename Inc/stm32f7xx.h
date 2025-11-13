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
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)

/* CMSIS Cortex-M7 Core Peripheral Access Layer */
typedef struct
{
  volatile uint32_t CPACR;   /* Offset: 0x088 (R/W)  Coprocessor Access Control Register */
  volatile uint32_t padding[897]; /* Padding to reach VTOR offset */
  volatile uint32_t VTOR;    /* Offset: 0xD08 (R/W)  Vector Table Offset Register */
} SCB_Type;

/* NVIC Register Definition */
typedef struct
{
  volatile uint32_t ISER[8];       /* Offset: 0x000 (R/W)  Interrupt Set Enable Register */
  uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];       /* Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
  uint32_t RESERVED1[24];
  volatile uint32_t ISPR[8];       /* Offset: 0x100 (R/W)  Interrupt Set Pending Register */
  uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];       /* Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
  uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];       /* Offset: 0x200 (R/W)  Interrupt Active bit Register */
  uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];       /* Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
  uint32_t RESERVED5[644];
  volatile uint32_t STIR;          /* Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_Type;

/* IRQ Numbers */
typedef enum
{
  EXTI15_10_IRQn              = 40,     /* EXTI Line[15:10] interrupts */
  USART6_IRQn                 = 71      /* USART6 global Interrupt */
} IRQn_Type;

#define __NVIC_PRIO_BITS          4U

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
  uint32_t      RESERVED3[2];      /* Reserved, 0x48-0x4C                                                       */
  volatile uint32_t AHB1LPENR;     /* RCC AHB1 peripheral clock enable in low power mode,  Address offset: 0x50 */
  volatile uint32_t AHB2LPENR;     /* RCC AHB2 peripheral clock enable in low power mode,  Address offset: 0x54 */
  volatile uint32_t AHB3LPENR;     /* RCC AHB3 peripheral clock enable in low power mode,  Address offset: 0x58 */
  uint32_t      RESERVED4;         /* Reserved, 0x5C                                                            */
  volatile uint32_t APB1LPENR;     /* RCC APB1 peripheral clock enable in low power mode,  Address offset: 0x60 */
  volatile uint32_t APB2LPENR;     /* RCC APB2 peripheral clock enable in low power mode,  Address offset: 0x64 */
  uint32_t      RESERVED5[2];      /* Reserved, 0x68-0x6C                                                       */
  volatile uint32_t BDCR;          /* RCC Backup domain control register,                  Address offset: 0x70 */
  volatile uint32_t CSR;           /* RCC clock control & status register,                 Address offset: 0x74 */
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
#define NVIC_BASE             (0xE000E100UL)
#define SCB                   ((SCB_Type *) SCB_CPACR_OFFSET)
#define NVIC                  ((NVIC_Type *) NVIC_BASE)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)
#define GPIOA                 ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOD                 ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOG                 ((GPIO_TypeDef *) GPIOG_BASE)
#define USART2                ((USART_TypeDef *) USART2_BASE)
#define USART3                ((USART_TypeDef *) USART3_BASE)
#define USART6                ((USART_TypeDef *) USART6_BASE)

/* RCC Register Bit Definitions */
#define RCC_AHB1ENR_GPIOAEN   (1UL << 0)   /* GPIOA clock enable */
#define RCC_AHB1ENR_GPIODEN   (1UL << 3)   /* GPIOD clock enable */
#define RCC_AHB1ENR_GPIOGEN   (1UL << 6)   /* GPIOG clock enable */
#define RCC_APB1ENR_USART2EN  (1UL << 17)  /* USART2 clock enable */
#define RCC_APB1ENR_USART3EN  (1UL << 18)  /* USART3 clock enable */
#define RCC_APB2ENR_USART6EN  (1UL << 5)   /* USART6 clock enable */

/* RCC CSR (Clock Control & Status Register) Bit Definitions */
#define RCC_CSR_LSION         (1UL << 0)   /* Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY        (1UL << 1)   /* Internal Low Speed oscillator ready */
#define RCC_CSR_RMVF          (1UL << 24)  /* Remove reset flag */
#define RCC_CSR_BORRSTF       (1UL << 25)  /* BOR reset flag */
#define RCC_CSR_PINRSTF       (1UL << 26)  /* PIN reset flag */
#define RCC_CSR_PORRSTF       (1UL << 27)  /* POR/PDR reset flag */
#define RCC_CSR_SFTRSTF       (1UL << 28)  /* Software reset flag */
#define RCC_CSR_IWDGRSTF      (1UL << 29)  /* Independent watchdog reset flag */
#define RCC_CSR_WWDGRSTF      (1UL << 30)  /* Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF      (1UL << 31)  /* Low-power reset flag */

/* USART ISR Register Bit Definitions */
#define USART_ISR_TXE         (1UL << 7)   /* Transmit data register empty */
#define USART_ISR_TC          (1UL << 6)   /* Transmission complete */
#define USART_ISR_RXNE        (1UL << 5)   /* Read data register not empty */

/* USART CR1 Register Bit Definitions */
#define USART_CR1_UE          (1UL << 0)   /* USART enable */
#define USART_CR1_TE          (1UL << 3)   /* Transmitter enable */
#define USART_CR1_RE          (1UL << 2)   /* Receiver enable */
#define USART_CR1_RXNEIE      (1UL << 5)   /* RXNE interrupt enable */

/* NVIC Functions */
static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  NVIC->ISER[((uint32_t)IRQn) >> 5] = (1UL << ((uint32_t)IRQn & 0x1F));
}

static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  NVIC->IP[(uint32_t)IRQn] = (uint8_t)((priority << (8 - __NVIC_PRIO_BITS)) & 0xFF);
}

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
