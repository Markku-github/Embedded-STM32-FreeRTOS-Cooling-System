# STM32-RTcore

A minimal **FreeRTOS demonstration** for **STM32F767ZI (Nucleo-F767ZI)** showing basic real-time task scheduling and bare-metal GPIO control.

## Overview

This project integrates a minimal FreeRTOS kernel into a bare-metal STM32F7 application.  
It replaces traditional delay loops with a properly scheduled LED task using `vTaskDelay()`.

**Main features**
- FreeRTOS kernel (ARM_CM7 port)
- One LED blink task
- Bare-metal GPIO (no HAL)
- CMake/Ninja build system

## Hardware

- **Board:** Nucleo-F767ZI  
- **MCU:** STM32F767ZIT6 (Cortex-M7, 512 KB RAM, 2 MB Flash)

## Build & Flash

**Requirements:**  
CMake ≥ 3.20, GNU Arm Embedded Toolchain, ST-Link or `st-flash`.

```bash
cmake --preset Debug
cmake --build build/Debug
st-flash write build/Debug/STM32-RTcore.bin 0x08000000
