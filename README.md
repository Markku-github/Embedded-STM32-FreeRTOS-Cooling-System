# STM32-RTcore

**Smart Cooling Controller** - A FreeRTOS-based industrial cooling system simulation for STM32F767ZI (Nucleo-F767ZI).

## Overview

This educational project demonstrates real-time operating system concepts by simulating an industrial cooling system controller. The system monitors temperature, adjusts fan speed (represented by LEDs), and responds to emergency conditions using FreeRTOS task scheduling, state machines, and interrupt handling.

**Current Status: Phase 1/5** 
- [x] Three-LED GPIO control (PB0, PB7, PB14)
- [ ] UART communication
- [ ] State machine controller
- [ ] Button interrupt handling
- [ ] System monitoring

**Target Features:**
- Multi-task real-time control system
- State machine (NORMAL → BOOST → ALERT → EMERGENCY → RECOVERY)
- UART temperature simulation and logging
- Emergency button interrupt (PC13)
- LED-based visual feedback (fan speed, alerts, emergency)
- System diagnostics (stack, CPU monitoring)

## Hardware

- **Board:** Nucleo-F767ZI  
- **MCU:** STM32F767ZIT6 (Cortex-M7, 512 KB RAM, 2 MB Flash)
- **LEDs:**
  - LD1 (Green, PB0) - Fan speed indicator
  - LD2 (Blue, PB7) - Alert indicator  
  - LD3 (Red, PB14) - Emergency indicator
- **Button:** USER button (PC13) - Emergency sensor (future)
- **UART:** Virtual COM port via ST-Link (future)

## Build & Flash

**Requirements:**  
CMake ≥ 3.20, GNU Arm Embedded Toolchain, ST-Link or `st-flash`.

```bash
cmake --preset Debug
cmake --build build/Debug
st-flash write build/Debug/STM32-RTcore.bin 0x08000000
