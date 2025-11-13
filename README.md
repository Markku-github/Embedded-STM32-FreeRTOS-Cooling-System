# STM32-RTcore

Smart Cooling Controller — FreeRTOS-based cooling controller demo for STM32F767ZI (Nucleo-F767ZI).

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-v10.x-blue)]()
[![License](https://img.shields.io/badge/license-MIT-green)]()

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware](#hardware)
- [Architecture](#architecture)
- [Getting Started](#getting-started)
- [Commands](#commands)

- [Troubleshooting](#troubleshooting)
- [Known Limitations](#known-limitations)
- [Contributing](#contributing)

## 🎯 Overview

This project demonstrates a professional embedded system implementation using FreeRTOS on the STM32F767ZI microcontroller. The system simulates a smart cooling controller with temperature monitoring, state machine control, LED indicators, and a comprehensive command interface.

**Key Highlights:**
- ✅ 8 concurrent FreeRTOS tasks with proper synchronization
- ✅ DMA-accelerated UART transmission for efficient logging
- ✅ Watchdog heartbeat monitoring for fault detection
- ✅ Robust error handling with timeout protection
- ✅ Buffer overflow protection throughout
- ✅ Comprehensive Doxygen documentation

## ✨ Features

### Core Functionality
- **State Machine:** 5-state system (IDLE, MONITORING, COOLING, CRITICAL, ALARM)
- **Temperature Control:** Manual temperature setting via UART commands (0-100°C)
- **LED Indicators:** 3 LEDs showing system state, fan speed, and alarms
- **Command Interface:** UART-based CLI with 5 commands
- **Emergency System:** Hardware button for immediate alarm triggering
- **Watchdog Protection:** Task-level health monitoring with IWDG

### Advanced Features
- **DMA Logging:** CPU-efficient UART transmission using DMA1 Stream3
- **Reset Detection:** Boot-time logging of reset reason (power-on, watchdog, etc.)
- **Heartbeat System:** Monitors 4 critical tasks, triggers reset on failure
- **Timeout Protection:** All FreeRTOS operations use finite timeouts

## 🔧 Hardware

### Board Specifications
- **Board:** NUCLEO-F767ZI
- **MCU:** STM32F767ZIT6
  - **Core:** ARM Cortex-M7 @ 16 MHz (HSI)
  - **RAM:** 512 KB
  - **Flash:** 2 MB
  - **FPU:** Single/Double precision floating-point

### Pin Configuration

#### LEDs
| LED | Pin  | Color | Function |
|-----|------|-------|----------|
| LD1 | PB0  | Green | Fan speed (off/blink/solid) |
| LD2 | PB7  | Blue  | State indicator (patterns) |
| LD3 | PB14 | Red   | Alarm (solid when active) |

#### Button
| Button | Pin  | Type | Function |
|--------|------|------|----------|
| USER (B1) | PC13 | Active HIGH | Emergency trigger |

#### UART Interfaces
| Interface | Pins | Baud Rate | Function |
|-----------|------|-----------|----------|
| USART3 | PD8 (TX), PD9 (RX) | 115200 | Logging (ST-Link VCP) |
| USART6 | PG14 (TX), PG9 (RX) | 115200 | Command interface (Arduino D0/D1) |

## 🏗️ Architecture

### Task Hierarchy

```
┌─────────────────────────────────────────────────────────────┐
│                     FreeRTOS Scheduler                      │
└─────────────────────────────────────────────────────────────┘
                            ▲
                            │
    ┌───────────────────────┼───────────────────────┐
    │                       │                       │
┌───▼────┐            ┌────▼─────┐           ┌────▼──────┐
│Watchdog│            │UserButton│           │Controller │
│Task    │            │  Task    │           │Task       │
│(Pri 4) │            │ (Pri 3)  │           │(Pri 2)    │
└────────┘            └──────────┘           └───────────┘
    │                       │                       │
    │                       │                       │
    │               Emergency Signal          State Machine
    │               (High Priority)         & LED Control
    │                       │                       │
    │                       ▼                       │
    ├───────────────────────┼───────────────────────┤
    │                       │                       │
┌───▼────┐            ┌────▼─────┐           ┌────▼──────┐
│FanCtrl │            │ Analysis │           │  BlueLED  │
│Task    │            │ Task     │           │  Task     │
│(Pri 2) │            │ (Pri 1)  │           │(Pri 1)    │
└────────┘            └──────────┘           └───────────┘
    │                       ▲                       │
    │                       │                       │
Actuator                Temp Queue             Indication
Control                 (10 items)                  │
    │                       │                       │
    └───────────────────────┼───────────────────────┘
                            │
                            ▼
                   ┌────────────────┐
                   │  Logger Task   │────► USART3 (DMA)
                   │   (Pri 0)      │
                   └────────────────┘
                            ▲
                            │
                        Log Queue
                        (20 msgs)
                            ▲
                            │
                   ┌────────────────┐
                   │ Command Task   │◄─── USART6 (ISR)
                   │   (Pri 0)      │
                   └────────────────┘
                            ▲
                            │
                        Cmd Queue
                        (64 chars)
```

### Task Details

| Task | Priority | Stack | Period | Function |
|------|----------|-------|--------|----------|
| **WatchdogTask** | 4 (Highest) | 256 words | 500 ms | Monitors task heartbeats, feeds IWDG |
| **UserButtonTask** | 3 (Critical) | 256 words | Event-driven | Emergency sensor simulation |
| **ControllerTask** | 2 (Control) | 256 words | 50 ms | State machine, LED control |
| **FanControlTask** | 2 (Control) | 192 words | 100 ms | Actuator control (green LED/fan) |
| **AnalysisTask** | 1 (Normal) | 256 words | 1000 ms | Temperature monitoring |
| **BlueLEDTask** | 1 (Normal) | 256 words | State-dependent | Blue LED patterns |
| **LoggerTask** | 0 (Low) | 256 words | Event-driven | DMA-based UART logging |
| **CommandTask** | 0 (Low) | 512 words | Event-driven | UART command parsing |

### System States

```
    ┌──────┐
    │ IDLE │ (Blue LED slow blink)
    └───┬──┘
        │
        ▼
  ┌────────────┐
  │ MONITORING │ 0 <= temp < 20°C (Blue LED fast blink)
  └─────┬──────┘
        │
        ▼
   ┌─────────┐
   │ COOLING │ 20°C <= temp < 80°C (Green LED blink basen on temp, Blue LED fast blink)
   └────┬────┘
        │
        ▼
  ┌──────────┐
  │ CRITICAL │ 80°C <= temp (Green LED on, Blue LED douple blink)
  └────┬─────┘
       │
       ▼
   ┌───────┐
   │ ALARM │ Emergency signal (Red LED on, Green LED on, Blue LED blink S.O.S.)
   └───────┘
```

### Synchronization Primitives

#### Queues
- **xLogQueue:** 20 x 128 bytes — Copy-by-value log message queue
- **xTempQueue:** 10 x TempData_t — Temperature measurements
- **xCmdQueue:** 64 x char — Command characters from USART6

#### Mutexes
- **xTempMutex:** Protects `currentTemperature` global
- **xEmergencyMutex:** Protects `emergencySignal` flag

All operations use **finite timeouts** (1000 ms for mutexes, 100-1000 ms for queues) to prevent deadlocks.

## 🚀 Getting Started

### Prerequisites

- **Toolchain:** ARM GCC (arm-none-eabi-gcc) 13.3.1+
- **Build System:** CMake 3.20+, Ninja
- **Flash Tool:** STM32CubeProgrammer
- **Serial Monitor:** PuTTY, screen, or similar (115200 baud)

### Building

```bash
# Configure (first time only)
cmake --preset Debug

# Build
cmake --build build/Debug

# Flash to board
STM32_Programmer_CLI -c port=SWD -d build/Debug/STM32-RTcore.hex -v -rst
```

### Memory Usage

**Current build statistics:**
- **Flash:** ~27 KB / 2048 KB (1.3%)
- **RAM:** ~71 KB / 512 KB (13.5%)

## 💻 Commands

Connect to USART6 (Arduino D0/D1 on Nucleo) at **115200 baud, 8N1**.

### Available Commands

#### `help`
Displays command list and usage.

```
=== Commands ===
temp <0-100> - Set temperature
status       - Show status
emergency    - Trigger emergency (ALARM state)
reset        - Reset to IDLE (temp=0, clear emergency)
help         - Show help
```

#### `temp <value>`
Set simulated temperature (0-100°C).

**Examples:**
```
> temp 25
Temp set to 25C

> temp 85
Temp set to 85C
```

**Validation:**
- Range: 0-100°C
- Input: Integer only
- Overflow protected

#### `status`
Show current system state and temperature.

**Example:**
```
> status
State: COOLING | Temp: 45C | Emerg: 0
```

#### `emergency`
Immediately trigger ALARM state.

```
> emergency
EMERGENCY TRIGGERED! System entering ALARM state.
Use 'reset' command to clear.
```

#### `reset`
Clear emergency and return to IDLE state.

```
> reset
System reset to IDLE state. Temp=0C, Emergency cleared.
```

#### `perf`
Display real-time task performance metrics (WCET analysis).

Captures runtime CPU load for all 8 FreeRTOS tasks with per-task execution times.

**Example Output:**
```
========== Task Load Analysis ==========
Task             | Max (ms) | Avg (ms) | Count
----------------------------------------
Watchdog         | 0      | 0      | 56
FanCtrl          | 0      | 0      | 554
Controller       | 51     | 48     | 28
BlueLED          | 0      | 0      | 554
Analysis         | 2      | 0      | 28
Logger           | 0      | 0      | 37
Command          | 1      | 0      | 65
----------------------------------------
Uptime: 27651 ms | Total task time: 1359 ms | CPU Load: ~4%
========================================
```

**Columns:**
- **Max (ms):** Longest single execution measured for task
- **Avg (ms):** Average execution per measurement cycle
- **Count:** Number of measurements sampled
- **CPU Load:** Percentage of CPU time used by all tasks (total task time / uptime × 100%)

**Notes:**
- Measurements exclude blocking I/O (UART DMA, queue timeouts)
- FreeRTOS tick resolution: 1 ms
- Suitable for identifying performance bottlenecks and CPU headroom

## 📊 Performance

### Watchdog System

The system implements a **task-level heartbeat monitoring** system:

- **WatchdogTask** monitors 4 critical tasks (Controller, Analysis, Logger, Command)
- Each task reports heartbeat every iteration
- If any task fails to report within **1500 ms**, watchdog stops feeding IWDG
- System resets via IWDG timeout (~2000 ms)
- Reset reason logged on next boot

**Monitored tasks:**
1. ControllerTask (50 ms period)
2. AnalysisTask (1000 ms period)
3. LoggerTask (event-driven)
4. CommandTask (event-driven)

### DMA Optimization

**Logger TX uses DMA1 Stream3:**
- CPU freed during transmission
- 512-byte DMA buffer
- Fallback to polling if DMA busy
- Transfer complete interrupt driven

**Benefits:**
- Reduced CPU load (~2% logger overhead)
- Can transmit long messages without blocking
- Other tasks continue during log transmission

### Performance Metrics

Measured with **FreeRTOS tick-based WCET profiling** (runtime measurement via `perf` command):

#### Task Execution Time Analysis

| Task | Max (ms) | Avg (ms) | Count | Function |
|------|----------|----------|-------|----------|
| **Controller** | 51 | 48 | 28 | State machine logic, LED control |
| **Analysis** | 2 | 0 | 28 | Temperature data processing |
| **Command** | 1 | 0 | 65 | Character parsing from UART6 |
| **FanCtrl** | 0 | 0 | 554 | Actuator control (very fast) |
| **BlueLED** | 0 | 0 | 554 | LED pattern updates (very fast) |
| **Logger** | 0 | 0 | 37 | Timestamp calculation (DMA I/O excluded) |
| **Watchdog** | 0 | 0 | 56 | Heartbeat monitoring |

#### System Load Summary

| Metric | Value | Target |
|--------|-------|--------|
| **Total CPU Usage** | ~4% | < 10% |
| **Total Task Time** | ~1360 ms | |
| **Uptime (sample)** | ~27650 ms | |
| **Idle Task** | ~96% | > 90% |
| **Scheduler Overhead** | < 1% | < 2% |

#### Notes on Measurements

- **Measurement Method:** WCET module uses FreeRTOS `xTaskGetTickCount()` (millisecond resolution)
- **Accuracy:** ±1-2 ms due to tick granularity
- **Exclusions:** WCET measurements exclude blocking I/O (UART DMA transfers, queue timeouts)
- **Command:** Use `perf` command on USART6 to capture live runtime metrics
- **CPU Load Formula:** (Total task execution time / Uptime) × 100%

**Stack High-Water Marks (minimum free):**
- All tasks: > 60 words remaining
- No stack overflow warnings
- Adequate margin for ISR nesting

## 🔍 Troubleshooting

### No Serial Output on USART3

**Symptoms:** No logs visible in terminal despite flashing successfully.

**Causes & Solutions:**

1. **Wrong COM port**
   - Check Device Manager for ST-Link Virtual COM Port
   - Try all available COM ports

2. **Baud rate mismatch**
   - Verify terminal set to **115200 baud, 8N1**
   - No flow control

3. **Driver issue**
   - Update ST-Link drivers from ST website
   - Reconnect USB cable

4. **UART not initialized**
   - Check `Logger_Init()` is called in `main()`

### System Resets Frequently

**Symptoms:** Device resets every ~2 seconds.

**Causes & Solutions:**

1. **Watchdog timeout**
   - Check if task is stuck (missing heartbeat)
   - Look for "CRITICAL: Task missing heartbeat" log before reset
   - Verify all monitored tasks are running

2. **Stack overflow**
   - Enable stack overflow hook: `configCHECK_FOR_STACK_OVERFLOW = 2`
   - Monitor task health with watchdog system
   - Increase stack size in `config.h` if needed

3. **Hard fault**
   - Enable hard fault handler debugging
   - Check for null pointer dereferences
   - Verify interrupt priorities

### Commands Not Responding

**Symptoms:** Typing commands on USART6 produces no response.

**Causes & Solutions:**

1. **Wrong UART port**
   - USART6 is on **Arduino D0/D1 pins**, not ST-Link VCP
   - Use USB-to-Serial adapter or second Nucleo board

2. **Buffer overflow**
   - Command max length: 63 characters
   - Excess characters silently ignored

3. **Queue full**
   - CommandTask may be blocked
   - Check for mutex timeout warnings in log

### LED Not Behaving as Expected

**Symptoms:** LEDs show wrong pattern or don't light up.

**Causes & Solutions:**

1. **Check system state**
   - Use `status` command to verify current state
   - State machine may be in different state than expected

2. **Temperature out of range**
   - Use `temp` command to set appropriate value
   - State transitions depend on temperature thresholds:
     - MONITORING: > 19°C
     - COOLING: > 20°C
     - CRITICAL: > 80°C

3. **Emergency active**
   - Emergency signal overrides state machine
   - Use `reset` command to clear

### DMA Transfer Issues

**Symptoms:** Corrupted log output or missing characters.

**Causes & Solutions:**

1. **DMA buffer overflow**
   - Messages > 512 bytes truncated
   - Split large messages into smaller chunks

2. **Concurrent transfers**
   - Logger falls back to polling if DMA busy
   - No data loss, but may be slower

3. **DMA not initialized**
   - Verify DMA1 clock enabled
   - Check DMA IRQ enabled in NVIC

## 🤝 Contributing

This project was developed as part of the university's Operating Systems and Concurrency course, but also as part of my own personal learning with RTOS.

## 📄 License

This project is open source and available under the MIT License.

## 🙏 Acknowledgments

- **FreeRTOS** kernel (MIT License)
- **STMicroelectronics** for STM32CubeF7 HAL reference
- **ARM CMSIS** for Cortex-M7 support

---

**Project Status:** Work in Progress
**Last Updated:** November 10, 2025  
**Author:** Markku Johannes Kirjava  
