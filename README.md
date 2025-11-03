# STM32-RTcore

**Smart Cooling Controller** - A FreeRTOS-based industrial cooling system simulation for STM32F767ZI (Nucleo-F767ZI).

## Overview

This educational project demonstrates real-time operating system concepts by simulating an industrial cooling system controller. The system monitors temperature, controls cooling indicators (LEDs), and manages state transitions using FreeRTOS task scheduling, queues, and a state machine.

**Current Status: Phase 2 - State Machine Implementation**
- [x] Three-LED GPIO control (PB0, PB7, PB14)
- [x] UART TX communication (bare-metal, 115200 baud, USART3)
- [x] State machine controller (IDLE → MONITORING → COOLING → ALARM)
- [x] FreeRTOS task architecture (ControllerTask, AnalysisTask, LoggerTask)
- [x] Inter-task communication (Temperature queue, Log queue)
- [ ] UART RX command input (USART2, Phase 2b)
- [ ] Button interrupt handling (Phase 3)
- [ ] System monitoring (Phase 4)

**System Architecture:**
- **ControllerTask**: Implements state machine, receives temperature data, controls Red LED
- **AnalysisTask**: Simulates temperature sensor, generates temperature data (25°C → 80°C sawtooth)
- **FanControlTask**: Controls Green LED (fan speed simulation) based on temperature and state
- **BlueLEDControlTask**: Controls Blue LED based on system state (different patterns per state)
- **LoggerTask**: Centralized logging via USART3 TX (receives log messages from queue)
- **Queues**: TempQueue (temperature data), LogQueue (log messages)

**State Machine:**
- **IDLE**: System standby (requires explicit start command via UART - auto-starts after 3s for demo)
- **MONITORING**: Normal monitoring (temp < 20°C)
- **COOLING**: Active cooling engaged (20°C ≤ temp < 80°C)
- **ALARM**: Critical alarm (temp ≥ 80°C or emergency button pressed)

## Hardware

- **Board:** Nucleo-F767ZI  
- **MCU:** STM32F767ZIT6 (Cortex-M7, 512 KB RAM, 2 MB Flash)
- **LEDs (Industrial Control Indicators):**
  - **LD1 (Green, PB0) - Fan Speed Indicator:**
    - OFF: IDLE/MONITORING states (no cooling needed, temp <20°C)
    - BLINKING: COOLING state (20-80°C), blink rate proportional to temperature
      - Slower at 20°C (~2s interval)
      - Faster near 80°C (~100ms interval)
    - SOLID: ALARM state (≥80°C, maximum fan speed)
    - Controlled by FanControlTask
  - **LD2 (Blue, PB7) - System State Indicator:**
    - BLINK (1500ms): IDLE state
    - BLINK (500ms): MONITORING state
    - SOLID: COOLING state (active cooling)
    - DOUBLE-PULSE: ALARM state (OFF 1s → ON 150ms → OFF 150ms → ON 150ms → repeat)
    - Controlled by BlueLEDControlTask
  - **LD3 (Red, PB14) - Alarm Indicator:**
    - OFF: Normal operation (IDLE, MONITORING, COOLING)
    - SOLID: ALARM state (critical temperature ≥80°C or emergency button pressed)
    - Controlled by ControllerTask
- **UART (Logging):** USART3 on PD8 (TX), PD9 (RX) - ST-Link Virtual COM port (115200 8N1)
- **UART (Commands):** USART2 on PA2 (TX), PA3 (RX) - External CH340 adapter (future, Phase 2b)

## Build & Flash

**Requirements:**  
CMake ≥ 3.20, GNU Arm Embedded Toolchain, STM32_Programmer_CLI (or `st-flash`).

```bash
# Build
cmake --preset Debug
cmake --build build/Debug

# Flash (using STM32_Programmer_CLI via VS Code task)
# Run "flash" task in VS Code, or manually:
STM32_Programmer_CLI.exe -c port=SWD -d build/Debug/STM32-RTcore.hex -v -rst
```

## Serial Console

Connect to the ST-Link virtual COM port to see system logs:
- **Settings:** 115200 baud, 8 data bits, no parity, 1 stop bit (8N1)
- **Windows:** Usually `COM3` (check Device Manager → Ports)
- **Expected Output:**
  ```
  === STM32-RTcore: Smart Cooling Controller ===
  System starting... Phase 2 - State Machine

  [2000 ms] [CTRL] ControllerTask started
  [2000 ms] [CTRL] State: MONITORING
  [1000 ms] [ANLY] AnalysisTask started
  [1000 ms] [ANLY] Temperature: 25.0 C
  [6000 ms] [ANLY] Temperature: 31.0 C
  [6100 ms] [CTRL] State: MONITORING -> COOLING (Temp: 31.0 C)
  ...
  [65000 ms] [ANLY] Temperature: 70.0 C
  [65100 ms] [CTRL] State: COOLING -> ALARM (Temp: 70.0 C)
  ```

## Expected Behavior

### Visual Feedback (LEDs)

**Green LED (LD1) - Fan Speed:**
- **IDLE/MONITORING** (temp <20°C): OFF (no cooling needed)
- **COOLING** (20-80°C): Blinking, rate proportional to temperature
  - At 20°C: Slow (~2s interval)
  - At 50°C: Medium (~1s interval)
  - At 80°C: Fast (~100ms interval)
- **ALARM** (≥80°C): SOLID (maximum fan speed)

**Blue LED (LD2) - System State:**
- **IDLE**: Blink (1500ms interval, 1.5 seconds ON/OFF)
- **MONITORING**: Blink (500ms interval, 0.5 seconds ON/OFF)
- **COOLING**: SOLID ON
- **ALARM**: Double-pulse pattern
  - OFF for 1000ms
  - ON for 150ms
  - OFF for 150ms
  - ON for 150ms
  - (repeat)

**Red LED (LD3) - Alarm:**
- **IDLE/MONITORING/COOLING**: OFF
- **ALARM**: SOLID ON

### System Timeline

1. **Startup (0-3s)**: IDLE state
   - Green LED: OFF
   - Blue LED: Blinking (1500ms)
   - Red LED: OFF
   
2. **3-23s**: MONITORING state (temp 10-20°C)
   - Green LED: OFF
   - Blue LED: Blinking (500ms)
   - Red LED: OFF
   
3. **23-83s**: COOLING state (temp 20-80°C)
   - Green LED: Blinking (speed increases with temperature)
   - Blue LED: SOLID ON
   - Red LED: OFF
   
4. **83-93s**: ALARM state (temp ≥80°C)
   - Green LED: SOLID ON
   - Blue LED: Double-pulse pattern
   - Red LED: SOLID ON
   
5. **Cycle**: At 93s, temperature resets to 10°C, returns to MONITORING

## Next Steps (Phase 2b)

- Add USART2 RX interrupt for command input (`TEMP <value>`, `RESET`)
- Implement CommsTask to parse commands
- Connect CH340 USB-UART adapter to PA2/PA3 for command terminal
