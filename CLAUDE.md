# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

CANBoard is firmware for a CAN-enabled IO board (STM32F303K8) designed for automotive DIY applications (steering wheels, button boxes, panels). It uses ChibiOS RTOS and communicates via CAN bus at configurable bitrates (default 500 Kbps).

**Hardware specs:** 8 digital inputs, 5 analog inputs (0-5V), 4 digital outputs (low-side switch), CAN bus interface.

## Build Commands

```bash
# Build (default board: canboard_v2)
make

# Build for specific board
make BOARD=canboard_v2

# Clean build artifacts
make clean
```

Toolchain: `arm-none-eabi-gcc`. ChibiOS is a git submodule — ensure it is initialized before building:
```bash
git submodule update --init --recursive
```

There are no automated tests for this embedded firmware project.

## Architecture

### Thread Model (ChibiOS)
- **CanboardThread** (`core/canboard.cpp`): Main 2ms cyclic update — processes CAN RX, updates all I/O and logic functions
- **CanRxThread**: Receives CAN frames, posts to RX mailbox
- **CanTxThread**: Transmits queued frames from TX mailbox (30µs spacing)
- **CanCyclicTxThread**: Periodic status message transmission

### Configuration System
Dual-buffer pattern: `stConfig` (active config) and `stConfigTemp` (staging). Config is persisted to flash via `core/config.cpp`. Remote configuration uses a parameter protocol over CAN:

- **`core/param_defs.h`**: Compile-time parameter table using X-macro pattern. Each parameter has an index/subindex, pointer to both active and temp value, type, and min/max/default.
- **`core/param_registry.cpp`**: Runtime lookup (`FindParam`), read/write (`ReadParam`/`WriteParam`) with type safety.
- **`core/config_handler.cpp`**: Applies temp config to active config and triggers re-initialization.

Parameter index space:
- `0x0000`: Device config (base ID, CAN speed, filter)
- `0x1200–0x120F`: Digital inputs
- `0x1300–0x130F`: CAN inputs
- `0x1400–0x140F`: Virtual inputs
- `0x1500–0x150F`: Conditions
- `0x1600–0x160F`: Counters
- `0x1700–0x170F`: Flashers
- `0x2000–0x200F`: CAN outputs
- `0x2100–0x210F`: Digital outputs
- `0x2200–0x220F`: Analog inputs

### Variable Map (`pVarMap`)
A flat array (`core/canboard.h`) of pointers to all I/O values, initialized in `InitVarMap()`. Used by logic functions (VirtualInput, Condition, Counter, etc.) to reference any input/output value by index without direct coupling.

### Logic Functions (`functions/`)
Each function type has a class with an `Update()` method called from the main cyclic thread:
- **Analog_Input**: 12-bit ADC, configurable as digital threshold
- **Digital_Input**: Debounced, supports momentary/toggle modes
- **Digital_Output**: Low-side switch, driven by VarMap index
- **CanInput**: Parses CAN frames with configurable bit fields, factor/offset, byte order
- **CanOutputs**: Maps VarMap values to CAN frames on a configurable interval
- **VirtualInput**: Boolean combination (AND/OR/XOR) of up to 3 VarMap inputs
- **Flasher**: Timed on/off blinking driven by an input
- **Counter**: Inc/dec/reset with edge detection, configurable wrap-around
- **Condition**: Threshold/comparison evaluation of a VarMap input

### Board Abstraction (`boards/canboard_v2/`)
- **`port.h`**: Defines `NUM_DIG_INPUTS`, `NUM_ANALOG_INPUTS`, etc. and `VAR_MAP_SIZE`
- **`port.cpp`**: ADC init/read, CAN bitrate configs (125K/250K/500K/1M)
- **`params.h`**: Expands parameter macros for this board's instance counts
- **`hw_devices.h/.cpp`**: Declares/instantiates hardware device arrays

### CAN Communication (`comms/`)
- **`can.cpp`**: Driver setup, filter management, thread lifecycle
- **`infomsg.cpp`**: One-shot event messages (sent once when trigger becomes true)
- **`request_msg.cpp`**: Handles incoming parameter read/write requests
- **`mailbox.cpp`**: Thread-safe frame queuing between RX/TX threads and application

## Adding a New Function Type

1. Add config struct to `core/config.h` and include in `CanboardConfig`
2. Add parameter entries to `core/param_defs.h` using the X-macro pattern
3. Implement class in `functions/`
4. Register values in `InitVarMap()` in `core/canboard.cpp`
5. Add `Update()` call in `CyclicUpdate()`
6. Add board instance count to `boards/canboard_v2/port.h`
7. Expand parameter macros in `boards/canboard_v2/params.h`

## Adding a New Board

Create a new directory under `boards/` mirroring `boards/canboard_v2/`. At minimum implement `port.h` (instance counts), `port.cpp` (HAL), `params.h` (parameter expansion), `hw_devices.h/.cpp`, and `board.mk`. Build with `make BOARD=<new_board>`.
