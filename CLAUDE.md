# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**RM2024_Reborn_LegWheel_Chassis** is a leg-wheel balancing robot control system for RoboMaster competitions. The project implements balance and motion control using LQR algorithms on an STM32F446 microcontroller.

## Build System

The project uses **CMake** with GCC ARM toolchain for cross-compilation. Key build commands:

```bash
# Configure with CMake presets
cmake --preset debug      # Debug build
cmake --preset release    # Release build

# Build the configured project
cmake --build out/build/debug
cmake --build out/build/release

# Direct CMake approach
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../arm-gcc-toolchain.cmake
make
```

**Output files** (in build directory):
- `EC_DM_Board.elf` - ELF executable
- `EC_DM_Board.hex`, `EC_DM_Board.bin` - Flashing formats
- `EC_DM_Board.map` - Memory map

## Hardware Platform

- **MCU**: STM32F446 (达妙 MC01 control board)
- **IMU**: BMI088 gyroscope
- **Hip motors**: Unitree A1 × 4 (serial dual-DMA communication, 0.32ms task duration)
- **Wheel motors**: Lingkong MF9025-16T × 2

## Code Architecture

### Core Directories

```
user/                          # Main application code
├── algorithm/                 # Control algorithms
│   ├── inc/                  # Headers: lqr.h, kalman_filter.h, pid.h, etc.
│   └── src/                  # Implementations
│       ├── lqr.cpp           # LQR controller (primary balance control)
│       ├── kalman_filter.cpp # Speed fusion Kalman filter
│       ├── pid.cpp           # PID controller
│       └── quaternion_ekf.cpp # Attitude estimation
├── application/               # Top-level tasks (1kHz core frequency)
│   ├── inc/
│   └── src/
│       ├── chassis.cpp       # Main chassis control logic
│       ├── infantry_chassis.cpp # Infantry chassis control
│       └── ins.cpp           # Inertial navigation system
├── bsp/                       # Board support package (HAL wrappers)
└── modules/                   # Peripheral drivers
    ├── inc/
    └── src/
        ├── unitree.cpp       # Unitree A1 motor driver
        ├── mf9025.cpp        # MF9025 wheel motor driver
        ├── bmi088_driver.cpp # BMI088 IMU driver
        └── referee.cpp       # Referee system communication

k_calc/                        # MATLAB LQR parameter calculation
├── get_k.m                   # Main LQR parameter calculation script
├── get_k_length.m            # Leg-length specific LQR computation
└── chassis_calc_heu.m        # Chassis calculation utilities
```

### Key Control Flow

1. **Sensor Input**: IMU data (BMI088) → attitude estimation (quaternion_ekf)
2. **State Estimation**: Wheel speed + IMU → Kalman filter → fused speed
3. **LQR Control**:
   - State variables: distance, speed, phi angle, phi angular velocity, theta angle, theta angular velocity, leg length
   - Output: wheel torque (T) and leg torque (Tp)
   - Uses precomputed polynomial coefficients from MATLAB (in `lqr.cpp:k[12][4]`)
4. **Motor Control**: LQR outputs → Unitree A1 (hip) + MF9025 (wheel) motor commands
5. **Communication**: Referee system, remote control, inter-board communication

### Real-Time Performance

- Core tasks run at **1kHz** (motor comms, control, communications)
- Unitree A1 communication optimized with dual-DMA: **0.32ms** task duration
- FreeRTOS task scheduling ensures real-time requirements

## LQR Parameter Generation Workflow

LQR parameters are precomputed in MATLAB and hardcoded in C++:

1. **MATLAB Modeling**: `k_calc/get_k.m` computes LQR K matrices for leg lengths 0.08m-0.36m (0.005m steps)
2. **Polynomial Fitting**: Cubic polynomial fit for each of 12 parameters (2×6 K matrix)
3. **Code Generation**: Fit coefficients copied to `lqr.cpp:k[12][4]` array
4. **Runtime Interpolation**: `Lqr::Calc()` computes control gains based on current leg length

**Important**: When modifying LQR parameters, update BOTH:
- MATLAB scripts (`k_calc/`): For accurate modeling
- C++ code (`lqr.cpp`): For actual control

## Development Workflow

### Hardware Configuration
- Edit `EC_DM_Board.ioc` with STM32CubeMX
- Regenerate code: CubeMX → Generate Code
- Merge changes with existing user code

### Code Development
1. Edit C++ files in `user/` directory
2. Build with CMake (commands above)
3. Flash to board using OpenOCD (`.vscode/launch.json` configured)
4. Monitor via serial debug output or UI client

### MATLAB Parameter Tuning
1. Modify robot parameters in `get_k.m`/`get_k_length.m`
2. Run MATLAB scripts to compute new LQR parameters
3. Copy polynomial coefficients to `lqr.cpp`
4. Test on hardware with updated parameters

## Key Files for Common Tasks

- **Balance control tuning**: `user/algorithm/src/lqr.cpp`, `k_calc/get_k.m`
- **Motor driver changes**: `user/modules/src/unitree.cpp`, `user/modules/src/mf9025.cpp`
- **Main control logic**: `user/application/src/chassis.cpp`
- **Build configuration**: `CMakeLists.txt`, `CMakePresets.json`
- **Hardware configuration**: `EC_DM_Board.ioc` (STM32CubeMX)

## Debugging

- **VSCode debugging**: Preconfigured in `.vscode/launch.json` (OpenOCD + GDB)
- **Serial debug**: Use `bsp_usart.cpp` debug output
- **Client UI**: `client_ui.cpp` for real-time monitoring
- **Memory analysis**: Check `EC_DM_Board.map` after build

## Code Style

- C++17 with object-oriented design
- Clang-format configuration in `.clang-format`
- Header files in `inc/`, implementations in `src/`
- FreeRTOS for task management
- STM32 HAL library for hardware access

## Notes

- The project combines **balance control** (LQR) with **motion control** (speed commands)
- **Jumping functionality** is implemented for terrain traversal
- **Closed-loop steering control** improves maneuverability
- **Speed fusion** (Kalman filter) combines wheel speed and IMU data for robustness
- **Modular design** allows easy replacement of motor drivers or sensors