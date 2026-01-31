# Modern Control Project

A comprehensive project implementing modern control systems theory with practical applications on Arduino microcontrollers. This repository demonstrates the design, simulation, and real-world implementation of control systems, including PID control strategies for line-following robots and other automated systems.

## Project Overview

This project combines theoretical control system design with practical embedded systems implementation. It includes MATLAB simulations for system analysis and transfer function modeling, Arduino code for real-time control execution, and hardware schematics for circuit implementation.

## Folder Structure

### Arduino Code

Contains embedded C++ code for Arduino microcontrollers implementing control algorithms.

- **arduino before PID/**: Baseline code without PID compensation. Used for initial system characterization and response testing.
- **arduino after PID/**: Optimized code with tuned PID controller implementation. Demonstrates improved system response and stability.

### MATLAB

Simulation and analysis tools for control system design and validation.

- `lineFollowerRealtime.m.txt`: Real-time simulation of line-following control system behavior.
- `matlab_code.m.txt`: General MATLAB scripts for system analysis and calculations.
- `Transfer_Function.mat`: Saved transfer function model for the controlled system.

### Hardware Design

- **fritzing diagram/**: Circuit schematics and wiring diagrams created in Fritzing for easy reference during hardware assembly.

### Documentation & Media

- **demo videos/**: Video demonstrations of the implemented control systems in action.
- **simulation/**: Additional simulation files and results.

## Key Features

- **PID Control Implementation**: Complete PID controller design and tuning for embedded systems.
- **MATLAB Integration**: Transfer function analysis and real-time simulation capabilities.
- **Hardware Schematics**: Fritzing diagrams for easy circuit reproduction.
- **Before/After Comparison**: Compare system performance with and without control compensation.

## Getting Started

1. Review the MATLAB simulations to understand the control theory and system characteristics.
2. Check the Fritzing diagram to understand the hardware setup.
3. Upload the Arduino code (start with `code_before_PID` to establish baseline, then `code_after_PID` for optimized performance).
4. Refer to demo videos for visual confirmation of expected behavior.

## Technologies Used

- **Arduino**: Real-time embedded control system
- **MATLAB**: Simulation, modeling, and analysis
- **Fritzing**: Hardware design and documentation

## License

This project is licensed under the MIT License.
