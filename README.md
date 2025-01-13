# C-ControlSystem (Control Systems Library in C) 

This library provides a modular framework for implementing various control strategies for drones, including:
- PID Control
- Model Predictive Control (MPC)
- Linear Quadratic Regulator (LQR)
- Robust Control (H-Infinity)

## Features
- Modular design for easy extension.
- Simulation-ready components.
- Integration with custom drone models.

## Folder Structure
- `include/`: Header files.
- `src/`: Source files for control algorithms and utilities.
- `tests/`: Unit tests for each module.
- `examples/`: Example applications demonstrating library usage.

## Getting Started
1. Clone the repository:
   ```bash
   git clone https://github.com/antshiv/controlSystems.git
   cd control_systems
   ```
2. Build the library:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```
3. Run examples:
   ```bash
   ./examples/example_pid
   ```

## Dependencies
- C
- CMake 3.10+
- Optional: [GNU Scientific Library (GSL)](https://www.gnu.org/software/gsl/) for advanced numerical computations.

## Contributing
Feel free to contribute by creating a pull request or opening an issue.