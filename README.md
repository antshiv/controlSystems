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
- `doc/`: Design notes and module primers (e.g. mixers).

## Getting Started
1. Clone the repository and pull submodules:
   ```bash
   git clone https://github.com/antshiv/controlSystems.git
   cd control_systems
   git submodule update --init --recursive
   ```
2. Build the library:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```
3. Build and run the attitude-hold example:
   ```bash
   cmake --build build --target attitude_hold_example
   ./attitude_hold_example
   ```

4. (Optional) Run tests:
   ```bash
   ctest --output-on-failure
   ```

## Dependencies
- C
- CMake 3.10+
- Optional: [GNU Scientific Library (GSL)](https://www.gnu.org/software/gsl/) for advanced numerical computations.

### External libraries

This project expects external components to be checked out under `external/` as git submodules. For example:

```
controlSystems/
├── external/
│   └── attitudeMathLibrary/   # git submodule add https://github.com/antshiv/attitudeMathLibrary.git
```

The control build pulls headers/sources from `external/attitudeMathLibrary`. After cloning, always run:

```bash
git submodule update --init --recursive
```

The same pattern should be followed when integrating the state estimation or dynamics libraries—add them under `external/` and propagate include/link directives in CMake.

## Contributing
Feel free to contribute by creating a pull request or opening an issue.
