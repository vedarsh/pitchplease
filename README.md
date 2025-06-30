#PitchPlease Source Repo:
# PitchPlease - CubeSat ADCS Standard Framework (CASF)

CASF is a universal ADCS (Attitude Determination and Control System) interface standard for CubeSat missions, inspired by drone flight controller standards like PX4 and ArduPilot.

## Features

- Hardware abstraction layer (HAL) for platform independence
- Plugin-based sensor/actuator architecture
- Cross-platform compatibility (ARM Cortex-M, Linux, RTOS)
- Mission-agnostic control algorithms
- Standardized telemetry and command interfaces
- Support for multiple operational modes
- Comprehensive error handling and safety features

## Build Requirements

- CMake 3.10 or higher
- C11 compatible compiler
- Standard math library

## Building the Project

```bash
# Create a build directory
mkdir build && cd build

# Configure the project
cmake ..

# Build the library and examples
make

# Run the example
./casf_example

# Run the tests (if enabled)
./casf_tests
```

## Configuration Options

You can customize the build with the following options:

```bash
# Build with tests enabled
cmake -DBUILD_TESTS=ON ..

# Use fixed-point math instead of floating-point
cmake -DUSE_FIXED_POINT=ON ..

# Build with dynamics simulation module
cmake -DBUILD_DYNAMICS=ON ..
```

## Project Structure

- `/src/control` - Core ADCS framework and control algorithms
- `/src/sensors` - Sensor drivers and abstractions
- `/src/actuators` - Actuator drivers and abstractions
- `/src/dynamics` - Spacecraft dynamics simulation
- `/examples` - Example applications
- `/tests` - Unit tests

## Usage Example

### Basic ADCS Control

```c
#include "control.h"
#include "sensors.h"

// Initialize HAL with platform-specific functions
casf_hal_t hal = {
    .get_time_us = platform_get_time_us,
    .delay_us = platform_delay_us,
    .log = platform_log,
    // ... other HAL functions
};

// Configure the mission parameters
casf_mission_config_t config = {
    .mission_name = "My CubeSat Mission",
    .spacecraft_id = "CUBE-01",
    .default_mode = CASF_MODE_SAFE,
    .control_frequency_hz = 10
};

// Initialize the CASF system
casf_system_t sys;
casf_init(&sys, &hal, &config);

// Register sensors and actuators
casf_register_sensor(&sys, &magnetometer_driver);
casf_register_sensor(&sys, &gyroscope_driver);
casf_register_actuator(&sys, &reaction_wheel_driver);

// Start the system
casf_start(&sys);

// Main control loop
while (1) {
    // Update the ADCS system
    casf_update(&sys);

    // Process commands, telemetry, etc.
    process_commands();

    // Delay to maintain desired control rate
    hal.delay_us(100000); // 100ms
}
```

### Dynamics Simulation

```c
#include "dynamics/dynamics.h"

// Configure dynamics simulator
casf_dynamics_config_t config = {0};
config.timestep = CASF_F(0.01f);  // 10ms simulation step
config.enable_gravity_gradient = 1;
config.enable_disturbances = 1;

// Set spacecraft properties
config.inertia_tensor[0].x = CASF_F(0.01f);  // Ixx
config.inertia_tensor[1].y = CASF_F(0.02f);  // Iyy
config.inertia_tensor[2].z = CASF_F(0.03f);  // Izz

// Initialize and configure simulator
casf_dynamics_simulator_t simulator;
casf_dynamics_init(&simulator, &config);

// Set initial conditions with tumbling spacecraft
casf_vector3_t initial_rates = {CASF_F(0.1f), CASF_F(0.05f), CASF_F(-0.02f)};
casf_dynamics_reset(&simulator, NULL, &initial_rates);

// Register control algorithm callback
casf_dynamics_register_control(&simulator, detumble_controller, user_data);

// Run simulation
casf_dynamics_run(&simulator, CASF_F(600.0f), data_logger, log_data);
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.