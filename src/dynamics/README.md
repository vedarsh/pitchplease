# CASF Dynamics Module

The Dynamics module provides spacecraft attitude dynamics simulation capabilities for testing and validating ADCS algorithms in a simulated environment.

## Features

- High-fidelity rigid body attitude dynamics simulation
- RK4 numerical integration for accurate propagation
- Environmental disturbance modeling including:
  - Gravity gradient torque
  - Magnetic torque
  - Solar radiation pressure
  - Atmospheric drag
- Control algorithm interface for closed-loop testing
- Realistic sensor simulation

## Usage

```c
// Configure dynamics simulator
casf_dynamics_config_t config = {0};
config.timestep = CASF_F(0.01f);  // 10ms simulation step
config.enable_gravity_gradient = 1;
config.enable_magnetic_torque = 1;
config.enable_disturbances = 1;

// Set spacecraft properties
config.inertia_tensor[0].x = CASF_F(0.01f);  // Ixx
config.inertia_tensor[1].y = CASF_F(0.02f);  // Iyy
config.inertia_tensor[2].z = CASF_F(0.03f);  // Izz

// Initialize simulator
casf_dynamics_simulator_t simulator;
casf_dynamics_init(&simulator, &config);

// Configure initial conditions (tumbling spacecraft)
casf_vector3_t initial_angular_velocity = {
    CASF_F(0.1f),   // 0.1 rad/s around x
    CASF_F(0.05f),  // 0.05 rad/s around y
    CASF_F(-0.02f)  // -0.02 rad/s around z
};

casf_dynamics_reset(&simulator, NULL, &initial_angular_velocity);

// Register control algorithm
casf_dynamics_register_control(&simulator, detumble_control_callback, user_data);

// Run simulation
casf_dynamics_run(&simulator, CASF_F(60.0f), data_logging_callback, log_data);
```

## Dynamics Model

The simulator implements Euler's rotational equations of motion for a rigid body:

```
I·α + ω × (I·ω) = τ
```

Where:
- I is the inertia tensor
- α is angular acceleration
- ω is angular velocity
- τ is the total torque

The quaternion kinematics equation is used for attitude propagation:

```
dq/dt = 0.5 * q ⊗ [0, ω]
```

RK4 numerical integration is used to solve these differential equations.

## Safety Features

The dynamics module includes several safeguards to prevent numerical instability:

- Quaternion normalization at each step to prevent drift
- Angular velocity and torque limiting to prevent numerical issues
- Protection against division by zero
- Bounds checking on all inputs
