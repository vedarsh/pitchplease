/**
 * CubeSat ADCS Standard Framework (CASF)
 * Dynamics Simulation Module
 *
 * This module provides spacecraft attitude dynamics simulation capabilities
 * for testing and validating ADCS algorithms in a simulated environment.
 */

#ifndef CASF_DYNAMICS_H
#define CASF_DYNAMICS_H

#include "../control/control.h"
#include <stdbool.h>

// Dynamics model configuration
typedef struct {
    // Spacecraft physical properties
    casf_vector3_t inertia_tensor[3];     // Inertia tensor (kg*m^2)
    casf_float_t mass;                     // Spacecraft mass (kg)

    // Environmental parameters
    casf_float_t magnetic_field_strength;   // Local magnetic field strength (T)
    casf_float_t solar_pressure;           // Solar radiation pressure (N/m^2)
    casf_float_t atmospheric_density;      // Atmospheric density (kg/m^3)
    casf_float_t orbital_altitude;         // Orbital altitude (m)
    casf_float_t orbital_inclination;      // Orbital inclination (rad)

    // Simulation parameters
    casf_float_t timestep;                 // Integration timestep (s)
    uint32_t max_iterations;              // Maximum iterations
    uint8_t enable_gravity_gradient : 1;   // Enable gravity gradient torque
    uint8_t enable_magnetic_torque : 1;    // Enable magnetic torque
    uint8_t enable_solar_pressure : 1;     // Enable solar pressure torque
    uint8_t enable_atmospheric_drag : 1;   // Enable atmospheric drag torque
    uint8_t enable_disturbances : 1;       // Enable random disturbances
    uint8_t reserved : 3;
} casf_dynamics_config_t;

// Spacecraft state for dynamics simulation
typedef struct {
    // Attitude state
    casf_quaternion_t attitude;            // Current attitude quaternion
    casf_vector3_t angular_velocity;       // Angular velocity (rad/s)

    // Environmental state
    casf_vector3_t magnetic_field_eci;     // Magnetic field in ECI frame (T)
    casf_vector3_t sun_vector_eci;         // Sun vector in ECI frame (unit vector)
    casf_vector3_t position_eci;           // Position in ECI frame (m)
    casf_vector3_t velocity_eci;           // Velocity in ECI frame (m/s)

    // Forces and torques
    casf_vector3_t external_torque;        // External torque (N*m)
    casf_vector3_t control_torque;         // Control torque (N*m)
    casf_vector3_t disturbance_torque;     // Disturbance torque (N*m)

    // Timing
    uint64_t timestamp_us;                 // Current simulation time (us)

    // Flags
    uint8_t simulation_running : 1;
    uint8_t reserved : 7;
} casf_dynamics_state_t;

// Dynamics simulator context
typedef struct {
    casf_dynamics_config_t config;
    casf_dynamics_state_t state;

    // Callback function for control torque calculation
    void (*control_callback)(const casf_dynamics_state_t* state, casf_vector3_t* control_torque, void* user_data);
    void* user_data;

    // Statistics
    uint32_t step_count;
    casf_float_t simulation_time;

    // Internal state
    casf_vector3_t prev_angular_velocity;
    casf_quaternion_t prev_attitude;
} casf_dynamics_simulator_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the dynamics simulator with the given configuration
 * @param simulator Pointer to the simulator context
 * @param config Configuration parameters
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_init(casf_dynamics_simulator_t* simulator, const casf_dynamics_config_t* config);

/**
 * Register a control callback function that will be called at each simulation step
 * @param simulator Pointer to the simulator context
 * @param callback Function pointer to the control algorithm
 * @param user_data User data pointer passed to the callback
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_register_control(casf_dynamics_simulator_t* simulator, 
                                          void (*callback)(const casf_dynamics_state_t*, casf_vector3_t*, void*),
                                          void* user_data);

/**
 * Reset the simulator to initial conditions
 * @param simulator Pointer to the simulator context
 * @param initial_attitude Initial attitude quaternion (NULL for identity)
 * @param initial_angular_velocity Initial angular velocity (NULL for zero)
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_reset(casf_dynamics_simulator_t* simulator, 
                               const casf_quaternion_t* initial_attitude,
                               const casf_vector3_t* initial_angular_velocity);

/**
 * Perform a single step of the dynamics simulation
 * @param simulator Pointer to the simulator context
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_step(casf_dynamics_simulator_t* simulator);

/**
 * Run the simulation for the specified duration
 * @param simulator Pointer to the simulator context
 * @param duration_s Duration in seconds
 * @param step_callback Optional callback function called after each step
 * @param user_data User data passed to the step callback
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_run(casf_dynamics_simulator_t* simulator, 
                             casf_float_t duration_s,
                             void (*step_callback)(const casf_dynamics_state_t*, void*),
                             void* user_data);

/**
 * Calculate the gravitational gradient torque
 * @param simulator Pointer to the simulator context
 * @param torque Output torque vector
 */
void casf_dynamics_gravity_gradient_torque(const casf_dynamics_simulator_t* simulator, casf_vector3_t* torque);

/**
 * Calculate the magnetic torque
 * @param simulator Pointer to the simulator context
 * @param magnetic_moment Magnetic moment of the spacecraft (A*m^2)
 * @param torque Output torque vector
 */
void casf_dynamics_magnetic_torque(const casf_dynamics_simulator_t* simulator,
                                const casf_vector3_t* magnetic_moment,
                                casf_vector3_t* torque);

/**
 * Set the magnetic field in the Earth-Centered Inertial (ECI) frame
 * @param simulator Pointer to the simulator context
 * @param magnetic_field Magnetic field vector in ECI (T)
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_set_magnetic_field(casf_dynamics_simulator_t* simulator,
                                            const casf_vector3_t* magnetic_field);

/**
 * Get the magnetic field in the body frame
 * @param simulator Pointer to the simulator context
 * @param magnetic_field_body Output magnetic field vector in body frame
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_get_magnetic_field_body(const casf_dynamics_simulator_t* simulator,
                                                 casf_vector3_t* magnetic_field_body);

/**
 * Add a disturbance torque to the simulation
 * @param simulator Pointer to the simulator context
 * @param torque Disturbance torque to add (N*m)
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_add_disturbance(casf_dynamics_simulator_t* simulator,
                                         const casf_vector3_t* torque);

/**
 * Compute the attitude error between current and target attitude
 * @param current_attitude Current attitude quaternion
 * @param target_attitude Target attitude quaternion
 * @param error_quaternion Output error quaternion
 */
void casf_dynamics_attitude_error(const casf_quaternion_t* current_attitude,
                               const casf_quaternion_t* target_attitude,
                               casf_quaternion_t* error_quaternion);

/**
 * Convert the dynamics state to CASF system state
 * @param simulator Pointer to the simulator context
 * @param system_state Output system state
 * @return CASF_OK on success, error code otherwise
 */
casf_result_t casf_dynamics_to_system_state(const casf_dynamics_simulator_t* simulator,
                                         casf_system_state_t* system_state);

#ifdef __cplusplus
}
#endif

#endif // CASF_DYNAMICS_H
