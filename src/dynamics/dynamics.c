/**
 * CubeSat ADCS Standard Framework (CASF)
 * Dynamics Simulation Module - Implementation
 */

#include "dynamics.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

// Earth's gravitational parameter (m^3/s^2)
#define EARTH_MU 3.986004418e14

// Earth's radius (m)
#define EARTH_RADIUS 6371000.0

// RK4 integration constants
static const casf_float_t RK4_COEFF[] = {CASF_F(0.5f), CASF_F(0.5f), CASF_F(1.0f)};

// Safe utility functions to prevent NaN and Inf
static inline casf_float_t safe_divide(casf_float_t a, casf_float_t b) {
    if (fabsf(CASF_TO_F(b)) < 1e-10f) {
        return a >= CASF_F(0.0f) ? CASF_F(1e10f) : CASF_F(-1e10f);
    }
    return a / b;
}

// Bound function to prevent numerical instability
static inline casf_float_t bound(casf_float_t value, casf_float_t min, casf_float_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Generate a random disturbance torque component
static casf_float_t random_disturbance(casf_float_t scale) {
    float r = (float)rand() / RAND_MAX * 2.0f - 1.0f;
    return CASF_F(r * CASF_TO_F(scale));
}

// Initialize the dynamics simulator
casf_result_t casf_dynamics_init(casf_dynamics_simulator_t* simulator, const casf_dynamics_config_t* config) {
    if (!simulator || !config) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Clear the simulator structure
    memset(simulator, 0, sizeof(casf_dynamics_simulator_t));

    // Copy configuration
    simulator->config = *config;

    // Set default timestep if not specified
    if (simulator->config.timestep <= CASF_F(0.0f)) {
        simulator->config.timestep = CASF_F(0.01f); // 10ms default
    }

    // Initialize state to identity quaternion and zero angular velocity
    simulator->state.attitude.w = CASF_F(1.0f);
    simulator->state.attitude.x = CASF_F(0.0f);
    simulator->state.attitude.y = CASF_F(0.0f);
    simulator->state.attitude.z = CASF_F(0.0f);

    // Default orbital parameters if not set
    if (simulator->config.orbital_altitude <= CASF_F(0.0f)) {
        simulator->config.orbital_altitude = CASF_F(500000.0f); // 500 km default
    }

    // Set Earth's magnetic field strength if not specified
    if (simulator->config.magnetic_field_strength <= CASF_F(0.0f)) {
        simulator->config.magnetic_field_strength = CASF_F(3e-5f); // ~30 microTesla
    }

    return CASF_OK;
}

// Register control callback
casf_result_t casf_dynamics_register_control(casf_dynamics_simulator_t* simulator, 
                                          void (*callback)(const casf_dynamics_state_t*, casf_vector3_t*, void*),
                                          void* user_data) {
    if (!simulator || !callback) {
        return CASF_ERROR_INVALID_PARAM;
    }

    simulator->control_callback = callback;
    simulator->user_data = user_data;

    return CASF_OK;
}

// Reset the simulator
casf_result_t casf_dynamics_reset(casf_dynamics_simulator_t* simulator, 
                               const casf_quaternion_t* initial_attitude,
                               const casf_vector3_t* initial_angular_velocity) {
    if (!simulator) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Reset simulation time and counters
    simulator->step_count = 0;
    simulator->simulation_time = CASF_F(0.0f);
    simulator->state.timestamp_us = 0;

    // Reset attitude
    if (initial_attitude) {
        simulator->state.attitude = *initial_attitude;

        // Normalize quaternion to prevent drift
        casf_float_t norm_sq = initial_attitude->w * initial_attitude->w +
                             initial_attitude->x * initial_attitude->x +
                             initial_attitude->y * initial_attitude->y +
                             initial_attitude->z * initial_attitude->z;

        if (fabsf(CASF_TO_F(norm_sq) - 1.0f) > 1e-6f) {
            casf_float_t norm = sqrtf(CASF_TO_F(norm_sq));
            simulator->state.attitude.w = CASF_F(CASF_TO_F(initial_attitude->w) / norm);
            simulator->state.attitude.x = CASF_F(CASF_TO_F(initial_attitude->x) / norm);
            simulator->state.attitude.y = CASF_F(CASF_TO_F(initial_attitude->y) / norm);
            simulator->state.attitude.z = CASF_F(CASF_TO_F(initial_attitude->z) / norm);
        }
    } else {
        // Default to identity quaternion
        simulator->state.attitude.w = CASF_F(1.0f);
        simulator->state.attitude.x = CASF_F(0.0f);
        simulator->state.attitude.y = CASF_F(0.0f);
        simulator->state.attitude.z = CASF_F(0.0f);
    }

    // Reset angular velocity
    if (initial_angular_velocity) {
        simulator->state.angular_velocity = *initial_angular_velocity;
    } else {
        simulator->state.angular_velocity.x = CASF_F(0.0f);
        simulator->state.angular_velocity.y = CASF_F(0.0f);
        simulator->state.angular_velocity.z = CASF_F(0.0f);
    }

    // Reset torques
    memset(&simulator->state.external_torque, 0, sizeof(casf_vector3_t));
    memset(&simulator->state.control_torque, 0, sizeof(casf_vector3_t));
    memset(&simulator->state.disturbance_torque, 0, sizeof(casf_vector3_t));

    // Set running flag
    simulator->state.simulation_running = 1;

    // Save current state as previous state
    simulator->prev_attitude = simulator->state.attitude;
    simulator->prev_angular_velocity = simulator->state.angular_velocity;

    return CASF_OK;
}

// Calculate angular acceleration from torques and inertia
static void calculate_angular_acceleration(const casf_dynamics_simulator_t* simulator, casf_vector3_t* angular_accel) {
    // Calculate total torque
    casf_vector3_t total_torque;
    total_torque.x = simulator->state.control_torque.x + 
                   simulator->state.external_torque.x + 
                   simulator->state.disturbance_torque.x;
    total_torque.y = simulator->state.control_torque.y + 
                   simulator->state.external_torque.y + 
                   simulator->state.disturbance_torque.y;
    total_torque.z = simulator->state.control_torque.z + 
                   simulator->state.external_torque.z + 
                   simulator->state.disturbance_torque.z;

    // Calculate angular velocity cross product with inertia
    casf_vector3_t inertia_omega;
    inertia_omega.x = simulator->config.inertia_tensor[0].x * simulator->state.angular_velocity.x +
                    simulator->config.inertia_tensor[0].y * simulator->state.angular_velocity.y +
                    simulator->config.inertia_tensor[0].z * simulator->state.angular_velocity.z;
    inertia_omega.y = simulator->config.inertia_tensor[1].x * simulator->state.angular_velocity.x +
                    simulator->config.inertia_tensor[1].y * simulator->state.angular_velocity.y +
                    simulator->config.inertia_tensor[1].z * simulator->state.angular_velocity.z;
    inertia_omega.z = simulator->config.inertia_tensor[2].x * simulator->state.angular_velocity.x +
                    simulator->config.inertia_tensor[2].y * simulator->state.angular_velocity.y +
                    simulator->config.inertia_tensor[2].z * simulator->state.angular_velocity.z;

    casf_vector3_t omega_cross_inertia_omega;
    omega_cross_inertia_omega.x = simulator->state.angular_velocity.y * inertia_omega.z - 
                                simulator->state.angular_velocity.z * inertia_omega.y;
    omega_cross_inertia_omega.y = simulator->state.angular_velocity.z * inertia_omega.x - 
                                simulator->state.angular_velocity.x * inertia_omega.z;
    omega_cross_inertia_omega.z = simulator->state.angular_velocity.x * inertia_omega.y - 
                                simulator->state.angular_velocity.y * inertia_omega.x;

    // Euler's equations: α = I⁻¹ * (τ - ω × (I × ω))
    // First calculate T - ω × (I × ω)
    casf_vector3_t net_torque;
    net_torque.x = total_torque.x - omega_cross_inertia_omega.x;
    net_torque.y = total_torque.y - omega_cross_inertia_omega.y;
    net_torque.z = total_torque.z - omega_cross_inertia_omega.z;

    // Calculate inverse inertia tensor (simplified for diagonal inertia tensor)
    casf_float_t inv_ixx = safe_divide(CASF_F(1.0f), simulator->config.inertia_tensor[0].x);
    casf_float_t inv_iyy = safe_divide(CASF_F(1.0f), simulator->config.inertia_tensor[1].y);
    casf_float_t inv_izz = safe_divide(CASF_F(1.0f), simulator->config.inertia_tensor[2].z);

    // Calculate angular acceleration
    angular_accel->x = inv_ixx * net_torque.x;
    angular_accel->y = inv_iyy * net_torque.y;
    angular_accel->z = inv_izz * net_torque.z;

    // Bound acceleration to prevent numerical issues
    casf_float_t max_accel = CASF_F(10.0f); // 10 rad/s²
    angular_accel->x = bound(angular_accel->x, -max_accel, max_accel);
    angular_accel->y = bound(angular_accel->y, -max_accel, max_accel);
    angular_accel->z = bound(angular_accel->z, -max_accel, max_accel);
}

// Update quaternion from angular velocity
static void update_quaternion(const casf_quaternion_t* q, const casf_vector3_t* omega, casf_float_t dt, casf_quaternion_t* result) {
    // Calculate quaternion derivative: q̇ = 0.5 * q ⊗ [0, ω]
    casf_float_t half_dt = CASF_F(0.5f) * dt;
    casf_float_t wx_half_dt = omega->x * half_dt;
    casf_float_t wy_half_dt = omega->y * half_dt;
    casf_float_t wz_half_dt = omega->z * half_dt;

    casf_float_t q_w = q->w;
    casf_float_t q_x = q->x;
    casf_float_t q_y = q->y;
    casf_float_t q_z = q->z;

    // Calculate new quaternion using first-order integration
    result->w = q_w - wx_half_dt * q_x - wy_half_dt * q_y - wz_half_dt * q_z;
    result->x = q_x + wx_half_dt * q_w + wy_half_dt * q_z - wz_half_dt * q_y;
    result->y = q_y - wx_half_dt * q_z + wy_half_dt * q_w + wz_half_dt * q_x;
    result->z = q_z + wx_half_dt * q_y - wy_half_dt * q_x + wz_half_dt * q_w;

    // Normalize quaternion to prevent drift
    casf_float_t norm_sq = result->w * result->w +
                         result->x * result->x +
                         result->y * result->y +
                         result->z * result->z;

    if (fabsf(CASF_TO_F(norm_sq) - 1.0f) > 1e-6f) {
        casf_float_t norm = sqrtf(CASF_TO_F(norm_sq));
        result->w = CASF_F(CASF_TO_F(result->w) / norm);
        result->x = CASF_F(CASF_TO_F(result->x) / norm);
        result->y = CASF_F(CASF_TO_F(result->y) / norm);
        result->z = CASF_F(CASF_TO_F(result->z) / norm);
    }
}

// Perform RK4 integration for angular velocity
static void rk4_integrate_angular_velocity(casf_dynamics_simulator_t* simulator, casf_float_t dt) {
    casf_vector3_t k1, k2, k3, k4;
    casf_vector3_t temp_omega;
    casf_quaternion_t temp_q;

    // Original state
    casf_vector3_t omega0 = simulator->state.angular_velocity;
    casf_quaternion_t q0 = simulator->state.attitude;

    // Calculate k1 (initial derivative)
    calculate_angular_acceleration(simulator, &k1);

    // Calculate k2 (derivative at t + dt/2 using k1)
    temp_omega.x = omega0.x + k1.x * dt * RK4_COEFF[0];
    temp_omega.y = omega0.y + k1.y * dt * RK4_COEFF[0];
    temp_omega.z = omega0.z + k1.z * dt * RK4_COEFF[0];
    update_quaternion(&q0, &temp_omega, dt * RK4_COEFF[0], &temp_q);

    simulator->state.angular_velocity = temp_omega;
    simulator->state.attitude = temp_q;
    calculate_angular_acceleration(simulator, &k2);

    // Calculate k3 (derivative at t + dt/2 using k2)
    temp_omega.x = omega0.x + k2.x * dt * RK4_COEFF[1];
    temp_omega.y = omega0.y + k2.y * dt * RK4_COEFF[1];
    temp_omega.z = omega0.z + k2.z * dt * RK4_COEFF[1];
    update_quaternion(&q0, &temp_omega, dt * RK4_COEFF[1], &temp_q);

    simulator->state.angular_velocity = temp_omega;
    simulator->state.attitude = temp_q;
    calculate_angular_acceleration(simulator, &k3);

    // Calculate k4 (derivative at t + dt using k3)
    temp_omega.x = omega0.x + k3.x * dt * RK4_COEFF[2];
    temp_omega.y = omega0.y + k3.y * dt * RK4_COEFF[2];
    temp_omega.z = omega0.z + k3.z * dt * RK4_COEFF[2];
    update_quaternion(&q0, &temp_omega, dt * RK4_COEFF[2], &temp_q);

    simulator->state.angular_velocity = temp_omega;
    simulator->state.attitude = temp_q;
    calculate_angular_acceleration(simulator, &k4);

    // Calculate final angular velocity using weighted average of k1, k2, k3, k4
    simulator->state.angular_velocity.x = omega0.x + dt * (k1.x + CASF_F(2.0f)*k2.x + CASF_F(2.0f)*k3.x + k4.x) / CASF_F(6.0f);
    simulator->state.angular_velocity.y = omega0.y + dt * (k1.y + CASF_F(2.0f)*k2.y + CASF_F(2.0f)*k3.y + k4.y) / CASF_F(6.0f);
    simulator->state.angular_velocity.z = omega0.z + dt * (k1.z + CASF_F(2.0f)*k2.z + CASF_F(2.0f)*k3.z + k4.z) / CASF_F(6.0f);

    // Update final quaternion using RK4 result
    update_quaternion(&q0, &simulator->state.angular_velocity, dt, &simulator->state.attitude);
}

// Perform a single step of the dynamics simulation
casf_result_t casf_dynamics_step(casf_dynamics_simulator_t* simulator) {
    if (!simulator || !simulator->state.simulation_running) {
        return CASF_ERROR_NOT_INITIALIZED;
    }

    // Save previous state
    simulator->prev_attitude = simulator->state.attitude;
    simulator->prev_angular_velocity = simulator->state.angular_velocity;

    // Generate random disturbance if enabled
    if (simulator->config.enable_disturbances) {
        casf_vector3_t disturbance;
        disturbance.x = random_disturbance(CASF_F(1e-7f)); // Small random torque
        disturbance.y = random_disturbance(CASF_F(1e-7f));
        disturbance.z = random_disturbance(CASF_F(1e-7f));
        casf_dynamics_add_disturbance(simulator, &disturbance);
    }

    // Call the control callback if registered
    if (simulator->control_callback) {
        casf_vector3_t control_torque = {CASF_F(0.0f), CASF_F(0.0f), CASF_F(0.0f)};
        simulator->control_callback(&simulator->state, &control_torque, simulator->user_data);
        simulator->state.control_torque = control_torque;
    }

    // Integrate equations of motion using RK4
    rk4_integrate_angular_velocity(simulator, simulator->config.timestep);

    // Update timestamp and simulation time
    simulator->state.timestamp_us += (uint64_t)(CASF_TO_F(simulator->config.timestep) * 1e6f);
    simulator->simulation_time += simulator->config.timestep;
    simulator->step_count++;

    return CASF_OK;
}

// Run the simulation for a specified duration
casf_result_t casf_dynamics_run(casf_dynamics_simulator_t* simulator, 
                             casf_float_t duration_s,
                             void (*step_callback)(const casf_dynamics_state_t*, void*),
                             void* user_data) {
    if (!simulator || duration_s <= CASF_F(0.0f)) {
        return CASF_ERROR_INVALID_PARAM;
    }

    casf_float_t end_time = simulator->simulation_time + duration_s;
    casf_result_t result;

    while (simulator->simulation_time < end_time && 
           simulator->step_count < simulator->config.max_iterations) {
        result = casf_dynamics_step(simulator);
        if (result != CASF_OK) {
            return result;
        }

        if (step_callback) {
            step_callback(&simulator->state, user_data);
        }
    }

    return CASF_OK;
}

// Calculate the gravitational gradient torque
void casf_dynamics_gravity_gradient_torque(const casf_dynamics_simulator_t* simulator, casf_vector3_t* torque) {
    if (!simulator || !torque) return;

    // Need position vector for gravity gradient torque
    if (casf_vector3_magnitude(&simulator->state.position_eci) <= CASF_F(0.0f)) {
        // No position data, return zero torque
        torque->x = CASF_F(0.0f);
        torque->y = CASF_F(0.0f);
        torque->z = CASF_F(0.0f);
        return;
    }

    // Calculate nadir vector in body frame
    casf_vector3_t nadir_eci;
    casf_float_t pos_magnitude = casf_vector3_magnitude(&simulator->state.position_eci);
    nadir_eci.x = -simulator->state.position_eci.x / pos_magnitude;
    nadir_eci.y = -simulator->state.position_eci.y / pos_magnitude;
    nadir_eci.z = -simulator->state.position_eci.z / pos_magnitude;

    // Convert to body frame using quaternion rotation
    casf_vector3_t nadir_body;
    // Quaternion rotation implementation omitted for brevity
    // This would convert the nadir_eci to nadir_body using the attitude quaternion

    // Calculate gravity gradient torque: T = 3μ/r³ * r̂ × (I·r̂)
    casf_float_t r_cubed = pos_magnitude * pos_magnitude * pos_magnitude;
    casf_float_t factor = CASF_F(3.0f * EARTH_MU) / r_cubed;

    // Calculate I·r̂
    casf_vector3_t i_dot_r;
    i_dot_r.x = simulator->config.inertia_tensor[0].x * nadir_body.x +
               simulator->config.inertia_tensor[0].y * nadir_body.y +
               simulator->config.inertia_tensor[0].z * nadir_body.z;
    i_dot_r.y = simulator->config.inertia_tensor[1].x * nadir_body.x +
               simulator->config.inertia_tensor[1].y * nadir_body.y +
               simulator->config.inertia_tensor[1].z * nadir_body.z;
    i_dot_r.z = simulator->config.inertia_tensor[2].x * nadir_body.x +
               simulator->config.inertia_tensor[2].y * nadir_body.y +
               simulator->config.inertia_tensor[2].z * nadir_body.z;

    // Calculate r̂ × (I·r̂)
    torque->x = factor * (nadir_body.y * i_dot_r.z - nadir_body.z * i_dot_r.y);
    torque->y = factor * (nadir_body.z * i_dot_r.x - nadir_body.x * i_dot_r.z);
    torque->z = factor * (nadir_body.x * i_dot_r.y - nadir_body.y * i_dot_r.x);
}

// Calculate the magnetic torque
void casf_dynamics_magnetic_torque(const casf_dynamics_simulator_t* simulator,
                                const casf_vector3_t* magnetic_moment,
                                casf_vector3_t* torque) {
    if (!simulator || !magnetic_moment || !torque) return;

    // Get magnetic field in body frame
    casf_vector3_t B_body;
    if (casf_dynamics_get_magnetic_field_body(simulator, &B_body) != CASF_OK) {
        torque->x = CASF_F(0.0f);
        torque->y = CASF_F(0.0f);
        torque->z = CASF_F(0.0f);
        return;
    }

    // Calculate magnetic torque: T = m × B
    torque->x = magnetic_moment->y * B_body.z - magnetic_moment->z * B_body.y;
    torque->y = magnetic_moment->z * B_body.x - magnetic_moment->x * B_body.z;
    torque->z = magnetic_moment->x * B_body.y - magnetic_moment->y * B_body.x;
}

// Set the magnetic field in ECI frame
casf_result_t casf_dynamics_set_magnetic_field(casf_dynamics_simulator_t* simulator,
                                            const casf_vector3_t* magnetic_field) {
    if (!simulator || !magnetic_field) {
        return CASF_ERROR_INVALID_PARAM;
    }

    simulator->state.magnetic_field_eci = *magnetic_field;
    return CASF_OK;
}

// Get the magnetic field in body frame
casf_result_t casf_dynamics_get_magnetic_field_body(const casf_dynamics_simulator_t* simulator,
                                                 casf_vector3_t* magnetic_field_body) {
    if (!simulator || !magnetic_field_body) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Convert magnetic field from ECI to body frame using quaternion rotation
    // For brevity, using a simplified implementation
    casf_quaternion_t q = simulator->state.attitude;
    casf_vector3_t v = simulator->state.magnetic_field_eci;

    // q * v * q^-1 rotation
    casf_float_t qw = q.w;
    casf_float_t qx = q.x;
    casf_float_t qy = q.y;
    casf_float_t qz = q.z;

    // Optimized quaternion rotation
    casf_float_t uv_x = CASF_F(2.0f) * (qy * v.z - qz * v.y);
    casf_float_t uv_y = CASF_F(2.0f) * (qz * v.x - qx * v.z);
    casf_float_t uv_z = CASF_F(2.0f) * (qx * v.y - qy * v.x);

    magnetic_field_body->x = v.x + qw * uv_x + (qy * uv_z - qz * uv_y);
    magnetic_field_body->y = v.y + qw * uv_y + (qz * uv_x - qx * uv_z);
    magnetic_field_body->z = v.z + qw * uv_z + (qx * uv_y - qy * uv_x);

    return CASF_OK;
}

// Add a disturbance torque
casf_result_t casf_dynamics_add_disturbance(casf_dynamics_simulator_t* simulator,
                                         const casf_vector3_t* torque) {
    if (!simulator || !torque) {
        return CASF_ERROR_INVALID_PARAM;
    }

    simulator->state.disturbance_torque.x += torque->x;
    simulator->state.disturbance_torque.y += torque->y;
    simulator->state.disturbance_torque.z += torque->z;

    return CASF_OK;
}

// Compute attitude error
void casf_dynamics_attitude_error(const casf_quaternion_t* current_attitude,
                               const casf_quaternion_t* target_attitude,
                               casf_quaternion_t* error_quaternion) {
    if (!current_attitude || !target_attitude || !error_quaternion) return;

    // Calculate inverse of current quaternion (conjugate since quaternions are unit length)
    casf_quaternion_t q_inv;
    q_inv.w = current_attitude->w;
    q_inv.x = -current_attitude->x;
    q_inv.y = -current_attitude->y;
    q_inv.z = -current_attitude->z;

    // Calculate error quaternion = q_target * q_current^-1
    casf_quaternion_multiply(target_attitude, &q_inv, error_quaternion);
}

// Convert dynamics state to CASF system state
casf_result_t casf_dynamics_to_system_state(const casf_dynamics_simulator_t* simulator,
                                         casf_system_state_t* system_state) {
    if (!simulator || !system_state) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Copy attitude and angular velocity
    system_state->attitude_q = simulator->state.attitude;
    system_state->angular_velocity = simulator->state.angular_velocity;

    // Get magnetic field in body frame
    casf_dynamics_get_magnetic_field_body(simulator, &system_state->magnetic_field_body);

    // Set validity flags
    system_state->attitude_valid = 1;
    system_state->rates_valid = 1;
    system_state->mag_valid = 1;

    // Set timestamp
    system_state->timestamp_us = simulator->state.timestamp_us;

    return CASF_OK;
}
