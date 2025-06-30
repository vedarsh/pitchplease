/**
 * CubeSat ADCS Standard Framework (CASF)
 * Dynamics Simulation Module - Implementation
 */

#include "dynamics.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

// Earth's gravitational parameter (m^3/s^2)
#define EARTH_MU 3.986004418e14f

// Earth's radius (m)
#define EARTH_RADIUS 6371000.0f

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

// Calculate angular acceleration from torques and inertia
static void calculate_angular_acceleration(const casf_dynamics_simulator_t* sim, casf_vector3_t* angular_accel) {
    // Calculate total torque
    casf_vector3_t total_torque;
    total_torque.x = sim->state.control_torque.x + 
                   sim->state.external_torque.x + 
                   sim->state.disturbance_torque.x;
    total_torque.y = sim->state.control_torque.y + 
                   sim->state.external_torque.y + 
                   sim->state.disturbance_torque.y;
    total_torque.z = sim->state.control_torque.z + 
                   sim->state.external_torque.z + 
                   sim->state.disturbance_torque.z;

    // Calculate angular velocity cross product with inertia
    casf_vector3_t inertia_omega;
    inertia_omega.x = sim->config.inertia_tensor[0].x * sim->state.angular_velocity.x +
                    sim->config.inertia_tensor[0].y * sim->state.angular_velocity.y +
                    sim->config.inertia_tensor[0].z * sim->state.angular_velocity.z;
    inertia_omega.y = sim->config.inertia_tensor[1].x * sim->state.angular_velocity.x +
                    sim->config.inertia_tensor[1].y * sim->state.angular_velocity.y +
                    sim->config.inertia_tensor[1].z * sim->state.angular_velocity.z;
    inertia_omega.z = sim->config.inertia_tensor[2].x * sim->state.angular_velocity.x +
                    sim->config.inertia_tensor[2].y * sim->state.angular_velocity.y +
                    sim->config.inertia_tensor[2].z * sim->state.angular_velocity.z;

    casf_vector3_t omega_cross_inertia_omega;
    omega_cross_inertia_omega.x = sim->state.angular_velocity.y * inertia_omega.z - 
                                sim->state.angular_velocity.z * inertia_omega.y;
    omega_cross_inertia_omega.y = sim->state.angular_velocity.z * inertia_omega.x - 
                                sim->state.angular_velocity.x * inertia_omega.z;
    omega_cross_inertia_omega.z = sim->state.angular_velocity.x * inertia_omega.y - 
                                sim->state.angular_velocity.y * inertia_omega.x;

    // Euler's equations: α = I⁻¹ * (τ - ω × (I × ω))
    // First calculate T - ω × (I × ω)
    casf_vector3_t net_torque;
    net_torque.x = total_torque.x - omega_cross_inertia_omega.x;
    net_torque.y = total_torque.y - omega_cross_inertia_omega.y;
    net_torque.z = total_torque.z - omega_cross_inertia_omega.z;

    // Calculate inverse inertia tensor (simplified for diagonal inertia tensor)
    casf_float_t inv_ixx = safe_divide(CASF_F(1.0f), sim->config.inertia_tensor[0].x);
    casf_float_t inv_iyy = safe_divide(CASF_F(1.0f), sim->config.inertia_tensor[1].y);
    casf_float_t inv_izz = safe_divide(CASF_F(1.0f), sim->config.inertia_tensor[2].z);

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
static void rk4_integrate_angular_velocity(casf_dynamics_simulator_t* sim, casf_float_t dt) {
    casf_vector3_t k1, k2, k3, k4;
    casf_vector3_t temp_omega;
    casf_quaternion_t temp_q;

    // Original state
    casf_vector3_t omega0 = sim->state.angular_velocity;
    casf_quaternion_t q0 = sim->state.attitude;

    // Calculate k1 (initial derivative)
    calculate_angular_acceleration(sim, &k1);

    // Calculate k2 (derivative at t + dt/2 using k1)
    temp_omega.x = omega0.x + k1.x * dt * RK4_COEFF[0];
    temp_omega.y = omega0.y + k1.y * dt * RK4_COEFF[0];
    temp_omega.z = omega0.z + k1.z * dt * RK4_COEFF[0];
    update_quaternion(&q0, &temp_omega, dt * RK4_COEFF[0], &temp_q);

    sim->state.angular_velocity = temp_omega;
    sim->state.attitude = temp_q;
    calculate_angular_acceleration(sim, &k2);

    // Calculate k3 (derivative at t + dt/2 using k2)
    temp_omega.x = omega0.x + k2.x * dt * RK4_COEFF[1];
    temp_omega.y = omega0.y + k2.y * dt * RK4_COEFF[1];
    temp_omega.z = omega0.z + k2.z * dt * RK4_COEFF[1];
    update_quaternion(&q0, &temp_omega, dt * RK4_COEFF[1], &temp_q);

    sim->state.angular_velocity = temp_omega;
    sim->state.attitude = temp_q;
    calculate_angular_acceleration(sim, &k3);

    // Calculate k4 (derivative at t + dt using k3)
    temp_omega.x = omega0.x + k3.x * dt * RK4_COEFF[2];
    temp_omega.y = omega0.y + k3.y * dt * RK4_COEFF[2];
    temp_omega.z = omega0.z + k3.z * dt * RK4_COEFF[2];
    update_quaternion(&q0, &temp_omega, dt * RK4_COEFF[2], &temp_q);

    sim->state.angular_velocity = temp_omega;
    sim->state.attitude = temp_q;
    calculate_angular_acceleration(sim, &k4);

    // Calculate final angular velocity using weighted average of k1, k2, k3, k4
    sim->state.angular_velocity.x = omega0.x + dt * (k1.x + CASF_F(2.0f)*k2.x + CASF_F(2.0f)*k3.x + k4.x) / CASF_F(6.0f);
    sim->state.angular_velocity.y = omega0.y + dt * (k1.y + CASF_F(2.0f)*k2.y + CASF_F(2.0f)*k3.y + k4.y) / CASF_F(6.0f);
    sim->state.angular_velocity.z = omega0.z + dt * (k1.z + CASF_F(2.0f)*k2.z + CASF_F(2.0f)*k3.z + k4.z) / CASF_F(6.0f);

    // Update final quaternion using RK4 result
    update_quaternion(&q0, &sim->state.angular_velocity, dt, &sim->state.attitude);
}

/**
 * Initialize the dynamics simulator with the given configuration
 */
casf_result_t casf_dynamics_init(casf_dynamics_simulator_t* sim, const casf_dynamics_config_t* config) {
    if (!sim || !config) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Clear the simulator structure
    memset(sim, 0, sizeof(casf_dynamics_simulator_t));

    // Copy configuration
    sim->config = *config;

    // Set default timestep if not specified
    if (sim->config.timestep <= CASF_F(0.0f)) {
        sim->config.timestep = CASF_F(0.01f); // 10ms default
    }

    // Initialize state to identity quaternion and zero angular velocity
    sim->state.attitude.w = CASF_F(1.0f);
    sim->state.attitude.x = CASF_F(0.0f);
    sim->state.attitude.y = CASF_F(0.0f);
    sim->state.attitude.z = CASF_F(0.0f);

    sim->state.angular_velocity = config->initial_angular_velocity;
    sim->state.timestamp_us = 0;

    // Initialize magnetic field to Earth's nominal value
    sim->magnetic_field_eci.x = CASF_F(2.0e-5f); // 20 μT
    sim->magnetic_field_eci.y = CASF_F(0.0f);
    sim->magnetic_field_eci.z = CASF_F(4.0e-5f); // 40 μT

    sim->initialized = 1;
    return CASF_OK;
}

/**
 * Reset the simulator to initial conditions
 */
casf_result_t casf_dynamics_reset(casf_dynamics_simulator_t* sim, 
                              const casf_quaternion_t* initial_attitude,
                              const casf_vector3_t* initial_angular_velocity) {
    if (!sim || !sim->initialized) {
        return CASF_ERROR_NOT_INITIALIZED;
    }

    // Reset simulation time and counters
    sim->step_count = 0;
    sim->simulation_time = CASF_F(0.0f);
    sim->state.timestamp_us = 0;

    // Reset attitude
    if (initial_attitude) {
        sim->state.attitude = *initial_attitude;

        // Normalize quaternion to prevent drift
        casf_float_t norm_sq = initial_attitude->w * initial_attitude->w +
                             initial_attitude->x * initial_attitude->x +
                             initial_attitude->y * initial_attitude->y +
                             initial_attitude->z * initial_attitude->z;

        if (fabsf(CASF_TO_F(norm_sq) - 1.0f) > 1e-6f) {
            casf_float_t norm = sqrtf(CASF_TO_F(norm_sq));
            sim->state.attitude.w = CASF_F(CASF_TO_F(initial_attitude->w) / norm);
            sim->state.attitude.x = CASF_F(CASF_TO_F(initial_attitude->x) / norm);
            sim->state.attitude.y = CASF_F(CASF_TO_F(initial_attitude->y) / norm);
            sim->state.attitude.z = CASF_F(CASF_TO_F(initial_attitude->z) / norm);
        }
    } else {
        // Default to identity quaternion
        sim->state.attitude.w = CASF_F(1.0f);
        sim->state.attitude.x = CASF_F(0.0f);
        sim->state.attitude.y = CASF_F(0.0f);
        sim->state.attitude.z = CASF_F(0.0f);
    }

    // Reset angular velocity
    if (initial_angular_velocity) {
        sim->state.angular_velocity = *initial_angular_velocity;
    } else {
        sim->state.angular_velocity.x = CASF_F(0.0f);
        sim->state.angular_velocity.y = CASF_F(0.0f);
        sim->state.angular_velocity.z = CASF_F(0.0f);
    }

    // Reset torques
    memset(&sim->state.external_torque, 0, sizeof(casf_vector3_t));
    memset(&sim->state.control_torque, 0, sizeof(casf_vector3_t));
    memset(&sim->state.disturbance_torque, 0, sizeof(casf_vector3_t));

    // Set running flag
    sim->state.simulation_running = 1;

    return CASF_OK;
}

/**
 * Register a control callback function that will be called at each simulation step
 */
casf_result_t casf_dynamics_register_control(casf_dynamics_simulator_t* sim, 
                                          void (*callback)(const casf_dynamics_state_t*, casf_vector3_t*, void*),
                                          void* user_data) {
    if (!sim || !sim->initialized || !callback) {
        return CASF_ERROR_INVALID_PARAM;
    }

    sim->control_callback = callback;
    sim->user_data = user_data;

    return CASF_OK;
}

/**
 * Perform a single step of the dynamics simulation
 */
casf_result_t casf_dynamics_step(casf_dynamics_simulator_t* sim) {
    if (!sim || !sim->initialized) {
        return CASF_ERROR_NOT_INITIALIZED;
    }

    // Generate random disturbance if enabled
    if (sim->config.enable_disturbances) {
        casf_vector3_t disturbance;
        disturbance.x = random_disturbance(CASF_F(1e-7f)); // Small random torque
        disturbance.y = random_disturbance(CASF_F(1e-7f));
        disturbance.z = random_disturbance(CASF_F(1e-7f));
        sim->state.disturbance_torque = disturbance;
    }

    // Call the control callback if registered
    if (sim->control_callback) {
        casf_vector3_t control_torque = {CASF_F(0.0f), CASF_F(0.0f), CASF_F(0.0f)};
        sim->control_callback(&sim->state, &control_torque, sim->user_data);
        sim->state.control_torque = control_torque;
    }

    // Integrate equations of motion using RK4
    rk4_integrate_angular_velocity(sim, sim->config.timestep);

    // Update timestamp and simulation time
    sim->state.timestamp_us += (uint64_t)(CASF_TO_F(sim->config.timestep) * 1e6f);
    sim->simulation_time += sim->config.timestep;
    sim->step_count++;

    return CASF_OK;
}

/**
 * Run the simulation for a specified duration
 */
casf_result_t casf_dynamics_run(casf_dynamics_simulator_t* sim, 
                             casf_float_t duration_s,
                             void (*step_callback)(const casf_dynamics_state_t*, void*),
                             void* user_data) {
    if (!sim || !sim->initialized || duration_s <= CASF_F(0.0f)) {
        return CASF_ERROR_INVALID_PARAM;
    }

    casf_float_t end_time = sim->simulation_time + duration_s;
    casf_result_t result;

    while (sim->simulation_time < end_time && 
           sim->step_count < sim->config.max_iterations) {
        result = casf_dynamics_step(sim);
        if (result != CASF_OK) {
            return result;
        }

        if (step_callback) {
            step_callback(&sim->state, user_data);
        }
    }

    return CASF_OK;
}

/**
 * Set the magnetic field in ECI frame
 */
casf_result_t casf_dynamics_set_magnetic_field(casf_dynamics_simulator_t* sim,
                                           const casf_vector3_t* field_eci) {
    if (!sim || !sim->initialized || !field_eci) {
        return CASF_ERROR_INVALID_PARAM;
    }

    sim->magnetic_field_eci = *field_eci;
    return CASF_OK;
}

/**
 * Get the magnetic field in body frame
 */
casf_result_t casf_dynamics_get_magnetic_field_body(const casf_dynamics_simulator_t* sim,
                                                casf_vector3_t* field_body) {
    if (!sim || !sim->initialized || !field_body) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Convert magnetic field from ECI to body frame using quaternion rotation
    // Simple rotation for this example
    casf_quaternion_t q = sim->state.attitude;
    casf_vector3_t field_eci = sim->magnetic_field_eci;

    // Apply quaternion rotation: v' = q * v * q^-1
    // For efficiency, we use the direct formula rather than explicit quaternion multiplication

    casf_float_t qw = q.w;
    casf_float_t qx = q.x;
    casf_float_t qy = q.y;
    casf_float_t qz = q.z;

    // Precompute some terms used multiple times
    casf_float_t qw2 = qw * qw;
    casf_float_t qx2 = qx * qx;
    casf_float_t qy2 = qy * qy;
    casf_float_t qz2 = qz * qz;

    casf_float_t qwx = qw * qx;
    casf_float_t qwy = qw * qy;
    casf_float_t qwz = qw * qz;
    casf_float_t qxy = qx * qy;
    casf_float_t qxz = qx * qz;
    casf_float_t qyz = qy * qz;

    // Rotation matrix elements
    casf_float_t r11 = qw2 + qx2 - qy2 - qz2;
    casf_float_t r12 = 2 * (qxy - qwz);
    casf_float_t r13 = 2 * (qxz + qwy);

    casf_float_t r21 = 2 * (qxy + qwz);
    casf_float_t r22 = qw2 - qx2 + qy2 - qz2;
    casf_float_t r23 = 2 * (qyz - qwx);

    casf_float_t r31 = 2 * (qxz - qwy);
    casf_float_t r32 = 2 * (qyz + qwx);
    casf_float_t r33 = qw2 - qx2 - qy2 + qz2;

    // Apply rotation
    field_body->x = r11 * field_eci.x + r12 * field_eci.y + r13 * field_eci.z;
    field_body->y = r21 * field_eci.x + r22 * field_eci.y + r23 * field_eci.z;
    field_body->z = r31 * field_eci.x + r32 * field_eci.y + r33 * field_eci.z;

    return CASF_OK;
}

/**
 * Calculate the gravitational gradient torque
 */
void casf_dynamics_gravity_gradient_torque(const casf_dynamics_simulator_t* sim, 
                                        casf_vector3_t* torque) {
    if (!sim || !sim->initialized || !torque) {
        return;
    }

    // Simplified gravity gradient calculation
    // Assumes diagonal inertia tensor and nadir pointing
    casf_float_t r = EARTH_RADIUS + sim->config.orbital_altitude;
    casf_float_t factor = CASF_F(3.0f) * CASF_F(EARTH_MU) / (r * r * r);

    // Different moments of inertia
    casf_float_t Ixx = sim->config.inertia_tensor[0].x;
    casf_float_t Iyy = sim->config.inertia_tensor[1].y;
    casf_float_t Izz = sim->config.inertia_tensor[2].z;

    // Simplified calculation (assumes z-axis points to nadir)
    torque->x = factor * (Izz - Iyy) * sim->state.attitude.y * sim->state.attitude.z;
    torque->y = factor * (Ixx - Izz) * sim->state.attitude.x * sim->state.attitude.z;
    torque->z = factor * (Iyy - Ixx) * sim->state.attitude.x * sim->state.attitude.y;
}

/**
 * Calculate the magnetic torque
 */
void casf_dynamics_magnetic_torque(const casf_dynamics_simulator_t* sim,
                                const casf_vector3_t* magnetic_moment,
                                casf_vector3_t* torque) {
    if (!sim || !sim->initialized || !magnetic_moment || !torque) {
        return;
    }

    casf_vector3_t magnetic_field_body;
    casf_dynamics_get_magnetic_field_body(sim, &magnetic_field_body);

    // Torque = m × B
    torque->x = magnetic_moment->y * magnetic_field_body.z - magnetic_moment->z * magnetic_field_body.y;
    torque->y = magnetic_moment->z * magnetic_field_body.x - magnetic_moment->x * magnetic_field_body.z;
    torque->z = magnetic_moment->x * magnetic_field_body.y - magnetic_moment->y * magnetic_field_body.x;
}

/**
 * Add a disturbance torque to the simulation
 */
casf_result_t casf_dynamics_add_disturbance(casf_dynamics_simulator_t* sim,
                                         const casf_vector3_t* torque) {
    if (!sim || !sim->initialized || !torque) {
        return CASF_ERROR_INVALID_PARAM;
    }

    sim->state.disturbance_torque.x += torque->x;
    sim->state.disturbance_torque.y += torque->y;
    sim->state.disturbance_torque.z += torque->z;

    return CASF_OK;
}

/**
 * Compute the attitude error between current and target attitude
 */
void casf_dynamics_attitude_error(const casf_quaternion_t* current_attitude,
                               const casf_quaternion_t* target_attitude,
                               casf_quaternion_t* error_quaternion) {
    if (!current_attitude || !target_attitude || !error_quaternion) {
        return;
    }

    // Error quaternion = target^-1 * current
    // Conjugate of target is the inverse (assuming normalized)
    casf_quaternion_t target_inverse;
    target_inverse.w = target_attitude->w;
    target_inverse.x = -target_attitude->x;
    target_inverse.y = -target_attitude->y;
    target_inverse.z = -target_attitude->z;

    // Multiply quaternions
    error_quaternion->w = target_inverse.w * current_attitude->w - 
                          target_inverse.x * current_attitude->x - 
                          target_inverse.y * current_attitude->y - 
                          target_inverse.z * current_attitude->z;

    error_quaternion->x = target_inverse.w * current_attitude->x + 
                          target_inverse.x * current_attitude->w + 
                          target_inverse.y * current_attitude->z - 
                          target_inverse.z * current_attitude->y;

    error_quaternion->y = target_inverse.w * current_attitude->y - 
                          target_inverse.x * current_attitude->z + 
                          target_inverse.y * current_attitude->w + 
                          target_inverse.z * current_attitude->x;

    error_quaternion->z = target_inverse.w * current_attitude->z + 
                          target_inverse.x * current_attitude->y - 
                          target_inverse.y * current_attitude->x + 
                          target_inverse.z * current_attitude->w;

    // Normalize the result
    casf_float_t norm = sqrtf(CASF_TO_F(error_quaternion->w * error_quaternion->w + 
                                      error_quaternion->x * error_quaternion->x + 
                                      error_quaternion->y * error_quaternion->y + 
                                      error_quaternion->z * error_quaternion->z));

    if (norm > 1e-6f) {
        error_quaternion->w = CASF_F(CASF_TO_F(error_quaternion->w) / norm);
        error_quaternion->x = CASF_F(CASF_TO_F(error_quaternion->x) / norm);
        error_quaternion->y = CASF_F(CASF_TO_F(error_quaternion->y) / norm);
        error_quaternion->z = CASF_F(CASF_TO_F(error_quaternion->z) / norm);
    }
}

/**
 * Convert the dynamics state to CASF system state
 */
casf_result_t casf_dynamics_to_system_state(const casf_dynamics_simulator_t* sim,
                                         casf_system_state_t* system_state) {
    if (!sim || !sim->initialized || !system_state) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Copy attitude and angular velocity
    system_state->attitude_q = sim->state.attitude;
    system_state->angular_velocity = sim->state.angular_velocity;

    // Convert quaternion to Euler angles
    casf_quaternion_t q = sim->state.attitude;

    // Roll (x-axis rotation)
    system_state->attitude_euler.roll = atan2f(2.0f * (CASF_TO_F(q.w) * CASF_TO_F(q.x) + CASF_TO_F(q.y) * CASF_TO_F(q.z)),
                                              1.0f - 2.0f * (CASF_TO_F(q.x) * CASF_TO_F(q.x) + CASF_TO_F(q.y) * CASF_TO_F(q.y)));

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (CASF_TO_F(q.w) * CASF_TO_F(q.y) - CASF_TO_F(q.z) * CASF_TO_F(q.x));
    if (fabsf(sinp) >= 1.0f) {
        system_state->attitude_euler.pitch = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    } else {
        system_state->attitude_euler.pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    system_state->attitude_euler.yaw = atan2f(2.0f * (CASF_TO_F(q.w) * CASF_TO_F(q.z) + CASF_TO_F(q.x) * CASF_TO_F(q.y)),
                                             1.0f - 2.0f * (CASF_TO_F(q.y) * CASF_TO_F(q.y) + CASF_TO_F(q.z) * CASF_TO_F(q.z)));

    // Copy other state variables
    system_state->timestamp_us = sim->state.timestamp_us;

    // Set validity flags
    system_state->attitude_valid = 1;
    system_state->rates_valid = 1;

    return CASF_OK;
}
