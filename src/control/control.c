// ==================== control.h ====================
/**
 * CubeSat ADCS Standard Framework (CASF)
 * Version 1.0
 *
 * Universal ADCS interface standard for CubeSat missions
 * Inspired by drone flight controller standards (PX4/ArduPilot)
 *
 * Features:
 * - Hardware abstraction layer (HAL)
 * - Plugin-based sensor/actuator architecture
 * - Cross-platform compatibility (ARM Cortex-M, Linux, RTOS)
 * - Mission-agnostic control algorithms
 * - Standardized telemetry and command interfaces
 */

// ==================== control.c ====================
#include "control.h"
#include <string.h>
#include <math.h>

// Version information
uint32_t casf_get_version(void) {
    return (CASF_VERSION_MAJOR << 16) | (CASF_VERSION_MINOR << 8) | CASF_VERSION_PATCH;
}

// String conversion utilities
const char* casf_result_to_string(casf_result_t result) {
    switch (result) {
        case CASF_OK: return "OK";
        case CASF_ERROR_INVALID_PARAM: return "Invalid Parameter";
        case CASF_ERROR_NO_MEMORY: return "No Memory";
        case CASF_ERROR_NOT_INITIALIZED: return "Not Initialized";
        case CASF_ERROR_DEVICE_NOT_FOUND: return "Device Not Found";
        case CASF_ERROR_COMMUNICATION: return "Communication Error";
        case CASF_ERROR_TIMEOUT: return "Timeout";
        case CASF_ERROR_CALIBRATION: return "Calibration Error";
        case CASF_ERROR_SAFETY_VIOLATION: return "Safety Violation";
        case CASF_ERROR_UNSUPPORTED: return "Unsupported";
        default: return "Unknown Error";
    }
}

const char* casf_mode_to_string(casf_mode_t mode) {
    switch (mode) {
        case CASF_MODE_SAFE: return "Safe";
        case CASF_MODE_DETUMBLE: return "Detumble";
        case CASF_MODE_COARSE_POINTING: return "Coarse Pointing";
        case CASF_MODE_FINE_POINTING: return "Fine Pointing";
        case CASF_MODE_TARGET_TRACKING: return "Target Tracking";
        case CASF_MODE_SCIENCE: return "Science";
        case CASF_MODE_COMMUNICATION: return "Communication";
        case CASF_MODE_DEPLOYMENT: return "Deployment";
        case CASF_MODE_MANUAL: return "Manual";
        default: return "Unknown";
    }
}

// System initialization
casf_result_t casf_init(casf_system_t* sys, const casf_hal_t* hal, const casf_mission_config_t* config) {
    if (!sys || !hal || !config) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Clear system structure
    memset(sys, 0, sizeof(casf_system_t));

    // Copy HAL and configuration
    sys->hal = *hal;
    sys->mission_config = *config;

    // Initialize system state
    sys->current_mode = config->default_mode;
    sys->requested_mode = config->default_mode;
    sys->active_control_params = &sys->mission_config.control_params[config->default_mode];

    // Initialize attitude to identity quaternion
    sys->state.attitude_q.w = CASF_F(1.0f);
    sys->state.attitude_q.x = CASF_F(0.0f);
    sys->state.attitude_q.y = CASF_F(0.0f);
    sys->state.attitude_q.z = CASF_F(0.0f);

    sys->system_start_time_us = hal->get_time_us();
    sys->initialized = 1;

    if (hal->log) {
        hal->log(0, "CASF v%u.%u.%u initialized",
                CASF_VERSION_MAJOR, CASF_VERSION_MINOR, CASF_VERSION_PATCH);
    }

    return CASF_OK;
}

// Component registration
casf_result_t casf_register_sensor(casf_system_t* sys, const casf_sensor_driver_t* driver) {
    if (!sys || !driver || !sys->initialized) {
        return CASF_ERROR_INVALID_PARAM;
    }

    if (sys->sensor_count >= MAX_SENSORS) {
        return CASF_ERROR_NO_MEMORY;
    }

    // Copy driver structure
    sys->sensors[sys->sensor_count] = *driver;

    // Probe and initialize the sensor
    casf_result_t result = driver->probe(&sys->hal, driver->context);
    if (result != CASF_OK) {
        return result;
    }

    result = driver->init(driver->context, &sys->hal);
    if (result != CASF_OK) {
        return result;
    }

    sys->sensor_count++;

    if (sys->hal.log) {
        sys->hal.log(0, "Registered sensor: %s (%s)", 
                 driver->name ? driver->name : "Unknown", 
                 driver->manufacturer ? driver->manufacturer : "Unknown");
    }

    return CASF_OK;
}

casf_result_t casf_register_actuator(casf_system_t* sys, const casf_actuator_driver_t* driver) {
    if (!sys || !driver || !sys->initialized) {
        return CASF_ERROR_INVALID_PARAM;
    }

    if (sys->actuator_count >= MAX_ACTUATORS) {
        return CASF_ERROR_NO_MEMORY;
    }

    // Copy driver structure
    sys->actuators[sys->actuator_count] = *driver;

    // Probe and initialize the actuator
    casf_result_t result = driver->probe(&sys->hal, driver->context);
    if (result != CASF_OK) {
        return result;
    }

    result = driver->init(driver->context, &sys->hal);
    if (result != CASF_OK) {
        return result;
    }

    sys->actuator_count++;

    if (sys->hal.log) {
        sys->hal.log(0, "Registered actuator: %s (%s)", 
                 driver->name ? driver->name : "Unknown", 
                 driver->manufacturer ? driver->manufacturer : "Unknown");
    }

    return CASF_OK;
}

// Mode control
casf_result_t casf_set_mode(casf_system_t* sys, casf_mode_t mode) {
    if (!sys || !sys->initialized || mode >= CASF_MODE_COUNT) {
        return CASF_ERROR_INVALID_PARAM;
    }

    if (sys->emergency_stop && mode != CASF_MODE_SAFE) {
        return CASF_ERROR_SAFETY_VIOLATION;
    }

    sys->requested_mode = mode;

    if (sys->hal.log) {
        sys->hal.log(0, "Mode change requested: %s -> %s",
                    casf_mode_to_string(sys->current_mode),
                    casf_mode_to_string(mode));
    }

    return CASF_OK;
}

casf_mode_t casf_get_mode(const casf_system_t* sys) {
    return sys ? sys->current_mode : CASF_MODE_SAFE;
}

// Main update function
casf_result_t casf_update(casf_system_t* sys) {
    if (!sys || !sys->initialized) {
        return CASF_ERROR_NOT_INITIALIZED;
    }

    uint64_t current_time = sys->hal.get_time_us();

    // Update sensors
    for (uint8_t i = 0; i < sys->sensor_count; i++) {
        casf_sensor_driver_t* sensor = &sys->sensors[i];
        if (!sensor->enabled) continue;

        uint32_t update_period_us = 1000000 / sensor->current_rate_hz;
        if (current_time - sensor->last_update_us >= update_period_us) {
            casf_sensor_data_t data;
            casf_result_t result = sensor->read(sensor->context, &data);

            if (result == CASF_OK) {
                // Update system state based on sensor type
                switch (sensor->type) {
                    case CASF_SENSOR_GYROSCOPE:
                        sys->state.angular_velocity = data.angular_rate;
                        sys->state.rates_valid = 1;
                        break;
                    case CASF_SENSOR_MAGNETOMETER:
                        sys->state.magnetic_field_body = data.magnetic_field;
                        sys->state.mag_valid = 1;
                        break;
                    case CASF_SENSOR_SUN_SENSOR_COARSE:
                    case CASF_SENSOR_SUN_SENSOR_FINE:
                        sys->state.sun_vector_body = data.sun_vector;
                        sys->state.sun_valid = 1;
                        break;
                    case CASF_SENSOR_STAR_TRACKER:
                        sys->state.attitude_q = data.attitude;
                        sys->state.attitude_valid = 1;
                        break;
                    case CASF_SENSOR_GPS:
                        sys->state.position_ecef = data.position;
                        sys->state.velocity_ecef = data.velocity;
                        sys->state.position_valid = 1;
                        break;
                    default:
                        break;
                }
                sensor->health_status = 100;
            } else {
                sensor->health_status = sensor->health_status > 10 ? sensor->health_status - 10 : 0;
            }

            sensor->last_update_us = current_time;
        }
    }

    // Run control cycle if it's time
    uint32_t control_period_us = 1000000 / sys->mission_config.control_frequency_hz;
    if (current_time - sys->last_control_update_us >= control_period_us) {
        casf_run_control_cycle(sys);
        sys->last_control_update_us = current_time;
        sys->control_cycle_count++;
    }

    // Handle mode transitions
    if (sys->requested_mode != sys->current_mode) {
        sys->current_mode = sys->requested_mode;
        sys->active_control_params = &sys->mission_config.control_params[sys->current_mode];
    }

    // Update system health metrics
    sys->system_uptime_s = (uint32_t)((current_time - sys->system_start_time_us) / 1000000);

    return CASF_OK;
}

// Control cycle execution
casf_result_t casf_run_control_cycle(casf_system_t* sys) {
    if (!sys || !sys->initialized) {
        return CASF_ERROR_NOT_INITIALIZED;
    }

    // Skip control if in safe mode or emergency stop
    if (sys->current_mode == CASF_MODE_SAFE || sys->emergency_stop) {
        // Send zero commands to all actuators
        casf_actuator_cmd_t zero_cmd = {0};
        for (uint8_t i = 0; i < sys->actuator_count; i++) {
            if (sys->actuators[i].enabled) {
                sys->actuators[i].command(sys->actuators[i].context, &zero_cmd);
            }
        }
        return CASF_OK;
    }

    // Attitude determination (simplified - normally would use Kalman filter)
    if (sys->state.attitude_valid) {
        // Use star tracker or other high-accuracy attitude sensor
        casf_quaternion_to_euler(&sys->state.attitude_q, &sys->state.attitude_euler);
    }

    // Control algorithm based on current mode
    casf_vector3_t torque_cmd = {0};

    switch (sys->current_mode) {
        case CASF_MODE_DETUMBLE:
            // B-dot control for detumbling
            if (sys->state.rates_valid && sys->state.mag_valid) {
                casf_vector3_cross(&sys->state.angular_velocity, &sys->state.magnetic_field_body, &torque_cmd);
                // Scale by detumble gain
                torque_cmd.x *= -CASF_F(0.1f);
                torque_cmd.y *= -CASF_F(0.1f);
                torque_cmd.z *= -CASF_F(0.1f);
            }
            break;

        case CASF_MODE_COARSE_POINTING:
        case CASF_MODE_FINE_POINTING:
            // PID attitude control
            if (sys->state.attitude_valid && sys->state.rates_valid) {
                // Calculate attitude error quaternion
                casf_quaternion_t q_error;
                casf_quaternion_t q_target_inv = sys->active_control_params->target_attitude;
                q_target_inv.x = -q_target_inv.x;
                q_target_inv.y = -q_target_inv.y;
                q_target_inv.z = -q_target_inv.z;

                casf_quaternion_multiply(&sys->state.attitude_q, &q_target_inv, &q_error);

                // Convert to attitude error vector (small angle approximation)
                sys->control_error.x = CASF_F(2.0f) * q_error.x;
                sys->control_error.y = CASF_F(2.0f) * q_error.y;
                sys->control_error.z = CASF_F(2.0f) * q_error.z;

                // Rate error
                casf_vector3_t rate_error;
                rate_error.x = sys->state.angular_velocity.x - sys->active_control_params->target_rates.x;
                rate_error.y = sys->state.angular_velocity.y - sys->active_control_params->target_rates.y;
                rate_error.z = sys->state.angular_velocity.z - sys->active_control_params->target_rates.z;

                // Integral term (with windup protection)
                casf_float_t max_integral = CASF_F(0.1f);
                sys->integral_error.x += sys->control_error.x;
                sys->integral_error.y += sys->control_error.y;
                sys->integral_error.z += sys->control_error.z;

                // Clamp integral terms
                if (sys->integral_error.x > max_integral) sys->integral_error.x = max_integral;
                if (sys->integral_error.x < -max_integral) sys->integral_error.x = -max_integral;
                if (sys->integral_error.y > max_integral) sys->integral_error.y = max_integral;
                if (sys->integral_error.y < -max_integral) sys->integral_error.y = -max_integral;
                if (sys->integral_error.z > max_integral) sys->integral_error.z = max_integral;
                if (sys->integral_error.z < -max_integral) sys->integral_error.z = -max_integral;

                // PID control law
                torque_cmd.x = sys->active_control_params->control_gains_p.x * sys->control_error.x +
                              sys->active_control_params->control_gains_i.x * sys->integral_error.x +
                              sys->active_control_params->control_gains_d.x * rate_error.x;

                torque_cmd.y = sys->active_control_params->control_gains_p.y * sys->control_error.y +
                              sys->active_control_params->control_gains_i.y * sys->integral_error.y +
                              sys->active_control_params->control_gains_d.y * rate_error.y;

                torque_cmd.z = sys->active_control_params->control_gains_p.z * sys->control_error.z +
                              sys->active_control_params->control_gains_i.z * sys->integral_error.z +
                              sys->active_control_params->control_gains_d.z * rate_error.z;
            }
            break;

        case CASF_MODE_TARGET_TRACKING:
            // Target tracking control (implement specific tracking algorithm)
            break;

        default:
            // Default to zero torque
            break;
    }

    // Apply torque limits
    casf_float_t torque_magnitude = casf_vector3_magnitude(&torque_cmd);
    if (torque_magnitude > sys->active_control_params->max_torque) {
        casf_float_t scale = sys->active_control_params->max_torque / torque_magnitude;
        torque_cmd.x *= scale;
        torque_cmd.y *= scale;
        torque_cmd.z *= scale;
    }

    sys->control_torque = torque_cmd;

    // Distribute commands to actuators
    casf_actuator_cmd_t cmd = {0};
    cmd.torque_cmd = torque_cmd;
    cmd.timestamp_us = sys->hal.get_time_us();

    for (uint8_t i = 0; i < sys->actuator_count; i++) {
        casf_actuator_driver_t* actuator = &sys->actuators[i];
        if (!actuator->enabled || !actuator->armed) continue;

        // Send command based on actuator type and capability
        switch (actuator->type) {
            case CASF_ACTUATOR_REACTION_WHEEL:
            case CASF_ACTUATOR_MAGNETORQUER:
                actuator->command(actuator->context, &cmd);
                break;
            default:
                break;
        }
    }

    return CASF_OK;
}

// State access functions
const casf_system_state_t* casf_get_state(const casf_system_t* sys) {
    return sys ? &sys->state : NULL;
}

casf_result_t casf_get_sensor_data(const casf_system_t* sys, casf_sensor_type_t type, casf_sensor_data_t* data) {
    if (!sys || !data) {
        return CASF_ERROR_INVALID_PARAM;
    }

    for (uint8_t i = 0; i < sys->sensor_count; i++) {
        if (sys->sensors[i].type == type && sys->sensors[i].enabled) {
            return sys->sensors[i].read(sys->sensors[i].context, data);
        }
    }

    return CASF_ERROR_DEVICE_NOT_FOUND;
}

// Control interface functions
casf_result_t casf_set_target_attitude(casf_system_t* sys, const casf_quaternion_t* attitude) {
    if (!sys || !attitude) {
        return CASF_ERROR_INVALID_PARAM;
    }

    sys->active_control_params->target_attitude = *attitude;

    if (sys->hal.log) {
        sys->hal.log(1, "Target attitude set: [%.3f, %.3f, %.3f, %.3f]",
                    CASF_TO_F(attitude->w), CASF_TO_F(attitude->x),
                    CASF_TO_F(attitude->y), CASF_TO_F(attitude->z));
    }

    return CASF_OK;
}

casf_result_t casf_set_target_rates(casf_system_t* sys, const casf_vector3_t* rates) {
    if (!sys || !rates) {
        return CASF_ERROR_INVALID_PARAM;
    }

    sys->active_control_params->target_rates = *rates;

    return CASF_OK;
}

// Safety functions
casf_result_t casf_emergency_stop(casf_system_t* sys) {
    if (!sys) {
        return CASF_ERROR_INVALID_PARAM;
    }

    sys->emergency_stop = 1;
    sys->current_mode = CASF_MODE_SAFE;

    // Emergency stop all actuators
    for (uint8_t i = 0; i < sys->actuator_count; i++) {
        if (sys->actuators[i].emergency_stop) {
            sys->actuators[i].emergency_stop(sys->actuators[i].context);
        }
        sys->actuators[i].armed = 0;
    }

    if (sys->hal.log) {
        sys->hal.log(0, "EMERGENCY STOP ACTIVATED");
    }

    return CASF_OK;
}

casf_result_t casf_safe_mode(casf_system_t* sys) {
    if (!sys) {
        return CASF_ERROR_INVALID_PARAM;
    }

    sys->safe_mode_active = 1;
    sys->current_mode = CASF_MODE_SAFE;

    // Reset integral terms
    sys->integral_error.x = CASF_F(0.0f);
    sys->integral_error.y = CASF_F(0.0f);
    sys->integral_error.z = CASF_F(0.0f);

    return CASF_OK;
}

// Math utilities
void casf_quaternion_multiply(const casf_quaternion_t* q1, const casf_quaternion_t* q2, casf_quaternion_t* result) {
    if (!q1 || !q2 || !result) return;

    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void casf_quaternion_to_euler(const casf_quaternion_t* q, casf_euler_t* euler) {
    if (!q || !euler) return;

#if USE_FIXED_POINT
    // Fixed-point quaternion to Euler conversion
    int32_t w = q->w, x = q->x, y = q->y, z = q->z;

    // Roll (x-axis rotation)
    int32_t sinr_cosp = 2 * (w * x + y * z);
    int32_t cosr_cosp = (1 << 16) - 2 * (x * x + y * y);
    euler->roll = CASF_F(atan2f((float)sinr_cosp / (1 << 16), (float)cosr_cosp / (1 << 16)));

    // Pitch (y-axis rotation)
    int32_t sinp = 2 * (w * y - z * x);
    if (sinp >= (1 << 16)) {
        euler->pitch = CASF_F(M_PI / 2);
    } else if (sinp <= -(1 << 16)) {
        euler->pitch = CASF_F(-M_PI / 2);
    } else {
        euler->pitch = CASF_F(asinf((float)sinp / (1 << 16)));
    }

    // Yaw (z-axis rotation)
    int32_t siny_cosp = 2 * (w * z + x * y);
    int32_t cosy_cosp = (1 << 16) - 2 * (y * y + z * z);
    euler->yaw = CASF_F(atan2f((float)siny_cosp / (1 << 16), (float)cosy_cosp / (1 << 16)));
#else
    // Floating-point quaternion to Euler conversion
    float w = q->w, x = q->x, y = q->y, z = q->z;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        euler->pitch = asinf(sinp);
    }

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp);
#endif
}

void casf_euler_to_quaternion(const casf_euler_t* euler, casf_quaternion_t* q) {
    if (!euler || !q) return;

#if USE_FIXED_POINT
    float roll = CASF_TO_F(euler->roll) * 0.5f;
    float pitch = CASF_TO_F(euler->pitch) * 0.5f;
    float yaw = CASF_TO_F(euler->yaw) * 0.5f;
#else
    float roll = euler->roll * 0.5f;
    float pitch = euler->pitch * 0.5f;
    float yaw = euler->yaw * 0.5f;
#endif

    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    q->w = CASF_F(cr * cp * cy + sr * sp * sy);
    q->x = CASF_F(sr * cp * cy - cr * sp * sy);
    q->y = CASF_F(cr * sp * cy + sr * cp * sy);
    q->z = CASF_F(cr * cp * sy - sr * sp * cy);
}

casf_float_t casf_vector3_magnitude(const casf_vector3_t* v) {
    if (!v) return CASF_F(0.0f);

#if USE_FIXED_POINT
    int64_t x = v->x, y = v->y, z = v->z;
    int64_t mag_sq = (x * x + y * y + z * z) >> 16;  // Scale down to prevent overflow
    return CASF_F(sqrtf((float)mag_sq / CASF_FLOAT_SCALE));
#else
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
#endif
}

void casf_vector3_normalize(casf_vector3_t* v) {
    if (!v) return;

    casf_float_t mag = casf_vector3_magnitude(v);
    if (mag > CASF_F(1e-6f)) {
        v->x /= mag;
        v->y /= mag;
        v->z /= mag;
    }
}

void casf_vector3_cross(const casf_vector3_t* a, const casf_vector3_t* b, casf_vector3_t* result) {
    if (!a || !b || !result) return;

    result->x = a->y * b->z - a->z * b->y;
    result->y = a->z * b->x - a->x * b->z;
    result->z = a->x * b->y - a->y * b->x;
}

// System lifecycle functions
casf_result_t casf_start(casf_system_t* sys) {
    if (!sys || !sys->initialized) {
        return CASF_ERROR_NOT_INITIALIZED;
    }

    sys->control_active = 1;
    sys->autonomous_mode = 1;

    if (sys->hal.log) {
        sys->hal.log(0, "CASF system started - entering %s mode",
                    casf_mode_to_string(sys->current_mode));
    }

    return CASF_OK;
}

casf_result_t casf_stop(casf_system_t* sys) {
    if (!sys) {
        return CASF_ERROR_INVALID_PARAM;
    }

    sys->control_active = 0;
    sys->autonomous_mode = 0;

    // Send zero commands to all actuators
    casf_actuator_cmd_t zero_cmd = {0};
    for (uint8_t i = 0; i < sys->actuator_count; i++) {
        if (sys->actuators[i].enabled) {
            sys->actuators[i].command(sys->actuators[i].context, &zero_cmd);
        }
    }

    return CASF_OK;
}

casf_result_t casf_shutdown(casf_system_t* sys) {
    if (!sys) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Stop system
    casf_stop(sys);

    // Shutdown all sensors
    for (uint8_t i = 0; i < sys->sensor_count; i++) {
        if (sys->sensors[i].shutdown) {
            sys->sensors[i].shutdown(sys->sensors[i].context);
        }
    }

    // Shutdown all actuators
    for (uint8_t i = 0; i < sys->actuator_count; i++) {
        if (sys->actuators[i].shutdown) {
            sys->actuators[i].shutdown(sys->actuators[i].context);
        }
    }

    sys->initialized = 0;

    if (sys->hal.log) {
        sys->hal.log(0, "CASF system shutdown complete");
    }

    return CASF_OK;
}

