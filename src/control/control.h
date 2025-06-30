/**
 * CubeSat ADCS Standard Framework (CASF)
 * Version 1.0
 *
 * Universal ADCS interface standard for CubeSat missions
 * Inspired by drone flight controller standards (PX4/ArduPilot)
 */

#ifndef CASF_CONTROL_H
#define CASF_CONTROL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Version information
#define CASF_VERSION_MAJOR 1
#define CASF_VERSION_MINOR 0
#define CASF_VERSION_PATCH 0

// Configuration
#define MAX_SENSORS 10
#define MAX_ACTUATORS 6
#define USE_FIXED_POINT 0

// For packed structures (platform independent)
#if defined(_MSC_VER)
    #define CASF_PACKED
#else
    #define CASF_PACKED __attribute__((packed))
#endif

// Fixed-point or floating-point math
#if USE_FIXED_POINT
    typedef int32_t casf_float_t;
    #define CASF_FLOAT_SCALE 65536
    #define CASF_F(x) ((casf_float_t)((x) * CASF_FLOAT_SCALE))
    #define CASF_TO_F(x) ((float)(x) / CASF_FLOAT_SCALE)
#else
    typedef float casf_float_t;
    #define CASF_F(x) (x)
    #define CASF_TO_F(x) (x)
#endif

// Forward include for basic types
#include "../sensors/sensor_types.h"

// Euler angles wrapper using casf_float_t instead of float
typedef struct casf_euler casf_euler_wrapper_t;

// Operation modes
typedef enum {
    CASF_MODE_SAFE = 0,         // Safe mode - minimal power, basic attitude
    CASF_MODE_DETUMBLE,         // Active detumbling
    CASF_MODE_COARSE_POINTING,  // Coarse sun/Earth pointing
    CASF_MODE_FINE_POINTING,    // Fine attitude control
    CASF_MODE_TARGET_TRACKING,  // Track specific targets
    CASF_MODE_SCIENCE,          // Science mission mode
    CASF_MODE_COMMUNICATION,    // Ground station communication
    CASF_MODE_DEPLOYMENT,       // Post-deployment stabilization
    CASF_MODE_MANUAL,           // Manual control override
    CASF_MODE_COUNT
} casf_mode_t;

// Extended sensor types (extends the basic types in sensor_types.h)
typedef enum {
    CASF_SENSOR_ACCELEROMETER = CASF_SENSOR_COUNT,
    CASF_SENSOR_EARTH_HORIZON,
    CASF_SENSOR_IMU,
    CASF_SENSOR_TYPE_COUNT
} casf_extended_sensor_type_t;

// Actuator types
typedef enum {
    CASF_ACTUATOR_REACTION_WHEEL = 0,
    CASF_ACTUATOR_MAGNETORQUER,
    CASF_ACTUATOR_THRUSTER_COLD_GAS,
    CASF_ACTUATOR_THRUSTER_CHEMICAL,
    CASF_ACTUATOR_THRUSTER_ELECTRIC,
    CASF_ACTUATOR_CONTROL_MOMENT_GYRO,
    CASF_ACTUATOR_GRAVITY_GRADIENT,
    CASF_ACTUATOR_CUSTOM,
    CASF_ACTUATOR_TYPE_COUNT
} casf_actuator_type_t;

// Forward declarations
typedef struct casf_system_s casf_system_t;

// Actuator command structure
typedef struct {
    casf_actuator_type_t type;
    uint64_t timestamp_us;
    union {
        casf_vector3_t torque_cmd;      // N⋅m for reaction wheels and magnetorquers
        casf_vector3_t force_cmd;       // N for thrusters
        casf_vector3_t dipole_cmd;      // Am² for magnetorquers
        casf_vector3_t wheel_speed_cmd; // rad/s for reaction wheels
        casf_vector3_t angular_momentum; // N⋅m⋅s
    };
    uint16_t power_level;              // 0-65535 (0-100%)
    uint16_t command_flags;            // Command-specific flags
} casf_actuator_cmd_t;

// Enhanced Hardware Abstraction Layer (HAL) - extends the basic one from sensor_types.h
typedef struct casf_extended_hal {
    // Basic HAL components
    casf_hal_t base_hal;

    // Time management
    void (*delay_us)(uint32_t microseconds);

    // Memory management
    void* (*malloc)(size_t size);
    void (*free)(void* ptr);

    // Hardware interfaces (optional, can be NULL)
    int (*i2c_transfer)(uint8_t addr, uint8_t* tx_data, size_t tx_len, uint8_t* rx_data, size_t rx_len);
    int (*spi_transfer)(uint8_t cs_pin, uint8_t* tx_data, uint8_t* rx_data, size_t len);
    int (*uart_write)(uint8_t port, const uint8_t* data, size_t len);
    int (*uart_read)(uint8_t port, uint8_t* data, size_t len);
    void (*gpio_set)(uint8_t pin, bool state);
    bool (*gpio_get)(uint8_t pin);

    // Platform info
    const char* platform_name;
} casf_extended_hal_t;

// Sensor driver interface
typedef struct {
    // Metadata
    const char* name;
    const char* manufacturer;
    const char* version;
    casf_sensor_type_t type;
    uint8_t id;
    uint32_t capabilities;          // Capability flags

    // Driver functions
    casf_result_t (*probe)(const casf_hal_t* hal, void* context);
    casf_result_t (*init)(void* context, const casf_hal_t* hal);
    casf_result_t (*read)(void* context, casf_sensor_data_t* data);
    casf_result_t (*calibrate)(void* context);
    casf_result_t (*configure)(void* context, const void* config, size_t config_size);
    casf_result_t (*self_test)(void* context);
    casf_result_t (*shutdown)(void* context);

    // Context
    void* context;

    // Performance characteristics
    uint32_t update_rate_max_hz;
    uint32_t update_rate_min_hz;
    uint16_t power_consumption_mw;
    uint16_t accuracy_rating;       // 0-65535

    // Runtime state
    uint32_t current_rate_hz;
    uint64_t last_update_us;
    uint8_t priority;               // 0-255
    uint8_t health_status;          // 0-100
    uint8_t enabled : 1;
    uint8_t calibrated : 1;
    uint8_t self_test_passed : 1;
    uint8_t reserved : 5;
} casf_sensor_driver_t;

// Actuator driver interface
typedef struct {
    // Metadata
    const char* name;
    const char* manufacturer;
    const char* version;
    casf_actuator_type_t type;
    uint8_t id;
    uint32_t capabilities;

    // Driver functions
    casf_result_t (*probe)(const casf_hal_t* hal, void* context);
    casf_result_t (*init)(void* context, const casf_hal_t* hal);
    casf_result_t (*command)(void* context, const casf_actuator_cmd_t* cmd);
    casf_result_t (*configure)(void* context, const void* config, size_t config_size);
    casf_result_t (*self_test)(void* context);
    casf_result_t (*emergency_stop)(void* context);
    casf_result_t (*shutdown)(void* context);

    // Context
    void* context;

    // Performance characteristics
    casf_vector3_t max_torque;
    casf_vector3_t max_force;
    uint32_t response_time_us;
    uint16_t power_consumption_mw;

    // Runtime state
    uint64_t last_command_us;
    casf_actuator_cmd_t last_cmd;
    uint8_t priority;
    uint8_t health_status;
    uint8_t enabled : 1;
    uint8_t armed : 1;
    uint8_t self_test_passed : 1;
    uint8_t emergency_stopped : 1;
    uint8_t reserved : 4;
} casf_actuator_driver_t;

// Control parameters for each mode
typedef struct {
    casf_quaternion_t target_attitude;
    casf_vector3_t target_rates;
    casf_vector3_t control_gains_p;
    casf_vector3_t control_gains_i;
    casf_vector3_t control_gains_d;
    casf_float_t deadband_attitude;  // radians
    casf_float_t deadband_rate;      // rad/s
    casf_float_t max_torque;
    casf_float_t max_rate;           // rad/s
    uint16_t control_period_ms;
    uint8_t auto_mode : 1;
    uint8_t rate_limit_enable : 1;
    uint8_t integrator_enable : 1;
    uint8_t reserved : 5;
} casf_control_params_t;

// Mission configuration
typedef struct {
    const char* mission_name;
    const char* spacecraft_id;
    casf_mode_t default_mode;
    uint32_t control_frequency_hz;
    uint32_t telemetry_frequency_hz;
    uint32_t housekeeping_frequency_hz;
    casf_control_params_t control_params[CASF_MODE_COUNT];
    casf_vector3_t inertia_tensor[3];

    // Safety parameters
    casf_float_t max_angular_rate;
    casf_float_t max_angular_acceleration;
    casf_float_t safe_mode_timeout_s;
    uint8_t enable_watchdog : 1;
    uint8_t enable_safe_mode : 1;
    uint8_t enable_autonomous_recovery : 1;
    uint8_t reserved : 5;
} casf_mission_config_t;

// System state
typedef struct {
    // Attitude state
    casf_quaternion_t attitude_q;    // Current attitude quaternion
    casf_euler_t attitude_euler;     // Euler angles (rad)
    casf_vector3_t angular_velocity; // Angular velocity (rad/s)
    casf_vector3_t angular_acceleration; // rad/s^2

    // Sensor measurements
    casf_vector3_t magnetic_field_body;  // Magnetic field in body frame
    casf_vector3_t sun_vector_body;      // Sun vector in body frame
    casf_vector3_t position_ecef;        // Position in ECEF frame
    casf_vector3_t velocity_ecef;        // Velocity in ECEF frame
    casf_vector3_t acceleration_ecef;    // ECEF acceleration
    casf_vector3_t nadir_vector_body;    // Body frame

    // System status
    uint64_t timestamp_us;
    uint32_t solution_quality;      // 0-100 confidence
    uint16_t active_sensors;        // Bitmask of active sensors
    uint16_t active_actuators;      // Bitmask of active actuators

    // Validity flags
    uint8_t attitude_valid : 1;
    uint8_t rates_valid : 1;
    uint8_t mag_valid : 1;
    uint8_t sun_valid : 1;
    uint8_t position_valid : 1;
    uint8_t reserved : 3;
} casf_system_state_t;

// Main system structure
typedef struct casf_system_s {
    // Hardware abstraction
    casf_hal_t hal;

    // Component arrays
    casf_sensor_driver_t sensors[MAX_SENSORS];
    casf_actuator_driver_t actuators[MAX_ACTUATORS];
    uint8_t sensor_count;
    uint8_t actuator_count;

    // System state
    casf_system_state_t state;
    casf_system_state_t previous_state;
    casf_mission_config_t mission_config;

    // Control system
    casf_mode_t current_mode;
    casf_mode_t requested_mode;
    casf_control_params_t* active_control_params;
    casf_vector3_t control_torque;
    casf_vector3_t control_error;
    casf_vector3_t integral_error;

    // Timing and synchronization
    uint64_t system_start_time_us;
    uint64_t last_control_update_us;
    uint64_t last_sensor_update_us;
    uint32_t control_cycle_count;

    // System health
    uint32_t system_uptime_s;
    uint16_t system_load_percent;
    uint8_t system_temperature_c;
    uint8_t power_level_percent;

    // Status flags
    uint8_t initialized : 1;
    uint8_t control_active : 1;
    uint8_t emergency_stop : 1;
    uint8_t autonomous_mode : 1;
    uint8_t safe_mode_active : 1;
    uint8_t watchdog_enabled : 1;
    uint8_t reserved : 2;
} casf_system_t;

// API Functions

#ifdef __cplusplus
extern "C" {
#endif

// System lifecycle
casf_result_t casf_init(casf_system_t* sys, const casf_hal_t* hal, const casf_mission_config_t* config);
casf_result_t casf_start(casf_system_t* sys);
casf_result_t casf_stop(casf_system_t* sys);
casf_result_t casf_shutdown(casf_system_t* sys);
casf_result_t casf_update(casf_system_t* sys);

// Component registration (for driver modules)
casf_result_t casf_register_sensor(casf_system_t* sys, const casf_sensor_driver_t* driver);
casf_result_t casf_register_actuator(casf_system_t* sys, const casf_actuator_driver_t* driver);
casf_result_t casf_unregister_sensor(casf_system_t* sys, const char* name);
casf_result_t casf_unregister_actuator(casf_system_t* sys, const char* name);

// Mode control
casf_result_t casf_set_mode(casf_system_t* sys, casf_mode_t mode);
casf_mode_t casf_get_mode(const casf_system_t* sys);
casf_result_t casf_emergency_stop(casf_system_t* sys);
casf_result_t casf_safe_mode(casf_system_t* sys);

// Control interface
casf_result_t casf_set_target_attitude(casf_system_t* sys, const casf_quaternion_t* attitude);
casf_result_t casf_set_target_rates(casf_system_t* sys, const casf_vector3_t* rates);
casf_result_t casf_update_control_params(casf_system_t* sys, const casf_control_params_t* params);

// State access
const casf_system_state_t* casf_get_state(const casf_system_t* sys);
casf_result_t casf_get_sensor_data(const casf_system_t* sys, casf_sensor_type_t type, casf_sensor_data_t* data);

// System execution (call from main loop)
casf_result_t casf_run_control_cycle(casf_system_t* sys);

// Utility functions
const char* casf_result_to_string(casf_result_t result);
const char* casf_mode_to_string(casf_mode_t mode);
uint32_t casf_get_version(void);

// Math utilities (optimized for embedded)
void casf_quaternion_multiply(const casf_quaternion_t* q1, const casf_quaternion_t* q2, casf_quaternion_t* result);
void casf_quaternion_to_euler(const casf_quaternion_t* q, casf_euler_t* euler);
void casf_euler_to_quaternion(const casf_euler_t* euler, casf_quaternion_t* q);
casf_float_t casf_vector3_magnitude(const casf_vector3_t* v);
void casf_vector3_normalize(casf_vector3_t* v);
void casf_vector3_cross(const casf_vector3_t* a, const casf_vector3_t* b, casf_vector3_t* result);

#ifdef __cplusplus
}
#endif







// ==================== API Functions ====================

#ifdef __cplusplus
extern "C" {
#endif

// System lifecycle
casf_result_t casf_init(casf_system_t* sys, const casf_hal_t* hal, const casf_mission_config_t* config);
casf_result_t casf_start(casf_system_t* sys);
casf_result_t casf_stop(casf_system_t* sys);
casf_result_t casf_shutdown(casf_system_t* sys);

// Component registration (for driver modules)
casf_result_t casf_register_sensor(casf_system_t* sys, const casf_sensor_driver_t* driver);
casf_result_t casf_register_actuator(casf_system_t* sys, const casf_actuator_driver_t* driver);
casf_result_t casf_unregister_sensor(casf_system_t* sys, const char* name);
casf_result_t casf_unregister_actuator(casf_system_t* sys, const char* name);

// Mode control
casf_result_t casf_set_mode(casf_system_t* sys, casf_mode_t mode);
casf_mode_t casf_get_mode(const casf_system_t* sys);
casf_result_t casf_emergency_stop(casf_system_t* sys);
casf_result_t casf_safe_mode(casf_system_t* sys);

// Control interface
casf_result_t casf_set_target_attitude(casf_system_t* sys, const casf_quaternion_t* attitude);
casf_result_t casf_set_target_rates(casf_system_t* sys, const casf_vector3_t* rates);
casf_result_t casf_update_control_params(casf_system_t* sys, const casf_control_params_t* params);

// State access
const casf_system_state_t* casf_get_state(const casf_system_t* sys);
casf_result_t casf_get_sensor_data(const casf_system_t* sys, casf_sensor_type_t type, casf_sensor_data_t* data);

// System execution (call from main loop)
casf_result_t casf_update(casf_system_t* sys);
casf_result_t casf_run_control_cycle(casf_system_t* sys);

// Utility functions
const char* casf_result_to_string(casf_result_t result);
const char* casf_mode_to_string(casf_mode_t mode);
uint32_t casf_get_version(void);

// Math utilities (optimized for embedded)
void casf_quaternion_multiply(const casf_quaternion_t* q1, const casf_quaternion_t* q2, casf_quaternion_t* result);
void casf_quaternion_to_euler(const casf_quaternion_t* q, casf_euler_t* euler);
void casf_euler_to_quaternion(const casf_euler_t* euler, casf_quaternion_t* q);
casf_float_t casf_vector3_magnitude(const casf_vector3_t* v);
void casf_vector3_normalize(casf_vector3_t* v);
void casf_vector3_cross(const casf_vector3_t* a, const casf_vector3_t* b, casf_vector3_t* result);

#ifdef __cplusplus
}
#endif

#endif // CASF_CONTROL_H
