#ifndef CASF_TYPES_H
#define CASF_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Result codes
typedef enum {
    CASF_OK = 0,
    CASF_ERROR_INVALID_PARAM,
    CASF_ERROR_NO_MEMORY,
    CASF_ERROR_NOT_INITIALIZED,
    CASF_ERROR_DEVICE_NOT_FOUND,
    CASF_ERROR_COMMUNICATION,
    CASF_ERROR_TIMEOUT,
    CASF_ERROR_CALIBRATION,
    CASF_ERROR_SAFETY_VIOLATION,
    CASF_ERROR_UNSUPPORTED
} casf_result_t;

// Sensor types
typedef enum {
    CASF_SENSOR_GYROSCOPE,
    CASF_SENSOR_MAGNETOMETER,
    CASF_SENSOR_SUN_SENSOR_COARSE,
    CASF_SENSOR_SUN_SENSOR_FINE,
    CASF_SENSOR_STAR_TRACKER,
    CASF_SENSOR_GPS,
    CASF_SENSOR_CUSTOM,
    CASF_SENSOR_COUNT
} casf_sensor_type_t;

// Basic vector structure
typedef struct {
    float x;
    float y;
    float z;
} casf_vector3_t;

// Quaternion
typedef struct {
    float w;
    float x;
    float y;
    float z;
} casf_quaternion_t;

// Euler angles
typedef struct {
    float roll;
    float pitch;
    float yaw;
} casf_euler_t;

// Sensor data union type
typedef struct {
    casf_sensor_type_t type;  // Type of sensor data contained
    uint64_t timestamp_us;    // Timestamp in microseconds
    casf_vector3_t angular_rate;
    casf_vector3_t magnetic_field;
    casf_vector3_t sun_vector;
    casf_quaternion_t attitude;
    casf_vector3_t position;
    casf_vector3_t velocity;
} casf_sensor_data_t;

// HAL forward declaration
typedef struct casf_hal {
    uint64_t (*get_time_us)(void);
    void (*log)(int level, const char* fmt, ...);
} casf_hal_t;

#endif // CASF_TYPES_H
