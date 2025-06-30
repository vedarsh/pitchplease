// ==================== sensors.c ====================
#include "sensors.h"
#include <string.h>

// Define CASF_F macro for float literals if not already defined
#ifndef CASF_F
#define CASF_F(x) (x)
#endif

casf_result_t casf_create_gyroscope_driver(casf_sensor_driver_t* driver,
                                        const char* name, 
                                        const char* manufacturer,
                                        const char* version,
                                        void* context) {
    casf_result_t result = casf_create_sensor_driver(driver, CASF_SENSOR_GYROSCOPE, name, 
                                                   manufacturer, version);
    if (result != CASF_OK) {
        return result;
    }

    driver->context = context;
    driver->current_rate_hz = 50;  // Default rate for gyroscopes
    driver->power_consumption_mw = 25;  // Typical power consumption

    return CASF_OK;
}

casf_result_t casf_create_magnetometer_driver(casf_sensor_driver_t* driver,
                                          const char* name, 
                                          const char* manufacturer,
                                          const char* version,
                                          void* context) {
    casf_result_t result = casf_create_sensor_driver(driver, CASF_SENSOR_MAGNETOMETER, name, 
                                                   manufacturer, version);
    if (result != CASF_OK) {
        return result;
    }

    driver->context = context;
    driver->current_rate_hz = 10;  // Default rate for magnetometers
    driver->power_consumption_mw = 15;  // Typical power consumption

    return CASF_OK;
}

casf_result_t casf_create_sun_sensor_driver(casf_sensor_driver_t* driver,
                                         casf_sensor_type_t type,
                                         const char* name, 
                                         const char* manufacturer,
                                         const char* version,
                                         void* context) {
    if (type != CASF_SENSOR_SUN_SENSOR_COARSE && type != CASF_SENSOR_SUN_SENSOR_FINE) {
        return CASF_ERROR_INVALID_PARAM;
    }

    casf_result_t result = casf_create_sensor_driver(driver, type, name, 
                                                   manufacturer, version);
    if (result != CASF_OK) {
        return result;
    }

    driver->context = context;
    driver->current_rate_hz = 5;  // Default rate for sun sensors
    driver->power_consumption_mw = type == CASF_SENSOR_SUN_SENSOR_FINE ? 50 : 20;

    return CASF_OK;
}

casf_result_t casf_create_star_tracker_driver(casf_sensor_driver_t* driver,
                                           const char* name, 
                                           const char* manufacturer,
                                           const char* version,
                                           void* context) {
    casf_result_t result = casf_create_sensor_driver(driver, CASF_SENSOR_STAR_TRACKER, name, 
                                                   manufacturer, version);
    if (result != CASF_OK) {
        return result;
    }

    driver->context = context;
    driver->current_rate_hz = 1;  // Default rate for star trackers
    driver->power_consumption_mw = 500;  // Typical power consumption

    return CASF_OK;
}

// Function declarations for sensor drivers moved to sensors.h

casf_result_t casf_gyro_configure(void* context, const void* config, size_t config_size) {
    if (!context || !config || config_size == 0) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific configuration for gyroscope

    return CASF_OK;
}

casf_result_t casf_gyro_shutdown(void* context) {
    if (!context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific shutdown for gyroscope

    return CASF_OK;
}

// Sun sensor driver implementation
casf_result_t casf_sun_probe(const casf_hal_t* hal, void* context) {
    if (!hal || !context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific probe logic for sun sensor

    return CASF_OK;
}

casf_result_t casf_sun_init(void* context, const casf_hal_t* hal) {
    if (!hal || !context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific initialization for sun sensor

    return CASF_OK;
}

casf_result_t casf_sun_read(void* context, casf_sensor_data_t* data) {
    if (!context || !data) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific reading logic for sun sensor

    data->type = CASF_SENSOR_SUN_SENSOR_COARSE; // or FINE depending on the sensor
    data->timestamp_us = 0; // Should be set by the hardware driver

    // Placeholder data - normalized sun vector
    data->sun_vector.x = CASF_F(0.0f);
    data->sun_vector.y = CASF_F(0.0f);
    data->sun_vector.z = CASF_F(1.0f); // Default pointing toward Z-axis

    return CASF_OK;
}

casf_result_t casf_sun_calibrate(void* context) {
    if (!context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific calibration for sun sensor

    return CASF_OK;
}

casf_result_t casf_sun_configure(void* context, const void* config, size_t config_size) {
    if (!context || !config || config_size == 0) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific configuration for sun sensor

    return CASF_OK;
}

casf_result_t casf_sun_shutdown(void* context) {
    if (!context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific shutdown for sun sensor

    return CASF_OK;
}

// Star tracker driver implementation
casf_result_t casf_star_probe(const casf_hal_t* hal, void* context) {
    if (!hal || !context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific probe logic for star tracker

    return CASF_OK;
}

casf_result_t casf_star_init(void* context, const casf_hal_t* hal) {
    if (!hal || !context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific initialization for star tracker

    return CASF_OK;
}

casf_result_t casf_star_read(void* context, casf_sensor_data_t* data) {
    if (!context || !data) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific reading logic for star tracker

    data->type = CASF_SENSOR_STAR_TRACKER;
    data->timestamp_us = 0; // Should be set by the hardware driver

    // Placeholder data - identity quaternion
    data->attitude.w = CASF_F(1.0f);
    data->attitude.x = CASF_F(0.0f);
    data->attitude.y = CASF_F(0.0f);
    data->attitude.z = CASF_F(0.0f);

    return CASF_OK;
}

casf_result_t casf_star_calibrate(void* context) {
    if (!context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific calibration for star tracker

    return CASF_OK;
}

casf_result_t casf_star_configure(void* context, const void* config, size_t config_size) {
    if (!context || !config || config_size == 0) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific configuration for star tracker

    return CASF_OK;
}

casf_result_t casf_star_shutdown(void* context) {
    if (!context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific shutdown for star tracker

    return CASF_OK;
}

// Forward declaration for the implementation below
casf_result_t casf_create_sensor_driver(casf_sensor_driver_t* driver, 
                                      casf_sensor_type_t type,
                                      const char* name,
                                      const char* manufacturer,
                                      const char* version);

    // Helper function to create a sensor driver with common parameters
    casf_result_t casf_create_sensor_driver(casf_sensor_driver_t* driver, 
                                      casf_sensor_type_t type,
                                      const char* name,
                                      const char* manufacturer,
                                      const char* version) {
    if (!driver || !name || !manufacturer) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Clear the driver structure
    memset(driver, 0, sizeof(casf_sensor_driver_t));

    // Set basic parameters
    driver->type = type;
    driver->name = name;
    driver->manufacturer = manufacturer;
    driver->version = version;
    driver->enabled = 1;
    driver->health_status = 100;

    // Set default values
    driver->current_rate_hz = 10; // Default 10 Hz
    driver->update_rate_max_hz = 100;    // Default max 100 Hz
    driver->update_rate_min_hz = 1;      // Default min 1 Hz

    // Assign function pointers based on sensor type
    switch (type) {
        case CASF_SENSOR_MAGNETOMETER:
            driver->probe = casf_mag_probe;
            driver->init = casf_mag_init;
            driver->read = casf_mag_read;
            driver->calibrate = casf_mag_calibrate;
            driver->configure = casf_mag_configure;
            driver->shutdown = casf_mag_shutdown;
            // Adding stub for self-test
            driver->self_test = NULL; // To be implemented
            break;

        case CASF_SENSOR_GYROSCOPE:
            driver->probe = casf_gyro_probe;
            driver->init = casf_gyro_init;
            driver->read = casf_gyro_read;
            driver->calibrate = casf_gyro_calibrate;
            driver->configure = casf_gyro_configure;
            driver->shutdown = casf_gyro_shutdown;
            // Adding stub for self-test
            driver->self_test = NULL; // To be implemented
            break;

        case CASF_SENSOR_SUN_SENSOR_COARSE:
        case CASF_SENSOR_SUN_SENSOR_FINE:
            driver->probe = casf_sun_probe;
            driver->init = casf_sun_init;
            driver->read = casf_sun_read;
            driver->calibrate = casf_sun_calibrate;
            driver->configure = casf_sun_configure;
            driver->shutdown = casf_sun_shutdown;
            // Adding stub for self-test
            driver->self_test = NULL; // To be implemented
            break;

        case CASF_SENSOR_STAR_TRACKER:
            driver->probe = casf_star_probe;
            driver->init = casf_star_init;
            driver->read = casf_star_read;
            driver->calibrate = casf_star_calibrate;
            driver->configure = casf_star_configure;
            driver->shutdown = casf_star_shutdown;
            // Adding stub for self-test
            driver->self_test = NULL; // To be implemented
            break;

        default:
            return CASF_ERROR_UNSUPPORTED;
    }

    return CASF_OK;
    }

casf_result_t casf_enable_sensor(casf_system_t* sys, casf_sensor_type_t type) {
    if (!sys || !sys->initialized) return CASF_ERROR_NOT_INITIALIZED;

    for (uint8_t i = 0; i < sys->sensor_count; ++i) {
        if (sys->sensors[i].type == type) {
            sys->sensors[i].enabled = 1;
            return CASF_OK;
        }
    }

    return CASF_ERROR_DEVICE_NOT_FOUND;
}

// Magnetometer driver implementation
casf_result_t casf_mag_probe(const casf_hal_t* hal, void* context) {
    if (!hal) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Context can be NULL for testing/simulation purposes
    // Implementation-specific probe logic for magnetometer
    // For example, checking if the device is present on I2C bus

    return CASF_OK;
}

casf_result_t casf_mag_init(void* context, const casf_hal_t* hal) {
    if (!hal) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Context can be NULL for testing/simulation purposes
    // Implementation-specific initialization for magnetometer

    return CASF_OK;
}

casf_result_t casf_mag_read(void* context, casf_sensor_data_t* data) {
    if (!data) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific reading logic for magnetometer
    // Context can be NULL for testing/simulation purposes

    data->type = CASF_SENSOR_MAGNETOMETER;
    data->timestamp_us = 0; // Should be set by the hardware driver

    // Placeholder data - in a real implementation, these would come from hardware
    data->magnetic_field.x = CASF_F(0.5f);
    data->magnetic_field.y = CASF_F(0.0f);
    data->magnetic_field.z = CASF_F(0.8f);

    return CASF_OK;
}

casf_result_t casf_mag_calibrate(void* context) {
    if (!context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific calibration for magnetometer

    return CASF_OK;
}

casf_result_t casf_mag_configure(void* context, const void* config, size_t config_size) {
    if (!context || !config || config_size == 0) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific configuration for magnetometer

    return CASF_OK;
}

casf_result_t casf_mag_shutdown(void* context) {
    if (!context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific shutdown for magnetometer

    return CASF_OK;
}

// Gyroscope driver implementation
casf_result_t casf_gyro_probe(const casf_hal_t* hal, void* context) {
    if (!hal || !context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific probe logic for gyroscope

    return CASF_OK;
}

casf_result_t casf_gyro_init(void* context, const casf_hal_t* hal) {
    if (!hal || !context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific initialization for gyroscope

    return CASF_OK;
}

casf_result_t casf_gyro_read(void* context, casf_sensor_data_t* data) {
    if (!context || !data) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific reading logic for gyroscope

    data->type = CASF_SENSOR_GYROSCOPE;
    data->timestamp_us = 0; // Should be set by the hardware driver

    // Placeholder data
    data->angular_rate.x = CASF_F(0.0f);
    data->angular_rate.y = CASF_F(0.0f);
    data->angular_rate.z = CASF_F(0.0f);

    return CASF_OK;
}

casf_result_t casf_gyro_calibrate(void* context) {
    if (!context) {
        return CASF_ERROR_INVALID_PARAM;
    }

    // Implementation-specific calibration for gyroscope

    return CASF_OK;
}

// Function declarations for sensor drivers moved to sensors.h

// Helper functions for sensor system control
// casf_enable_sensor is already defined above

casf_result_t casf_disable_sensor(casf_system_t* sys, casf_sensor_type_t type) {
    if (!sys || !sys->initialized) return CASF_ERROR_NOT_INITIALIZED;

    for (uint8_t i = 0; i < sys->sensor_count; ++i) {
        if (sys->sensors[i].type == type) {
            sys->sensors[i].enabled = 0;
            return CASF_OK;
        }
    }

    return CASF_ERROR_DEVICE_NOT_FOUND;
}
