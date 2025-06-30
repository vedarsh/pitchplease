/**
 * CubeSat ADCS Standard Framework (CASF)
 * Sensors Interface
 */

#ifndef CASF_SENSORS_H
#define CASF_SENSORS_H

#include "sensor_types.h"
#include "control.h"

// Forward declarations
typedef struct casf_system_s casf_system_t;

#ifdef __cplusplus
extern "C" {
#endif

// Magnetometer driver implementation
casf_result_t casf_mag_probe(const casf_hal_t* hal, void* context);
casf_result_t casf_mag_init(void* context, const casf_hal_t* hal);
casf_result_t casf_mag_read(void* context, casf_sensor_data_t* data);
casf_result_t casf_mag_calibrate(void* context);
casf_result_t casf_mag_configure(void* context, const void* config, size_t config_size);
casf_result_t casf_mag_shutdown(void* context);
casf_result_t casf_mag_self_test(void* context);

// Gyroscope driver implementation
casf_result_t casf_gyro_probe(const casf_hal_t* hal, void* context);
casf_result_t casf_gyro_init(void* context, const casf_hal_t* hal);
casf_result_t casf_gyro_read(void* context, casf_sensor_data_t* data);
casf_result_t casf_gyro_calibrate(void* context);
casf_result_t casf_gyro_configure(void* context, const void* config, size_t config_size);
casf_result_t casf_gyro_shutdown(void* context);
casf_result_t casf_gyro_self_test(void* context);

// Sun sensor driver implementation
casf_result_t casf_sun_probe(const casf_hal_t* hal, void* context);
casf_result_t casf_sun_init(void* context, const casf_hal_t* hal);
casf_result_t casf_sun_read(void* context, casf_sensor_data_t* data);
casf_result_t casf_sun_calibrate(void* context);
casf_result_t casf_sun_configure(void* context, const void* config, size_t config_size);
casf_result_t casf_sun_shutdown(void* context);
casf_result_t casf_sun_self_test(void* context);

// Star tracker driver implementation
casf_result_t casf_star_probe(const casf_hal_t* hal, void* context);
casf_result_t casf_star_init(void* context, const casf_hal_t* hal);
casf_result_t casf_star_read(void* context, casf_sensor_data_t* data);
casf_result_t casf_star_calibrate(void* context);
casf_result_t casf_star_configure(void* context, const void* config, size_t config_size);
casf_result_t casf_star_shutdown(void* context);
casf_result_t casf_star_self_test(void* context);

// Sensor creation helpers
casf_result_t casf_create_gyroscope_driver(casf_sensor_driver_t* driver,
                                        const char* name, 
                                        const char* manufacturer,
                                        const char* version,
                                        void* context);

casf_result_t casf_create_magnetometer_driver(casf_sensor_driver_t* driver,
                                          const char* name, 
                                          const char* manufacturer,
                                          const char* version,
                                          void* context);

casf_result_t casf_create_sun_sensor_driver(casf_sensor_driver_t* driver,
                                         casf_sensor_type_t type, // COARSE or FINE
                                         const char* name, 
                                         const char* manufacturer,
                                         const char* version,
                                         void* context);

casf_result_t casf_create_star_tracker_driver(casf_sensor_driver_t* driver,
                                           const char* name, 
                                           const char* manufacturer,
                                           const char* version,
                                           void* context);

                                           // Base driver creation function
                                           casf_result_t casf_create_sensor_driver(casf_sensor_driver_t* driver,
                                                                                 casf_sensor_type_t type,
                                                                                 const char* name,
                                                                                 const char* manufacturer,
                                                                                 const char* version);

// Sensor control
casf_result_t casf_enable_sensor(casf_system_t* sys, casf_sensor_type_t type);
casf_result_t casf_disable_sensor(casf_system_t* sys, casf_sensor_type_t type);

#ifdef __cplusplus
}
#endif

#endif // CASF_SENSORS_H
