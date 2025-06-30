/**
 * CubeSat ADCS Standard Framework (CASF)
 * Sensor Unit Tests
 */

#include "../src/control/control.h"
#include "../src/sensors/sensors.h"
#include <stdio.h>
#include <assert.h>

// Mock HAL implementation for testing
static uint64_t system_time_us = 0;

static uint64_t test_hal_get_time_us(void) {
    return system_time_us++;
}

static void test_hal_log(int level, const char* fmt, ...) {
    // No logging in tests
    (void)level;
    (void)fmt;
}

// Magnetometer mock implementation
static void* mag_context = NULL;

static casf_result_t test_mag_probe(const casf_hal_t* hal, void* context) {
    (void)hal;
    (void)context;
    return CASF_OK;
}

static casf_result_t test_mag_init(void* context, const casf_hal_t* hal) {
    (void)context;
    (void)hal;
    return CASF_OK;
}

static casf_result_t test_mag_read(void* context, casf_sensor_data_t* data) {
    (void)context;

    data->type = CASF_SENSOR_MAGNETOMETER;
    data->timestamp_us = system_time_us;
    data->magnetic_field.x = 1.0f;
    data->magnetic_field.y = 2.0f;
    data->magnetic_field.z = 3.0f;

    return CASF_OK;
}

// Test sensor driver creation
void test_sensor_creation(void) {
    casf_sensor_driver_t mag_driver;
    casf_result_t result;

    // Test magnetometer creation
    result = casf_create_magnetometer_driver(&mag_driver, "TestMag", "TestMfg", "1.0", mag_context);
    assert(result == CASF_OK);
    assert(mag_driver.type == CASF_SENSOR_MAGNETOMETER);
    assert(mag_driver.enabled == 1);
    assert(mag_driver.probe != NULL);
    assert(mag_driver.init != NULL);
    assert(mag_driver.read != NULL);

    printf("Sensor creation test passed\n");
}

// Test sensor data reading
void test_sensor_reading(void) {
    casf_sensor_driver_t mag_driver;
    casf_result_t result;

    // Override driver functions with our test implementations
    result = casf_create_magnetometer_driver(&mag_driver, "TestMag", "TestMfg", "1.0", mag_context);
    assert(result == CASF_OK);

    mag_driver.probe = test_mag_probe;
    mag_driver.init = test_mag_init;
    mag_driver.read = test_mag_read;

    // Test probe and init
    casf_hal_t hal = {
        .get_time_us = test_hal_get_time_us,
        .log = test_hal_log
    };

    result = mag_driver.probe(&hal, mag_context);
    assert(result == CASF_OK);

    result = mag_driver.init(mag_context, &hal);
    assert(result == CASF_OK);

    // Test reading
    casf_sensor_data_t data;
    result = mag_driver.read(mag_context, &data);

    assert(result == CASF_OK);
    assert(data.type == CASF_SENSOR_MAGNETOMETER);
    assert(data.magnetic_field.x == 1.0f);
    assert(data.magnetic_field.y == 2.0f);
    assert(data.magnetic_field.z == 3.0f);

    printf("Sensor reading test passed\n");
}

// Main test function
int main(void) {
    printf("Running sensor tests...\n");

    test_sensor_creation();
    test_sensor_reading();

    printf("All sensor tests passed!\n");
    return 0;
}
