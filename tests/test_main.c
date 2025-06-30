#include "control.h"
#include "sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

// Simple HAL implementation for testing
uint64_t test_time_us = 0;

uint64_t test_get_time_us(void) {
    return test_time_us;
}

void test_delay_us(uint32_t us) {
    test_time_us += us;
}

void test_log(uint8_t level, const char* format, ...) {
    // No-op for tests
    (void)level;
    (void)format;
}

// Test fixture setup
casf_hal_t create_test_hal(void) {
    casf_hal_t hal = {0};
    hal.get_time_us = test_get_time_us;
    hal.delay_us = test_delay_us;
    hal.log = test_log;
    hal.platform_name = "Test Platform";
    return hal;
}

casf_mission_config_t create_test_config(void) {
    casf_mission_config_t config = {0};
    config.mission_name = "Test Mission";
    config.spacecraft_id = "TEST-01";
    config.default_mode = CASF_MODE_SAFE;
    config.control_frequency_hz = 10;
    return config;
}

// Test cases
void test_init_shutdown(void) {
    printf("Testing init/shutdown... ");

    casf_hal_t hal = create_test_hal();
    casf_mission_config_t config = create_test_config();
    casf_system_t sys;

    // Test initialization
    casf_result_t result = casf_init(&sys, &hal, &config);
    assert(result == CASF_OK);
    assert(sys.initialized);
    assert(sys.current_mode == config.default_mode);

    // Test shutdown
    result = casf_shutdown(&sys);
    assert(result == CASF_OK);
    assert(!sys.initialized);

    printf("PASSED\n");
}

void test_version(void) {
    printf("Testing version functions... ");

    uint32_t version = casf_get_version();
    assert(version == ((CASF_VERSION_MAJOR << 16) | (CASF_VERSION_MINOR << 8) | CASF_VERSION_PATCH));

    // Test string conversions
    assert(strcmp(casf_result_to_string(CASF_OK), "OK") == 0);
    assert(strcmp(casf_mode_to_string(CASF_MODE_SAFE), "Safe") == 0);

    printf("PASSED\n");
}

void test_sensor_registration(void) {
    printf("Testing sensor registration... ");

    casf_hal_t hal = create_test_hal();
    casf_mission_config_t config = create_test_config();
    casf_system_t sys;

    // Initialize system
    casf_result_t result = casf_init(&sys, &hal, &config);
    assert(result == CASF_OK);

    // Create a sensor driver
    casf_sensor_driver_t driver;
    result = casf_create_sensor_driver(&driver, CASF_SENSOR_MAGNETOMETER, "Test Magnetometer", "Test Manufacturer");
    assert(result == CASF_OK);

    // Register the sensor
    result = casf_register_sensor(&sys, &driver);
    assert(result == CASF_OK);
    assert(sys.sensor_count == 1);

    // Verify the sensor was registered correctly
    assert(strcmp(sys.sensors[0].name, "Test Magnetometer") == 0);
    assert(sys.sensors[0].type == CASF_SENSOR_MAGNETOMETER);

    // Clean up
    result = casf_shutdown(&sys);
    assert(result == CASF_OK);

    printf("PASSED\n");
}

void test_mode_changes(void) {
    printf("Testing mode changes... ");

    casf_hal_t hal = create_test_hal();
    casf_mission_config_t config = create_test_config();
    casf_system_t sys;

    // Initialize system
    casf_result_t result = casf_init(&sys, &hal, &config);
    assert(result == CASF_OK);

    // Test setting mode
    result = casf_set_mode(&sys, CASF_MODE_DETUMBLE);
    assert(result == CASF_OK);
    assert(sys.requested_mode == CASF_MODE_DETUMBLE);

    // Run update to apply mode change
    result = casf_update(&sys);
    assert(result == CASF_OK);
    assert(sys.current_mode == CASF_MODE_DETUMBLE);

    // Test getting mode
    assert(casf_get_mode(&sys) == CASF_MODE_DETUMBLE);

    // Test emergency stop
    result = casf_emergency_stop(&sys);
    assert(result == CASF_OK);
    assert(sys.emergency_stop);
    assert(sys.current_mode == CASF_MODE_SAFE);

    // Try setting a non-safe mode while in emergency stop (should fail)
    result = casf_set_mode(&sys, CASF_MODE_DETUMBLE);
    assert(result == CASF_ERROR_SAFETY_VIOLATION);

    // Clean up
    result = casf_shutdown(&sys);
    assert(result == CASF_OK);

    printf("PASSED\n");
}

void test_math_utilities(void) {
    printf("Testing math utilities... ");

    // Test quaternion multiplication
    casf_quaternion_t q1 = {CASF_F(1.0f), CASF_F(0.0f), CASF_F(0.0f), CASF_F(0.0f)};
    casf_quaternion_t q2 = {CASF_F(0.0f), CASF_F(1.0f), CASF_F(0.0f), CASF_F(0.0f)};
    casf_quaternion_t result;

    casf_quaternion_multiply(&q1, &q2, &result);
    assert(result.w == CASF_F(0.0f));
    assert(result.x == CASF_F(1.0f));
    assert(result.y == CASF_F(0.0f));
    assert(result.z == CASF_F(0.0f));

    // Test vector magnitude
    casf_vector3_t v = {CASF_F(3.0f), CASF_F(4.0f), CASF_F(0.0f)};
    casf_float_t mag = casf_vector3_magnitude(&v);

    // Allow small floating-point differences
    float mag_f = CASF_TO_F(mag);
    assert(mag_f > 4.99f && mag_f < 5.01f);

    // Test vector cross product
    casf_vector3_t a = {CASF_F(1.0f), CASF_F(0.0f), CASF_F(0.0f)};
    casf_vector3_t b = {CASF_F(0.0f), CASF_F(1.0f), CASF_F(0.0f)};
    casf_vector3_t cross;

    casf_vector3_cross(&a, &b, &cross);
    assert(cross.x == CASF_F(0.0f));
    assert(cross.y == CASF_F(0.0f));
    assert(cross.z == CASF_F(1.0f));

    printf("PASSED\n");
}

// Main test runner
int main() {
    printf("Running CASF unit tests\n");
    printf("======================\n\n");

    test_init_shutdown();
    test_version();
    test_sensor_registration();
    test_mode_changes();
    test_math_utilities();

    printf("\nAll tests passed!\n");
    return 0;
}
