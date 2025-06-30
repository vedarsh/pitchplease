/**
 * CubeSat ADCS Standard Framework (CASF)
 * Control System Unit Tests
 */

#include "../src/control/control.h"
#include "../src/sensors/sensors.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

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

// Test system initialization
void test_system_init(void) {
    // Initialize HAL
    casf_hal_t hal = {
        .get_time_us = test_hal_get_time_us,
        .log = test_hal_log,
        .platform_name = "Test Platform"
    };

    // Initialize configuration
    casf_mission_config_t config = {
        .mission_name = "Test Mission",
        .spacecraft_id = "TEST-01",
        .default_mode = CASF_MODE_SAFE,
        .control_frequency_hz = 10
    };

    // Initialize system
    casf_system_t system;
    casf_result_t result = casf_init(&system, &hal, &config);

    assert(result == CASF_OK);
    assert(system.initialized == 1);
    assert(system.current_mode == CASF_MODE_SAFE);
    assert(system.sensor_count == 0);
    assert(system.actuator_count == 0);

    printf("System initialization test passed\n");
}

// Test mode changes
void test_mode_changes(void) {
    // Initialize HAL
    casf_hal_t hal = {
        .get_time_us = test_hal_get_time_us,
        .log = test_hal_log
    };

    // Initialize configuration
    casf_mission_config_t config = {
        .mission_name = "Test Mission",
        .spacecraft_id = "TEST-01",
        .default_mode = CASF_MODE_SAFE,
        .control_frequency_hz = 10
    };

    // Initialize system
    casf_system_t system;
    casf_result_t result = casf_init(&system, &hal, &config);
    assert(result == CASF_OK);

    // Test mode changes
    result = casf_set_mode(&system, CASF_MODE_DETUMBLE);
    assert(result == CASF_OK);
    assert(system.requested_mode == CASF_MODE_DETUMBLE);

    // Update system to apply mode change
    result = casf_update(&system);
    assert(result == CASF_OK);
    assert(system.current_mode == CASF_MODE_DETUMBLE);

    // Test invalid mode
    result = casf_set_mode(&system, CASF_MODE_COUNT);
    assert(result == CASF_ERROR_INVALID_PARAM);

    printf("Mode change test passed\n");
}

// Test quaternion math
void test_quaternion_math(void) {
    // Test quaternion multiplication
    casf_quaternion_t q1 = {1.0f, 0.0f, 0.0f, 0.0f}; // Identity
    casf_quaternion_t q2 = {0.0f, 1.0f, 0.0f, 0.0f}; // 180° around X
    casf_quaternion_t result;

    casf_quaternion_multiply(&q1, &q2, &result);

    // Should be q2 when multiplying by identity
    assert(fabs(result.w - q2.w) < 0.001f);
    assert(fabs(result.x - q2.x) < 0.001f);
    assert(fabs(result.y - q2.y) < 0.001f);
    assert(fabs(result.z - q2.z) < 0.001f);

    // Test quaternion to Euler conversion
    // Identity quaternion should give zero Euler angles
    casf_quaternion_t identity = {1.0f, 0.0f, 0.0f, 0.0f};
    casf_euler_t euler;

    casf_quaternion_to_euler(&identity, &euler);

    assert(fabs(euler.roll) < 0.001f);
    assert(fabs(euler.pitch) < 0.001f);
    assert(fabs(euler.yaw) < 0.001f);

    printf("Quaternion math test passed\n");
}

// Main test function
int main(void) {
    printf("Running control system tests...\n");

    test_system_init();
    test_mode_changes();
    test_quaternion_math();

    printf("All control system tests passed!\n");
    return 0;
}
