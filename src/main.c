/**
 * CubeSat ADCS Standard Framework (CASF)
 * Main application entry point
 */

#include "control/control.h"
#include "sensors/sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

// Simple HAL implementation
static uint64_t system_time_us = 0;

static uint64_t hal_get_time_us(void) {
    return system_time_us++;
}

static void hal_log(int level, const char* fmt, ...) {
    // Simple logging to stdout
    va_list args;
    va_start(args, fmt);
    printf("[%d] ", level);
    vprintf(fmt, args);
    printf("\n");
    va_end(args);
}

int main(void) {
    // Initialize HAL
    casf_hal_t hal = {
        .get_time_us = hal_get_time_us,
        .log = hal_log
    };
/**
 * PitchPlease - CubeSat ADCS Standard Framework (CASF)
 * Main Application
 */

#include "control/control.h"
#include "sensors/sensors.h"
#ifdef BUILD_DYNAMICS
#include "dynamics/dynamics.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[]) {
    printf("PitchPlease - CubeSat ADCS Standard Framework (CASF)\n");
    printf("======================================\n\n");

    printf("Available commands:\n");
    printf("  -h, --help    : Show this help message\n");

#ifdef BUILD_DYNAMICS
    printf("  --detumble    : Run detumbling simulation\n");
#endif

    printf("\n");

    // Parse command line arguments
    if (argc > 1) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            // Help already displayed above
            return 0;
        }
#ifdef BUILD_DYNAMICS
        else if (strcmp(argv[1], "--detumble") == 0) {
            printf("Please run the casf_detumbling example directly.\n");
            return 0;
        }
#endif
        else {
            printf("Unknown command: %s\n\n", argv[1]);
        }
    }

    printf("This is a placeholder main application.\n");
    printf("Please run the specific examples for functionality demonstrations.\n\n");

    return 0;
}
    // For compatibility with extended_hal interface
    casf_extended_hal_t extended_hal = {0};
    extended_hal.base_hal = hal;

    // Initialize system
    casf_system_t system;
    casf_mission_config_t config = {0}; // Default configuration

    casf_result_t result = casf_init(&system, &hal, &config);
    if (result != CASF_OK) {
        printf("System initialization failed: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Create and register sensors
    casf_sensor_driver_t magnetometer;
    // Create a context for the magnetometer
    void* mag_context = malloc(sizeof(int)); // Simple context allocation
    if (!mag_context) {
        printf("Failed to allocate context for magnetometer\n");
        return 1;
    }
    result = casf_create_magnetometer_driver(&magnetometer, "MAG3110", "NXP", "1.0", mag_context);
    if (result != CASF_OK) {
        printf("Failed to create magnetometer: %s\n", casf_result_to_string(result));
        free(mag_context);
        return 1;
    }

    result = casf_register_sensor(&system, &magnetometer);
    if (result != CASF_OK) {
        printf("Failed to register magnetometer: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Start the system
    result = casf_start(&system);
    if (result != CASF_OK) {
        printf("System start failed: %s\n", casf_result_to_string(result));
        return 1;
    }

    printf("CASF System initialized and running.\n");

    // Run for a few control cycles
    for (int i = 0; i < 10; i++) {
        result = casf_run_control_cycle(&system);
        if (result != CASF_OK) {
            printf("Control cycle failed: %s\n", casf_result_to_string(result));
            break;
        }
    }

    // Shutdown the system
    casf_shutdown(&system);

    // Free allocated memory
    if (magnetometer.context) {
        free(magnetometer.context);
    }

    return 0;
}
