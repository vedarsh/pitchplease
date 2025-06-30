/**
 * PitchPlease - CubeSat ADCS Standard Framework (CASF)
 * Main Application
 */

#include "control.h"
#include "sensors.h"
#ifdef BUILD_DYNAMICS
#include "dynamics.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

int main(int argc, char* argv[]) {
    printf("PitchPlease - CubeSat ADCS Standard Framework (CASF)\n");
    printf("======================================\n\n");

    // Initialize HAL
    casf_hal_t hal = {
        .get_time_us = hal_get_time_us,
        .log = hal_log
    };

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

    // Initialize system
    casf_system_t system;
    casf_mission_config_t config = {0}; // Default configuration
    config.mission_name = "PitchPlease Demo";
    config.spacecraft_id = "PITCH-01";
    config.default_mode = CASF_MODE_SAFE;
    config.control_frequency_hz = 10;

    casf_result_t result = casf_init(&system, &hal, &config);
    if (result != CASF_OK) {
        printf("System initialization failed: %s\n", casf_result_to_string(result));
        return 1;
    }

    printf("CASF System initialized.\n");
    printf("To see active demonstrations, run the examples:\n");
    printf("  ./casf_example - Basic ADCS functionality\n");

#ifdef BUILD_DYNAMICS
    printf("  ./casf_detumbling - Detumbling simulation\n");
#endif

    printf("\n");

    // Shutdown the system
    casf_shutdown(&system);

    return 0;
}
