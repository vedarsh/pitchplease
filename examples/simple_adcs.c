#include "control.h"
#include "sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <unistd.h>
#include <time.h>

// Mock HAL implementation for example purposes
uint64_t mock_get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

void mock_delay_us(uint32_t us) {
    usleep(us);
}

void mock_log(int level, const char* format, ...) {
    va_list args;
    va_start(args, format);
    printf("[LOG %d] ", level);
    vprintf(format, args);
    printf("\n");
    va_end(args);
}

// Mock magnetometer context
typedef struct {
    float noise_level;
    float field_strength;
} mock_mag_context_t;

// Mock gyroscope context
typedef struct {
    float noise_level;
    float drift_rate;
} mock_gyro_context_t;

// Simulated magnetic field calculation
void calculate_magnetic_field(float time_s, casf_vector3_t* field) {
    // Simple rotating magnetic field
    field->x = CASF_F(0.4f * cosf(time_s * 0.1f));
    field->y = CASF_F(0.4f * sinf(time_s * 0.1f));
    field->z = CASF_F(0.8f);

    // Normalize to 1.0 strength
    float mag = sqrtf(CASF_TO_F(field->x) * CASF_TO_F(field->x) + 
                     CASF_TO_F(field->y) * CASF_TO_F(field->y) + 
                     CASF_TO_F(field->z) * CASF_TO_F(field->z));

    field->x = CASF_F(CASF_TO_F(field->x) / mag);
    field->y = CASF_F(CASF_TO_F(field->y) / mag);
    field->z = CASF_F(CASF_TO_F(field->z) / mag);
}

// Simulation functions for magnetometer
casf_result_t mock_mag_read(void* context, casf_sensor_data_t* data) {
    mock_mag_context_t* ctx = (mock_mag_context_t*)context;

    // Get simulated time
    static uint64_t start_time = 0;
    uint64_t current_time = mock_get_time_us();
    if (start_time == 0) start_time = current_time;

    float time_s = (current_time - start_time) / 1000000.0f;

    // Calculate ideal magnetic field
    calculate_magnetic_field(time_s, &data->magnetic_field);

    // Add noise
    float noise_x = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * ctx->noise_level;
    float noise_y = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * ctx->noise_level;
    float noise_z = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * ctx->noise_level;

    data->magnetic_field.x += CASF_F(noise_x);
    data->magnetic_field.y += CASF_F(noise_y);
    data->magnetic_field.z += CASF_F(noise_z);

    // Scale by field strength
    data->magnetic_field.x = CASF_F(CASF_TO_F(data->magnetic_field.x) * ctx->field_strength);
    data->magnetic_field.y = CASF_F(CASF_TO_F(data->magnetic_field.y) * ctx->field_strength);
    data->magnetic_field.z = CASF_F(CASF_TO_F(data->magnetic_field.z) * ctx->field_strength);

    data->type = CASF_SENSOR_MAGNETOMETER;
    data->timestamp_us = current_time;

    return CASF_OK;
}

// Simulation functions for gyroscope
casf_result_t mock_gyro_read(void* context, casf_sensor_data_t* data) {
    mock_gyro_context_t* ctx = (mock_gyro_context_t*)context;

    // Get simulated time
    static uint64_t start_time = 0;
    uint64_t current_time = mock_get_time_us();
    if (start_time == 0) start_time = current_time;

    float time_s = (current_time - start_time) / 1000000.0f;

    // Simulated angular velocity (spinning around Z axis)
    data->angular_rate.x = CASF_F(sinf(time_s * 0.2f) * 0.05f);
    data->angular_rate.y = CASF_F(cosf(time_s * 0.3f) * 0.08f);
    data->angular_rate.z = CASF_F(0.2f + sinf(time_s * 0.1f) * 0.05f);

    // Add noise and drift
    float noise_x = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * ctx->noise_level;
    float noise_y = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * ctx->noise_level;
    float noise_z = ((float)rand() / RAND_MAX * 2.0f - 1.0f) * ctx->noise_level;

    data->angular_rate.x += CASF_F(noise_x + ctx->drift_rate * time_s * 0.001f);
    data->angular_rate.y += CASF_F(noise_y + ctx->drift_rate * time_s * 0.002f);
    data->angular_rate.z += CASF_F(noise_z);

    data->type = CASF_SENSOR_GYROSCOPE;
    data->timestamp_us = current_time;

    return CASF_OK;
}

int main() {
    printf("CASF Example Application\n");
    printf("======================\n\n");

    // Initialize random number generator
    srand(time(NULL));

    // Create HAL
    casf_extended_hal_t extended_hal = {0};
    extended_hal.base_hal.get_time_us = mock_get_time_us;
    extended_hal.base_hal.log = mock_log;
    extended_hal.delay_us = mock_delay_us;
    extended_hal.platform_name = "Example Platform";

    // Create a pointer to the base HAL structure
    casf_hal_t* hal = &extended_hal.base_hal;

    // Create mission configuration
    casf_mission_config_t config = {0};
    config.mission_name = "CASF Example Mission";
    config.spacecraft_id = "CASF-01";
    config.default_mode = CASF_MODE_DETUMBLE;
    config.control_frequency_hz = 10;

    // Configure inertia tensor (kg*m^2)
    config.inertia_tensor[0].x = CASF_F(0.01f);
    config.inertia_tensor[0].y = CASF_F(0.0f);
    config.inertia_tensor[0].z = CASF_F(0.0f);

    config.inertia_tensor[1].x = CASF_F(0.0f);
    config.inertia_tensor[1].y = CASF_F(0.01f);
    config.inertia_tensor[1].z = CASF_F(0.0f);

    config.inertia_tensor[2].x = CASF_F(0.0f);
    config.inertia_tensor[2].y = CASF_F(0.0f);
    config.inertia_tensor[2].z = CASF_F(0.01f);

    // Configure detumble mode
    config.control_params[CASF_MODE_DETUMBLE].max_torque = CASF_F(0.001f);

    // Initialize CASF system
    casf_system_t sys;
    casf_result_t result = casf_init(&sys, hal, &config);
    if (result != CASF_OK) {
        printf("Failed to initialize CASF system: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Create magnetometer sensor
    mock_mag_context_t* mag_ctx = malloc(sizeof(mock_mag_context_t));
    if (!mag_ctx) {
        printf("Failed to allocate memory for magnetometer context\n");
        return 1;
    }
    mag_ctx->noise_level = 0.01f;
    mag_ctx->field_strength = 1.0f;

    casf_sensor_driver_t mag_driver;
    result = casf_create_sensor_driver(&mag_driver, CASF_SENSOR_MAGNETOMETER, "Mock Magnetometer", "CASF Example", "1.0");
    if (result != CASF_OK) {
        printf("Failed to create magnetometer driver: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Override with mock implementation
    mag_driver.context = mag_ctx;
    mag_driver.read = mock_mag_read;
    mag_driver.current_rate_hz = 20;

    // Register magnetometer
    result = casf_register_sensor(&sys, &mag_driver);
    if (result != CASF_OK) {
        printf("Failed to register magnetometer: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Create gyroscope sensor
    mock_gyro_context_t* gyro_ctx = malloc(sizeof(mock_gyro_context_t));
    if (!gyro_ctx) {
        printf("Failed to allocate memory for gyroscope context\n");
        return 1;
    }
    gyro_ctx->noise_level = 0.005f;
    gyro_ctx->drift_rate = 0.001f;

    casf_sensor_driver_t gyro_driver;
    result = casf_create_sensor_driver(&gyro_driver, CASF_SENSOR_GYROSCOPE, "Mock Gyroscope", "CASF Example", "1.0");
    if (result != CASF_OK) {
        printf("Failed to create gyroscope driver: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Override with mock implementation
    gyro_driver.context = gyro_ctx;
    gyro_driver.read = mock_gyro_read;
    gyro_driver.current_rate_hz = 50;

    // Register gyroscope
    result = casf_register_sensor(&sys, &gyro_driver);
    if (result != CASF_OK) {
        printf("Failed to register gyroscope: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Start CASF system
    result = casf_start(&sys);
    if (result != CASF_OK) {
        printf("Failed to start CASF system: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Run simulation for 10 seconds
    printf("Running ADCS simulation for 10 seconds...\n");
    printf("Time(s) | Mode | AngVel(rad/s) | MagField\n");
    printf("----------------------------------------------\n");

    uint64_t start_time = mock_get_time_us();
    uint64_t end_time = start_time + 10000000; // 10 seconds

    while (mock_get_time_us() < end_time) {
        // Update CASF system
        result = casf_update(&sys);
        if (result != CASF_OK) {
            printf("Error updating CASF system: %s\n", casf_result_to_string(result));
            break;
        }

        // Print system state every 500ms
        static uint64_t last_print = 0;
        uint64_t now = mock_get_time_us();
        if (now - last_print >= 500000) {
            float time_s = (now - start_time) / 1000000.0f;
            const casf_system_state_t* state = casf_get_state(&sys);
/**
 * PitchPlease - CubeSat ADCS Standard Framework (CASF)
 * Simple ADCS Example
 */

#include "control/control.h"
#include "sensors/sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>  // For sleep function

// Simulation time
static uint64_t sim_time_us = 0;

// HAL implementation for example
static uint64_t example_get_time_us(void) {
    return sim_time_us;
}

static void example_delay_us(uint32_t us) {
    sim_time_us += us;
    // Simulate delay with a scaled sleep
    usleep(us / 1000); // Scale for faster simulation
}

static void example_log(int level, const char* fmt, ...) {
    char prefix[32];
    switch (level) {
        case 0: strcpy(prefix, "[ERROR] "); break;
        case 1: strcpy(prefix, "[WARN]  "); break;
        case 2: strcpy(prefix, "[INFO]  "); break;
        case 3: strcpy(prefix, "[DEBUG] "); break;
        default: strcpy(prefix, "[LOG]   "); break;
    }

    printf("%s", prefix);

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    printf("\n");
}

int main(int argc, char* argv[]) {
    printf("CASF Simple ADCS Example\n");
    printf("======================\n\n");

    // Initialize HAL
    casf_hal_t hal = {
        .get_time_us = example_get_time_us,
        .delay_us = example_delay_us,
        .log = example_log
    };

    // Configure the mission parameters
    casf_mission_config_t config = {
        .mission_name = "CASF Example Mission",
        .spacecraft_id = "CASF-01",
        .default_mode = CASF_MODE_SAFE,
        .control_frequency_hz = 10
    };

    // Initialize the CASF system
    casf_system_t sys;
    casf_result_t result = casf_init(&sys, &hal, &config);
    if (result != CASF_OK) {
        printf("Failed to initialize CASF system: %d\n", result);
        return 1;
    }

    // This is a simple placeholder example
    printf("CASF system initialized successfully!\n");
    printf("Running simple control loop...\n\n");

    // Simple control loop (10 iterations)
    for (int i = 0; i < 10; i++) {
        // Update the ADCS system
        result = casf_update(&sys);
        if (result != CASF_OK) {
            printf("Error during update: %d\n", result);
            break;
        }

        // Simulate some processing
        printf("Update cycle %d complete, time: %lu ms\n", i+1, sim_time_us/1000);

        // Delay to maintain control rate
        hal.delay_us(100000); // 100ms
    }

    printf("\nSimple ADCS example completed successfully!\n");
    return 0;
}
            printf("%6.2f | %s | [%6.3f %6.3f %6.3f] | [%6.3f %6.3f %6.3f]\n",
                   time_s,
                   casf_mode_to_string(casf_get_mode(&sys)),
                   CASF_TO_F(state->angular_velocity.x),
                   CASF_TO_F(state->angular_velocity.y),
                   CASF_TO_F(state->angular_velocity.z),
                   CASF_TO_F(state->magnetic_field_body.x),
                   CASF_TO_F(state->magnetic_field_body.y),
                   CASF_TO_F(state->magnetic_field_body.z));

            last_print = now;
        }

        // Short delay
        mock_delay_us(10000); // 10 ms
    }

    // Shutdown
    result = casf_shutdown(&sys);
    if (result != CASF_OK) {
        printf("Error shutting down CASF system: %s\n", casf_result_to_string(result));
    }

    // Cleanup
    free(mag_ctx);
    free(gyro_ctx);

    printf("\nSimulation complete!\n");
    return 0;
}
