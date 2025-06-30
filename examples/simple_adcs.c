/**
 * Simple ADCS Example
 * Demonstrates basic CASF functionality with mock sensors
 */

#include "control.h"
#include "sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

// Global time counter for simulation
static uint64_t simulation_time_us = 0;

// Mock sensor contexts
typedef struct {
    float bias_x, bias_y, bias_z;
    float noise_level;
} mock_mag_context_t;

typedef struct {
    float bias_x, bias_y, bias_z;
    float noise_level;
} mock_gyro_context_t;

// Mock HAL functions
static uint64_t example_get_time_us(void) 
{
    simulation_time_us += 1000; // Increment by 1ms
    return simulation_time_us;
}

static void example_delay_us(uint32_t us) 
{
    simulation_time_us += us;
}

static void example_log(int level, const char* fmt, ...) 
{
    va_list args;
    va_start(args, fmt);
    
    printf("[%d] ", level);
    vprintf(fmt, args);
    printf("\n");
    
    va_end(args);
}

// Mock sensor implementations
void calculate_magnetic_field(float time_s, casf_vector3_t* field) 
{
    // Simulate simple dipole field that rotates over time
    float rotation = time_s * 0.01f; // 100s for full rotation
    
    field->x = CASF_F(25000.0f * cosf(rotation)); // nT
    field->y = CASF_F(15000.0f * sinf(rotation));
    field->z = CASF_F(-40000.0f);
}

casf_result_t mock_mag_read(void* context, casf_sensor_data_t* data) 
{
    if (!context || !data) {
        return CASF_ERROR_INVALID_PARAM;
    }
    
    mock_mag_context_t* mag_ctx = (mock_mag_context_t*)context;
    float time_s = simulation_time_us / 1000000.0f;
    
    // Calculate base magnetic field
    calculate_magnetic_field(time_s, &data->magnetic_field);
    
    // Add bias and noise
    data->magnetic_field.x += CASF_F(mag_ctx->bias_x + (rand() / (float)RAND_MAX - 0.5f) * mag_ctx->noise_level);
    data->magnetic_field.y += CASF_F(mag_ctx->bias_y + (rand() / (float)RAND_MAX - 0.5f) * mag_ctx->noise_level);
    data->magnetic_field.z += CASF_F(mag_ctx->bias_z + (rand() / (float)RAND_MAX - 0.5f) * mag_ctx->noise_level);
    
    data->timestamp_us = simulation_time_us;
    // Temperature field not available in sensor_data_t
    
    return CASF_OK;
}

casf_result_t mock_gyro_read(void* context, casf_sensor_data_t* data) 
{
    if (!context || !data) {
        return CASF_ERROR_INVALID_PARAM;
    }
    
    mock_gyro_context_t* gyro_ctx = (mock_gyro_context_t*)context;
    
    // Simulate decaying tumble rates
    float time_s = simulation_time_us / 1000000.0f;
    float decay = expf(-time_s / 100.0f); // 100s time constant
    
    data->angular_rate.x = CASF_F(0.1f * decay + gyro_ctx->bias_x + (rand() / (float)RAND_MAX - 0.5f) * gyro_ctx->noise_level);
    data->angular_rate.y = CASF_F(0.05f * decay + gyro_ctx->bias_y + (rand() / (float)RAND_MAX - 0.5f) * gyro_ctx->noise_level);
    data->angular_rate.z = CASF_F(-0.02f * decay + gyro_ctx->bias_z + (rand() / (float)RAND_MAX - 0.5f) * gyro_ctx->noise_level);
    
    data->timestamp_us = simulation_time_us;
    // Temperature field not available in sensor_data_t
    
    return CASF_OK;
}

// Basic probe function that always succeeds
casf_result_t mock_sensor_probe(const casf_hal_t* hal, void* context) {
    (void)hal;
    (void)context;
    return CASF_OK;
}

// Basic init function that always succeeds
casf_result_t mock_sensor_init(void* context, const casf_hal_t* hal) {
    (void)context;
    (void)hal;
    return CASF_OK;
}

int main(int argc, char* argv[]) 
{
    printf("CASF Simple ADCS Example\n");
    printf("========================\n\n");
    
    // Initialize HAL
    casf_hal_t hal = {
        .get_time_us = example_get_time_us,
        .log = example_log
    };
    
    // Configure mission
    casf_mission_config_t config = {0};
    config.mission_name = "Simple ADCS Demo";
    config.spacecraft_id = "DEMO-01";
    config.default_mode = CASF_MODE_SAFE;
    config.control_frequency_hz = 10;
    
    // Initialize system
    casf_system_t system;
    casf_result_t result = casf_init(&system, &hal, &config);
    if (result != CASF_OK) {
        printf("Failed to initialize CASF system: %s\n", casf_result_to_string(result));
        return 1;
    }
    
    // Create mock sensor contexts
    static mock_mag_context_t mag_context = {
        .bias_x = 100.0f, .bias_y = -50.0f, .bias_z = 200.0f,
        .noise_level = 500.0f
    };
    
    static mock_gyro_context_t gyro_context = {
        .bias_x = 0.001f, .bias_y = -0.0005f, .bias_z = 0.0008f,
        .noise_level = 0.01f
    };
    
    // Create sensor drivers
    casf_sensor_driver_t mag_driver = {
        .name = "Mock Magnetometer",
        .manufacturer = "CASF Demo",
        .version = "1.0",
        .type = CASF_SENSOR_MAGNETOMETER,
        .id = 1,
        .probe = mock_sensor_probe,
        .init = mock_sensor_init,
        .read = mock_mag_read,
        .context = &mag_context,
        .update_rate_max_hz = 100,
        .update_rate_min_hz = 1,
        .current_rate_hz = 10,
        .enabled = 1,
        .calibrated = 1
    };
    
    casf_sensor_driver_t gyro_driver = {
        .name = "Mock Gyroscope",
        .manufacturer = "CASF Demo", 
        .version = "1.0",
        .type = CASF_SENSOR_GYROSCOPE,
        .id = 2,
        .probe = mock_sensor_probe,
        .init = mock_sensor_init,
        .read = mock_gyro_read,
        .context = &gyro_context,
        .update_rate_max_hz = 100,
        .update_rate_min_hz = 1,
        .current_rate_hz = 50,
        .enabled = 1,
        .calibrated = 1
    };
    
    // Register sensors
    result = casf_register_sensor(&system, &mag_driver);
    if (result != CASF_OK) {
        printf("Failed to register magnetometer: %s\n", casf_result_to_string(result));
        return 1;
    }
    
    result = casf_register_sensor(&system, &gyro_driver);
    if (result != CASF_OK) {
        printf("Failed to register gyroscope: %s\n", casf_result_to_string(result));
        return 1;
    }
    
    printf("Registered %d sensors\n", system.sensor_count);
    
    // Start the system
    result = casf_start(&system);
    if (result != CASF_OK) {
        printf("Failed to start CASF system: %s\n", casf_result_to_string(result));
        return 1;
    }
    
    printf("System started in %s mode\n\n", casf_mode_to_string(casf_get_mode(&system)));
    
    // Switch to detumble mode
    result = casf_set_mode(&system, CASF_MODE_DETUMBLE);
    if (result != CASF_OK) {
        printf("Failed to set detumble mode: %s\n", casf_result_to_string(result));
        return 1;
    }
    
    // Run simulation for 60 seconds
    printf("Running detumble simulation for 60 seconds...\n");
    printf("Time(s) | Mode     | Rates (rad/s)           | Mag Field (nT)\n");
    printf("--------|----------|-------------------------|-------------------------\n");
    
    for (int i = 0; i < 600; i++) { // 60 seconds at 10Hz
        // Update the system
        result = casf_update(&system);
        if (result != CASF_OK) {
            printf("System update failed: %s\n", casf_result_to_string(result));
            break;
        }
        
        // Print status every 5 seconds
        if (i % 50 == 0) {
            const casf_system_state_t* state = casf_get_state(&system);
            if (state) {
                printf("%6.1f  | %-8s | %6.3f %6.3f %6.3f | %8.0f %8.0f %8.0f\n",
                       simulation_time_us / 1000000.0f,
                       casf_mode_to_string(casf_get_mode(&system)),
                       CASF_TO_F(state->angular_velocity.x),
                       CASF_TO_F(state->angular_velocity.y), 
                       CASF_TO_F(state->angular_velocity.z),
                       CASF_TO_F(state->magnetic_field_body.x),
                       CASF_TO_F(state->magnetic_field_body.y),
                       CASF_TO_F(state->magnetic_field_body.z));
            }
        }
        
        // Simulate 100ms delay
        example_delay_us(100000);
    }
    
    printf("\nSimulation complete!\n");
    
    // Display final system status
    const casf_system_state_t* final_state = casf_get_state(&system);
    if (final_state) {
        printf("\nFinal System State:\n");
        printf("  Mode: %s\n", casf_mode_to_string(casf_get_mode(&system)));
        printf("  Angular rates: %.4f, %.4f, %.4f rad/s\n",
               CASF_TO_F(final_state->angular_velocity.x),
               CASF_TO_F(final_state->angular_velocity.y),
               CASF_TO_F(final_state->angular_velocity.z));
        printf("  Sensor health: Mag=%d%%, Gyro=%d%%\n",
               system.sensors[0].health_status,
               system.sensors[1].health_status);
        printf("  Control cycles: %u\n", system.control_cycle_count);
        printf("  System uptime: %u seconds\n", system.system_uptime_s);
    }
    
    // Shutdown
    result = casf_shutdown(&system);
    if (result != CASF_OK) {
        printf("Failed to shutdown system: %s\n", casf_result_to_string(result));
    }
    
    printf("\nCASF system shutdown complete.\n");
    return 0;
}