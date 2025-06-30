/**
 * CubeSat ADCS Standard Framework (CASF)
 * Detumbling Example Application
 * 
 * This example demonstrates the implementation of a B-dot detumbling controller
 * using the CASF dynamics simulator to validate the algorithm.
 */

#include "../src/control/control.h"
#include "../src/sensors/sensors.h"
#include "../src/dynamics/dynamics.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>

// Time tracking variables
static uint64_t sim_time_us = 0;
static uint64_t last_log_time_us = 0;

// HAL implementation for simulation
static uint64_t sim_get_time_us(void) {
    return sim_time_us;
}

static void sim_log(int level, const char* fmt, ...) {
    // Format log timestamp
    char timestamp[32];
    float time_s = (float)sim_time_us / 1000000.0f;
    sprintf(timestamp, "[%7.3f s]", time_s);

    // Print log message with timestamp
    va_list args;
    va_start(args, fmt);
    printf("%s ", timestamp);
    vprintf(fmt, args);
    printf("\n");
    va_end(args);
}

// Data logging structure
typedef struct {
    FILE* file;
    uint32_t sample_count;
} log_data_t;

// Create simulated magnetic field for detumbling
void create_simulated_magnetic_field(casf_float_t time_s, casf_vector3_t* field) {
    // Inclined dipole model (simplified Earth magnetic field)
    const float period = 6000.0f; // Orbital period in seconds (100 min)
    const float orbit_pos = CASF_TO_F(time_s) * (2.0f * M_PI / period);

    // Dipole tilt and orbit inclination effects
    field->x = CASF_F(cos(orbit_pos) * 0.6f);
    field->y = CASF_F(sin(orbit_pos) * 0.6f);
    field->z = CASF_F(0.8f); // Primarily along z-axis

    // Normalize and scale to typical LEO field strength (~30-50 μT)
    float magnitude = sqrtf(CASF_TO_F(field->x)*CASF_TO_F(field->x) + 
                           CASF_TO_F(field->y)*CASF_TO_F(field->y) + 
                           CASF_TO_F(field->z)*CASF_TO_F(field->z));

    field->x = CASF_F(CASF_TO_F(field->x) / magnitude * 4e-5f);
    field->y = CASF_F(CASF_TO_F(field->y) / magnitude * 4e-5f);
    field->z = CASF_F(CASF_TO_F(field->z) / magnitude * 4e-5f);
}

// B-dot detumbling control algorithm callback
void bdot_control_callback(const casf_dynamics_state_t* state, casf_vector3_t* control_torque, void* user_data) {
    // Extract B-dot gain from user data
    casf_float_t* gain_ptr = (casf_float_t*)user_data;
    casf_float_t gain = *gain_ptr;

    // Get current and previous magnetic field measurements
    static casf_vector3_t prev_b_field = {CASF_F(0.0f), CASF_F(0.0f), CASF_F(0.0f)};
    casf_vector3_t current_b_field;
    casf_dynamics_get_magnetic_field_body((casf_dynamics_simulator_t*)state, &current_b_field);

    // Calculate B-dot (time derivative of magnetic field)
    casf_vector3_t b_dot;
    casf_float_t dt = CASF_F(0.01f); // 10ms timestep
    b_dot.x = (current_b_field.x - prev_b_field.x) / dt;
    b_dot.y = (current_b_field.y - prev_b_field.y) / dt;
    b_dot.z = (current_b_field.z - prev_b_field.z) / dt;

    // Store current B field for next iteration
    prev_b_field = current_b_field;

    // Compute control torque: T = -K * B × Ḃ (cross product)
    // For B-dot control, directly: T = -K * Ḃ
    control_torque->x = -gain * b_dot.x;
    control_torque->y = -gain * b_dot.y;
    control_torque->z = -gain * b_dot.z;

    // Limit maximum control torque (typical magnetorquer limit for a 3U CubeSat)
    casf_float_t max_torque = CASF_F(2e-4f); // 0.2 mN-m
    casf_float_t torque_magnitude = sqrtf(CASF_TO_F(control_torque->x)*CASF_TO_F(control_torque->x) + 
                                         CASF_TO_F(control_torque->y)*CASF_TO_F(control_torque->y) + 
                                         CASF_TO_F(control_torque->z)*CASF_TO_F(control_torque->z));

    if (torque_magnitude > CASF_TO_F(max_torque)) {
        casf_float_t scale = max_torque / CASF_F(torque_magnitude);
        control_torque->x *= scale;
        control_torque->y *= scale;
        control_torque->z *= scale;
    }
}

// Data logging callback for simulation
void data_logging_callback(const casf_dynamics_state_t* state, void* user_data) {
    log_data_t* log_data = (log_data_t*)user_data;
    sim_time_us = state->timestamp_us;

    // Log data at 1Hz
    if (sim_time_us - last_log_time_us >= 1000000) {
        float time_s = (float)sim_time_us / 1000000.0f;
        float wx = CASF_TO_F(state->angular_velocity.x);
        float wy = CASF_TO_F(state->angular_velocity.y);
        float wz = CASF_TO_F(state->angular_velocity.z);
        float omega_magnitude = sqrtf(wx*wx + wy*wy + wz*wz);

        // Print to console
        printf("[%6.2f s] Angular velocity: [%8.5f %8.5f %8.5f] rad/s (|ω| = %8.5f rad/s)\n", 
               time_s, wx, wy, wz, omega_magnitude);

        // Write to log file
        if (log_data->file) {
            // Get magnetic field
            casf_vector3_t b_body;
            casf_dynamics_get_magnetic_field_body((casf_dynamics_simulator_t*)state, &b_body);

            // Log time, angular velocity, and magnetic field
            fprintf(log_data->file, "%f,%f,%f,%f,%f,%f,%f,%f\n",
                    time_s, 
                    wx, wy, wz, omega_magnitude,
                    CASF_TO_F(b_body.x), CASF_TO_F(b_body.y), CASF_TO_F(b_body.z));

            fflush(log_data->file);
        }

        log_data->sample_count++;
        last_log_time_us = sim_time_us;
    }

    // Update magnetic field based on orbital position
    casf_vector3_t magnetic_field;
    create_simulated_magnetic_field(CASF_F((float)sim_time_us / 1000000.0f), &magnetic_field);
    casf_dynamics_set_magnetic_field((casf_dynamics_simulator_t*)state, &magnetic_field);
}

int main(int argc, char* argv[]) {
    printf("CASF Detumbling Example\n");
    printf("======================\n\n");

    // Initialize random number generator
    srand((unsigned int)time(NULL));

    // Create log file
    log_data_t log_data = {0};
    log_data.file = fopen("detumbling_data.csv", "w");
    if (log_data.file) {
        fprintf(log_data.file, "time,wx,wy,wz,omega_mag,bx,by,bz\n");
    } else {
        printf("Warning: Could not create log file\n");
    }

    // Configure dynamics simulator
    casf_dynamics_config_t config = {0};
    config.timestep = CASF_F(0.01f);  // 10ms simulation step
    config.max_iterations = 100000;    // Maximum iterations
    config.enable_gravity_gradient = 1;
    config.enable_magnetic_torque = 1;
    config.enable_disturbances = 1;

    // Set spacecraft properties (typical 3U CubeSat)
    config.mass = CASF_F(4.0f);  // 4 kg

    // Inertia tensor (kg*m^2) - Diagonal for simplicity
    config.inertia_tensor[0].x = CASF_F(0.0333f);
    config.inertia_tensor[0].y = CASF_F(0.0f);
    config.inertia_tensor[0].z = CASF_F(0.0f);

    config.inertia_tensor[1].x = CASF_F(0.0f);
    config.inertia_tensor[1].y = CASF_F(0.0333f);
    config.inertia_tensor[1].z = CASF_F(0.0f);

    config.inertia_tensor[2].x = CASF_F(0.0f);
    config.inertia_tensor[2].y = CASF_F(0.0f);
    config.inertia_tensor[2].z = CASF_F(0.0067f);

    // Initialize dynamics simulator
    casf_dynamics_simulator_t simulator;
    casf_result_t result = casf_dynamics_init(&simulator, &config);
    if (result != CASF_OK) {
        printf("Failed to initialize dynamics simulator: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Set initial tumbling state (severe post-deployment tumbling)
    casf_vector3_t initial_angular_velocity = {
        CASF_F(0.15f),    // 0.15 rad/s around x (~8.6 deg/s)
        CASF_F(-0.1f),    // -0.1 rad/s around y (~-5.7 deg/s)
        CASF_F(0.2f)      // 0.2 rad/s around z (~11.5 deg/s)
    };

    result = casf_dynamics_reset(&simulator, NULL, &initial_angular_velocity);
    if (result != CASF_OK) {
        printf("Failed to reset dynamics simulator: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Set initial magnetic field
    casf_vector3_t initial_magnetic_field;
    create_simulated_magnetic_field(CASF_F(0.0f), &initial_magnetic_field);
    casf_dynamics_set_magnetic_field(&simulator, &initial_magnetic_field);

    // Configure B-dot controller gain
    casf_float_t bdot_gain = CASF_F(1.0e3f);
    result = casf_dynamics_register_control(&simulator, bdot_control_callback, &bdot_gain);
    if (result != CASF_OK) {
        printf("Failed to register control callback: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Run simulation for 300 seconds
    printf("Running detumbling simulation for 300 seconds...\n\n");
    result = casf_dynamics_run(&simulator, CASF_F(300.0f), data_logging_callback, &log_data);
    if (result != CASF_OK) {
        printf("Simulation failed: %s\n", casf_result_to_string(result));
        return 1;
    }

    // Print final state
    printf("\nFinal state after %d seconds:\n", (int)(simulator.simulation_time));
    printf("Angular velocity: [%8.5f %8.5f %8.5f] rad/s\n",
           CASF_TO_F(simulator.state.angular_velocity.x),
           CASF_TO_F(simulator.state.angular_velocity.y),
           CASF_TO_F(simulator.state.angular_velocity.z));

    float final_rate_magnitude = sqrtf(
        CASF_TO_F(simulator.state.angular_velocity.x) * CASF_TO_F(simulator.state.angular_velocity.x) +
        CASF_TO_F(simulator.state.angular_velocity.y) * CASF_TO_F(simulator.state.angular_velocity.y) +
        CASF_TO_F(simulator.state.angular_velocity.z) * CASF_TO_F(simulator.state.angular_velocity.z));

    printf("Angular velocity magnitude: %8.5f rad/s (%8.5f deg/s)\n",
           final_rate_magnitude, final_rate_magnitude * 180.0f / M_PI);

    // Cleanup
    if (log_data.file) {
        fclose(log_data.file);
        printf("\nLog data written to detumbling_data.csv\n");
    }

    printf("\nSimulation complete!\n");
    return 0;
}
