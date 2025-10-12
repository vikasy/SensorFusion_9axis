/*******************************************************************************
 * Demo: How to Use Test Data Generator
 *
 * This file demonstrates how to generate synthetic IMU data for testing
 *
 * Compile and run:
 *   gcc -std=c99 -I. -I../../code/algo/inc test_data_generator.c \
 *       test_data_generator_demo.c -lm -o demo
 *   ./demo
 *
 * Author: Vikas Yadav
 * Date: 2025-10-11
 ******************************************************************************/

#include <stdio.h>
#include "test_data_generator.h"

int main(void) {
    printf("=== Test Data Generator Demo ===\n\n");

    // Example 1: Generate a single static sample
    printf("Example 1: Static IMU at 30° pitch\n");
    test_imu_sample_t static_sample;
    generate_static_sample(0.0, 30.0, 0.0, &static_sample);

    printf("  Accel: [%.3f, %.3f, %.3f] m/s^2\n",
           static_sample.accel[0], static_sample.accel[1], static_sample.accel[2]);
    printf("  Gyro:  [%.3f, %.3f, %.3f] rad/s\n",
           static_sample.gyro[0], static_sample.gyro[1], static_sample.gyro[2]);
    printf("  Mag:   [%.3f, %.3f, %.3f] uT\n\n",
           static_sample.mag[0], static_sample.mag[1], static_sample.mag[2]);

    // Example 2: Generate a sequence of samples
    printf("Example 2: Static sequence (2 seconds @ 100Hz)\n");

    test_data_config_t config = {
        .scenario = SCENARIO_STATIC,
        .duration_sec = 2.0,
        .sample_rate_hz = 100.0,
        .noise_accel = 0.01,      // 0.01 m/s^2 noise
        .noise_gyro = 0.001,      // 0.001 rad/s noise
        .noise_mag = 0.5,         // 0.5 uT noise
        .bias_accel = {0.0, 0.0, 0.0},
        .bias_gyro = {0.01, -0.01, 0.005},  // Small gyro bias
        .initial_orientation = {0.0, 0.0, 0.0}
    };

    test_imu_sample_t samples[200];  // 2 sec * 100 Hz = 200 samples
    uint32_t count = generate_imu_sequence(&config, samples, 200);

    printf("  Generated %u samples\n", count);
    printf("  First sample:\n");
    printf("    Accel: [%.3f, %.3f, %.3f] m/s^2\n",
           samples[0].accel[0], samples[0].accel[1], samples[0].accel[2]);
    printf("  Last sample:\n");
    printf("    Accel: [%.3f, %.3f, %.3f] m/s^2\n\n",
           samples[count-1].accel[0], samples[count-1].accel[1], samples[count-1].accel[2]);

    // Example 3: Rotating scenario
    printf("Example 3: Constant rotation (1 second @ 100Hz)\n");

    config.scenario = SCENARIO_CONSTANT_ROTATION;
    config.duration_sec = 1.0;
    config.noise_accel = 0.0;  // No noise for clarity
    config.noise_gyro = 0.0;
    config.noise_mag = 0.0;

    test_imu_sample_t rotation_samples[100];
    count = generate_imu_sequence(&config, rotation_samples, 100);

    printf("  Generated %u samples\n", count);
    printf("  Sample at t=0:\n");
    printf("    Gyro: [%.4f, %.4f, %.4f] rad/s\n",
           rotation_samples[0].gyro[0], rotation_samples[0].gyro[1], rotation_samples[0].gyro[2]);
    printf("  Sample at t=0.5s:\n");
    printf("    Gyro: [%.4f, %.4f, %.4f] rad/s\n\n",
           rotation_samples[50].gyro[0], rotation_samples[50].gyro[1], rotation_samples[50].gyro[2]);

    // Example 4: Add noise to existing sample
    printf("Example 4: Adding noise to a sample\n");

    test_imu_sample_t clean_sample;
    generate_static_sample(0.0, 0.0, 0.0, &clean_sample);
    printf("  Clean accel: [%.6f, %.6f, %.6f] m/s^2\n",
           clean_sample.accel[0], clean_sample.accel[1], clean_sample.accel[2]);

    test_imu_sample_t noisy_sample = clean_sample;
    add_imu_noise(&noisy_sample, 0.05, 0.01, 1.0, 12345);
    printf("  Noisy accel: [%.6f, %.6f, %.6f] m/s^2\n\n",
           noisy_sample.accel[0], noisy_sample.accel[1], noisy_sample.accel[2]);

    printf("=== Demo Complete ===\n");
    return 0;
}
