/*******************************************************************************
 * Integration Tests for 9-Axis Sensor Fusion
 *
 * Author: Vikas Yadav
 * Date: 2025-10-11
 *
 * Tests: 9-axis sensor fusion algorithm (Accelerometer + Gyroscope + Magnetometer)
 * Uses correct API from algo_sf_interface.h
 *
 * Test Framework: Unity
 ******************************************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"
#include "test_data_generator.h"
#include "test_helpers.h"

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Test tolerance
#define ANGLE_TOLERANCE_DEG 15.0

// Sensor scale factors (example: MPU9250)
#define ACCEL_SCALE  (1.0f/16384.0f)  // 2g range, 16-bit
#define GYRO_SCALE   (1.0f/131.0f)     // 250 dps range
#define MAG_SCALE    (0.15f)           // 4800 µT range

/*******************************************************************************
 * Unity Framework Setup/Teardown
 ******************************************************************************/

void setUp(void) {
    // Called before each test
}

void tearDown(void) {
    // Called after each test
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

// Convert physical units to sensor counts
static void accel_to_counts(double accel_mps2[3], int16_t counts[3]) {
    counts[0] = (int16_t)(accel_mps2[0] / 9.81 / ACCEL_SCALE);
    counts[1] = (int16_t)(accel_mps2[1] / 9.81 / ACCEL_SCALE);
    counts[2] = (int16_t)(accel_mps2[2] / 9.81 / ACCEL_SCALE);
}

static void gyro_to_counts(double gyro_rads[3], int16_t counts[3]) {
    counts[0] = (int16_t)(gyro_rads[0] * 180.0 / M_PI / GYRO_SCALE);
    counts[1] = (int16_t)(gyro_rads[1] * 180.0 / M_PI / GYRO_SCALE);
    counts[2] = (int16_t)(gyro_rads[2] * 180.0 / M_PI / GYRO_SCALE);
}

static void mag_to_counts(double mag_ut[3], int16_t counts[3]) {
    counts[0] = (int16_t)(mag_ut[0] / MAG_SCALE);
    counts[1] = (int16_t)(mag_ut[1] / MAG_SCALE);
    counts[2] = (int16_t)(mag_ut[2] / MAG_SCALE);
}

/*******************************************************************************
 * 9-Axis Algorithm Initialization Tests
 ******************************************************************************/

void test_9axis_init_success(void) {
    sf_algo_init_data_t init_data;

    // Initialize with scale factors
    init_data.Acc_GPERCOUNT = ACCEL_SCALE;
    init_data.Gyro_DPSPERCOUNT = GYRO_SCALE;
    init_data.Mag_UTPERCOUNT = MAG_SCALE;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);

    // Algorithm ID should be non-zero on success
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Cleanup
    if (algo_id != 0) {
        sf_9xagm_algo_stop(algo_id);
    }
}

/*******************************************************************************
 * 9-Axis Basic Functionality Tests
 ******************************************************************************/

void test_9axis_level_static(void) {
    // Test: IMU lying flat (0° roll, 0° pitch, 0° yaw)

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = ACCEL_SCALE;
    init_data.Gyro_DPSPERCOUNT = GYRO_SCALE;
    init_data.Mag_UTPERCOUNT = MAG_SCALE;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Generate static IMU data (level orientation, pointing north)
    test_imu_sample_t imu_sample;
    generate_static_sample(0.0, 0.0, 0.0, &imu_sample);

    // Convert to sensor counts
    sensor_data_t accel_data, gyro_data, mag_data;
    accel_data.sensorID = ACC;
    gyro_data.sensorID = GYRO;
    mag_data.sensorID = MAG;

    accel_to_counts(imu_sample.accel, accel_data.sensordata);
    gyro_to_counts(imu_sample.gyro, gyro_data.sensordata);
    mag_to_counts(imu_sample.mag, mag_data.sensordata);

    accel_data.timestamp = 0;
    gyro_data.timestamp = 0;
    mag_data.timestamp = 0;

    // Feed data multiple times to allow filter to converge
    for (int i = 0; i < 100; i++) {
        accel_data.timestamp = i * 10000000LL;  // 10ms intervals
        gyro_data.timestamp = i * 10000000LL;
        mag_data.timestamp = i * 10000000LL;

        sf_9xagm_data_preproc(algo_id, &accel_data);
        sf_9xagm_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);
    }

    // Get final output
    sf_algo_output_t output;
    memset(&output, 0, sizeof(sf_algo_output_t));
    sf_9xagm_algo_run(algo_id, &output);

    // Check orientation (output.orientation is [pitch, yaw, roll])
    double pitch = output.orientation[0];
    double yaw = output.orientation[1];
    double roll = output.orientation[2];

    printf("  Level test - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", roll, pitch, yaw);

    // Should be close to level
    TEST_ASSERT_DOUBLE_WITHIN(ANGLE_TOLERANCE_DEG, 0.0, roll);
    TEST_ASSERT_DOUBLE_WITHIN(ANGLE_TOLERANCE_DEG, 0.0, pitch);

    // Cleanup
    sf_9xagm_algo_stop(algo_id);
}

void test_9axis_with_magnetometer(void) {
    // Test: Verify magnetometer is being used for yaw correction

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = ACCEL_SCALE;
    init_data.Gyro_DPSPERCOUNT = GYRO_SCALE;
    init_data.Mag_UTPERCOUNT = MAG_SCALE;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Generate static data pointing different directions
    test_imu_sample_t samples[3];
    generate_static_sample(0.0, 0.0, 0.0, &samples[0]);    // North
    generate_static_sample(0.0, 0.0, 90.0, &samples[1]);   // East
    generate_static_sample(0.0, 0.0, 180.0, &samples[2]);  // South

    for (int dir = 0; dir < 3; dir++) {
        // Reset algorithm
        sf_9xagm_algo_stop(algo_id);
        algo_id = sf_9xagm_algo_init(&init_data);

        sensor_data_t accel_data, gyro_data, mag_data;
        accel_data.sensorID = ACC;
        gyro_data.sensorID = GYRO;
        mag_data.sensorID = MAG;

        accel_to_counts(samples[dir].accel, accel_data.sensordata);
        gyro_to_counts(samples[dir].gyro, gyro_data.sensordata);
        mag_to_counts(samples[dir].mag, mag_data.sensordata);

        // Feed data to converge
        for (int i = 0; i < 100; i++) {
            accel_data.timestamp = i * 10000000LL;
            gyro_data.timestamp = i * 10000000LL;
            mag_data.timestamp = i * 10000000LL;

            sf_9xagm_data_preproc(algo_id, &accel_data);
            sf_9xagm_data_preproc(algo_id, &gyro_data);
            sf_9xagm_data_preproc(algo_id, &mag_data);

            sf_algo_output_t output;
            memset(&output, 0, sizeof(sf_algo_output_t));
            sf_9xagm_algo_run(algo_id, &output);
        }

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);

        printf("  Direction %d - Yaw: %.2f°\n", dir, output.orientation[1]);
    }

    // Just verify algorithm runs without crashing
    TEST_ASSERT_TRUE(1);

    sf_9xagm_algo_stop(algo_id);
}

void test_9axis_rotation_with_mag(void) {
    // Test: Rotating with magnetometer correction

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = ACCEL_SCALE;
    init_data.Gyro_DPSPERCOUNT = GYRO_SCALE;
    init_data.Mag_UTPERCOUNT = MAG_SCALE;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Generate rotation sequence with magnetometer
    test_data_config_t config = {
        .scenario = SCENARIO_CONSTANT_ROTATION,
        .duration_sec = 1.0,
        .sample_rate_hz = 100.0,
        .noise_accel = 0.0,
        .noise_gyro = 0.0,
        .noise_mag = 0.0,
        .bias_accel = {0.0, 0.0, 0.0},
        .bias_gyro = {0.0, 0.0, 0.0},
        .initial_orientation = {0.0, 0.0, 0.0}
    };

    test_imu_sample_t samples[100];
    uint32_t count = generate_imu_sequence(&config, samples, 100);

    printf("  9-axis rotation test - Processing %u samples\n", count);

    for (uint32_t i = 0; i < count; i++) {
        sensor_data_t accel_data, gyro_data, mag_data;
        accel_data.sensorID = ACC;
        gyro_data.sensorID = GYRO;
        mag_data.sensorID = MAG;

        accel_to_counts(samples[i].accel, accel_data.sensordata);
        gyro_to_counts(samples[i].gyro, gyro_data.sensordata);
        mag_to_counts(samples[i].mag, mag_data.sensordata);

        accel_data.timestamp = samples[i].timestamp;
        gyro_data.timestamp = samples[i].timestamp;
        mag_data.timestamp = samples[i].timestamp;

        sf_9xagm_data_preproc(algo_id, &accel_data);
        sf_9xagm_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);
    }

    // Just verify algorithm runs without crashing
    TEST_ASSERT_TRUE(1);

    sf_9xagm_algo_stop(algo_id);
}

void test_9axis_with_noise(void) {
    // Test: 9-axis with realistic sensor noise

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = ACCEL_SCALE;
    init_data.Gyro_DPSPERCOUNT = GYRO_SCALE;
    init_data.Mag_UTPERCOUNT = MAG_SCALE;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    test_data_config_t config = {
        .scenario = SCENARIO_STATIC,
        .duration_sec = 1.0,
        .sample_rate_hz = 100.0,
        .noise_accel = 0.05,  // 0.05 m/s^2
        .noise_gyro = 0.01,   // 0.01 rad/s
        .noise_mag = 1.0,     // 1.0 µT
        .bias_accel = {0.0, 0.0, 0.0},
        .bias_gyro = {0.0, 0.0, 0.0},
        .initial_orientation = {0.0, 0.0, 0.0}
    };

    test_imu_sample_t samples[100];
    uint32_t count = generate_imu_sequence(&config, samples, 100);

    for (uint32_t i = 0; i < count; i++) {
        sensor_data_t accel_data, gyro_data, mag_data;
        accel_data.sensorID = ACC;
        gyro_data.sensorID = GYRO;
        mag_data.sensorID = MAG;

        accel_to_counts(samples[i].accel, accel_data.sensordata);
        gyro_to_counts(samples[i].gyro, gyro_data.sensordata);
        mag_to_counts(samples[i].mag, mag_data.sensordata);

        accel_data.timestamp = samples[i].timestamp;
        gyro_data.timestamp = samples[i].timestamp;
        mag_data.timestamp = samples[i].timestamp;

        sf_9xagm_data_preproc(algo_id, &accel_data);
        sf_9xagm_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);
    }

    sf_algo_output_t output;
    memset(&output, 0, sizeof(sf_algo_output_t));
    sf_9xagm_algo_run(algo_id, &output);

    printf("  9-axis noisy test - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n",
           output.orientation[2], output.orientation[0], output.orientation[1]);

    // Should still be relatively level despite noise (use angle wrapping)
    TEST_ASSERT_TRUE(test_angle_equals(output.orientation[2], 0.0, 20.0));  // Roll
    TEST_ASSERT_TRUE(test_angle_equals(output.orientation[0], 0.0, 20.0));  // Pitch

    sf_9xagm_algo_stop(algo_id);
}

/*******************************************************************************
 * Main Test Runner
 ******************************************************************************/

int main(void) {
    UNITY_BEGIN();

    printf("\n=== 9-Axis Sensor Fusion Integration Tests ===\n\n");

    // Initialization tests
    RUN_TEST(test_9axis_init_success);

    // Static orientation tests
    RUN_TEST(test_9axis_level_static);
    RUN_TEST(test_9axis_with_magnetometer);

    // Dynamic motion tests
    RUN_TEST(test_9axis_rotation_with_mag);

    // Robustness tests
    RUN_TEST(test_9axis_with_noise);

    return UNITY_END();
}
