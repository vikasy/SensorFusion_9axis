/*******************************************************************************
 * Algorithm Validation Tests with Known Ground Truth
 *
 * Author: Vikas Yadav
 * Date: 2025-10-11
 *
 * Purpose: Validate sensor fusion algorithm accuracy using known orientations
 *          and MPU9250 sensor specifications
 *
 * Test Framework: Unity
 ******************************************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "unity.h"
#include "algo_sf_6x_sensor_fusion.h"
#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"
#include "sensor_spec_agm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Test tolerances
#define ANGLE_TOLERANCE_STRICT_DEG    2.0    // For static, well-conditioned cases
#define ANGLE_TOLERANCE_NORMAL_DEG    5.0    // For normal operation
#define ANGLE_TOLERANCE_LOOSE_DEG     10.0   // For noisy/dynamic cases

/*******************************************************************************
 * Unity Framework Setup/Teardown
 ******************************************************************************/

void setUp(void) {}
void tearDown(void) {}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief Convert physical acceleration (m/s²) to MPU9250 raw counts
 */
static void accel_mps2_to_counts(double accel_mps2[3], int16_t counts[3]) {
    for (int i = 0; i < 3; i++) {
        double accel_g = accel_mps2[i] / MPU9250_GRAVITY_MPS2;
        counts[i] = (int16_t)(accel_g * MPU9250_COUNTSPERG);
    }
}

/**
 * @brief Convert angular velocity (rad/s) to MPU9250 raw counts
 */
static void gyro_rads_to_counts(double gyro_rads[3], int16_t counts[3]) {
    for (int i = 0; i < 3; i++) {
        double gyro_dps = gyro_rads[i] * 180.0 / M_PI;
        counts[i] = (int16_t)(gyro_dps * MPU9250_COUNTSPERDPS);
    }
}

/**
 * @brief Convert magnetic field (µT) to MPU9250 raw counts
 */
static void mag_ut_to_counts(double mag_ut[3], int16_t counts[3]) {
    for (int i = 0; i < 3; i++) {
        counts[i] = (int16_t)(mag_ut[i] * AK8963_COUNTSPERUT);
    }
}

/**
 * @brief Calculate expected accelerometer readings for given orientation
 *
 * @param roll_deg Roll angle in degrees
 * @param pitch_deg Pitch angle in degrees
 * @param accel_out Output acceleration vector [x, y, z] in m/s²
 *
 * IMPORTANT: This algorithm uses a non-standard Euler angle convention!
 * Empirical testing (test_coordinate_frame) revealed:
 * - X-axis acceleration controls ROLL (not pitch as in aerospace convention)
 * - Y-axis acceleration controls PITCH (not roll as in aerospace convention)
 * - Z-axis is standard (up = +1g when level)
 *
 * For a sensor at rest:
 * - Level: [0, 0, +1g]
 * - Positive roll (tilted about X): positive X acceleration
 * - Positive pitch (tilted about Y): negative Y acceleration
 */
static void calculate_expected_accel(double roll_deg, double pitch_deg, double accel_out[3]) {
    double roll_rad = roll_deg * M_PI / 180.0;
    double pitch_rad = pitch_deg * M_PI / 180.0;
    double g = MPU9250_GRAVITY_MPS2;

    // Based on empirical results:
    // Roll affects X-axis, Pitch affects Y-axis (swapped from standard!)
    accel_out[0] = g * sin(roll_rad) * cos(pitch_rad);   // X: affected by roll
    accel_out[1] = -g * sin(pitch_rad) * cos(roll_rad);  // Y: affected by pitch (negative)
    accel_out[2] = g * cos(roll_rad) * cos(pitch_rad);   // Z: standard gravity component
}

/**
 * @brief Calculate expected magnetometer readings for given yaw angle
 *
 * @param yaw_deg Yaw angle in degrees (0 = North)
 * @param mag_out Output magnetic field vector [x, y, z] in µT
 *
 * Assumes horizontal magnetic field pointing North with typical strength
 */
static void calculate_expected_mag(double yaw_deg, double mag_out[3]) {
    double yaw_rad = yaw_deg * M_PI / 180.0;

    // Earth's magnetic field (typical mid-latitude, horizontal component)
    double mag_north = AK8963_EARTH_MAG_TYPICAL_UT;

    // Rotate into body frame
    mag_out[0] = mag_north * cos(yaw_rad);
    mag_out[1] = mag_north * sin(yaw_rad);
    mag_out[2] = 0.0;  // Simplified: assume horizontal field
}

/**
 * @brief Run sensor fusion with static data until convergence
 */
static void run_until_convergence_6axis(uintptr_t algo_id,
                                        int16_t accel_counts[3],
                                        int16_t gyro_counts[3],
                                        sf_algo_output_t *output) {
    sensor_data_t accel_data, gyro_data;

    accel_data.sensorID = ACC;
    gyro_data.sensorID = GYRO;

    memcpy(accel_data.sensordata, accel_counts, 3 * sizeof(int16_t));
    memcpy(gyro_data.sensordata, gyro_counts, 3 * sizeof(int16_t));

    // Feed same data 200 times to allow convergence
    for (int i = 0; i < 200; i++) {
        accel_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        gyro_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;

        sf_6xag_data_preproc(algo_id, &accel_data);
        sf_6xag_data_preproc(algo_id, &gyro_data);

        memset(output, 0, sizeof(sf_algo_output_t));
        sf_6xag_algo_run(algo_id, output);
    }
}

static void run_until_convergence_9axis(uintptr_t algo_id,
                                        int16_t accel_counts[3],
                                        int16_t gyro_counts[3],
                                        int16_t mag_counts[3],
                                        sf_algo_output_t *output) {
    sensor_data_t accel_data, gyro_data, mag_data;

    accel_data.sensorID = ACC;
    gyro_data.sensorID = GYRO;
    mag_data.sensorID = MAG;

    memcpy(accel_data.sensordata, accel_counts, 3 * sizeof(int16_t));
    memcpy(gyro_data.sensordata, gyro_counts, 3 * sizeof(int16_t));
    memcpy(mag_data.sensordata, mag_counts, 3 * sizeof(int16_t));

    // Feed same data 200 times to allow convergence
    for (int i = 0; i < 200; i++) {
        accel_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        gyro_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        mag_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;

        sf_9xagm_data_preproc(algo_id, &accel_data);
        sf_9xagm_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        memset(output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, output);
    }
}

/**
 * @brief Normalize angle to [-180, 180] range
 */
static double normalize_angle_180(double angle_deg) {
    while (angle_deg > 180.0) angle_deg -= 360.0;
    while (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}

/**
 * @brief Compare two angles with wraparound handling
 */
static double angle_difference(double angle1_deg, double angle2_deg) {
    double diff = normalize_angle_180(angle1_deg - angle2_deg);
    return fabs(diff);
}

/*******************************************************************************
 * 6-Axis Validation Tests
 ******************************************************************************/

void test_6axis_level_orientation(void) {
    printf("\n  6-Axis Validation: Level orientation (0° roll, 0° pitch)\n");

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_6xag_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Expected orientation: level (0° roll, 0° pitch)
    double expected_roll = 0.0;
    double expected_pitch = 0.0;

    // Calculate expected sensor readings
    double accel_mps2[3];
    calculate_expected_accel(expected_roll, expected_pitch, accel_mps2);

    double gyro_rads[3] = {0.0, 0.0, 0.0};  // At rest

    // Convert to raw counts
    int16_t accel_counts[3], gyro_counts[3];
    accel_mps2_to_counts(accel_mps2, accel_counts);
    gyro_rads_to_counts(gyro_rads, gyro_counts);

    printf("    Expected accel (m/s²): [%.3f, %.3f, %.3f]\n",
           accel_mps2[0], accel_mps2[1], accel_mps2[2]);
    printf("    Accel raw counts: [%d, %d, %d]\n",
           accel_counts[0], accel_counts[1], accel_counts[2]);

    // Run algorithm until convergence
    sf_algo_output_t output;
    run_until_convergence_6axis(algo_id, accel_counts, gyro_counts, &output);

    // Extract results (output.orientation is [yaw, pitch, roll])
    double calculated_yaw = output.orientation[0];
    double calculated_pitch = output.orientation[1];
    double calculated_roll = output.orientation[2];

    printf("    Expected: Roll=%.2f°, Pitch=%.2f°\n", expected_roll, expected_pitch);
    printf("    Calculated: Roll=%.2f°, Pitch=%.2f°\n", calculated_roll, calculated_pitch);

    // Validate results
    double roll_error = angle_difference(calculated_roll, expected_roll);
    double pitch_error = angle_difference(calculated_pitch, expected_pitch);

    printf("    Error: Roll=%.2f°, Pitch=%.2f°\n", roll_error, pitch_error);

    TEST_ASSERT_TRUE(roll_error < ANGLE_TOLERANCE_STRICT_DEG);
    TEST_ASSERT_TRUE(pitch_error < ANGLE_TOLERANCE_STRICT_DEG);

    sf_6xag_algo_stop(algo_id);
}

void test_6axis_30deg_pitch(void) {
    printf("\n  6-Axis Validation: 30° pitch forward\n");

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_6xag_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Expected orientation: 30° pitch forward
    double expected_roll = 0.0;
    double expected_pitch = 30.0;

    // Calculate expected sensor readings
    double accel_mps2[3];
    calculate_expected_accel(expected_roll, expected_pitch, accel_mps2);

    double gyro_rads[3] = {0.0, 0.0, 0.0};

    // Convert to raw counts
    int16_t accel_counts[3], gyro_counts[3];
    accel_mps2_to_counts(accel_mps2, accel_counts);
    gyro_rads_to_counts(gyro_rads, gyro_counts);

    printf("    Expected accel (m/s²): [%.3f, %.3f, %.3f]\n",
           accel_mps2[0], accel_mps2[1], accel_mps2[2]);

    // Run algorithm until convergence
    sf_algo_output_t output;
    run_until_convergence_6axis(algo_id, accel_counts, gyro_counts, &output);

    double calculated_yaw = output.orientation[0];
    double calculated_pitch = output.orientation[1];
    double calculated_roll = output.orientation[2];

    printf("    Expected: Roll=%.2f°, Pitch=%.2f°\n", expected_roll, expected_pitch);
    printf("    Calculated: Roll=%.2f°, Pitch=%.2f°\n", calculated_roll, calculated_pitch);

    double roll_error = angle_difference(calculated_roll, expected_roll);
    double pitch_error = angle_difference(calculated_pitch, expected_pitch);

    printf("    Error: Roll=%.2f°, Pitch=%.2f°\n", roll_error, pitch_error);

    TEST_ASSERT_TRUE(roll_error < ANGLE_TOLERANCE_NORMAL_DEG);
    TEST_ASSERT_TRUE(pitch_error < ANGLE_TOLERANCE_NORMAL_DEG);

    sf_6xag_algo_stop(algo_id);
}

void test_6axis_45deg_roll(void) {
    printf("\n  6-Axis Validation: 45° roll right\n");

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_6xag_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Expected orientation: 45° roll right
    double expected_roll = 45.0;
    double expected_pitch = 0.0;

    // Calculate expected sensor readings
    double accel_mps2[3];
    calculate_expected_accel(expected_roll, expected_pitch, accel_mps2);

    double gyro_rads[3] = {0.0, 0.0, 0.0};

    // Convert to raw counts
    int16_t accel_counts[3], gyro_counts[3];
    accel_mps2_to_counts(accel_mps2, accel_counts);
    gyro_rads_to_counts(gyro_rads, gyro_counts);

    printf("    Expected accel (m/s²): [%.3f, %.3f, %.3f]\n",
           accel_mps2[0], accel_mps2[1], accel_mps2[2]);

    // Run algorithm until convergence
    sf_algo_output_t output;
    run_until_convergence_6axis(algo_id, accel_counts, gyro_counts, &output);

    double calculated_yaw = output.orientation[0];
    double calculated_pitch = output.orientation[1];
    double calculated_roll = output.orientation[2];

    printf("    Expected: Roll=%.2f°, Pitch=%.2f°\n", expected_roll, expected_pitch);
    printf("    Calculated: Roll=%.2f°, Pitch=%.2f°\n", calculated_roll, calculated_pitch);

    double roll_error = angle_difference(calculated_roll, expected_roll);
    double pitch_error = angle_difference(calculated_pitch, expected_pitch);

    printf("    Error: Roll=%.2f°, Pitch=%.2f°\n", roll_error, pitch_error);

    TEST_ASSERT_TRUE(roll_error < ANGLE_TOLERANCE_NORMAL_DEG);
    TEST_ASSERT_TRUE(pitch_error < ANGLE_TOLERANCE_NORMAL_DEG);

    sf_6xag_algo_stop(algo_id);
}

/*******************************************************************************
 * 9-Axis Validation Tests
 ******************************************************************************/

void test_9axis_level_north(void) {
    printf("\n  9-Axis Validation: Level, pointing North (0° roll, 0° pitch, 0° yaw)\n");

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Expected orientation
    double expected_roll = 0.0;
    double expected_pitch = 0.0;
    double expected_yaw = 0.0;  // North

    // Calculate expected sensor readings
    double accel_mps2[3], mag_ut[3];
    calculate_expected_accel(expected_roll, expected_pitch, accel_mps2);
    calculate_expected_mag(expected_yaw, mag_ut);

    double gyro_rads[3] = {0.0, 0.0, 0.0};

    // Convert to raw counts
    int16_t accel_counts[3], gyro_counts[3], mag_counts[3];
    accel_mps2_to_counts(accel_mps2, accel_counts);
    gyro_rads_to_counts(gyro_rads, gyro_counts);
    mag_ut_to_counts(mag_ut, mag_counts);

    printf("    Expected accel (m/s²): [%.3f, %.3f, %.3f]\n",
           accel_mps2[0], accel_mps2[1], accel_mps2[2]);
    printf("    Expected mag (µT): [%.3f, %.3f, %.3f]\n",
           mag_ut[0], mag_ut[1], mag_ut[2]);

    // Run algorithm until convergence
    sf_algo_output_t output;
    run_until_convergence_9axis(algo_id, accel_counts, gyro_counts, mag_counts, &output);

    double calculated_pitch = output.orientation[0];
    double calculated_yaw = output.orientation[1];
    double calculated_roll = output.orientation[2];

    printf("    Expected: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
           expected_roll, expected_pitch, expected_yaw);
    printf("    Calculated: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
           calculated_roll, calculated_pitch, calculated_yaw);

    double roll_error = angle_difference(calculated_roll, expected_roll);
    double pitch_error = angle_difference(calculated_pitch, expected_pitch);
    double yaw_error = angle_difference(calculated_yaw, expected_yaw);

    printf("    Error: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
           roll_error, pitch_error, yaw_error);

    TEST_ASSERT_TRUE(roll_error < ANGLE_TOLERANCE_STRICT_DEG);
    TEST_ASSERT_TRUE(pitch_error < ANGLE_TOLERANCE_STRICT_DEG);
    TEST_ASSERT_TRUE(yaw_error < ANGLE_TOLERANCE_NORMAL_DEG);

    sf_9xagm_algo_stop(algo_id);
}

void test_9axis_level_east(void) {
    printf("\n  9-Axis Validation: Level, pointing East (0° roll, 0° pitch, 90° yaw)\n");

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);
    TEST_ASSERT_NOT_EQUAL(0, algo_id);

    // Expected orientation
    double expected_roll = 0.0;
    double expected_pitch = 0.0;
    double expected_yaw = 90.0;  // East

    // Calculate expected sensor readings
    double accel_mps2[3], mag_ut[3];
    calculate_expected_accel(expected_roll, expected_pitch, accel_mps2);
    calculate_expected_mag(expected_yaw, mag_ut);

    double gyro_rads[3] = {0.0, 0.0, 0.0};

    // Convert to raw counts
    int16_t accel_counts[3], gyro_counts[3], mag_counts[3];
    accel_mps2_to_counts(accel_mps2, accel_counts);
    gyro_rads_to_counts(gyro_rads, gyro_counts);
    mag_ut_to_counts(mag_ut, mag_counts);

    printf("    Expected mag (µT): [%.3f, %.3f, %.3f]\n",
           mag_ut[0], mag_ut[1], mag_ut[2]);

    // Run algorithm until convergence
    sf_algo_output_t output;
    run_until_convergence_9axis(algo_id, accel_counts, gyro_counts, mag_counts, &output);

    double calculated_pitch = output.orientation[0];
    double calculated_yaw = output.orientation[1];
    double calculated_roll = output.orientation[2];

    printf("    Expected: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
           expected_roll, expected_pitch, expected_yaw);
    printf("    Calculated: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
           calculated_roll, calculated_pitch, calculated_yaw);

    double roll_error = angle_difference(calculated_roll, expected_roll);
    double pitch_error = angle_difference(calculated_pitch, expected_pitch);
    double yaw_error = angle_difference(calculated_yaw, expected_yaw);

    printf("    Error: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n",
           roll_error, pitch_error, yaw_error);

    TEST_ASSERT_TRUE(roll_error < ANGLE_TOLERANCE_STRICT_DEG);
    TEST_ASSERT_TRUE(pitch_error < ANGLE_TOLERANCE_STRICT_DEG);
    TEST_ASSERT_TRUE(yaw_error < ANGLE_TOLERANCE_NORMAL_DEG);

    sf_9xagm_algo_stop(algo_id);
}

/*******************************************************************************
 * Main Test Runner
 ******************************************************************************/

int main(void) {
    UNITY_BEGIN();

    printf("\n");
    printf("================================================================================\n");
    printf("  Sensor Fusion Algorithm Validation Tests\n");
    printf("  Sensor: MPU9250 9-Axis IMU\n");
    printf("  Accel Range: ±%.0fg | Gyro Range: ±%.0f dps | Mag Range: ±%.0f µT\n",
           MPU9250_ACC_RANGE_G, MPU9250_GYRO_RANGE_DPS, AK8963_MAG_RANGE_UT);
    printf("  Sample Rate: %d Hz\n", MPU9250_SAMPLE_RATE_HZ);
    printf("================================================================================\n");

    // 6-Axis validation tests
    printf("\n--- 6-Axis Fusion Validation ---\n");
    RUN_TEST(test_6axis_level_orientation);
    RUN_TEST(test_6axis_30deg_pitch);
    RUN_TEST(test_6axis_45deg_roll);

    // 9-Axis validation tests
    printf("\n--- 9-Axis Fusion Validation ---\n");
    RUN_TEST(test_9axis_level_north);
    RUN_TEST(test_9axis_level_east);

    printf("\n");
    return UNITY_END();
}
