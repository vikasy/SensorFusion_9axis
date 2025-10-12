/**
 * @file test_fusion_testdata.c
 * @brief End-to-end validation using testdata/fusion datasets
 *
 * Tests sensor fusion algorithms with converted Excel testdata that includes
 * sensor inputs and expected quaternion/orientation outputs
 *
 * Author: Vikas Yadav / Claude Code
 * Date: 2025-10-12
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "algo_sf_6x_sensor_fusion.h"
#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"
#include "sensor_spec_agm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Include testdata headers
#include "test_input_output.h"

// Only include one dataset at compile time to avoid symbol conflicts
// Use -DTEST_DATASET to specify which dataset to compile
#if defined(TEST_DATASET_0922)
    #include "test_input_output_0922.h"
#elif defined(TEST_DATASET_0923_STANDSTILL)
    #include "test_input_output_0923_standstill.h"
#elif defined(TEST_DATASET_0923_MOVING)
    #include "test_input_output_0923_moving.h"
#elif defined(TEST_DATASET_0930)
    #include "test_input_output_0930.h"
#elif defined(TEST_DATASET_1012)
    #include "test_input_output_1012.h"
#else
    // Default to 0922 if no dataset specified
    #include "test_input_output_0922.h"
    #define TEST_DATASET_0922
#endif

// Tolerance for validation
#define QUAT_DISTANCE_TOLERANCE 0.05  // Quaternion distance tolerance
#define ANGLE_TOLERANCE_DEG 10.0       // Euler angle tolerance (degrees)

typedef struct {
    int total_samples;
    int input_samples;
    int output_samples;
    int valid_comparisons;
    double quat_error_sum;
    double angle_error_sum;
    double max_quat_error;
    double max_angle_error;
    int passed;
    int failed;
} test_statistics_t;

// Normalize angle to [-180, 180)
double normalize_angle(double angle) {
    while (angle >= 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

// Compute quaternion distance: 1 - |q1 · q2|
double quaternion_distance(double q1[4], double q2[4]) {
    double dot = fabs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]);
    if (dot > 1.0) dot = 1.0;
    return 1.0 - dot;
}

// Compute angular error from quaternion distance
double angular_error_from_quats(double q1[4], double q2[4]) {
    double dot = fabs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]);
    if (dot > 1.0) dot = 1.0;
    double angle_rad = 2.0 * acos(dot);
    return angle_rad * 180.0 / M_PI;
}

// Run test with given dataset
void run_testdata_test(const char *test_name,
                       const test_sensor_sample_t *input_data,
                       int input_count,
                       const test_expected_output_t *expected_data,
                       int expected_count,
                       int use_9axis) {

    printf("\n=======================================================================\n");
    printf("TEST: %s (%s)\n", test_name, use_9axis ? "9-axis" : "6-axis");
    printf("=======================================================================\n");
    printf("Input samples: %d\n", input_count);
    printf("Expected outputs: %d\n", expected_count);

    // Initialize algorithm
    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id;
    if (use_9axis) {
        algo_id = sf_9xagm_algo_init(&init_data);
    } else {
        algo_id = sf_6xag_algo_init(&init_data);
    }

    if (algo_id == 0) {
        printf("ERROR: Failed to initialize algorithm\n");
        return;
    }

    printf("✓ Algorithm initialized\n");

    // Statistics
    test_statistics_t stats = {0};
    int expected_idx = 0;

    // Process all input samples
    for (int i = 0; i < input_count; i++) {
        sensor_data_t sensor_data;
        sensor_data.timestamp = input_data[i].ts;
        sensor_data.sensordata[0] = input_data[i].x;
        sensor_data.sensordata[1] = input_data[i].y;
        sensor_data.sensordata[2] = input_data[i].z;

        // Set sensor ID: 0=accel, 1=gyro, 2=mag
        if (input_data[i].id == 0) {
            sensor_data.sensorID = ACC;
        } else if (input_data[i].id == 1) {
            sensor_data.sensorID = GYRO;
        } else if (input_data[i].id == 2) {
            sensor_data.sensorID = MAG;
        } else {
            continue;  // Unknown sensor ID
        }

        // Preprocess sensor data
        if (use_9axis) {
            sf_9xagm_data_preproc(algo_id, &sensor_data);
        } else {
            sf_6xag_data_preproc(algo_id, &sensor_data);
        }

        // Run algorithm
        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));

        if (use_9axis) {
            sf_9xagm_algo_run(algo_id, &output);
        } else {
            sf_6xag_algo_run(algo_id, &output);
        }

        // Check if we have expected output for this timestamp
        if (expected_idx < expected_count &&
            expected_data[expected_idx].ts == input_data[i].ts) {

            // Compare quaternions
            double est_quat[4] = {
                output.quat.q0,
                output.quat.q1,
                output.quat.q2,
                output.quat.q3
            };

            double gt_quat[4] = {
                expected_data[expected_idx].q0,
                expected_data[expected_idx].q1,
                expected_data[expected_idx].q2,
                expected_data[expected_idx].q3
            };

            double quat_dist = quaternion_distance(est_quat, gt_quat);
            double angle_error = angular_error_from_quats(est_quat, gt_quat);

            stats.quat_error_sum += quat_dist;
            stats.angle_error_sum += angle_error;

            if (quat_dist > stats.max_quat_error) {
                stats.max_quat_error = quat_dist;
            }

            if (angle_error > stats.max_angle_error) {
                stats.max_angle_error = angle_error;
            }

            // Check if within tolerance
            if (quat_dist < QUAT_DISTANCE_TOLERANCE && angle_error < ANGLE_TOLERANCE_DEG) {
                stats.passed++;
            } else {
                stats.failed++;
            }

            stats.valid_comparisons++;
            expected_idx++;
        }

        // Progress indicator
        if ((i + 1) % 1000 == 0) {
            printf("  Processed %d/%d samples...\n", i + 1, input_count);
        }
    }

    // Compute final statistics
    double avg_quat_error = (stats.valid_comparisons > 0) ?
        stats.quat_error_sum / stats.valid_comparisons : 0.0;
    double avg_angle_error = (stats.valid_comparisons > 0) ?
        stats.angle_error_sum / stats.valid_comparisons : 0.0;

    // Print results
    printf("\n");
    printf("VALIDATION RESULTS:\n");
    printf("  Valid comparisons: %d\n", stats.valid_comparisons);
    printf("  Passed: %d\n", stats.passed);
    printf("  Failed: %d\n", stats.failed);
    printf("\n");
    printf("QUATERNION ACCURACY:\n");
    printf("  Mean Distance:      %.6f\n", avg_quat_error);
    printf("  Max Distance:       %.6f\n", stats.max_quat_error);
    printf("\n");
    printf("ANGULAR ACCURACY:\n");
    printf("  Mean Error: %.3f°\n", avg_angle_error);
    printf("  Max Error:  %.3f°\n", stats.max_angle_error);
    printf("\n");

    // Overall pass/fail
    double pass_rate = (stats.valid_comparisons > 0) ?
        (100.0 * stats.passed) / stats.valid_comparisons : 0.0;

    if (pass_rate >= 90.0) {
        printf("RESULT: ✓ PASS (%.1f%% within tolerance)\n", pass_rate);
    } else {
        printf("RESULT: ✗ FAIL (%.1f%% within tolerance, expected ≥90%%)\n", pass_rate);
    }
    printf("=======================================================================\n");

    // Cleanup
    if (use_9axis) {
        sf_9xagm_algo_stop(algo_id);
    } else {
        sf_6xag_algo_stop(algo_id);
    }
}

int main(int argc, char *argv[]) {
    printf("\n");
    printf("=======================================================================\n");
    printf("SENSOR FUSION - TESTDATA VALIDATION\n");
    printf("=======================================================================\n");
    printf("Testing with converted Excel testdata (sensor inputs + ground truth)\n");
    printf("\n");

    int use_9axis = 0;  // Default to 6-axis

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--9axis") == 0) {
            use_9axis = 1;
        } else if (strcmp(argv[i], "--6axis") == 0) {
            use_9axis = 0;
        }
    }

    printf("Mode: %s\n", use_9axis ? "9-axis (with magnetometer)" : "6-axis (accel+gyro only)");
    printf("\n");

    // Determine which dataset is compiled in
    const char *dataset_name = "unknown";
    #if defined(TEST_DATASET_0922)
        dataset_name = "test_input_output_0922";
    #elif defined(TEST_DATASET_0923_STANDSTILL)
        dataset_name = "test_input_output_0923_standstill";
    #elif defined(TEST_DATASET_0923_MOVING)
        dataset_name = "test_input_output_0923_moving";
    #elif defined(TEST_DATASET_0930)
        dataset_name = "test_input_output_0930";
    #elif defined(TEST_DATASET_1012)
        dataset_name = "test_input_output_1012";
    #endif

    // Run test with compiled-in dataset
    run_testdata_test(dataset_name,
                     sensor_input_data,
                     SENSOR_INPUT_DATA_COUNT,
                     expected_output_data,
                     EXPECTED_OUTPUT_DATA_COUNT,
                     use_9axis);

    printf("\n");
    printf("=======================================================================\n");
    printf("TEST COMPLETE\n");
    printf("=======================================================================\n");

    return 0;
}
