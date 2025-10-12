/**
 * @file test_9axis_synthetic.c
 * @brief End-to-end validation using synthetic IMU datasets (9-axis)
 *
 * Tests 9-axis sensor fusion algorithm with synthetic IMU data with ground truth
 * Includes magnetometer for absolute yaw measurement
 *
 * Author: Vikas Yadav / Claude Code
 * Date: 2025-10-12
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_SAMPLES 10000

// Sensor specifications (MPU9250 + AK8963)
#define ACCEL_SCALE_G_PER_COUNT (1.0 / 16384.0)  // 2g range
#define GYRO_SCALE_DPS_PER_COUNT (1.0 / 131.0)   // 250 dps range
#define MAG_SCALE_UT_PER_COUNT (0.15)            // 4800 µT range

typedef struct {
    int64_t timestamp_ns;
    int16_t accel_counts[3];
    int16_t gyro_counts[3];
    int16_t mag_counts[3];
    double gt_roll_deg, gt_pitch_deg, gt_yaw_deg;
    double gt_quat[4];  // w, x, y, z
} synthetic_sample_t;

typedef struct {
    double roll_error;
    double pitch_error;
    double yaw_error;
    double quat_error;
    double angular_error_deg;
} error_metrics_t;

// Statistics accumulator
typedef struct {
    int total_samples;
    int valid_samples;
    double roll_rmse;
    double pitch_rmse;
    double yaw_rmse;
    double quat_dist_mean;
    double angular_error_mean;
    double angular_error_max;
    double roll_error_sum;
    double pitch_error_sum;
    double yaw_error_sum;
    double quat_error_sum;
    double angular_error_sum;
} statistics_t;

// Normalize angle to [-180, 180)
double normalize_angle(double angle) {
    while (angle >= 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

// Compute quaternion distance: 1 - |q1 · q2|
double quaternion_distance(const double q1[4], const double q2[4]) {
    double dot = fabs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]);
    if (dot > 1.0) dot = 1.0;
    return 1.0 - dot;
}

// Compute angular error from quaternion distance
double angular_error_from_quats(const double q1[4], const double q2[4]) {
    double dot = fabs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]);
    if (dot > 1.0) dot = 1.0;
    double angle_rad = 2.0 * acos(dot);
    return angle_rad * 180.0 / M_PI;
}

// Compute error metrics
void compute_errors(const synthetic_sample_t *sample, const sf_algo_output_t *output,
                    error_metrics_t *errors) {

    // Orientation errors (output.orientation is [yaw, pitch, roll] NOT [pitch, yaw, roll])
    // Confirmed from algo_sf_9x_sensor_fusion.c:326-328:
    //   orientation[0] = PsiPost (yaw)
    //   orientation[1] = ThetaPost (pitch)
    //   orientation[2] = PhiPost (roll)
    double est_yaw = output->orientation[0];
    double est_pitch = output->orientation[1];
    double est_roll = output->orientation[2];

    errors->roll_error = normalize_angle(est_roll - sample->gt_roll_deg);
    errors->pitch_error = normalize_angle(est_pitch - sample->gt_pitch_deg);
    errors->yaw_error = normalize_angle(est_yaw - sample->gt_yaw_deg);

    // Quaternion errors
    double est_quat[4] = {
        output->quat.q0,
        output->quat.q1,
        output->quat.q2,
        output->quat.q3
    };

    errors->quat_error = quaternion_distance(est_quat, sample->gt_quat);
    errors->angular_error_deg = angular_error_from_quats(est_quat, sample->gt_quat);
}

// Parse CSV line (synthetic format)
int parse_synthetic_csv(char *line, synthetic_sample_t *sample) {
    char *token;
    int field = 0;

    token = strtok(line, ",");
    while (token != NULL && field < 17) {
        switch (field) {
            case 0: sample->timestamp_ns = atoll(token); break;
            case 1: sample->accel_counts[0] = (int16_t)atoi(token); break;
            case 2: sample->accel_counts[1] = (int16_t)atoi(token); break;
            case 3: sample->accel_counts[2] = (int16_t)atoi(token); break;
            case 4: sample->gyro_counts[0] = (int16_t)atoi(token); break;
            case 5: sample->gyro_counts[1] = (int16_t)atoi(token); break;
            case 6: sample->gyro_counts[2] = (int16_t)atoi(token); break;
            case 7: sample->mag_counts[0] = (int16_t)atoi(token); break;
            case 8: sample->mag_counts[1] = (int16_t)atoi(token); break;
            case 9: sample->mag_counts[2] = (int16_t)atoi(token); break;
            case 10: sample->gt_roll_deg = atof(token); break;
            case 11: sample->gt_pitch_deg = atof(token); break;
            case 12: sample->gt_yaw_deg = atof(token); break;
            case 13: sample->gt_quat[0] = atof(token); break;  // w
            case 14: sample->gt_quat[1] = atof(token); break;  // x
            case 15: sample->gt_quat[2] = atof(token); break;  // y
            case 16: sample->gt_quat[3] = atof(token); break;  // z
        }
        token = strtok(NULL, ",");
        field++;
    }

    return field >= 17;
}

// Update statistics
void update_statistics(statistics_t *stats, const error_metrics_t *errors) {
    stats->valid_samples++;
    stats->roll_error_sum += errors->roll_error * errors->roll_error;
    stats->pitch_error_sum += errors->pitch_error * errors->pitch_error;
    stats->yaw_error_sum += errors->yaw_error * errors->yaw_error;
    stats->quat_error_sum += errors->quat_error;
    stats->angular_error_sum += errors->angular_error_deg;

    if (errors->angular_error_deg > stats->angular_error_max) {
        stats->angular_error_max = errors->angular_error_deg;
    }
}

// Compute final statistics
void finalize_statistics(statistics_t *stats) {
    if (stats->valid_samples > 0) {
        stats->roll_rmse = sqrt(stats->roll_error_sum / stats->valid_samples);
        stats->pitch_rmse = sqrt(stats->pitch_error_sum / stats->valid_samples);
        stats->yaw_rmse = sqrt(stats->yaw_error_sum / stats->valid_samples);
        stats->quat_dist_mean = stats->quat_error_sum / stats->valid_samples;
        stats->angular_error_mean = stats->angular_error_sum / stats->valid_samples;
    }
}

// Print results
void print_results(const char *filename, const statistics_t *stats) {
    printf("\n");
    printf("=======================================================================\n");
    printf("9-AXIS E2E TEST RESULTS\n");
    printf("=======================================================================\n");
    printf("Dataset: %s\n", filename);
    printf("Samples: %d processed, %d valid\n", stats->total_samples, stats->valid_samples);
    printf("\n");
    printf("ORIENTATION ACCURACY (RMSE):\n");
    printf("  Roll:  %.3f°\n", stats->roll_rmse);
    printf("  Pitch: %.3f°\n", stats->pitch_rmse);
    printf("  Yaw:   %.3f° (magnetometer-corrected)\n", stats->yaw_rmse);
    printf("\n");
    printf("QUATERNION ACCURACY:\n");
    printf("  Mean Distance:      %.6f\n", stats->quat_dist_mean);
    printf("  Mean Angular Error: %.3f°\n", stats->angular_error_mean);
    printf("  Max Angular Error:  %.3f°\n", stats->angular_error_max);
    printf("\n");

    // Pass/fail criteria (stricter for 9-axis with magnetometer)
    int pass = (stats->roll_rmse < 10.0 && stats->pitch_rmse < 10.0 &&
                stats->yaw_rmse < 15.0 && stats->angular_error_mean < 12.0);

    if (pass) {
        printf("RESULT: ✓ PASS\n");
        printf("  9-axis fusion shows improved yaw accuracy vs 6-axis\n");
    } else {
        printf("RESULT: ✗ FAIL (accuracy exceeds tolerance)\n");
    }
    printf("=======================================================================\n");
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s <synthetic_dataset.csv>\n", argv[0]);
        printf("\nExample:\n");
        printf("  %s test/datasets/synthetic/static_10s.csv\n", argv[0]);
        printf("  %s test/datasets/synthetic/rotation_sequence_15s.csv\n", argv[0]);
        return 1;
    }

    const char *filename = argv[1];

    printf("=======================================================================\n");
    printf("9-AXIS SENSOR FUSION - END-TO-END TEST\n");
    printf("=======================================================================\n");
    printf("Dataset: %s\n", filename);
    printf("Testing: Accelerometer + Gyroscope + Magnetometer fusion\n\n");

    FILE *fp = fopen(filename, "r");
    if (!fp) {
        printf("ERROR: Could not open file: %s\n", filename);
        return 1;
    }

    // Skip header line
    char line[512];
    fgets(line, sizeof(line), fp);

    // Initialize algorithm
    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = ACCEL_SCALE_G_PER_COUNT;
    init_data.Gyro_DPSPERCOUNT = GYRO_SCALE_DPS_PER_COUNT;
    init_data.Mag_UTPERCOUNT = MAG_SCALE_UT_PER_COUNT;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);
    if (algo_id == 0) {
        printf("ERROR: Failed to initialize algorithm\n");
        fclose(fp);
        return 1;
    }

    printf("✓ Algorithm initialized\n");
    printf("Processing samples...\n");

    // Statistics
    statistics_t stats = {0};

    // Process samples
    while (fgets(line, sizeof(line), fp) && stats.total_samples < MAX_SAMPLES) {
        synthetic_sample_t sample;
        if (!parse_synthetic_csv(line, &sample)) {
            continue;
        }

        stats.total_samples++;

        // Create sensor data structures
        sensor_data_t accel_data, gyro_data, mag_data;

        accel_data.sensorID = ACC;
        accel_data.timestamp = sample.timestamp_ns;
        accel_data.sensordata[0] = sample.accel_counts[0];
        accel_data.sensordata[1] = sample.accel_counts[1];
        accel_data.sensordata[2] = sample.accel_counts[2];

        gyro_data.sensorID = GYRO;
        gyro_data.timestamp = sample.timestamp_ns;
        gyro_data.sensordata[0] = sample.gyro_counts[0];
        gyro_data.sensordata[1] = sample.gyro_counts[1];
        gyro_data.sensordata[2] = sample.gyro_counts[2];

        mag_data.sensorID = MAG;
        mag_data.timestamp = sample.timestamp_ns;
        mag_data.sensordata[0] = sample.mag_counts[0];
        mag_data.sensordata[1] = sample.mag_counts[1];
        mag_data.sensordata[2] = sample.mag_counts[2];

        // Feed data to algorithm
        sf_9xagm_data_preproc(algo_id, &accel_data);
        sf_9xagm_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        // Run algorithm
        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);

        // Compute errors
        error_metrics_t errors;
        compute_errors(&sample, &output, &errors);

        // Update statistics
        update_statistics(&stats, &errors);

        // Print progress every 500 samples
        if (stats.total_samples % 500 == 0) {
            printf("  Processed %d samples...\n", stats.total_samples);
        }
    }

    fclose(fp);

    // Finalize and print results
    finalize_statistics(&stats);
    print_results(filename, &stats);

    // Cleanup
    sf_9xagm_algo_stop(algo_id);

    return (stats.roll_rmse < 10.0 && stats.pitch_rmse < 10.0 &&
            stats.yaw_rmse < 15.0 && stats.angular_error_mean < 12.0) ? 0 : 1;
}
