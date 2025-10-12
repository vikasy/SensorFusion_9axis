/**
 * @file test_9axis_e2e.c
 * @brief End-to-end validation using RepoIMU real-world dataset
 *
 * Tests 9-axis sensor fusion algorithm with actual IMU data and ground truth quaternions
 * from the RepoIMU dataset (Vicon optical motion capture reference)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"
#include "sensor_spec_agm.h"

#define MAX_SAMPLES 20000
#define TOLERANCE_DEGREES 10.0  // Relaxed tolerance for real-world data

typedef struct {
    double timestamp;
    double quat_w, quat_x, quat_y, quat_z;  // Ground truth from Vicon
    double acc_x, acc_y, acc_z;              // Accelerometer m/s²
    double gyro_x, gyro_y, gyro_z;           // Gyroscope rad/s
    double mag_x, mag_y, mag_z;              // Magnetometer µT
} imu_sample_t;

// Convert quaternion to Euler angles (ZYX convention)
void quaternion_to_euler(double qw, double qx, double qy, double qz,
                         double *roll, double *pitch, double *yaw) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    *roll = atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1.0)
        *pitch = copysign(M_PI / 2.0, sinp) * 180.0 / M_PI;
    else
        *pitch = asin(sinp) * 180.0 / M_PI;

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    *yaw = atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
}

// Convert physical units to sensor counts
int16_t accel_to_counts(double accel_ms2) {
    double accel_g = accel_ms2 / 9.80665;
    int32_t counts = (int32_t)(accel_g * MPU9250_COUNTSPERG);
    if (counts > 32767) counts = 32767;
    if (counts < -32768) counts = -32768;
    return (int16_t)counts;
}

int16_t gyro_to_counts(double gyro_rads) {
    double gyro_dps = gyro_rads * 180.0 / M_PI;
    int32_t counts = (int32_t)(gyro_dps * MPU9250_COUNTSPERDPS);
    if (counts > 32767) counts = 32767;
    if (counts < -32768) counts = -32768;
    return (int16_t)counts;
}

int16_t mag_to_counts(double mag_ut) {
    int32_t counts = (int32_t)(mag_ut * AK8963_COUNTSPERUT);
    if (counts > 32767) counts = 32767;
    if (counts < -32768) counts = -32768;
    return (int16_t)counts;
}

// Parse CSV line
int parse_csv_line(char *line, imu_sample_t *sample) {
    char *token;
    int field = 0;

    token = strtok(line, ";");
    while (token != NULL && field < 14) {
        double value = atof(token);
        switch (field) {
            case 0: sample->timestamp = value; break;
            case 1: sample->quat_w = value; break;
            case 2: sample->quat_x = value; break;
            case 3: sample->quat_y = value; break;
            case 4: sample->quat_z = value; break;
            case 5: sample->acc_x = value; break;
            case 6: sample->acc_y = value; break;
            case 7: sample->acc_z = value; break;
            case 8: sample->gyro_x = value; break;
            case 9: sample->gyro_y = value; break;
            case 10: sample->gyro_z = value; break;
            case 11: sample->mag_x = value; break;
            case 12: sample->mag_y = value; break;
            case 13: sample->mag_z = value; break;
        }
        token = strtok(NULL, ";");
        field++;
    }

    return field >= 14;
}

int main(int argc, char *argv[]) {
    const char *filename = (argc > 1) ? argv[1] : "test/validation/TStick_Test02_Trial1.csv";

    printf("=======================================================\n");
    printf("9-AXIS END-TO-END VALIDATION WITH REPOIMU DATASET\n");
    printf("=======================================================\n");
    printf("Dataset: %s\n\n", filename);

    FILE *fp = fopen(filename, "r");
    if (!fp) {
        printf("ERROR: Could not open file: %s\n", filename);
        return 1;
    }

    // Skip header lines
    char line[512];
    fgets(line, sizeof(line), fp);  // Header line 1
    fgets(line, sizeof(line), fp);  // Header line 2

    // Initialize algorithm
    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);

    int sample_count = 0;
    int pass_count = 0;
    int fail_count = 0;
    double max_error = 0.0;
    double sum_error = 0.0;

    // Test every 100th sample to reduce computation
    int skip_counter = 0;

    while (fgets(line, sizeof(line), fp) && sample_count < 100) {
        skip_counter++;
        if (skip_counter % 100 != 0) continue;  // Test every 100th sample

        imu_sample_t sample;
        if (!parse_csv_line(line, &sample)) continue;

        // Skip samples with zero quaternion (no ground truth)
        if (sample.quat_w == 0 && sample.quat_x == 0 &&
            sample.quat_y == 0 && sample.quat_z == 0) continue;

        // Create accelerometer data
        sensor_data_t accel_data;
        accel_data.sensordata[0] = accel_to_counts(sample.acc_x);
        accel_data.sensordata[1] = accel_to_counts(sample.acc_y);
        accel_data.sensordata[2] = accel_to_counts(sample.acc_z);
        accel_data.sensorID = ACC;
        accel_data.timestamp = (int64_t)(sample.timestamp * 1000000000.0);

        // Create gyroscope data
        sensor_data_t gyro_data;
        gyro_data.sensordata[0] = gyro_to_counts(sample.gyro_x);
        gyro_data.sensordata[1] = gyro_to_counts(sample.gyro_y);
        gyro_data.sensordata[2] = gyro_to_counts(sample.gyro_z);
        gyro_data.sensorID = GYRO;
        gyro_data.timestamp = (int64_t)(sample.timestamp * 1000000000.0);

        // Create magnetometer data
        sensor_data_t mag_data;
        mag_data.sensordata[0] = mag_to_counts(sample.mag_x);
        mag_data.sensordata[1] = mag_to_counts(sample.mag_y);
        mag_data.sensordata[2] = mag_to_counts(sample.mag_z);
        mag_data.sensorID = MAG;
        mag_data.timestamp = (int64_t)(sample.timestamp * 1000000000.0);

        // Feed data to algorithm
        sf_9xagm_data_preproc(algo_id, &accel_data);
        sf_9xagm_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        // Run algorithm
        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);

        // Get ground truth Euler angles
        double gt_roll, gt_pitch, gt_yaw;
        quaternion_to_euler(sample.quat_w, sample.quat_x, sample.quat_y, sample.quat_z,
                           &gt_roll, &gt_pitch, &gt_yaw);

        // Get calculated Euler angles from algorithm
        // orientation[0]=yaw, orientation[1]=pitch, orientation[2]=roll
        double calc_yaw = output.orientation[0];
        double calc_pitch = output.orientation[1];
        double calc_roll = output.orientation[2];

        // Calculate errors
        double error_roll = fabs(calc_roll - gt_roll);
        double error_pitch = fabs(calc_pitch - gt_pitch);
        double error_yaw = fabs(calc_yaw - gt_yaw);

        // Handle angle wrapping
        if (error_roll > 180.0) error_roll = 360.0 - error_roll;
        if (error_pitch > 180.0) error_pitch = 360.0 - error_pitch;
        if (error_yaw > 180.0) error_yaw = 360.0 - error_yaw;

        double total_error = sqrt(error_roll * error_roll + error_pitch * error_pitch + error_yaw * error_yaw);
        sum_error += total_error;
        if (total_error > max_error) max_error = total_error;

        int pass = (error_roll < TOLERANCE_DEGREES &&
                   error_pitch < TOLERANCE_DEGREES &&
                   error_yaw < TOLERANCE_DEGREES);

        if (sample_count < 5 || !pass) {  // Show first 5 and all failures
            printf("Sample %d (t=%.2fs):\n", sample_count + 1, sample.timestamp);
            printf("  Ground Truth: Roll=%7.2f°  Pitch=%7.2f°  Yaw=%7.2f°\n",
                   gt_roll, gt_pitch, gt_yaw);
            printf("  Calculated:   Roll=%7.2f°  Pitch=%7.2f°  Yaw=%7.2f°\n",
                   calc_roll, calc_pitch, calc_yaw);
            printf("  Error:        Roll=%7.2f°  Pitch=%7.2f°  Yaw=%7.2f°  Total=%.2f°  %s\n\n",
                   error_roll, error_pitch, error_yaw, total_error, pass ? "✓ PASS" : "✗ FAIL");
        }

        if (pass) pass_count++;
        else fail_count++;

        sample_count++;
    }

    sf_9xagm_algo_stop(algo_id);

    fclose(fp);

    // Print summary
    printf("=======================================================\n");
    printf("SUMMARY\n");
    printf("=======================================================\n");
    printf("Total samples tested: %d\n", sample_count);
    printf("Passed: %d (%.1f%%)\n", pass_count, 100.0 * pass_count / sample_count);
    printf("Failed: %d (%.1f%%)\n", fail_count, 100.0 * fail_count / sample_count);
    printf("Average error: %.2f°\n", sum_error / sample_count);
    printf("Maximum error: %.2f°\n", max_error);
    printf("Tolerance: %.1f°\n", TOLERANCE_DEGREES);
    printf("\n");

    if (fail_count == 0) {
        printf("✓ ALL TESTS PASSED\n");
        return 0;
    } else {
        printf("✗ SOME TESTS FAILED\n");
        return 1;
    }
}
