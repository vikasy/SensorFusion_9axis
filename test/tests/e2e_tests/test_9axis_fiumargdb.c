/**
 * @file test_9axis_fiumargdb.c
 * @brief End-to-end validation using FIUMARGDB real-world dataset
 *
 * Tests 9-axis sensor fusion algorithm with actual MARG data and ground truth quaternions
 * from the FIUMARGDB dataset (optical motion capture reference)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"
#include "sensor_spec_agm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_SAMPLES 20000
#define TOLERANCE_DEGREES 10.0  // Relaxed tolerance for real-world data

typedef struct {
    double timestamp;
    double pos_x, pos_y, pos_z;
    double cam_qw, cam_qx, cam_qy, cam_qz;  // Ground truth from camera/Vicon
    double ss_qw, ss_qx, ss_qy, ss_qz;      // Sensor suite quaternion
    double gyro_x, gyro_y, gyro_z;           // Gyroscope (units unknown, assume rad/s)
    double acc_x, acc_y, acc_z;              // Accelerometer (units unknown, assume m/s²)
    double mag_x, mag_y, mag_z;              // Magnetometer (units unknown, assume µT)
    double stillness;
    double isTracked;
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

// Convert FIUMARGDB units to sensor counts
// FIUMARGDB units (from showMARGSignals.m):
// - Accelerometer: Multiples of g (already in g units)
// - Gyroscope: rad/s
// - Magnetometer: Gauss (need to convert: 1 Gauss = 100 µT)
int16_t accel_to_counts(double accel_g) {
    // Input is already in g units, convert directly to counts
    int32_t counts = (int32_t)(accel_g * MPU9250_COUNTSPERG);
    if (counts > 32767) counts = 32767;
    if (counts < -32768) counts = -32768;
    return (int16_t)counts;
}

int16_t gyro_to_counts(double gyro_rads) {
    // Input is in rad/s, convert to dps then to counts
    double gyro_dps = gyro_rads * 180.0 / M_PI;
    int32_t counts = (int32_t)(gyro_dps * MPU9250_COUNTSPERDPS);
    if (counts > 32767) counts = 32767;
    if (counts < -32768) counts = -32768;
    return (int16_t)counts;
}

int16_t mag_to_counts(double mag_gauss) {
    // Input is in Gauss, convert to µT (1 Gauss = 100 µT)
    double mag_ut = mag_gauss * 100.0;
    int32_t counts = (int32_t)(mag_ut * AK8963_COUNTSPERUT);
    if (counts > 32767) counts = 32767;
    if (counts < -32768) counts = -32768;
    return (int16_t)counts;
}

// Parse FIUMARGDB CSV line (comma-separated)
int parse_csv_line(char *line, imu_sample_t *sample) {
    char *token;
    int field = 0;

    token = strtok(line, ",");
    while (token != NULL && field < 23) {
        double value = atof(token);
        switch (field) {
            case 0: sample->timestamp = value; break;
            case 1: sample->pos_x = value; break;
            case 2: sample->pos_y = value; break;
            case 3: sample->pos_z = value; break;
            case 4: sample->cam_qx = value; break;
            case 5: sample->cam_qy = value; break;
            case 6: sample->cam_qz = value; break;
            case 7: sample->cam_qw = value; break;
            case 8: sample->ss_qx = value; break;
            case 9: sample->ss_qy = value; break;
            case 10: sample->ss_qz = value; break;
            case 11: sample->ss_qw = value; break;
            case 12: sample->gyro_x = value; break;
            case 13: sample->gyro_y = value; break;
            case 14: sample->gyro_z = value; break;
            case 15: sample->acc_x = value; break;
            case 16: sample->acc_y = value; break;
            case 17: sample->acc_z = value; break;
            case 18: sample->mag_x = value; break;
            case 19: sample->mag_y = value; break;
            case 20: sample->mag_z = value; break;
            case 21: sample->stillness = value; break;
            case 22: sample->isTracked = value; break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    return field >= 23;
}

int main(int argc, char *argv[]) {
    const char *filename = (argc > 1) ? argv[1] : "../test/datasets/fiumargdb/rec01.csv";

    printf("=======================================================\n");
    printf("9-AXIS END-TO-END VALIDATION WITH FIUMARGDB DATASET\n");
    printf("=======================================================\n");
    printf("Dataset: %s\n\n", filename);

    FILE *fp = fopen(filename, "r");
    if (!fp) {
        printf("ERROR: Could not open file: %s\n", filename);
        return 1;
    }

    // Skip header line
    char line[1024];
    fgets(line, sizeof(line), fp);

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

    // Test every 50th sample to reduce computation
    int skip_counter = 0;

    while (fgets(line, sizeof(line), fp) && sample_count < 100) {
        skip_counter++;
        if (skip_counter % 50 != 0) continue;  // Test every 50th sample

        imu_sample_t sample;
        if (!parse_csv_line(line, &sample)) continue;

        // Skip samples with zero quaternion (no ground truth) or not tracked
        if (sample.isTracked < 0.5) continue;
        if (sample.cam_qw == 0 && sample.cam_qx == 0 &&
            sample.cam_qy == 0 && sample.cam_qz == 0) continue;

        // Create accelerometer data
        sensor_data_t accel_data;
        accel_data.sensordata[0] = accel_to_counts(sample.acc_x);
        accel_data.sensordata[1] = accel_to_counts(sample.acc_y);
        accel_data.sensordata[2] = accel_to_counts(sample.acc_z);
        accel_data.sensorID = ACC;
        accel_data.timestamp = (int64_t)(sample.timestamp * 1000000.0);

        // Create gyroscope data
        sensor_data_t gyro_data;
        gyro_data.sensordata[0] = gyro_to_counts(sample.gyro_x);
        gyro_data.sensordata[1] = gyro_to_counts(sample.gyro_y);
        gyro_data.sensordata[2] = gyro_to_counts(sample.gyro_z);
        gyro_data.sensorID = GYRO;
        gyro_data.timestamp = (int64_t)(sample.timestamp * 1000000.0);

        // Create magnetometer data
        sensor_data_t mag_data;
        mag_data.sensordata[0] = mag_to_counts(sample.mag_x);
        mag_data.sensordata[1] = mag_to_counts(sample.mag_y);
        mag_data.sensordata[2] = mag_to_counts(sample.mag_z);
        mag_data.sensorID = MAG;
        mag_data.timestamp = (int64_t)(sample.timestamp * 1000000.0);

        // Feed data to algorithm
        sf_9xagm_data_preproc(algo_id, &accel_data);
        sf_9xagm_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        // Run algorithm
        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);

        // Get ground truth Euler angles from camera quaternion
        double gt_roll, gt_pitch, gt_yaw;
        quaternion_to_euler(sample.cam_qw, sample.cam_qx, sample.cam_qy, sample.cam_qz,
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
            printf("Sample %d (t=%.2fms):\n", sample_count + 1, sample.timestamp);
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
