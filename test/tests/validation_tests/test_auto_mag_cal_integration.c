/**
 * @file test_auto_mag_cal_integration.c
 * @brief Test automatic magnetometer calibration integrated into 9-axis fusion
 *
 * This test demonstrates that the mag_cal module is now fully integrated
 * into the 9-axis sensor fusion algorithm and performs calibration automatically.
 *
 * @author Vikas Yadav
 * @date 2025
 */

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_interface.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_mag_cal.h"
#include "sensor_spec_agm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Simulate magnetometer data with hard iron offset
typedef struct {
    float mag_field_true[3];  // True magnetic field
    float hard_iron[3];       // Hard iron offset to simulate
    int sample_count;
} test_context_t;

// Generate magnetometer sample with hard iron offset and rotation
static void generate_mag_sample(test_context_t *ctx, int index, float mag_out[3]) {
    // Rotate the magnetic field vector to simulate device rotation
    float theta = (float)index * 2.0f * M_PI / 100.0f; // Full rotation over 100 samples
    float phi = (float)index * M_PI / 50.0f;          // Half rotation in another axis

    // Rotate true field
    float rotated[3];
    rotated[0] = ctx->mag_field_true[0] * cosf(theta) - ctx->mag_field_true[1] * sinf(theta);
    rotated[1] = ctx->mag_field_true[0] * sinf(theta) + ctx->mag_field_true[1] * cosf(theta);
    rotated[2] = ctx->mag_field_true[2] * cosf(phi) - rotated[1] * sinf(phi);

    // Add hard iron offset
    for (int i = 0; i < 3; i++) {
        mag_out[i] = rotated[i] + ctx->hard_iron[i];
    }

    ctx->sample_count++;
}

// Generate accelerometer sample (pointing down in gravity direction)
static void generate_accel_sample(float accel_out[3]) {
    accel_out[0] = 0.0f;
    accel_out[1] = 0.0f;
    accel_out[2] = 9.81f; // 1g down
}

// Generate gyroscope sample (stationary or slow rotation)
static void generate_gyro_sample(int index, float gyro_out[3]) {
    // Slow rotation to avoid motion rejection
    gyro_out[0] = 0.1f * sinf((float)index * 0.1f);
    gyro_out[1] = 0.1f * cosf((float)index * 0.1f);
    gyro_out[2] = 0.05f;
}

int main(void) {
    printf("========================================\n");
    printf("9-Axis Fusion Auto-Calibration Test\n");
    printf("========================================\n\n");

    // Initialize test context
    test_context_t ctx;
    ctx.mag_field_true[0] = 50.0f;  // True mag field (μT)
    ctx.mag_field_true[1] = 0.0f;
    ctx.mag_field_true[2] = 30.0f;
    ctx.hard_iron[0] = 10.0f;       // Hard iron offset to be calibrated
    ctx.hard_iron[1] = -5.0f;
    ctx.hard_iron[2] = 8.0f;
    ctx.sample_count = 0;

    printf("Test Setup:\n");
    printf("  True magnetic field: [%.1f, %.1f, %.1f] μT\n",
           ctx.mag_field_true[0], ctx.mag_field_true[1], ctx.mag_field_true[2]);
    printf("  Hard iron offset:    [%.1f, %.1f, %.1f] μT\n",
           ctx.hard_iron[0], ctx.hard_iron[1], ctx.hard_iron[2]);
    printf("\n");

    // Initialize 9-axis sensor fusion
    printf("STEP 1: Initialize 9-Axis Fusion\n");
    printf("--------------------------------------------\n");

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);
    if (algo_id == 0) {
        printf("✗ Failed to initialize fusion algorithm\n");
        return 1;
    }

    printf("✓ Fusion algorithm initialized\n");

    // Access state to check mag_cal status
    state_vec_9XAGM_t *state = (state_vec_9XAGM_t*)algo_id;
    if (state->mag_cal_state == NULL) {
        printf("✗ Mag calibration module not initialized\n");
        sf_9xagm_algo_stop(algo_id);
        return 1;
    }

    mag_cal_state_t *mag_cal = (mag_cal_state_t*)state->mag_cal_state;
    printf("✓ Mag calibration module active\n");
    printf("  Initial status: %d (0=Uncalibrated)\n", mag_cal_get_status(mag_cal));
    printf("\n");

    // Run fusion with sensor data
    printf("STEP 2: Process Sensor Data\n");
    printf("--------------------------------------------\n");
    printf("Processing 120 samples (should auto-calibrate at ~50-60 samples)...\n\n");

    bool calibration_triggered = false;
    mag_cal_status_t prev_status = MAG_CAL_STATUS_UNCALIBRATED;

    for (int i = 0; i < 120; i++) {
        // Generate sensor data
        float accel_f[3], gyro_f[3], mag_f[3];
        generate_accel_sample(accel_f);
        generate_gyro_sample(i, gyro_f);
        generate_mag_sample(&ctx, i, mag_f);

        // Convert to sensor_data_t format
        sensor_data_t accel_data, gyro_data, mag_data;

        // Accelerometer
        accel_data.sensorID = ACC;
        accel_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        accel_data.sensordata[0] = (int16_t)((accel_f[0] / MPU9250_GRAVITY_MPS2) * MPU9250_COUNTSPERG);
        accel_data.sensordata[1] = (int16_t)((accel_f[1] / MPU9250_GRAVITY_MPS2) * MPU9250_COUNTSPERG);
        accel_data.sensordata[2] = (int16_t)((accel_f[2] / MPU9250_GRAVITY_MPS2) * MPU9250_COUNTSPERG);

        // Gyroscope (convert dps to counts)
        gyro_data.sensorID = GYRO;
        gyro_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        gyro_data.sensordata[0] = (int16_t)(gyro_f[0] / MPU9250_FDPSPERCOUNT);
        gyro_data.sensordata[1] = (int16_t)(gyro_f[1] / MPU9250_FDPSPERCOUNT);
        gyro_data.sensordata[2] = (int16_t)(gyro_f[2] / MPU9250_FDPSPERCOUNT);

        // Magnetometer (convert μT to counts)
        mag_data.sensorID = MAG;
        mag_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        mag_data.sensordata[0] = (int16_t)(mag_f[0] / AK8963_FUTPERCOUNT);
        mag_data.sensordata[1] = (int16_t)(mag_f[1] / AK8963_FUTPERCOUNT);
        mag_data.sensordata[2] = (int16_t)(mag_f[2] / AK8963_FUTPERCOUNT);

        // Feed data to fusion algorithm
        sf_6xag_data_preproc(algo_id, &accel_data);
        sf_6xag_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        // Run fusion algorithm
        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);

        // Check calibration status
        mag_cal_status_t current_status = mag_cal_get_status(mag_cal);

        if (current_status != prev_status) {
            printf("  [Sample %3d] Status changed: %d -> %d\n", i, prev_status, current_status);

            if (current_status == MAG_CAL_STATUS_COLLECTING) {
                printf("               Started collecting calibration samples\n");
            } else if (current_status == MAG_CAL_STATUS_CALIBRATED) {
                printf("               ✓ Auto-calibration complete!\n");
                printf("               Quality: %.4f\n", mag_cal_get_quality(mag_cal));
                calibration_triggered = true;
            } else if (current_status == MAG_CAL_STATUS_POOR_QUALITY) {
                printf("               ⚠ Calibration quality insufficient, continuing collection\n");
            }

            prev_status = current_status;
        }
    }

    printf("\n");

    // Check final calibration results
    printf("STEP 3: Verify Calibration Results\n");
    printf("--------------------------------------------\n");

    if (mag_cal_is_valid(mag_cal)) {
        mag_cal_params_t params;
        if (mag_cal_get_params(mag_cal, &params)) {
            printf("✓ Calibration successful\n\n");

            printf("Estimated Hard Iron Offset:\n");
            printf("  Offset: [%.2f, %.2f, %.2f] μT\n",
                   params.offset[0], params.offset[1], params.offset[2]);

            printf("\nExpected Hard Iron Offset:\n");
            printf("  Offset: [%.2f, %.2f, %.2f] μT\n",
                   ctx.hard_iron[0], ctx.hard_iron[1], ctx.hard_iron[2]);

            printf("\nError:\n");
            printf("  Error:  [%.2f, %.2f, %.2f] μT\n",
                   params.offset[0] - ctx.hard_iron[0],
                   params.offset[1] - ctx.hard_iron[1],
                   params.offset[2] - ctx.hard_iron[2]);

            // Calculate error magnitude
            float error_mag = sqrtf(
                powf(params.offset[0] - ctx.hard_iron[0], 2) +
                powf(params.offset[1] - ctx.hard_iron[1], 2) +
                powf(params.offset[2] - ctx.hard_iron[2], 2)
            );

            printf("\nCalibration Accuracy:\n");
            printf("  Error magnitude: %.2f μT\n", error_mag);
            printf("  Quality metric:  %.4f\n", mag_cal_get_quality(mag_cal));
            printf("  Samples used:    %d\n", mag_cal->num_samples);

            if (error_mag < 5.0f) {
                printf("\n✓✓ PASS: Calibration error within acceptable range (<5 μT)\n");
            } else {
                printf("\n⚠ WARNING: Calibration error higher than expected\n");
            }

            // Verify sync to fusion state
            printf("\nFusion State Sync:\n");
            printf("  State MagCalOffset: [%.2f, %.2f, %.2f]\n",
                   state->MagCalOffset[0], state->MagCalOffset[1], state->MagCalOffset[2]);

            bool synced = true;
            for (int i = 0; i < 3; i++) {
                if (fabs(state->MagCalOffset[i] - params.offset[i]) > 0.01) {
                    synced = false;
                    break;
                }
            }

            if (synced) {
                printf("  ✓ Calibration parameters synced to fusion state\n");
            } else {
                printf("  ✗ Calibration parameters NOT synced\n");
            }
        }
    } else {
        printf("✗ Calibration not valid\n");
        printf("  Final status: %d\n", mag_cal_get_status(mag_cal));
        printf("  Samples collected: %d\n", mag_cal->num_samples);
    }

    // Cleanup
    printf("\n");
    printf("STEP 4: Cleanup\n");
    printf("--------------------------------------------\n");
    sf_9xagm_algo_stop(algo_id);
    printf("✓ Fusion algorithm stopped and cleaned up\n");

    printf("\n========================================\n");
    printf("Test Complete\n");
    printf("========================================\n");

    if (calibration_triggered && mag_cal_is_valid((mag_cal_state_t*)state->mag_cal_state)) {
        return 0; // Success
    } else {
        printf("\n⚠ Test incomplete: Calibration did not complete automatically\n");
        return 1; // Failure
    }
}
