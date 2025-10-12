/**
 * @file test_orientation_mapping.c
 * @brief Diagnostic test to verify orientation output mapping
 *
 * This test creates specific simple rotations and prints both the algorithm
 * output and what we expect, to identify the correct axis mapping.
 *
 * Author: Vikas Yadav / Claude Code
 * Date: 2025-10-12
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "algo_sf_6x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ACCEL_SCALE_G_PER_COUNT (1.0 / 16384.0)  // 2g range
#define GYRO_SCALE_DPS_PER_COUNT (1.0 / 131.0)   // 250 dps range
#define MAG_SCALE_UT_PER_COUNT (0.15)

void print_orientation_output(const char *test_name, const sf_algo_output_t *output) {
    printf("\n%s:\n", test_name);
    printf("  output.orientation[0] = %.3f°\n", output->orientation[0]);
    printf("  output.orientation[1] = %.3f°\n", output->orientation[1]);
    printf("  output.orientation[2] = %.3f°\n", output->orientation[2]);
    printf("  output.quat = {%.4f, %.4f, %.4f, %.4f}\n",
           output->quat.q0, output->quat.q1, output->quat.q2, output->quat.q3);
}

int main(void) {
    printf("=======================================================================\n");
    printf("ORIENTATION OUTPUT MAPPING DIAGNOSTIC\n");
    printf("=======================================================================\n");
    printf("\nThis test verifies the correct mapping of orientation[3] array.\n");
    printf("The interface header says: 'pitch, yaw, and roll angles'\n");
    printf("But the actual mapping needs verification.\n\n");

    // Initialize algorithm
    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = ACCEL_SCALE_G_PER_COUNT;
    init_data.Gyro_DPSPERCOUNT = GYRO_SCALE_DPS_PER_COUNT;
    init_data.Mag_UTPERCOUNT = MAG_SCALE_UT_PER_COUNT;

    uintptr_t algo_id = sf_6xag_algo_init(&init_data);
    if (algo_id == 0) {
        printf("ERROR: Failed to initialize algorithm\n");
        return 1;
    }

    printf("Algorithm initialized.\n");

    // Test 1: Static level - should be all zeros
    printf("\n-----------------------------------------------------------------------\n");
    printf("TEST 1: STATIC LEVEL (expected: roll=0°, pitch=0°, yaw=0°)\n");
    printf("-----------------------------------------------------------------------\n");

    for (int i = 0; i < 50; i++) {
        sensor_data_t accel_data, gyro_data;
        int64_t timestamp = i * 10000000LL;  // 10ms intervals

        // Flat on table: Z-axis pointing up with gravity
        accel_data.sensorID = ACC;
        accel_data.timestamp = timestamp;
        accel_data.sensordata[0] = 0;      // X: 0g
        accel_data.sensordata[1] = 0;      // Y: 0g
        accel_data.sensordata[2] = 16384;  // Z: +1g (pointing up)

        // No rotation
        gyro_data.sensorID = GYRO;
        gyro_data.timestamp = timestamp;
        gyro_data.sensordata[0] = 0;
        gyro_data.sensordata[1] = 0;
        gyro_data.sensordata[2] = 0;

        sf_6xag_data_preproc(algo_id, &accel_data);
        sf_6xag_data_preproc(algo_id, &gyro_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_6xag_algo_run(algo_id, &output);

        if (i == 49) {  // Print last sample
            print_orientation_output("Static Level", &output);
        }
    }

    // Test 2: Tilted 90° about X-axis (nose up)
    printf("\n-----------------------------------------------------------------------\n");
    printf("TEST 2: TILTED 90° PITCH UP (nose up, expected: roll=0°, pitch=90°, yaw=0°)\n");
    printf("-----------------------------------------------------------------------\n");

    // Re-initialize for fresh start
    sf_6xag_algo_stop(algo_id);
    algo_id = sf_6xag_algo_init(&init_data);

    for (int i = 0; i < 50; i++) {
        sensor_data_t accel_data, gyro_data;
        int64_t timestamp = i * 10000000LL;

        // Pitched up 90°: gravity along -Y axis
        accel_data.sensorID = ACC;
        accel_data.timestamp = timestamp;
        accel_data.sensordata[0] = 0;       // X: 0g
        accel_data.sensordata[1] = -16384;  // Y: -1g
        accel_data.sensordata[2] = 0;       // Z: 0g

        gyro_data.sensorID = GYRO;
        gyro_data.timestamp = timestamp;
        gyro_data.sensordata[0] = 0;
        gyro_data.sensordata[1] = 0;
        gyro_data.sensordata[2] = 0;

        sf_6xag_data_preproc(algo_id, &accel_data);
        sf_6xag_data_preproc(algo_id, &gyro_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_6xag_algo_run(algo_id, &output);

        if (i == 49) {
            print_orientation_output("Pitch 90° Up", &output);
        }
    }

    // Test 3: Tilted 90° about Y-axis (left side down)
    printf("\n-----------------------------------------------------------------------\n");
    printf("TEST 3: TILTED 90° ROLL LEFT (left side down, expected: roll=90°, pitch=0°, yaw=0°)\n");
    printf("-----------------------------------------------------------------------\n");

    sf_6xag_algo_stop(algo_id);
    algo_id = sf_6xag_algo_init(&init_data);

    for (int i = 0; i < 50; i++) {
        sensor_data_t accel_data, gyro_data;
        int64_t timestamp = i * 10000000LL;

        // Rolled left 90°: gravity along +X axis
        accel_data.sensorID = ACC;
        accel_data.timestamp = timestamp;
        accel_data.sensordata[0] = 16384;  // X: +1g
        accel_data.sensordata[1] = 0;      // Y: 0g
        accel_data.sensordata[2] = 0;      // Z: 0g

        gyro_data.sensorID = GYRO;
        gyro_data.timestamp = timestamp;
        gyro_data.sensordata[0] = 0;
        gyro_data.sensordata[1] = 0;
        gyro_data.sensordata[2] = 0;

        sf_6xag_data_preproc(algo_id, &accel_data);
        sf_6xag_data_preproc(algo_id, &gyro_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_6xag_algo_run(algo_id, &output);

        if (i == 49) {
            print_orientation_output("Roll 90° Left", &output);
        }
    }

    // Test 4: Upside down
    printf("\n-----------------------------------------------------------------------\n");
    printf("TEST 4: UPSIDE DOWN (expected: roll=180° or pitch=180°)\n");
    printf("-----------------------------------------------------------------------\n");

    sf_6xag_algo_stop(algo_id);
    algo_id = sf_6xag_algo_init(&init_data);

    for (int i = 0; i < 50; i++) {
        sensor_data_t accel_data, gyro_data;
        int64_t timestamp = i * 10000000LL;

        // Upside down: Z-axis pointing down
        accel_data.sensorID = ACC;
        accel_data.timestamp = timestamp;
        accel_data.sensordata[0] = 0;       // X: 0g
        accel_data.sensordata[1] = 0;       // Y: 0g
        accel_data.sensordata[2] = -16384;  // Z: -1g

        gyro_data.sensorID = GYRO;
        gyro_data.timestamp = timestamp;
        gyro_data.sensordata[0] = 0;
        gyro_data.sensordata[1] = 0;
        gyro_data.sensordata[2] = 0;

        sf_6xag_data_preproc(algo_id, &accel_data);
        sf_6xag_data_preproc(algo_id, &gyro_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_6xag_algo_run(algo_id, &output);

        if (i == 49) {
            print_orientation_output("Upside Down", &output);
        }
    }

    printf("\n=======================================================================\n");
    printf("ANALYSIS\n");
    printf("=======================================================================\n");
    printf("\nBased on the outputs above, determine the mapping:\n");
    printf("  orientation[0] = ?\n");
    printf("  orientation[1] = ?\n");
    printf("  orientation[2] = ?\n");
    printf("\nExpected from source code analysis:\n");
    printf("  orientation[0] = PsiPost   (yaw)\n");
    printf("  orientation[1] = ThetaPost (pitch)\n");
    printf("  orientation[2] = PhiPost   (roll)\n");
    printf("\nBut interface header claims: [pitch, yaw, roll]\n");
    printf("=======================================================================\n");

    sf_6xag_algo_stop(algo_id);
    return 0;
}
