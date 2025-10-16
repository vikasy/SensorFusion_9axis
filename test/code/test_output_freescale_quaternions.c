/**
 * Test program to output C quaternions using FREESCALE sensor specs
 *
 * Uses FREESCALE (FXOS8700CQ + FXAS21000) sensor specifications
 * to process the same test data for comparison with INVENSENSE
 *
 * Author: Vikas Yadav
 * Date: 2025-10-12
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Define FREESCALE platform before including sensor specs
#ifndef SENSOR_PLATFORM_FREESCALE
#define SENSOR_PLATFORM_FREESCALE
#endif

#include "../code/algo/inc/algo_sf_fusion.h"
#include "../code/algo/inc/algo_sf_6x_sensor_fusion.h"
#include "../code/algo/inc/algo_sf_sensordata.h"
#include "../code/app/inc/main.h"
#include "../code/app/inc/sensor_spec_agm.h"
#include "../test/data/testdata/fusion/test_input_output_0922.h"

// FREESCALE sensor specifications
#define FSL_ACCEL_RANGE_G 4.0f
#define FSL_ACCEL_MAX_COUNT 8191  // 14-bit
#define FSL_ACC_SCALE (FSL_ACCEL_RANGE_G / FSL_ACCEL_MAX_COUNT)

#define FSL_GYRO_RANGE_DPS 1000.0f
#define FSL_GYRO_MAX_COUNT 32767  // 16-bit
#define FSL_GYRO_SCALE (FSL_GYRO_RANGE_DPS / FSL_GYRO_MAX_COUNT)

#define FSL_MAG_RANGE_UT 1200.0f
#define FSL_MAG_MAX_COUNT 32767
#define FSL_MAG_SCALE (FSL_MAG_RANGE_UT / FSL_MAG_MAX_COUNT)

int main(void) {
    printf("================================================================================\n");
    printf("C Quaternion Output Generator - FREESCALE Platform\n");
    printf("================================================================================\n");
    printf("Platform: NXP FRDM-STBC-AGM01\n");
    printf("  Accelerometer: FXOS8700CQ, %.1fg range, 14-bit\n", FSL_ACCEL_RANGE_G);
    printf("    Scale: %.10f g/count\n", FSL_ACC_SCALE);
    printf("  Gyroscope: FXAS21000, %.1fdps range, 16-bit\n", FSL_GYRO_RANGE_DPS);
    printf("    Scale: %.10f dps/count\n", FSL_GYRO_SCALE);
    printf("================================================================================\n\n");

    // Initialize sensor fusion with FREESCALE specs
    sf_algo_init_data_t algo_init_data;
    algo_init_data.Acc_GPERCOUNT = FSL_ACC_SCALE;
    algo_init_data.Gyro_DPSPERCOUNT = FSL_GYRO_SCALE;
    algo_init_data.Mag_UTPERCOUNT = FSL_MAG_SCALE;

    uintptr_t sf_algo_id = sf_6xag_algo_init(&algo_init_data);
    printf("Initialized 6-axis sensor fusion algorithm\n\n");

    // Open output file
    FILE* fp = fopen("c_freescale_quaternions_0922.csv", "w");
    if (!fp) {
        printf("Error: Cannot create output file\n");
        return 1;
    }

    // Write header
    fprintf(fp, "sample,timestamp,sensor_id,q0,q1,q2,q3\n");

    // Process test data
    int len = sizeof(sensor_input_data) / sizeof(test_sensor_sample_t);
    printf("Total input samples: %d\n", len);

    int sf_run_cnt = 0;
    uint32_t sf_ready_to_run = 0;
    sensor_data_t sensor;
    sf_algo_output_t algo_output;

    printf("Processing with FREESCALE specs...\n");

    for (int i = 0; i < len; i++) {
        int sensor_id = sensor_input_data[i].id;

        // Skip magnetometer
        if (sensor_id == 2) {
            continue;
        }

        // Prepare sensor data
        sensor.sensordata[0] = sensor_input_data[i].x;
        sensor.sensordata[1] = sensor_input_data[i].y;
        sensor.sensordata[2] = sensor_input_data[i].z;
        sensor.timestamp = sensor_input_data[i].ts;
        sensor.sensorID = (sensor_id == 0) ? ACC : GYRO;

        // Preprocess
        sf_ready_to_run = sf_6xag_data_preproc(sf_algo_id, &sensor);

        // Run fusion
        if (sf_ready_to_run == 3) {  // Both ACC and GYRO ready
            sf_6xag_algo_run(sf_algo_id, &algo_output);
            sf_ready_to_run = 0;

            // Write quaternion to CSV
            fprintf(fp, "%d,%llu,%d,%.15f,%.15f,%.15f,%.15f\n",
                    sf_run_cnt,
                    sensor_input_data[i].ts,
                    sensor_id,
                    algo_output.quat.q0,
                    algo_output.quat.q1,
                    algo_output.quat.q2,
                    algo_output.quat.q3);

            sf_run_cnt++;

            // Progress indicator
            if (sf_run_cnt % 100 == 0) {
                printf("  Fusion runs: %d...\n", sf_run_cnt);
            }
        }
    }

    fclose(fp);

    printf("\nComplete! Processed %d input samples\n", len);
    printf("Generated %d fusion outputs (quaternions)\n", sf_run_cnt);
    printf("Output saved to: c_freescale_quaternions_0922.csv\n");

    sf_6xag_algo_stop(sf_algo_id);
    return 0;
}
