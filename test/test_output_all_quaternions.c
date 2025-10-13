/**
 * Test program to output all C quaternions to CSV for comparison with Python
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../code/algo/inc/algo_sf_fusion.h"
#include "../code/algo/inc/algo_sf_6x_sensor_fusion.h"
#include "../code/app/inc/main.h"
#include "../test/data/testdata/fusion/test_input_output_0922.h"

int main(void) {
    printf("=== C Quaternion Output Generator ===\n\n");

    // Initialize sensor fusion
    sf_algo_init_data_t algo_init_data;
    algo_init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    algo_init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    algo_init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t sf_algo_id = sf_6xag_algo_init(&algo_init_data);
    printf("Initialized 6-axis sensor fusion algorithm\n\n");

    // Open output file
    FILE* fp = fopen("c_reference_quaternions_0922.csv", "w");
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

    printf("Processing...\n");

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
    printf("Output saved to: c_reference_quaternions_0922.csv\n");

    sf_6xag_algo_stop(sf_algo_id);
    return 0;
}
