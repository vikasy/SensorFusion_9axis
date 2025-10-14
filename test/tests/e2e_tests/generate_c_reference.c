/**
 * @file generate_c_reference.c
 * @brief Generate C reference quaternions for accuracy comparison
 *
 * Runs sensor fusion on test data and saves quaternion outputs to CSV.
 */

#include <stdio.h>
#include <stdint.h>
#include "algo_sf_fusion.h"
#include "algo_sf_6x_sensor_fusion.h"
#include "sensor_spec_agm.h"
#include "test_input_output_0922.h"

int main(void)
{
    printf("=======================================================================\n");
    printf("GENERATING C REFERENCE QUATERNIONS\n");
    printf("=======================================================================\n\n");

    // Initialize sensor fusion
    sf_algo_init_data_t algo_init_data;
    algo_init_data.Acc_GPERCOUNT = ACCEL_FGPERCOUNT;
    algo_init_data.Gyro_DPSPERCOUNT = GYRO_FDPSPERCOUNT;
    algo_init_data.Mag_UTPERCOUNT = MAG_FUTPERCOUNT;

    uintptr_t sf_algo_id = sf_6xag_algo_init(&algo_init_data);
    printf("6-axis sensor fusion initialized\n");
    printf("  Accel scale: %.10f g/count\n", ACCEL_FGPERCOUNT);
    printf("  Gyro scale:  %.10f dps/count\n\n", GYRO_FDPSPERCOUNT);

    // Open output file
    FILE *fp = fopen("c_reference_quaternions_0922.csv", "w");
    if (!fp) {
        fprintf(stderr, "Error: Cannot create output file\n");
        return 1;
    }

    // Write CSV header
    fprintf(fp, "sample_idx,timestamp,sensor_id,sensor_data,q0,q1,q2,q3\n");

    // Process all sensor data
    sensor_data_t sensor;
    sf_algo_output_t algo_output;
    uint32_t sf_ready_to_run = 0;
    int fusion_count = 0;
    int total_samples = sizeof(sensor_input_data) / sizeof(sensor_input_data[0]);

    printf("Processing %d input samples...\n", total_samples);

    for (int i = 0; i < total_samples; i++) {
        int sensor_id = sensor_input_data[i].id;

        // Skip magnetometer for 6-axis fusion
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

        // Run fusion when both sensors ready
        if (sf_ready_to_run == 3) {
            sf_6xag_algo_run(sf_algo_id, &algo_output);
            sf_ready_to_run = 0;
            fusion_count++;

            // Write quaternion output to CSV
            fprintf(fp, "%d,%ld,%d,[%d;%d;%d],%.16f,%.16f,%.16f,%.16f\n",
                    i,
                    sensor.timestamp,
                    sensor_id,
                    sensor_input_data[i].x,
                    sensor_input_data[i].y,
                    sensor_input_data[i].z,
                    algo_output.quat.q0,
                    algo_output.quat.q1,
                    algo_output.quat.q2,
                    algo_output.quat.q3);

            if (fusion_count % 100 == 0) {
                printf("  Fusion runs: %d...\n", fusion_count);
            }
        }
    }

    fclose(fp);

    printf("\nComplete! Processed %d input samples\n", total_samples);
    printf("Generated %d fusion outputs (reference quaternions)\n", fusion_count);
    printf("Output saved to: c_reference_quaternions_0922.csv\n\n");

    sf_6xag_algo_stop(sf_algo_id);
    return 0;
}
