#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "algo_sf_fusion.h"
#include "algo_sf_6x_sensor_fusion.h"
#include "sensor_spec_agm.h"
#include "main.h"
#include "test_input_output_0922.h"

int main(void)
{
    printf("6-Axis Sensor Fusion Demo\n");
    printf("==========================\n\n");

    // Initialize sensor fusion with INVENSENSE specs
    sf_algo_init_data_t algo_init_data;
    algo_init_data.Acc_GPERCOUNT = ACCEL_FGPERCOUNT;
    algo_init_data.Gyro_DPSPERCOUNT = GYRO_FDPSPERCOUNT;
    algo_init_data.Mag_UTPERCOUNT = MAG_FUTPERCOUNT;

    uintptr_t sf_algo_id = sf_6xag_algo_init(&algo_init_data);
    printf("Initialized 6-axis sensor fusion\n");
    printf("  Accelerometer scale: %.10f g/count\n", ACCEL_FGPERCOUNT);
    printf("  Gyroscope scale: %.10f dps/count\n\n", GYRO_FDPSPERCOUNT);

    // Process sensor data from test vectors
    sensor_data_t sensor;
    sf_algo_output_t algo_output;
    uint32_t sf_ready_to_run = 0;
    int fusion_count = 0;

    printf("Processing sensor samples...\n\n");

    // Simple test: just run the algorithm on a few samples
    for (int i = 0; i < 100 && i < (int)(sizeof(sensor_input_data)/sizeof(sensor_input_data[0])); i++) {
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

            // Print first few quaternion outputs
            if (fusion_count <= 5) {
                printf("Fusion #%d: q = [%.6f, %.6f, %.6f, %.6f]\n",
                       fusion_count,
                       algo_output.quat.q0,
                       algo_output.quat.q1,
                       algo_output.quat.q2,
                       algo_output.quat.q3);
            }
        }
    }

    printf("\nTotal fusion updates: %d\n", fusion_count);
    printf("Sensor fusion demo complete!\n");

    sf_6xag_algo_stop(sf_algo_id);
    return 0;
}
