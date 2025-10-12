/*******************************************************************************
 * Simple 6-Axis Validation Test
 *
 * Tests the 6-axis algorithm with corrected coordinate frame understanding
 ******************************************************************************/

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "algo_sf_6x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"
#include "sensor_spec_agm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void test_orientation(const char* description, double roll_deg, double pitch_deg) {
    // Initialize algorithm
    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_6xag_algo_init(&init_data);

    // Calculate expected accelerometer readings (corrected formula)
    double roll_rad = roll_deg * M_PI / 180.0;
    double pitch_rad = pitch_deg * M_PI / 180.0;
    double g = MPU9250_GRAVITY_MPS2;

    // X affects roll, Y affects pitch (empirically determined)
    double accel_x = g * sin(roll_rad) * cos(pitch_rad);
    double accel_y = -g * sin(pitch_rad) * cos(roll_rad);
    double accel_z = g * cos(roll_rad) * cos(pitch_rad);

    // Convert to counts
    int16_t ax = (int16_t)((accel_x / g) * MPU9250_COUNTSPERG);
    int16_t ay = (int16_t)((accel_y / g) * MPU9250_COUNTSPERG);
    int16_t az = (int16_t)((accel_z / g) * MPU9250_COUNTSPERG);

    // Prepare sensor data
    sensor_data_t accel_data, gyro_data;
    accel_data.sensordata[0] = ax;
    accel_data.sensordata[1] = ay;
    accel_data.sensordata[2] = az;
    accel_data.sensorID = ACC;

    gyro_data.sensordata[0] = 0;
    gyro_data.sensordata[1] = 0;
    gyro_data.sensordata[2] = 0;
    gyro_data.sensorID = GYRO;

    // Run algorithm
    for (int i = 0; i < 200; i++) {
        accel_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        gyro_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;

        sf_6xag_data_preproc(algo_id, &accel_data);
        sf_6xag_data_preproc(algo_id, &gyro_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_6xag_algo_run(algo_id, &output);
    }

    // Get final output
    sf_algo_output_t output;
    memset(&output, 0, sizeof(sf_algo_output_t));
    sf_6xag_algo_run(algo_id, &output);

    // Output: [yaw, pitch, roll]
    double yaw_calc = output.orientation[0];
    double pitch_calc = output.orientation[1];
    double roll_calc = output.orientation[2];

    printf("\n%s\n", description);
    printf("  Expected: Roll=%.2f°, Pitch=%.2f°\n", roll_deg, pitch_deg);
    printf("  Input accel (m/s²): [%.3f, %.3f, %.3f]\n", accel_x, accel_y, accel_z);
    printf("  Input counts: [%d, %d, %d]\n", ax, ay, az);
    printf("  Calculated: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n", roll_calc, pitch_calc, yaw_calc);
    printf("  Error: Roll=%.2f°, Pitch=%.2f°\n",
           fabs(roll_calc - roll_deg), fabs(pitch_calc - pitch_deg));
    printf("  Quaternion: [%.4f, %.4f, %.4f, %.4f]\n",
           output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3);

    // Check if within tolerance
    if (fabs(roll_calc - roll_deg) < 5.0 && fabs(pitch_calc - pitch_deg) < 5.0) {
        printf("  ✓ PASS\n");
    } else {
        printf("  ✗ FAIL\n");
    }

    sf_6xag_algo_stop(algo_id);
}

int main(void) {
    printf("================================================================================\n");
    printf("  6-Axis Sensor Fusion Validation (Corrected Coordinate Frame)\n");
    printf("  MPU9250 Sensor: ±4g range, 8192 counts/g\n");
    printf("================================================================================\n");

    test_orientation("Test 1: Level (0° roll, 0° pitch)", 0.0, 0.0);
    test_orientation("Test 2: 30° Roll", 30.0, 0.0);
    test_orientation("Test 3: 30° Pitch", 0.0, 30.0);
    test_orientation("Test 4: 45° Roll", 45.0, 0.0);
    test_orientation("Test 5: -30° Roll", -30.0, 0.0);
    test_orientation("Test 6: -30° Pitch", 0.0, -30.0);

    printf("\n================================================================================\n");
    printf("  Validation complete!\n");
    printf("================================================================================\n");

    return 0;
}
