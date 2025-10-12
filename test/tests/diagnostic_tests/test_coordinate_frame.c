/*******************************************************************************
 * Coordinate Frame Diagnostic Test
 *
 * Purpose: Empirically determine the exact coordinate frame convention used
 *          by the sensor fusion algorithm by testing with known accelerometer
 *          inputs and observing the output orientation angles.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "algo_sf_6x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"
#include "sensor_spec_agm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void test_case(const char* description, int16_t ax, int16_t ay, int16_t az) {
    // Initialize algorithm
    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_6xag_algo_init(&init_data);

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

    // Feed data multiple times for convergence
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

    // Convert counts to g
    double ax_g = ax * MPU9250_FGPERCOUNT;
    double ay_g = ay * MPU9250_FGPERCOUNT;
    double az_g = az * MPU9250_FGPERCOUNT;

    printf("\n%s\n", description);
    printf("  Input (counts): [%6d, %6d, %6d]\n", ax, ay, az);
    printf("  Input (g):      [%7.3f, %7.3f, %7.3f]\n", ax_g, ay_g, az_g);
    printf("  Output:         Yaw=%.2f°, Pitch=%.2f°, Roll=%.2f°\n",
           output.orientation[0], output.orientation[1], output.orientation[2]);
    printf("  Quaternion:     [%.4f, %.4f, %.4f, %.4f]\n",
           output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3);

    sf_6xag_algo_stop(algo_id);
}

int main(void) {
    printf("================================================================================\n");
    printf("  Coordinate Frame Diagnostic Tests\n");
    printf("  MPU9250 Sensor: ±4g range, 8192 counts/g\n");
    printf("================================================================================\n");

    // Test 1: Level (Z-up)
    test_case("Test 1: Level (Z-up)", 0, 0, 8192);

    // Test 2: Pure positive X acceleration (1g in X direction)
    test_case("Test 2: +1g in X-axis", 8192, 0, 0);

    // Test 3: Pure negative X acceleration (-1g in X direction)
    test_case("Test 3: -1g in X-axis", -8192, 0, 0);

    // Test 4: Pure positive Y acceleration (1g in Y direction)
    test_case("Test 4: +1g in Y-axis", 0, 8192, 0);

    // Test 5: Pure negative Y acceleration (-1g in Y direction)
    test_case("Test 5: -1g in Y-axis", 0, -8192, 0);

    // Test 6: Upside down (Z-down)
    test_case("Test 6: Upside down (Z-down)", 0, 0, -8192);

    // Test 7: 45° in X-Y plane
    test_case("Test 7: 45° in X-Y plane", 0, 5792, 5792);

    // Test 8: 30° tilt
    int16_t ax_30deg = (int16_t)(8192 * sin(30.0 * M_PI / 180.0));
    int16_t az_30deg = (int16_t)(8192 * cos(30.0 * M_PI / 180.0));
    test_case("Test 8: 30° tilt in X-Z plane", ax_30deg, 0, az_30deg);

    printf("\n================================================================================\n");
    printf("Based on these results, we can determine the coordinate frame convention.\n");
    printf("================================================================================\n");

    return 0;
}
