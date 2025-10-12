/*******************************************************************************
 * Realistic Sensor Fusion Validation
 *
 * Tests the algorithms with realistic small-tilt scenarios (< 20°)
 * as the tilt-from-gravity algorithm is designed for small tilts
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "algo_sf_6x_sensor_fusion.h"
#include "algo_sf_9x_sensor_fusion.h"
#include "algo_sf_sensordata.h"
#include "algo_sf_interface.h"
#include "sensor_spec_agm.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void test_6axis(const char* description, double roll_deg, double pitch_deg) {
    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_6xag_algo_init(&init_data);

    // Calculate accelerometer readings for small tilts
    double roll_rad = roll_deg * M_PI / 180.0;
    double pitch_rad = pitch_deg * M_PI / 180.0;
    double g = MPU9250_GRAVITY_MPS2;

    double accel_x = g * sin(roll_rad) * cos(pitch_rad);
    double accel_y = -g * sin(pitch_rad) * cos(roll_rad);
    double accel_z = g * cos(roll_rad) * cos(pitch_rad);

    int16_t ax = (int16_t)((accel_x / g) * MPU9250_COUNTSPERG);
    int16_t ay = (int16_t)((accel_y / g) * MPU9250_COUNTSPERG);
    int16_t az = (int16_t)((accel_z / g) * MPU9250_COUNTSPERG);

    sensor_data_t accel_data, gyro_data;
    accel_data.sensordata[0] = ax;
    accel_data.sensordata[1] = ay;
    accel_data.sensordata[2] = az;
    accel_data.sensorID = ACC;

    gyro_data.sensordata[0] = 0;
    gyro_data.sensordata[1] = 0;
    gyro_data.sensordata[2] = 0;
    gyro_data.sensorID = GYRO;

    // Run to convergence
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

    double roll_calc = output.orientation[2];
    double pitch_calc = output.orientation[1];

    double roll_error = fabs(roll_calc - roll_deg);
    double pitch_error = fabs(pitch_calc - pitch_deg);

    printf("\n%s\n", description);
    printf("  Expected: Roll=%.2f°, Pitch=%.2f°\n", roll_deg, pitch_deg);
    printf("  Calculated: Roll=%.2f°, Pitch=%.2f°\n", roll_calc, pitch_calc);
    printf("  Error: Roll=%.2f°, Pitch=%.2f°\n", roll_error, pitch_error);

    if (roll_error < 2.0 && pitch_error < 2.0) {
        printf("  ✓ PASS (< 2° tolerance)\n");
    } else if (roll_error < 5.0 && pitch_error < 5.0) {
        printf("  ~ ACCEPTABLE (< 5° tolerance)\n");
    } else {
        printf("  ✗ FAIL\n");
    }

    sf_6xag_algo_stop(algo_id);
}

void test_9axis(const char* description, double roll_deg, double pitch_deg, double yaw_deg) {
    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    uintptr_t algo_id = sf_9xagm_algo_init(&init_data);

    // Calculate accelerometer
    double roll_rad = roll_deg * M_PI / 180.0;
    double pitch_rad = pitch_deg * M_PI / 180.0;
    double yaw_rad = yaw_deg * M_PI / 180.0;
    double g = MPU9250_GRAVITY_MPS2;

    double accel_x = g * sin(roll_rad) * cos(pitch_rad);
    double accel_y = -g * sin(pitch_rad) * cos(roll_rad);
    double accel_z = g * cos(roll_rad) * cos(pitch_rad);

    // Calculate magnetometer (assuming horizontal Earth field pointing North)
    double mag_field = AK8963_EARTH_MAG_TYPICAL_UT;
    double mag_x = mag_field * cos(yaw_rad);
    double mag_y = mag_field * sin(yaw_rad);
    double mag_z = 0.0;

    int16_t ax = (int16_t)((accel_x / g) * MPU9250_COUNTSPERG);
    int16_t ay = (int16_t)((accel_y / g) * MPU9250_COUNTSPERG);
    int16_t az = (int16_t)((accel_z / g) * MPU9250_COUNTSPERG);

    int16_t mx = (int16_t)(mag_x * AK8963_COUNTSPERUT);
    int16_t my = (int16_t)(mag_y * AK8963_COUNTSPERUT);
    int16_t mz = (int16_t)(mag_z * AK8963_COUNTSPERUT);

    sensor_data_t accel_data, gyro_data, mag_data;
    accel_data.sensordata[0] = ax;
    accel_data.sensordata[1] = ay;
    accel_data.sensordata[2] = az;
    accel_data.sensorID = ACC;

    gyro_data.sensordata[0] = 0;
    gyro_data.sensordata[1] = 0;
    gyro_data.sensordata[2] = 0;
    gyro_data.sensorID = GYRO;

    mag_data.sensordata[0] = mx;
    mag_data.sensordata[1] = my;
    mag_data.sensordata[2] = mz;
    mag_data.sensorID = MAG;

    // Run to convergence
    for (int i = 0; i < 200; i++) {
        accel_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        gyro_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;
        mag_data.timestamp = i * MPU9250_SAMPLE_PERIOD_NS;

        sf_9xagm_data_preproc(algo_id, &accel_data);
        sf_9xagm_data_preproc(algo_id, &gyro_data);
        sf_9xagm_data_preproc(algo_id, &mag_data);

        sf_algo_output_t output;
        memset(&output, 0, sizeof(sf_algo_output_t));
        sf_9xagm_algo_run(algo_id, &output);
    }

    // Get final output
    sf_algo_output_t output;
    memset(&output, 0, sizeof(sf_algo_output_t));
    sf_9xagm_algo_run(algo_id, &output);

    double yaw_calc = output.orientation[0];
    double pitch_calc = output.orientation[1];
    double roll_calc = output.orientation[2];

    // Normalize yaw to [0, 360)
    while (yaw_calc < 0) yaw_calc += 360.0;
    while (yaw_calc >= 360) yaw_calc -= 360.0;

    double roll_error = fabs(roll_calc - roll_deg);
    double pitch_error = fabs(pitch_calc - pitch_deg);
    double yaw_error = fabs(yaw_calc - yaw_deg);
    if (yaw_error > 180) yaw_error = 360 - yaw_error;  // Handle wraparound

    printf("\n%s\n", description);
    printf("  Expected: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n", roll_deg, pitch_deg, yaw_deg);
    printf("  Calculated: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n", roll_calc, pitch_calc, yaw_calc);
    printf("  Error: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n", roll_error, pitch_error, yaw_error);

    if (roll_error < 2.0 && pitch_error < 2.0 && yaw_error < 5.0) {
        printf("  ✓ PASS\n");
    } else if (roll_error < 5.0 && pitch_error < 5.0 && yaw_error < 10.0) {
        printf("  ~ ACCEPTABLE\n");
    } else {
        printf("  ✗ FAIL\n");
    }

    sf_9xagm_algo_stop(algo_id);
}

int main(void) {
    printf("================================================================================\n");
    printf("  Realistic Sensor Fusion Validation\n");
    printf("  Testing small-tilt scenarios (< 20°) where tilt-from-gravity works best\n");
    printf("================================================================================\n");

    printf("\n=== 6-Axis Fusion Tests ===\n");
    test_6axis("6-Axis: Level (0°, 0°)", 0.0, 0.0);
    test_6axis("6-Axis: Small roll (5°)", 5.0, 0.0);
    test_6axis("6-Axis: Small pitch (5°)", 0.0, 5.0);
    test_6axis("6-Axis: Medium roll (10°)", 10.0, 0.0);
    test_6axis("6-Axis: Medium pitch (10°)", 0.0, 10.0);
    test_6axis("6-Axis: Combined (10°, 10°)", 10.0, 10.0);
    test_6axis("6-Axis: Moderate roll (15°)", 15.0, 0.0);

    printf("\n=== 9-Axis Fusion Tests ===\n");
    test_9axis("9-Axis: Level North (0°, 0°, 0°)", 0.0, 0.0, 0.0);
    test_9axis("9-Axis: Level East (0°, 0°, 90°)", 0.0, 0.0, 90.0);
    test_9axis("9-Axis: Level South (0°, 0°, 180°)", 0.0, 0.0, 180.0);
    test_9axis("9-Axis: Level West (0°, 0°, 270°)", 0.0, 0.0, 270.0);
    test_9axis("9-Axis: Tilted NE (5°, 5°, 45°)", 5.0, 5.0, 45.0);
    test_9axis("9-Axis: Tilted SW (10°, 10°, 225°)", 10.0, 10.0, 225.0);

    printf("\n================================================================================\n");
    printf("  Validation Complete!\n");
    printf("================================================================================\n");

    return 0;
}
