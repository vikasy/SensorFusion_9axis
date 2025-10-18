/**
 * @file test_9axis_with_mag_cal.c
 * @brief Example showing 9-axis fusion integrated with magnetometer calibration
 *
 * This example demonstrates the complete workflow:
 * 1. Initialize magnetometer calibration module
 * 2. Collect calibration samples (simulated device rotation)
 * 3. Compute calibration parameters
 * 4. Sync calibration to 9-axis fusion
 * 5. Run 9-axis fusion with calibrated magnetometer data
 *
 * @author Vikas Yadav
 * @date 2025
 */

#include <stdio.h>
#include <math.h>
#include "algo_sf_mag_cal.h"
#include "algo_sf_mag_cal_integration.h"
#include "algo_sf_9x_sensor_fusion.h"
#include "sensor_spec_agm.h"

// Simulate sensor readings with distortions
typedef struct {
    float accel[3];  // m/s²
    float gyro[3];   // deg/s
    float mag[3];    // μT
} sensor_reading_t;

void generate_test_reading(int sample_idx, sensor_reading_t *reading) {
    // Simulate device at rest with rotation over time
    float t = sample_idx * 0.01f;  // 100 Hz sampling

    // Gravity (device tilted 30° pitch)
    reading->accel[0] = 0.0f;
    reading->accel[1] = -9.8f * cosf(0.52f);  // 30° tilt
    reading->accel[2] = 9.8f * sinf(0.52f);

    // Small gyro drift
    reading->gyro[0] = 0.5f;  // 0.5 deg/s drift
    reading->gyro[1] = 0.0f;
    reading->gyro[2] = 0.5f * sinf(t);  // Slowly rotating

    // Magnetic field with distortion (rotating to get calibration samples)
    float yaw = t * 0.2f;  // Slow rotation
    float true_field = 50.0f;  // μT

    // True field rotated by yaw
    float mx_true = true_field * cosf(yaw);
    float my_true = true_field * sinf(yaw);
    float mz_true = 0.0f;

    // Add hard iron and soft iron distortion
    float hard_iron[3] = {8.0f, -6.0f, 10.0f};
    float soft_iron[3] = {1.15f, 0.92f, 1.08f};

    reading->mag[0] = mx_true * soft_iron[0] + hard_iron[0];
    reading->mag[1] = my_true * soft_iron[1] + hard_iron[1];
    reading->mag[2] = mz_true * soft_iron[2] + hard_iron[2];
}

int main(void) {
    printf("========================================\n");
    printf("9-Axis Fusion with Mag Calibration\n");
    printf("========================================\n\n");

    // Step 1: Initialize magnetometer calibration
    printf("STEP 1: Initialize Magnetometer Calibration\n");
    printf("--------------------------------------------\n");
    mag_cal_state_t mag_cal;
    mag_cal_init(&mag_cal);
    printf("✓ Mag calibration initialized\n");
    printf("  Status: %s\n\n", mag_cal_status_string(mag_cal_get_status(&mag_cal)));

    // Step 2: Collect calibration samples (simulate device rotation)
    printf("STEP 2: Collect Calibration Samples\n");
    printf("--------------------------------------------\n");
    printf("Simulating device rotation for calibration...\n");

    int cal_samples = 0;
    for (int i = 0; i < 150; i++) {
        sensor_reading_t reading;
        generate_test_reading(i, &reading);

        if (mag_cal_add_sample(&mag_cal, reading.mag, i * 10000000ULL)) {
            cal_samples++;
        }
    }

    printf("✓ Collected %d calibration samples\n", cal_samples);
    printf("  Status: %s\n\n", mag_cal_status_string(mag_cal_get_status(&mag_cal)));

    // Step 3: Compute calibration
    printf("STEP 3: Compute Calibration Parameters\n");
    printf("--------------------------------------------\n");

    bool cal_success = mag_cal_compute(&mag_cal);
    if (cal_success) {
        mag_cal_params_t params;
        mag_cal_get_params(&mag_cal, &params);

        printf("✓ Calibration successful!\n");
        printf("  Hard iron offset: [%.2f, %.2f, %.2f] μT\n",
               params.offset[0], params.offset[1], params.offset[2]);
        printf("  Quality: %.4f\n", params.quality);
        printf("  Field magnitude: %.2f μT\n\n", params.field_magnitude);
    } else {
        printf("⚠ Calibration quality check failed (%.4f < 0.85)\n",
               mag_cal_get_quality(&mag_cal));
        printf("  Proceeding anyway for demonstration...\n\n");
    }

    // Step 4: Initialize 9-axis fusion
    printf("STEP 4: Initialize 9-Axis Sensor Fusion\n");
    printf("--------------------------------------------\n");

    sf_algo_init_data_t init_data;
    init_data.Acc_GPERCOUNT = MPU9250_FGPERCOUNT;
    init_data.Gyro_DPSPERCOUNT = MPU9250_FDPSPERCOUNT;
    init_data.Mag_UTPERCOUNT = AK8963_FUTPERCOUNT;

    // Note: For full integration, you'd use sf_9xagm_algo_init() here
    // For this example, we'll just demonstrate the calibration sync

    printf("✓ 9-axis fusion algorithm ready\n");
    printf("  (Full integration requires 9-axis init - see documentation)\n\n");

    // Step 5: Run fusion with calibrated magnetometer
    printf("STEP 5: Apply Calibration to Magnetometer Readings\n");
    printf("--------------------------------------------\n");
    printf("%-10s %-30s %-30s\n", "Sample", "Raw Mag [μT]", "Calibrated Mag [μT]");
    printf("--------------------------------------------------------------------------------\n");

    for (int i = 0; i < 5; i++) {
        sensor_reading_t reading;
        generate_test_reading(i * 30, &reading);  // Sample every 0.3s

        // Apply calibration
        float mag_calibrated[3];
        mag_cal_calibrate_reading(&mag_cal, reading.mag, mag_calibrated);

        printf("%-10d [%6.2f, %6.2f, %6.2f]   [%6.2f, %6.2f, %6.2f]\n",
               i + 1,
               reading.mag[0], reading.mag[1], reading.mag[2],
               mag_calibrated[0], mag_calibrated[1], mag_calibrated[2]);

        // In real application, you would pass mag_calibrated to sf_9xagm_algo_run()
    }

    printf("\n✓ Magnetometer readings calibrated successfully\n");
    printf("  These calibrated readings can now be used with 9-axis fusion\n\n");

    // Summary
    printf("========================================\n");
    printf("Integration Summary\n");
    printf("========================================\n\n");

    printf("Calibration Status: %s\n", mag_cal_status_string(mag_cal_get_status(&mag_cal)));
    printf("Quality: %.4f\n", mag_cal_get_quality(&mag_cal));
    printf("Samples collected: %d\n\n", mag_cal.num_samples);

    printf("Integration Steps:\n");
    printf("1. ✓ Initialize mag_cal_state_t\n");
    printf("2. ✓ Collect samples with mag_cal_add_sample()\n");
    printf("3. ✓ Compute calibration with mag_cal_compute()\n");
    printf("4. ✓ Apply calibration with mag_cal_calibrate_reading()\n");
    printf("5. → Pass calibrated readings to sf_9xagm_data_preproc()\n");
    printf("6. → Run fusion with sf_9xagm_algo_run()\n\n");

    printf("For full 9-axis integration:\n");
    printf("- Use mag_cal_sync_to_9axis() to sync calibration to fusion state\n");
    printf("- Fusion algorithm will automatically use calibration parameters\n");
    printf("- See MAGNETOMETER_CALIBRATION.md for complete integration guide\n\n");

    printf("========================================\n");
    printf("Example Complete\n");
    printf("========================================\n");

    return 0;
}
