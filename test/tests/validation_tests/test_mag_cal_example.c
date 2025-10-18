/**
 * @file test_mag_cal_example.c
 * @brief Example usage of magnetometer calibration module
 *
 * This example demonstrates:
 * 1. Initialize calibration module
 * 2. Add magnetometer samples (simulating device rotation)
 * 3. Compute calibration parameters
 * 4. Apply calibration to raw data
 * 5. Verify calibration quality
 *
 * @author Vikas Yadav
 * @date 2025
 */

#include <stdio.h>
#include <math.h>
#include "algo_sf_mag_cal.h"

// Simulate magnetometer data with hard iron and soft iron distortion
void generate_test_samples(mag_cal_state_t *state) {
    printf("Generating test magnetometer samples...\n");
    printf("(Simulating device rotation in Earth's magnetic field)\n\n");

    // True magnetic field parameters
    const float true_field_strength = 50.0f;  // μT (typical Earth field)

    // Distortion parameters (what we want to calibrate out)
    const float hard_iron[3] = {10.0f, -5.0f, 8.0f};  // Constant offset
    const float soft_iron[3] = {1.2f, 0.9f, 1.1f};    // Axis scaling

    // Generate samples by rotating device through different orientations
    int num_orientations = 100;
    int accepted = 0;

    for (int i = 0; i < num_orientations; i++) {
        // Sample sphere uniformly
        float theta = (float)i * 3.14159f / (num_orientations / 2);  // 0 to π
        float phi = (float)i * 2.0f * 3.14159f / num_orientations;   // 0 to 2π

        // True magnetic field vector (unit sphere * field strength)
        float mx_true = true_field_strength * sinf(theta) * cosf(phi);
        float my_true = true_field_strength * sinf(theta) * sinf(phi);
        float mz_true = true_field_strength * cosf(theta);

        // Apply distortions (simulate raw sensor readings)
        float mag_raw[3];
        mag_raw[0] = mx_true * soft_iron[0] + hard_iron[0];
        mag_raw[1] = my_true * soft_iron[1] + hard_iron[1];
        mag_raw[2] = mz_true * soft_iron[2] + hard_iron[2];

        // Add sample to calibration
        bool added = mag_cal_add_sample(state, mag_raw, i * 1000000ULL);
        if (added) accepted++;
    }

    printf("Added %d / %d samples to calibration\n", accepted, num_orientations);
    printf("True distortion parameters:\n");
    printf("  Hard iron offset: [%.2f, %.2f, %.2f] μT\n",
           hard_iron[0], hard_iron[1], hard_iron[2]);
    printf("  Soft iron scale:  [%.2f, %.2f, %.2f]\n\n",
           soft_iron[0], soft_iron[1], soft_iron[2]);
}

int main(void) {
    printf("========================================\n");
    printf("Magnetometer Calibration Example\n");
    printf("========================================\n\n");

    // Initialize calibration module
    mag_cal_state_t cal_state;
    mag_cal_init(&cal_state);

    printf("Status after init: %d (0=uncalibrated)\n\n", mag_cal_get_status(&cal_state));

    // Generate test samples (simulating device rotation)
    generate_test_samples(&cal_state);

    // Compute calibration
    printf("Computing calibration parameters...\n");
    bool success = mag_cal_compute(&cal_state);

    if (!success) {
        printf("❌ Calibration failed!\n");
        printf("Status: %d\n", mag_cal_get_status(&cal_state));
        return 1;
    }

    printf("✓ Calibration successful!\n\n");

    // Get calibration parameters
    mag_cal_params_t params;
    mag_cal_get_params(&cal_state, &params);

    printf("Calibration Results:\n");
    printf("  Hard iron offset: [%.2f, %.2f, %.2f] μT\n",
           params.offset[0], params.offset[1], params.offset[2]);
    printf("  Soft iron matrix:\n");
    printf("    [%.3f  %.3f  %.3f]\n", params.matrix[0][0], params.matrix[0][1], params.matrix[0][2]);
    printf("    [%.3f  %.3f  %.3f]\n", params.matrix[1][0], params.matrix[1][1], params.matrix[1][2]);
    printf("    [%.3f  %.3f  %.3f]\n", params.matrix[2][0], params.matrix[2][1], params.matrix[2][2]);
    printf("  Field magnitude:  %.2f μT\n", params.field_magnitude);
    printf("  Calibration quality: %.4f (1.0 = perfect)\n", params.quality);
    printf("  Status: %d (2=calibrated)\n\n", params.status);

    // Test calibration by applying to some raw samples
    printf("Testing calibration on sample data:\n");
    printf("%-15s %-25s %-25s %-10s\n", "", "Raw [μT]", "Calibrated [μT]", "Magnitude");
    printf("--------------------------------------------------------------------------------\n");

    // Test samples with known distortion
    float test_samples[5][3] = {
        {50.0f * 1.2f + 10.0f,  0.0f * 0.9f - 5.0f,  0.0f * 1.1f + 8.0f},  // X-axis
        { 0.0f * 1.2f + 10.0f, 50.0f * 0.9f - 5.0f,  0.0f * 1.1f + 8.0f},  // Y-axis
        { 0.0f * 1.2f + 10.0f,  0.0f * 0.9f - 5.0f, 50.0f * 1.1f + 8.0f},  // Z-axis
        {35.4f * 1.2f + 10.0f, 35.4f * 0.9f - 5.0f,  0.0f * 1.1f + 8.0f},  // XY-plane
        {28.9f * 1.2f + 10.0f, 28.9f * 0.9f - 5.0f, 28.9f * 1.1f + 8.0f},  // Diagonal
    };

    for (int i = 0; i < 5; i++) {
        float mag_cal[3];
        mag_cal_apply(&cal_state, test_samples[i], mag_cal);

        float mag_raw = sqrtf(test_samples[i][0]*test_samples[i][0] +
                              test_samples[i][1]*test_samples[i][1] +
                              test_samples[i][2]*test_samples[i][2]);
        float mag_calibrated = sqrtf(mag_cal[0]*mag_cal[0] +
                                      mag_cal[1]*mag_cal[1] +
                                      mag_cal[2]*mag_cal[2]);

        printf("Sample %d:      [%6.2f %6.2f %6.2f]  [%6.2f %6.2f %6.2f]  %.2f\n",
               i+1,
               test_samples[i][0], test_samples[i][1], test_samples[i][2],
               mag_cal[0], mag_cal[1], mag_cal[2],
               mag_calibrated);
    }

    printf("\n✓ After calibration, all samples should have magnitude ≈ 50 μT\n");
    printf("\n========================================\n");
    printf("Calibration Example Complete\n");
    printf("========================================\n");

    return 0;
}
