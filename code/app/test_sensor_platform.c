/*******************************************************************************
 * Sensor Platform Configuration Test
 *
 * This program tests the multi-platform sensor specification header.
 * Compile with different flags to test each platform:
 *
 *   gcc -DSENSOR_PLATFORM_INVENSENSE test_sensor_platform.c -o test_invensense
 *   gcc -DSENSOR_PLATFORM_FREESCALE test_sensor_platform.c -o test_freescale
 *
 * Author: Vikas Yadav
 * Date: 2025-10-12
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <math.h>

// Uncomment one of these to test platform-specific builds:
// #define SENSOR_PLATFORM_INVENSENSE
// #define SENSOR_PLATFORM_FREESCALE

#include "inc/sensor_spec_agm.h"

/*******************************************************************************
 * Print Platform Information
 ******************************************************************************/
void print_platform_info(void) {
    printf("==============================================================================\n");
    printf(" SENSOR PLATFORM CONFIGURATION\n");
    printf("==============================================================================\n");
    printf("Platform Name:     %s\n", SENSOR_PLATFORM_NAME);
    printf("Platform Type:     %s\n", SENSOR_PLATFORM_TYPE);
    printf("Manufacturer:      %s\n", SENSOR_PLATFORM_MANUFACTURER);
    printf("Spec Version:      %s\n", SENSOR_SPEC_VERSION);
    printf("Spec Date:         %s\n", SENSOR_SPEC_DATE);
    printf("\n");
}

/*******************************************************************************
 * Print Sensor Specifications
 ******************************************************************************/
void print_sensor_specs(void) {
    printf("==============================================================================\n");
    printf(" SENSOR SPECIFICATIONS\n");
    printf("==============================================================================\n");

    printf("\nACCELEROMETER:\n");
    printf("  Full-Scale Range:    ±%.1f g\n", ACCEL_RANGE_G);
    printf("  Scale Factor:        %.10f g/count\n", ACCEL_FGPERCOUNT);
    printf("  Sensitivity:         %.2f counts/g\n", ACCEL_COUNTSPERG);
    printf("  m/s² per count:      %.10f\n", ACCEL_MPSPERCOUNT);

    printf("\nGYROSCOPE:\n");
    printf("  Full-Scale Range:    ±%.1f dps\n", GYRO_RANGE_DPS);
    printf("  Scale Factor:        %.10f dps/count\n", GYRO_FDPSPERCOUNT);
    printf("  Sensitivity:         %.2f counts/dps\n", GYRO_COUNTSPERDPS);
    printf("  rad/s per count:     %.10f\n", GYRO_RADSPERCOUNT);

    printf("\nMAGNETOMETER:\n");
    printf("  Full-Scale Range:    ±%.1f µT\n", MAG_RANGE_UT);
    printf("  Scale Factor:        %.10f µT/count\n", MAG_FUTPERCOUNT);
    printf("  Sensitivity:         %.4f counts/µT\n", MAG_COUNTSPERUT);

    printf("\nSAMPLING CONFIGURATION:\n");
    printf("  Sample Rate:         %d Hz\n", SAMPLE_RATE_HZ);
    printf("  Fusion Rate:         %d Hz\n", FUSION_RATE_HZ);
    printf("  Standard Gravity:    %.5f m/s²\n", GRAVITY_MPS2);
    printf("\n");
}

/*******************************************************************************
 * Print Platform-Specific Details
 ******************************************************************************/
void print_platform_specific_details(void) {
    printf("==============================================================================\n");
    printf(" PLATFORM-SPECIFIC DETAILS\n");
    printf("==============================================================================\n");

#ifdef SENSOR_PLATFORM_INVENSENSE
    printf("\n*** INVENSENSE MPU-9250 CONFIGURATION ***\n\n");

    printf("Accelerometer (MPU-9250):\n");
    printf("  16-bit resolution, Max Count: %d\n", MPU9250_ACCEL_MAX_COUNT);
    printf("  Sensitivity (±4g):  %.1f LSB/g\n", MPU9250_ACCEL_SENSITIVITY_4G);
    printf("  Noise Density:      %.1f µg/√Hz\n", MPU9250_ACCEL_NOISE_DENSITY);

    printf("\nGyroscope (MPU-9250):\n");
    printf("  16-bit resolution, Max Count: %d\n", MPU9250_GYRO_MAX_COUNT);
    printf("  Sensitivity (±1000dps): %.1f LSB/dps\n", MPU9250_GYRO_SENSITIVITY_1000DPS);
    printf("  Noise Density:      %.3f dps/√Hz\n", MPU9250_GYRO_NOISE_DENSITY);

    printf("\nMagnetometer (AK8963):\n");
    printf("  16-bit resolution, Max Count: %d\n", AK8963_MAG_MAX_COUNT);
    printf("  Sensitivity (16-bit): %.2f µT/LSB\n", AK8963_MAG_SENSITIVITY_16BIT);
    printf("  Noise:              %.1f µT RMS\n", AK8963_MAG_NOISE);

#elif defined(SENSOR_PLATFORM_FREESCALE)
    printf("\n*** FREESCALE FRDM-STBC-AGM01 CONFIGURATION ***\n\n");

    printf("Accelerometer (FXOS8700CQ):\n");
    printf("  14-bit resolution, Max Count: %d\n", FXOS8700_ACCEL_MAX_COUNT);
    printf("  Sensitivity (±4g):  %.1f counts/g\n", FXOS8700_ACCEL_SENSITIVITY_4G);
    printf("  Noise Density:      %.1f µg/√Hz\n", FXOS8700_ACCEL_NOISE_DENSITY);

    printf("\nGyroscope (FXAS21000):\n");
    printf("  16-bit resolution, Max Count: %d\n", FXAS21000_GYRO_MAX_COUNT);
    printf("  Sensitivity (±1000dps): %.1f LSB/dps\n", FXAS21000_GYRO_SENSITIVITY_1000DPS);
    printf("  Noise Density:      %.3f dps/√Hz\n", FXAS21000_GYRO_NOISE_DENSITY);

    printf("\nMagnetometer (FXOS8700CQ):\n");
    printf("  16-bit resolution, Max Count: %d\n", FXOS8700_MAG_MAX_COUNT);
    printf("  Sensitivity:        %.1f µT/LSB\n", FXOS8700_MAG_SENSITIVITY);
    printf("  Noise:              %.1f µT RMS\n", FXOS8700_MAG_NOISE);
#endif

    printf("\n");
}

/*******************************************************************************
 * Test Conversion Functions
 ******************************************************************************/
void test_conversions(void) {
    printf("==============================================================================\n");
    printf(" CONVERSION TESTS\n");
    printf("==============================================================================\n");

    // Test with typical "level" orientation: Z-axis pointing up = +1g
    // For both platforms at ±4g range, we expect different raw values due to different resolutions

    printf("\nTest Case: Sensor at rest, Z-axis up (+1g gravity)\n\n");

#ifdef SENSOR_PLATFORM_INVENSENSE
    // MPU9250: 16-bit, ±4g range, sensitivity = 8192 LSB/g
    int16_t test_accel_raw_z = 8192;   // Should be +1g
    printf("  INVENSENSE (16-bit, 8192 LSB/g):\n");
#elif defined(SENSOR_PLATFORM_FREESCALE)
    // FXOS8700: 14-bit, ±4g range, sensitivity = 2048 counts/g
    int16_t test_accel_raw_z = 2048;   // Should be +1g
    printf("  FREESCALE (14-bit, 2048 counts/g):\n");
#endif

    float accel_g = test_accel_raw_z * ACCEL_FGPERCOUNT;
    float accel_mps2 = test_accel_raw_z * ACCEL_MPSPERCOUNT;

    printf("    Raw Count:     %d\n", test_accel_raw_z);
    printf("    Converted (g): %.6f g\n", accel_g);
    printf("    Converted:     %.6f m/s²\n", accel_mps2);
    printf("    Expected:      1.000000 g = 9.806650 m/s²\n");
    printf("    Error:         %.6f g\n", fabs(accel_g - 1.0f));

    // Test gyroscope (both platforms have 16-bit gyro)
    printf("\nTest Case: Gyroscope at rest (0 dps)\n\n");
    int16_t test_gyro_raw = 0;
    float gyro_dps = test_gyro_raw * GYRO_FDPSPERCOUNT;
    float gyro_rads = test_gyro_raw * GYRO_RADSPERCOUNT;

#ifdef SENSOR_PLATFORM_INVENSENSE
    printf("  INVENSENSE (32.8 LSB/dps):\n");
#elif defined(SENSOR_PLATFORM_FREESCALE)
    printf("  FREESCALE (32.0 LSB/dps):\n");
#endif

    printf("    Raw Count:       %d\n", test_gyro_raw);
    printf("    Converted:       %.6f dps\n", gyro_dps);
    printf("    Converted:       %.6f rad/s\n", gyro_rads);
    printf("    Expected:        0.000000 dps\n");

    // Test magnetometer
    printf("\nTest Case: Typical Earth's magnetic field (~50 µT)\n\n");

#ifdef SENSOR_PLATFORM_INVENSENSE
    // AK8963: ±4800 µT range
    int16_t test_mag_raw = 10000;  // Example value
    printf("  INVENSENSE AK8963 (±4800 µT range):\n");
#elif defined(SENSOR_PLATFORM_FREESCALE)
    // FXOS8700: ±1200 µT range
    int16_t test_mag_raw = 10000;  // Example value
    printf("  FREESCALE FXOS8700 (±1200 µT range):\n");
#endif

    float mag_ut = test_mag_raw * MAG_FUTPERCOUNT;
    printf("    Raw Count:       %d\n", test_mag_raw);
    printf("    Converted:       %.3f µT\n", mag_ut);

    printf("\n");
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(int argc, char *argv[]) {
    printf("\n");
    print_platform_info();
    print_sensor_specs();
    print_platform_specific_details();
    test_conversions();

    printf("==============================================================================\n");
    printf(" TEST COMPLETE\n");
    printf("==============================================================================\n\n");

    return 0;
}
