#ifndef SENSOR_SPEC_AGM_H
#define SENSOR_SPEC_AGM_H

/*******************************************************************************
 * MPU9250 9-Axis IMU Sensor Specifications
 *
 * Author: Vikas Yadav
 * Date: 2025-10-11
 *
 * Complete sensor specifications for InvenSense MPU-9250
 * 9-axis Motion Tracking Device (Accelerometer + Gyroscope + Magnetometer)
 ******************************************************************************/

/*******************************************************************************
 * SENSOR OVERVIEW
 ******************************************************************************/
#define MPU9250_SENSOR_NAME           "InvenSense MPU-9250"
#define MPU9250_SENSOR_TYPE           "9-Axis MEMS IMU"
#define MPU9250_MANUFACTURER          "InvenSense (TDK)"

/*******************************************************************************
 * ACCELEROMETER SPECIFICATIONS
 ******************************************************************************/

// Full-Scale Range Options: ±2g, ±4g, ±8g, ±16g
// Current Configuration: ±4g
#define MPU9250_ACCEL_FSR_2G          2.0F
#define MPU9250_ACCEL_FSR_4G          4.0F      // Current configuration
#define MPU9250_ACCEL_FSR_8G          8.0F
#define MPU9250_ACCEL_FSR_16G         16.0F

// Selected Accelerometer Range
#define MPU9250_ACC_RANGE_G           MPU9250_ACCEL_FSR_4G  // ±4g full-scale range

// 16-bit ADC Resolution (signed: -32768 to +32767)
#define MPU9250_ACCEL_BITS            16
#define MPU9250_ACCEL_MAX_COUNT       32767

// Scale Factors for ±4g range (matching main.h naming)
#define MPU9250_FGPERCOUNT            (MPU9250_ACC_RANGE_G / MPU9250_ACCEL_MAX_COUNT)  // g per count (±4g / 32767 counts)
#define MPU9250_COUNTSPERG            (1.0F / MPU9250_FGPERCOUNT)   // counts per g (1 / 0.0001220703125)
#define MPU9250_ACCEL_MPSPERCOUNT     (MPU9250_FGPERCOUNT * 9.81F)  // m/s^2 per count (9.81 * 0.0001220703125)

// Accelerometer Sensitivity (from datasheet)
#define MPU9250_ACCEL_SENSITIVITY_2G  16384.0F   // LSB/g
#define MPU9250_ACCEL_SENSITIVITY_4G  8192.0F    // LSB/g (current)
#define MPU9250_ACCEL_SENSITIVITY_8G  4096.0F    // LSB/g
#define MPU9250_ACCEL_SENSITIVITY_16G 2048.0F    // LSB/g

// Accelerometer Performance (from datasheet)
#define MPU9250_ACCEL_NOISE_DENSITY   300.0F     // µg/√Hz
#define MPU9250_ACCEL_ZERO_G_OFFSET   80.0F      // mg (typical)
#define MPU9250_ACCEL_LINEARITY       0.5F       // % of FS (best fit straight line)

/*******************************************************************************
 * GYROSCOPE SPECIFICATIONS
 ******************************************************************************/

// Full-Scale Range Options: ±250, ±500, ±1000, ±2000 dps
// Current Configuration: ±1000 dps
#define MPU9250_GYRO_FSR_250DPS       250.0F
#define MPU9250_GYRO_FSR_500DPS       500.0F
#define MPU9250_GYRO_FSR_1000DPS      1000.0F   // Current configuration
#define MPU9250_GYRO_FSR_2000DPS      2000.0F

// Selected Gyroscope Range
#define MPU9250_GYRO_RANGE_DPS        MPU9250_GYRO_FSR_1000DPS  // ±1000dps full-scale range

// 16-bit ADC Resolution (signed: -32768 to +32767)
#define MPU9250_GYRO_BITS             16
#define MPU9250_GYRO_MAX_COUNT        32767

// Scale Factors for ±1000 dps range (matching main.h naming)
#define MPU9250_FDPSPERCOUNT          (MPU9250_GYRO_RANGE_DPS / MPU9250_GYRO_MAX_COUNT)   // dps per count (±1000dps / 32767 counts =0.030487804878)
#define MPU9250_COUNTSPERDPS          (1.0F / MPU9250_FDPSPERCOUNT)   // counts per dps (1 / 0.030487804878)
#define MPU9250_GYRO_RADSPERCOUNT     (MPU9250_FDPSPERCOUNT * M_PI / 180.0F)  // rad/s per count

// Gyroscope Sensitivity (from datasheet)
#define MPU9250_GYRO_SENSITIVITY_250DPS   131.0F    // LSB/(dps)
#define MPU9250_GYRO_SENSITIVITY_500DPS   65.5F     // LSB/(dps)
#define MPU9250_GYRO_SENSITIVITY_1000DPS  32.8F     // LSB/(dps) (current)
#define MPU9250_GYRO_SENSITIVITY_2000DPS  16.4F     // LSB/(dps)

// Gyroscope Performance (from datasheet)
#define MPU9250_GYRO_NOISE_DENSITY    0.01F          // dps/√Hz
#define MPU9250_GYRO_ZERO_RATE_OFFSET 20.0F          // dps (typical)
#define MPU9250_GYRO_LINEARITY        0.1F           // % of FS (best fit straight line)

/*******************************************************************************
 * MAGNETOMETER SPECIFICATIONS (AK8963 Compass)
 ******************************************************************************/

// Full-Scale Range: ±4800 µT (fixed)
#define AK8963_MAG_RANGE_UT          4800.0F   // ±4800 µT full-scale range

// 16-bit ADC Resolution (signed: -32768 to +32767)
// Note: AK8963 uses 14-bit mode (default) or 16-bit mode
#define AK8963_MAG_BITS              16
#define AK8963_MAG_MAX_COUNT         32767

// Scale Factors for ±4800 µT range (16-bit mode) - matching main.h naming
#define AK8963_FUTPERCOUNT           (AK8963_MAG_RANGE_UT / AK8963_MAG_MAX_COUNT)  // µT per count (±4800µT / 32767 counts)
#define AK8963_COUNTSPERUT           (1.0F / AK8963_FUTPERCOUNT)   // counts per µT (1 / 0.146489)

// Magnetometer Sensitivity
#define AK8963_MAG_SENSITIVITY_14BIT 0.6F              // µT/LSB
#define AK8963_MAG_SENSITIVITY_16BIT 0.15F             // µT/LSB (current)

// Magnetometer Performance (from AK8963 datasheet)
#define AK8963_MAG_NOISE             0.6F              // µT RMS (typ)  
#define AK8963_MAG_MEASUREMENT_RANGE 4912.0F           // µT (±4912 µT)

/*******************************************************************************
 * SAMPLING RATE CONFIGURATION
 ******************************************************************************/

// Gyroscope Output Data Rate (ODR)
#define MPU9250_GYRO_ODR_HZ           100               // 100 Hz (configurable: 4 Hz - 8 kHz)

// Accelerometer Output Data Rate (ODR)
#define MPU9250_ACCEL_ODR_HZ          100               // 100 Hz (same as gyro when synchronized)

// Magnetometer Output Data Rate (AK8963)
#define AK8963_MAG_ODR_HZ            100               // 100 Hz (configurable: 8 Hz, 100 Hz)

// Effective Sampling Rates Used in Algorithm
#define MPU9250_SAMPLE_RATE_HZ        100               // Primary sampling rate
#define MPU9250_FUSION_RATE_HZ        25                // Kalman filter update rate

// Sampling Period
#define MPU9250_SAMPLE_PERIOD_MS      10                // ms (1000 / 100)
#define MPU9250_SAMPLE_PERIOD_US      10000             // µs
#define MPU9250_SAMPLE_PERIOD_NS      10000000LL        // ns

/*******************************************************************************
 * OPERATING CONDITIONS
 ******************************************************************************/

// Operating Temperature Range
#define MPU9250_TEMP_MIN_C            -40               // °C
#define MPU9250_TEMP_MAX_C            85                // °C
#define MPU9250_TEMP_TYPICAL_C        25                // °C (for specs)

// Power Supply
#define MPU9250_VDD_MIN_V             2.4F              // V
#define MPU9250_VDD_MAX_V             3.6F              // V
#define MPU9250_VDD_TYPICAL_V         3.3F              // V

/*******************************************************************************
 * ORIENTATION AND AXIS CONVENTION
 ******************************************************************************/

// Coordinate System: Right-handed, NED (North-East-Down) convention
// X-axis: Forward (North)
// Y-axis: Right (East)
// Z-axis: Down

// Gravity Vector (standard)
#define MPU9250_GRAVITY_MPS2          9.80665F          // m/s^2 (standard gravity)

// Earth's Magnetic Field (typical values)
#define AK8963_EARTH_MAG_MIN_UT      25.0F             // µT (equator)
#define AK8963_EARTH_MAG_MAX_UT      65.0F             // µT (poles)
#define AK8963_EARTH_MAG_TYPICAL_UT  50.0F             // µT (mid-latitudes)

/*******************************************************************************
 * CONVERSION MACROS
 ******************************************************************************/

// Accelerometer conversions
#define MPU9250_ACCEL_RAW_TO_G(raw)       ((float)(raw) * MPU9250_ACCEL_GPERCOUNT)
#define MPU9250_ACCEL_RAW_TO_MPS2(raw)    ((float)(raw) * MPU9250_ACCEL_MPSPERCOUNT)
#define MPU9250_ACCEL_G_TO_RAW(g)         ((int16_t)((g) * MPU9250_ACCEL_COUNTSPERG))

// Gyroscope conversions
#define MPU9250_GYRO_RAW_TO_DPS(raw)      ((float)(raw) * MPU9250_GYRO_DPSPERCOUNT)
#define MPU9250_GYRO_RAW_TO_RADS(raw)     ((float)(raw) * MPU9250_GYRO_RADSPERCOUNT)
#define MPU9250_GYRO_DPS_TO_RAW(dps)      ((int16_t)((dps) * MPU9250_GYRO_COUNTSPERDPS))

// Magnetometer conversions
#define AK8963_MAG_RAW_TO_UT(raw)        ((float)(raw) * AK8963_MAG_UTPERCOUNT)
#define AK8963_MAG_UT_TO_RAW(ut)         ((int16_t)((ut) * AK8963_MAG_COUNTSPERUT))

// Angular conversions
#define MPU9250_DEG_TO_RAD(deg)           ((deg) * M_PI / 180.0F)  // π/180
#define MPU9250_RAD_TO_DEG(rad)           ((rad) * 180.0F / M_PI)    // 180/π

/*******************************************************************************
 * EXPECTED SENSOR VALUES FOR COMMON ORIENTATIONS
 ******************************************************************************/

// Level (flat on table, Z-axis up)
// Accel: [0, 0, +1g] or [0, 0, +9.81 m/s²]
// Gyro:  [0, 0, 0] dps
// Mag:   Depends on orientation, typically horizontal component of Earth's field

// Upside down (Z-axis down)
// Accel: [0, 0, -1g] or [0, 0, -9.81 m/s²]
// Gyro:  [0, 0, 0] dps

// On side (X or Y axis vertical)
// Accel: [±1g, 0, 0] or [0, ±1g, 0]
// Gyro:  [0, 0, 0] dps

/*******************************************************************************
 * VALIDATION RANGES
 ******************************************************************************/

// Expected sensor ranges during normal operation (at rest, room temperature)
#define MPU9250_ACCEL_EXPECTED_MIN_G      -1.2F         // g (should be close to ±1g when at rest)
#define MPU9250_ACCEL_EXPECTED_MAX_G      1.2F          // g
#define MPU9250_GYRO_EXPECTED_DRIFT_DPS   5.0F          // dps (typical drift at rest)
#define AK8963_MAG_EXPECTED_MIN_UT        20.0F         // µT
#define AK8963_MAG_EXPECTED_MAX_UT        70.0F         // µT

#endif /* SENSOR_SPEC_AGM_H */
