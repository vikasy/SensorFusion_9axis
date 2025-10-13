#ifndef SENSOR_SPEC_AGM_H
#define SENSOR_SPEC_AGM_H

/*******************************************************************************
 * Multi-Sensor Support: AGM Sensor Specifications
 *
 * Author: Vikas Yadav
 * Date: 2025-10-12
 *
 * Supported sensor platforms:
 *   - SENSOR_PLATFORM_INVENSENSE: MPU-9250 (Accelerometer + Gyroscope + Magnetometer)
 *   - SENSOR_PLATFORM_FREESCALE:  FXOS8700CQ (Accel+Mag) + FXAS21000 (Gyro)
 *
 * Usage: Define one of the following before including this header:
 *   #define SENSOR_PLATFORM_INVENSENSE
 *   or
 *   #define SENSOR_PLATFORM_FREESCALE
 ******************************************************************************/

/*******************************************************************************
 * PLATFORM SELECTION
 ******************************************************************************/
// Default to INVENSENSE if nothing is specified
#if !defined(SENSOR_PLATFORM_INVENSENSE) && !defined(SENSOR_PLATFORM_FREESCALE)
    #define SENSOR_PLATFORM_INVENSENSE
    #warning "No sensor platform defined, defaulting to SENSOR_PLATFORM_INVENSENSE (MPU9250)"
#endif

// Ensure only one platform is selected
#if defined(SENSOR_PLATFORM_INVENSENSE) && defined(SENSOR_PLATFORM_FREESCALE)
    #error "Multiple sensor platforms defined! Please define only one."
#endif

/*******************************************************************************
 * PLATFORM-SPECIFIC SENSOR CONFIGURATIONS
 ******************************************************************************/

#ifdef SENSOR_PLATFORM_INVENSENSE
/*==============================================================================
 * INVENSENSE MPU-9250 Platform Configuration
 *============================================================================*/

/*******************************************************************************
 * SENSOR OVERVIEW
 ******************************************************************************/
#define SENSOR_PLATFORM_NAME          "InvenSense MPU-9250"
#define SENSOR_PLATFORM_TYPE          "9-Axis MEMS IMU"
#define SENSOR_PLATFORM_MANUFACTURER  "InvenSense (TDK)"
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

#endif /* SENSOR_PLATFORM_INVENSENSE */

/*==============================================================================*/

#ifdef SENSOR_PLATFORM_FREESCALE
/*==============================================================================
 * FREESCALE FRDM-STBC-AGM01 Platform Configuration
 * FXOS8700CQ: 6-axis Accelerometer + Magnetometer
 * FXAS21000: 3-axis Gyroscope
 *============================================================================*/

/*******************************************************************************
 * SENSOR OVERVIEW
 ******************************************************************************/
#define SENSOR_PLATFORM_NAME          "NXP FRDM-STBC-AGM01"
#define SENSOR_PLATFORM_TYPE          "9-Axis Sensor Shield"
#define SENSOR_PLATFORM_MANUFACTURER  "NXP Semiconductors (Freescale)"

#define FXOS8700CQ_SENSOR_NAME        "FXOS8700CQ"
#define FXOS8700CQ_SENSOR_TYPE        "6-Axis Accel+Mag"
#define FXOS8700CQ_MANUFACTURER       "NXP (Freescale)"

#define FXAS21000_SENSOR_NAME         "FXAS21000"
#define FXAS21000_SENSOR_TYPE         "3-Axis Gyroscope"
#define FXAS21000_MANUFACTURER        "NXP (Freescale)"

/*******************************************************************************
 * ACCELEROMETER SPECIFICATIONS (FXOS8700CQ)
 ******************************************************************************/

// Full-Scale Range Options: ±2g, ±4g, ±8g
// Current Configuration: ±4g
#define FXOS8700_ACCEL_FSR_2G         2.0F
#define FXOS8700_ACCEL_FSR_4G         4.0F      // Current configuration
#define FXOS8700_ACCEL_FSR_8G         8.0F

// Selected Accelerometer Range
#define FXOS8700_ACC_RANGE_G          FXOS8700_ACCEL_FSR_4G  // ±4g full-scale range

// 14-bit ADC Resolution (signed: -8192 to +8191)
#define FXOS8700_ACCEL_BITS           14
#define FXOS8700_ACCEL_MAX_COUNT      8191

// Scale Factors for ±4g range
#define FXOS8700_FGPERCOUNT           (FXOS8700_ACC_RANGE_G / FXOS8700_ACCEL_MAX_COUNT)  // g per count
#define FXOS8700_COUNTSPERG           (1.0F / FXOS8700_FGPERCOUNT)   // counts per g
#define FXOS8700_ACCEL_MPSPERCOUNT    (FXOS8700_FGPERCOUNT * 9.81F)  // m/s^2 per count

// Accelerometer Sensitivity (from datasheet)
#define FXOS8700_ACCEL_SENSITIVITY_2G 4096.0F    // counts/g
#define FXOS8700_ACCEL_SENSITIVITY_4G 2048.0F    // counts/g (current)
#define FXOS8700_ACCEL_SENSITIVITY_8G 1024.0F    // counts/g

// Accelerometer Performance (from datasheet)
#define FXOS8700_ACCEL_NOISE_DENSITY  126.0F     // µg/√Hz (typ)
#define FXOS8700_ACCEL_ZERO_G_OFFSET  40.0F      // mg (typical)
#define FXOS8700_ACCEL_LINEARITY      0.5F       // % of FS

/*******************************************************************************
 * GYROSCOPE SPECIFICATIONS (FXAS21000)
 ******************************************************************************/

// Full-Scale Range Options: ±250, ±500, ±1000, ±2000 dps
// Current Configuration: ±1000 dps
#define FXAS21000_GYRO_FSR_250DPS     250.0F
#define FXAS21000_GYRO_FSR_500DPS     500.0F
#define FXAS21000_GYRO_FSR_1000DPS    1000.0F   // Current configuration
#define FXAS21000_GYRO_FSR_2000DPS    2000.0F

// Selected Gyroscope Range
#define FXAS21000_GYRO_RANGE_DPS      FXAS21000_GYRO_FSR_1000DPS  // ±1000dps full-scale range

// 16-bit ADC Resolution (signed: -32768 to +32767)
#define FXAS21000_GYRO_BITS           16
#define FXAS21000_GYRO_MAX_COUNT      32767

// Scale Factors for ±1000 dps range
#define FXAS21000_FDPSPERCOUNT        (FXAS21000_GYRO_RANGE_DPS / FXAS21000_GYRO_MAX_COUNT)   // dps per count
#define FXAS21000_COUNTSPERDPS        (1.0F / FXAS21000_FDPSPERCOUNT)   // counts per dps
#define FXAS21000_GYRO_RADSPERCOUNT   (FXAS21000_FDPSPERCOUNT * M_PI / 180.0F)  // rad/s per count

// Gyroscope Sensitivity (from datasheet)
#define FXAS21000_GYRO_SENSITIVITY_250DPS   128.0F    // LSB/(dps)
#define FXAS21000_GYRO_SENSITIVITY_500DPS   64.0F     // LSB/(dps)
#define FXAS21000_GYRO_SENSITIVITY_1000DPS  32.0F     // LSB/(dps) (current)
#define FXAS21000_GYRO_SENSITIVITY_2000DPS  16.0F     // LSB/(dps)

// Gyroscope Performance (from datasheet)
#define FXAS21000_GYRO_NOISE_DENSITY  0.025F         // dps/√Hz (typ)
#define FXAS21000_GYRO_ZERO_RATE_OFFSET 50.0F        // dps (typ)
#define FXAS21000_GYRO_LINEARITY      0.1F           // % of FS

/*******************************************************************************
 * MAGNETOMETER SPECIFICATIONS (FXOS8700CQ)
 ******************************************************************************/

// Full-Scale Range: ±1200 µT (fixed)
#define FXOS8700_MAG_RANGE_UT        1200.0F   // ±1200 µT full-scale range

// 16-bit ADC Resolution (signed: -32768 to +32767)
#define FXOS8700_MAG_BITS            16
#define FXOS8700_MAG_MAX_COUNT       32767

// Scale Factors for ±1200 µT range
#define FXOS8700_FUTPERCOUNT         (FXOS8700_MAG_RANGE_UT / FXOS8700_MAG_MAX_COUNT)  // µT per count
#define FXOS8700_COUNTSPERUT         (1.0F / FXOS8700_FUTPERCOUNT)   // counts per µT

// Magnetometer Sensitivity (from datasheet)
#define FXOS8700_MAG_SENSITIVITY     0.1F              // µT/LSB

// Magnetometer Performance (from datasheet)
#define FXOS8700_MAG_NOISE           0.4F              // µT RMS (typ)
#define FXOS8700_MAG_MEASUREMENT_RANGE 1200.0F         // µT (±1200 µT)

/*******************************************************************************
 * SAMPLING RATE CONFIGURATION
 ******************************************************************************/

// Gyroscope Output Data Rate (FXAS21000)
#define FXAS21000_GYRO_ODR_HZ        100               // 100 Hz (configurable: 12.5 Hz - 800 Hz)

// Accelerometer Output Data Rate (FXOS8700CQ)
#define FXOS8700_ACCEL_ODR_HZ        100               // 100 Hz (configurable: 1.56 Hz - 800 Hz)

// Magnetometer Output Data Rate (FXOS8700CQ)
#define FXOS8700_MAG_ODR_HZ          100               // 100 Hz (hybrid mode, same as accel)

// Effective Sampling Rates Used in Algorithm
#define FREESCALE_SAMPLE_RATE_HZ     100               // Primary sampling rate
#define FREESCALE_FUSION_RATE_HZ     25                // Kalman filter update rate

// Sampling Period
#define FREESCALE_SAMPLE_PERIOD_MS   10                // ms (1000 / 100)
#define FREESCALE_SAMPLE_PERIOD_US   10000             // µs
#define FREESCALE_SAMPLE_PERIOD_NS   10000000LL        // ns

/*******************************************************************************
 * OPERATING CONDITIONS
 ******************************************************************************/

// Operating Temperature Range
#define FXOS8700_TEMP_MIN_C          -40               // °C
#define FXOS8700_TEMP_MAX_C          85                // °C
#define FXAS21000_TEMP_MIN_C         -40               // °C
#define FXAS21000_TEMP_MAX_C         85                // °C
#define FREESCALE_TEMP_TYPICAL_C     25                // °C (for specs)

// Power Supply
#define FXOS8700_VDD_MIN_V           1.95F             // V
#define FXOS8700_VDD_MAX_V           3.6F              // V
#define FXAS21000_VDD_MIN_V          2.25F             // V
#define FXAS21000_VDD_MAX_V          3.6F              // V
#define FREESCALE_VDD_TYPICAL_V      3.3F              // V

/*******************************************************************************
 * ORIENTATION AND AXIS CONVENTION
 ******************************************************************************/

// Coordinate System: Right-handed, NED (North-East-Down) convention
// X-axis: Forward (North)
// Y-axis: Right (East)
// Z-axis: Down

// Gravity Vector (standard)
#define FREESCALE_GRAVITY_MPS2       9.80665F          // m/s^2 (standard gravity)

// Earth's Magnetic Field (typical values)
#define FXOS8700_EARTH_MAG_MIN_UT    25.0F             // µT (equator)
#define FXOS8700_EARTH_MAG_MAX_UT    65.0F             // µT (poles)
#define FXOS8700_EARTH_MAG_TYPICAL_UT 50.0F            // µT (mid-latitudes)

/*******************************************************************************
 * CONVERSION MACROS
 ******************************************************************************/

// Accelerometer conversions
#define FXOS8700_ACCEL_RAW_TO_G(raw)      ((float)(raw) * FXOS8700_FGPERCOUNT)
#define FXOS8700_ACCEL_RAW_TO_MPS2(raw)   ((float)(raw) * FXOS8700_ACCEL_MPSPERCOUNT)
#define FXOS8700_ACCEL_G_TO_RAW(g)        ((int16_t)((g) * FXOS8700_COUNTSPERG))

// Gyroscope conversions
#define FXAS21000_GYRO_RAW_TO_DPS(raw)    ((float)(raw) * FXAS21000_FDPSPERCOUNT)
#define FXAS21000_GYRO_RAW_TO_RADS(raw)   ((float)(raw) * FXAS21000_GYRO_RADSPERCOUNT)
#define FXAS21000_GYRO_DPS_TO_RAW(dps)    ((int16_t)((dps) * FXAS21000_COUNTSPERDPS))

// Magnetometer conversions
#define FXOS8700_MAG_RAW_TO_UT(raw)       ((float)(raw) * FXOS8700_FUTPERCOUNT)
#define FXOS8700_MAG_UT_TO_RAW(ut)        ((int16_t)((ut) * FXOS8700_COUNTSPERUT))

// Angular conversions
#define FREESCALE_DEG_TO_RAD(deg)         ((deg) * M_PI / 180.0F)  // π/180
#define FREESCALE_RAD_TO_DEG(rad)         ((rad) * 180.0F / M_PI)    // 180/π

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
#define FXOS8700_ACCEL_EXPECTED_MIN_G     -1.2F         // g (should be close to ±1g when at rest)
#define FXOS8700_ACCEL_EXPECTED_MAX_G     1.2F          // g
#define FXAS21000_GYRO_EXPECTED_DRIFT_DPS 5.0F          // dps (typical drift at rest)
#define FXOS8700_MAG_EXPECTED_MIN_UT      20.0F         // µT
#define FXOS8700_MAG_EXPECTED_MAX_UT      70.0F         // µT

#endif /* SENSOR_PLATFORM_FREESCALE */

/*==============================================================================*/

/*******************************************************************************
 * UNIFIED INTERFACE MACROS (Platform-Independent)
 ******************************************************************************/

#ifdef SENSOR_PLATFORM_INVENSENSE
    // Map to InvenSense MPU9250
    #define ACCEL_RANGE_G             MPU9250_ACC_RANGE_G
    #define ACCEL_FGPERCOUNT          MPU9250_FGPERCOUNT
    #define ACCEL_COUNTSPERG          MPU9250_COUNTSPERG
    #define ACCEL_MPSPERCOUNT         MPU9250_ACCEL_MPSPERCOUNT

    #define GYRO_RANGE_DPS            MPU9250_GYRO_RANGE_DPS
    #define GYRO_FDPSPERCOUNT         MPU9250_FDPSPERCOUNT
    #define GYRO_COUNTSPERDPS         MPU9250_COUNTSPERDPS
    #define GYRO_RADSPERCOUNT         MPU9250_GYRO_RADSPERCOUNT

    #define MAG_RANGE_UT              AK8963_MAG_RANGE_UT
    #define MAG_FUTPERCOUNT           AK8963_FUTPERCOUNT
    #define MAG_COUNTSPERUT           AK8963_COUNTSPERUT

    #define SAMPLE_RATE_HZ            MPU9250_SAMPLE_RATE_HZ
    #define FUSION_RATE_HZ            MPU9250_FUSION_RATE_HZ
    #define GRAVITY_MPS2              MPU9250_GRAVITY_MPS2

#elif defined(SENSOR_PLATFORM_FREESCALE)
    // Map to Freescale FXOS8700CQ + FXAS21000
    #define ACCEL_RANGE_G             FXOS8700_ACC_RANGE_G
    #define ACCEL_FGPERCOUNT          FXOS8700_FGPERCOUNT
    #define ACCEL_COUNTSPERG          FXOS8700_COUNTSPERG
    #define ACCEL_MPSPERCOUNT         FXOS8700_ACCEL_MPSPERCOUNT

    #define GYRO_RANGE_DPS            FXAS21000_GYRO_RANGE_DPS
    #define GYRO_FDPSPERCOUNT         FXAS21000_FDPSPERCOUNT
    #define GYRO_COUNTSPERDPS         FXAS21000_COUNTSPERDPS
    #define GYRO_RADSPERCOUNT         FXAS21000_GYRO_RADSPERCOUNT

    #define MAG_RANGE_UT              FXOS8700_MAG_RANGE_UT
    #define MAG_FUTPERCOUNT           FXOS8700_FUTPERCOUNT
    #define MAG_COUNTSPERUT           FXOS8700_COUNTSPERUT

    #define SAMPLE_RATE_HZ            FREESCALE_SAMPLE_RATE_HZ
    #define FUSION_RATE_HZ            FREESCALE_FUSION_RATE_HZ
    #define GRAVITY_MPS2              FREESCALE_GRAVITY_MPS2
#endif

/*******************************************************************************
 * PLATFORM INFORMATION (for runtime display)
 ******************************************************************************/
#define SENSOR_SPEC_VERSION           "2.0.0"
#define SENSOR_SPEC_DATE              "2025-10-12"

#endif /* SENSOR_SPEC_AGM_H */
