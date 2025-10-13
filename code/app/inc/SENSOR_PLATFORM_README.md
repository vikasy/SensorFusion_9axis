# Sensor Platform Configuration Guide

## Overview

The `sensor_spec_agm.h` header file now supports multiple sensor platforms through compile-time flags. This allows the same codebase to work with different IMU hardware configurations.

## Supported Platforms

### 1. INVENSENSE Platform (Default)
**Hardware**: InvenSense MPU-9250
**Sensors**:
- MPU-9250: 3-axis Accelerometer + 3-axis Gyroscope
- AK8963: 3-axis Magnetometer (integrated)

**Specifications**:
- Accelerometer: ±4g range, 16-bit resolution
- Gyroscope: ±1000 dps range, 16-bit resolution
- Magnetometer: ±4800 µT range, 16-bit resolution

### 2. FREESCALE Platform
**Hardware**: NXP FRDM-STBC-AGM01 Development Kit
**Sensors**:
- FXOS8700CQ: 6-axis Accelerometer + Magnetometer
- FXAS21000: 3-axis Gyroscope

**Specifications**:
- Accelerometer: ±4g range, 14-bit resolution
- Gyroscope: ±1000 dps range, 16-bit resolution
- Magnetometer: ±1200 µT range, 16-bit resolution

## Usage

### Method 1: Compile-Time Flag in Source Code

Define the platform before including the header:

```c
// For InvenSense MPU-9250
#define SENSOR_PLATFORM_INVENSENSE
#include "sensor_spec_agm.h"
```

or

```c
// For Freescale FRDM-STBC-AGM01
#define SENSOR_PLATFORM_FREESCALE
#include "sensor_spec_agm.h"
```

### Method 2: Compiler Command Line

Pass the flag during compilation:

```bash
# For InvenSense MPU-9250
gcc -DSENSOR_PLATFORM_INVENSENSE main.c -o app

# For Freescale FRDM-STBC-AGM01
gcc -DSENSOR_PLATFORM_FREESCALE main.c -o app
```

### Method 3: CMake Configuration

Add to your CMakeLists.txt:

```cmake
# For InvenSense MPU-9250
add_compile_definitions(SENSOR_PLATFORM_INVENSENSE)

# Or for Freescale FRDM-STBC-AGM01
add_compile_definitions(SENSOR_PLATFORM_FREESCALE)
```

### Method 4: Makefile

Add to your Makefile:

```makefile
# For InvenSense MPU-9250
CFLAGS += -DSENSOR_PLATFORM_INVENSENSE

# Or for Freescale FRDM-STBC-AGM01
CFLAGS += -DSENSOR_PLATFORM_FREESCALE
```

## Platform-Independent Code

The header provides unified interface macros that work across all platforms:

```c
#include "sensor_spec_agm.h"

// These macros automatically map to the correct platform-specific values
float accel_scale = ACCEL_FGPERCOUNT;      // g per count
float gyro_scale = GYRO_FDPSPERCOUNT;      // dps per count
float mag_scale = MAG_FUTPERCOUNT;         // µT per count

int sample_rate = SAMPLE_RATE_HZ;          // Sampling rate
int fusion_rate = FUSION_RATE_HZ;          // Kalman filter update rate
float gravity = GRAVITY_MPS2;              // Standard gravity
```

## Platform-Specific Values

### INVENSENSE Platform Constants

**Direct Access** (when you need platform-specific features):
```c
#ifdef SENSOR_PLATFORM_INVENSENSE
    float mpu_accel_range = MPU9250_ACC_RANGE_G;        // 4.0g
    float mpu_gyro_range = MPU9250_GYRO_RANGE_DPS;      // 1000.0 dps
    float ak_mag_range = AK8963_MAG_RANGE_UT;           // 4800.0 µT

    // Sensitivity values from datasheet
    float accel_sensitivity = MPU9250_ACCEL_SENSITIVITY_4G;  // 8192.0 LSB/g
    float gyro_sensitivity = MPU9250_GYRO_SENSITIVITY_1000DPS; // 32.8 LSB/dps
    float mag_sensitivity = AK8963_MAG_SENSITIVITY_16BIT;    // 0.15 µT/LSB
#endif
```

### FREESCALE Platform Constants

**Direct Access**:
```c
#ifdef SENSOR_PLATFORM_FREESCALE
    float fxos_accel_range = FXOS8700_ACC_RANGE_G;       // 4.0g
    float fxas_gyro_range = FXAS21000_GYRO_RANGE_DPS;    // 1000.0 dps
    float fxos_mag_range = FXOS8700_MAG_RANGE_UT;        // 1200.0 µT

    // Sensitivity values from datasheet
    float accel_sensitivity = FXOS8700_ACCEL_SENSITIVITY_4G;  // 2048.0 counts/g
    float gyro_sensitivity = FXAS21000_GYRO_SENSITIVITY_1000DPS; // 32.0 LSB/dps
    float mag_sensitivity = FXOS8700_MAG_SENSITIVITY;         // 0.1 µT/LSB
#endif
```

## Conversion Macros

### Platform-Independent (Recommended)

Use the unified macros that work across all platforms:

```c
// Convert raw sensor counts to physical units
float accel_g = (float)accel_raw * ACCEL_FGPERCOUNT;
float gyro_dps = (float)gyro_raw * GYRO_FDPSPERCOUNT;
float mag_ut = (float)mag_raw * MAG_FUTPERCOUNT;
```

### Platform-Specific

Each platform also provides its own conversion macros:

**INVENSENSE**:
```c
float accel_g = MPU9250_ACCEL_RAW_TO_G(accel_raw);
float accel_mps2 = MPU9250_ACCEL_RAW_TO_MPS2(accel_raw);
float gyro_dps = MPU9250_GYRO_RAW_TO_DPS(gyro_raw);
float gyro_rads = MPU9250_GYRO_RAW_TO_RADS(gyro_raw);
float mag_ut = AK8963_MAG_RAW_TO_UT(mag_raw);
```

**FREESCALE**:
```c
float accel_g = FXOS8700_ACCEL_RAW_TO_G(accel_raw);
float accel_mps2 = FXOS8700_ACCEL_RAW_TO_MPS2(accel_raw);
float gyro_dps = FXAS21000_GYRO_RAW_TO_DPS(gyro_raw);
float gyro_rads = FXAS21000_GYRO_RAW_TO_RADS(gyro_raw);
float mag_ut = FXOS8700_MAG_RAW_TO_UT(mag_raw);
```

## Runtime Platform Information

Query platform information at runtime:

```c
#include <stdio.h>
#include "sensor_spec_agm.h"

void print_platform_info(void) {
    printf("Sensor Platform: %s\n", SENSOR_PLATFORM_NAME);
    printf("Platform Type: %s\n", SENSOR_PLATFORM_TYPE);
    printf("Manufacturer: %s\n", SENSOR_PLATFORM_MANUFACTURER);
    printf("Spec Version: %s\n", SENSOR_SPEC_VERSION);
    printf("Spec Date: %s\n", SENSOR_SPEC_DATE);

    printf("\nSensor Ranges:\n");
    printf("  Accelerometer: ±%.1fg\n", ACCEL_RANGE_G);
    printf("  Gyroscope: ±%.1f dps\n", GYRO_RANGE_DPS);
    printf("  Magnetometer: ±%.1f µT\n", MAG_RANGE_UT);

    printf("\nSampling Configuration:\n");
    printf("  Sample Rate: %d Hz\n", SAMPLE_RATE_HZ);
    printf("  Fusion Rate: %d Hz\n", FUSION_RATE_HZ);
}
```

## Complete Example

```c
#define SENSOR_PLATFORM_INVENSENSE  // or SENSOR_PLATFORM_FREESCALE
#include "sensor_spec_agm.h"

void process_sensor_data(int16_t acc_raw[3], int16_t gyro_raw[3], int16_t mag_raw[3]) {
    // Convert to physical units (platform-independent)
    float acc_g[3], gyro_dps[3], mag_ut[3];

    for (int i = 0; i < 3; i++) {
        acc_g[i] = acc_raw[i] * ACCEL_FGPERCOUNT;
        gyro_dps[i] = gyro_raw[i] * GYRO_FDPSPERCOUNT;
        mag_ut[i] = mag_raw[i] * MAG_FUTPERCOUNT;
    }

    // Your sensor fusion algorithm here...
}

int main(void) {
    print_platform_info();

    // Example sensor readings (raw ADC counts)
    int16_t acc_raw[3] = {0, 0, 8192};   // ~1g on Z-axis (INVENSENSE)
    int16_t gyro_raw[3] = {0, 0, 0};      // At rest
    int16_t mag_raw[3] = {1000, 0, 2000}; // Sample magnetic field

    process_sensor_data(acc_raw, gyro_raw, mag_raw);

    return 0;
}
```

## Key Differences Between Platforms

| Feature | INVENSENSE (MPU-9250) | FREESCALE (FXOS8700CQ+FXAS21000) |
|---------|----------------------|----------------------------------|
| **Accelerometer Resolution** | 16-bit | 14-bit |
| **Accelerometer LSB/g (±4g)** | 8192 | 2048 |
| **Gyroscope LSB/dps (±1000dps)** | 32.8 | 32.0 |
| **Magnetometer Range** | ±4800 µT | ±1200 µT |
| **Integrated Solution** | Yes (single chip) | No (2 separate chips) |
| **Noise (Accel)** | 300 µg/√Hz | 126 µg/√Hz |
| **Noise (Gyro)** | 0.01 dps/√Hz | 0.025 dps/√Hz |

## Notes

1. **Default Platform**: If no platform is defined, the code defaults to INVENSENSE with a warning.

2. **Mutual Exclusion**: Defining both platforms simultaneously will cause a compilation error.

3. **Backward Compatibility**: Existing code using MPU9250-specific macros will continue to work when SENSOR_PLATFORM_INVENSENSE is defined.

4. **Portability**: For maximum portability, use the unified interface macros (ACCEL_*, GYRO_*, MAG_*) instead of platform-specific names.

5. **Documentation**: All values are derived from official datasheets:
   - InvenSense MPU-9250 Product Specification
   - NXP FXOS8700CQ Datasheet
   - NXP FXAS21000 Datasheet

## Version History

- **v2.0.0 (2025-10-12)**: Added multi-platform support with compile-time flags
- **v1.0.0 (2025-10-11)**: Initial version with MPU-9250 specifications only
