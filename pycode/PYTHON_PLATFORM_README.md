# Python Multi-Platform Sensor Configuration

## Overview

The `sensor_platform_config.py` module provides platform-independent sensor specifications for Python-based sensor fusion implementations. It mirrors the C header file functionality while leveraging Python's object-oriented features.

## Installation

No installation required - just import the module:

```python
from sensor_platform_config import SensorPlatform, INVENSENSE, FREESCALE
```

## Quick Start

### Basic Usage

```python
from sensor_platform_config import SensorPlatform, INVENSENSE, FREESCALE

# Use default platform (INVENSENSE)
platform = SensorPlatform()

# Or specify platform explicitly
platform = SensorPlatform(INVENSENSE)
platform = SensorPlatform(FREESCALE)

# Access sensor specifications
accel_scale = platform.accel_scale_factor  # g/count
gyro_scale = platform.gyro_scale_factor    # dps/count
mag_scale = platform.mag_scale_factor      # µT/count
```

### Convert Raw Sensor Data

```python
# Convert raw ADC counts to physical units
accel_g = platform.accel_raw_to_g(acc_raw)
accel_mps2 = platform.accel_raw_to_mps2(acc_raw)

gyro_dps = platform.gyro_raw_to_dps(gyro_raw)
gyro_rads = platform.gyro_raw_to_rads(gyro_raw)

mag_ut = platform.mag_raw_to_ut(mag_raw)
```

### Convert Physical Units to Raw Counts

```python
# Convert physical units to raw ADC counts
acc_raw = platform.accel_g_to_raw(1.0)      # 1g
gyro_raw = platform.gyro_dps_to_raw(100.0)  # 100 dps
mag_raw = platform.mag_ut_to_raw(50.0)      # 50 µT
```

## Supported Platforms

### INVENSENSE Platform
- **Hardware**: InvenSense MPU-9250
- **Accelerometer**: 16-bit, ±4g, 8192 LSB/g
- **Gyroscope**: 16-bit, ±1000dps, 32.8 LSB/dps
- **Magnetometer**: 16-bit, ±4800µT, AK8963

### FREESCALE Platform
- **Hardware**: NXP FRDM-STBC-AGM01
- **Accelerometer**: 14-bit, ±4g, 2048 counts/g (FXOS8700CQ)
- **Gyroscope**: 16-bit, ±1000dps, 32.0 LSB/dps (FXAS21000)
- **Magnetometer**: 16-bit, ±1200µT (FXOS8700CQ)

## Platform-Independent Properties

The module provides unified properties that work across all platforms:

| Property | Type | Description |
|----------|------|-------------|
| `accel_range_g` | float | Accelerometer full-scale range (g) |
| `accel_scale_factor` | float | g per ADC count |
| `accel_counts_per_g` | float | ADC counts per g |
| `accel_mps2_per_count` | float | m/s² per ADC count |
| `gyro_range_dps` | float | Gyroscope full-scale range (dps) |
| `gyro_scale_factor` | float | dps per ADC count |
| `gyro_counts_per_dps` | float | ADC counts per dps |
| `gyro_rads_per_count` | float | rad/s per ADC count |
| `mag_range_ut` | float | Magnetometer full-scale range (µT) |
| `mag_scale_factor` | float | µT per ADC count |
| `mag_counts_per_ut` | float | ADC counts per µT |
| `sample_rate_hz` | int | Primary sampling rate (Hz) |
| `fusion_rate_hz` | int | Kalman filter update rate (Hz) |

## Detailed Specifications

Access detailed specifications for each sensor:

```python
# Accelerometer specs
print(f"Accelerometer: {platform.accelerometer.name}")
print(f"  Range: ±{platform.accelerometer.range_g}g")
print(f"  Resolution: {platform.accelerometer.bits}-bit")
print(f"  Sensitivity: {platform.accelerometer.sensitivity} counts/g")
print(f"  Noise: {platform.accelerometer.noise_density} µg/√Hz")

# Gyroscope specs
print(f"Gyroscope: {platform.gyroscope.name}")
print(f"  Range: ±{platform.gyroscope.range_dps}dps")
print(f"  Resolution: {platform.gyroscope.bits}-bit")
print(f"  Sensitivity: {platform.gyroscope.sensitivity} LSB/dps")
print(f"  Noise: {platform.gyroscope.noise_density} dps/√Hz")

# Magnetometer specs
print(f"Magnetometer: {platform.magnetometer.name}")
print(f"  Range: ±{platform.magnetometer.range_ut}µT")
print(f"  Sensitivity: {platform.magnetometer.sensitivity} µT/LSB")
print(f"  Noise: {platform.magnetometer.noise} µT RMS")

# Sampling configuration
print(f"Sample Rate: {platform.sampling.sample_rate_hz} Hz")
print(f"Fusion Rate: {platform.sampling.fusion_rate_hz} Hz")
print(f"Sample Period: {platform.sampling.sample_period_ms} ms")
```

## Print All Specifications

```python
# Print complete platform specifications
platform.print_specs()
```

Output:
```
================================================================================
 SENSOR PLATFORM: InvenSense MPU-9250
================================================================================
Type:          9-Axis MEMS IMU
Manufacturer:  InvenSense (TDK)

ACCELEROMETER:
  Name:            MPU-9250
  Range:           ±4.0 g
  Resolution:      16-bit
  Max Count:       32767
  Sensitivity:     8192.0 counts/g
  Scale Factor:    0.0001220740 g/count
  Noise Density:   300.0 µg/√Hz

GYROSCOPE:
  Name:            MPU-9250
  Range:           ±1000.0 dps
  Resolution:      16-bit
  Max Count:       32767
  Sensitivity:     32.8 LSB/dps
  Scale Factor:    0.0305185095 dps/count
  Noise Density:   0.010 dps/√Hz

MAGNETOMETER:
  Name:            AK8963
  Range:           ±4800.0 µT
  Resolution:      16-bit
  Max Count:       32767
  Sensitivity:     0.15 µT/LSB
  Scale Factor:    0.1464888455 µT/count
  Noise:           0.6 µT RMS

SAMPLING:
  Sample Rate:     100 Hz
  Fusion Rate:     25 Hz
  Sample Period:   10.00 ms
================================================================================
```

## Integration with Sensor Fusion

### Example: Process Sensor Data

```python
import numpy as np
from sensor_platform_config import SensorPlatform, INVENSENSE

# Initialize platform
platform = SensorPlatform(INVENSENSE)

def process_sensor_data(acc_raw, gyro_raw, mag_raw):
    """
    Process raw sensor data using platform-specific conversions

    Args:
        acc_raw: Accelerometer raw counts [x, y, z]
        gyro_raw: Gyroscope raw counts [x, y, z]
        mag_raw: Magnetometer raw counts [x, y, z]

    Returns:
        Tuple of (accel_g, gyro_dps, mag_ut) in physical units
    """
    # Convert to physical units
    accel_g = np.array([platform.accel_raw_to_g(acc_raw[i])
                        for i in range(3)])

    gyro_dps = np.array([platform.gyro_raw_to_dps(gyro_raw[i])
                         for i in range(3)])

    mag_ut = np.array([platform.mag_raw_to_ut(mag_raw[i])
                       for i in range(3)])

    return accel_g, gyro_dps, mag_ut


# Example usage
acc_raw = np.array([100, 200, 8192])  # ~1g on Z-axis
gyro_raw = np.array([0, 0, 0])        # At rest
mag_raw = np.array([1000, 0, 2000])   # Example magnetic field

accel_g, gyro_dps, mag_ut = process_sensor_data(acc_raw, gyro_raw, mag_raw)

print(f"Accelerometer: {accel_g} g")
print(f"Gyroscope: {gyro_dps} dps")
print(f"Magnetometer: {mag_ut} µT")
```

### Example: Sensor Fusion Class Integration

```python
from sensor_platform_config import SensorPlatform, INVENSENSE
import numpy as np

class SensorFusion:
    def __init__(self, platform_type=INVENSENSE):
        """Initialize sensor fusion with specified platform"""
        self.platform = SensorPlatform(platform_type)

        # Use platform-specific scale factors
        self.acc_scale = self.platform.accel_scale_factor
        self.gyro_scale = self.platform.gyro_scale_factor
        self.mag_scale = self.platform.mag_scale_factor

        # Use platform-specific sampling rates
        self.sample_rate = self.platform.sample_rate_hz
        self.fusion_rate = self.platform.fusion_rate_hz
        self.dt = 1.0 / self.fusion_rate

        print(f"Initialized {self.platform.platform_name}")
        print(f"  Sample Rate: {self.sample_rate} Hz")
        print(f"  Fusion Rate: {self.fusion_rate} Hz")

    def update(self, acc_raw, gyro_raw, mag_raw):
        """Process sensor update"""
        # Convert to physical units using platform conversions
        acc_mps2 = np.array([self.platform.accel_raw_to_mps2(v)
                             for v in acc_raw])
        gyro_rads = np.array([self.platform.gyro_raw_to_rads(v)
                              for v in gyro_raw])
        mag_ut = np.array([self.platform.mag_raw_to_ut(v)
                           for v in mag_raw])

        # Your sensor fusion algorithm here...
        # ... using acc_mps2, gyro_rads, mag_ut ...

        return acc_mps2, gyro_rads, mag_ut


# Usage
fusion = SensorFusion(INVENSENSE)
# fusion = SensorFusion(FREESCALE)  # Switch to Freescale platform

acc, gyro, mag = fusion.update(acc_raw, gyro_raw, mag_raw)
```

## Switching Between Platforms

### Runtime Platform Selection

```python
import sys
from sensor_platform_config import SensorPlatform, INVENSENSE, FREESCALE

# Select platform from command line
if len(sys.argv) > 1 and sys.argv[1].upper() == 'FREESCALE':
    platform = SensorPlatform(FREESCALE)
else:
    platform = SensorPlatform(INVENSENSE)

print(f"Using platform: {platform.platform_name}")
```

### Configuration File

```python
import json
from sensor_platform_config import SensorPlatform, INVENSENSE, FREESCALE

# Load configuration
with open('config.json', 'r') as f:
    config = json.load(f)

# Select platform from config
platform_name = config.get('platform', 'INVENSENSE')
platform = SensorPlatform(INVENSENSE if platform_name == 'INVENSENSE'
                         else FREESCALE)

print(f"Loaded platform: {platform.platform_name}")
```

## Constants

The module provides useful constants:

```python
# Gravity constant
SensorPlatform.GRAVITY_MPS2  # 9.80665 m/s²

# Angular conversions
SensorPlatform.DEG_TO_RAD  # π/180
SensorPlatform.RAD_TO_DEG  # 180/π
```

## Advanced Usage

### Custom Platform Configuration

You can extend the module to support custom platforms:

```python
from sensor_platform_config import Platform, AccelerometerSpec, GyroscopeSpec, MagnetometerSpec, SamplingConfig

# Define custom platform
SensorPlatform._PLATFORMS[Platform.CUSTOM] = {
    'name': 'Custom IMU',
    'type': 'Custom 9-Axis',
    'manufacturer': 'Custom Manufacturer',
    'accelerometer': AccelerometerSpec(...),
    'gyroscope': GyroscopeSpec(...),
    'magnetometer': MagnetometerSpec(...),
    'sampling': SamplingConfig(...),
}
```

### Batch Conversion

```python
import numpy as np

# Convert array of raw values
acc_raw_array = np.array([[100, 200, 8192],
                          [110, 210, 8200],
                          [120, 220, 8210]])

# Vectorized conversion
acc_g_array = acc_raw_array * platform.accel_scale_factor
```

## Testing

Run the module directly to test both platforms:

```bash
python3 sensor_platform_config.py
```

This will:
- Print specifications for both INVENSENSE and FREESCALE platforms
- Test conversion functions
- Validate scale factor accuracy

## Comparison with C Implementation

The Python module mirrors the C header file (`sensor_spec_agm.h`):

| Feature | C Header | Python Module |
|---------|----------|---------------|
| Platform Selection | Compile-time (`#define`) | Runtime (constructor parameter) |
| Specifications | Preprocessor macros | Class properties |
| Conversions | Macro functions | Class methods |
| Validation | Compile errors | Runtime exceptions |
| Type Safety | Limited | Full (dataclasses) |
| Documentation | Comments | Docstrings + type hints |

## Version History

- **v2.0.0 (2025-10-12)**: Multi-platform support
  - Added INVENSENSE and FREESCALE platforms
  - Object-oriented design with dataclasses
  - Comprehensive conversion methods
  - Unified platform-independent interface

## See Also

- C implementation: `/code/app/inc/sensor_spec_agm.h`
- C README: `/code/app/inc/SENSOR_PLATFORM_README.md`
- Test program: Run `python3 sensor_platform_config.py`
