# Synthetic IMU Datasets

**Generated:** 2025-10-13 21:08:03
**Sensor:** MPU9250 (Accel + Gyro) + AK8963 (Magnetometer)
**Sample Rate:** 100.0 Hz

## Sensor Specifications

### Accelerometer (MPU9250)
- Range: ±4.0 g
- Resolution: 16 bits
- Scale Factor: 8192.00 counts/g
- Noise Density: 300 µg/√Hz
- Bias Stability: ±0.01 g

### Gyroscope (MPU9250)
- Range: ±1000.0 dps
- Resolution: 16 bits
- Scale Factor: 32.77 counts/dps
- Noise Density: 0.01 dps/√Hz
- Bias Stability: ±0.5 dps

### Magnetometer (AK8963)
- Range: ±4800.0 µT
- Resolution: 16 bits
- Scale Factor: 6.83 counts/µT
- Noise: 0.6 µT RMS
- Earth Field Strength: 50.0 µT
- Magnetic Inclination: 60.0°

## Dataset Files

### static_10s.csv
- Samples: 1000
- Duration: 10.00 seconds
- Description: Static 10S

### rotation_z_30dps.csv
- Samples: 1000
- Duration: 10.00 seconds
- Description: Rotation Z 30Dps

### rotation_x_20dps.csv
- Samples: 1000
- Duration: 10.00 seconds
- Description: Rotation X 20Dps

### rotation_y_15dps.csv
- Samples: 1000
- Duration: 10.00 seconds
- Description: Rotation Y 15Dps

### rotation_sequence.csv
- Samples: 1500
- Duration: 15.00 seconds
- Description: Rotation Sequence

### vibration_5hz.csv
- Samples: 1000
- Duration: 10.00 seconds
- Description: Vibration 5Hz

### static_high_noise.csv
- Samples: 1000
- Duration: 10.00 seconds
- Description: Static High Noise

### static_high_bias.csv
- Samples: 1000
- Duration: 10.00 seconds
- Description: Static High Bias

### complex_motion.csv
- Samples: 2000
- Duration: 20.00 seconds
- Description: Complex Motion

### static_60s.csv
- Samples: 6000
- Duration: 60.00 seconds
- Description: Static 60S


## CSV Format

Each CSV file contains the following columns:

### Sensor Data (Counts)
- `timestamp_ns`: Timestamp in nanoseconds
- `accel_x_counts`, `accel_y_counts`, `accel_z_counts`: Accelerometer readings (int16)
- `gyro_x_counts`, `gyro_y_counts`, `gyro_z_counts`: Gyroscope readings (int16)
- `mag_x_counts`, `mag_y_counts`, `mag_z_counts`: Magnetometer readings (int16)

### Ground Truth (for validation)
- `gt_roll_deg`, `gt_pitch_deg`, `gt_yaw_deg`: True Euler angles in degrees
- `gt_quat_w`, `gt_quat_x`, `gt_quat_y`, `gt_quat_z`: True quaternion (scalar-first)

## Usage with Tests

### 6-Axis Tests (Accel + Gyro)
```bash
./build/bin/test_6axis_e2e test/datasets/synthetic/static_10s.csv
./build/bin/test_6axis_e2e test/datasets/synthetic/rotation_z_30dps_10s.csv
```

### 9-Axis Tests (Accel + Gyro + Mag)
```bash
./build/bin/test_9axis_e2e test/datasets/synthetic/static_10s.csv
./build/bin/test_9axis_e2e test/datasets/synthetic/rotation_sequence_15s.csv
```

## Regenerating Datasets

To regenerate all datasets:
```bash
cd scripts
python3 generate_synthetic_datasets.py
```

To generate specific scenarios with custom parameters:
```bash
python3 generate_synthetic_datasets.py --duration 30 --sample-rate 200
```
