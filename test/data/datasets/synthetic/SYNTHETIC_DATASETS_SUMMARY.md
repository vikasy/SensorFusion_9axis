# Synthetic IMU Datasets - Generation Summary

**Generated:** October 12, 2025
**Script:** `scripts/generate_synthetic_datasets.py`
**Location:** `test/datasets/synthetic/`

---

## Overview

Successfully generated 10 comprehensive synthetic IMU datasets for testing both 6-axis and 9-axis sensor fusion algorithms. All datasets include ground truth orientation for validation.

---

## Generated Datasets

| Dataset | Samples | Duration | Size | Description |
|---------|---------|----------|------|-------------|
| `static_10s.csv` | 1,000 | 10 sec | 71 KB | Static scenario (calibration baseline) |
| `rotation_z_30dps_10s.csv` | 1,000 | 10 sec | 117 KB | Constant yaw rotation (30°/s) |
| `rotation_x_20dps_10s.csv` | 1,000 | 10 sec | 120 KB | Constant roll rotation (20°/s) |
| `rotation_y_15dps_10s.csv` | 1,000 | 10 sec | 120 KB | Constant pitch rotation (15°/s) |
| `rotation_sequence_15s.csv` | 1,500 | 15 sec | 236 KB | Smooth rotation through waypoints |
| `vibration_5hz_10s.csv` | 1,000 | 10 sec | 71 KB | Sinusoidal vibration (5 Hz) |
| `static_high_noise_10s.csv` | 1,000 | 10 sec | 74 KB | Static with 5x noise |
| `static_high_bias_10s.csv` | 1,000 | 10 sec | 71 KB | Static with 3x bias |
| `complex_motion_20s.csv` | 2,000 | 20 sec | 355 KB | Complex multi-axis motion |
| `static_60s.csv` | 6,000 | 60 sec | 429 KB | Long static for drift analysis |

**Total: 16,500 samples across 10 scenarios**

---

## Sensor Specifications (MPU9250 + AK8963)

### Accelerometer
- **Range:** ±2 g
- **Resolution:** 16 bits
- **Scale:** 16,384 counts/g
- **Noise:** 300 µg/√Hz
- **Bias:** ±0.01 g

### Gyroscope
- **Range:** ±250 dps
- **Resolution:** 16 bits
- **Scale:** 131 counts/dps
- **Noise:** 0.005 dps/√Hz
- **Bias:** ±0.5 dps

### Magnetometer
- **Range:** ±4800 µT
- **Resolution:** 16 bits
- **Scale:** 6.8 counts/µT
- **Noise:** 0.6 µT RMS
- **Earth Field:** 50 µT @ 60° inclination

**Sampling Rate:** 100 Hz (10 ms period)

---

## CSV Format

Each CSV file contains:

### Sensor Data Columns (Int16 Counts)
```
timestamp_ns          - Timestamp in nanoseconds
accel_x_counts        - Accelerometer X-axis (int16)
accel_y_counts        - Accelerometer Y-axis (int16)
accel_z_counts        - Accelerometer Z-axis (int16)
gyro_x_counts         - Gyroscope X-axis (int16)
gyro_y_counts         - Gyroscope Y-axis (int16)
gyro_z_counts         - Gyroscope Z-axis (int16)
mag_x_counts          - Magnetometer X-axis (int16)
mag_y_counts          - Magnetometer Y-axis (int16)
mag_z_counts          - Magnetometer Z-axis (int16)
```

### Ground Truth Columns (Float64)
```
gt_roll_deg           - True roll angle (degrees)
gt_pitch_deg          - True pitch angle (degrees)
gt_yaw_deg            - True yaw angle (degrees)
gt_quat_w             - True quaternion scalar component
gt_quat_x             - True quaternion X component
gt_quat_y             - True quaternion Y component
gt_quat_z             - True quaternion Z component
```

---

## Usage Examples

### Testing 6-Axis Fusion (Accel + Gyro)

```bash
# Static calibration test
./build/bin/test_6axis_e2e test/datasets/synthetic/static_10s.csv

# Rotation test
./build/bin/test_6axis_e2e test/datasets/synthetic/rotation_z_30dps_10s.csv

# Complex motion test
./build/bin/test_6axis_e2e test/datasets/synthetic/complex_motion_20s.csv
```

### Testing 9-Axis Fusion (Accel + Gyro + Mag)

```bash
# Static test with magnetometer
./build/bin/test_9axis_e2e test/datasets/synthetic/static_10s.csv

# Rotation sequence (tests magnetometer fusion)
./build/bin/test_9axis_e2e test/datasets/synthetic/rotation_sequence_15s.csv

# Complex motion
./build/bin/test_9axis_e2e test/datasets/synthetic/complex_motion_20s.csv
```

### Robustness Testing

```bash
# High noise scenario
./build/bin/test_6axis_e2e test/datasets/synthetic/static_high_noise_10s.csv
./build/bin/test_9axis_e2e test/datasets/synthetic/static_high_noise_10s.csv

# High bias scenario
./build/bin/test_6axis_e2e test/datasets/synthetic/static_high_bias_10s.csv
./build/bin/test_9axis_e2e test/datasets/synthetic/static_high_bias_10s.csv

# Vibration scenario
./build/bin/test_6axis_e2e test/datasets/synthetic/vibration_5hz_10s.csv

# Long-term drift analysis
./build/bin/test_6axis_e2e test/datasets/synthetic/static_60s.csv
./build/bin/test_9axis_e2e test/datasets/synthetic/static_60s.csv
```

---

## Motion Scenarios Explained

### 1. Static (Calibration)
- **Purpose:** Baseline calibration, noise characterization
- **Motion:** None - sensor at rest
- **Use Case:** Verify sensor bias removal, initial orientation lock

### 2-4. Single-Axis Constant Rotation
- **Purpose:** Test individual axis rotation tracking
- **Motion:** Constant angular velocity around one axis
- **Use Case:** Validate gyroscope integration, quaternion update

### 5. Smooth Rotation Sequence
- **Purpose:** Test multi-axis smooth transitions
- **Motion:** SLERP interpolation through orientation waypoints
- **Use Case:** Validate complex orientation tracking, magnetometer fusion

### 6. Vibration
- **Purpose:** Test linear acceleration rejection
- **Motion:** Sinusoidal vertical acceleration
- **Use Case:** Verify gravity vector stability during vibration

### 7. High Noise
- **Purpose:** Test robustness to sensor noise
- **Motion:** Static with 5x noise
- **Use Case:** Validate Kalman filter noise rejection

### 8. High Bias
- **Purpose:** Test bias estimation and removal
- **Motion:** Static with 3x bias
- **Use Case:** Verify bias compensation

### 9. Complex Motion
- **Purpose:** Comprehensive algorithm testing
- **Motion:** Multi-axis rotation through 9 waypoints with noise
- **Use Case:** Full algorithm validation

### 10. Long Static (Drift Analysis)
- **Purpose:** Long-term stability analysis
- **Motion:** Static for 60 seconds
- **Use Case:** Measure drift, bias stability over time

---

## Regenerating Datasets

To regenerate all datasets with default parameters:

```bash
cd scripts
python3 generate_synthetic_datasets.py
```

To customize generation:

```bash
# Specify custom output directory
python3 generate_synthetic_datasets.py --output ../test/datasets/custom

# Edit script to modify scenarios, noise levels, or durations
nano generate_synthetic_datasets.py
```

---

## Validation Metrics

For each test run, the following metrics should be computed:

### Orientation Accuracy
- **Roll/Pitch Error:** Compare output orientation to ground truth
- **Yaw Error:** Measure drift over time (6-axis) or accuracy (9-axis)
- **RMSE:** Root mean square error across all samples

### Quaternion Accuracy
- **Quaternion Distance:** `d = 1 - |q_est · q_true|`
- **Angular Error:** `θ = 2 * arccos(|q_est · q_true|)`

### Stability Metrics
- **Static Noise:** Standard deviation during static periods
- **Drift Rate:** Orientation change per second during static
- **Convergence Time:** Time to reach steady-state

### Computational Performance
- **Execution Time:** Per-sample processing time
- **CPU Usage:** Average CPU utilization
- **Memory Usage:** Peak memory footprint

---

## Features of Synthetic Data

### Advantages
✅ **Perfect Ground Truth:** Exact orientation known at every timestep
✅ **Reproducible:** Same results with same seed
✅ **Configurable:** Easily adjust noise, bias, motion profiles
✅ **Comprehensive:** Multiple scenarios covering edge cases
✅ **Realistic:** Based on actual sensor specifications

### Limitations
⚠️ **Simplified Noise Model:** White Gaussian noise (real sensors have colored noise)
⚠️ **Ideal Magnetometer:** No hard/soft iron distortion
⚠️ **No External Disturbances:** No magnetic anomalies, electromagnetic interference
⚠️ **Perfect Alignment:** No sensor misalignment modeled

---

## Integration with Test Suite

These datasets are designed to work with:

1. **E2E Tests:** `test/e2e/test_6axis_e2e.c`, `test_9axis_e2e.c`
2. **Validation Tests:** `test/validation/test_realistic_validation.c`
3. **Performance Benchmarks:** Timing and accuracy measurements

Update E2E tests to:
- Read CSV files
- Parse sensor data
- Run sensor fusion algorithm
- Compare output to ground truth
- Report accuracy metrics

---

## Next Steps

1. ✅ **Generate Datasets** - COMPLETE
2. ⚠️ **Update E2E Tests** - TODO: Fix API to read CSV files
3. ⚠️ **Add Validation Logic** - TODO: Compute error metrics
4. ⚠️ **Create Benchmark Suite** - TODO: Performance testing
5. ⚠️ **Document Results** - TODO: Test report with metrics

---

## Generator Script Details

**Script:** `scripts/generate_synthetic_datasets.py`

**Key Functions:**
- `MotionScenario`: Defines motion profiles (static, rotation, etc.)
- `IMUDataGenerator`: Generates realistic sensor data with noise/bias
- `generate_dataset()`: Creates complete dataset with ground truth
- `save_dataset_csv()`: Exports to CSV format

**Dependencies:**
- `numpy`: Matrix operations, random number generation
- `scipy`: Quaternion math, SLERP interpolation
- `pandas`: CSV export

**Customization:**
Edit the script to:
- Add new motion scenarios
- Adjust sensor specifications
- Change noise/bias parameters
- Modify sampling rate or duration

---

**Generated by:** Claude Code
**Date:** October 12, 2025
**Project:** SensorFusion 9-Axis IMU Library
