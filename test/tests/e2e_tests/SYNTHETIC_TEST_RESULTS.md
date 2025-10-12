# Synthetic Dataset E2E Test Results

**Date:** October 12, 2025
**Test Executables:** `test_6axis_synthetic`, `test_9axis_synthetic`
**Datasets:** 10 synthetic IMU datasets (16,500 samples total)

---

## Executive Summary

Successfully validated both 6-axis and 9-axis sensor fusion algorithms using synthetic IMU datasets with ground truth. The algorithms show **excellent performance on static and near-static scenarios** but reveal **significant coordinate frame issues during dynamic motion**.

### Key Findings

✅ **Static Performance:** Both algorithms achieve <8° RMSE on static datasets
✅ **Vibration Rejection:** Successfully maintains accuracy during 5 Hz vibration
✅ **9-axis Improvement:** Magnetometer fusion provides ~20% better quaternion accuracy
⚠️ **Dynamic Motion Issues:** Large errors (>75°) during rotation sequences indicate coordinate frame misalignment

---

## Test Results Summary

| Dataset | 6-Axis Pass | 9-Axis Pass | 6-Axis Ang Error | 9-Axis Ang Error | Notes |
|---------|-------------|-------------|------------------|------------------|-------|
| **static_10s.csv** | ✓ PASS | ✓ PASS | 7.06° | 5.52° | Excellent accuracy |
| **rotation_z_30dps_10s.csv** | ✗ FAIL | ✗ FAIL | 88.02° | 88.08° | Coordinate frame issue |
| **rotation_sequence_15s.csv** | ✗ FAIL | ✗ FAIL | 75.70° | 76.01° | Coordinate frame issue |
| **complex_motion_20s.csv** | ✗ FAIL | ✗ FAIL | 98.95° | 98.92° | Coordinate frame issue |
| **vibration_5hz_10s.csv** | ✓ PASS | ✓ PASS | 7.06° | 5.57° | Good vibration rejection |
| **static_high_noise_10s.csv** | Not tested | Not tested | - | - | Future test |
| **static_high_bias_10s.csv** | Not tested | Not tested | - | - | Future test |
| **rotation_x_20dps_10s.csv** | Not tested | Not tested | - | - | Future test |
| **rotation_y_15dps_10s.csv** | Not tested | Not tested | - | - | Future test |
| **static_60s.csv** | Not tested | Not tested | - | - | Future test |

**Summary: 2/5 tests passing for both algorithms (40% pass rate)**

---

## Detailed Results

### 1. Static Calibration Test (static_10s.csv)

**Scenario:** Sensor at rest for 10 seconds (1,000 samples)

#### 6-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  1.815°
  Pitch: 7.834°
  Yaw:   0.613° (expected drift for 6-axis)

QUATERNION ACCURACY:
  Mean Distance:      0.002468
  Mean Angular Error: 7.057°
  Max Angular Error:  13.750°

RESULT: ✓ PASS
```

#### 9-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  1.815°
  Pitch: 6.161°
  Yaw:   0.613° (magnetometer-corrected)

QUATERNION ACCURACY:
  Mean Distance:      0.001580
  Mean Angular Error: 5.520°
  Max Angular Error:  11.747°

RESULT: ✓ PASS
  9-axis fusion shows improved yaw accuracy vs 6-axis
```

**Analysis:**
- Both algorithms perform well on static data
- 9-axis shows **21.8% improvement** in mean angular error (7.06° → 5.52°)
- 9-axis shows **36.0% improvement** in quaternion distance (0.002468 → 0.001580)
- Pitch RMSE improved by **21.4%** with magnetometer (7.83° → 6.16°)

---

### 2. Z-Axis Rotation Test (rotation_z_30dps_10s.csv)

**Scenario:** Constant yaw rotation at 30°/s for 10 seconds (1,000 samples)

#### 6-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  0.399°
  Pitch: 100.658°
  Yaw:   112.743° (expected drift for 6-axis)

QUATERNION ACCURACY:
  Mean Distance:      0.350858
  Mean Angular Error: 88.024°
  Max Angular Error:  179.990°

RESULT: ✗ FAIL (accuracy exceeds tolerance)
```

#### 9-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  0.399°
  Pitch: 100.672°
  Yaw:   112.743° (magnetometer-corrected)

QUATERNION ACCURACY:
  Mean Distance:      0.351169
  Mean Angular Error: 88.083°
  Max Angular Error:  179.662°

RESULT: ✗ FAIL (accuracy exceeds tolerance)
```

**Analysis:**
- **Critical Issue:** Both algorithms fail during rotation
- Roll tracking is excellent (0.4°) but pitch/yaw completely wrong
- Magnetometer does NOT help (errors nearly identical)
- **Root Cause:** Likely coordinate frame mismatch between:
  - Synthetic data ground truth frame
  - Algorithm's internal reference frame
  - Output orientation convention

---

### 3. Smooth Rotation Sequence Test (rotation_sequence_15s.csv)

**Scenario:** SLERP interpolation through multiple orientation waypoints (1,500 samples)

#### 6-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  30.277°
  Pitch: 86.788°
  Yaw:   94.906° (expected drift for 6-axis)

QUATERNION ACCURACY:
  Mean Distance:      0.279190
  Mean Angular Error: 75.703°
  Max Angular Error:  179.880°

RESULT: ✗ FAIL (accuracy exceeds tolerance)
```

#### 9-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  30.284°
  Pitch: 87.109°
  Yaw:   94.911° (magnetometer-corrected)

QUATERNION ACCURACY:
  Mean Distance:      0.280348
  Mean Angular Error: 76.012°
  Max Angular Error:  179.904°

RESULT: ✗ FAIL (accuracy exceeds tolerance)
```

**Analysis:**
- Similar failure pattern as single-axis rotation
- Errors distributed across all axes (roll: 30°, pitch: 87°, yaw: 95°)
- Again, magnetometer provides no improvement
- Confirms systematic coordinate frame issue

---

### 4. Complex Motion Test (complex_motion_20s.csv)

**Scenario:** Multi-axis rotation through 9 waypoints with noise (2,000 samples)

#### 6-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  36.254°
  Pitch: 94.385°
  Yaw:   100.440° (expected drift for 6-axis)

QUATERNION ACCURACY:
  Mean Distance:      0.401385
  Mean Angular Error: 98.951°
  Max Angular Error:  179.933°

RESULT: ✗ FAIL (accuracy exceeds tolerance)
```

#### 9-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  36.289°
  Pitch: 94.465°
  Yaw:   100.438° (magnetometer-corrected)

QUATERNION ACCURACY:
  Mean Distance:      0.401228
  Mean Angular Error: 98.915°
  Max Angular Error:  179.983°

RESULT: ✗ FAIL (accuracy exceeds tolerance)
```

**Analysis:**
- Worst performance of all tests (~99° mean error)
- Errors nearly reach maximum possible (180°)
- Same coordinate frame issue compounded by complex motion
- Need to resolve frame alignment before further testing

---

### 5. Vibration Test (vibration_5hz_10s.csv)

**Scenario:** Sinusoidal vertical acceleration at 5 Hz while static (1,000 samples)

#### 6-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  1.813°
  Pitch: 7.839°
  Yaw:   0.615° (expected drift for 6-axis)

QUATERNION ACCURACY:
  Mean Distance:      0.002471
  Mean Angular Error: 7.061°
  Max Angular Error:  13.760°

RESULT: ✓ PASS
```

#### 9-Axis Results
```
ORIENTATION ACCURACY (RMSE):
  Roll:  1.813°
  Pitch: 6.229°
  Yaw:   0.615° (magnetometer-corrected)

QUATERNION ACCURACY:
  Mean Distance:      0.001611
  Mean Angular Error: 5.573°
  Max Angular Error:  11.837°

RESULT: ✓ PASS
  9-axis fusion shows improved yaw accuracy vs 6-axis
```

**Analysis:**
- **Excellent vibration rejection** - nearly identical to pure static test
- Proves accelerometer filtering is working correctly
- Algorithm successfully distinguishes gravity from linear acceleration
- 9-axis still provides 21% improvement in angular error

---

## Performance Comparison: 6-Axis vs 9-Axis

### Static Scenarios (Passing Tests)

| Metric | 6-Axis | 9-Axis | Improvement |
|--------|--------|--------|-------------|
| Mean Angular Error | 7.06° | 5.55° | **21.4%** |
| Quaternion Distance | 0.00246 | 0.00160 | **35.0%** |
| Pitch RMSE | 7.84° | 6.20° | **20.9%** |
| Max Angular Error | 13.76° | 11.79° | **14.3%** |

**Conclusion:** Magnetometer provides consistent 15-35% accuracy improvement on static/near-static data.

### Dynamic Scenarios (Failing Tests)

| Metric | 6-Axis | 9-Axis | Difference |
|--------|--------|--------|------------|
| Mean Angular Error | 87.6° | 87.7° | +0.1° (no improvement) |
| Quaternion Distance | 0.344 | 0.344 | 0.0% |

**Conclusion:** Magnetometer provides **no benefit** during rotation failures. This confirms the issue is not sensor-related but rather a fundamental coordinate frame problem.

---

## Root Cause Analysis

### Hypothesis: Coordinate Frame Mismatch

The test results strongly suggest a **coordinate frame alignment issue** between:

1. **Synthetic Dataset Convention:**
   - Ground truth generated using `scipy.spatial.transform.Rotation`
   - Standard aerospace convention: Roll-Pitch-Yaw (intrinsic ZYX)
   - Quaternion: `[w, x, y, z]` with scalar-first ordering

2. **Algorithm Output Convention:**
   - Output array: `orientation[3]` = `[pitch, yaw, roll]`
   - Quaternion: `quat = {q0, q1, q2, q3}` (likely scalar-first)

3. **Test Code Mapping:**
   ```c
   double est_roll = output->orientation[2];   // Array index 2
   double est_pitch = output->orientation[0];  // Array index 0
   double est_yaw = output->orientation[1];    // Array index 1
   ```

### Evidence Supporting This Hypothesis:

✅ **Static tests pass** - Small errors indicate axes are approximately correct at rest
✅ **Roll tracking excellent** (0.4°) during Z-rotation - Suggests at least one axis is aligned
✅ **Magnetometer doesn't help** - If it were a sensor fusion issue, mag would improve results
✅ **Errors proportional to motion** - Static: 7°, Rotation: 88°, Complex: 99°
✅ **Errors near 90°** - Suggests axes may be swapped or rotated by 90°

### Possible Issues:

1. **Axis Mapping:** Algorithm may output axes in different order than expected
2. **Rotation Sequence:** Algorithm may use different Euler angle convention (e.g., XYZ vs ZYX)
3. **Reference Frame:** Algorithm may use body frame while test expects earth frame (or vice versa)
4. **Sign Convention:** One or more axes may be inverted

---

## Recommendations

### Immediate Actions (Critical)

1. **Verify Coordinate Frame Conventions**
   - Document algorithm's input/output reference frames
   - Check if algorithm uses NED (North-East-Down) vs ENU (East-North-Up)
   - Verify Euler angle rotation sequence (intrinsic vs extrinsic)

2. **Create Diagnostic Test**
   - Generate dataset with single-axis rotations: 90° about X, Y, Z independently
   - Compare algorithm output to ground truth
   - Identify which axes are swapped/inverted

3. **Update Test Code**
   - Fix axis mapping in `compute_errors()` function
   - Add coordinate frame transformation if needed
   - Re-run all tests with corrected mapping

### Future Testing (After Fix)

1. **Complete Remaining Tests**
   - `static_high_noise_10s.csv` - Validate noise rejection
   - `static_high_bias_10s.csv` - Validate bias compensation
   - `rotation_x_20dps_10s.csv` - Test X-axis rotation
   - `rotation_y_15dps_10s.csv` - Test Y-axis rotation
   - `static_60s.csv` - Long-term drift analysis

2. **Add More Validation**
   - Real-world dataset comparison (RepoIMU, FIUMARG)
   - Cross-validation with known-good implementations
   - Sensitivity analysis for tuning parameters

3. **Performance Benchmarking**
   - Execution time per sample
   - Memory usage profiling
   - CPU utilization under load

---

## Test Infrastructure Status

### What's Working ✅

- CSV parsing and data loading
- Error metric computation (RMSE, quaternion distance, angular error)
- Statistics accumulation and reporting
- Algorithm initialization and execution
- Build system integration

### What Needs Work ⚠️

- **Coordinate frame alignment** (critical blocker)
- Axis mapping verification
- Reference frame documentation
- More comprehensive test coverage

---

## Test Execution Commands

```bash
# Navigate to project root
cd /path/to/SensorFusion_9axis

# Test 6-axis fusion
./build/bin/test_6axis_synthetic test/datasets/synthetic/static_10s.csv
./build/bin/test_6axis_synthetic test/datasets/synthetic/rotation_z_30dps_10s.csv
./build/bin/test_6axis_synthetic test/datasets/synthetic/rotation_sequence_15s.csv
./build/bin/test_6axis_synthetic test/datasets/synthetic/complex_motion_20s.csv
./build/bin/test_6axis_synthetic test/datasets/synthetic/vibration_5hz_10s.csv

# Test 9-axis fusion
./build/bin/test_9axis_synthetic test/datasets/synthetic/static_10s.csv
./build/bin/test_9axis_synthetic test/datasets/synthetic/rotation_z_30dps_10s.csv
./build/bin/test_9axis_synthetic test/datasets/synthetic/rotation_sequence_15s.csv
./build/bin/test_9axis_synthetic test/datasets/synthetic/complex_motion_20s.csv
./build/bin/test_9axis_synthetic test/datasets/synthetic/vibration_5hz_10s.csv
```

---

## Appendix: Test Pass/Fail Criteria

### 6-Axis Fusion (Accel + Gyro)
```c
PASS if:
  - Roll RMSE < 10.0°
  - Pitch RMSE < 10.0°
  - Mean Angular Error < 15.0°
```

### 9-Axis Fusion (Accel + Gyro + Mag)
```c
PASS if:
  - Roll RMSE < 10.0°
  - Pitch RMSE < 10.0°
  - Yaw RMSE < 15.0° (stricter due to magnetometer)
  - Mean Angular Error < 12.0° (stricter than 6-axis)
```

---

## Appendix: Dataset Specifications

| Dataset | Samples | Duration | Motion | Noise Level |
|---------|---------|----------|--------|-------------|
| static_10s.csv | 1,000 | 10s | None | Normal (1x) |
| rotation_z_30dps_10s.csv | 1,000 | 10s | Yaw 30°/s | Normal |
| rotation_x_20dps_10s.csv | 1,000 | 10s | Roll 20°/s | Normal |
| rotation_y_15dps_10s.csv | 1,000 | 10s | Pitch 15°/s | Normal |
| rotation_sequence_15s.csv | 1,500 | 15s | Multi-axis SLERP | Normal |
| vibration_5hz_10s.csv | 1,000 | 10s | 5 Hz vertical | Normal |
| static_high_noise_10s.csv | 1,000 | 10s | None | High (5x) |
| static_high_bias_10s.csv | 1,000 | 10s | None | Normal + 3x bias |
| complex_motion_20s.csv | 2,000 | 20s | 9 waypoints | High (2x) |
| static_60s.csv | 6,000 | 60s | None | Normal |

**Total: 16,500 samples across 10 scenarios**

---

**Report Generated:** October 12, 2025
**Tool:** Claude Code
**Project:** SensorFusion 9-Axis IMU Library
**Status:** ⚠️ Coordinate frame issue identified - requires investigation before further testing
