# RepoIMU Dataset - Complete Reference Documentation

**Date:** 2025-10-15
**Author:** Vikas Yadav
**Status:** ⚠️ DATA INTEGRITY ISSUES FOUND

---

## Table of Contents

1. [Dataset Overview](#dataset-overview)
2. [Official Sources](#official-sources)
3. [Sensor Specifications](#sensor-specifications)
4. [Coordinate Frame Definitions](#coordinate-frame-definitions)
5. [Data Format](#data-format)
6. [Critical Issues Found](#critical-issues-found)
7. [Comparison with Synthetic Data](#comparison-with-synthetic-data)
8. [Usage Recommendations](#usage-recommendations)
9. [References](#references)

---

## Dataset Overview

**RepoIMU**: Reference Data Set for Accuracy Evaluation of Orientation Estimation Algorithms for Inertial Motion Capture Systems

### Purpose
- Validate IMU-based orientation estimation algorithms
- Provide high-accuracy ground truth from optical motion capture
- Enable standardized algorithm benchmarking

### Content
- **Two experimental setups:**
  1. **T-stick (Wand)**: 29 trials, ~90 seconds each
  2. **3-segment pendulum**: Multiple trials with complex motion

### Data Provided
- Synchronized IMU sensor readings (100 Hz)
- Vicon optical motion capture ground truth (100 Hz)
- Full range of orientation changes
- Various acceleration profiles

---

## Official Sources

### Primary Repository
- **GitHub**: https://github.com/agnieszkaszczesna/RepoIMU
- **Maintainer**: Agnieszka Szczęsna (Silesian University of Technology)
- **License**: Open for research use

### Research Paper
**Citation:**
```
Szczęsna, A., Skurowski, P., Pruszowski, P., Pęszor, D., Paszkuta, M., Wojciechowski, K. (2016).
Reference Data Set for Accuracy Evaluation of Orientation Estimation Algorithms for Inertial Motion Capture Systems.
In: Chmielewski, L., Datta, A., Kozera, R., Wojciechowski, K. (eds)
Computer Vision and Graphics. ICCVG 2016.
Lecture Notes in Computer Science, vol 9972. Springer, Cham.
```

**DOI**: https://doi.org/10.1007/978-3-319-46418-3_45

### Dataset Access
- **Direct download**: Available from GitHub repository
- **Mirror**: Research Gate (https://www.researchgate.net/publication/307076277)
- **Institution**: Polish-Japanese Academy of Information Technology

---

## Sensor Specifications

### IMU Hardware: XSens MTi Series

**⚠️ IMPORTANT CORRECTION:**
- The sensor is **XSens MTi** (NOT MPU9250 as previously documented in `E2E_TEST_RESULTS.md`)
- XSens MTi is a professional-grade 9-axis MARG sensor

### XSens MTi Components

#### 3-Axis Accelerometer
- **Technology**: MEMS capacitive
- **Default Range**: ±16g (configurable)
- **Other ranges**: ±2g, ±4g, ±8g
- **Output**: Calibrated acceleration in m/s²
- **Noise**: ~0.002 m/s²/√Hz
- **Bias stability**: ~0.004 m/s² (1σ)
- **Temperature compensated**: Yes

#### 3-Axis Gyroscope
- **Technology**: MEMS vibratory
- **Default Range**: ±2000 deg/s (configurable)
- **Other ranges**: ±125, ±250, ±500, ±1000 deg/s
- **Output**: Calibrated angular rate in deg/s or rad/s
- **Noise**: ~0.01 deg/s/√Hz (typical)
- **Bias stability**: ~10 deg/h (1σ)
- **Temperature compensated**: Yes

#### 3-Axis Magnetometer
- **Technology**: AMR (Anisotropic Magneto-Resistive)
- **Range**: ±800 µT (±8 Gauss)
- **Output**: Calibrated magnetic field in µT or mG
- **Noise**: ~0.5 mG RMS
- **Resolution**: 0.5 mG

#### Additional Features
- **Onboard processor**: Low-power MCU
- **Crystal**: High-accuracy timing reference
- **Calibration**: Factory-calibrated (hard/soft iron, temperature)
- **Sample rate**: Configurable up to 400 Hz (dataset uses 100 Hz)

---

## Coordinate Frame Definitions

### XSens MTi Coordinate System

**Default Frame: ENU (East-North-Up)**

```
     Z (Up)
     ↑
     |
     |
     +----→ X (East)
    /
   /
  ↙
 Y (North)
```

**Axes Definition:**
- **X-axis**: Points East (right when facing North)
- **Y-axis**: Points North (forward)
- **Z-axis**: Points Up (vertical, away from Earth)
- **Handedness**: Right-handed coordinate system

**Alternative Output Options:**
- NED (North-East-Down)
- NWU (North-West-Up)

### Vicon Optical Motion Capture System

**Frame**: Depends on calibration setup
- Typically uses laboratory frame (arbitrary origin)
- 6 reflective markers define rigid body
- Sub-millimeter accuracy
- Outputs quaternion orientation

### ⚠️ Critical Finding: Frame Misalignment

**From Szczęsna et al. (2016) paper:**
> "The coordinate system of IMU and ground truth are not aligned."

**Implications:**
1. **Direct comparison is NOT valid** without alignment
2. **Calibration step required** (Section 4.2 in paper)
3. **Method proposed**: Quaternion averaging for partial alignment
4. **Two rotations needed**: Paper addresses one, user must handle second

**This means:**
- Vicon quaternions are in Vicon lab frame
- IMU quaternions would be in sensor (ENU) frame
- A rotation transform R_vicon_to_imu is needed
- Transform must be determined per dataset/setup

---

## Data Format

### Files in This Directory

1. **TStick_Test01_Static.csv**
   - Duration: ~181 seconds (18,156 samples)
   - Motion: Static calibration data
   - Purpose: Bias estimation, noise characterization

2. **TStick_Test02_Trial1.csv**
   - Duration: ~90 seconds (8,995 samples)
   - Motion: Dynamic rotation and translation
   - Purpose: Algorithm validation under motion

### CSV Structure

**Header Row 1:**
```
Time (s);Vicon Orientation;;;;IMU Acceleration;;;IMU Gyroscope;;;IMU Magnetometer;;;
```

**Header Row 2:**
```
;W;X;Y;Z;X;Y;Z;X;Y;Z;X;Y;Z;
```

**Data Columns:**

| Column | Name | Units | Description |
|--------|------|-------|-------------|
| 1 | Time | seconds | Timestamp (synchronized) |
| 2 | Vicon_qW | - | Quaternion W (scalar) component |
| 3 | Vicon_qX | - | Quaternion X component |
| 4 | Vicon_qY | - | Quaternion Y component |
| 5 | Vicon_qZ | - | Quaternion Z component |
| 6 | IMU_accX | m/s² | Acceleration X-axis (ENU frame) |
| 7 | IMU_accY | m/s² | Acceleration Y-axis (ENU frame) |
| 8 | IMU_accZ | m/s² | Acceleration Z-axis (ENU frame) |
| 9 | IMU_gyroX | deg/s or rad/s | Angular rate X-axis |
| 10 | IMU_gyroY | deg/s or rad/s | Angular rate Y-axis |
| 11 | IMU_gyroZ | deg/s or rad/s | Angular rate Z-axis |
| 12 | IMU_magX | µT or mG | Magnetic field X-axis |
| 13 | IMU_magY | µT or mG | Magnetic field Y-axis |
| 14 | IMU_magZ | µT or mG | Magnetic field Z-axis |

**Delimiter**: Semicolon (`;`)

### Data Characteristics

**Sample Rate**: 100 Hz (0.01 s period)

**Synchronization**:
- Vicon and IMU data are time-synchronized
- Timestamps align within ±5 ms (typical)

**Quaternion Convention**:
- Format: [W, X, Y, Z] (scalar-first)
- Normalized: ||q|| = 1
- Represents rotation from reference frame to body frame

---

## Critical Issues Found

### ⚠️ Issue #1: Zero Quaternions in Ground Truth

**Observation:**
All Vicon quaternion values in both CSV files are **[0, 0, 0, 0]**.

**Example from TStick_Test01_Static.csv:**
```
Time (s);W;X;Y;Z;...
0.01;0;0;0;0;...
0.02;0;0;0;0;...
0.03;0;0;0;0;...
...
```

**Expected:**
Identity quaternion should be **[1, 0, 0, 0]**, not zeros.

**Analysis:**
- ✗ Quaternion magnitude: ||[0,0,0,0]|| = 0 (invalid!)
- ✓ Should be: ||[1,0,0,0]|| = 1 (unit quaternion)
- **Conclusion**: Data is corrupted or incorrectly extracted

**Possible Causes:**
1. **File format conversion error**: Original RepoIMU data may use different CSV format
2. **Column mismatch**: Quaternion values may be in different columns
3. **Download corruption**: Files may have been truncated or corrupted
4. **Extraction script bug**: Conversion from original format failed

**Impact:**
- ❌ **Cannot use these files for validation** without fixing quaternions
- ❌ Ground truth is completely invalid
- ❌ All comparison tests will fail

### ⚠️ Issue #2: Unknown Gyroscope Units

**Observation:**
Gyroscope values range from approximately -0.01 to +0.01.

**Analysis:**
- If **rad/s**: Very small rotation rates (~0.6 deg/s max) ✓ Plausible for static data
- If **deg/s**: Extremely small rates (~0.01 deg/s) ✓ Also plausible for static

**Typical values in TStick_Test01_Static.csv:**
```
IMU_gyroX: -0.012684
IMU_gyroY: -0.008069
IMU_gyroZ: 0.005511
```

**Conclusion**: Likely **rad/s** based on XSens MTi default output, but **needs verification**.

### ⚠️ Issue #3: Unknown Magnetometer Units

**Observation:**
Magnetometer values range from approximately -0.9 to +0.4.

**Analysis:**
- If **µT**: Extremely weak field (~0.9 µT vs Earth's ~50 µT) ✗ Implausible
- If **mG**: Reasonable field strength (~9 mG = ~0.9 µT) ✗ Still too weak
- If **normalized**: Values are mag/||mag|| ✓ Possible
- If **Gauss**: 0.9 Gauss = 90 µT ✓ Plausible

**Typical values in TStick_Test01_Static.csv:**
```
IMU_magX: -0.027895
IMU_magY: 0.39902
IMU_magZ: -0.89274
```

**Magnitude**: sqrt(0.028² + 0.399² + 0.893²) ≈ 0.978

**Conclusion**: Likely **normalized magnetic field** (unitless), but **needs verification**.

### ⚠️ Issue #4: Coordinate Frame Mismatch

**Your Synthetic Data**: NED (North-East-Down)
- X: North
- Y: East
- Z: Down
- Gravity: [0, 0, +9.81] m/s²

**RepoIMU XSens MTi**: ENU (East-North-Up)
- X: East
- Y: North
- Z: Up
- Gravity: [0, 0, -9.81] m/s²

**Transform Required:**
```python
# NED to ENU conversion:
acc_ENU = [acc_NED[1], acc_NED[0], -acc_NED[2]]
# X_enu = Y_ned (East)
# Y_enu = X_ned (North)
# Z_enu = -Z_ned (Down → Up)
```

**Quaternion Transform:**
```python
# Rotation from NED to ENU:
R_ned_to_enu = [[0, 1, 0],
                [1, 0, 0],
                [0, 0, -1]]
q_enu = rotate_quaternion(q_ned, R_ned_to_enu)
```

---

## Comparison with Synthetic Data

### Coordinate Frame

| Property | Synthetic Data | RepoIMU Data |
|----------|---------------|--------------|
| **Convention** | NED | ENU |
| **X-axis** | North | East |
| **Y-axis** | East | North |
| **Z-axis** | Down | Up |
| **Gravity** | [0,0,+9.81] | [0,0,-9.81] |
| **Status** | ✓ Consistent | ✗ **MISMATCH** |

### Sensor Hardware

| Property | Synthetic (MPU9250) | RepoIMU (XSens MTi) |
|----------|---------------------|---------------------|
| **Accelerometer** | ±4g, 8192 counts/g | ±16g, m/s² output |
| **Gyroscope** | ±1000 dps, 32.768 counts/dps | ±2000 dps, deg/s output |
| **Magnetometer** | ±4800 µT, 6.8 counts/µT | ±800 µT, µT output |
| **Output** | **Raw counts (int16)** | **Calibrated physical units** |
| **Calibration** | None (raw) | **Factory calibrated** |
| **Status** | ✗ **DIFFERENT SENSOR** | ✗ **DIFFERENT FORMAT** |

### Data Format

| Property | Synthetic Data | RepoIMU Data |
|----------|---------------|--------------|
| **Accel format** | Counts (int16) | m/s² (float) |
| **Gyro format** | Counts (int16) | rad/s or deg/s (float) |
| **Mag format** | Counts (int16) | Normalized? (float) |
| **GT format** | [w,x,y,z] ✓ | [0,0,0,0] ✗ INVALID |
| **Frame alignment** | ✓ Aligned | ✗ **NOT ALIGNED** |
| **Status** | ✓ Ready to use | ✗ **NEEDS CONVERSION** |

### Data Integrity

| Dataset | GT Quaternions | Sensor Data | Usability |
|---------|---------------|-------------|-----------|
| **Synthetic** | ✓ Valid [1,0,0,0] | ✓ Consistent | ✅ **READY** |
| **RepoIMU** | ✗ Invalid [0,0,0,0] | ? Unknown units | ❌ **BROKEN** |

---

## Usage Recommendations

### ❌ Current Status: NOT USABLE

The RepoIMU dataset files in this directory **cannot be used** for validation in their current state due to:

1. **Invalid ground truth quaternions** (all zeros)
2. **Unknown sensor units** (gyro, mag)
3. **Coordinate frame mismatch** (ENU vs NED)
4. **Sensor mismatch** (XSens vs MPU9250)
5. **Missing alignment calibration** (IMU ↔ Vicon)

### ✅ Recommended Path Forward

#### Option 1: Fix RepoIMU Data (High Effort)

**Steps:**
1. **Re-download original data** from https://github.com/agnieszkaszczesna/RepoIMU
2. **Verify file format** - check if quaternions are in different columns/format
3. **Extract correctly** - ensure proper CSV parsing
4. **Determine units**:
   - Check gyro: multiply sample values to see if they match expected rotation
   - Check mag: compute magnitude, should be ~0.5 Gauss (50 µT)
5. **Apply coordinate transform** (ENU → NED):
   ```python
   acc_ned = [acc_enu[1], acc_enu[0], -acc_enu[2]]
   gyro_ned = [gyro_enu[1], gyro_enu[0], -gyro_enu[2]]
   mag_ned = [mag_enu[1], mag_enu[0], -mag_enu[2]]
   ```
6. **Convert to counts**:
   - Use MPU9250 scale factors from synthetic data generator
   - `accel_counts = (accel_m_s2 / 9.81) * 8192`
   - `gyro_counts = gyro_dps * 32.768`
   - `mag_counts = mag_uT * 6.8`
7. **Apply Vicon-to-IMU alignment** (from paper Section 4.2)
8. **Validate** against known motion (e.g., 90° rotation should show 90° change)

**Estimated effort**: 2-4 hours

#### Option 2: Use Synthetic Data (Low Effort - RECOMMENDED)

**Rationale:**
- ✅ Synthetic datasets are **proven to work** (4.786° mean error)
- ✅ **Internally consistent** coordinate frames
- ✅ **Correct sensor specifications** (MPU9250)
- ✅ **Valid ground truth** quaternions
- ✅ **10 diverse scenarios** (static, rotation, complex, vibration)

**Recommendation:**
1. **Continue parameter optimization** with synthetic data
2. **Achieve target performance** (< 5° error) on synthetic first
3. **Later**: Fix RepoIMU or collect real hardware data
4. **Final validation**: Real IMU hardware testing

**Estimated effort**: 0 hours (already working)

#### Option 3: Collect New Reference Data (Medium Effort)

**Approach:**
1. **Use actual MPU9250 hardware** (matches your code)
2. **Collect controlled motion data** (known rotations)
3. **Use C implementation as reference** for Python validation
4. **Or**: Use external AHRS algorithm as ground truth

**Estimated effort**: 1-2 days (includes hardware setup)

### 🎯 Immediate Action

**For current testing needs:**
```python
# DO NOT USE RepoIMU data yet
# CONTINUE WITH SYNTHETIC DATA:

# Working dataset:
dataset = "test/data/datasets/synthetic/rotation_sequence_15s.csv"

# Test command:
cd pycode/test
python test_integration.py  # Uses synthetic data
```

**For future reference:**
- Document RepoIMU issues (✅ Done - this file)
- Add to backlog: "Fix RepoIMU dataset extraction"
- Prioritize after synthetic data optimization complete

---

## References

### Papers

1. **Szczęsna, A., et al. (2016)**
   "Reference Data Set for Accuracy Evaluation of Orientation Estimation Algorithms for Inertial Motion Capture Systems"
   International Conference on Computer Vision and Graphics (ICCVG 2016)
   Springer LNCS vol 9972
   https://doi.org/10.1007/978-3-319-46418-3_45

2. **XSens MTi Documentation**
   "MTi Family Reference Manual"
   XSens Technologies B.V. / Movella
   https://www.xsens.com/hubfs/Downloads/Manuals/MTi_familyreference_manual.pdf

3. **XSens MTi-1 Series Datasheet**
   "IMU, VRU, AHRS and GNSS/INS module"
   https://www.xsens.com/hubfs/Downloads/Manuals/MTi-1-series-datasheet.pdf

### Online Resources

- **RepoIMU GitHub**: https://github.com/agnieszkaszczesna/RepoIMU
- **XSens Products**: https://www.movella.com/products/sensor-modules
- **Vicon Motion Capture**: https://www.vicon.com/

### Related Documentation

- **Local**: `test/data/DATASETS_GUIDE.md` - Dataset organization
- **Local**: `test/tests/e2e_tests/E2E_TEST_RESULTS.md` - Test results (contains errors - XSens ≠ MPU9250)
- **Local**: `pycode/test/REALISTIC_DATA_FINDINGS.md` - Analysis of synthetic "realistic" datasets

---

## Revision History

| Date | Version | Author | Changes |
|------|---------|--------|---------|
| 2025-10-15 | 1.0 | Vikas Yadav | Initial documentation - comprehensive investigation results |

---

## Contact

**Dataset Issues**: Contact dataset maintainers at https://github.com/agnieszkaszczesna/RepoIMU/issues

**This Documentation**: Part of SensorFusion_9axis project
**Location**: `test/data/datasets/repoimu/REPOIMU_DATASET_INFO.md`

---

**END OF DOCUMENT**
