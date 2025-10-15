# Production Deployment Guide

**SensorFusion 9-Axis Python Implementation**
**Version:** 1.0.0 (Production Ready)
**Date:** 2025-10-14

---

## Quick Start

### Installation

```bash
# Clone the repository
git clone https://github.com/your-username/SensorFusion_9axis.git
cd SensorFusion_9axis

# Install Python dependencies
pip install numpy scipy matplotlib pandas

# Verify installation
python3 pycode/test_integration.py
```

Expected output:
```
======================================================================
SENSOR FUSION INTEGRATION TESTS
======================================================================
...
RESULTS: 7 passed, 0 failed
======================================================================
```

---

## Production Files

### Core Implementation (`pycode/`)

**Essential files for production deployment:**

```
pycode/
├── sensor_fusion_6axis.py         # 6-axis (accel + gyro) EKF implementation
├── sensor_fusion_9axis.py         # 9-axis (accel + gyro + mag) EKF implementation
├── sensor_platform_config.py      # Sensor platform configurations
├── example_usage.py               # Usage examples
└── QuatMath/                      # Quaternion mathematics library
    ├── QuatNormal.py
    ├── Quat2RodMat.py
    ├── QuatProduct.py
    └── (18+ quaternion functions)
```

**Testing & Validation:**

```
pycode/
├── test_integration.py            # Integration tests (7 tests)
├── test_all_synthetic_datasets.py # Comprehensive validation (10 datasets)
├── regression_test.py             # Automated regression detection
└── regression_baseline.json       # Baseline metrics
```

**Optimization Tools (optional):**

```
pycode/
├── optimize_bias_parameters_focused.py    # Parameter optimization
├── parameter_optimization_results_focused.json
└── plot_all_synthetic_datasets.py         # Visualization
```

---

## Basic Usage

### 6-Axis Sensor Fusion (Accelerometer + Gyroscope)

```python
#!/usr/bin/env python3
import numpy as np
from pycode.sensor_fusion_6axis import SensorFusion6Axis
from pycode.sensor_platform_config import SensorPlatform, INVENSENSE

# Initialize platform and fusion
platform = SensorPlatform(INVENSENSE)
sf = SensorFusion6Axis(
    acc_scale=platform.accel_scale_factor,
    gyro_scale=platform.gyro_scale_factor
)

# Sensor data loop
timestamp_ns = 0
dt_ns = 10_000_000  # 10ms = 100Hz

for i in range(1000):
    timestamp_ns += dt_ns

    # Get sensor readings (raw counts from sensor)
    accel_counts = np.array([ax_counts, ay_counts, az_counts])
    gyro_counts = np.array([gx_counts, gy_counts, gz_counts])

    # Preprocess sensor data
    ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp_ns)
    ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp_ns)

    # Run fusion algorithm when gyro is ready
    if ready_gyro & 0x2:
        output = sf.run()

        # Extract results
        yaw_deg = output.orientation[0]
        pitch_deg = output.orientation[1]
        roll_deg = output.orientation[2]

        # Quaternion (w, x, y, z)
        quat = [output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3]

        # Gyro bias estimate (dps)
        bias_x = sf.bias_post_s[0]
        bias_y = sf.bias_post_s[1]
        bias_z = sf.bias_post_s[2]
```

### 9-Axis Sensor Fusion (Accelerometer + Gyroscope + Magnetometer)

```python
#!/usr/bin/env python3
import numpy as np
from pycode.sensor_fusion_9axis import SensorFusion9Axis
from pycode.sensor_platform_config import SensorPlatform, INVENSENSE

# Initialize with optimized parameters (default)
platform = SensorPlatform(INVENSENSE)
sf = SensorFusion9Axis(
    acc_scale=platform.accel_scale_factor,
    gyro_scale=platform.gyro_scale_factor,
    mag_scale=platform.mag_scale_factor,
    # Optimized parameters (defaults - can be omitted)
    motion_threshold_slow=12.0,   # dps
    motion_threshold_fast=45.0,   # dps
    max_bias_rate_moderate=0.07,  # dps/cycle
    max_bias_rate_fast=0.015      # dps/cycle
)

# Sensor data loop
timestamp_ns = 0
dt_ns = 10_000_000  # 10ms = 100Hz

for i in range(1000):
    timestamp_ns += dt_ns

    # Get sensor readings (raw counts)
    accel_counts = np.array([ax_counts, ay_counts, az_counts])
    gyro_counts = np.array([gx_counts, gy_counts, gz_counts])
    mag_counts = np.array([mx_counts, my_counts, mz_counts])

    # Preprocess all sensors
    ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp_ns)
    ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp_ns)
    ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp_ns)

    # Run fusion when gyro is ready
    if ready_gyro & 0x2:
        output = sf.run()

        # Extract orientation (degrees)
        yaw_deg = output.orientation[0]    # Absolute yaw (magnetometer-corrected)
        pitch_deg = output.orientation[1]
        roll_deg = output.orientation[2]
```

---

## Advanced Features

### Motion-Adaptive Bias Rate Limiting

The 9-axis implementation includes optimized motion-adaptive bias limiting:

```python
# Parameters control how aggressively bias estimates converge
sf = SensorFusion9Axis(
    acc_scale=platform.accel_scale_factor,
    gyro_scale=platform.gyro_scale_factor,
    mag_scale=platform.mag_scale_factor,

    # Bias limiting parameters (optimized via grid search)
    motion_threshold_slow=12.0,    # Below: allow full convergence
    motion_threshold_fast=45.0,    # Above: strong limiting
    max_bias_rate_moderate=0.07,   # 12-45 dps: moderate limit
    max_bias_rate_fast=0.015       # >45 dps: strong limit
)
```

**How it works:**
- During **static** or **slow motion** (<12 dps): Full bias convergence allowed
- During **moderate rotation** (12-45 dps): Limited to 0.07 dps/cycle (prevents tracking motion)
- During **fast rotation** (>45 dps): Limited to 0.015 dps/cycle (strong protection)

**Benefits:**
- Prevents tracking motion as gyro bias
- 13.4% error reduction on complex motion
- 44.1% bias reduction on complex motion

---

## Sensor Platform Configuration

### Invensense Platform (MPU9250 + AK8963)

```python
from pycode.sensor_platform_config import SensorPlatform, INVENSENSE

platform = SensorPlatform(INVENSENSE)

# Scale factors (automatically configured)
platform.accel_scale_factor  # 0.00012207 g/count (±2g range)
platform.gyro_scale_factor   # 0.0076336 dps/count (±250 dps range)
platform.mag_scale_factor    # 0.15 µT/count
```

### Freescale Platform (FXOS8700 + FXAS21002 + MAG3110)

```python
from pycode.sensor_platform_config import SensorPlatform, FREESCALE

platform = SensorPlatform(FREESCALE)

# Scale factors
platform.accel_scale_factor  # 0.000244 g/count (±2g range)
platform.gyro_scale_factor   # 0.0625 dps/count (±2000 dps range)
platform.mag_scale_factor    # 0.1 µT/count
```

### Custom Platform

```python
from pycode.sensor_fusion_9axis import SensorFusion9Axis

# Define your own scale factors
sf = SensorFusion9Axis(
    acc_scale=0.000122,   # g/count
    gyro_scale=0.00763,   # dps/count
    mag_scale=0.15        # µT/count
)
```

---

## Performance Characteristics

### Accuracy (Based on 10-dataset validation)

| Scenario | Mean Orientation Error | Bias Convergence | Status |
|----------|----------------------|------------------|--------|
| Static (60s) | 0.577° | <0.5 dps | ✅ Excellent |
| Static (10s) | 2.716° | <1.0 dps | ✅ Good |
| Single-axis rotation (15-30 dps) | 1.2-1.8° | <1.0 dps | ✅ Good |
| Rotation sequence (multi-axis) | 18.9° | 12.8 dps | ⚠️ Acceptable |
| Complex motion | 44.2° | 13.9 dps | ⚠️ Challenging |
| Vibration (5Hz) | 0.787° | <0.6 dps | ✅ Excellent |

### Computational Performance

- **Update rate:** ~100-200 Hz on modern hardware
- **Convergence time:** ~2 seconds (200 samples @ 100Hz)
- **Memory usage:** ~3 KB per instance

---

## Validation & Testing

### Run Integration Tests

```bash
cd /path/to/SensorFusion_9axis
python3 pycode/test_integration.py
```

Tests verify:
- ✅ Platform configuration
- ✅ 6-axis and 9-axis initialization
- ✅ Static convergence
- ✅ Motion-adaptive bias rate limiting
- ✅ Quaternion normalization

### Run Comprehensive Validation

```bash
python3 pycode/test_all_synthetic_datasets.py
```

Validates on 10 synthetic datasets:
- Static scenarios (various bias/noise levels)
- Single-axis rotations (X, Y, Z)
- Multi-axis rotation sequences
- Complex motion patterns
- Vibration scenarios

### Run Regression Tests

```bash
python3 pycode/regression_test.py
```

Checks for regressions against baseline metrics.

---

## Known Limitations

### 1. Rotation Sequence Accuracy

**Scenario:** Rapid multi-axis rotation (15+ dps on multiple axes)
**Issue:** Mean orientation error ~18.9° during complex rotation sequences
**Root Cause:** EKF small-angle linearization breaks down during rapid multi-axis rotation
**Impact:** Medium - Affects aggressive maneuvers
**Mitigation:** Consider UKF or IMM algorithms for high-dynamics applications
**Status:** Acceptable for typical IMU use cases

### 2. Complex Motion Performance

**Scenario:** Arbitrary 3D motion with high dynamics
**Issue:** Mean error ~44° on most challenging scenarios
**Root Cause:** Combination of centripetal acceleration and rotation invalidating accelerometer measurements
**Impact:** Medium - Only affects extreme motion profiles
**Mitigation:** Use application-specific motion constraints or gating
**Status:** Acceptable - Most applications don't require this level of performance

### 3. Coordinate Frame Convention

**Important:** Output orientation is `[Yaw, Pitch, Roll]` not `[Roll, Pitch, Yaw]`

```python
output = sf.run()
yaw = output.orientation[0]    # Rotation around Z-axis
pitch = output.orientation[1]  # Rotation around Y-axis
roll = output.orientation[2]   # Rotation around X-axis
```

This is **intentional** and consistent with the C implementation.

---

## Troubleshooting

### Problem: Large orientation errors after initialization

**Cause:** Sensor not stabilized before fusion starts
**Solution:** Wait for sensor stabilization (>1 second) before processing data

```python
# Wait for stabilization
for i in range(100):  # 1 second @ 100Hz
    sf.preprocess_sensor_data(0, accel_counts, timestamp)
    sf.preprocess_sensor_data(1, gyro_counts, timestamp)
    # Don't use output yet
```

### Problem: Bias estimates growing unbounded

**Cause:** Incorrect sensor scale factors
**Solution:** Verify scale factors match your sensor specifications

```python
# Check your sensor's datasheet
# For MPU9250:
#   Gyro ±250 dps → 131 LSB/dps → scale = 1/131 = 0.00763 dps/count
#   Accel ±2g → 16384 LSB/g → scale = 1/16384 = 0.000061 g/count
```

### Problem: Yaw drifting on 9-axis

**Cause:** Magnetic disturbances (nearby metal, electronics)
**Solution:** Calibrate magnetometer or disable mag updates in disturbed environments

```python
# Check magnetic field magnitude
mag_magnitude = np.linalg.norm(sf.mag_data.count_avg)
# Should be ~45-65 µT for Earth's field
# Much lower/higher → magnetic disturbance present
```

---

## Migration from C Implementation

### Key Differences

1. **Object-oriented API:**
   - C: `sf_9xagm_algo_init()`, `sf_9xagm_algo_run()`
   - Python: `sf = SensorFusion9Axis()`, `output = sf.run()`

2. **No explicit memory management:**
   - C: Must call `sf_9xagm_algo_stop()`
   - Python: Automatic garbage collection

3. **NumPy arrays vs C arrays:**
   - C: `double accel[3]`
   - Python: `accel = np.array([ax, ay, az])`

### API Mapping

| C Function | Python Equivalent |
|------------|-------------------|
| `sf_9xagm_algo_init()` | `SensorFusion9Axis()` |
| `sf_9xagm_data_preproc()` | `.preprocess_sensor_data()` |
| `sf_9xagm_algo_run()` | `.run()` |
| `sf_9xagm_algo_stop()` | (automatic) |

---

## Optimization History

The current implementation includes optimizations from iterative improvement:

### Iteration 1: Motion-Adaptive Bias Rate Limiting ✅ ENABLED
- **Goal:** Prevent tracking motion as gyro bias
- **Method:** Three-tier threshold system (slow/moderate/fast motion)
- **Results:** -13.4% error, -44.1% bias on complex motion

### Phase 1: Grid Search Optimization ✅ ENABLED
- **Goal:** Find optimal bias limiting parameters
- **Method:** Tested 11 parameter combinations
- **Results:** Best parameters: slow=12.0, fast=45.0, moderate_rate=0.07, fast_rate=0.015

### Iteration 2: Adaptive Process Noise Scaling ❌ DISABLED
- **Goal:** Improve rotation sequence accuracy
- **Method:** Motion-dependent process noise scaling
- **Results:** No benefit, caused regressions
- **Status:** Code present but disabled (for reference)

See `pycode/ITERATION1_RESULTS.md`, `pycode/PHASE1_OPTIMIZATION_RESULTS.md`, and `pycode/ITERATION2_RESULTS.md` for complete details.

---

## Support & Documentation

### Documentation Files

- **This file:** Production deployment guide
- **`README.md`:** Complete project documentation
- **`pycode/archive/README.md`:** Archived development scripts
- **`pycode/IMPROVEMENT_ROADMAP.md`:** Optimization strategy
- **`test/README.md`:** C test suite documentation

### Getting Help

1. Check the integration tests: `pycode/test_integration.py`
2. Review example usage: `pycode/example_usage.py`
3. See performance reports: `pycode/synthetic_test_results/`
4. Consult optimization docs: `pycode/ITERATION1_RESULTS.md`

---

## Version History

**v1.0.0** (2025-10-14) - Production Release
- ✅ Motion-adaptive bias rate limiting
- ✅ Optimized parameters via grid search
- ✅ Comprehensive test suite (7 integration tests passing)
- ✅ 10-dataset validation complete
- ✅ Clean production codebase (60+ debug scripts archived)
- ✅ Complete documentation

---

## License

MIT License - See LICENSE file for details
