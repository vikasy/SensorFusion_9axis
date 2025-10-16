# C/Python Synchronized Debugging Framework - Quick Start

## Overview

This framework enables systematic debugging of C sensor fusion implementation by comparing it against the known-good Python implementation using identical test data.

## Purpose

When a C test fails:
1. Export the exact test data that caused the failure
2. Run Python implementation with same data (known-good)
3. Compare internal states sample-by-sample
4. Find exact divergence point
5. Root cause the C bug

## Tools Created

### 1. Python Debug Framework (`pycode/debug_compare.py`)

**Features:**
- Detailed state logging at every sample
- Stop at specific sample number
- JSON output for automated comparison
- Verbose console output

**Usage:**
```python
python3 pycode/debug_compare.py

# Or programmatically:
from debug_compare import run_python_with_debug, generate_simple_test_data

test_data = generate_simple_test_data(num_samples=100)
sf, logger = run_python_with_debug(test_data, fusion_type='6axis', verbose=True)
```

### 2. Test Data Exporter (`test/scripts/export_test_data.py`)

Converts C test output to Python-compatible JSON format.

**Usage:**
```bash
python3 test/scripts/export_test_data.py \
    build/c_test_output.txt \
    pycode/c_test_data.json
```

### 3. Comprehensive Documentation (`doc/DEBUG_FRAMEWORK.md`)

Complete step-by-step guide with examples.

## Quick Start Example

### Step 1: Run C Test

```bash
# Run C test and capture output
./build/bin/test_6axis_fusion > c_output.txt 2>&1

# Check for failures
grep "ERROR:" c_output.txt
# Output: ERROR: at sample 42
```

### Step 2: Add Debug Logging to C Test

Edit your C test file to add:

```c
// Print sensor data before feeding to algorithm
printf("SENSOR_DATA,%d,%d,%d,%d,%d,%d,%d,0,0,0,%llu\n",
       i, acc[0], acc[1], acc[2],
       gyro[0], gyro[1], gyro[2], timestamp);

// Print state after update
printf("STATE,%d,AFTER,quat,[%.6f,%.6f,%.6f,%.6f],orient,[%.3f,%.3f,%.3f]\n",
       i, state->quat_post.q0, state->quat_post.q1,
       state->quat_post.q2, state->quat_post.q3,
       state->algo_output.orientation_S[0],
       state->algo_output.orientation_S[1],
       state->algo_output.orientation_S[2]);
```

### Step 3: Export Test Data

```bash
python3 test/scripts/export_test_data.py c_output.txt pycode/c_test_data.json
```

### Step 4: Run Python with Same Data

```python
import json, numpy as np
from debug_compare import run_python_with_debug

# Load test data
with open('c_test_data.json') as f:
    data = json.load(f)

test_data = {k: np.array(v) for k, v in data.items()}

# Run with debug logging - stop at failure point
sf, logger = run_python_with_debug(
    test_data,
    fusion_type='6axis',
    stop_at_sample=42,  # Where C failed
    verbose=True
)
```

### Step 5: Compare Outputs

Look at the debug logs side-by-side:
- `c_output.txt` - C state at each sample
- `debug_python_6axis.json` - Python state at each sample

Find where they diverge and examine:
- Input data (should be identical)
- Quaternion values
- Gyro bias
- Operation mode
- Timestamps

## What the Framework Logs

For each sample, both BEFORE and AFTER update:
- **Timestamps**: nom_updt_ts, meas_updt_ts
- **Quaternion**: [q0, q1, q2, q3]
- **Orientation**: [yaw, pitch, roll] in degrees
- **Gyro Bias**: [bx, by, bz] in rad/s
- **Linear Acceleration**: [ax, ay, az] in m/s²
- **Gravity**: [gx, gy, gz] in m/s²
- **Status**: orientation_init, operation_mode
- **Sensor Data**: acc, gyro, mag with timestamps

## Example Output

```
======================================================================
Sample #42 - BEFORE_UPDATE
======================================================================
Sensor Data:
  ACC: [    0.000,     0.000, -16384.000] @ ts=420000000
  GYRO: [    0.000,     0.000,     0.000] @ ts=420000000

Quaternion:
  [q0=1.000000, q1=0.000000, q2=-0.000000, q3=0.000000]

Orientation (deg):
  Yaw=   0.000° Pitch=   0.000° Roll=   0.000°

Gyro Bias (rad/s):
  [  0.000000,   0.000000,   0.000000]

Status:
  Orient Init: True
  Op Mode: 0x0000
```

## Files Structure

```
SensorFusion_9axis/
├── pycode/
│   ├── debug_compare.py              # Main Python debug framework
│   ├── sensor_fusion_6axis.py        # Known-good 6-axis implementation
│   ├── sensor_fusion_9axis.py        # Known-good 9-axis implementation
│   └── test_sensor_fusion.py         # Verified tests (all passing)
├── test/scripts/
│   └── export_test_data.py           # C test data exporter
├── doc/
│   └── DEBUG_FRAMEWORK.md            # Comprehensive documentation
└── DEBUG_README.md                   # This file
```

## Common Debugging Scenarios

### Scenario 1: Quaternion Diverges

**Symptom:** Python and C quaternions different after sample N

**Check:**
1. Are input sensor values identical?
2. Are timestamps identical (watch for unit mismatches)?
3. Is dt calculation correct?
4. Is quaternion integration formula correct?
5. Is quaternion normalized after integration?

### Scenario 2: Orientation Angles Wrong

**Symptom:** Quaternion similar but angles different

**Check:**
1. Quaternion to rotation matrix conversion
2. Rotation matrix to Euler angle conversion
3. Gimbal lock handling
4. Angle wrapping (-180 to +180)

### Scenario 3: Bias Estimation Issues

**Symptom:** Gyro bias growing incorrectly

**Check:**
1. Kalman gain computation
2. Measurement innovation
3. Covariance update
4. Process noise parameters

## Next Steps

1. **Identify failing C test**
   ```bash
   cd build
   ctest --verbose
   ```

2. **Add debug logging to that test** (see doc/DEBUG_FRAMEWORK.md)

3. **Run test and export data**
   ```bash
   ./bin/test_XXX > output.txt 2>&1
   python3 ../test/scripts/export_test_data.py output.txt ../pycode/test_data.json
   ```

4. **Run Python with same data**
   ```bash
   cd pycode
   python3 -c "
   import json, numpy as np
   from debug_compare import run_python_with_debug
   with open('test_data.json') as f: data = json.load(f)
   test_data = {k: np.array(v) for k, v in data.items()}
   run_python_with_debug(test_data, fusion_type='6axis', verbose=True)
   "
   ```

5. **Compare and fix C code**

6. **Repeat for next failing test**

## Support

- Full documentation: `doc/DEBUG_FRAMEWORK.md`
- Python implementation: `pycode/README.md`
- C implementation: `code/algo/`

---

**Status:** Framework complete and tested ✅
**Date:** 2025-10-12
