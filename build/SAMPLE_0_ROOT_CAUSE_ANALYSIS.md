# Sample 0 Root Cause Analysis - C vs Python Divergence

**Date:** 2025-10-12
**Dataset:** test_input_output_0922.h
**Sample:** Sample 0 (first output, timestamp 72146875)

---

## Executive Summary

**ROOT CAUSE IDENTIFIED:** Python implementation outputs identity quaternion at sample 0, while C implementation outputs properly initialized quaternion with 3.8° rotation.

**The key difference:** Both implementations claim `orient_init = True` at sample 0, but:
- **C implementation:** Has already computed tilt initialization and applied it to the quaternion
- **Python implementation:** Sets `orient_init = True` but quaternion remains identity [1, 0, 0, 0]

---

## Sample 0 Side-by-Side Comparison

### Input Data (IDENTICAL)
```
Input sample index: 0
Timestamp: 72146875
Sensor ID: 0 (ACC, 1=GYRO, 2=MAG)
Sensor counts: [-51, -529, 8018]
```

### Output Quaternion (DIFFERENT)

| Component | C Implementation | Python Implementation | Difference |
|-----------|-----------------|----------------------|------------|
| q0 | 0.9994524717 | 1.0000000000 | -0.0005476 |
| q1 | -0.0329344124 | 0.0000000000 | -0.0329344 |
| q2 | 0.0031716754 | 0.0000000000 | 0.0031717 |
| q3 | 0.0001045145 | 0.0000000000 | 0.0001045 |

### Output Orientation (DIFFERENT)

| Angle | C Implementation | Python Implementation | Difference |
|-------|-----------------|----------------------|------------|
| Roll  | -3.774707° | 0.000000° | -3.775° |
| Pitch | 0.363645° | -0.000000° | 0.364° |
| Yaw   | 360.000000° | 0.000000° | 0.000° |

### Linear Acceleration (IDENTICAL)

| Component | C Implementation | Python Implementation |
|-----------|-----------------|----------------------|
| ax | 0.000000 | 0.000000 |
| ay | 0.000000 | 0.000000 |
| az | 0.000000 | 0.000000 |

### Gravity (DIFFERENT)

| Component | C Implementation | Python Implementation | Difference |
|-----------|-----------------|----------------------|------------|
| gx | 0.062198 | -0.000000 | 0.062198 |
| gy | 0.645155 | -0.000000 | 0.645155 |
| gz | -9.778543 | -9.806650 | 0.028107 |

### Internal State (DIFFERENT)

| Variable | C Implementation | Python Implementation | Notes |
|----------|-----------------|----------------------|-------|
| orient_init | (not visible in C output) | True | Python shows orientation initialized |
| acc_count | (not visible in C output) | 1 | Python has buffered 1 accelerometer sample |
| gyro_count | (not visible in C output) | 0 | Python has no gyroscope samples yet |
| nom_updt_ts | (not visible in C output) | 0 | Python has not run time update yet |
| meas_updt_ts | (not visible in C output) | 72146875 | Python shows measurement update at this timestamp |

---

## Key Observations

### 1. Quaternion Analysis

**C quaternion:** [0.9994524717, -0.0329344124, 0.0031716754, 0.0001045145]
- Norm: √(0.9994^2 + 0.0329^2 + 0.0032^2 + 0.0001^2) ≈ 1.0000 ✓
- Represents rotation from initial accelerometer reading
- Roll = -3.775°, Pitch = 0.364°

**Python quaternion:** [1.0000000000, 0.0000000000, 0.0000000000, 0.0000000000]
- Identity quaternion (no rotation)
- Roll = 0°, Pitch = 0°

### 2. Gravity Vector Analysis

**Expected gravity from accelerometer:**
- Raw counts: [-51, -529, 8018]
- Scale: 1/16384 counts/g
- Physical: [-0.003113, -0.032288, 0.489563] g
- Magnitude: √(0.003113² + 0.032288² + 0.489563²) ≈ 0.4906 g

**C gravity output:**
- [0.062198, 0.645155, -9.778543] m/s²
- Magnitude: √(0.062² + 0.645² + 9.779²) ≈ 9.8 m/s² ✓
- This is the gravity vector in the world frame after rotation

**Python gravity output:**
- [-0.000000, -0.000000, -9.806650] m/s²
- Magnitude: 9.807 m/s² ✓
- This is just [0, 0, -g] in world frame (no rotation applied)

### 3. Initialization State Analysis

**Python shows:**
- `orient_init = True` - claims orientation is initialized
- `acc_count = 1` - has received 1 accelerometer sample
- `gyro_count = 0` - no gyroscope samples yet
- `meas_updt_ts = 72146875` - measurement update ran at this timestamp

**But quaternion is identity!** This means:
- Python sets `orient_init = True` flag
- Python runs measurement update
- **But quaternion is NOT actually initialized from accelerometer**

---

## Root Cause Hypothesis

### C Implementation Behavior (CORRECT)

1. Sample 0 arrives (accelerometer, ts=72146875)
2. C buffers the accelerometer data
3. C detects this is first sample, runs tilt initialization
4. C computes quaternion from accelerometer: q = [0.9995, -0.0329, 0.0032, 0.0001]
5. C sets orientation initialized flag
6. C outputs the initialized quaternion

**Result:** First output has proper tilt initialization

### Python Implementation Behavior (INCORRECT)

1. Sample 0 arrives (accelerometer, ts=72146875)
2. Python buffers the accelerometer data (`acc_count = 1`)
3. Python sets `orient_init = True` flag
4. Python runs measurement update (`meas_updt_ts = 72146875`)
5. **Python does NOT compute quaternion from accelerometer**
6. Python outputs identity quaternion [1, 0, 0, 0]

**Result:** First output is identity, tilt initialization did not apply

---

## Code Investigation Needed

### In Python `sensor_fusion_6axis.py`:

Need to check where `orient_init` is set and verify that `_init_orient()` is called:

```python
def _init_orient(self):
    """Initialize orientation from accelerometer (tilt initialization)"""
    # Lines 253-269 in sensor_fusion_6axis.py
    # This should compute initial quaternion from accelerometer
    # Need to verify this is called BEFORE first output
```

### Suspected Issue:

Python may be setting `orient_init = True` but NOT actually calling `_init_orient()` to compute the quaternion from the accelerometer reading.

**OR**

Python may be calling `_init_orient()` but the quaternion is being reset to identity somewhere before output.

---

## Action Items

1. **Add more debug logging to Python:**
   - Print when `_init_orient()` is called
   - Print accelerometer data passed to `_init_orient()`
   - Print quaternion immediately after `_init_orient()` returns
   - Print quaternion before returning from `run()`

2. **Add more debug logging to C:**
   - Print when tilt initialization occurs in C code
   - Print accelerometer data used for initialization
   - Print quaternion after tilt initialization

3. **Find the exact line where divergence occurs:**
   - Trace through Python `run()` method step by step
   - Identify where quaternion should be initialized but isn't

4. **Fix the Python implementation:**
   - Ensure `_init_orient()` is called before first output
   - Ensure quaternion from `_init_orient()` is preserved until output
   - Ensure Python matches C's initialization behavior

---

## Expected Fix

After fix, Python sample 0 output should match C:
```
Output Quaternion:
  q0 = 0.9994524717 (not 1.0000000000)
  q1 = -0.0329344124 (not 0.0000000000)
  q2 = 0.0031716754 (not 0.0000000000)
  q3 = 0.0001045145 (not 0.0000000000)

Output Orientation (degrees):
  Roll  = -3.774707 (not 0.000000)
  Pitch = 0.363645 (not -0.000000)
  Yaw   = 360.000000 (not 0.000000)

Gravity:
  gx = 0.062198 (not -0.000000)
  gy = 0.645155 (not -0.000000)
  gz = -9.778543 (not -9.806650)
```

---

**Date Completed:** 2025-10-12
**Next Step:** Investigate Python `_init_orient()` method and verify it's being called correctly
