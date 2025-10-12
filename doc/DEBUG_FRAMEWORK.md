# C/Python Synchronized Debugging Framework

## Purpose

This framework enables synchronized debugging between C and Python implementations of sensor fusion algorithms. When a C test fails, you can run the exact same test data through Python (which is known-good) and compare internal states sample-by-sample to identify the root cause.

## Workflow

```
┌─────────────────┐
│  Run C Test     │
│  (with -DDEBUG) │
└────────┬────────┘
         │
         ▼
  ┌──────────────┐
  │  C Test      │
  │  Fails at    │──────┐
  │  Sample N    │      │
  └──────────────┘      │
                        │
         ┌──────────────┘
         │
         ▼
┌────────────────────────┐
│  Export Test Data      │
│  + C Debug Logs        │
└─────────┬──────────────┘
          │
          ▼
┌─────────────────────────┐
│  Run Python with        │
│  Same Test Data         │
│  (verbose debug mode)   │
└──────────┬──────────────┘
           │
           ▼
┌──────────────────────────┐
│  Compare Logs            │
│  Sample-by-Sample        │
└───────────┬──────────────┘
            │
            ▼
┌────────────────────────────┐
│  Find First Divergence     │
│  Examine Internal States   │
│  Root Cause the Bug        │
└────────────────────────────┘
```

## Step-by-Step Instructions

### Step 1: Add Debug Logging to C Test

Edit the C test file (e.g., `test/tests/integration_tests/test_6axis_fusion.c`):

```c
// Add at top of file
#define DEBUG_LOGGING 1

// Add this helper function
void print_sensor_data(int sample,
                       const int16_t acc[3],
                       const int16_t gyro[3],
                       const int16_t mag[3],
                       uint64_t timestamp) {
#ifdef DEBUG_LOGGING
    printf("SENSOR_DATA,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%llu\n",
           sample, acc[0], acc[1], acc[2],
           gyro[0], gyro[1], gyro[2],
           mag[0], mag[1], mag[2], timestamp);
#endif
}

// Add this helper to print internal state
void print_fusion_state(int sample, const char* label, algo_sf_6x_state_vec_t* state) {
#ifdef DEBUG_LOGGING
    printf("STATE,%d,%s,", sample, label);
    printf("quat,[%.6f,%.6f,%.6f,%.6f],",
           state->quat_post.q0, state->quat_post.q1,
           state->quat_post.q2, state->quat_post.q3);
    printf("orient,[%.3f,%.3f,%.3f],",
           state->algo_output.orientation_S[0],
           state->algo_output.orientation_S[1],
           state->algo_output.orientation_S[2]);
    printf("bias,[%.6f,%.6f,%.6f],",
           state->bias_post_s[0], state->bias_post_s[1], state->bias_post_s[2]);
    printf("gravity,[%.3f,%.3f,%.3f]\n",
           state->algo_output.gravity_S[0],
           state->algo_output.gravity_S[1],
           state->algo_output.gravity_S[2]);
#endif
}

// In your test function, add logging before/after each update:
for (int i = 0; i < num_samples; i++) {

    // Log sensor data
    print_sensor_data(i, acc_data[i], gyro_data[i], mag_data[i], timestamps[i]);

    // Log state BEFORE update
    print_fusion_state(i, "BEFORE", state);

    // Feed data and run fusion
    algo_sf_6x_preprocess_sensor_data(state, SENSOR_ID_ACC, acc_data[i], timestamps[i]);
    algo_sf_6x_preprocess_sensor_data(state, SENSOR_ID_GYRO, gyro_data[i], timestamps[i]);
    int ret = algo_sf_6x_run(state);

    // Log state AFTER update
    print_fusion_state(i, "AFTER", state);

    // Check for errors
    if (ret != 0) {
        printf("ERROR: algo_sf_6x_run returned %d at sample %d\n", ret, i);
        break;  // STOP at first error
    }
}
```

### Step 2: Run C Test and Capture Output

```bash
# Rebuild with debug flags
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make

# Run test and save output
./bin/test_6axis_fusion > c_test_output.txt 2>&1

# Check for errors
grep "ERROR:" c_test_output.txt
```

### Step 3: Export Test Data to Python Format

```bash
cd ..
python3 test/scripts/export_test_data.py \
    build/c_test_output.txt \
    pycode/c_test_data.json
```

This extracts all `SENSOR_DATA,*` lines and creates a JSON file with:
```json
{
  "acc": [[x1, y1, z1], [x2, y2, z2], ...],
  "gyro": [[x1, y1, z1], [x2, y2, z2], ...],
  "mag": [[x1, y1, z1], [x2, y2, z2], ...],
  "acc_ts": [ts1, ts2, ...],
  "gyro_ts": [ts1, ts2, ...],
  "mag_ts": [ts1, ts2, ...]
}
```

### Step 4: Run Python with Same Test Data

```bash
cd pycode
python3 debug_compare.py --input c_test_data.json --fusion-type 6axis --verbose
```

Or use Python interactively:

```python
import json
import numpy as np
from debug_compare import run_python_with_debug

# Load test data
with open('c_test_data.json') as f:
    test_data_json = json.load(f)

# Convert to numpy
test_data = {
    'acc': np.array(test_data_json['acc']),
    'gyro': np.array(test_data_json['gyro']),
    'mag': np.array(test_data_json['mag']),
    'acc_ts': np.array(test_data_json['acc_ts']),
    'gyro_ts': np.array(test_data_json['gyro_ts']),
    'mag_ts': np.array(test_data_json['mag_ts'])
}

# Run with debug logging
sf, logger = run_python_with_debug(
    test_data,
    fusion_type='6axis',
    stop_at_sample=None,  # or set to sample number where C failed
    verbose=True
)
```

### Step 5: Parse C Debug Logs

Create a parser for C STATE logs:

```python
def parse_c_debug_log(filename):
    """Parse C debug log into same format as Python logger"""
    import re

    log_entries = []

    with open(filename) as f:
        for line in f:
            if line.startswith('STATE,'):
                parts = line.strip().split(',')
                # STATE,sample,label,quat,[q0,q1,q2,q3],orient,[y,p,r],...

                sample = int(parts[1])
                label = parts[2]

                # Extract quaternion
                quat_str = parts[4]  # [q0,q1,q2,q3]
                quat = [float(x) for x in quat_str.strip('[]').split(',')]

                # Extract orientation
                orient_str = parts[6]  # [yaw,pitch,roll]
                orient = [float(x) for x in orient_str.strip('[]').split(',')]

                # Extract bias
                bias_str = parts[8]  # [bx,by,bz]
                bias = [float(x) for x in bias_str.strip('[]').split(',')]

                # Extract gravity
                grav_str = parts[10]  # [gx,gy,gz]
                grav = [float(x) for x in grav_str.strip('[]').split(',')]

                entry = {
                    'sample': sample,
                    'label': label,
                    'quaternion': {'q0': quat[0], 'q1': quat[1], 'q2': quat[2], 'q3': quat[3]},
                    'orientation': {'yaw': orient[0], 'pitch': orient[1], 'roll': orient[2]},
                    'gyro_bias': {'x': bias[0], 'y': bias[1], 'z': bias[2]},
                    'gravity': {'x': grav[0], 'y': grav[1], 'z': grav[2]}
                }

                log_entries.append(entry)

    return log_entries

# Save to JSON for comparison
c_log = parse_c_debug_log('build/c_test_output.txt')
with open('debug_c_6axis.json', 'w') as f:
    json.dump(c_log, f, indent=2)
```

### Step 6: Compare Logs

```python
from debug_compare import compare_c_python_logs

# Compare logs sample by sample
divergence_found = compare_c_python_logs(
    'debug_c_6axis.json',
    'debug_python_6axis.json',
    tolerance=1e-3
)

if divergence_found:
    print("\nDivergence detected! Examine logs to find root cause.")
```

### Step 7: Analyze Divergence

When divergence is found at sample N, examine:

1. **Input Data at Sample N**
   - Are the sensor values identical?
   - Are the timestamps identical?

2. **State Before Update**
   - Is the quaternion identical?
   - Is the gyro bias identical?
   - Is the covariance matrix identical?

3. **During Update**
   - Add more detailed logging inside time_update() and measurement_update()
   - Print omega, dt, quaternion delta
   - Print Kalman gain, innovation

4. **State After Update**
   - Which value diverged first?
   - By how much?

## Common Issues to Check

### Timestamp Issues
- C using milliseconds, Python using nanoseconds?
- Integer overflow in timestamp calculations?
- Incorrect time delta computation?

### Numerical Precision
- Float vs double precision differences?
- Matrix operations accumulating errors?
- Quaternion normalization needed?

### Algorithm Logic
- Conditional branches taken differently?
- Covariance update formula correct?
- Kalman gain computation correct?

### Memory/Initialization
- Uninitialized variables?
- Memory corruption?
- Incorrect array indexing?

## Example Debug Session

```bash
# 1. Run C test
./build/bin/test_6axis_fusion > c_output.txt 2>&1

# Look for failure
grep "ERROR:" c_output.txt
# Output: ERROR: algo_sf_6x_run returned -1 at sample 42

# 2. Export test data
python3 test/scripts/export_test_data.py c_output.txt pycode/c_test_data.json

# 3. Run Python up to failure point
cd pycode
python3 << EOF
import json, numpy as np
from debug_compare import run_python_with_debug

with open('c_test_data.json') as f:
    data_json = json.load(f)

test_data = {k: np.array(v) for k, v in data_json.items()}

sf, logger = run_python_with_debug(
    test_data,
    fusion_type='6axis',
    stop_at_sample=42,  # Stop where C failed
    verbose=True
)
EOF

# 4. Compare sample 42
python3 << EOF
import json

with open('debug_python_6axis.json') as f:
    py_log = json.load(f)

# Get sample 42 BEFORE update
sample_42 = [x for x in py_log if x['sample'] == 42 and x['label'] == 'BEFORE_UPDATE'][0]

print("Python state at sample 42 BEFORE update:")
print(f"  Quat: {sample_42['quaternion']}")
print(f"  Orient: {sample_42['orientation']}")

# Compare with C log...
EOF
```

## Tools Summary

| Tool | Purpose |
|------|---------|
| `debug_compare.py` | Main Python debugging framework |
| `export_test_data.py` | Export C test data to Python format |
| `parse_c_debug_log()` | Parse C STATE logs to JSON |
| `compare_c_python_logs()` | Find first divergence point |
| Modified C test | Add verbose logging to C tests |

## Tips

1. **Start Simple**: Use simple test data (level device, no rotation) first
2. **Stop at First Error**: Don't continue past failure point
3. **Log Everything**: More detail = easier debugging
4. **Compare Incrementally**: Check state at every step
5. **Check Assumptions**: Verify units, coordinate frames, etc.

## Next Steps

Once you find and fix the C bug:
1. Re-run the C test
2. Verify it passes
3. Move to next failing test
4. Repeat process

This systematic approach ensures all C code bugs are found and fixed.
