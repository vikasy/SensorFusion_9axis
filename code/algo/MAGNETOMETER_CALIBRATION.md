# Magnetometer Calibration Module

**Module:** `algo_sf_mag_cal.c` / `algo_sf_mag_cal.h`
**Method:** Least Squares Ellipsoid Fitting
**Author:** Vikas Yadav
**Date:** 2025

---

## Overview

The magnetometer calibration module corrects for magnetic distortions that affect raw magnetometer readings. These distortions include:

1. **Hard Iron Distortion** - Constant magnetic offset from device electronics (battery, motors, etc.)
2. **Soft Iron Distortion** - Magnetic field warping from ferromagnetic materials in device
3. **Sensor Scaling Errors** - Manufacturing variations in sensor sensitivity

### The Problem

In an ideal scenario, rotating a magnetometer in Earth's uniform magnetic field would trace a **sphere** centered at the origin with radius equal to the local field strength (~50 μT).

However, real-world distortions cause the magnetometer to trace an **ellipsoid** that is:
- **Offset** from origin (hard iron)
- **Deformed** into non-spherical shape (soft iron)
- **Scaled** differently on each axis (sensor errors)

### The Solution

The calibration algorithm:
1. Collects magnetometer samples while user rotates device
2. Fits samples to ellipsoid equation using least squares
3. Extracts calibration parameters (offset and correction matrix)
4. Transforms raw readings back to ideal sphere

**Result:** Calibrated magnetometer provides accurate heading (compass direction) for 9-axis sensor fusion.

---

## Mathematical Background

### Ellipsoid Equation

The distorted magnetometer readings fit an ellipsoid:

```
(m - c)ᵀ · A · (m - c) = 1
```

where:
- `m` = raw magnetometer vector [mx, my, mz]
- `c` = center (hard iron offset)
- `A` = shape matrix (related to soft iron)

### Calibration Transformation

To correct raw readings:

```
m_calibrated = A^(-1/2) · (m_raw - c)
```

This transformation:
1. Removes hard iron offset (`m_raw - c`)
2. Undoes soft iron deformation (`A^(-1/2)`)
3. Results in sphere with correct field magnitude

### Implementation

The current implementation uses a simplified approach:
- **Hard iron:** Estimated as center of bounding box (min/max on each axis)
- **Soft iron:** Diagonal scaling matrix to equalize axis radii
- **Quality metric:** Standard deviation of calibrated magnitudes

For production use, consider implementing full ellipsoid fitting with SVD (Singular Value Decomposition) for better accuracy.

---

## API Usage

### 1. Initialize Calibration Module

```c
#include "algo_sf_mag_cal.h"

mag_cal_state_t cal_state;
mag_cal_init(&cal_state);
```

**Initial state:** Identity calibration (no correction applied)

### 2. Collect Magnetometer Samples

```c
// While user rotates device, add samples
float mag_raw[3] = {mx, my, mz};  // μT
uint64_t timestamp = /* current time in ns */;

bool added = mag_cal_add_sample(&cal_state, mag_raw, timestamp);

// Returns true if sample was accepted (diverse enough)
// Returns false if sample too similar to recent samples
```

**Requirements:**
- Minimum 50 samples from different orientations
- Samples should span full 3D rotation (not just one plane)
- Algorithm automatically rejects duplicate/similar samples

### 3. Compute Calibration Parameters

```c
bool success = mag_cal_compute(&cal_state);

if (success) {
    printf("Calibration successful!\n");
} else {
    mag_cal_status_t status = mag_cal_get_status(&cal_state);
    // Handle failure: insufficient samples, poor quality, etc.
}
```

**Automatic quality checks:**
- Minimum variance requirement (samples must span different orientations)
- Quality threshold (sphere fit must be good)
- Returns `false` if calibration fails checks

### 4. Apply Calibration to Raw Data

```c
float mag_raw[3] = {mx_raw, my_raw, mz_raw};
float mag_cal[3];

mag_cal_apply(&cal_state, mag_raw, mag_cal);

// mag_cal now contains calibrated magnetometer reading
// Use this for sensor fusion heading estimation
```

**Behavior:**
- If calibrated: applies correction transformation
- If uncalibrated: passes through raw data unchanged

### 5. Save/Load Calibration (Optional)

```c
// Save calibration to non-volatile storage
mag_cal_params_t params;
if (mag_cal_get_params(&cal_state, &params)) {
    // Write params to EEPROM/flash/file
    save_to_storage(&params, sizeof(params));
}

// Load calibration from storage
mag_cal_params_t loaded_params;
load_from_storage(&loaded_params, sizeof(loaded_params));
mag_cal_set_params(&cal_state, &loaded_params);
```

**Use case:** Avoid re-calibrating every device boot

---

## Calibration Status

The module tracks calibration state through `mag_cal_status_t`:

| Status | Value | Meaning |
|--------|-------|---------|
| `MAG_CAL_STATUS_UNCALIBRATED` | 0 | No valid calibration available |
| `MAG_CAL_STATUS_COLLECTING` | 1 | Collecting samples (enough for calibration) |
| `MAG_CAL_STATUS_CALIBRATED` | 2 | Calibration complete and valid |
| `MAG_CAL_STATUS_POOR_QUALITY` | 3 | Calibration failed quality check |
| `MAG_CAL_STATUS_INSUFFICIENT` | 4 | Not enough samples yet |

Check status:
```c
mag_cal_status_t status = mag_cal_get_status(&cal_state);
bool valid = mag_cal_is_valid(&cal_state);
float quality = mag_cal_get_quality(&cal_state);  // 0.0 to 1.0
```

---

## Configuration Parameters

Adjust these in `algo_sf_mag_cal.h` for your application:

```c
// Minimum samples needed for calibration
#define MAG_CAL_MIN_SAMPLES 50

// Maximum samples stored (circular buffer)
#define MAG_CAL_MAX_SAMPLES 200

// Minimum variance in samples (μT²)
// Higher = requires more diverse orientations
#define MAG_CAL_MIN_VARIANCE 100.0

// Quality threshold (0.0 to 1.0)
// Higher = stricter sphere fit requirement
#define MAG_CAL_QUALITY_THRESHOLD 0.90
```

---

## Quality Metric

The calibration quality metric indicates how well the calibrated samples fit a sphere:

```
quality = 1.0 - (std_dev / mean_magnitude)
```

where:
- `std_dev` = standard deviation of calibrated magnitudes
- `mean_magnitude` = average magnitude after calibration

**Interpretation:**
- `1.0` = Perfect sphere (ideal)
- `> 0.95` = Excellent calibration
- `0.90 - 0.95` = Good calibration (default threshold)
- `< 0.90` = Poor calibration (rejected)

---

## Example: Complete Calibration Workflow

```c
#include "algo_sf_mag_cal.h"
#include <stdio.h>

int main(void) {
    mag_cal_state_t cal_state;
    mag_cal_init(&cal_state);

    // Step 1: Collect samples while user rotates device
    printf("Please rotate device slowly in figure-8 pattern...\n");

    for (int i = 0; i < 200; i++) {
        // Read raw magnetometer
        float mag_raw[3];
        read_magnetometer(mag_raw);  // Your sensor read function

        // Add sample
        if (mag_cal_add_sample(&cal_state, mag_raw, timestamp)) {
            printf("Sample %d added\n", i);
        }

        delay_ms(100);  // Sample at ~10 Hz
    }

    // Step 2: Compute calibration
    if (mag_cal_compute(&cal_state)) {
        mag_cal_params_t params;
        mag_cal_get_params(&cal_state, &params);

        printf("Calibration successful!\n");
        printf("Hard iron: [%.2f, %.2f, %.2f] μT\n",
               params.offset[0], params.offset[1], params.offset[2]);
        printf("Quality: %.3f\n", params.quality);

        // Save to non-volatile storage
        save_calibration(&params);
    } else {
        printf("Calibration failed - try again\n");
        return 1;
    }

    // Step 3: Use calibration in sensor fusion loop
    while (1) {
        float mag_raw[3];
        read_magnetometer(mag_raw);

        // Apply calibration
        float mag_cal[3];
        mag_cal_apply(&cal_state, mag_raw, mag_cal);

        // Use mag_cal for 9-axis fusion
        sensor_fusion_update(accel, gyro, mag_cal);

        delay_ms(10);
    }

    return 0;
}
```

---

## User Instructions for Calibration

When implementing calibration UI, instruct users to:

1. **Start calibration** - Hold device away from metal objects
2. **Figure-8 pattern** - Slowly rotate device in figure-8 motion
3. **All orientations** - Ensure device points in all directions (up, down, sides)
4. **Smooth motion** - Avoid jerky movements
5. **Duration** - Continue for 20-30 seconds

**Good practice:**
- Outdoor calibration (away from buildings)
- Same location where device will be used
- Re-calibrate if moving to different geographic location

**Bad practice:**
- Near cars, metal furniture, electronics
- Only rotating in one plane (e.g., just horizontal)
- Too fast rotation (motion blur)

---

## Integration with 9-Axis Sensor Fusion

The magnetometer calibration integrates seamlessly with the sensor fusion algorithm:

```c
// In your sensor fusion loop:

// 1. Read raw sensors
float accel_raw[3], gyro_raw[3], mag_raw[3];
read_sensors(accel_raw, gyro_raw, mag_raw);

// 2. Apply mag calibration
float mag_cal[3];
mag_cal_apply(&cal_state, mag_raw, mag_cal);

// 3. Run 9-axis fusion with calibrated magnetometer
sensor_data_t sensor;
sensor.sensordata[0] = mag_cal[0];  // Use calibrated mag
sensor.sensordata[1] = mag_cal[1];
sensor.sensordata[2] = mag_cal[2];
sensor.sensorID = MAG;
sensor.timestamp = get_timestamp();

sf_9xag_data_preproc(sf_algo_id, &sensor);
// ... continue with fusion algorithm
```

**Result:** Accurate heading estimation for 9-axis orientation tracking.

---

## Algorithm Limitations

### Current Implementation (Simplified)

The current implementation uses bounding box approximation:
- **Pros:** Fast, low memory, no matrix decomposition needed
- **Cons:** Less accurate for strong soft iron distortion

### Full Ellipsoid Fitting (Future Enhancement)

For production applications, consider implementing:
- Least squares fitting of full ellipsoid equation (9 parameters)
- Singular Value Decomposition (SVD) for matrix square root
- Iterative refinement for better convergence

**Reference:** "Ellipsoid Fit" by Yury Petrov (MATLAB Central)

### Known Limitations

1. **Requires diverse samples** - Must rotate through all orientations
2. **Static calibration** - Assumes distortions are constant (re-calibrate if hardware changes)
3. **No online adaptation** - Calibration fixed after computation (not adaptive)
4. **Simplified soft iron** - Diagonal matrix only (ignores off-diagonal terms)

---

## Troubleshooting

### Calibration Fails (Poor Quality)

**Causes:**
- Samples not diverse enough (only rotated in one plane)
- Too few samples
- External magnetic interference during collection

**Solutions:**
- Instruct user to rotate device in figure-8 pattern
- Increase collection time (get more samples)
- Move away from metal objects

### Calibration Doesn't Improve Heading

**Causes:**
- Calibration done near metal/magnets (captured interference as "field")
- Soft iron distortion too strong for diagonal approximation
- Magnetometer sensor damaged/saturated

**Solutions:**
- Re-calibrate outdoors away from metal
- Implement full ellipsoid fitting with SVD
- Check magnetometer readings for sanity (< 200 μT typical)

### Quality Metric Always Low

**Causes:**
- `MAG_CAL_QUALITY_THRESHOLD` too strict for application
- Sensor noise high
- Strong local magnetic anomalies

**Solutions:**
- Lower threshold to 0.85 (acceptable for many applications)
- Average multiple samples before adding to calibration
- Warn user about magnetic environment

---

## Files

- **Header:** `code/algo/inc/algo_sf_mag_cal.h` - API definitions
- **Implementation:** `code/algo/src/algo_sf_mag_cal.c` - Calibration algorithm
- **Example:** `test/tests/validation_tests/test_mag_cal_example.c` - Usage example
- **Documentation:** This file

---

## References

1. **Ellipsoid Fitting:**
   - Li, Q., & Griffiths, J. G. (2004). "Least squares ellipsoid specific fitting"
   - Petrov, Y. (2009). "Ellipsoid Fit" (MATLAB Central)

2. **Magnetometer Calibration:**
   - Freescale AN4246: "Calibrating an eCompass in the Presence of Hard- and Soft-Iron Interference"
   - Renaudin, V., et al. (2010). "Complete triaxis magnetometer calibration in the magnetic domain"

3. **Applications:**
   - Used in smartphones, drones, robotics for accurate heading
   - Critical for indoor navigation and AR applications
   - Improves 9-axis sensor fusion accuracy

---

## Future Enhancements

1. **Full SVD-based ellipsoid fitting** - Better soft iron correction
2. **Online adaptive calibration** - Update parameters during operation
3. **Outlier rejection** - Robust to magnetic interference spikes
4. **Multi-position calibration** - Specify required orientations for user
5. **Magnetic field map** - Detect and avoid local anomalies
6. **Auto-trigger** - Detect when re-calibration needed (field magnitude drift)

---

**For questions or contributions, contact:** Vikas Yadav
