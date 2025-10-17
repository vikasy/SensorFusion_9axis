# Sensor Fusion Testing & Optimization Roadmap

**Date:** 2025-10-15
**Status:** Phase 2 - Real-World Data Testing
**Current Parameters:** Optimized for synthetic data (R=10.0)

## Phase 1: Synthetic Data Testing ✅ COMPLETE

**Status:** Complete - Parameters optimized
**Results:** Mean error 4.786°, Final bias 0.785 dps

### Regression Test Suite Established:
1. **Core Regression (2 datasets):**
   - `rotation_sequence_15s` - Multi-axis rotation validation
   - `complex_motion_20s` - Fast motion + bias limiting validation

2. **Extended Regression (10 datasets):**
   - All synthetic scenarios (static, rotations, complex, vibration)

**Purpose:** Prevent parameter degradation during future optimization

---

## Phase 2: Real-World Data Testing ⚠️ BLOCKED - DATA ISSUE FOUND

**STATUS:** Testing revealed critical coordinate frame inconsistency in realistic datasets
**DATE:** 2025-10-15

### ⚠️ CRITICAL FINDING: Realistic Dataset Coordinate Frame Mismatch

Initial testing on `walking.csv` revealed **mean error of 58.6°** (vs target < 5°), with 100% of samples showing error > 10°.

**Root Cause Analysis:**

1. **Coordinate Frame Inconsistency:**
   - Synthetic datasets: NED convention (Z+ points up when stationary, +1g)
   - Realistic datasets (sample 0): ENU-like (Z- points up when stationary, -1g)
   - Realistic datasets (sample 50): NED-like (Z+ points up, +1g)

2. **Ground Truth Quaternion Issue:**
   - Both samples 0 and 50 have GT quat ≈ [1,0,0,0] (identity)
   - But accelerometer measurements show opposite Z-directions
   - This indicates GT quaternions are in a DIFFERENT reference frame than sensor data
   - OR GT quaternions are incorrect/not synchronized with sensor readings

3. **Evidence:**
   ```
   Sample 0 (Realistic walking.csv):
     Accel (norm): [-0.002, 0.009, -0.999]  ← Z pointing DOWN
     GT Quat:      [1.0, 0.0, 0.0, 0.0]      ← Identity (no rotation)
     GT RPY:       [0°, 0°, 0°]

   Sample 50 (Realistic walking.csv):
     Accel (norm): [-0.003, 0.008, +0.999]  ← Z pointing UP
     GT Quat:      [1.0, 0.0001, -0.00004, 0]  ← Near-identity
     GT RPY:       [0.01°, -0.00°, -0.00°]
   ```

4. **Impact:**
   - Cannot use realistic datasets for testing without fixing coordinate frames
   - 146° initial error at sample 0 suggests fusion initializes with wrong orientation
   - Bias diverges to 2.97 dps (should be < 2 dps)

**Next Steps Required:**
1. Contact dataset provider to clarify coordinate frame conventions
2. Determine if GT quaternions represent:
   - Sensor-to-world transform?
   - World-to-sensor transform?
   - Different reference frame entirely?
3. Check if GT quaternions need to be inverted or conjugated
4. Verify if accel/gyro/mag are in consistent body frame
5. Consider generating new ground truth from sensor data using known algorithm

**Temporary Resolution:**
- Continue using synthetic datasets for parameter optimization
- Synthetic datasets are internally consistent and working well (mean error 4.786°)
- Revisit realistic datasets once coordinate frame issue is resolved

---

## Phase 2: Real-World Data Testing 🔄 ORIGINAL PLAN (ON HOLD)

### Recommended Dataset Progression

The realistic datasets are ordered by complexity and real-world applicability:

#### **RECOMMENDED ORDER:**

### 1. **START WITH: `walking.csv`** ⭐ BEST FIRST CHOICE
   - **Size:** 374 KB (~3,700 samples, ~37 seconds at 100Hz)
   - **Characteristics:**
     - Periodic motion (walking gait ~2 Hz)
     - Moderate accelerations (footsteps)
     - Low-moderate rotation rates (natural body sway)
     - Predictable motion pattern
   - **Why First:**
     - Most common real-world scenario
     - Well-understood motion profile
     - Good balance of static and dynamic behavior
     - Validates filter convergence during rhythmic motion
   - **Key Validation Points:**
     - Bias convergence during periodic motion
     - Acceleration vs. gravity separation
     - Orientation stability during footsteps
     - No spurious bias tracking from walking motion

---

### 2. **NEXT: `handheld_device.csv`**
   - **Size:** 379 KB (~3,800 samples, ~38 seconds)
   - **Characteristics:**
     - User holding/manipulating device
     - Mix of static holds and quick movements
     - Random orientation changes
     - Sudden accelerations (picking up, putting down)
   - **Why Second:**
     - Tests real user interaction patterns
     - Validates bias limiting during quick gestures
     - Important for phone/tablet applications
   - **Key Validation Points:**
     - Bias stability during quick movements
     - Recovery from sudden orientation changes
     - Filter performance with unpredictable motion

---

### 3. **THEN: `climbing_stairs.csv`**
   - **Size:** 354 KB (~3,500 samples, ~35 seconds)
   - **Characteristics:**
     - Vertical motion component (climbing)
     - Periodic footsteps + vertical acceleration
     - Slight forward tilt during climbing
     - Impact forces from steps
   - **Why Third:**
     - Tests vertical acceleration handling
     - More complex than walking (non-horizontal motion)
     - Common real-world activity
   - **Key Validation Points:**
     - Gravity/acceleration separation during vertical motion
     - Tilt angle accuracy while climbing
     - Bias convergence with vertical accelerations

---

### 4. **ADVANCED: `flying_drone.csv`**
   - **Size:** 420 KB (~4,200 samples, ~42 seconds)
   - **Characteristics:**
     - Continuous motion (hovering + flight)
     - Faster rotation rates (yaw corrections)
     - Propeller vibrations
     - Three-axis motion simultaneously
   - **Why Fourth:**
     - Tests high-rate motion
     - Validates vibration rejection
     - Important for aerial robotics
   - **Key Validation Points:**
     - High rotation rate handling (>45 dps)
     - Vibration filtering effectiveness
     - 3D motion in all axes
     - Magnetometer heading accuracy

---

### 5. **MOST COMPLEX: `driving_in_car.csv`**
   - **Size:** 999 KB (~10,000 samples, ~100 seconds)
   - **Characteristics:**
     - Longest duration test
     - Vehicle dynamics (acceleration, braking, turning)
     - Road vibrations
     - Magnetic disturbances (car metal)
     - Long-term bias drift observation
   - **Why Last:**
     - Most complex real-world scenario
     - Tests long-term stability
     - Magnetic interference challenges
     - Validates extended operation
   - **Key Validation Points:**
     - Long-term bias stability (100 seconds)
     - Magnetic disturbance rejection
     - Vehicle dynamics handling
     - Sustained accuracy over time

---

## Testing Strategy for Each Dataset

### Step 1: Baseline Performance
```bash
cd pycode/test
python test_on_realistic_data.py --dataset walking
```

**Analyze:**
- Mean quaternion error
- Max quaternion error
- Bias convergence rate
- Bias final value
- Error distribution (histogram)

### Step 2: Identify Issues
- Where does error spike? (plot error vs time)
- When does bias diverge? (plot bias vs time)
- Motion correlation? (compare error to acceleration/rotation magnitude)

### Step 3: Parameter Tuning (if needed)
**Potential adjustments based on real-world data:**

1. **If bias tracks motion (false bias):**
   - Increase motion thresholds (slow/fast)
   - Strengthen bias rate limiting

2. **If error is too high during motion:**
   - Adjust R matrix (measurement noise)
   - Tune QOrient (orientation process noise)

3. **If convergence is too slow:**
   - Increase QBias
   - Reduce bias rate limiting

4. **If magnetometer interferes:**
   - Tune magnetic disturbance detection threshold
   - Adjust SF_9XAGM_QMagDist

### Step 4: Validation
- Re-run regression tests (synthetic data)
- Ensure no degradation on synthetic baseline
- Document parameter changes and rationale

---

## Expected Challenges by Dataset

### `walking.csv`
- ⚠️ Periodic footstep acceleration might be tracked as bias
- ⚠️ Orientation may drift slightly during gait cycle
- ✓ Should work well with current parameters

### `handheld_device.csv`
- ⚠️ Quick gestures may cause temporary error spikes
- ⚠️ Bias might not converge if constantly moving
- ⚠️ May need to tune bias rate limiting

### `climbing_stairs.csv`
- ⚠️ Vertical acceleration separation from gravity critical
- ⚠️ Tilt angle accuracy important
- ⚠️ May expose QLinAcc tuning needs

### `flying_drone.csv`
- ⚠️ High rotation rates may trigger strong bias limiting
- ⚠️ Vibration may require filtering
- ⚠️ Fast motion adaptive parameters will be tested heavily

### `driving_in_car.csv`
- ⚠️ Magnetic disturbances from vehicle
- ⚠️ Long-term bias drift observation
- ⚠️ Complex vehicle dynamics
- ⚠️ May need magnetometer disturbance tuning

---

## Optimization Metrics Priority

### For Real-World Data:

1. **Mean Quaternion Error** (primary)
   - Target: < 5° for walking/handheld
   - Target: < 10° for stairs/drone/driving

2. **Max Quaternion Error** (robustness)
   - Target: < 15° for walking/handheld
   - Target: < 20° for complex scenarios

3. **Bias Convergence** (secondary)
   - Target: < 2 dps final bias
   - Convergence within 20 seconds preferred

4. **Stability** (critical)
   - No divergence
   - No unbounded error growth
   - Consistent performance across dataset

---

## Next Steps

### Immediate (Phase 2a):
1. ✅ Create regression test suite (DONE)
2. ⏳ **Test on `walking.csv`** ← START HERE
3. Analyze baseline performance
4. Identify any issues

### Short-term (Phase 2b):
5. Test remaining realistic datasets (order above)
6. Document performance on each
7. Identify common issues across datasets

### Medium-term (Phase 2c):
8. Parameter tuning if needed
9. Re-validate with synthetic regression tests
10. Document final optimized parameters

### Long-term (Phase 3):
11. Real IMU hardware testing
12. Field validation
13. Production deployment

---

## Success Criteria

### Phase 2 Complete When:
- ✓ All 5 realistic datasets tested
- ✓ Mean error < 10° on all datasets
- ✓ No divergence or instability
- ✓ Synthetic regression tests still pass
- ✓ Parameters documented and justified

---

## Recommendation Summary

**START WITH: `walking.csv`**

**Rationale:**
1. Most common real-world use case
2. Moderate complexity (not too easy, not too hard)
3. Good size for quick iteration (~37 seconds)
4. Well-understood motion profile
5. Will reveal most common real-world issues
6. If this works well, confidence in other scenarios increases

**Testing Command:**
```bash
cd pycode/test
python test_on_realistic_data.py --dataset walking --plot
```

This will test the walking dataset and generate:
- Performance metrics
- Error plots over time
- Bias convergence plots
- Comparison to ground truth

---

**Author:** AI Assistant
**Last Updated:** 2025-10-15
**Status:** Ready for Phase 2 Testing
