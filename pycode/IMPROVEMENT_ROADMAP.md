# Improvement Roadmap - Part 2 Datasets

**Date**: 2025-10-14
**Status**: Ready for implementation
**Strategy**: Iterative improvement with regression protection

---

## Current State Summary

### ✅ Part 1: Working Well (8 datasets)
- **Static motion**: 0.577° - 5.241° mean error
- **Single-axis rotation**: 0.955° - 2.729° mean error
- **Vibration**: 0.787° mean error
- **Status**: Production ready, must not regress

### ⚠ Part 2: Needs Improvement (2 datasets)
- **rotation_sequence_15s**: 17.648° mean error (Target: <5°)
- **complex_motion_20s**: 68.571° mean error (Target: <10°)
- **Status**: Algorithm limitations, iterative improvement needed

---

## Root Cause Analysis Summary

### Dataset: rotation_sequence_15s

**Performance:**
- Mean error: 17.648° ± 19.379°
- Max error: 60.459°
- Final bias: 16.688 dps (should be <1 dps)

**Timeline of Failure:**
- Cycles 0-484: Good performance (<5° error)
- Cycle 484: First >5° error (divergence begins)
- Cycle 660: First >10° error (significant divergence)
- Cycle 953: First >20° error (severe divergence)
- Cycle 1456: Peak error 60.459°

**Root Causes Identified:**

1. **Bias Tracking Motion (Primary Issue)**
   - Bias rate of change: up to 211.9 dps/sec
   - Physical reality: <0.01 dps/sec
   - **Conclusion**: Bias is tracking rotation, not actual sensor bias
   - **Evidence**: Strong correlation between bias magnitude and error

2. **High Rotation Rates**
   - Ground truth yaw rate: mean 24.1 dps, max 60.0 dps
   - Total rotation rate: mean 28.8 dps
   - **Issue**: Fast rotations stress linearization

3. **Multi-Axis Coupling**
   - Yaw dominates error (63.5% of cycles)
   - Pitch/roll errors grow due to coupling
   - **Issue**: Cross-axis effects accumulate

4. **Covariance Evolution**
   - Bias covariance grows unchecked
   - Filter becomes overconfident
   - **Issue**: Stops trusting sensor corrections

### Dataset: complex_motion_20s

**Performance:**
- Mean error: 68.571° ± 59.780°
- Max error: 179.960° (complete orientation loss)
- Final bias: 55.575 dps (physically impossible!)

**Catastrophic Failure Indicators:**
- 38.6% of cycles have >90° error
- Bias reaches 120.6 dps at peak
- Median error: 48.3° (not just outliers!)

**Root Causes Identified:**

1. **Linear Acceleration (Primary Issue)**
   - Accelerometer measures: **gravity + linear_accel**
   - Filter assumes: **only gravity**
   - **Result**: Linear accel misinterpreted as tilt
   - **Consequence**: Completely wrong orientation

2. **Compounded Multi-Axis Rotation**
   - Same issues as rotation_sequence
   - But worse due to linear accel corruption

3. **Complete Bias Divergence**
   - 55.6 dps final bias
   - Real sensors: 0.01-1.0 dps
   - **Conclusion**: Filter completely lost

---

## Proposed Improvements (Ordered by Priority)

### Iteration 1: Bias Rate Limiting (LOW RISK)

**Objective**: Prevent bias from tracking motion

**Implementation:**
```python
# Add to sensor_fusion_9axis.py
MAX_BIAS_RATE = 0.1  # dps per cycle (0.01 dps/sec at 100Hz)

def update_bias_with_rate_limit(self, new_bias, dt):
    """Limit how fast bias can change"""
    max_change = MAX_BIAS_RATE * dt
    bias_change = new_bias - self.bias_post_s

    # Clip change to physically realistic rate
    bias_change = np.clip(bias_change, -max_change, max_change)

    return self.bias_post_s + bias_change
```

**Expected Impact:**
- rotation_sequence: 50-70% bias improvement (16.7 → 5-8 dps)
- complex_motion: 30-50% bias improvement (55.6 → 27-38 dps)
- Part 1: No regression (bias changes are slow)

**Risk Level**: ★☆☆☆☆ Very Low
**Effort**: ★☆☆☆☆ Very Low (10-20 lines)
**Potential Gain**: ★★★★☆ High (solves primary issue)

---

### Iteration 2: Adaptive Process Noise (MEDIUM RISK)

**Objective**: Allow filter to track faster changes during rapid rotation

**Implementation:**
```python
# Add rotation detection
def detect_rotation_magnitude(self, gyro_data):
    """Detect total rotation rate"""
    rotation_magnitude = np.linalg.norm(gyro_data)

    # Thresholds
    SLOW_ROTATION = 10.0  # dps
    FAST_ROTATION = 30.0  # dps

    if rotation_magnitude < SLOW_ROTATION:
        return 1.0  # Normal
    elif rotation_magnitude < FAST_ROTATION:
        return 2.0  # Increase Q by 2x
    else:
        return 5.0  # Increase Q by 5x

def update_process_noise_adaptively(self):
    """Scale process noise based on motion"""
    scale = self.detect_rotation_magnitude(self.gyro_current)

    # Scale orientation process noise, not bias
    self.Q_mat[0:3, 0:3] *= scale
```

**Expected Impact:**
- rotation_sequence: 20-30% error improvement (17.6 → 12-14°)
- complex_motion: 10-20% error improvement
- Part 1: Minor impact (rotation rates are lower)

**Risk Level**: ★★☆☆☆ Low-Medium
**Effort**: ★★☆☆☆ Low-Medium (50-100 lines)
**Potential Gain**: ★★★☆☆ Medium

---

### Iteration 3: Motion Detection & Adaptive Measurement Noise (MEDIUM RISK)

**Objective**: Detect linear acceleration and reduce accelerometer trust

**Implementation:**
```python
def detect_linear_acceleration(self, accel_data):
    """Detect if linear acceleration is present"""
    accel_magnitude = np.linalg.norm(accel_data)
    gravity = 9.81  # m/s^2

    # Check if acceleration magnitude deviates from gravity
    deviation = abs(accel_magnitude - gravity)

    THRESHOLD = 0.5  # m/s^2 (tune based on sensor noise)

    return deviation > THRESHOLD

def update_measurement_noise_adaptively(self):
    """Increase measurement noise when linear accel detected"""
    if self.detect_linear_acceleration(self.accel_current):
        # Reduce trust in accelerometer
        self.R_mat[0:3, 0:3] *= 10.0  # 10x higher measurement noise
        return True
    return False
```

**Expected Impact:**
- complex_motion: 60-80% improvement (68.6 → 13-27°)
- rotation_sequence: Minor impact (no linear accel)
- Part 1: No impact (no linear accel in static/rotation datasets)

**Risk Level**: ★★★☆☆ Medium
**Effort**: ★★★☆☆ Medium (100-200 lines including tuning)
**Potential Gain**: ★★★★★ Very High (solves complex_motion)

---

### Iteration 4: Multiplicative Extended Kalman Filter (HIGH RISK)

**Objective**: Replace angle-based error state with quaternion error

**Implementation:**
- Rewrite error state representation
- Use quaternion multiplication for error injection
- No small-angle assumption needed

**Expected Impact:**
- rotation_sequence: 70-90% improvement (17.6 → 1.8-5.3°)
- complex_motion: 40-60% improvement (still needs motion detection)
- Part 1: Should maintain but requires extensive testing

**Risk Level**: ★★★★★ Very High (core algorithm change)
**Effort**: ★★★★★ Very High (500+ lines, weeks of work)
**Potential Gain**: ★★★★★ Very High (fundamental improvement)

**Recommendation**: Only pursue if Iterations 1-3 insufficient

---

## Implementation Plan

### Phase 1: Low-Risk Improvements (Week 1)

**Steps:**
1. Implement bias rate limiting
2. Run regression tests on Part 1
3. Test on Part 2 datasets
4. Document results

**Success Criteria:**
- ✅ Part 1: All 8 datasets pass regression tests
- ✅ Part 2: rotation_sequence bias <8 dps
- ✅ Part 2: complex_motion bias <40 dps

**If Failed:**
- Review rate limit threshold (may be too aggressive)
- Check bias update equation
- Revert and analyze

---

### Phase 2: Adaptive Algorithms (Week 2)

**Steps:**
1. Implement adaptive process noise
2. Run regression tests on Part 1
3. Test on Part 2 datasets
4. If passed, implement motion detection
5. Re-run all tests
6. Document results

**Success Criteria:**
- ✅ Part 1: All 8 datasets maintain performance (within 5% of baseline)
- ✅ rotation_sequence: Mean error <12°
- ✅ complex_motion: Mean error <25°

**If Failed:**
- Check motion detection thresholds
- Verify noise scaling factors
- Consider parameter tuning
- Revert if >10% Part 1 regression

---

### Phase 3: Evaluation & Decision (Week 3)

**Evaluate Results:**
- If Part 2 meets targets (rotation_sequence <5°, complex_motion <10°): ✅ DONE
- If Part 2 improved but not sufficient: → Phase 4
- If Part 2 not improved: → Debug and re-tune Phase 1-2

**Decision Point:**
- **Option A**: Sufficient improvement → Production ready
- **Option B**: Need more improvement → Proceed to MEKF (Phase 4)
- **Option C**: Failed → Re-analyze root causes, consider other approaches

---

### Phase 4: MEKF Implementation (If Needed - Week 4+)

**Only if Phases 1-3 insufficient**

**Steps:**
1. Design MEKF implementation
2. Implement with feature flag
3. Extensive regression testing
4. Benchmark vs. current implementation
5. Decision: Keep MEKF or stay with tuned indirect EKF

---

## Testing Protocol (For Each Iteration)

### Step 1: Pre-Change Baseline
```bash
# Ensure current results are baseline
python3 regression_test.py
```
Expected: All tests pass

### Step 2: Implement Change
- Make code changes
- Add comments explaining modifications
- Update any affected constants

### Step 3: Run Regression Tests
```bash
# Test Part 1 FIRST
python3 regression_test.py --verbose
```
**CRITICAL**: If any Part 1 tests fail → STOP and revert

### Step 4: Test on Part 2
```bash
# Re-run synthetic tests
python3 test_all_synthetic_datasets.py

# Generate plots
python3 plot_all_synthetic_datasets.py

# Analyze Part 2 improvements
python3 analyze_part2_failures.py
```

### Step 5: Evaluate Results

**Metrics to Compare:**
- Part 1: Zero regressions >10%
- Part 2: Error reduction ≥25%
- Bias: Physical plausibility (<3 dps)

### Step 6: Decision
- ✅ Improvement achieved, no regressions → Commit
- ⚠ Small Part 1 regression (<5%), large Part 2 gain (>50%) → Discuss
- ❌ Part 1 regression or no Part 2 improvement → Revert

---

## Success Metrics

### Minimum Viable (Must Achieve)
- ✅ Part 1: Zero regressions >10%
- ✅ Part 2 rotation_sequence: <10° mean error (currently 17.6°)
- ✅ Part 2 complex_motion: <40° mean error (currently 68.6°)

### Target (Goal)
- ✅ Part 1: Zero regressions >5%
- ✅ Part 2 rotation_sequence: <5° mean error
- ✅ Part 2 complex_motion: <20° mean error
- ✅ All datasets: Bias <3 dps

### Stretch (Ideal)
- ✅ Part 1: All improve or maintain within 2%
- ✅ Part 2 rotation_sequence: <3° mean error
- ✅ Part 2 complex_motion: <10° mean error
- ✅ All 10 datasets: <5° mean error

---

## Risk Mitigation

### Risk: Part 1 Regression
**Mitigation:**
- Run regression tests BEFORE Part 2 testing
- Automated pass/fail (no subjective evaluation)
- Immediate revert if failed
- Version control all changes

### Risk: Part 2 No Improvement
**Mitigation:**
- Small incremental changes
- Test each change independently
- Document what doesn't work (avoid repeating)
- Have fallback plan (accept current limitations)

### Risk: Over-Tuning
**Mitigation:**
- Physical constraints on parameters
- Multiple dataset validation
- Avoid dataset-specific tuning
- Keep changes general and principled

---

## Tools & Scripts Available

### Regression Testing
- **`regression_test.py`** - Automated Part 1 regression suite
- **`regression_baseline.json`** - Current baseline metrics

### Batch Testing
- **`test_all_synthetic_datasets.py`** - Test all 10 datasets
- **`plot_all_synthetic_datasets.py`** - Generate visualizations

### Analysis
- **`analyze_part2_failures.py`** - Deep dive into failures

### Documentation
- **`DATASET_CATEGORIZATION.md`** - Part 1 vs Part 2 breakdown
- **`SYNTHETIC_DATASETS_TEST_REPORT.md`** - Full test report
- **`IMPROVEMENT_ROADMAP.md`** - This document

---

## Next Steps

1. **Review this roadmap** - Ensure agreement on approach
2. **Implement Iteration 1** - Bias rate limiting (lowest risk, highest ROI)
3. **Test and evaluate** - Follow testing protocol
4. **Iterate** - Proceed to Iteration 2 if successful
5. **Document results** - Update reports with improvements

---

## Conclusion

We have a clear path forward:

✅ **Part 1 is protected** - Automated regression testing prevents breaking working features

✅ **Part 2 root causes identified** - Bias tracking motion + linear acceleration

✅ **Solutions proposed** - Ordered by risk and effort

✅ **Testing framework ready** - Can iterate quickly and safely

**Recommended Start**: Implement **Iteration 1 (Bias Rate Limiting)** first. It's low risk, low effort, and addresses the primary failure mode in both Part 2 datasets.

---

**End of Roadmap**
