# Sensor Fusion Parameter Tuning Recommendations

**Date:** 2025-10-17
**Based on:** Realistic Dataset Investigation

---

## Default Parameters (Optimized)

The following parameters are optimal for most scenarios and should be used as defaults:

### Process Noise (Q Matrix)
- `QOrient = 10.0` - Orientation process noise
- `QBias = 10.0` - Gyro bias process noise
- `QLinAcc = 1.0` - Linear acceleration process noise (DEFAULT)
- `QBiasOrient = 0.1` - Bias-orientation cross-correlation

### Measurement Noise (R Matrix)
- `QVACC = 2e-6` - Accelerometer quantization noise
- `QWACC = 10.0` - Accelerometer white noise (dominates R ≈ 10.0)
- `QVGYRO = 0.01` - Gyro quantization noise
- `QWGYRO = 1e-9` - Gyro white noise

### Initial Error Covariance (P Matrix)
- `P[0][0] = 0.1 rad²` - Orientation error (≈18° uncertainty)
- `P[1][1] = 0.76 rad²` - Gyro bias error (≈50 dps uncertainty)
- `P[2][2] = 0.1 (m/s²)²` - Linear acceleration error

---

## High-Dynamic Scenario Parameters

For scenarios with **aggressive 3D maneuvers**, high rotation rates, and rapid transitions:

### When to Use:
- Flying drones/quadcopters with banking turns
- Aggressive vehicle maneuvers (racing, off-road)
- Rapid hand-held device motion
- Any scenario with mean rotation rates >4 dps

### Modified Parameters:
**`QLinAcc = 5.0`** (increased from 1.0)

### Expected Performance Improvement:
Based on flying_drone dataset testing:
- Mean error: 13.7° → 10.7° (22% improvement)
- Poor samples (>10° error): 50.3% → 19.1% (61% reduction)
- Tradeoff: Max error may increase slightly (63° → 76°)

### Implementation:
```python
# In sensor_fusion_9axis.py or sensor_fusion_6axis.py initialization

# For high-dynamic scenarios:
self.proc_noise_var_lin_acc = 5.0  # Instead of default 1.0
```

---

## Parameter Tuning Investigation Results

### QLinAcc (Linear Acceleration Process Noise)
Tested range: 0.01 to 100.0 (4 orders of magnitude)

**Walking dataset** (initialization problem):
- 100× variation resulted in <1° difference
- Conclusion: Cannot fix initialization issues with parameter tuning

**Flying drone dataset** (high dynamics):
- QLinAcc=5.0 provided meaningful 22% improvement
- Conclusion: Helps with scenarios that have sustained high linear accelerations

### QVACC (Accelerometer Quantization Noise)
Tested range: 2e-6 to 100.0 (50 million× variation)

**All datasets**:
- Changes <0.05° across all datasets
- Reason: QWACC=10.0 dominates R matrix
- Conclusion: Keep QVACC=2e-6 (optimal)

### P[0][0] (Initial Orientation Uncertainty)
Tested range: 0.1 to 100.0 rad² (1000× variation)

**Walking dataset**:
- No difference in error (58.590° identical across all values)
- Conclusion: Cannot fix wrong reference frame by increasing uncertainty

---

## Key Lessons

1. **Initialization is Critical:**
   - Proper dataset design (static start) matters more than parameter tuning
   - Kalman filter cannot recover from wrong initial reference frame
   - Walking dataset: 58.6° → 3.5° by adding 3s static period (not by tuning)

2. **Parameter Limits:**
   - Most parameters affect error magnitude within a reference frame
   - They cannot change which reference frame is correct
   - Extreme parameter variations (100-1000×) often have minimal impact

3. **Scenario-Specific Tuning:**
   - One size does NOT fit all
   - Walking: Dataset fix worked, parameter tuning didn't
   - Drone: Parameter tuning (QLinAcc=5) worked better than dataset fix
   - Match the approach to the scenario characteristics

4. **When to Tune:**
   - ✅ Runtime performance issues with known-good initialization
   - ✅ High-dynamic scenarios with sustained linear accelerations
   - ❌ Initialization failures (fix dataset/initialization logic instead)
   - ❌ Reference frame mismatch issues

---

## Recommendation Decision Tree

```
Is initialization failing (large error from start)?
├─ YES: Fix initialization, not parameters
│  ├─ Add static startup period (3+ seconds)
│  ├─ Improve _init_orient() robustness
│  └─ Verify coordinate frame consistency
│
└─ NO: Are you in a high-dynamic scenario?
   ├─ YES: Try QLinAcc = 5.0
   │  └─ Test: Mean rotation >4 dps, rapid maneuvers
   │
   └─ NO: Use default parameters (already optimal)
```

---

## Testing Validation

All recommendations validated on realistic motion datasets:

| Dataset | Scenario | QLinAcc | Mean Error | Status |
|---------|----------|---------|------------|--------|
| Walking | Periodic gait | 1.0 (default) | 3.5° | ✅ Excellent |
| Handheld | Phone/tablet | 1.0 (default) | 1.6° | ✅ Excellent |
| Climbing Stairs | Vertical motion | 1.0 (default) | 3.2° | ✅ Excellent |
| Driving | Car motion | 1.0 (default) | 5.5° | ✅ Good |
| **Flying Drone** | **Aggressive 3D** | **5.0 (tuned)** | **10.7°** | ✅ **Acceptable** |

---

## Future Considerations

1. **Adaptive Parameter Selection:**
   - Detect high-dynamic motion at runtime
   - Automatically increase QLinAcc when needed
   - Return to default when motion settles

2. **Parameter Profiles:**
   - Create named presets: "default", "high_dynamic", "low_noise"
   - Allow easy switching based on use case
   - Document expected performance characteristics

3. **Real-World Validation:**
   - Test recommendations on actual hardware
   - Collect field data from various scenarios
   - Refine based on real-world performance

---

**Reference:** See `REALISTIC_DATASET_INVESTIGATION_SUMMARY.md` for complete investigation details.
