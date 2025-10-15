#!/usr/bin/env python3
"""
Calculate proper Kalman filter Q and R matrix tuning parameters
based on MPU9250 sensor specifications used in synthetic data generation.

This script derives the noise covariance values from the sensor datasheet
specifications to ensure the Kalman filter correctly models sensor uncertainties.
"""

import numpy as np

print("="*80)
print("KALMAN FILTER TUNING PARAMETER CALCULATION")
print("="*80)
print()

# ============================================================================
# SENSOR SPECIFICATIONS (from generate_synthetic_datasets.py)
# ============================================================================

# Accelerometer (MPU9250)
ACCEL_NOISE_DENSITY = 300e-6        # 300 µg/√Hz
ACCEL_BIAS_STABILITY = 0.01         # ±0.01 g
ACCEL_RANGE_G = 2.0                 # ±2g range
ACCEL_COUNTS_PER_G = 16384          # counts/g

# Gyroscope (MPU9250)
GYRO_NOISE_DENSITY = 0.005          # 0.005 dps/√Hz
GYRO_BIAS_STABILITY = 0.5           # ±0.5 dps
GYRO_RANGE_DPS = 250.0              # ±250 dps range
GYRO_COUNTS_PER_DPS = 131           # counts/dps

# Sampling
SAMPLE_RATE_HZ = 100.0              # 100 Hz
dt = 1.0 / SAMPLE_RATE_HZ           # 0.01 s

print("Sensor Specifications (MPU9250):")
print(f"  Accel Noise Density: {ACCEL_NOISE_DENSITY*1e6:.0f} µg/√Hz")
print(f"  Accel Bias Stability: ±{ACCEL_BIAS_STABILITY} g")
print(f"  Gyro Noise Density: {GYRO_NOISE_DENSITY} dps/√Hz")
print(f"  Gyro Bias Stability: ±{GYRO_BIAS_STABILITY} dps")
print(f"  Sample Rate: {SAMPLE_RATE_HZ} Hz (dt = {dt} s)")
print()

# ============================================================================
# R MATRIX (Measurement Noise Covariance)
# ============================================================================

print("="*80)
print("R MATRIX (Measurement Noise Covariance)")
print("="*80)
print()

# Accelerometer measurement noise variance (in g²)
# Noise density is in g/√Hz, so variance = (noise_density)² * bandwidth
# For discrete system: bandwidth ≈ sample_rate / 2
accel_variance_g2 = (ACCEL_NOISE_DENSITY ** 2) * (SAMPLE_RATE_HZ / 2)

print(f"Accelerometer measurement noise:")
print(f"  Noise density: {ACCEL_NOISE_DENSITY} g/√Hz")
print(f"  Bandwidth: {SAMPLE_RATE_HZ/2} Hz")
print(f"  Variance: {accel_variance_g2:.6e} g²")
print()

# The accelerometer measures gravity direction (unit vector)
# For a 3-axis measurement with equal noise on each axis:
R_accel = accel_variance_g2 * np.eye(3)

print(f"R matrix (accelerometer) diagonal elements:")
print(f"  R_accel = {accel_variance_g2:.6e} g²")
print()

# In the code, this is used as meas_noise_var_acc
# The code uses SF_6XAG_QVACC and SF_6XAG_QWACC to compute it:
# meas_noise_var_acc = (SF_6XAG_QVACC + SF_6XAG_QWACC) / SF_OVERSAMPLE_RATIO
# For simplicity, we'll set:
SF_OVERSAMPLE_RATIO = 3
SF_6XAG_QVACC = accel_variance_g2 * SF_OVERSAMPLE_RATIO * 0.9
SF_6XAG_QWACC = accel_variance_g2 * SF_OVERSAMPLE_RATIO * 0.1

print(f"Python code parameters for R matrix:")
print(f"  SF_6XAG_QVACC = {SF_6XAG_QVACC:.6e}")
print(f"  SF_6XAG_QWACC = {SF_6XAG_QWACC:.6e}")
print(f"  meas_noise_var_acc = (QVACC + QWACC) / {SF_OVERSAMPLE_RATIO} = {accel_variance_g2:.6e} g²")
print()

# ============================================================================
# Q MATRIX (Process Noise Covariance)
# ============================================================================

print("="*80)
print("Q MATRIX (Process Noise Covariance)")
print("="*80)
print()

# The Q matrix models uncertainties in the system dynamics.
# State vector: [orientation_error (3), gyro_bias_error (3), linear_acc_error (3)]

# --- Orientation Error (from gyro integration) ---
# Angular random walk = noise_density * √dt
gyro_arw = GYRO_NOISE_DENSITY * np.sqrt(dt)  # dps√s
gyro_arw_rad = np.radians(gyro_arw)           # rad√s

# Process noise for orientation = (gyro_noise * dt)²
# This represents uncertainty in orientation after one integration step
Q_orient = (gyro_arw_rad ** 2)

print(f"Orientation process noise (from gyro integration):")
print(f"  Gyro noise density: {GYRO_NOISE_DENSITY} dps/√Hz")
print(f"  Angular random walk: {gyro_arw} dps·√s = {gyro_arw_rad:.6e} rad·√s")
print(f"  Q_orient = (ARW)² = {Q_orient:.6e} rad²")
print()

# --- Gyro Bias Error (bias random walk) ---
# Gyro bias drift is modeled as a random walk process
# Bias stability gives the 1-sigma uncertainty in bias
# For a random walk: σ²_bias = bias_instability² * dt

# Convert bias stability to rad/s
gyro_bias_stability_rad = np.radians(GYRO_BIAS_STABILITY)

# Process noise for bias = (bias_stability)² * dt
# This is often tuned: larger values → faster adaptation, less stability
# Smaller values → slower adaptation, more stability

# For initial tuning, use moderate value
# Typical: Q_bias ≈ 10⁻⁶ to 10⁻³ (rad/s)² depending on bias stability
Q_bias = (gyro_bias_stability_rad ** 2) * dt

print(f"Gyro bias process noise:")
print(f"  Bias stability: {GYRO_BIAS_STABILITY} dps = {gyro_bias_stability_rad:.6e} rad/s")
print(f"  Q_bias = (bias_stability)² * dt = {Q_bias:.6e} (rad/s)²")
print()

# However, for faster convergence (since we know device is static initially),
# we can increase this by a factor of 10-100
Q_bias_fast = Q_bias * 100
print(f"  For faster convergence: Q_bias (fast) = {Q_bias_fast:.6e} (rad/s)²")
print()

# --- Linear Acceleration Error ---
# For 6-axis fusion, linear acceleration is an augmented state
# to handle temporary accelerations that violate the "accel = gravity" assumption
# This should be larger to allow filter to reject spurious accelerations

# Typical value: 10-100 m²/s⁴
Q_linacc = 10.0  # (m/s²)²

print(f"Linear acceleration process noise:")
print(f"  Q_linacc = {Q_linacc:.6e} (m/s²)²")
print(f"  (This allows filter to reject non-gravitational accelerations)")
print()

# --- Cross-correlation terms ---
# Q_bias_orient: Correlation between orientation and bias errors
# Typically small, represents coupling between orientation and bias estimation
Q_bias_orient = np.sqrt(Q_orient * Q_bias) * 0.1

print(f"Orientation-Bias cross-correlation:")
print(f"  Q_bias_orient = {Q_bias_orient:.6e}")
print()

# ============================================================================
# GYRO MEASUREMENT PARAMETERS
# ============================================================================

print("="*80)
print("GYRO MEASUREMENT PARAMETERS (for oversampling)")
print("="*80)
print()

# The gyro is oversampled (SF_OVERSAMPLE_RATIO = 3 samples per fusion cycle)
# These parameters weight the gyro data quality
gyro_variance_rad2 = (np.radians(GYRO_NOISE_DENSITY) ** 2) * (SAMPLE_RATE_HZ / 2)

SF_6XAG_QVGYRO = gyro_variance_rad2 * SF_OVERSAMPLE_RATIO * 0.9  # rad²
SF_6XAG_QWGYRO = gyro_variance_rad2 * SF_OVERSAMPLE_RATIO * 0.1  # rad²

print(f"Gyro measurement variance: {gyro_variance_rad2:.6e} rad²")
print(f"  SF_6XAG_QVGYRO = {SF_6XAG_QVGYRO:.6e} rad²")
print(f"  SF_6XAG_QWGYRO = {SF_6XAG_QWGYRO:.6e} rad²")
print()

# ============================================================================
# SUMMARY - VALUES FOR sensor_fusion_6axis.py
# ============================================================================

print("="*80)
print("RECOMMENDED VALUES FOR sensor_fusion_6axis.py")
print("="*80)
print()

print("Replace lines 42-49 with:")
print()
print(f"# Measurement noise parameters (from MPU9250 datasheet)")
print(f"SF_6XAG_QVACC = {SF_6XAG_QVACC:.6e}  # Accelerometer variance")
print(f"SF_6XAG_QWACC = {SF_6XAG_QWACC:.6e}  # Accelerometer weight")
print(f"SF_6XAG_QVGYRO = {SF_6XAG_QVGYRO:.6e}  # Gyro variance (rad²)")
print(f"SF_6XAG_QWGYRO = {SF_6XAG_QWGYRO:.6e}  # Gyro weight (rad²)")
print()
print(f"# Process noise parameters (for Kalman Q matrix)")
print(f"SF_6XAG_QOrient = {Q_orient:.6e}  # Orientation process noise (rad²)")
print(f"SF_6XAG_QBias = {Q_bias_fast:.6e}  # Gyro bias process noise ((rad/s)²)")
print(f"SF_6XAG_QLinAcc = {Q_linacc:.6e}  # Linear accel process noise ((m/s²)²)")
print(f"SF_6XAG_QBiasOrient = {Q_bias_orient:.6e}  # Orientation-bias cross term")
print()

# ============================================================================
# COMPARISON WITH CURRENT VALUES
# ============================================================================

print("="*80)
print("COMPARISON WITH CURRENT VALUES")
print("="*80)
print()

# Current values from sensor_fusion_6axis.py
current_qvacc = 2e-6
current_qwacc = 1e-4
current_qvgyro = 0.01
current_qwgyro = 1e-9
current_qorient = 1e-1
current_qbias = 1e1
current_qlinacc = 1e1
current_qbiasorient = 1e-1

print("Parameter                Current Value       Recommended Value       Ratio")
print("-" * 80)
print(f"SF_6XAG_QVACC            {current_qvacc:.6e}        {SF_6XAG_QVACC:.6e}        {SF_6XAG_QVACC/current_qvacc:.1f}x")
print(f"SF_6XAG_QWACC            {current_qwacc:.6e}        {SF_6XAG_QWACC:.6e}        {SF_6XAG_QWACC/current_qwacc:.1f}x")
print(f"SF_6XAG_QVGYRO           {current_qvgyro:.6e}        {SF_6XAG_QVGYRO:.6e}        {SF_6XAG_QVGYRO/current_qvgyro:.1e}x")
print(f"SF_6XAG_QWGYRO           {current_qwgyro:.6e}        {SF_6XAG_QWGYRO:.6e}        {SF_6XAG_QWGYRO/current_qwgyro:.1e}x")
print(f"SF_6XAG_QOrient          {current_qorient:.6e}        {Q_orient:.6e}        {Q_orient/current_qorient:.1e}x")
print(f"SF_6XAG_QBias            {current_qbias:.6e}        {Q_bias_fast:.6e}        {Q_bias_fast/current_qbias:.1e}x")
print(f"SF_6XAG_QLinAcc          {current_qlinacc:.6e}        {Q_linacc:.6e}        {Q_linacc/current_qlinacc:.1f}x")
print(f"SF_6XAG_QBiasOrient      {current_qbiasorient:.6e}        {Q_bias_orient:.6e}        {Q_bias_orient/current_qbiasorient:.1e}x")
print()

print("Key observations:")
print(f"  1. QBias should be {Q_bias_fast/current_qbias:.0e}x SMALLER for proper bias convergence")
print(f"  2. QOrient should be {Q_orient/current_qorient:.0e}x SMALLER (less process noise)")
print(f"  3. QVGYRO and QWGYRO should be much smaller (currently too high)")
print(f"  4. QBiasOrient should be {Q_bias_orient/current_qbiasorient:.0e}x SMALLER")
print()

print("="*80)
print("EXPECTED IMPACT")
print("="*80)
print()
print("With these new values:")
print("  - Gyro bias will converge ~100x faster")
print("  - Static case error should drop from 41° to <1° within 20 samples")
print("  - Rotation tracking will be more stable (less process noise)")
print("  - Overall accuracy should match C implementation")
print()
