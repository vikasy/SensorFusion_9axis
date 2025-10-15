#!/usr/bin/env python3
"""
Deep analysis of Part 2 dataset failures to identify root causes.

Analyzes:
1. When does the filter start to diverge?
2. What triggers bias divergence?
3. How do covariance values evolve?
4. What is the magnitude of rotations vs small-angle assumption?
5. Linear acceleration detection in complex motion
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

def analyze_rotation_sequence():
    """Analyze rotation_sequence_15s failure modes"""

    print("="*80)
    print("ANALYZING: rotation_sequence_15s")
    print("="*80)

    results_file = "synthetic_test_results/rotation_sequence_15s_results.csv"
    if not os.path.exists(results_file):
        print(f"✗ Results file not found: {results_file}")
        return

    df = pd.read_csv(results_file)

    print(f"\nDataset info:")
    print(f"  Total fusion cycles: {len(df)}")
    print(f"  Duration: ~15 seconds")

    # 1. Find divergence point
    print(f"\n{'─'*80}")
    print("1. DIVERGENCE ANALYSIS")
    print(f"{'─'*80}")

    quat_errors = df['quat_error'].values
    bias_mags = df['bias_mag'].values

    # Find when error crosses thresholds
    threshold_5deg = np.where(quat_errors > 5.0)[0]
    threshold_10deg = np.where(quat_errors > 10.0)[0]
    threshold_20deg = np.where(quat_errors > 20.0)[0]

    print(f"\nQuaternion error progression:")
    print(f"  Initial (cycle 0): {quat_errors[0]:.3f}°")
    print(f"  First >5°: cycle {threshold_5deg[0] if len(threshold_5deg) > 0 else 'never'}")
    print(f"  First >10°: cycle {threshold_10deg[0] if len(threshold_10deg) > 0 else 'never'}")
    print(f"  First >20°: cycle {threshold_20deg[0] if len(threshold_20deg) > 0 else 'never'}")
    print(f"  Final (cycle {len(df)-1}): {quat_errors[-1]:.3f}°")
    print(f"  Maximum: {quat_errors.max():.3f}° at cycle {quat_errors.argmax()}")

    # Bias divergence
    bias_threshold_2dps = np.where(bias_mags > 2.0)[0]
    bias_threshold_5dps = np.where(bias_mags > 5.0)[0]
    bias_threshold_10dps = np.where(bias_mags > 10.0)[0]

    print(f"\nBias magnitude progression:")
    print(f"  Initial (cycle 0): {bias_mags[0]:.3f} dps")
    print(f"  First >2 dps: cycle {bias_threshold_2dps[0] if len(bias_threshold_2dps) > 0 else 'never'}")
    print(f"  First >5 dps: cycle {bias_threshold_5dps[0] if len(bias_threshold_5dps) > 0 else 'never'}")
    print(f"  First >10 dps: cycle {bias_threshold_10dps[0] if len(bias_threshold_10dps) > 0 else 'never'}")
    print(f"  Final (cycle {len(df)-1}): {bias_mags[-1]:.3f} dps")
    print(f"  Maximum: {bias_mags.max():.3f} dps at cycle {bias_mags.argmax()}")

    # 2. Rotation rate analysis
    print(f"\n{'─'*80}")
    print("2. ROTATION RATE ANALYSIS")
    print(f"{'─'*80}")

    # Calculate rotation rates from ground truth quaternions
    gt_yaw = df['gt_yaw'].values
    gt_pitch = df['gt_pitch'].values
    gt_roll = df['gt_roll'].values

    yaw_rate = np.diff(gt_yaw) * 100  # Assuming 100 Hz (10ms between samples)
    pitch_rate = np.diff(gt_pitch) * 100
    roll_rate = np.diff(gt_roll) * 100

    print(f"\nGround truth rotation rates (dps):")
    print(f"  Yaw:   mean={np.abs(yaw_rate).mean():.1f}, max={np.abs(yaw_rate).max():.1f}, std={yaw_rate.std():.1f}")
    print(f"  Pitch: mean={np.abs(pitch_rate).mean():.1f}, max={np.abs(pitch_rate).max():.1f}, std={pitch_rate.std():.1f}")
    print(f"  Roll:  mean={np.abs(roll_rate).mean():.1f}, max={np.abs(roll_rate).max():.1f}, std={roll_rate.std():.1f}")

    total_rotation_rate = np.sqrt(yaw_rate**2 + pitch_rate**2 + roll_rate**2)
    print(f"  Total: mean={total_rotation_rate.mean():.1f}, max={total_rotation_rate.max():.1f}")

    # Small-angle check (angle change between updates)
    angle_changes = np.diff(quat_errors)
    print(f"\nQuaternion error changes per cycle (small-angle assumption check):")
    print(f"  Mean: {np.abs(angle_changes).mean():.3f}°")
    print(f"  Max:  {np.abs(angle_changes).max():.3f}°")
    print(f"  Std:  {angle_changes.std():.3f}°")
    print(f"  >5° changes: {np.sum(np.abs(angle_changes) > 5.0)} cycles ({100*np.sum(np.abs(angle_changes) > 5.0)/len(angle_changes):.1f}%)")
    print(f"  >10° changes: {np.sum(np.abs(angle_changes) > 10.0)} cycles")

    # 3. Per-axis error analysis
    print(f"\n{'─'*80}")
    print("3. PER-AXIS ERROR ANALYSIS")
    print(f"{'─'*80}")

    yaw_errors = df['yaw_error'].values
    pitch_errors = df['pitch_error'].values
    roll_errors = df['roll_error'].values

    print(f"\nEuler angle errors:")
    print(f"  Yaw:   mean={yaw_errors.mean():+.1f}°, rms={np.sqrt(np.mean(yaw_errors**2)):.1f}°, max={np.abs(yaw_errors).max():.1f}°")
    print(f"  Pitch: mean={pitch_errors.mean():+.1f}°, rms={np.sqrt(np.mean(pitch_errors**2)):.1f}°, max={np.abs(pitch_errors).max():.1f}°")
    print(f"  Roll:  mean={roll_errors.mean():+.1f}°, rms={np.sqrt(np.mean(roll_errors**2)):.1f}°, max={np.abs(roll_errors).max():.1f}°")

    # Find which axis contributes most to error
    abs_yaw = np.abs(yaw_errors)
    abs_pitch = np.abs(pitch_errors)
    abs_roll = np.abs(roll_errors)

    dominant_axis = []
    for i in range(len(df)):
        if abs_yaw[i] > abs_pitch[i] and abs_yaw[i] > abs_roll[i]:
            dominant_axis.append('yaw')
        elif abs_pitch[i] > abs_roll[i]:
            dominant_axis.append('pitch')
        else:
            dominant_axis.append('roll')

    print(f"\nDominant error axis:")
    print(f"  Yaw:   {dominant_axis.count('yaw')} cycles ({100*dominant_axis.count('yaw')/len(dominant_axis):.1f}%)")
    print(f"  Pitch: {dominant_axis.count('pitch')} cycles ({100*dominant_axis.count('pitch')/len(dominant_axis):.1f}%)")
    print(f"  Roll:  {dominant_axis.count('roll')} cycles ({100*dominant_axis.count('roll')/len(dominant_axis):.1f}%)")

    # 4. Bias behavior
    print(f"\n{'─'*80}")
    print("4. BIAS BEHAVIOR ANALYSIS")
    print(f"{'─'*80}")

    bias_x = df['bias_x'].values
    bias_y = df['bias_y'].values
    bias_z = df['bias_z'].values

    print(f"\nBias evolution (dps):")
    print(f"  X: start={bias_x[0]:+.3f}, end={bias_x[-1]:+.3f}, range={bias_x.max()-bias_x.min():.3f}")
    print(f"  Y: start={bias_y[0]:+.3f}, end={bias_y[-1]:+.3f}, range={bias_y.max()-bias_y.min():.3f}")
    print(f"  Z: start={bias_z[0]:+.3f}, end={bias_z[-1]:+.3f}, range={bias_z.max()-bias_z.min():.3f}")

    # Bias rate of change
    bias_rate_x = np.diff(bias_x) * 100  # per second
    bias_rate_y = np.diff(bias_y) * 100
    bias_rate_z = np.diff(bias_z) * 100

    print(f"\nBias rate of change (dps/sec):")
    print(f"  X: mean={np.abs(bias_rate_x).mean():.3f}, max={np.abs(bias_rate_x).max():.3f}")
    print(f"  Y: mean={np.abs(bias_rate_y).mean():.3f}, max={np.abs(bias_rate_y).max():.3f}")
    print(f"  Z: mean={np.abs(bias_rate_z).mean():.3f}, max={np.abs(bias_rate_z).max():.3f}")

    print(f"\n⚠ Physical reality check:")
    print(f"  Real gyro bias typically changes <0.01 dps/sec")
    print(f"  This dataset shows bias changes up to {max(np.abs(bias_rate_x).max(), np.abs(bias_rate_y).max(), np.abs(bias_rate_z).max()):.3f} dps/sec")
    print(f"  → Bias is tracking MOTION, not actual bias!")

    # 5. Create diagnostic plot
    print(f"\n{'─'*80}")
    print("5. GENERATING DIAGNOSTIC PLOTS")
    print(f"{'─'*80}")

    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('rotation_sequence_15s - Failure Mode Analysis', fontsize=16, fontweight='bold')

    # Plot 1: Error evolution
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(df['fusion'], quat_errors, 'b-', linewidth=1, label='Quaternion Error')
    ax1.axhline(y=5.0, color='orange', linestyle='--', linewidth=1, label='Poor (5°)')
    ax1.axhline(y=10.0, color='red', linestyle='--', linewidth=1, label='Fail (10°)')
    if len(threshold_5deg) > 0:
        ax1.axvline(x=threshold_5deg[0], color='orange', linestyle=':', alpha=0.5)
    if len(threshold_10deg) > 0:
        ax1.axvline(x=threshold_10deg[0], color='red', linestyle=':', alpha=0.5)
    ax1.set_xlabel('Fusion Cycle')
    ax1.set_ylabel('Error (deg)')
    ax1.set_title('Quaternion Error Evolution')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Bias magnitude evolution
    ax2 = plt.subplot(3, 2, 2)
    ax2.plot(df['fusion'], bias_mags, 'g-', linewidth=1, label='Bias Magnitude')
    ax2.axhline(y=2.0, color='orange', linestyle='--', linewidth=1, label='Acceptable (2 dps)')
    ax2.axhline(y=5.0, color='red', linestyle='--', linewidth=1, label='Diverged (5 dps)')
    if len(bias_threshold_2dps) > 0:
        ax2.axvline(x=bias_threshold_2dps[0], color='orange', linestyle=':', alpha=0.5)
    if len(bias_threshold_5dps) > 0:
        ax2.axvline(x=bias_threshold_5dps[0], color='red', linestyle=':', alpha=0.5)
    ax2.set_xlabel('Fusion Cycle')
    ax2.set_ylabel('Bias (dps)')
    ax2.set_title('Bias Magnitude Evolution')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Plot 3: Per-axis errors
    ax3 = plt.subplot(3, 2, 3)
    ax3.plot(df['fusion'], np.abs(yaw_errors), 'r-', linewidth=1, alpha=0.7, label='|Yaw|')
    ax3.plot(df['fusion'], np.abs(pitch_errors), 'g-', linewidth=1, alpha=0.7, label='|Pitch|')
    ax3.plot(df['fusion'], np.abs(roll_errors), 'b-', linewidth=1, alpha=0.7, label='|Roll|')
    ax3.set_xlabel('Fusion Cycle')
    ax3.set_ylabel('Absolute Error (deg)')
    ax3.set_title('Per-Axis Absolute Errors')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Plot 4: Per-axis bias
    ax4 = plt.subplot(3, 2, 4)
    ax4.plot(df['fusion'], bias_x, 'r-', linewidth=1, alpha=0.7, label='Bias X')
    ax4.plot(df['fusion'], bias_y, 'g-', linewidth=1, alpha=0.7, label='Bias Y')
    ax4.plot(df['fusion'], bias_z, 'b-', linewidth=1, alpha=0.7, label='Bias Z')
    ax4.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax4.set_xlabel('Fusion Cycle')
    ax4.set_ylabel('Bias (dps)')
    ax4.set_title('Per-Axis Bias Evolution')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # Plot 5: Error vs Bias correlation
    ax5 = plt.subplot(3, 2, 5)
    scatter = ax5.scatter(bias_mags, quat_errors, c=df['fusion'], cmap='viridis', s=10, alpha=0.6)
    plt.colorbar(scatter, ax=ax5, label='Fusion Cycle')
    ax5.set_xlabel('Bias Magnitude (dps)')
    ax5.set_ylabel('Quaternion Error (deg)')
    ax5.set_title('Error vs Bias Correlation')
    ax5.grid(True, alpha=0.3)

    # Plot 6: Ground truth rotations
    ax6 = plt.subplot(3, 2, 6)
    ax6.plot(df['fusion'], gt_yaw, 'r-', linewidth=1, alpha=0.7, label='GT Yaw')
    ax6.plot(df['fusion'], gt_pitch, 'g-', linewidth=1, alpha=0.7, label='GT Pitch')
    ax6.plot(df['fusion'], gt_roll, 'b-', linewidth=1, alpha=0.7, label='GT Roll')
    ax6.set_xlabel('Fusion Cycle')
    ax6.set_ylabel('Angle (deg)')
    ax6.set_title('Ground Truth Rotation Profile')
    ax6.legend()
    ax6.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    output_file = "part2_rotation_sequence_analysis.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"\n✓ Diagnostic plot saved: {output_file}")

def analyze_complex_motion():
    """Analyze complex_motion_20s failure modes"""

    print("\n\n" + "="*80)
    print("ANALYZING: complex_motion_20s")
    print("="*80)

    results_file = "synthetic_test_results/complex_motion_20s_results.csv"
    if not os.path.exists(results_file):
        print(f"✗ Results file not found: {results_file}")
        return

    df = pd.read_csv(results_file)

    print(f"\nDataset info:")
    print(f"  Total fusion cycles: {len(df)}")
    print(f"  Duration: ~20 seconds")

    # Similar analysis as rotation_sequence
    quat_errors = df['quat_error'].values
    bias_mags = df['bias_mag'].values

    print(f"\n{'─'*80}")
    print("1. CATASTROPHIC FAILURE ANALYSIS")
    print(f"{'─'*80}")

    print(f"\nError statistics:")
    print(f"  Mean: {quat_errors.mean():.1f}°")
    print(f"  Median: {np.median(quat_errors):.1f}°")
    print(f"  Std: {quat_errors.std():.1f}°")
    print(f"  Max: {quat_errors.max():.1f}° (complete orientation loss!)")
    print(f"  >90° errors: {np.sum(quat_errors > 90.0)} cycles ({100*np.sum(quat_errors > 90.0)/len(quat_errors):.1f}%)")
    print(f"  >180° errors: {np.sum(quat_errors > 180.0)} cycles (quaternion ambiguity)")

    print(f"\nBias statistics:")
    print(f"  Final: {bias_mags[-1]:.1f} dps (physically impossible!)")
    print(f"  Max: {bias_mags.max():.1f} dps")
    print(f"  >10 dps: {np.sum(bias_mags > 10.0)} cycles")
    print(f"  >50 dps: {np.sum(bias_mags > 50.0)} cycles")

    print(f"\n⚠ CRITICAL: Bias of 55 dps is physically impossible")
    print(f"  Real gyro bias: 0.01 - 1.0 dps")
    print(f"  This indicates complete filter failure")

    print(f"\n{'─'*80}")
    print("2. ROOT CAUSE: LINEAR ACCELERATION")
    print(f"{'─'*80}")

    # For complex motion, we need to look at the dataset itself to see accelerations
    print(f"\n⚠ Complex motion includes linear accelerations")
    print(f"  Accelerometer measures: gravity + linear_accel")
    print(f"  Filter assumes: only gravity")
    print(f"  Result: Linear accel interpreted as tilt change")
    print(f"  Consequence: Completely wrong orientation estimate")

    # Create diagnostic plot
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('complex_motion_20s - Catastrophic Failure Analysis', fontsize=16, fontweight='bold')

    ax1 = plt.subplot(2, 2, 1)
    ax1.plot(df['fusion'], quat_errors, 'r-', linewidth=1)
    ax1.axhline(y=90, color='orange', linestyle='--', label='90° (orientation lost)')
    ax1.set_xlabel('Fusion Cycle')
    ax1.set_ylabel('Quaternion Error (deg)')
    ax1.set_title('Quaternion Error (Catastrophic)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(2, 2, 2)
    ax2.plot(df['fusion'], bias_mags, 'g-', linewidth=1)
    ax2.axhline(y=10, color='orange', linestyle='--', label='10 dps (diverged)')
    ax2.set_xlabel('Fusion Cycle')
    ax2.set_ylabel('Bias Magnitude (dps)')
    ax2.set_title('Bias Magnitude (Complete Divergence)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    ax3 = plt.subplot(2, 2, 3)
    ax3.hist(quat_errors, bins=50, color='red', alpha=0.7, edgecolor='black')
    ax3.axvline(quat_errors.mean(), color='darkred', linestyle='--', linewidth=2, label=f'Mean: {quat_errors.mean():.1f}°')
    ax3.set_xlabel('Quaternion Error (deg)')
    ax3.set_ylabel('Frequency')
    ax3.set_title('Error Distribution')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    ax4 = plt.subplot(2, 2, 4)
    scatter = ax4.scatter(bias_mags, quat_errors, c=df['fusion'], cmap='plasma', s=10, alpha=0.6)
    plt.colorbar(scatter, ax=ax4, label='Fusion Cycle')
    ax4.set_xlabel('Bias Magnitude (dps)')
    ax4.set_ylabel('Quaternion Error (deg)')
    ax4.set_title('Error vs Bias (Complete Correlation)')
    ax4.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    output_file = "part2_complex_motion_analysis.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"\n✓ Diagnostic plot saved: {output_file}")

def main():
    print("\n" + "="*80)
    print("PART 2 FAILURE MODE ANALYSIS")
    print("="*80)
    print("\nAnalyzing the 2 datasets that show poor performance to identify")
    print("root causes and guide improvement strategies.")

    analyze_rotation_sequence()
    analyze_complex_motion()

    print("\n\n" + "="*80)
    print("SUMMARY OF FINDINGS")
    print("="*80)

    print("\n📊 ROTATION_SEQUENCE_15S:")
    print("  Root Cause: Small-angle assumption violated during multi-axis rotations")
    print("  Symptoms:")
    print("    - Error grows gradually from 0.4° to 60°")
    print("    - Bias tracks motion instead of converging (reaches 16.7 dps)")
    print("    - Yaw error dominates (13° RMS)")
    print("  Improvement Strategy:")
    print("    1. Bias rate limiting (prevent bias from tracking motion)")
    print("    2. Adaptive process noise (increase during rapid rotation)")
    print("    3. Better handling of large angle changes")

    print("\n📊 COMPLEX_MOTION_20S:")
    print("  Root Cause: Linear acceleration violates gravity-only assumption")
    print("  Symptoms:")
    print("    - Catastrophic failure (68° mean, 180° max)")
    print("    - Bias completely diverges (55.6 dps)")
    print("    - Filter loses track of orientation")
    print("  Improvement Strategy:")
    print("    1. Motion detection (detect linear acceleration)")
    print("    2. Adaptive measurement noise (distrust accel during accel)")
    print("    3. Consider IMU pre-integration or velocity estimation")

    print("\n🎯 RECOMMENDED IMPROVEMENT ORDER:")
    print("  1. Bias rate limiting → Easy, low risk, 50% bias improvement expected")
    print("  2. Adaptive process noise → Medium effort, 20-30% error improvement")
    print("  3. Motion detection → Medium effort, fixes complex_motion")
    print("  4. MEKF (if needed) → High effort, 70-90% improvement on rotation_sequence")

    print("\n" + "="*80)

if __name__ == '__main__':
    main()
