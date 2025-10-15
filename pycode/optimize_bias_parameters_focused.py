#!/usr/bin/env python3
"""
Phase 1: Focused Grid Search Optimization for Bias Limiting Parameters

Tests a small focused set of parameter combinations around current values
to find improvements quickly.

Author: Automated optimization script
Date: 2025-10-14
"""

import numpy as np
import pandas as pd
import sys
import os
import json
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))

# Import after path setup
import sensor_fusion_9axis
from sensor_platform_config import SensorPlatform, INVENSENSE

# Load baseline results for comparison
BASELINE_FILE = "regression_baseline.json"

# FOCUSED parameter search spaces - test variations around current values
# Current: THRESHOLD_SLOW=15.0, THRESHOLD_FAST=40.0, RATE_MODERATE=0.05, RATE_FAST=0.01

# Test combinations:
# 1. Current baseline
# 2. More aggressive slow threshold (allow more convergence)
# 3. Less aggressive slow threshold (more protection)
# 4. Higher fast threshold (less frequent strong limiting)
# 5. Lower fast threshold (more frequent strong limiting)
# 6. More aggressive moderate rate
# 7. Less aggressive moderate rate
# 8. More aggressive fast rate
# 9. Less aggressive fast rate

PARAMETER_COMBINATIONS = [
    # (slow, fast, moderate_rate, fast_rate, description)
    (15.0, 40.0, 0.05, 0.01, "Current baseline"),
    (12.0, 40.0, 0.05, 0.01, "Lower slow threshold (allow more convergence)"),
    (18.0, 40.0, 0.05, 0.01, "Higher slow threshold (more protection)"),
    (15.0, 35.0, 0.05, 0.01, "Lower fast threshold (more limiting)"),
    (15.0, 45.0, 0.05, 0.01, "Higher fast threshold (less limiting)"),
    (15.0, 40.0, 0.07, 0.01, "Higher moderate rate (allow faster change)"),
    (15.0, 40.0, 0.03, 0.01, "Lower moderate rate (more constraining)"),
    (15.0, 40.0, 0.05, 0.015, "Higher fast rate (allow faster change)"),
    (15.0, 40.0, 0.05, 0.007, "Lower fast rate (more constraining)"),
    (12.0, 45.0, 0.07, 0.015, "Most permissive (allow faster convergence)"),
    (18.0, 35.0, 0.03, 0.007, "Most restrictive (strong protection)"),
]

# Part 1 datasets (must not regress)
PART1_DATASETS = [
    'static_60s',
    'static_10s',
    'static_high_bias_10s',
    'static_high_noise_10s',
    'rotation_x_20dps_10s',
    'rotation_y_15dps_10s',
    'rotation_z_30dps_10s',
    'vibration_5hz_10s'
]

# Part 2 datasets (target for improvement)
PART2_DATASETS = [
    'rotation_sequence_15s',
    'complex_motion_20s'
]

# Dataset paths
DATASET_BASE = "../test/data/datasets/synthetic"

def create_fusion_with_parameters(platform, threshold_slow, threshold_fast, rate_moderate, rate_fast):
    """Create SensorFusion9Axis instance with specified parameters"""
    from sensor_fusion_9axis import SensorFusion9Axis
    return SensorFusion9Axis(
        acc_scale=platform.accel_scale_factor,
        gyro_scale=platform.gyro_scale_factor,
        mag_scale=platform.mag_scale_factor,
        motion_threshold_slow=threshold_slow,
        motion_threshold_fast=threshold_fast,
        max_bias_rate_moderate=rate_moderate,
        max_bias_rate_fast=rate_fast
    )

def load_baseline():
    """Load baseline metrics from regression test"""
    if not os.path.exists(BASELINE_FILE):
        print(f"ERROR: Baseline file {BASELINE_FILE} not found!")
        print("Run regression_test.py --update-baseline first")
        sys.exit(1)

    with open(BASELINE_FILE, 'r') as f:
        baseline = json.load(f)

    return baseline['datasets']

def test_parameter_combination(threshold_slow, threshold_fast, rate_moderate, rate_fast):
    """
    Test a single parameter combination on all datasets

    Returns:
        dict with results for all datasets
    """
    # Test all datasets
    results = {}

    for dataset_name in PART1_DATASETS + PART2_DATASETS:
        dataset_path = os.path.join(DATASET_BASE, f"{dataset_name}.csv")

        if not os.path.exists(dataset_path):
            print(f"  WARNING: Dataset {dataset_name} not found")
            continue

        try:
            # Load dataset
            df = pd.read_csv(dataset_path)

            # Initialize fusion with specified parameters
            platform = SensorPlatform(INVENSENSE)
            sf = create_fusion_with_parameters(
                platform, threshold_slow, threshold_fast, rate_moderate, rate_fast
            )

            # Process all samples
            fusion_count = 0
            quat_errors = []
            bias_values = []

            for idx in range(len(df)):
                row = df.iloc[idx]
                timestamp = int(row['timestamp_ns'])

                accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']], dtype=np.float64)
                gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']], dtype=np.float64)
                mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']], dtype=np.float64)

                ready_acc = sf.preprocess_sensor_data(0, accel_counts, timestamp)
                ready_gyro = sf.preprocess_sensor_data(1, gyro_counts, timestamp)
                ready_mag = sf.preprocess_sensor_data(2, mag_counts, timestamp)

                if ready_gyro & 0x2:
                    fusion_count += 1
                    output = sf.run()

                    # Get ground truth
                    gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
                    python_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

                    # Compute error
                    dot_product = np.dot(gt_quat, python_quat)
                    quat_error = 2 * np.arccos(np.clip(abs(dot_product), 0, 1)) * 180 / np.pi
                    quat_errors.append(quat_error)

                    # Get bias
                    bias = sf.bias_post_s.copy()
                    bias_mag = np.linalg.norm(bias)
                    bias_values.append(bias_mag)

            # Store results
            results[dataset_name] = {
                'quat_error_mean': np.mean(quat_errors),
                'quat_error_max': np.max(quat_errors),
                'bias_final': bias_values[-1] if bias_values else 0,
                'bias_max': np.max(bias_values) if bias_values else 0
            }

        except Exception as e:
            print(f"  ERROR testing {dataset_name}: {e}")
            results[dataset_name] = None

    return results

def compute_score(results, baseline):
    """
    Compute optimization score

    Score components:
    - Part 1 regression penalty: -100 per dataset that regresses >10%
    - Part 2 improvement reward: +error_reduction + bias_reduction

    Returns:
        tuple: (total_score, part1_regressions, part2_improvement)
    """
    score = 0
    part1_regressions = 0
    part2_improvements = {}

    # Check Part 1 (regression penalty)
    for dataset in PART1_DATASETS:
        if dataset not in results or results[dataset] is None:
            score -= 1000  # Missing data penalty
            continue

        if dataset not in baseline:
            continue

        curr = results[dataset]
        base = baseline[dataset]

        # Check key metrics
        for metric in ['quat_error_mean', 'quat_error_max', 'bias_final']:
            if metric in curr and metric in base:
                base_val = base[metric]
                curr_val = curr[metric]

                if base_val > 0:
                    pct_change = ((curr_val - base_val) / base_val) * 100

                    if pct_change > 10.0:  # Regression threshold
                        score -= 100
                        part1_regressions += 1

    # Check Part 2 (improvement reward)
    # Baseline for Part 2 (from Iteration 1 results)
    part2_baseline = {
        'rotation_sequence_15s': {'quat_error_mean': 18.836, 'bias_final': 12.442},
        'complex_motion_20s': {'quat_error_mean': 53.592, 'bias_final': 23.880}
    }

    for dataset in PART2_DATASETS:
        if dataset not in results or results[dataset] is None:
            continue

        if dataset not in part2_baseline:
            continue

        curr = results[dataset]
        base = part2_baseline[dataset]

        # Error reduction (higher is better)
        error_reduction = base['quat_error_mean'] - curr['quat_error_mean']

        # Bias reduction (higher is better)
        bias_reduction = base['bias_final'] - curr['bias_final']

        # Weight: error reduction + 2*bias reduction
        dataset_improvement = error_reduction + 2 * bias_reduction
        score += dataset_improvement

        part2_improvements[dataset] = {
            'error_reduction': error_reduction,
            'bias_reduction': bias_reduction
        }

    return score, part1_regressions, part2_improvements

def main():
    print("="*80)
    print("PHASE 1: FOCUSED GRID SEARCH OPTIMIZATION")
    print("="*80)
    print(f"\nTesting {len(PARAMETER_COMBINATIONS)} focused parameter combinations")

    # Load baseline
    print("\nLoading baseline metrics...")
    baseline = load_baseline()
    print(f"  Loaded {len(baseline)} baseline datasets")

    # Estimate time
    estimated_time_per_combo = 15  # seconds
    estimated_total_time = len(PARAMETER_COMBINATIONS) * estimated_time_per_combo / 60
    print(f"\nEstimated time: {estimated_total_time:.1f} minutes")

    # Run optimization
    print("\n" + "="*80)
    print("STARTING OPTIMIZATION")
    print("="*80)

    all_results = []
    best_score = -np.inf
    best_params = None

    start_time = time.time()

    for idx, (slow, fast, moderate, rate_fast, desc) in enumerate(PARAMETER_COMBINATIONS):
        combo_num = idx + 1

        print(f"\n[{combo_num}/{len(PARAMETER_COMBINATIONS)}] {desc}")
        print(f"  slow={slow}, fast={fast}, moderate={moderate}, fast_rate={rate_fast}")

        # Test combination
        results = test_parameter_combination(slow, fast, moderate, rate_fast)

        # Compute score
        score, regressions, improvements = compute_score(results, baseline)

        print(f"  Score: {score:.2f}, Part1 regressions: {regressions}")

        # Store results
        all_results.append({
            'threshold_slow': slow,
            'threshold_fast': fast,
            'rate_moderate': moderate,
            'rate_fast': rate_fast,
            'description': desc,
            'score': score,
            'part1_regressions': regressions,
            'part2_improvements': improvements,
            'results': results
        })

        # Track best
        if score > best_score:
            best_score = score
            best_params = (slow, fast, moderate, rate_fast)
            print(f"  *** NEW BEST SCORE: {score:.2f} ***")

    elapsed_time = time.time() - start_time

    # Save all results
    output_file = "parameter_optimization_results_focused.json"
    with open(output_file, 'w') as f:
        # Convert numpy types to native Python types for JSON serialization
        def convert_types(obj):
            if isinstance(obj, dict):
                return {k: convert_types(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [convert_types(item) for item in obj]
            elif isinstance(obj, (np.integer, np.floating)):
                return float(obj)
            else:
                return obj

        json.dump(convert_types(all_results), f, indent=2)

    print("\n" + "="*80)
    print("OPTIMIZATION COMPLETE")
    print("="*80)
    print(f"\nTotal time: {elapsed_time/60:.1f} minutes")
    print(f"Results saved to: {output_file}")

    # Print best results
    print("\n" + "="*80)
    print("BEST PARAMETERS FOUND")
    print("="*80)
    print(f"\nScore: {best_score:.2f}")
    print(f"Parameters:")
    print(f"  THRESHOLD_SLOW: {best_params[0]}")
    print(f"  THRESHOLD_FAST: {best_params[1]}")
    print(f"  RATE_MODERATE: {best_params[2]}")
    print(f"  RATE_FAST: {best_params[3]}")

    # Find best result details
    best_result = [r for r in all_results if r['score'] == best_score][0]
    print(f"\nDescription: {best_result['description']}")
    print(f"Part 1 regressions: {best_result['part1_regressions']}")
    print(f"\nPart 2 improvements:")
    for dataset, improvements in best_result['part2_improvements'].items():
        print(f"  {dataset}:")
        print(f"    Error reduction: {improvements['error_reduction']:.2f}°")
        print(f"    Bias reduction: {improvements['bias_reduction']:.2f} dps")

    # Compare with current parameters
    current_result = [r for r in all_results if r['description'] == "Current baseline"][0]
    current_score = current_result['score']
    improvement = ((best_score - current_score) / abs(current_score) * 100) if current_score != 0 else 0
    print(f"\nImprovement vs current parameters:")
    print(f"  Current score: {current_score:.2f}")
    print(f"  Best score: {best_score:.2f}")
    print(f"  Improvement: {improvement:+.1f}%")

if __name__ == '__main__':
    main()
