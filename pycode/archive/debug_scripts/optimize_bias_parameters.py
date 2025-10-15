#!/usr/bin/env python3
"""
Phase 1: Grid Search Optimization for Bias Limiting Parameters

Systematically tests combinations of motion thresholds and rate limits
to find optimal values that:
1. Maintain Part 1 performance (no regressions)
2. Maximize Part 2 improvements (error and bias reduction)

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
from test_all_synthetic_datasets import test_dataset

# Load baseline results for comparison
BASELINE_FILE = "regression_baseline.json"

# Parameter search spaces - REDUCED for faster optimization
# Focus on values near current settings (15.0, 40.0, 0.05, 0.01)
THRESHOLD_SLOW_VALUES = [10.0, 15.0, 20.0]
THRESHOLD_FAST_VALUES = [30.0, 40.0, 50.0]
RATE_MODERATE_VALUES = [0.03, 0.05, 0.07]
RATE_FAST_VALUES = [0.005, 0.01, 0.015]

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

def set_parameters(threshold_slow, threshold_fast, rate_moderate, rate_fast):
    """Dynamically update parameters in sensor_fusion_9axis module"""
    sensor_fusion_9axis.SF_MOTION_THRESHOLD_SLOW = threshold_slow
    sensor_fusion_9axis.SF_MOTION_THRESHOLD_FAST = threshold_fast
    sensor_fusion_9axis.SF_MAX_BIAS_RATE_MODERATE = rate_moderate
    sensor_fusion_9axis.SF_MAX_BIAS_RATE_FAST = rate_fast

def load_baseline():
    """Load baseline metrics from regression test"""
    if not os.path.exists(BASELINE_FILE):
        print(f"ERROR: Baseline file {BASELINE_FILE} not found!")
        print("Run regression_test.py --update-baseline first")
        sys.exit(1)

    with open(BASELINE_FILE, 'r') as f:
        baseline = json.load(f)

    return baseline['datasets']

def test_parameter_combination(threshold_slow, threshold_fast, rate_moderate, rate_fast, output_dir="param_search_temp"):
    """
    Test a single parameter combination on all datasets

    Returns:
        dict with scores for Part 1 and Part 2
    """
    # Set parameters
    set_parameters(threshold_slow, threshold_fast, rate_moderate, rate_fast)

    # Reload module to pick up new parameters
    if 'sensor_fusion_9axis' in sys.modules:
        del sys.modules['sensor_fusion_9axis']
    from sensor_fusion_9axis import SensorFusion9Axis
    from sensor_platform_config import SensorPlatform, INVENSENSE

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

            # Initialize fusion
            platform = SensorPlatform(INVENSENSE)
            sf = SensorFusion9Axis(
                acc_scale=platform.accel_scale_factor,
                gyro_scale=platform.gyro_scale_factor,
                mag_scale=platform.mag_scale_factor
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
        'rotation_sequence_15s': {'quat_error_mean': 17.648, 'bias_final': 16.688},
        'complex_motion_20s': {'quat_error_mean': 68.571, 'bias_final': 55.575}
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
    print("PHASE 1: GRID SEARCH OPTIMIZATION")
    print("="*80)
    print("\nOptimizing 4 bias limiting parameters:")
    print(f"  - THRESHOLD_SLOW: {THRESHOLD_SLOW_VALUES}")
    print(f"  - THRESHOLD_FAST: {THRESHOLD_FAST_VALUES}")
    print(f"  - RATE_MODERATE: {RATE_MODERATE_VALUES}")
    print(f"  - RATE_FAST: {RATE_FAST_VALUES}")

    # Load baseline
    print("\nLoading baseline metrics...")
    baseline = load_baseline()
    print(f"  Loaded {len(baseline)} baseline datasets")

    # Calculate total combinations
    # Constraint: THRESHOLD_FAST > THRESHOLD_SLOW + 10
    valid_threshold_combos = []
    for slow in THRESHOLD_SLOW_VALUES:
        for fast in THRESHOLD_FAST_VALUES:
            if fast > slow + 10:
                valid_threshold_combos.append((slow, fast))

    # Constraint: RATE_MODERATE > RATE_FAST
    valid_rate_combos = []
    for moderate in RATE_MODERATE_VALUES:
        for fast in RATE_FAST_VALUES:
            if moderate > fast:
                valid_rate_combos.append((moderate, fast))

    total_combinations = len(valid_threshold_combos) * len(valid_rate_combos)

    print(f"\nValid combinations to test:")
    print(f"  - Threshold pairs: {len(valid_threshold_combos)}")
    print(f"  - Rate pairs: {len(valid_rate_combos)}")
    print(f"  - Total: {total_combinations}")

    # Estimate time
    estimated_time_per_combo = 15  # seconds (based on previous runs)
    estimated_total_time = total_combinations * estimated_time_per_combo / 60
    print(f"\nEstimated time: {estimated_total_time:.1f} minutes")

    proceed = input("\nProceed with grid search? (y/n): ")
    if proceed.lower() != 'y':
        print("Aborted")
        return

    # Run grid search
    print("\n" + "="*80)
    print("STARTING GRID SEARCH")
    print("="*80)

    all_results = []
    best_score = -np.inf
    best_params = None

    combo_num = 0
    start_time = time.time()

    for slow, fast in valid_threshold_combos:
        for moderate, rate_fast in valid_rate_combos:
            combo_num += 1

            print(f"\n[{combo_num}/{total_combinations}] Testing: slow={slow}, fast={fast}, moderate={moderate}, fast_rate={rate_fast}")

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
    output_file = "parameter_optimization_results.json"
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
    print(f"\nPart 1 regressions: {best_result['part1_regressions']}")
    print(f"Part 2 improvements:")
    for dataset, improvements in best_result['part2_improvements'].items():
        print(f"  {dataset}:")
        print(f"    Error reduction: {improvements['error_reduction']:.2f}°")
        print(f"    Bias reduction: {improvements['bias_reduction']:.2f} dps")

    # Compare with current parameters
    current_params = (15.0, 40.0, 0.05, 0.01)
    current_result = [r for r in all_results if (
        r['threshold_slow'] == current_params[0] and
        r['threshold_fast'] == current_params[1] and
        r['rate_moderate'] == current_params[2] and
        r['rate_fast'] == current_params[3]
    )]

    if current_result:
        current_score = current_result[0]['score']
        improvement = ((best_score - current_score) / abs(current_score) * 100) if current_score != 0 else 0
        print(f"\nImprovement vs current parameters:")
        print(f"  Current score: {current_score:.2f}")
        print(f"  Best score: {best_score:.2f}")
        print(f"  Improvement: {improvement:+.1f}%")

if __name__ == '__main__':
    main()
