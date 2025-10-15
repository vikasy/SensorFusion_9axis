#!/usr/bin/env python3
"""
Automated Regression Test Suite for Part 1 Datasets

This script tests that any algorithm changes do not regress the performance
on the 8 "working" datasets (Part 1). Must pass before testing Part 2 improvements.

Usage:
    python3 regression_test.py                    # Run all tests
    python3 regression_test.py --update-baseline  # Update baseline from current results
    python3 regression_test.py --verbose          # Show detailed output
"""

import numpy as np
import pandas as pd
import sys
import os
import argparse
import json
from pathlib import Path

# Part 1 Datasets (must not regress)
PART1_DATASETS = {
    'static': [
        'static_60s',
        'static_10s',
        'static_high_bias_10s',
        'static_high_noise_10s'
    ],
    'rotation': [
        'rotation_y_15dps_10s',
        'rotation_z_30dps_10s',
        'rotation_x_20dps_10s'
    ],
    'vibration': [
        'vibration_5hz_10s'
    ]
}

# Acceptance criteria for Part 1 (absolute limits)
ACCEPTANCE_CRITERIA = {
    'static_60s': {
        'quat_error_mean': 6.0,  # Current: 0.577°
        'quat_error_max': 2.0,   # Current: 0.883°
        'bias_final': 2.0,        # Current: 0.461 dps
    },
    'static_10s': {
        'quat_error_mean': 6.0,  # Current: 2.716°
        'quat_error_max': 8.0,   # Current: 5.032°
        'bias_final': 2.0,        # Current: 0.830 dps
    },
    'static_high_bias_10s': {
        'quat_error_mean': 6.0,  # Current: 2.633°
        'quat_error_max': 7.0,   # Current: 4.603°
        'bias_final': 2.0,        # Current: 1.002 dps
    },
    'static_high_noise_10s': {
        'quat_error_mean': 8.0,  # Current: 5.241° (marginal)
        'quat_error_max': 12.0,  # Current: 9.269°
        'bias_final': 3.0,        # Current: 1.519 dps
    },
    'rotation_y_15dps_10s': {
        'quat_error_mean': 3.5,  # Current: 0.955°
        'quat_error_max': 3.0,   # Current: 1.661°
        'bias_final': 1.5,        # Current: 0.801 dps
    },
    'rotation_z_30dps_10s': {
        'quat_error_mean': 3.5,  # Current: 1.539°
        'quat_error_max': 5.0,   # Current: 2.874°
        'bias_final': 1.5,        # Current: 0.639 dps
    },
    'rotation_x_20dps_10s': {
        'quat_error_mean': 3.5,  # Current: 2.729°
        'quat_error_max': 7.0,   # Current: 5.154°
        'bias_final': 1.5,        # Current: 1.040 dps
    },
    'vibration_5hz_10s': {
        'quat_error_mean': 1.5,  # Current: 0.787°
        'quat_error_max': 2.5,   # Current: 1.400°
        'bias_final': 1.0,        # Current: 0.605 dps
    }
}

# Regression tolerance (% increase from baseline)
REGRESSION_TOLERANCE = {
    'quat_error_mean': 10.0,  # Allow 10% increase
    'quat_error_max': 10.0,   # Allow 10% increase
    'bias_final': 10.0,        # Allow 10% increase
}

class RegressionTest:
    def __init__(self, baseline_file='regression_baseline.json', verbose=False):
        self.baseline_file = baseline_file
        self.verbose = verbose
        self.baseline = self.load_baseline()
        self.results = {}
        self.failures = []

    def load_baseline(self):
        """Load baseline metrics from file"""
        if not os.path.exists(self.baseline_file):
            print(f"⚠ WARNING: No baseline file found at {self.baseline_file}")
            print("Run with --update-baseline to create baseline from current results")
            return None

        with open(self.baseline_file, 'r') as f:
            baseline = json.load(f)

        print(f"✓ Loaded baseline from: {self.baseline_file}")
        print(f"  Baseline date: {baseline.get('date', 'unknown')}")
        print(f"  Datasets: {len(baseline.get('datasets', {}))}")
        return baseline

    def load_current_results(self, results_dir='synthetic_test_results'):
        """Load current test results"""
        summary_file = os.path.join(results_dir, 'summary_all_datasets.csv')

        if not os.path.exists(summary_file):
            print(f"✗ ERROR: Results file not found: {summary_file}")
            print("Run test_all_synthetic_datasets.py first to generate results")
            return False

        df = pd.read_csv(summary_file)

        # Extract metrics for Part 1 datasets
        for category, datasets in PART1_DATASETS.items():
            for dataset_name in datasets:
                row = df[df['dataset'] == dataset_name]
                if len(row) == 0:
                    print(f"⚠ WARNING: Dataset {dataset_name} not found in results")
                    continue

                row = row.iloc[0]
                self.results[dataset_name] = {
                    'quat_error_mean': row['quat_error_mean'],
                    'quat_error_std': row['quat_error_std'],
                    'quat_error_max': row['quat_error_max'],
                    'bias_final': row['bias_final'],
                    'bias_max': row['bias_max'],
                    'excellent_pct': row['excellent_pct'],
                    'good_pct': row['good_pct'],
                }

        return True

    def check_acceptance_criteria(self, dataset_name, metrics):
        """Check if metrics pass absolute acceptance criteria"""
        if dataset_name not in ACCEPTANCE_CRITERIA:
            return True, []

        criteria = ACCEPTANCE_CRITERIA[dataset_name]
        violations = []

        for metric, threshold in criteria.items():
            if metric in metrics:
                value = metrics[metric]
                if value > threshold:
                    violations.append({
                        'metric': metric,
                        'value': value,
                        'threshold': threshold,
                        'excess': value - threshold
                    })

        return len(violations) == 0, violations

    def check_regression(self, dataset_name, current_metrics):
        """Check if current results regress from baseline"""
        if self.baseline is None:
            return True, []  # No baseline to compare against

        if dataset_name not in self.baseline.get('datasets', {}):
            return True, []  # Dataset not in baseline

        baseline_metrics = self.baseline['datasets'][dataset_name]
        regressions = []

        for metric, tolerance_pct in REGRESSION_TOLERANCE.items():
            if metric in baseline_metrics and metric in current_metrics:
                baseline_val = baseline_metrics[metric]
                current_val = current_metrics[metric]

                # Calculate percent change
                if baseline_val > 0:
                    pct_change = ((current_val - baseline_val) / baseline_val) * 100

                    if pct_change > tolerance_pct:
                        regressions.append({
                            'metric': metric,
                            'baseline': baseline_val,
                            'current': current_val,
                            'pct_change': pct_change,
                            'tolerance': tolerance_pct
                        })

        return len(regressions) == 0, regressions

    def run_tests(self):
        """Run all regression tests"""
        print("\n" + "="*80)
        print("REGRESSION TEST SUITE - PART 1 DATASETS")
        print("="*80)

        if not self.load_current_results():
            return False

        all_passed = True
        summary = {
            'total': 0,
            'passed': 0,
            'failed': 0,
            'acceptance_failures': 0,
            'regression_failures': 0
        }

        for category, datasets in PART1_DATASETS.items():
            print(f"\n{'─'*80}")
            print(f"Category: {category.upper()}")
            print(f"{'─'*80}")

            for dataset_name in datasets:
                summary['total'] += 1

                if dataset_name not in self.results:
                    print(f"\n✗ {dataset_name}: NOT TESTED")
                    all_passed = False
                    summary['failed'] += 1
                    continue

                metrics = self.results[dataset_name]

                # Check acceptance criteria
                passes_acceptance, violations = self.check_acceptance_criteria(dataset_name, metrics)

                # Check regression vs baseline
                passes_regression, regressions = self.check_regression(dataset_name, metrics)

                # Overall result
                passed = passes_acceptance and passes_regression

                if passed:
                    status = "✓ PASS"
                    summary['passed'] += 1
                else:
                    status = "✗ FAIL"
                    summary['failed'] += 1
                    all_passed = False

                    if not passes_acceptance:
                        summary['acceptance_failures'] += 1
                    if not passes_regression:
                        summary['regression_failures'] += 1

                # Print result
                print(f"\n{status}: {dataset_name}")
                print(f"  Mean Error: {metrics['quat_error_mean']:.3f}° (max: {metrics['quat_error_max']:.3f}°)")
                print(f"  Final Bias: {metrics['bias_final']:.3f} dps")

                # Print violations
                if not passes_acceptance:
                    print(f"  ⚠ ACCEPTANCE CRITERIA VIOLATED:")
                    for v in violations:
                        print(f"    - {v['metric']}: {v['value']:.3f} > {v['threshold']:.3f} (excess: {v['excess']:.3f})")

                if not passes_regression:
                    print(f"  ⚠ REGRESSION DETECTED:")
                    for r in regressions:
                        print(f"    - {r['metric']}: {r['current']:.3f} vs baseline {r['baseline']:.3f} ({r['pct_change']:+.1f}% > {r['tolerance']:.1f}%)")

                # Verbose details
                if self.verbose and self.baseline and dataset_name in self.baseline.get('datasets', {}):
                    baseline_metrics = self.baseline['datasets'][dataset_name]
                    print(f"  Detailed comparison:")
                    for metric in ['quat_error_mean', 'quat_error_max', 'bias_final']:
                        if metric in metrics and metric in baseline_metrics:
                            curr = metrics[metric]
                            base = baseline_metrics[metric]
                            diff = curr - base
                            pct = (diff / base * 100) if base > 0 else 0
                            print(f"    {metric}: {curr:.3f} (baseline: {base:.3f}, diff: {diff:+.3f}, {pct:+.1f}%)")

        # Print summary
        print("\n" + "="*80)
        print("TEST SUMMARY")
        print("="*80)
        print(f"Total datasets tested: {summary['total']}")
        print(f"Passed: {summary['passed']} ({100*summary['passed']/summary['total']:.1f}%)")
        print(f"Failed: {summary['failed']} ({100*summary['failed']/summary['total']:.1f}%)")

        if summary['acceptance_failures'] > 0:
            print(f"  - Acceptance criteria violations: {summary['acceptance_failures']}")
        if summary['regression_failures'] > 0:
            print(f"  - Regressions from baseline: {summary['regression_failures']}")

        if all_passed:
            print("\n✓✓✓ ALL REGRESSION TESTS PASSED ✓✓✓")
            print("Safe to proceed with Part 2 improvements")
        else:
            print("\n✗✗✗ REGRESSION TESTS FAILED ✗✗✗")
            print("DO NOT proceed with changes - fix regressions first")

        return all_passed

    def update_baseline(self):
        """Update baseline from current results"""
        if not self.load_current_results():
            print("✗ Cannot update baseline - no current results available")
            return False

        from datetime import datetime

        baseline = {
            'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'description': 'Part 1 baseline after QBias=10.0 fix',
            'datasets': self.results
        }

        with open(self.baseline_file, 'w') as f:
            json.dump(baseline, f, indent=2)

        print(f"\n✓ Baseline updated: {self.baseline_file}")
        print(f"  Date: {baseline['date']}")
        print(f"  Datasets: {len(self.results)}")

        return True

def main():
    parser = argparse.ArgumentParser(description='Regression test suite for Part 1 datasets')
    parser.add_argument('--update-baseline', action='store_true',
                       help='Update baseline from current results')
    parser.add_argument('--baseline-file', default='regression_baseline.json',
                       help='Path to baseline file (default: regression_baseline.json)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Show detailed output')

    args = parser.parse_args()

    tester = RegressionTest(baseline_file=args.baseline_file, verbose=args.verbose)

    if args.update_baseline:
        success = tester.update_baseline()
        sys.exit(0 if success else 1)
    else:
        success = tester.run_tests()
        sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
