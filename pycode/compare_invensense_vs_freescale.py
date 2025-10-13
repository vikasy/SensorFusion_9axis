#!/usr/bin/env python3
"""
Compare INVENSENSE vs FREESCALE sensor fusion outputs

Compares quaternion outputs from MATLAB, Python, and C implementations
using both INVENSENSE (MPU9250) and FREESCALE (FXOS8700CQ+FXAS21000) sensor specs

Author: Vikas Yadav
Date: 2025-10-12
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def load_quaternions(csv_file):
    """Load quaternion data from CSV file"""
    df = pd.read_csv(csv_file)
    return df[['q0', 'q1', 'q2', 'q3']].values

def quaternion_distance(q1, q2):
    """
    Compute distance between two quaternions
    Handles quaternion double-cover (q and -q represent same rotation)
    """
    # Direct distance
    dist_pos = np.linalg.norm(q1 - q2)
    # Distance to negative (equivalent rotation)
    dist_neg = np.linalg.norm(q1 + q2)
    # Return minimum (accounts for double-cover)
    return min(dist_pos, dist_neg)

def compare_quaternion_arrays(quat1, quat2, label1, label2):
    """Compare two arrays of quaternions"""
    n_samples = min(len(quat1), len(quat2))

    distances = np.array([quaternion_distance(quat1[i], quat2[i])
                         for i in range(n_samples)])

    print(f"\n{label1} vs {label2}:")
    print(f"  Samples compared: {n_samples}")
    print(f"  Mean distance: {np.mean(distances):.6e}")
    print(f"  Max distance: {np.max(distances):.6e}")
    print(f"  Min distance: {np.min(distances):.6e}")
    print(f"  Std distance: {np.std(distances):.6e}")

    return distances

def main():
    # File paths
    base_dir = Path(__file__).parent.parent

    files = {
        'matlab_inv': base_dir / 'matlab' / 'matlab_reference_quaternions_0922.csv',
        'matlab_fsl': base_dir / 'matlab' / 'matlab_freescale_quaternions_0922.csv',
        'python_inv': base_dir / 'pycode' / 'python_reference_quaternions_0922.csv',
        'python_fsl': base_dir / 'pycode' / 'python_freescale_quaternions_0922.csv',
        'c_inv': base_dir / 'pycode' / 'c_reference_quaternions_0922.csv',
        'c_fsl': base_dir / 'c_freescale_quaternions_0922.csv',
    }

    # Check files exist
    for name, path in files.items():
        if not path.exists():
            print(f"ERROR: File not found: {path}")
            return 1

    print("=" * 80)
    print("INVENSENSE vs FREESCALE Sensor Fusion Comparison")
    print("=" * 80)
    print("\nLoading quaternion data...")

    # Load all data
    data = {}
    for name, path in files.items():
        data[name] = load_quaternions(path)
        print(f"  {name}: {len(data[name])} samples")

    # Compare INVENSENSE vs FREESCALE for each implementation
    print("\n" + "=" * 80)
    print("WITHIN-IMPLEMENTATION COMPARISONS (INVENSENSE vs FREESCALE)")
    print("=" * 80)

    matlab_dist = compare_quaternion_arrays(
        data['matlab_inv'], data['matlab_fsl'],
        "MATLAB INVENSENSE", "MATLAB FREESCALE"
    )

    python_dist = compare_quaternion_arrays(
        data['python_inv'], data['python_fsl'],
        "Python INVENSENSE", "Python FREESCALE"
    )

    c_dist = compare_quaternion_arrays(
        data['c_inv'], data['c_fsl'],
        "C INVENSENSE", "C FREESCALE"
    )

    # Compare across implementations for INVENSENSE
    print("\n" + "=" * 80)
    print("CROSS-IMPLEMENTATION COMPARISONS (INVENSENSE specs)")
    print("=" * 80)

    matlab_python_inv_dist = compare_quaternion_arrays(
        data['matlab_inv'], data['python_inv'],
        "MATLAB INVENSENSE", "Python INVENSENSE"
    )

    matlab_c_inv_dist = compare_quaternion_arrays(
        data['matlab_inv'], data['c_inv'],
        "MATLAB INVENSENSE", "C INVENSENSE"
    )

    python_c_inv_dist = compare_quaternion_arrays(
        data['python_inv'], data['c_inv'],
        "Python INVENSENSE", "C INVENSENSE"
    )

    # Compare across implementations for FREESCALE
    print("\n" + "=" * 80)
    print("CROSS-IMPLEMENTATION COMPARISONS (FREESCALE specs)")
    print("=" * 80)

    matlab_python_fsl_dist = compare_quaternion_arrays(
        data['matlab_fsl'], data['python_fsl'],
        "MATLAB FREESCALE", "Python FREESCALE"
    )

    matlab_c_fsl_dist = compare_quaternion_arrays(
        data['matlab_fsl'], data['c_fsl'],
        "MATLAB FREESCALE", "C FREESCALE"
    )

    python_c_fsl_dist = compare_quaternion_arrays(
        data['python_fsl'], data['c_fsl'],
        "Python FREESCALE", "C FREESCALE"
    )

    # Create comparison plots
    print("\n" + "=" * 80)
    print("Generating comparison plots...")
    print("=" * 80)

    fig = plt.figure(figsize=(16, 12))

    # Plot 1: Quaternion components over time (MATLAB)
    ax1 = plt.subplot(3, 3, 1)
    samples = np.arange(len(data['matlab_inv']))
    ax1.plot(samples, data['matlab_inv'][:, 0], 'b-', label='INVENSENSE q0', linewidth=1)
    ax1.plot(samples, data['matlab_fsl'][:, 0], 'r--', label='FREESCALE q0', linewidth=1)
    ax1.set_xlabel('Sample')
    ax1.set_ylabel('q0')
    ax1.set_title('MATLAB: q0 Component')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(samples, data['matlab_inv'][:, 1], 'b-', label='INVENSENSE q1', linewidth=1)
    ax2.plot(samples, data['matlab_fsl'][:, 1], 'r--', label='FREESCALE q1', linewidth=1)
    ax2.set_xlabel('Sample')
    ax2.set_ylabel('q1')
    ax2.set_title('MATLAB: q1 Component')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(samples, matlab_dist, 'g-', linewidth=1)
    ax3.set_xlabel('Sample')
    ax3.set_ylabel('Quaternion Distance')
    ax3.set_title('MATLAB: INV vs FSL Distance')
    ax3.grid(True, alpha=0.3)

    # Plot 2: Quaternion components over time (Python)
    ax4 = plt.subplot(3, 3, 4)
    samples = np.arange(len(data['python_inv']))
    ax4.plot(samples, data['python_inv'][:, 0], 'b-', label='INVENSENSE q0', linewidth=1)
    ax4.plot(samples, data['python_fsl'][:, 0], 'r--', label='FREESCALE q0', linewidth=1)
    ax4.set_xlabel('Sample')
    ax4.set_ylabel('q0')
    ax4.set_title('Python: q0 Component')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    ax5 = plt.subplot(3, 3, 5)
    ax5.plot(samples, data['python_inv'][:, 1], 'b-', label='INVENSENSE q1', linewidth=1)
    ax5.plot(samples, data['python_fsl'][:, 1], 'r--', label='FREESCALE q1', linewidth=1)
    ax5.set_xlabel('Sample')
    ax5.set_ylabel('q1')
    ax5.set_title('Python: q1 Component')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    ax6 = plt.subplot(3, 3, 6)
    ax6.plot(samples, python_dist, 'g-', linewidth=1)
    ax6.set_xlabel('Sample')
    ax6.set_ylabel('Quaternion Distance')
    ax6.set_title('Python: INV vs FSL Distance')
    ax6.grid(True, alpha=0.3)

    # Plot 3: Quaternion components over time (C)
    ax7 = plt.subplot(3, 3, 7)
    samples = np.arange(len(data['c_inv']))
    ax7.plot(samples, data['c_inv'][:, 0], 'b-', label='INVENSENSE q0', linewidth=1)
    ax7.plot(samples, data['c_fsl'][:, 0], 'r--', label='FREESCALE q0', linewidth=1)
    ax7.set_xlabel('Sample')
    ax7.set_ylabel('q0')
    ax7.set_title('C: q0 Component')
    ax7.legend()
    ax7.grid(True, alpha=0.3)

    ax8 = plt.subplot(3, 3, 8)
    ax8.plot(samples, data['c_inv'][:, 1], 'b-', label='INVENSENSE q1', linewidth=1)
    ax8.plot(samples, data['c_fsl'][:, 1], 'r--', label='FREESCALE q1', linewidth=1)
    ax8.set_xlabel('Sample')
    ax8.set_ylabel('q1')
    ax8.set_title('C: q1 Component')
    ax8.legend()
    ax8.grid(True, alpha=0.3)

    ax9 = plt.subplot(3, 3, 9)
    ax9.plot(samples, c_dist, 'g-', linewidth=1)
    ax9.set_xlabel('Sample')
    ax9.set_ylabel('Quaternion Distance')
    ax9.set_title('C: INV vs FSL Distance')
    ax9.grid(True, alpha=0.3)

    plt.tight_layout()
    output_file = base_dir / 'pycode' / 'invensense_vs_freescale_comparison.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_file}")

    # Create second figure: Distance histograms
    fig2 = plt.figure(figsize=(15, 5))

    ax1 = plt.subplot(1, 3, 1)
    ax1.hist(matlab_dist, bins=50, alpha=0.7, color='blue', edgecolor='black')
    ax1.set_xlabel('Quaternion Distance')
    ax1.set_ylabel('Frequency')
    ax1.set_title('MATLAB: INVENSENSE vs FREESCALE\nDistance Distribution')
    ax1.grid(True, alpha=0.3)
    ax1.axvline(np.mean(matlab_dist), color='r', linestyle='--', label=f'Mean: {np.mean(matlab_dist):.6e}')
    ax1.legend()

    ax2 = plt.subplot(1, 3, 2)
    ax2.hist(python_dist, bins=50, alpha=0.7, color='green', edgecolor='black')
    ax2.set_xlabel('Quaternion Distance')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Python: INVENSENSE vs FREESCALE\nDistance Distribution')
    ax2.grid(True, alpha=0.3)
    ax2.axvline(np.mean(python_dist), color='r', linestyle='--', label=f'Mean: {np.mean(python_dist):.6e}')
    ax2.legend()

    ax3 = plt.subplot(1, 3, 3)
    ax3.hist(c_dist, bins=50, alpha=0.7, color='orange', edgecolor='black')
    ax3.set_xlabel('Quaternion Distance')
    ax3.set_ylabel('Frequency')
    ax3.set_title('C: INVENSENSE vs FREESCALE\nDistance Distribution')
    ax3.grid(True, alpha=0.3)
    ax3.axvline(np.mean(c_dist), color='r', linestyle='--', label=f'Mean: {np.mean(c_dist):.6e}')
    ax3.legend()

    plt.tight_layout()
    output_file2 = base_dir / 'pycode' / 'invensense_vs_freescale_histograms.png'
    plt.savefig(output_file2, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {output_file2}")

    print("\n" + "=" * 80)
    print("Analysis complete!")
    print("=" * 80)

    return 0

if __name__ == '__main__':
    import sys
    sys.exit(main())
