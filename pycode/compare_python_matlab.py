#!/usr/bin/env python3
"""
Compare Python sensor fusion output with MATLAB reference output.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def quaternion_distance(q1, q2):
    """Calculate quaternion distance (1 - |dot product|)"""
    dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]
    return 1.0 - abs(dot)

def main():
    # Load both CSV files
    print("Loading Python reference...")
    python_df = pd.read_csv('python_reference_quaternions_0922.csv')

    print("Loading MATLAB reference...")
    matlab_df = pd.read_csv('matlab_reference_quaternions_0922.csv')

    print(f"Python samples: {len(python_df)}")
    print(f"MATLAB samples: {len(matlab_df)}")

    # MATLAB skips 2 samples (reset + init), so Python index i+2 corresponds to MATLAB index i
    python_offset = 2

    # Compare quaternions
    tolerance = 3e-5
    matches = 0
    max_distance = 0.0
    first_mismatch = None

    distances = []

    for i in range(min(len(python_df) - python_offset, len(matlab_df))):
        q_py = [python_df.loc[i + python_offset, 'q0'], python_df.loc[i + python_offset, 'q1'],
                python_df.loc[i + python_offset, 'q2'], python_df.loc[i + python_offset, 'q3']]
        q_mat = [matlab_df.loc[i, 'q0'], matlab_df.loc[i, 'q1'],
                 matlab_df.loc[i, 'q2'], matlab_df.loc[i, 'q3']]

        dist = quaternion_distance(q_py, q_mat)
        distances.append(dist)

        if dist > max_distance:
            max_distance = dist

        if dist <= tolerance:
            matches += 1
        elif first_mismatch is None:
            first_mismatch = i
            print(f"\nFirst mismatch at sample {i}:")
            print(f"  Python sample {i + python_offset}:  q=[{q_py[0]:.15f}, {q_py[1]:.15f}, {q_py[2]:.15f}, {q_py[3]:.15f}]")
            print(f"  MATLAB sample {i}:  q=[{q_mat[0]:.15f}, {q_mat[1]:.15f}, {q_mat[2]:.15f}, {q_mat[3]:.15f}]")
            print(f"  Distance: {dist:.10e}")

    total_samples = min(len(python_df), len(matlab_df))
    print(f"\n=== Comparison Results ===")
    print(f"Total samples compared: {total_samples}")
    print(f"Samples within tolerance ({tolerance}): {matches}")
    print(f"Samples exceeding tolerance: {total_samples - matches}")
    print(f"Maximum distance: {max_distance:.10e}")

    if first_mismatch is not None:
        print(f"First mismatch at sample: {first_mismatch}")
    else:
        print("ALL SAMPLES MATCH!")

    # Create visualization
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('Python vs MATLAB Quaternion Comparison', fontsize=16)

    samples = range(len(distances))

    # Plot each quaternion component
    for idx, comp in enumerate(['q0', 'q1', 'q2', 'q3']):
        row = idx // 2
        col = idx % 2
        ax = axes[row, col]

        ax.plot(samples, python_df[comp][python_offset:python_offset+len(distances)], 'b-', label='Python', linewidth=1)
        ax.plot(samples, matlab_df[comp][:len(distances)], 'r--', label='MATLAB', linewidth=1)
        ax.set_xlabel('Sample')
        ax.set_ylabel(f'Quaternion Component {comp}')
        ax.set_title(f'Quaternion Component {comp}')
        ax.legend()
        ax.grid(True, alpha=0.3)

    # Plot quaternion distance (log scale)
    ax = axes[2, 0]
    ax.semilogy(samples, distances, 'g-', linewidth=1)
    ax.axhline(y=tolerance, color='r', linestyle='--', linewidth=2, label=f'Tolerance ({tolerance:.0e})')
    ax.set_xlabel('Sample')
    ax.set_ylabel('Quaternion Distance')
    ax.set_title('Quaternion Distance (log scale)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot component-wise differences (log scale)
    ax = axes[2, 1]
    for comp in ['q0', 'q1', 'q2', 'q3']:
        diff = np.abs(python_df[comp][python_offset:python_offset+len(distances)].values - matlab_df[comp][:len(distances)].values)
        diff = np.maximum(diff, 1e-16)  # Avoid log(0)
        ax.semilogy(samples, diff, label=f'|Δ{comp}|', linewidth=1)
    ax.set_xlabel('Sample')
    ax.set_ylabel('Absolute Difference')
    ax.set_title('Component-wise Differences (log scale)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('python_matlab_comparison.png', dpi=150, bbox_inches='tight')
    print(f"\nPlot saved as: python_matlab_comparison.png")

    # Save detailed comparison to CSV (only aligned samples)
    comparison_df = python_df[python_offset:python_offset+len(distances)].copy()
    comparison_df.columns = ['py_' + col for col in comparison_df.columns]
    for col in matlab_df.columns:
        if col in matlab_df.columns:
            comparison_df['mat_' + col] = matlab_df[col][:len(distances)].values
    comparison_df['distance'] = distances
    comparison_df.to_csv('python_matlab_comparison.csv', index=False)
    print(f"Detailed comparison saved as: python_matlab_comparison.csv")

if __name__ == '__main__':
    main()
