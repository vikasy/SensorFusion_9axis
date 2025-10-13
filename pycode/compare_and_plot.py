"""
Compare C and Python quaternion outputs side-by-side and plot differences
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def quaternion_distance(q1, q2):
    """Calculate quaternion distance (1 - |dot product|)"""
    dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]
    return 1.0 - abs(dot)

def main():
    # Load Python reference
    print("Loading Python reference quaternions...")
    py_df = pd.read_csv('python_reference_quaternions_0922.csv')
    print(f"Python samples: {len(py_df)}")

    # Load C reference (if it exists, otherwise we'll generate it)
    try:
        print("Loading C reference quaternions...")
        c_df = pd.read_csv('c_reference_quaternions_0922.csv')
        print(f"C samples: {len(c_df)}")
    except FileNotFoundError:
        print("C reference not found. Please run C program to generate it first.")
        print("We'll proceed with Python data only for now.")
        c_df = None

    if c_df is not None:
        # Merge on sample number
        merged = pd.merge(py_df, c_df, on='sample', suffixes=('_py', '_c'))

        # Calculate quaternion distance for each sample
        distances = []
        for idx, row in merged.iterrows():
            q_py = [row['q0_py'], row['q1_py'], row['q2_py'], row['q3_py']]
            q_c = [row['q0_c'], row['q1_c'], row['q2_c'], row['q3_c']]
            dist = quaternion_distance(q_py, q_c)
            distances.append(dist)

        merged['quat_distance'] = distances

        # Save side-by-side comparison
        print("\nSaving side-by-side comparison...")
        comparison_file = 'quaternion_comparison_detailed.csv'
        merged.to_csv(comparison_file, index=False)
        print(f"Saved to: {comparison_file}")

        # Print first mismatches
        tolerance = 3e-5
        mismatches = merged[merged['quat_distance'] > tolerance]
        if len(mismatches) > 0:
            print(f"\nFirst mismatch at sample {mismatches.iloc[0]['sample']}")
            print(f"Distance: {mismatches.iloc[0]['quat_distance']:.6e}")
        else:
            print(f"\nAll {len(merged)} samples match within tolerance {tolerance:.1e}")

        # Create plots
        print("\nGenerating plots...")
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle('C vs Python Quaternion Comparison', fontsize=16)

        samples = merged['sample'].values

        # Plot each quaternion component
        for i, comp in enumerate(['q0', 'q1', 'q2', 'q3']):
            row = i // 2
            col = i % 2
            ax = axes[row, col]

            ax.plot(samples, merged[f'{comp}_py'], 'b-', label='Python', linewidth=1, alpha=0.7)
            ax.plot(samples, merged[f'{comp}_c'], 'r--', label='C', linewidth=1, alpha=0.7)
            ax.set_xlabel('Sample')
            ax.set_ylabel(comp)
            ax.set_title(f'Quaternion Component {comp}')
            ax.legend()
            ax.grid(True, alpha=0.3)

        # Plot quaternion distance
        ax = axes[2, 0]
        ax.semilogy(samples, merged['quat_distance'], 'g-', linewidth=1)
        ax.axhline(y=tolerance, color='r', linestyle='--', label=f'Tolerance ({tolerance:.1e})')
        ax.set_xlabel('Sample')
        ax.set_ylabel('Quaternion Distance')
        ax.set_title('Quaternion Distance (log scale)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Plot component-wise differences
        ax = axes[2, 1]
        for comp in ['q0', 'q1', 'q2', 'q3']:
            diff = np.abs(merged[f'{comp}_c'] - merged[f'{comp}_py'])
            ax.semilogy(samples, diff, label=f'|Δ{comp}|', linewidth=1, alpha=0.7)
        ax.set_xlabel('Sample')
        ax.set_ylabel('Absolute Difference')
        ax.set_title('Component-wise Differences (log scale)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plot_file = 'quaternion_comparison.png'
        plt.savefig(plot_file, dpi=150)
        print(f"Plot saved to: {plot_file}")

        # Show statistics
        print("\n" + "="*80)
        print("COMPARISON STATISTICS")
        print("="*80)
        print(f"Total samples compared: {len(merged)}")
        print(f"Samples within tolerance: {len(merged[merged['quat_distance'] <= tolerance])}")
        print(f"Samples exceeding tolerance: {len(mismatches)}")
        print(f"\nQuaternion distance:")
        print(f"  Min:    {merged['quat_distance'].min():.6e}")
        print(f"  Max:    {merged['quat_distance'].max():.6e}")
        print(f"  Mean:   {merged['quat_distance'].mean():.6e}")
        print(f"  Median: {merged['quat_distance'].median():.6e}")

        # Print samples with largest distances
        print("\nTop 10 samples with largest distance:")
        print(merged.nlargest(10, 'quat_distance')[['sample', 'quat_distance', 'q0_c', 'q0_py', 'q1_c', 'q1_py']])

    else:
        # Just plot Python data
        print("\nGenerating Python-only plots...")
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Python Quaternion Output', fontsize=16)

        samples = py_df['sample'].values

        for i, comp in enumerate(['q0', 'q1', 'q2', 'q3']):
            row = i // 2
            col = i % 2
            ax = axes[row, col]

            ax.plot(samples, py_df[comp], 'b-', linewidth=1)
            ax.set_xlabel('Sample')
            ax.set_ylabel(comp)
            ax.set_title(f'Quaternion Component {comp}')
            ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig('python_quaternions.png', dpi=150)
        print("Plot saved to: python_quaternions.png")

if __name__ == '__main__':
    main()
