#!/usr/bin/env python3
"""
Generate comprehensive plots comparing Python implementation vs ground truth.
Requires matplotlib.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def main():
    # Check if results file exists
    results_file = "full_dataset_comparison_results.csv"
    if not os.path.exists(results_file):
        print(f"Error: {results_file} not found!")
        print("Please run test_full_dataset_comparison.py first.")
        return

    print("="*80)
    print("GENERATING COMPARISON PLOTS")
    print("="*80)

    # Load results
    df = pd.read_csv(results_file)
    print(f"\nLoaded {len(df)} fusion cycles from {results_file}")

    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('9-Axis Sensor Fusion: Python vs Ground Truth Comparison', fontsize=16, fontweight='bold')

    # ===== Plot 1: Quaternion Error Over Time =====
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(df['fusion'], df['quat_error'], 'b-', linewidth=1, alpha=0.7)
    ax1.axhline(y=0.5, color='g', linestyle='--', linewidth=1, label='Good (0.5°)')
    ax1.axhline(y=1.0, color='orange', linestyle='--', linewidth=1, label='Acceptable (1.0°)')
    ax1.set_xlabel('Fusion Cycle', fontsize=10)
    ax1.set_ylabel('Quaternion Error (deg)', fontsize=10)
    ax1.set_title('Quaternion Angular Distance', fontsize=11, fontweight='bold')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ===== Plot 2: Euler Angle Errors =====
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(df['fusion'], df['yaw_error'], 'r-', linewidth=1, alpha=0.7, label='Yaw')
    ax2.plot(df['fusion'], df['pitch_error'], 'g-', linewidth=1, alpha=0.7, label='Pitch')
    ax2.plot(df['fusion'], df['roll_error'], 'b-', linewidth=1, alpha=0.7, label='Roll')
    ax2.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax2.set_xlabel('Fusion Cycle', fontsize=10)
    ax2.set_ylabel('Angle Error (deg)', fontsize=10)
    ax2.set_title('Euler Angle Errors', fontsize=11, fontweight='bold')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # ===== Plot 3: Gyro Bias Evolution =====
    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(df['fusion'], df['bias_x'], 'r-', linewidth=1, alpha=0.7, label='Bias X')
    ax3.plot(df['fusion'], df['bias_y'], 'g-', linewidth=1, alpha=0.7, label='Bias Y')
    ax3.plot(df['fusion'], df['bias_z'], 'b-', linewidth=1, alpha=0.7, label='Bias Z')
    ax3.plot(df['fusion'], df['bias_mag'], 'k-', linewidth=1.5, alpha=0.9, label='Magnitude')
    ax3.set_xlabel('Fusion Cycle', fontsize=10)
    ax3.set_ylabel('Gyro Bias (dps)', fontsize=10)
    ax3.set_title('Gyro Bias Estimates', fontsize=11, fontweight='bold')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ===== Plot 4: Quaternion Components - Ground Truth =====
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(df['fusion'], df['gt_quat_w'], 'k-', linewidth=1, alpha=0.7, label='w (GT)')
    ax4.plot(df['fusion'], df['gt_quat_x'], 'r-', linewidth=1, alpha=0.7, label='x (GT)')
    ax4.plot(df['fusion'], df['gt_quat_y'], 'g-', linewidth=1, alpha=0.7, label='y (GT)')
    ax4.plot(df['fusion'], df['gt_quat_z'], 'b-', linewidth=1, alpha=0.7, label='z (GT)')
    ax4.set_xlabel('Fusion Cycle', fontsize=10)
    ax4.set_ylabel('Quaternion Component', fontsize=10)
    ax4.set_title('Quaternion Components (Ground Truth)', fontsize=11, fontweight='bold')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    # ===== Plot 5: Quaternion Components - Python =====
    ax5 = plt.subplot(3, 3, 5)
    ax5.plot(df['fusion'], df['py_quat_w'], 'k-', linewidth=1, alpha=0.7, label='w (Python)')
    ax5.plot(df['fusion'], df['py_quat_x'], 'r-', linewidth=1, alpha=0.7, label='x (Python)')
    ax5.plot(df['fusion'], df['py_quat_y'], 'g-', linewidth=1, alpha=0.7, label='y (Python)')
    ax5.plot(df['fusion'], df['py_quat_z'], 'b-', linewidth=1, alpha=0.7, label='z (Python)')
    ax5.set_xlabel('Fusion Cycle', fontsize=10)
    ax5.set_ylabel('Quaternion Component', fontsize=10)
    ax5.set_title('Quaternion Components (Python)', fontsize=11, fontweight='bold')
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    # ===== Plot 6: Quaternion Component Differences =====
    ax6 = plt.subplot(3, 3, 6)
    ax6.plot(df['fusion'], df['py_quat_w'] - df['gt_quat_w'], 'k-', linewidth=1, alpha=0.7, label='Δw')
    ax6.plot(df['fusion'], df['py_quat_x'] - df['gt_quat_x'], 'r-', linewidth=1, alpha=0.7, label='Δx')
    ax6.plot(df['fusion'], df['py_quat_y'] - df['gt_quat_y'], 'g-', linewidth=1, alpha=0.7, label='Δy')
    ax6.plot(df['fusion'], df['py_quat_z'] - df['gt_quat_z'], 'b-', linewidth=1, alpha=0.7, label='Δz')
    ax6.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
    ax6.set_xlabel('Fusion Cycle', fontsize=10)
    ax6.set_ylabel('Component Difference', fontsize=10)
    ax6.set_title('Quaternion Component Errors', fontsize=11, fontweight='bold')
    ax6.legend(fontsize=8)
    ax6.grid(True, alpha=0.3)

    # ===== Plot 7: Yaw Comparison =====
    ax7 = plt.subplot(3, 3, 7)
    ax7.plot(df['fusion'], df['gt_yaw'], 'k--', linewidth=1.5, alpha=0.7, label='Ground Truth')
    ax7.plot(df['fusion'], df['py_yaw'], 'r-', linewidth=1, alpha=0.7, label='Python')
    ax7.set_xlabel('Fusion Cycle', fontsize=10)
    ax7.set_ylabel('Yaw Angle (deg)', fontsize=10)
    ax7.set_title('Yaw Angle Comparison', fontsize=11, fontweight='bold')
    ax7.legend(fontsize=8)
    ax7.grid(True, alpha=0.3)

    # ===== Plot 8: Pitch Comparison =====
    ax8 = plt.subplot(3, 3, 8)
    ax8.plot(df['fusion'], df['gt_pitch'], 'k--', linewidth=1.5, alpha=0.7, label='Ground Truth')
    ax8.plot(df['fusion'], df['py_pitch'], 'g-', linewidth=1, alpha=0.7, label='Python')
    ax8.set_xlabel('Fusion Cycle', fontsize=10)
    ax8.set_ylabel('Pitch Angle (deg)', fontsize=10)
    ax8.set_title('Pitch Angle Comparison', fontsize=11, fontweight='bold')
    ax8.legend(fontsize=8)
    ax8.grid(True, alpha=0.3)

    # ===== Plot 9: Roll Comparison =====
    ax9 = plt.subplot(3, 3, 9)
    ax9.plot(df['fusion'], df['gt_roll'], 'k--', linewidth=1.5, alpha=0.7, label='Ground Truth')
    ax9.plot(df['fusion'], df['py_roll'], 'b-', linewidth=1, alpha=0.7, label='Python')
    ax9.set_xlabel('Fusion Cycle', fontsize=10)
    ax9.set_ylabel('Roll Angle (deg)', fontsize=10)
    ax9.set_title('Roll Angle Comparison', fontsize=11, fontweight='bold')
    ax9.legend(fontsize=8)
    ax9.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # Save figure
    output_file = "comparison_plots.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\n✓ Saved comprehensive plot to: {output_file}")

    # Create second figure: Error histograms
    fig2 = plt.figure(figsize=(12, 8))
    fig2.suptitle('Error Distribution Analysis', fontsize=16, fontweight='bold')

    # Quaternion error histogram
    ax1 = plt.subplot(2, 2, 1)
    ax1.hist(df['quat_error'], bins=50, color='blue', alpha=0.7, edgecolor='black')
    ax1.axvline(df['quat_error'].mean(), color='red', linestyle='--', linewidth=2, label=f'Mean: {df["quat_error"].mean():.3f}°')
    ax1.axvline(df['quat_error'].median(), color='green', linestyle='--', linewidth=2, label=f'Median: {df["quat_error"].median():.3f}°')
    ax1.set_xlabel('Quaternion Error (deg)', fontsize=10)
    ax1.set_ylabel('Frequency', fontsize=10)
    ax1.set_title('Quaternion Error Distribution', fontsize=11, fontweight='bold')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Yaw error histogram
    ax2 = plt.subplot(2, 2, 2)
    ax2.hist(df['yaw_error'], bins=50, color='red', alpha=0.7, edgecolor='black')
    ax2.axvline(df['yaw_error'].mean(), color='darkred', linestyle='--', linewidth=2, label=f'Mean: {df["yaw_error"].mean():.3f}°')
    ax2.axvline(0, color='black', linestyle='-', linewidth=1)
    ax2.set_xlabel('Yaw Error (deg)', fontsize=10)
    ax2.set_ylabel('Frequency', fontsize=10)
    ax2.set_title('Yaw Error Distribution', fontsize=11, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    # Pitch error histogram
    ax3 = plt.subplot(2, 2, 3)
    ax3.hist(df['pitch_error'], bins=50, color='green', alpha=0.7, edgecolor='black')
    ax3.axvline(df['pitch_error'].mean(), color='darkgreen', linestyle='--', linewidth=2, label=f'Mean: {df["pitch_error"].mean():.3f}°')
    ax3.axvline(0, color='black', linestyle='-', linewidth=1)
    ax3.set_xlabel('Pitch Error (deg)', fontsize=10)
    ax3.set_ylabel('Frequency', fontsize=10)
    ax3.set_title('Pitch Error Distribution', fontsize=11, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)

    # Roll error histogram
    ax4 = plt.subplot(2, 2, 4)
    ax4.hist(df['roll_error'], bins=50, color='purple', alpha=0.7, edgecolor='black')
    ax4.axvline(df['roll_error'].mean(), color='indigo', linestyle='--', linewidth=2, label=f'Mean: {df["roll_error"].mean():.3f}°')
    ax4.axvline(0, color='black', linestyle='-', linewidth=1)
    ax4.set_xlabel('Roll Error (deg)', fontsize=10)
    ax4.set_ylabel('Frequency', fontsize=10)
    ax4.set_title('Roll Error Distribution', fontsize=11, fontweight='bold')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # Save histogram figure
    output_file2 = "error_histograms.png"
    plt.savefig(output_file2, dpi=150, bbox_inches='tight')
    print(f"✓ Saved error histograms to: {output_file2}")

    # Don't show plots interactively (save only)
    # plt.show()

    print("\n" + "="*80)
    print("PLOTTING COMPLETE")
    print("="*80)

if __name__ == '__main__':
    main()
