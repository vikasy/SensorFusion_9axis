#!/usr/bin/env python3
"""
Plot input sensor data and fusion output for walking dataset

Author: Vikas Yadav
Date: 2025-10-17
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sensor_fusion_9axis import SensorFusion9Axis

def plot_walking_dataset():
    """Generate comprehensive plots for walking dataset"""

    # Load walking dataset
    dataset_path = '../../test/data/datasets/realistic/walking.csv'
    df = pd.read_csv(dataset_path)

    print(f"Loaded {len(df)} samples from walking.csv")

    # Sensor specifications (MPU9250 + AK8963)
    accel_scale = 0.0001220703125  # g/count (±4g range, 16-bit)
    gyro_scale = 0.030518  # dps/count (±1000 dps range, 16-bit)
    mag_scale = 0.15  # µT/count (±4800 µT range, 16-bit)

    # Initialize sensor fusion
    fusion = SensorFusion9Axis(accel_scale, gyro_scale, mag_scale)

    # Storage for fusion outputs
    outputs = {
        'quat_w': [], 'quat_x': [], 'quat_y': [], 'quat_z': [],
        'roll': [], 'pitch': [], 'yaw': [],
        'bias_x': [], 'bias_y': [], 'bias_z': [],
        'error': []
    }

    # Process all samples
    print("Processing samples...")
    for idx, row in df.iterrows():
        # Prepare sensor data
        accel_counts = np.array([row['accel_x_counts'], row['accel_y_counts'], row['accel_z_counts']])
        gyro_counts = np.array([row['gyro_x_counts'], row['gyro_y_counts'], row['gyro_z_counts']])
        mag_counts = np.array([row['mag_x_counts'], row['mag_y_counts'], row['mag_z_counts']])

        timestamp_ns = idx * 10_000_000  # 100 Hz = 10ms period

        # Feed to fusion (4 samples per fusion cycle)
        for _ in range(4):
            fusion.preprocess_sensor_data(0, accel_counts, timestamp_ns)
            fusion.preprocess_sensor_data(1, gyro_counts, timestamp_ns)
            fusion.preprocess_sensor_data(2, mag_counts, timestamp_ns)

        # Run fusion
        output = fusion.run()

        # Store results
        outputs['quat_w'].append(output.quat.q0)
        outputs['quat_x'].append(output.quat.q1)
        outputs['quat_y'].append(output.quat.q2)
        outputs['quat_z'].append(output.quat.q3)
        outputs['roll'].append(output.orientation[2])  # [yaw, pitch, roll]
        outputs['pitch'].append(output.orientation[1])
        outputs['yaw'].append(output.orientation[0])
        outputs['bias_x'].append(fusion.bias_post_s[0])
        outputs['bias_y'].append(fusion.bias_post_s[1])
        outputs['bias_z'].append(fusion.bias_post_s[2])

        # Compute quaternion error
        gt_quat = np.array([row['gt_quat_w'], row['gt_quat_x'], row['gt_quat_y'], row['gt_quat_z']])
        est_quat = np.array([output.quat.q0, output.quat.q1, output.quat.q2, output.quat.q3])

        # Quaternion distance
        dot_product = np.abs(np.dot(gt_quat, est_quat))
        dot_product = np.clip(dot_product, 0.0, 1.0)
        angle_error = 2.0 * np.arccos(dot_product) * (180.0 / np.pi)
        outputs['error'].append(angle_error)

    print(f"Processed {len(outputs['quat_w'])} fusion outputs")

    # Convert to numpy arrays for plotting
    time_sec = df['timestamp_ns'].values / 1e9  # Convert nanoseconds to seconds

    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))

    # 1. Accelerometer data
    ax1 = plt.subplot(4, 2, 1)
    accel_g = df[['accel_x_counts', 'accel_y_counts', 'accel_z_counts']].values * accel_scale
    plt.plot(time_sec, accel_g[:, 0], 'r-', label='X', linewidth=0.5)
    plt.plot(time_sec, accel_g[:, 1], 'g-', label='Y', linewidth=0.5)
    plt.plot(time_sec, accel_g[:, 2], 'b-', label='Z', linewidth=0.5)
    plt.axhline(y=1.0, color='k', linestyle='--', alpha=0.3, linewidth=0.5)
    plt.axhline(y=-1.0, color='k', linestyle='--', alpha=0.3, linewidth=0.5)
    plt.grid(True, alpha=0.3)
    plt.ylabel('Acceleration (g)')
    plt.title('Accelerometer Input')
    plt.legend(loc='upper right')
    plt.xlim([time_sec[0], time_sec[-1]])

    # 2. Gyroscope data
    ax2 = plt.subplot(4, 2, 2)
    gyro_dps = df[['gyro_x_counts', 'gyro_y_counts', 'gyro_z_counts']].values * gyro_scale
    plt.plot(time_sec, gyro_dps[:, 0], 'r-', label='X', linewidth=0.5)
    plt.plot(time_sec, gyro_dps[:, 1], 'g-', label='Y', linewidth=0.5)
    plt.plot(time_sec, gyro_dps[:, 2], 'b-', label='Z', linewidth=0.5)
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=0.5)
    plt.grid(True, alpha=0.3)
    plt.ylabel('Angular Rate (dps)')
    plt.title('Gyroscope Input')
    plt.legend(loc='upper right')
    plt.xlim([time_sec[0], time_sec[-1]])

    # 3. Magnetometer data
    ax3 = plt.subplot(4, 2, 3)
    mag_ut = df[['mag_x_counts', 'mag_y_counts', 'mag_z_counts']].values * mag_scale
    plt.plot(time_sec, mag_ut[:, 0], 'r-', label='X', linewidth=0.5)
    plt.plot(time_sec, mag_ut[:, 1], 'g-', label='Y', linewidth=0.5)
    plt.plot(time_sec, mag_ut[:, 2], 'b-', label='Z', linewidth=0.5)
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=0.5)
    plt.grid(True, alpha=0.3)
    plt.ylabel('Magnetic Field (µT)')
    plt.title('Magnetometer Input')
    plt.legend(loc='upper right')
    plt.xlim([time_sec[0], time_sec[-1]])

    # 4. Quaternion output
    ax4 = plt.subplot(4, 2, 4)
    plt.plot(time_sec, outputs['quat_w'], 'k-', label='W', linewidth=1)
    plt.plot(time_sec, outputs['quat_x'], 'r-', label='X', linewidth=0.5)
    plt.plot(time_sec, outputs['quat_y'], 'g-', label='Y', linewidth=0.5)
    plt.plot(time_sec, outputs['quat_z'], 'b-', label='Z', linewidth=0.5)
    # Ground truth (dashed)
    plt.plot(time_sec, df['gt_quat_w'], 'k--', alpha=0.5, linewidth=0.5, label='GT W')
    plt.plot(time_sec, df['gt_quat_x'], 'r--', alpha=0.5, linewidth=0.5)
    plt.plot(time_sec, df['gt_quat_y'], 'g--', alpha=0.5, linewidth=0.5)
    plt.plot(time_sec, df['gt_quat_z'], 'b--', alpha=0.5, linewidth=0.5)
    plt.grid(True, alpha=0.3)
    plt.ylabel('Quaternion')
    plt.title('Quaternion Output (solid=fusion, dashed=GT)')
    plt.legend(loc='upper right', ncol=2)
    plt.xlim([time_sec[0], time_sec[-1]])

    # 5. Euler angles
    ax5 = plt.subplot(4, 2, 5)
    plt.plot(time_sec, outputs['roll'], 'r-', label='Roll', linewidth=0.5)
    plt.plot(time_sec, outputs['pitch'], 'g-', label='Pitch', linewidth=0.5)
    plt.plot(time_sec, outputs['yaw'], 'b-', label='Yaw', linewidth=0.5)
    # Ground truth (dashed)
    plt.plot(time_sec, df['gt_roll_deg'], 'r--', alpha=0.5, linewidth=0.5)
    plt.plot(time_sec, df['gt_pitch_deg'], 'g--', alpha=0.5, linewidth=0.5)
    plt.plot(time_sec, df['gt_yaw_deg'], 'b--', alpha=0.5, linewidth=0.5)
    plt.grid(True, alpha=0.3)
    plt.ylabel('Angle (deg)')
    plt.title('Euler Angles (solid=fusion, dashed=GT)')
    plt.legend(loc='upper right')
    plt.xlim([time_sec[0], time_sec[-1]])

    # 6. Gyro bias estimate
    ax6 = plt.subplot(4, 2, 6)
    plt.plot(time_sec, outputs['bias_x'], 'r-', label='X', linewidth=0.5)
    plt.plot(time_sec, outputs['bias_y'], 'g-', label='Y', linewidth=0.5)
    plt.plot(time_sec, outputs['bias_z'], 'b-', label='Z', linewidth=0.5)
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3, linewidth=0.5)
    plt.grid(True, alpha=0.3)
    plt.ylabel('Bias (dps)')
    plt.title('Gyro Bias Estimate')
    plt.legend(loc='upper right')
    plt.xlim([time_sec[0], time_sec[-1]])

    # 7. Quaternion error
    ax7 = plt.subplot(4, 2, 7)
    plt.plot(time_sec, outputs['error'], 'r-', linewidth=0.5)
    plt.axhline(y=10, color='orange', linestyle='--', alpha=0.5, linewidth=1, label='10° threshold')
    plt.axhline(y=5, color='g', linestyle='--', alpha=0.5, linewidth=1, label='5° target')
    plt.grid(True, alpha=0.3)
    plt.ylabel('Error (deg)')
    plt.xlabel('Time (s)')
    plt.title('Quaternion Error vs Ground Truth')
    plt.legend(loc='upper right')
    plt.xlim([time_sec[0], time_sec[-1]])
    plt.ylim([0, max(outputs['error']) * 1.1])

    # 8. Acceleration magnitude
    ax8 = plt.subplot(4, 2, 8)
    accel_mag = np.linalg.norm(accel_g, axis=1)
    plt.plot(time_sec, accel_mag, 'b-', linewidth=0.5)
    plt.axhline(y=1.0, color='g', linestyle='--', alpha=0.5, linewidth=1, label='1g (static)')
    plt.axhline(y=0.0, color='k', linestyle='-', alpha=0.3, linewidth=0.5)
    plt.grid(True, alpha=0.3)
    plt.ylabel('Magnitude (g)')
    plt.xlabel('Time (s)')
    plt.title('Acceleration Magnitude (shows linear acceleration)')
    plt.legend(loc='upper right')
    plt.xlim([time_sec[0], time_sec[-1]])

    plt.tight_layout()

    # Print statistics
    print("\n" + "="*80)
    print("WALKING DATASET STATISTICS")
    print("="*80)
    print(f"\nDuration: {time_sec[-1]:.2f} seconds")
    print(f"Samples: {len(df)}")
    print(f"\nInitial acceleration (sample 0): [{accel_g[0, 0]:.3f}, {accel_g[0, 1]:.3f}, {accel_g[0, 2]:.3f}] g")
    print(f"Initial accel magnitude: {accel_mag[0]:.3f} g  <- {accel_mag[0]:.1f}g LINEAR ACCEL!")
    print(f"\nMean quaternion error: {np.mean(outputs['error']):.2f}°")
    print(f"Max quaternion error: {np.max(outputs['error']):.2f}°")
    print(f"Min quaternion error: {np.min(outputs['error']):.2f}°")
    print(f"\nFinal gyro bias: [{outputs['bias_x'][-1]:.3f}, {outputs['bias_y'][-1]:.3f}, {outputs['bias_z'][-1]:.3f}] dps")
    print(f"Bias magnitude: {np.linalg.norm([outputs['bias_x'][-1], outputs['bias_y'][-1], outputs['bias_z'][-1]]):.3f} dps")

    # Save figure
    output_path = 'walking_dataset_plots.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_path}")
    print("="*80)

if __name__ == '__main__':
    plot_walking_dataset()
