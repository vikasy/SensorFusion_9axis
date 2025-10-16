#!/usr/bin/env python3
"""
Generate realistic motion scenarios for sensor fusion testing.

Scenarios:
1. Climbing stairs - periodic vertical motion with tilt
2. Driving in car - vibration, turns, acceleration/braking
3. Flying robot/drone - 3D motion, hover, maneuvers
4. Walking - gait cycle with periodic motion
5. Handheld device - typical phone/tablet motion
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd
from pathlib import Path
import sys

# Import sensor specs from existing generator
sys.path.insert(0, str(Path(__file__).parent))
from generate_synthetic_datasets import SensorSpec, IMUDataGenerator, save_dataset_csv

class RealisticMotion:
    """Generate realistic motion profiles"""

    @staticmethod
    def climbing_stairs(duration_s=30.0, sample_rate_hz=100.0, step_duration_s=1.5):
        """
        Climbing stairs motion:
        - Periodic vertical acceleration (lifting body)
        - Slight forward tilt during ascent
        - Impact at step landing
        - Realistic stair climbing cadence
        """
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)
        dt = 1.0 / sample_rate_hz

        # Number of steps
        num_steps = int(duration_s / step_duration_s)

        orientations = []
        angular_velocities = np.zeros((num_samples, 3))
        linear_accelerations = np.zeros((num_samples, 3))

        current_rot = R.identity()
        step_phase = 0.0

        for i, t in enumerate(timestamps):
            # Which step are we on?
            step_phase = (t % step_duration_s) / step_duration_s

            # Pitch forward slightly during step (tilt forward ~5-10°)
            if step_phase < 0.6:  # Ascending phase
                target_pitch = 8.0 * (step_phase / 0.6)  # Ramp up to 8°
            else:  # Landing phase
                target_pitch = 8.0 * (1.0 - (step_phase - 0.6) / 0.4)  # Ramp back to 0°

            target_rot = R.from_euler('y', target_pitch, degrees=True)

            if i > 0:
                # Smooth transition to target orientation
                alpha = 0.1  # Smoothing factor
                current_rot = R.from_quat(
                    (1 - alpha) * current_rot.as_quat() + alpha * target_rot.as_quat()
                )
                current_rot = R.from_quat(current_rot.as_quat() / np.linalg.norm(current_rot.as_quat()))

                # Angular velocity from orientation change
                delta_rot = current_rot * orientations[-1].inv()
                rotvec = delta_rot.as_rotvec()
                angular_velocities[i] = rotvec / dt

            orientations.append(current_rot)

            # Linear acceleration profile for stair climbing
            # Vertical: lifting + landing impact
            # Forward: propulsion
            if step_phase < 0.3:  # Push-off phase
                linear_accelerations[i, 2] = 0.8 * np.sin(step_phase / 0.3 * np.pi)  # Upward
                linear_accelerations[i, 0] = 0.3 * np.sin(step_phase / 0.3 * np.pi)  # Forward
            elif step_phase < 0.7:  # Flight phase
                linear_accelerations[i, 2] = -0.2  # Slight descent
            else:  # Landing phase
                impact_phase = (step_phase - 0.7) / 0.3
                linear_accelerations[i, 2] = -1.5 * np.exp(-impact_phase * 10)  # Impact spike

            # Add some lateral sway
            linear_accelerations[i, 1] = 0.1 * np.sin(2 * np.pi * t / step_duration_s)

        return timestamps, orientations, angular_velocities, linear_accelerations

    @staticmethod
    def driving_in_car(duration_s=60.0, sample_rate_hz=100.0):
        """
        Driving in car:
        - Engine vibration (10-30 Hz)
        - Road bumps (random)
        - Turns (banking)
        - Acceleration/braking
        - Mostly level but some pitch changes on hills
        """
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)
        dt = 1.0 / sample_rate_hz

        orientations = []
        angular_velocities = np.zeros((num_samples, 3))
        linear_accelerations = np.zeros((num_samples, 3))

        current_rot = R.identity()
        current_yaw = 0.0
        current_pitch = 0.0
        current_roll = 0.0

        # Define maneuvers
        maneuvers = [
            (5, 15, 'left_turn', 30.0),    # Turn left 30° over 10s
            (20, 25, 'acceleration', 0.5),  # Accelerate at 0.5g
            (30, 35, 'right_turn', 40.0),  # Turn right 40° over 5s
            (40, 45, 'braking', -0.6),      # Brake at 0.6g
            (50, 55, 'hill', 8.0),          # Drive up 8° hill
        ]

        for i, t in enumerate(timestamps):
            # Check which maneuver we're in
            active_maneuver = None
            for t_start, t_end, maneuver_type, param in maneuvers:
                if t_start <= t < t_end:
                    active_maneuver = (maneuver_type, param, (t - t_start) / (t_end - t_start))
                    break

            # Update orientation based on maneuver
            if active_maneuver:
                maneuver_type, param, progress = active_maneuver

                if maneuver_type == 'left_turn':
                    current_yaw += param / sample_rate_hz / (maneuvers[0][1] - maneuvers[0][0])
                    current_roll = -5.0 * np.sin(progress * np.pi)  # Bank into turn
                elif maneuver_type == 'right_turn':
                    current_yaw -= param / sample_rate_hz / (maneuvers[2][1] - maneuvers[2][0])
                    current_roll = 5.0 * np.sin(progress * np.pi)  # Bank into turn
                elif maneuver_type == 'hill':
                    current_pitch = param * np.sin(progress * np.pi / 2)  # Ramp up pitch
                else:
                    current_roll *= 0.95  # Decay roll when not turning

                # Apply maneuver acceleration
                if maneuver_type == 'acceleration':
                    linear_accelerations[i, 0] = param
                elif maneuver_type == 'braking':
                    linear_accelerations[i, 0] = param
            else:
                # Return to level
                current_roll *= 0.95
                current_pitch *= 0.95

            # Set orientation
            current_rot = R.from_euler('zyx', [current_yaw, current_pitch, current_roll], degrees=True)
            orientations.append(current_rot)

            if i > 0:
                delta_rot = current_rot * orientations[-2].inv()
                rotvec = delta_rot.as_rotvec()
                angular_velocities[i] = rotvec / dt

            # Engine vibration (10-20 Hz)
            vib_freq = 15.0
            linear_accelerations[i, 0] += 0.02 * np.sin(2 * np.pi * vib_freq * t)
            linear_accelerations[i, 2] += 0.03 * np.sin(2 * np.pi * vib_freq * t)

            # Road bumps (random)
            if np.random.random() < 0.01:  # 1% chance per sample
                linear_accelerations[i, 2] += np.random.uniform(-0.5, 0.5)

            # Road noise
            linear_accelerations[i] += np.random.normal(0, 0.05, 3)

        return timestamps, orientations, angular_velocities, linear_accelerations

    @staticmethod
    def flying_drone(duration_s=30.0, sample_rate_hz=100.0):
        """
        Flying drone/robot:
        - Takeoff with vertical acceleration
        - Hover with small oscillations
        - Forward flight
        - Banking turns
        - Landing
        """
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)
        dt = 1.0 / sample_rate_hz

        orientations = []
        angular_velocities = np.zeros((num_samples, 3))
        linear_accelerations = np.zeros((num_samples, 3))

        current_rot = R.identity()

        # Flight phases
        phases = [
            (0, 3, 'takeoff'),      # 0-3s: Takeoff
            (3, 8, 'hover'),        # 3-8s: Hover
            (8, 15, 'forward'),     # 8-15s: Forward flight
            (15, 20, 'turn_left'),  # 15-20s: Left turn
            (20, 25, 'forward'),    # 20-25s: Forward flight
            (25, 30, 'landing'),    # 25-30s: Landing
        ]

        for i, t in enumerate(timestamps):
            # Determine current phase
            current_phase = 'hover'
            phase_progress = 0.0
            for t_start, t_end, phase_name in phases:
                if t_start <= t < t_end:
                    current_phase = phase_name
                    phase_progress = (t - t_start) / (t_end - t_start)
                    break

            # Orientation and acceleration based on phase
            if current_phase == 'takeoff':
                # Vertical acceleration
                linear_accelerations[i, 2] = 1.5 * (1.0 - phase_progress)
                # Slight tilt back during takeoff
                pitch = -10.0 * phase_progress
                current_rot = R.from_euler('y', pitch, degrees=True)

            elif current_phase == 'hover':
                # Small oscillations during hover
                hover_freq = 0.5
                pitch = 2.0 * np.sin(2 * np.pi * hover_freq * t)
                roll = 1.5 * np.sin(2 * np.pi * hover_freq * t + np.pi/4)
                current_rot = R.from_euler('yx', [pitch, roll], degrees=True)
                # Very small vertical oscillation
                linear_accelerations[i, 2] = 0.1 * np.sin(2 * np.pi * 1.0 * t)

            elif current_phase == 'forward':
                # Pitch forward for forward flight
                pitch = -15.0
                current_rot = R.from_euler('y', pitch, degrees=True)
                # Forward acceleration
                linear_accelerations[i, 0] = 0.8

            elif current_phase == 'turn_left':
                # Bank left, increase yaw rate
                roll = -30.0
                yaw_rate = 20.0  # deg/s
                pitch = -10.0
                yaw = yaw_rate * (t - 15.0)
                current_rot = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
                # Centripetal acceleration
                linear_accelerations[i, 1] = -0.3

            elif current_phase == 'landing':
                # Gradual descent
                pitch = -5.0 * (1.0 - phase_progress)
                current_rot = R.from_euler('y', pitch, degrees=True)
                linear_accelerations[i, 2] = -1.0 * phase_progress

            orientations.append(current_rot)

            if i > 0:
                delta_rot = current_rot * orientations[-1].inv()
                rotvec = delta_rot.as_rotvec()
                angular_velocities[i] = rotvec / dt

            # Propeller vibration
            vib_freq = 50.0
            linear_accelerations[i, 2] += 0.05 * np.sin(2 * np.pi * vib_freq * t)

        return timestamps, orientations, angular_velocities, linear_accelerations

    @staticmethod
    def walking(duration_s=20.0, sample_rate_hz=100.0, step_duration_s=1.0):
        """
        Walking gait cycle:
        - Heel strike impact
        - Weight transfer
        - Push-off
        - Periodic roll/pitch oscillations
        """
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)
        dt = 1.0 / sample_rate_hz

        orientations = []
        angular_velocities = np.zeros((num_samples, 3))
        linear_accelerations = np.zeros((num_samples, 3))

        current_rot = R.identity()

        for i, t in enumerate(timestamps):
            # Gait phase (0-1 for one complete step)
            gait_phase = (t % step_duration_s) / step_duration_s

            # Pitch oscillation (forward tilt during step)
            pitch = 3.0 * np.sin(2 * np.pi * gait_phase)

            # Roll oscillation (weight shift side to side)
            # Left step: roll right, right step: roll left
            roll = 4.0 * np.sin(4 * np.pi * gait_phase)

            current_rot = R.from_euler('yx', [pitch, roll], degrees=True)
            orientations.append(current_rot)

            if i > 0:
                delta_rot = current_rot * orientations[-1].inv()
                rotvec = delta_rot.as_rotvec()
                angular_velocities[i] = rotvec / dt

            # Vertical acceleration profile
            if gait_phase < 0.1:  # Heel strike
                linear_accelerations[i, 2] = -2.0 * np.exp(-gait_phase * 50)
            elif gait_phase < 0.6:  # Stance phase
                linear_accelerations[i, 2] = 0.3 * np.sin((gait_phase - 0.1) / 0.5 * np.pi)
            else:  # Swing phase
                linear_accelerations[i, 2] = -0.2

            # Forward acceleration (propulsion)
            linear_accelerations[i, 0] = 0.4 * np.sin(2 * np.pi * gait_phase)

            # Lateral acceleration (sway)
            linear_accelerations[i, 1] = 0.2 * np.sin(4 * np.pi * gait_phase)

        return timestamps, orientations, angular_velocities, linear_accelerations

    @staticmethod
    def handheld_device(duration_s=20.0, sample_rate_hz=100.0):
        """
        Handheld device (phone/tablet) typical motion:
        - Start horizontal (reading)
        - Tilt up (looking at camera)
        - Random small movements
        - Return to horizontal
        """
        num_samples = int(duration_s * sample_rate_hz)
        timestamps = np.linspace(0, duration_s, num_samples)
        dt = 1.0 / sample_rate_hz

        orientations = []
        angular_velocities = np.zeros((num_samples, 3))
        linear_accelerations = np.zeros((num_samples, 3))

        current_rot = R.identity()

        # Define poses
        poses = [
            (0, 5, 'reading', 0, 0, 0),           # Horizontal
            (5, 8, 'transition', 0, 45, 0),       # Tilt up
            (8, 12, 'selfie', 0, 45, 0),          # Hold for selfie
            (12, 15, 'transition', 0, 0, 0),      # Back to horizontal
            (15, 20, 'reading', 0, 0, 0),         # Reading again
        ]

        for i, t in enumerate(timestamps):
            # Find current pose
            target_roll, target_pitch, target_yaw = 0, 0, 0
            for t_start, t_end, pose_name, roll, pitch, yaw in poses:
                if t_start <= t < t_end:
                    if pose_name == 'transition':
                        # Smooth transition
                        progress = (t - t_start) / (t_end - t_start)
                        prev_pose = poses[[j for j, p in enumerate(poses) if p[1] == t_start][0]]
                        target_roll = prev_pose[3] + progress * (roll - prev_pose[3])
                        target_pitch = prev_pose[4] + progress * (pitch - prev_pose[4])
                        target_yaw = prev_pose[5] + progress * (yaw - prev_pose[5])
                    else:
                        target_roll, target_pitch, target_yaw = roll, pitch, yaw
                    break

            # Add small random movements (hand tremor)
            target_roll += np.random.normal(0, 0.5)
            target_pitch += np.random.normal(0, 0.5)
            target_yaw += np.random.normal(0, 0.5)

            target_rot = R.from_euler('zyx', [target_yaw, target_pitch, target_roll], degrees=True)

            # Smooth transition
            alpha = 0.05
            if i > 0:
                current_rot = R.from_quat(
                    (1 - alpha) * current_rot.as_quat() + alpha * target_rot.as_quat()
                )
                current_rot = R.from_quat(current_rot.as_quat() / np.linalg.norm(current_rot.as_quat()))

                delta_rot = current_rot * orientations[-1].inv()
                rotvec = delta_rot.as_rotvec()
                angular_velocities[i] = rotvec / dt
            else:
                current_rot = target_rot

            orientations.append(current_rot)

            # Small random linear accelerations (hand movement)
            linear_accelerations[i] = np.random.normal(0, 0.1, 3)

        return timestamps, orientations, angular_velocities, linear_accelerations


def main():
    output_dir = Path(__file__).parent.parent / "data" / "datasets" / "realistic"
    output_dir.mkdir(parents=True, exist_ok=True)

    generator = IMUDataGenerator(seed=42)
    spec = SensorSpec()

    print("="*80)
    print("GENERATING REALISTIC MOTION SCENARIOS")
    print("="*80)
    print()

    scenarios = [
        ("Climbing Stairs", RealisticMotion.climbing_stairs, (30.0, spec.SAMPLE_RATE_HZ)),
        ("Driving in Car", RealisticMotion.driving_in_car, (60.0, spec.SAMPLE_RATE_HZ)),
        ("Flying Drone", RealisticMotion.flying_drone, (30.0, spec.SAMPLE_RATE_HZ)),
        ("Walking", RealisticMotion.walking, (20.0, spec.SAMPLE_RATE_HZ)),
        ("Handheld Device", RealisticMotion.handheld_device, (20.0, spec.SAMPLE_RATE_HZ)),
    ]

    for name, motion_func, args in scenarios:
        print(f"Generating: {name}")

        t, r, w, a = motion_func(*args)
        dataset = generator.generate_dataset(t, r, w, a)

        filename = name.lower().replace(" ", "_") + ".csv"
        save_dataset_csv(dataset, output_dir / filename)
        print()

    print("="*80)
    print(f"✓ Generated {len(scenarios)} realistic motion scenarios")
    print(f"Location: {output_dir}")
    print("="*80)

if __name__ == "__main__":
    main()
