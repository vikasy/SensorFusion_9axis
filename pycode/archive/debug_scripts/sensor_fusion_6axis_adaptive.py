#!/usr/bin/env python3
"""
Adaptive 6-Axis Sensor Fusion with Linear Acceleration Detection
Extends base implementation with:
1. Robust initialization - waits for stable gravity before initializing orientation
2. Adaptive measurement noise based on detected linear acceleration
"""

import numpy as np
from sensor_fusion_6axis import SensorFusion6Axis, GTOMSEC2

class SensorFusion6AxisAdaptive(SensorFusion6Axis):
    """
    Enhanced 6-axis fusion with linear acceleration detection.
    Increases measurement noise R when linear acceleration detected.
    Waits for stable gravity before initializing orientation.
    """

    def __init__(self, acc_scale: float, gyro_scale: float):
        super().__init__(acc_scale, gyro_scale)

        # Store base measurement noise
        self.meas_noise_var_acc_base = self.meas_noise_var_acc

        # Linear acceleration detection parameters
        self.lin_acc_detect_threshold = 0.15  # 0.15g deviation from 1g
        self.lin_acc_detect_alpha = 0.9  # LPF smoothing
        self.lin_acc_magnitude = 1.0  # Current filtered accel magnitude

        # Adaptive R scaling
        self.R_scale_min = 1.0   # No scaling when static
        self.R_scale_max = 100.0  # 100x scaling when strong linear accel

        # Robust initialization
        self.init_accel_stable_count = 0  # Count of stable samples
        self.init_accel_stable_required = 10  # Require 10 stable samples (0.4s at 25Hz)
        self.init_accel_stable_threshold = 0.1  # 0.1g deviation threshold for "stable"

        print("6-axis SF algo initialized (ADAPTIVE + ROBUST INIT)")

    def detect_linear_acceleration(self, accel_counts: np.ndarray) -> float:
        """
        Detect linear acceleration by checking deviation from 1g.

        Args:
            accel_counts: Raw accelerometer counts [x, y, z]

        Returns:
            Scale factor for measurement noise (1.0 = no scaling, >1.0 = reduce trust)
        """
        # Convert to g
        accel_g = accel_counts * self.acc_data.scale_factor
        accel_mag = np.linalg.norm(accel_g)

        # Low-pass filter the magnitude
        self.lin_acc_magnitude = (self.lin_acc_detect_alpha * self.lin_acc_magnitude +
                                  (1 - self.lin_acc_detect_alpha) * accel_mag)

        # Calculate deviation from 1g (expected for pure gravity)
        deviation = abs(self.lin_acc_magnitude - 1.0)

        # Map deviation to R scale factor
        if deviation < self.lin_acc_detect_threshold:
            # Small deviation - trust accelerometer
            R_scale = self.R_scale_min
        else:
            # Large deviation - reduce trust linearly
            # deviation 0.15g -> scale 1.0
            # deviation 0.50g -> scale 50.0
            # deviation 1.00g -> scale 100.0
            excess_deviation = deviation - self.lin_acc_detect_threshold
            R_scale = self.R_scale_min + (self.R_scale_max - self.R_scale_min) * min(excess_deviation / 0.85, 1.0)

        return R_scale

    def _check_orientation_init_ready(self) -> bool:
        """
        Check if accelerometer is stable enough for orientation initialization.
        Requires several consecutive samples near 1g.

        Returns:
            True if ready to initialize, False otherwise
        """
        # Check current accelerometer magnitude
        accel_avg_g = self.acc_data.count_avg * self.acc_data.scale_factor
        accel_mag = np.linalg.norm(accel_avg_g)

        deviation = abs(accel_mag - 1.0)

        if deviation < self.init_accel_stable_threshold:
            self.init_accel_stable_count += 1
        else:
            # Reset counter if unstable
            self.init_accel_stable_count = 0

        return self.init_accel_stable_count >= self.init_accel_stable_required

    def run(self):
        """
        Override run() to add robust initialization check.
        Waits for stable gravity before initializing orientation.
        """
        # If not yet initialized, check if we're ready
        if not self.orient_init:
            if not self._check_orientation_init_ready():
                # Not ready yet - return identity orientation
                from sensor_fusion_6axis import AlgoOutput, Quaternion, AlgoType
                output = AlgoOutput()
                output.algo_type = AlgoType.SF_6AG
                output.quat = Quaternion()
                output.orientation = np.zeros(3)
                output.gravity = np.array([0.0, 0.0, -GTOMSEC2])
                return output

        # Ready to initialize or already initialized - proceed normally
        return super().run()

    def measurement_update(self, debug=False):
        """
        Enhanced measurement update with adaptive R based on linear acceleration.
        """
        # Detect linear acceleration using most recent accelerometer sample
        accel_counts = self.acc_data.count_buff[3]  # Last sample in buffer
        R_scale = self.detect_linear_acceleration(accel_counts)

        # Adapt measurement noise
        self.meas_noise_var_acc = self.meas_noise_var_acc_base * R_scale

        if debug:
            accel_g = accel_counts * self.acc_data.scale_factor
            accel_mag = np.linalg.norm(accel_g)
            print(f"Accel mag: {accel_mag:.3f}g, R_scale: {R_scale:.1f}x")

        # Call parent measurement update with adapted R
        super().measurement_update(debug)

        # Restore base R for next iteration
        self.meas_noise_var_acc = self.meas_noise_var_acc_base
