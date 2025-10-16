#!/usr/bin/env python3
"""
Multi-Platform Sensor Configuration for Python

Provides sensor specifications for multiple IMU platforms:
  - INVENSENSE: MPU-9250 (Accelerometer + Gyroscope + Magnetometer)
  - FREESCALE:  FXOS8700CQ (Accel+Mag) + FXAS21000 (Gyro)

Usage:
    from sensor_platform_config import SensorPlatform, INVENSENSE, FREESCALE

    # Use default platform
    platform = SensorPlatform()

    # Or specify platform
    platform = SensorPlatform(INVENSENSE)
    platform = SensorPlatform(FREESCALE)

Author: Vikas Yadav
Date: 2025-10-12
Version: 2.0.0
"""

from enum import Enum
from dataclasses import dataclass
from typing import Dict
import math

# Platform identifiers
class Platform(Enum):
    INVENSENSE = "INVENSENSE"
    FREESCALE = "FREESCALE"

# Convenience constants
INVENSENSE = Platform.INVENSENSE
FREESCALE = Platform.FREESCALE

@dataclass
class AccelerometerSpec:
    """Accelerometer specifications"""
    name: str
    range_g: float
    bits: int
    max_count: int
    sensitivity: float  # LSB/g or counts/g
    noise_density: float  # µg/√Hz
    zero_g_offset: float  # mg

    @property
    def scale_factor(self) -> float:
        """g per count"""
        return self.range_g / self.max_count

    @property
    def counts_per_g(self) -> float:
        """counts per g"""
        return 1.0 / self.scale_factor

    @property
    def mps2_per_count(self) -> float:
        """m/s² per count"""
        return self.scale_factor * 9.80665

@dataclass
class GyroscopeSpec:
    """Gyroscope specifications"""
    name: str
    range_dps: float
    bits: int
    max_count: int
    sensitivity: float  # LSB/dps
    noise_density: float  # dps/√Hz
    zero_rate_offset: float  # dps

    @property
    def scale_factor(self) -> float:
        """dps per count"""
        return self.range_dps / self.max_count

    @property
    def counts_per_dps(self) -> float:
        """counts per dps"""
        return 1.0 / self.scale_factor

    @property
    def rads_per_count(self) -> float:
        """rad/s per count"""
        return self.scale_factor * math.pi / 180.0

@dataclass
class MagnetometerSpec:
    """Magnetometer specifications"""
    name: str
    range_ut: float
    bits: int
    max_count: int
    sensitivity: float  # µT/LSB
    noise: float  # µT RMS

    @property
    def scale_factor(self) -> float:
        """µT per count"""
        return self.range_ut / self.max_count

    @property
    def counts_per_ut(self) -> float:
        """counts per µT"""
        return 1.0 / self.scale_factor

@dataclass
class SamplingConfig:
    """Sampling rate configuration"""
    gyro_odr_hz: int
    accel_odr_hz: int
    mag_odr_hz: int
    sample_rate_hz: int
    fusion_rate_hz: int

    @property
    def sample_period_ms(self) -> float:
        """Sampling period in milliseconds"""
        return 1000.0 / self.sample_rate_hz

    @property
    def sample_period_us(self) -> float:
        """Sampling period in microseconds"""
        return 1000000.0 / self.sample_rate_hz

    @property
    def sample_period_s(self) -> float:
        """Sampling period in seconds"""
        return 1.0 / self.sample_rate_hz


class SensorPlatform:
    """
    Multi-platform sensor configuration

    Provides unified interface to sensor specifications across different platforms.
    """

    # Platform configurations
    _PLATFORMS: Dict[Platform, dict] = {
        Platform.INVENSENSE: {
            'name': 'InvenSense MPU-9250',
            'type': '9-Axis MEMS IMU',
            'manufacturer': 'InvenSense (TDK)',
            'accelerometer': AccelerometerSpec(
                name='MPU-9250',
                range_g=4.0,
                bits=16,
                max_count=32767,
                sensitivity=8192.0,  # LSB/g for ±4g
                noise_density=300.0,  # µg/√Hz
                zero_g_offset=80.0,  # mg
            ),
            'gyroscope': GyroscopeSpec(
                name='MPU-9250',
                range_dps=1000.0,
                bits=16,
                max_count=32767,
                sensitivity=32.8,  # LSB/dps for ±1000dps
                noise_density=0.01,  # dps/√Hz
                zero_rate_offset=20.0,  # dps
            ),
            'magnetometer': MagnetometerSpec(
                name='AK8963',
                range_ut=4800.0,
                bits=16,
                max_count=32767,
                sensitivity=0.15,  # µT/LSB (16-bit mode)
                noise=0.6,  # µT RMS
            ),
            'sampling': SamplingConfig(
                gyro_odr_hz=100,
                accel_odr_hz=100,
                mag_odr_hz=100,
                sample_rate_hz=100,
                fusion_rate_hz=25,
            ),
        },
        Platform.FREESCALE: {
            'name': 'NXP FRDM-STBC-AGM01',
            'type': '9-Axis Sensor Shield',
            'manufacturer': 'NXP Semiconductors (Freescale)',
            'accelerometer': AccelerometerSpec(
                name='FXOS8700CQ',
                range_g=4.0,
                bits=14,
                max_count=8191,
                sensitivity=2048.0,  # counts/g for ±4g
                noise_density=126.0,  # µg/√Hz
                zero_g_offset=40.0,  # mg
            ),
            'gyroscope': GyroscopeSpec(
                name='FXAS21000',
                range_dps=1000.0,
                bits=16,
                max_count=32767,
                sensitivity=32.0,  # LSB/dps for ±1000dps
                noise_density=0.025,  # dps/√Hz
                zero_rate_offset=50.0,  # dps
            ),
            'magnetometer': MagnetometerSpec(
                name='FXOS8700CQ',
                range_ut=1200.0,
                bits=16,
                max_count=32767,
                sensitivity=0.1,  # µT/LSB
                noise=0.4,  # µT RMS
            ),
            'sampling': SamplingConfig(
                gyro_odr_hz=100,
                accel_odr_hz=100,
                mag_odr_hz=100,
                sample_rate_hz=100,
                fusion_rate_hz=25,
            ),
        },
    }

    # Constants
    GRAVITY_MPS2 = 9.80665  # Standard gravity (m/s²)
    DEG_TO_RAD = math.pi / 180.0
    RAD_TO_DEG = 180.0 / math.pi

    def __init__(self, platform: Platform = Platform.INVENSENSE):
        """
        Initialize sensor platform configuration

        Args:
            platform: Platform identifier (INVENSENSE or FREESCALE)
        """
        if platform not in self._PLATFORMS:
            raise ValueError(f"Unknown platform: {platform}. "
                           f"Supported: {list(self._PLATFORMS.keys())}")

        self._platform = platform
        config = self._PLATFORMS[platform]

        # Platform info
        self.platform_name = config['name']
        self.platform_type = config['type']
        self.manufacturer = config['manufacturer']

        # Sensor specs
        self.accelerometer = config['accelerometer']
        self.gyroscope = config['gyroscope']
        self.magnetometer = config['magnetometer']
        self.sampling = config['sampling']

    @property
    def platform(self) -> Platform:
        """Current platform"""
        return self._platform

    # Unified interface properties (platform-independent)

    @property
    def accel_range_g(self) -> float:
        """Accelerometer full-scale range (g)"""
        return self.accelerometer.range_g

    @property
    def accel_scale_factor(self) -> float:
        """Accelerometer scale factor (g/count)"""
        return self.accelerometer.scale_factor

    @property
    def accel_counts_per_g(self) -> float:
        """Accelerometer sensitivity (counts/g)"""
        return self.accelerometer.counts_per_g

    @property
    def accel_mps2_per_count(self) -> float:
        """Accelerometer scale factor (m/s²/count)"""
        return self.accelerometer.mps2_per_count

    @property
    def gyro_range_dps(self) -> float:
        """Gyroscope full-scale range (dps)"""
        return self.gyroscope.range_dps

    @property
    def gyro_scale_factor(self) -> float:
        """Gyroscope scale factor (dps/count)"""
        return self.gyroscope.scale_factor

    @property
    def gyro_counts_per_dps(self) -> float:
        """Gyroscope sensitivity (counts/dps)"""
        return self.gyroscope.counts_per_dps

    @property
    def gyro_rads_per_count(self) -> float:
        """Gyroscope scale factor (rad/s/count)"""
        return self.gyroscope.rads_per_count

    @property
    def mag_range_ut(self) -> float:
        """Magnetometer full-scale range (µT)"""
        return self.magnetometer.range_ut

    @property
    def mag_scale_factor(self) -> float:
        """Magnetometer scale factor (µT/count)"""
        return self.magnetometer.scale_factor

    @property
    def mag_counts_per_ut(self) -> float:
        """Magnetometer sensitivity (counts/µT)"""
        return self.magnetometer.counts_per_ut

    @property
    def sample_rate_hz(self) -> int:
        """Primary sampling rate (Hz)"""
        return self.sampling.sample_rate_hz

    @property
    def fusion_rate_hz(self) -> int:
        """Kalman filter update rate (Hz)"""
        return self.sampling.fusion_rate_hz

    # Conversion methods

    def accel_raw_to_g(self, raw: float) -> float:
        """Convert accelerometer raw count to g"""
        return raw * self.accel_scale_factor

    def accel_raw_to_mps2(self, raw: float) -> float:
        """Convert accelerometer raw count to m/s²"""
        return raw * self.accel_mps2_per_count

    def accel_g_to_raw(self, g: float) -> int:
        """Convert g to accelerometer raw count"""
        return int(g * self.accel_counts_per_g)

    def gyro_raw_to_dps(self, raw: float) -> float:
        """Convert gyroscope raw count to dps"""
        return raw * self.gyro_scale_factor

    def gyro_raw_to_rads(self, raw: float) -> float:
        """Convert gyroscope raw count to rad/s"""
        return raw * self.gyro_rads_per_count

    def gyro_dps_to_raw(self, dps: float) -> int:
        """Convert dps to gyroscope raw count"""
        return int(dps * self.gyro_counts_per_dps)

    def mag_raw_to_ut(self, raw: float) -> float:
        """Convert magnetometer raw count to µT"""
        return raw * self.mag_scale_factor

    def mag_ut_to_raw(self, ut: float) -> int:
        """Convert µT to magnetometer raw count"""
        return int(ut * self.mag_counts_per_ut)

    def __str__(self) -> str:
        """String representation"""
        return (f"SensorPlatform({self._platform.value})\n"
                f"  Name: {self.platform_name}\n"
                f"  Type: {self.platform_type}\n"
                f"  Manufacturer: {self.manufacturer}")

    def print_specs(self):
        """Print detailed specifications"""
        print("=" * 80)
        print(f" SENSOR PLATFORM: {self.platform_name}")
        print("=" * 80)
        print(f"Type:          {self.platform_type}")
        print(f"Manufacturer:  {self.manufacturer}")
        print()

        print("ACCELEROMETER:")
        print(f"  Name:            {self.accelerometer.name}")
        print(f"  Range:           ±{self.accelerometer.range_g} g")
        print(f"  Resolution:      {self.accelerometer.bits}-bit")
        print(f"  Max Count:       {self.accelerometer.max_count}")
        print(f"  Sensitivity:     {self.accelerometer.sensitivity:.1f} counts/g")
        print(f"  Scale Factor:    {self.accelerometer.scale_factor:.10f} g/count")
        print(f"  Noise Density:   {self.accelerometer.noise_density:.1f} µg/√Hz")
        print()

        print("GYROSCOPE:")
        print(f"  Name:            {self.gyroscope.name}")
        print(f"  Range:           ±{self.gyroscope.range_dps} dps")
        print(f"  Resolution:      {self.gyroscope.bits}-bit")
        print(f"  Max Count:       {self.gyroscope.max_count}")
        print(f"  Sensitivity:     {self.gyroscope.sensitivity:.1f} LSB/dps")
        print(f"  Scale Factor:    {self.gyroscope.scale_factor:.10f} dps/count")
        print(f"  Noise Density:   {self.gyroscope.noise_density:.3f} dps/√Hz")
        print()

        print("MAGNETOMETER:")
        print(f"  Name:            {self.magnetometer.name}")
        print(f"  Range:           ±{self.magnetometer.range_ut} µT")
        print(f"  Resolution:      {self.magnetometer.bits}-bit")
        print(f"  Max Count:       {self.magnetometer.max_count}")
        print(f"  Sensitivity:     {self.magnetometer.sensitivity:.2f} µT/LSB")
        print(f"  Scale Factor:    {self.magnetometer.scale_factor:.10f} µT/count")
        print(f"  Noise:           {self.magnetometer.noise:.1f} µT RMS")
        print()

        print("SAMPLING:")
        print(f"  Sample Rate:     {self.sampling.sample_rate_hz} Hz")
        print(f"  Fusion Rate:     {self.sampling.fusion_rate_hz} Hz")
        print(f"  Sample Period:   {self.sampling.sample_period_ms:.2f} ms")
        print("=" * 80)


# Default platform instance
_default_platform = None

def get_default_platform() -> SensorPlatform:
    """Get default platform instance (singleton)"""
    global _default_platform
    if _default_platform is None:
        _default_platform = SensorPlatform(Platform.INVENSENSE)
    return _default_platform

def set_default_platform(platform: Platform):
    """Set default platform"""
    global _default_platform
    _default_platform = SensorPlatform(platform)


# Module-level convenience functions
def get_platform_names():
    """Get list of supported platform names"""
    return [p.value for p in Platform]


if __name__ == '__main__':
    """Test/demonstration code"""
    print("Testing Multi-Platform Sensor Configuration\n")

    # Test INVENSENSE platform
    inv = SensorPlatform(INVENSENSE)
    inv.print_specs()
    print()

    # Test FREESCALE platform
    fsl = SensorPlatform(FREESCALE)
    fsl.print_specs()
    print()

    # Test conversions
    print("=" * 80)
    print(" CONVERSION TESTS")
    print("=" * 80)

    for platform_type in [INVENSENSE, FREESCALE]:
        p = SensorPlatform(platform_type)
        print(f"\n{platform_type.value} Platform:")

        # Test 1g conversion
        raw_1g = p.accel_g_to_raw(1.0)
        converted_g = p.accel_raw_to_g(raw_1g)
        print(f"  1g = {raw_1g} counts")
        print(f"  {raw_1g} counts = {converted_g:.6f} g")
        print(f"  Error: {abs(converted_g - 1.0):.6f} g")

        # Test 100 dps conversion
        raw_100dps = p.gyro_dps_to_raw(100.0)
        converted_dps = p.gyro_raw_to_dps(raw_100dps)
        print(f"  100 dps = {raw_100dps} counts")
        print(f"  {raw_100dps} counts = {converted_dps:.6f} dps")
        print(f"  Error: {abs(converted_dps - 100.0):.6f} dps")

    print("\n" + "=" * 80)
    print("All tests complete!")
