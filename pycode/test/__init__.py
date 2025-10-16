"""
Sensor Fusion Test Module

This module contains the core sensor fusion implementations:
- sensor_fusion_6axis.py: 6-axis (accel + gyro) fusion
- sensor_fusion_9axis.py: 9-axis (accel + gyro + mag) fusion
- sensor_platform_config.py: Platform-specific sensor configurations
- test_integration.py: Integration tests

Author: Vikas Yadav
Date: 2025-10-15
"""

__all__ = [
    'sensor_fusion_6axis',
    'sensor_fusion_9axis',
    'sensor_platform_config',
    'test_integration'
]
