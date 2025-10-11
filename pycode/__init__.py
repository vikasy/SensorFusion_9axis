"""
SensorFusion Python Package

Converted MATLAB sensor fusion algorithms for Python/NumPy
"""

# Import main modules
try:
    from . import quatmath
    from . import sensor_fusion
except ImportError:
    # Allow individual module imports
    pass

__version__ = "1.0.0"
__author__ = "Converted from MATLAB"
