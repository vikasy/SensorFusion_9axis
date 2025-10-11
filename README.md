# SensorFusion 9-Axis 🧭

A comprehensive sensor fusion library supporting both **C/C++** and **Python** implementations, converted from MATLAB algorithms for 6-axis and 9-axis IMU sensor fusion.

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![Python](https://img.shields.io/badge/python-3.7+-blue)]()
[![CMake](https://img.shields.io/badge/cmake-3.10+-blue)]()
[![License](https://img.shields.io/badge/license-MIT-green)]()

## 🚀 Features

- **Dual Implementation:** Both C/C++ and Python versions with identical functionality  
- **Multi-Axis Support:** 6-axis (Accelerometer + Gyroscope) and 9-axis (+ Magnetometer) sensor fusion
- **Quaternion Mathematics:** Complete quaternion math library with 18+ functions
- **Cross-Platform:** Windows, macOS, Linux support via CMake
- **Comprehensive Testing:** Full test suite with SciPy validation for Python functions
- **MATLAB Compatibility:** Function aliases for seamless MATLAB-to-Python migration

## 📁 Project Structure

```
SensorFusion_9axis/
├── README.md                          # This file
├── build/                             # CMake build system  
│   ├── build.sh                       # Cross-platform build script
│   └── CMakeLists.txt                 # CMake configuration
├── code/                              # C/C++ source code
│   ├── algo/                          # Core algorithms
│   ├── app/                           # Application code  
│   ├── math/                          # Mathematical functions
│   └── utils/                         # Utility functions
├── pycode/                            # Python implementation ✨
│   ├── SF_*.py                        # Main sensor fusion algorithms
│   ├── QuatMath/                      # Quaternion mathematics library
│   │   ├── *.py                       # 18+ quaternion functions
│   │   └── test/                      # Comprehensive test suite
│   └── __pycache__/                   # Python bytecode cache
├── matlab/                            # Original MATLAB algorithms
├── test/                              # C/C++ test data and cases
└── doc/                               # Documentation
```

## 🛠️ Quick Start

### Prerequisites

**For C/C++ Build:**
- CMake 3.10+
- GCC/Clang/MSVC compiler
- Make (Unix) or Visual Studio (Windows)

**For Python:**
- Python 3.7+  
- NumPy, SciPy, matplotlib

### Build C/C++ Version

```bash
# Clone the repository
git clone <your-repo-url>
cd SensorFusion_9axis

# Build 9-axis version
cd build
./build.sh 9axis

# Build 6-axis version  
./build.sh 6axis

# Run the executable
./bin/SensorFusion
```

### Python Installation

```bash
# Install Python dependencies
pip install numpy scipy matplotlib

# Test the Python implementation
python pycode/QuatMath/test/run_all_tests.py

# Use in your projects
python -c "
from pycode.QuatMath.QuatProduct import quat_product
from pycode.QuatMath.Deg2Rad import deg_to_rad
print('90° =', deg_to_rad(90), 'radians')
"
```

## 🧪 Testing

### Python Test Suite
```bash
cd pycode/QuatMath/test

# Run all tests with performance benchmarks
python run_all_tests.py

# Run individual test components
python test_quatmath_functions.py      # Function validation
python benchmark_performance.py        # Performance benchmarks  
python generate_test_data.py          # Test data generation
```

**Test Results:** ✅ All 7 test suites passing with 0 failures

### C/C++ Testing
```bash
cd build
./build.sh test    # Run C/C++ test suite
```

## 📊 Performance

| Function Category | Python Performance | SciPy Comparison |
|-------------------|-------------------|------------------|
| Angle Conversions | 18-84 µs/call | 22x slower for arrays |
| Quaternion Ops | 5-71 µs/call | Competitive |
| Matrix Conversions | 11-17 µs/call | Faster for some ops |

*Benchmarked on macOS with Python 3.7*

## 🔧 API Reference

### Core Sensor Fusion Functions

**Python:**
```python
from pycode.SF_Init_State import SF_Init_State
from pycode.SF_Main import SF_Main

# Initialize sensor fusion
state = SF_Init_State('9X_AGM')  # or '6X_AG'

# Update with sensor data
accel = [0, 0, 1]    # g
gyro = [0, 0, 0]     # rad/s  
mag = [1, 0, 0]      # µT
updated_state = SF_Main(accel, gyro, mag)

# Get orientation (Euler angles)
roll = updated_state['PhiPost']    # degrees
pitch = updated_state['ThetaPost'] # degrees  
yaw = updated_state['PsiPost']     # degrees
```

### QuatMath Library

**Essential Functions:**
```python
from pycode.QuatMath import *

# Angle conversions
rad = deg_to_rad(180)              # π radians
deg = rad_to_deg(3.14159)          # 180 degrees

# Quaternion operations  
q_norm = quat_normalize([1,2,3,4]) # Unit quaternion
q_conj = quat_conjugate(q_norm)    # Conjugate
q_mult = quat_product(q1, q2)      # Multiplication

# Conversions
euler = quat_to_euler_angles(quat) # [roll, pitch, yaw]
dcm = quat_to_dcm(quat)           # 3x3 rotation matrix
```

**MATLAB Compatibility:**
```python
# Use original MATLAB function names
euler_angles = Quat2EulerAng(quaternion)  # Same as quat_to_euler_angles()
rad_value = Deg2Rad(degree_value)         # Same as deg_to_rad()
```

## 🧮 Supported Fusion Types

| Type | Sensors | Use Case | Drift Correction |
|------|---------|----------|------------------|
| **3X_A** | Accelerometer only | Static orientation | No |
| **3X_G** | Gyroscope only | Short-term dynamics | No |
| **3X_M** | Magnetometer only | Compass heading | No |
| **6X_AG** | Accel + Gyro | Dynamic with gravity ref | Partial |
| **6X_AM** | Accel + Mag | Static with mag ref | Yes |
| **9X_AGM** | Accel + Gyro + Mag | Full orientation | Yes (recommended) |

## 🛠️ Build System (C/C++)

This project uses **CMake** as the primary build system for cross-platform compatibility.

### Build Options

```bash
cd build
./build.sh 6axis     # Build 6-axis sensor fusion (default)
./build.sh 9axis     # Build 9-axis sensor fusion
./build.sh clean     # Clean build files
```

### Build Options

- **6axis**: Accelerometer + Gyroscope sensor fusion
- **9axis**: Accelerometer + Gyroscope + Magnetometer sensor fusion
- **debug**: Build with debug symbols (default)
- **release**: Build optimized release version

### Examples

```bash
cd build

# Build 6-axis debug version (default)
./build.sh 6axis

# Build 9-axis release version  
./build.sh 9axis release

# Clean and rebuild
./build.sh clean
./build.sh 9axis
```

### Manual CMake Build

```bash
cd build
mkdir -p cmake && cd cmake
cmake .. -DFUSION=6axis -DCMAKE_BUILD_TYPE=Debug
cmake --build . -j4
```

### Output Files

- **Executable**: `bin/SensorFusion`
- **Static Library**: `lib/libsensorfusion.a`
- **Object Files**: `build/obj/` (temporary)

### VS Code Integration

The project includes VS Code configuration:

- **Debug Configurations**: Pre-configured for both 6-axis and 9-axis debugging
- **Build Tasks**: Integrated CMake build tasks
- **IntelliSense**: Full C/C++ language support

### Project Structure

```
├── bin/                   # Output executables
├── lib/                   # Output libraries
├── build/                 # Build system and artifacts
│   ├── CMakeLists.txt    # CMake configuration
│   ├── build.sh          # Build script
│   ├── test_with_timeout.sh # Test script
│   ├── cmake/            # CMake build directory
│   └── obj/              # Object files (temporary)
└── code/                  # Source code
    ├── algo/             # Algorithm implementation
    │   ├── inc/         # Algorithm headers
    │   └── src/         # Algorithm sources
    └── app/             # Application code
        ├── inc/         # Application headers
        └── src/         # Application sources (main.c)
```

### Algorithm Selection

The build system automatically selects the appropriate algorithm files:

- **6-axis**: Uses `algo_sf_6x_sensor_fusion.c`
- **9-axis**: Uses `algo_sf_9x_sensor_fusion.c`

Preprocessor definitions are automatically set:
- `USE_6AXIS_FUSION` for 6-axis builds
- `USE_9AXIS_FUSION` for 9-axis builds

### Cross-Platform Support

- **macOS**: Native support with Clang
- **Linux**: GCC/Clang support
- **Windows**: MSVC/MinGW support via CMake

### Requirements

- CMake 3.10 or higher
- C99-compatible compiler
- Math library (`libm`)

### Debugging

Use VS Code's integrated debugger or run with GDB/LLDB:

```bash
# Build debug version
./build.sh 6axis debug

# Debug with LLDB (macOS)
lldb ./bin/SensorFusion

# Debug with GDB (Linux)
gdb ./bin/SensorFusion
```