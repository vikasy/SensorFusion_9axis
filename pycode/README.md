# SensorFusion Python Package

This directory contains Python implementations of sensor fusion algorithms converted from MATLAB code.

## Structure

```
pycode/
├── __init__.py                  # Main package initialization
├── sensor_fusion_main.py        # Main sensor fusion class and entry point
├── SF_*.py                      # Main sensor fusion algorithms
├── QuatMath/                    # Quaternion mathematics library
│   ├── __init__.py
│   ├── QuatProduct.py           # Quaternion multiplication
│   ├── QuatConjugate.py         # Quaternion conjugate
│   ├── QuatNormal.py            # Quaternion normalization
│   ├── Deg2Rad.py               # Degree to radian conversion
│   ├── Rad2Deg.py               # Radian to degree conversion
│   └── *.py                     # Other quaternion operations
└── README.md                    # This file
```

## Installation

1. **Install required packages:**
   ```bash
   pip install numpy matplotlib scipy pandas
   ```

2. **Test the conversion:**
   ```bash
   python ../test_python_conversion.py
   ```

## Usage

### Basic Example

```python
import sys
sys.path.append('pycode')

from sensor_fusion_main import SensorFusion
from QuatMath.Deg2Rad import deg_to_rad
from QuatMath.QuatProduct import quat_product

# Create sensor fusion instance
sf = SensorFusion(fusion_type='9X_AGM')

# Convert angles
angle_rad = deg_to_rad(45)  # Convert 45° to radians

# Quaternion operations
q1 = [1, 0, 0, 0]  # Identity quaternion
q2 = [0.707, 0, 0, 0.707]  # 90° rotation about z-axis
result = quat_product(q1, q2)
print(f"Quaternion product: {result}")
```

### Sensor Data Processing

```python
# Load sensor data (placeholder - implement based on your data format)
data = sf.load_data('sensor_data.xlsx', method=2)

# Process sensor fusion
for i in range(len(data['acc'])):
    acc = data['acc'][i, 1:4]    # Accelerometer [x, y, z]
    gyro = data['gyro'][i, 1:4]  # Gyroscope [x, y, z] 
    mag = data['mag'][i, 1:4]    # Magnetometer [x, y, z]
    timestamp = data['acc'][i, 0]
    
    # Update fusion algorithm
    result = sf.update_fusion(acc, gyro, mag, timestamp)
    
    # Extract results
    quaternion = result['quaternion']  # [w, x, y, z]
    roll = result['phi']               # Roll angle (degrees)
    pitch = result['theta']            # Pitch angle (degrees) 
    yaw = result['psi']                # Yaw angle (degrees)
```

## Key Functions

### QuatMath Module

- **`deg_to_rad(degrees)`** - Convert degrees to radians
- **`rad_to_deg(radians)`** - Convert radians to degrees  
- **`quat_product(p, q)`** - Multiply two quaternions
- **`quat_conjugate(q)`** - Compute quaternion conjugate
- **`quat_normalize(q)`** - Normalize quaternion
- **`quat_to_euler(q)`** - Convert quaternion to Euler angles
- **`euler_to_quat(angles)`** - Convert Euler angles to quaternion

### Main Sensor Fusion

- **`SensorFusion(fusion_type)`** - Initialize sensor fusion algorithm
- **`load_data(file, method)`** - Load sensor data from file
- **`update_fusion(acc, gyro, mag, ts)`** - Update fusion with sensor data
- **`plot_results(results)`** - Plot quaternions and orientation angles

## Conversion Notes

⚠️ **Important**: These files are auto-converted from MATLAB and may require manual adjustments:

1. **Array Indexing**: MATLAB uses 1-based indexing, Python uses 0-based
2. **Matrix Operations**: Some MATLAB syntax may not translate perfectly
3. **Function Signatures**: Parameter passing may need adjustment
4. **Global Variables**: MATLAB global variables converted to class attributes
5. **Plot Functions**: MATLAB plotting converted to matplotlib equivalents

## Testing

Run the test script to verify basic functionality:

```bash
python ../test_python_conversion.py
```

Expected output:
```
✓ Successfully imported deg_to_rad
90° = 1.570796 rad (expected 1.570796)
✓ Successfully imported quat_product  
Identity * rotation = [0.707 0.    0.    0.707]
```

## Development

To improve the converted code:

1. **Review Function Logic**: Check mathematical operations for correctness
2. **Fix Array Operations**: Ensure proper NumPy array handling
3. **Add Error Checking**: Include input validation and error handling
4. **Optimize Performance**: Use vectorized NumPy operations where possible
5. **Add Unit Tests**: Create comprehensive test suite for validation

## Original MATLAB Files

The converted Python functions correspond to these MATLAB files:

- `SF_Main.m` → `SF_Main.py` - Main sensor fusion algorithm
- `SF_Init_State.m` → `SF_Init_State.py` - State initialization
- `SF_Update_AGM.m` → `SF_Update_AGM.py` - AGM sensor fusion update
- `QuatMath/*.m` → `QuatMath/*.py` - Quaternion mathematics library

## License

Converted from original MATLAB sensor fusion algorithms. Please refer to the original license terms.