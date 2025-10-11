#!/usr/bin/env python3
"""
Test data generator for QuatMath functions

Generates comprehensive test cases with known inputs and expected outputs
for validation against Python's built-in quaternion libraries.
"""

import numpy as np
import sys
import os

# Try to import scipy's spatial.transform.Rotation for reference quaternion operations
try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
    print("✓ SciPy available for reference quaternion operations")
except ImportError:
    SCIPY_AVAILABLE = False
    print("⚠️  SciPy not available - installing for reference operations...")
    import subprocess
    try:
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'scipy'])
        from scipy.spatial.transform import Rotation as R
        SCIPY_AVAILABLE = True
        print("✓ SciPy installed and imported")
    except:
        SCIPY_AVAILABLE = False
        print("✗ Could not install SciPy - using basic numpy operations only")

class QuatMathTestData:
    """Generate test data for QuatMath functions"""
    
    def __init__(self):
        self.test_cases = {}
        self._generate_basic_test_data()
        self._generate_rotation_test_data()
        self._generate_conversion_test_data()
    
    def _generate_basic_test_data(self):
        """Generate basic test cases for fundamental operations"""
        
        # Basic quaternions
        self.test_cases['basic_quats'] = {
            'identity': np.array([1.0, 0.0, 0.0, 0.0]),
            'x_90deg': np.array([np.cos(np.pi/4), np.sin(np.pi/4), 0.0, 0.0]),
            'y_90deg': np.array([np.cos(np.pi/4), 0.0, np.sin(np.pi/4), 0.0]),
            'z_90deg': np.array([np.cos(np.pi/4), 0.0, 0.0, np.sin(np.pi/4)]),
            'arbitrary': np.array([0.707, 0.408, 0.408, 0.408]),  # Normalized
            'small_rotation': np.array([0.9998, 0.0175, 0.0087, 0.0052]),  # ~2° rotation
        }
        
        # Angle conversion test cases
        self.test_cases['angles'] = {
            'zero': 0.0,
            'pi_4': np.pi / 4,
            'pi_2': np.pi / 2,
            'pi': np.pi,
            '3pi_2': 3 * np.pi / 2,
            '2pi': 2 * np.pi,
            'negative_pi': -np.pi,
            'small_angle': 0.01745,  # ~1 degree
            'large_angle': 6.28318,  # ~360 degrees
        }
        
        # Euler angles (roll, pitch, yaw) in radians
        self.test_cases['euler_angles'] = {
            'zero': np.array([0.0, 0.0, 0.0]),
            'roll_90': np.array([np.pi/2, 0.0, 0.0]),
            'pitch_90': np.array([0.0, np.pi/2, 0.0]),
            'yaw_90': np.array([0.0, 0.0, np.pi/2]),
            'combined_45': np.array([np.pi/4, np.pi/4, np.pi/4]),
            'gimbal_lock': np.array([0.0, np.pi/2, np.pi/4]),
        }
        
        # Axis-angle representations
        self.test_cases['axis_angles'] = {
            'x_axis_90': {'axis': np.array([1.0, 0.0, 0.0]), 'angle': np.pi/2},
            'y_axis_90': {'axis': np.array([0.0, 1.0, 0.0]), 'angle': np.pi/2},
            'z_axis_90': {'axis': np.array([0.0, 0.0, 1.0]), 'angle': np.pi/2},
            'diagonal_45': {'axis': np.array([1/np.sqrt(3), 1/np.sqrt(3), 1/np.sqrt(3)]), 'angle': np.pi/4},
        }
    
    def _generate_rotation_test_data(self):
        """Generate test data for rotation operations"""
        
        # Vectors to rotate
        self.test_cases['vectors'] = {
            'unit_x': np.array([1.0, 0.0, 0.0]),
            'unit_y': np.array([0.0, 1.0, 0.0]),
            'unit_z': np.array([0.0, 0.0, 1.0]),
            'arbitrary': np.array([1.0, 2.0, 3.0]),
            'normalized': np.array([0.577, 0.577, 0.577]),  # Normalized [1,1,1]
        }
        
        # Cross product test cases
        self.test_cases['cross_products'] = {
            'orthogonal': {
                'a': np.array([1.0, 0.0, 0.0]),
                'b': np.array([0.0, 1.0, 0.0]),
                'expected': np.array([0.0, 0.0, 1.0])
            },
            'parallel': {
                'a': np.array([1.0, 0.0, 0.0]),
                'b': np.array([2.0, 0.0, 0.0]),
                'expected': np.array([0.0, 0.0, 0.0])
            },
        }
    
    def _generate_conversion_test_data(self):
        """Generate test data with expected results using SciPy if available"""
        
        if not SCIPY_AVAILABLE:
            return
            
        # Generate expected quaternion results using SciPy
        self.test_cases['scipy_reference'] = {}
        
        # Euler to quaternion conversions
        for name, euler in self.test_cases['euler_angles'].items():
            try:
                rot = R.from_euler('xyz', euler)
                quat_wxyz = rot.as_quat()  # SciPy returns [x,y,z,w]
                # Convert to [w,x,y,z] format
                quat_reference = np.array([quat_wxyz[3], quat_wxyz[0], quat_wxyz[1], quat_wxyz[2]])
                self.test_cases['scipy_reference'][f'euler_{name}'] = {
                    'input': euler,
                    'expected_quat': quat_reference,
                    'expected_matrix': rot.as_matrix()
                }
            except:
                pass
        
        # Quaternion to Euler conversions
        for name, quat in self.test_cases['basic_quats'].items():
            try:
                # Convert [w,x,y,z] to [x,y,z,w] for SciPy
                quat_xyzw = np.array([quat[1], quat[2], quat[3], quat[0]])
                rot = R.from_quat(quat_xyzw)
                euler_reference = rot.as_euler('xyz')
                matrix_reference = rot.as_matrix()
                
                self.test_cases['scipy_reference'][f'quat_{name}'] = {
                    'input': quat,
                    'expected_euler': euler_reference,
                    'expected_matrix': matrix_reference
                }
            except:
                pass
    
    def get_test_case(self, category, name=None):
        """Get test case by category and optional name"""
        if name is None:
            return self.test_cases.get(category, {})
        return self.test_cases.get(category, {}).get(name, None)
    
    def get_all_categories(self):
        """Get all available test categories"""
        return list(self.test_cases.keys())
    
    def print_summary(self):
        """Print summary of available test data"""
        print("📊 QuatMath Test Data Summary")
        print("=" * 40)
        
        for category, data in self.test_cases.items():
            if isinstance(data, dict):
                print(f"🔹 {category}: {len(data)} test cases")
                for name in list(data.keys())[:3]:  # Show first 3
                    print(f"   • {name}")
                if len(data) > 3:
                    print(f"   ... and {len(data)-3} more")
            else:
                print(f"🔹 {category}: {type(data).__name__}")
        
        print(f"\n✅ Total categories: {len(self.test_cases)}")


def main():
    """Generate and display test data"""
    test_data = QuatMathTestData()
    test_data.print_summary()
    
    # Save test data for use by other test scripts
    import pickle
    
    test_data_file = os.path.join(os.path.dirname(__file__), 'quatmath_test_data.pkl')
    with open(test_data_file, 'wb') as f:
        pickle.dump(test_data.test_cases, f)
    
    print(f"\n💾 Test data saved to: {test_data_file}")
    
    return test_data


if __name__ == "__main__":
    test_data = main()