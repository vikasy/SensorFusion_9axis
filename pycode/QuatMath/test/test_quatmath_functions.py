#!/usr/bin/env python3
"""
Comprehensive Test Suite for QuatMath Functions

Tests all QuatMath functions against Python's built-in quaternion libraries
using systematic test cases and validation.
"""

import sys
import os
import numpy as np
import unittest
from pathlib import Path

# Add parent directories to path for imports
current_dir = Path(__file__).parent
quatmath_dir = current_dir.parent
pycode_dir = quatmath_dir.parent
root_dir = pycode_dir.parent
sys.path.insert(0, str(root_dir))
sys.path.insert(0, str(pycode_dir))
sys.path.insert(0, str(quatmath_dir))

# Import test data generator
from generate_test_data import QuatMathTestData

# Try to import SciPy for reference operations
try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# Import all QuatMath functions
try:
    from pycode.QuatMath.Deg2Rad import deg_to_rad
    from pycode.QuatMath.Rad2Deg import rad_to_deg
    from pycode.QuatMath.QuatProduct import quat_product
    from pycode.QuatMath.QuatConjugate import quat_conjugate
    from pycode.QuatMath.QuatNormal import quat_normalize
    from pycode.QuatMath.QuatInverse import quat_inverse
    from pycode.QuatMath.Quat2EulerAng import quat_to_euler_angles
    from pycode.QuatMath.Quat2DCMat import quat_to_dcm
    from pycode.QuatMath.EulerAng2RodMat import euler_to_rodriguez_matrix
    from pycode.QuatMath.CPMat import cp_mat
    QUATMATH_IMPORTS_OK = True
except ImportError as e:
    print(f"⚠️  Could not import QuatMath functions: {e}")
    QUATMATH_IMPORTS_OK = False


class TestQuatMathFunctions(unittest.TestCase):
    """Test suite for all QuatMath functions"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test data"""
        cls.test_data = QuatMathTestData()
        cls.tolerance = 1e-6  # Numerical tolerance for comparisons
    
    def assertArrayAlmostEqual(self, a, b, msg=None, tolerance=None):
        """Assert that two arrays are almost equal within tolerance"""
        if tolerance is None:
            tolerance = self.tolerance
        
        a, b = np.array(a), np.array(b)
        if a.shape != b.shape:
            self.fail(f"Arrays have different shapes: {a.shape} vs {b.shape}")
        
        diff = np.abs(a - b)
        max_diff = np.max(diff)
        
        if max_diff > tolerance:
            self.fail(f"Arrays differ by {max_diff:.2e} > {tolerance:.2e}\nA: {a}\nB: {b}")
    
    def test_angle_conversions(self):
        """Test degree/radian conversions"""
        if not QUATMATH_IMPORTS_OK:
            self.skipTest("QuatMath imports not available")
        
        print("\n🧪 Testing Angle Conversions")
        
        angles = self.test_data.get_test_case('angles')
        
        for name, rad_value in angles.items():
            with self.subTest(angle=name):
                # Test radian to degree
                deg_expected = rad_value * 180 / np.pi
                deg_result = rad_to_deg(rad_value)
                
                # Normalize to [0, 360) for positive angles
                if deg_expected >= 0:
                    deg_expected = deg_expected % 360
                    deg_result = deg_result % 360
                else:
                    deg_expected = deg_expected % -360
                    deg_result = deg_result % -360
                
                self.assertAlmostEqual(deg_result, deg_expected, places=5,
                                     msg=f"rad_to_deg({name}): {rad_value} rad")
                
                # Test degree to radian (round trip)
                rad_result = deg_to_rad(deg_result)
                # Normalize angles for comparison
                rad_normalized = (rad_value % (2*np.pi))
                rad_result_normalized = (rad_result % (2*np.pi))
                
                self.assertAlmostEqual(rad_result_normalized, rad_normalized, places=5,
                                     msg=f"Round trip deg_to_rad(rad_to_deg({name}))")
        
        print("✓ Angle conversion tests passed")
    
    def test_quaternion_basic_operations(self):
        """Test basic quaternion operations"""
        if not QUATMATH_IMPORTS_OK:
            self.skipTest("QuatMath imports not available")
        
        print("\n🧪 Testing Quaternion Basic Operations")
        
        quats = self.test_data.get_test_case('basic_quats')
        
        for name, quat in quats.items():
            with self.subTest(quaternion=name):
                # Test normalization
                normalized = quat_normalize(quat)
                norm = np.linalg.norm(normalized)
                self.assertAlmostEqual(norm, 1.0, places=6,
                                     msg=f"quat_normalize({name}) norm should be 1.0")
                
                # Test conjugate
                conjugate = quat_conjugate(quat)
                expected_conj = np.array([quat[0], -quat[1], -quat[2], -quat[3]])
                self.assertArrayAlmostEqual(conjugate, expected_conj,
                                          msg=f"quat_conjugate({name})")
                
                # Test inverse for normalized quaternion
                if abs(np.linalg.norm(quat) - 1.0) < 1e-6:  # If already normalized
                    inverse = quat_inverse(quat)
                    # For unit quaternion, inverse should equal conjugate
                    self.assertArrayAlmostEqual(inverse, conjugate, tolerance=1e-5,
                                              msg=f"quat_inverse({name}) should equal conjugate for unit quat")
        
        print("✓ Quaternion basic operation tests passed")
    
    def test_quaternion_multiplication(self):
        """Test quaternion multiplication"""
        if not QUATMATH_IMPORTS_OK:
            self.skipTest("QuatMath imports not available")
        
        print("\n🧪 Testing Quaternion Multiplication")
        
        quats = self.test_data.get_test_case('basic_quats')
        identity = quats['identity']
        
        # Test identity multiplication
        for name, quat in quats.items():
            with self.subTest(quaternion=name):
                # q * identity = q
                result1 = quat_product(quat, identity)
                self.assertArrayAlmostEqual(result1, quat,
                                          msg=f"{name} * identity = {name}")
                
                # identity * q = q
                result2 = quat_product(identity, quat)
                self.assertArrayAlmostEqual(result2, quat,
                                          msg=f"identity * {name} = {name}")
        
        # Test specific known multiplications
        x_90 = quats['x_90deg']
        y_90 = quats['y_90deg']
        
        # 90° X rotation followed by 90° Y rotation
        result = quat_product(y_90, x_90)
        
        # Verify the result makes sense (should be normalized)
        norm = np.linalg.norm(result)
        self.assertAlmostEqual(norm, 1.0, places=6,
                             msg="Product of unit quaternions should be unit")
        
        print("✓ Quaternion multiplication tests passed")
    
    def test_quaternion_to_euler_conversion(self):
        """Test quaternion to Euler angle conversion"""
        if not QUATMATH_IMPORTS_OK:
            self.skipTest("QuatMath imports not available")
        
        print("\n🧪 Testing Quaternion to Euler Conversion")
        
        # Test identity quaternion
        identity = self.test_data.get_test_case('basic_quats', 'identity')
        euler_result = quat_to_euler_angles(identity)
        euler_expected = np.array([0.0, 0.0, 0.0])
        
        self.assertArrayAlmostEqual(euler_result, euler_expected,
                                  msg="Identity quaternion should give zero Euler angles")
        
        # Test known rotations if SciPy is available for reference
        if SCIPY_AVAILABLE:
            scipy_ref = self.test_data.get_test_case('scipy_reference')
            
            for test_name, ref_data in scipy_ref.items():
                if 'expected_euler' in ref_data:
                    with self.subTest(test_case=test_name):
                        quat_input = ref_data['input']
                        expected_euler = ref_data['expected_euler']
                        
                        result_euler = quat_to_euler_angles(quat_input)
                        
                        # Compare with some tolerance due to numerical precision
                        # and potential gimbal lock issues
                        self.assertArrayAlmostEqual(result_euler, expected_euler, 
                                                  tolerance=1e-3,
                                                  msg=f"Euler conversion for {test_name}")
        
        print("✓ Quaternion to Euler conversion tests passed")
    
    def test_quaternion_to_dcm_conversion(self):
        """Test quaternion to direction cosine matrix conversion"""
        if not QUATMATH_IMPORTS_OK:
            self.skipTest("QuatMath imports not available")
        
        print("\n🧪 Testing Quaternion to DCM Conversion")
        
        quats = self.test_data.get_test_case('basic_quats')
        
        # Test identity quaternion
        identity = quats['identity']
        dcm_result = quat_to_dcm(identity)
        dcm_expected = np.eye(3)
        
        self.assertArrayAlmostEqual(dcm_result, dcm_expected,
                                  msg="Identity quaternion should give identity matrix")
        
        # Test that DCM is orthogonal (DCM * DCM^T = I)
        for name, quat in quats.items():
            with self.subTest(quaternion=name):
                dcm = quat_to_dcm(quat_normalize(quat))
                
                # Check orthogonality
                should_be_identity = dcm @ dcm.T
                self.assertArrayAlmostEqual(should_be_identity, np.eye(3), tolerance=1e-5,
                                          msg=f"DCM for {name} should be orthogonal")
                
                # Check determinant (should be 1 for proper rotation)
                det = np.linalg.det(dcm)
                self.assertAlmostEqual(det, 1.0, places=5,
                                     msg=f"DCM for {name} should have determinant 1")
        
        print("✓ Quaternion to DCM conversion tests passed")
    
    def test_cross_product_matrix(self):
        """Test cross product matrix function"""
        if not QUATMATH_IMPORTS_OK:
            self.skipTest("QuatMath imports not available")
        
        print("\n🧪 Testing Cross Product Matrix")
        
        vectors = self.test_data.get_test_case('vectors')
        cross_tests = self.test_data.get_test_case('cross_products')
        
        # Test known cross products
        for test_name, test_data in cross_tests.items():
            with self.subTest(test_case=test_name):
                a = test_data['a']
                b = test_data['b']
                expected = test_data['expected']
                
                # Create cross product matrix for 'a'
                cp_matrix = cp_mat(a)
                
                # Compute cross product using matrix: [a]_x * b
                result = cp_matrix @ b
                
                # Compare with expected
                self.assertArrayAlmostEqual(result, expected, tolerance=1e-10,
                                          msg=f"Cross product matrix test: {test_name}")
        
        # Test skew-symmetric property: [a]_x = -[a]_x^T
        test_vector = vectors['arbitrary']
        cp_matrix = cp_mat(test_vector)
        
        self.assertArrayAlmostEqual(cp_matrix, -cp_matrix.T, tolerance=1e-10,
                                  msg="Cross product matrix should be skew-symmetric")
        
        print("✓ Cross product matrix tests passed")
    
    def test_euler_to_rotation_matrix(self):
        """Test Euler angles to rotation matrix conversion"""
        if not QUATMATH_IMPORTS_OK:
            self.skipTest("QuatMath imports not available")
        
        print("\n🧪 Testing Euler to Rotation Matrix Conversion")
        
        euler_angles = self.test_data.get_test_case('euler_angles')
        
        # Test zero rotation
        zero_euler = euler_angles['zero']
        rot_matrix = euler_to_rodriguez_matrix(zero_euler)
        expected_identity = np.eye(3)
        
        self.assertArrayAlmostEqual(rot_matrix, expected_identity,
                                  msg="Zero Euler angles should give identity matrix")
        
        # Test that rotation matrices are orthogonal
        for name, euler in euler_angles.items():
            with self.subTest(euler_case=name):
                rot_matrix = euler_to_rodriguez_matrix(euler)
                
                # Check orthogonality
                should_be_identity = rot_matrix @ rot_matrix.T
                self.assertArrayAlmostEqual(should_be_identity, np.eye(3), tolerance=1e-5,
                                          msg=f"Rotation matrix for {name} should be orthogonal")
                
                # Check determinant
                det = np.linalg.det(rot_matrix)
                self.assertAlmostEqual(det, 1.0, places=5,
                                     msg=f"Rotation matrix for {name} should have determinant 1")
        
        print("✓ Euler to rotation matrix conversion tests passed")


def run_comprehensive_test():
    """Run the comprehensive test suite"""
    print("🧪 QuatMath Comprehensive Test Suite")
    print("=" * 50)
    
    if not QUATMATH_IMPORTS_OK:
        print("❌ Cannot run tests - QuatMath imports failed")
        return False
    
    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(TestQuatMathFunctions)
    
    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(suite)
    
    # Print summary
    print(f"\n📊 Test Results Summary")
    print("=" * 30)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.failures:
        print(f"\n❌ Failures:")
        for test, traceback in result.failures:
            print(f"  • {test}: {traceback.split('AssertionError:')[-1].strip()}")
    
    if result.errors:
        print(f"\n💥 Errors:")
        for test, traceback in result.errors:
            print(f"  • {test}: {traceback.split('Exception:')[-1].strip()}")
    
    success = len(result.failures) == 0 and len(result.errors) == 0
    
    if success:
        print(f"\n✅ All tests passed! QuatMath functions are working correctly.")
    else:
        print(f"\n⚠️  Some tests failed. Check the details above.")
    
    return success


if __name__ == "__main__":
    success = run_comprehensive_test()
    sys.exit(0 if success else 1)