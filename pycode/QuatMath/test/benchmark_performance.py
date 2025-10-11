#!/usr/bin/env python3
"""
Performance Benchmark for QuatMath Functions

Compares performance of our QuatMath functions against SciPy's implementations
and measures execution times for different input sizes.
"""

import sys
import os
import time
import numpy as np
from pathlib import Path

# Add parent directories to path
current_dir = Path(__file__).parent
quatmath_dir = current_dir.parent
pycode_dir = quatmath_dir.parent
root_dir = pycode_dir.parent
sys.path.insert(0, str(root_dir))
sys.path.insert(0, str(pycode_dir))
sys.path.insert(0, str(quatmath_dir))

# Try to import SciPy for comparison
try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

# Import QuatMath functions
try:
    from pycode.QuatMath.Deg2Rad import deg_to_rad
    from pycode.QuatMath.Rad2Deg import rad_to_deg
    from pycode.QuatMath.QuatProduct import quat_product
    from pycode.QuatMath.QuatConjugate import quat_conjugate
    from pycode.QuatMath.QuatNormal import quat_normalize
    from pycode.QuatMath.Quat2EulerAng import quat_to_euler_angles
    from pycode.QuatMath.Quat2DCMat import quat_to_dcm
    QUATMATH_AVAILABLE = True
except ImportError as e:
    print(f"❌ Could not import QuatMath functions: {e}")
    QUATMATH_AVAILABLE = False


def time_function(func, *args, iterations=1000):
    """Time a function execution"""
    # Warm up
    for _ in range(10):
        func(*args)
    
    # Actual timing
    start_time = time.perf_counter()
    for _ in range(iterations):
        result = func(*args)
    end_time = time.perf_counter()
    
    avg_time = (end_time - start_time) / iterations
    return avg_time, result


def benchmark_angle_conversions():
    """Benchmark angle conversion functions"""
    print("🏃 Benchmarking Angle Conversions")
    print("-" * 40)
    
    if not QUATMATH_AVAILABLE:
        print("❌ QuatMath functions not available")
        return
    
    # Test data
    angles_deg = np.array([0, 30, 45, 90, 180, 270, 360])
    angles_rad = np.deg2rad(angles_deg)
    
    iterations = 10000
    
    # Benchmark deg_to_rad
    total_time = 0
    for angle in angles_deg:
        avg_time, _ = time_function(deg_to_rad, angle, iterations=iterations)
        total_time += avg_time
    
    print(f"✓ deg_to_rad: {total_time*1e6:.2f} µs/call (avg over {len(angles_deg)} angles)")
    
    # Benchmark rad_to_deg
    total_time = 0
    for angle in angles_rad:
        avg_time, _ = time_function(rad_to_deg, angle, iterations=iterations)
        total_time += avg_time
    
    print(f"✓ rad_to_deg: {total_time*1e6:.2f} µs/call (avg over {len(angles_rad)} angles)")
    
    # Compare with numpy
    numpy_deg_time, _ = time_function(np.rad2deg, angles_rad[0], iterations=iterations)
    numpy_rad_time, _ = time_function(np.deg2rad, angles_deg[0], iterations=iterations)
    
    print(f"📊 NumPy np.deg2rad: {numpy_rad_time*1e6:.2f} µs/call")
    print(f"📊 NumPy np.rad2deg: {numpy_deg_time*1e6:.2f} µs/call")


def benchmark_quaternion_operations():
    """Benchmark quaternion operations"""
    print("\n🏃 Benchmarking Quaternion Operations")
    print("-" * 42)
    
    if not QUATMATH_AVAILABLE:
        print("❌ QuatMath functions not available")
        return
    
    # Test quaternions
    test_quats = [
        np.array([1.0, 0.0, 0.0, 0.0]),     # Identity
        np.array([0.707, 0.707, 0.0, 0.0]), # 90° X rotation
        np.array([0.6, 0.4, 0.5, 0.5]),     # Arbitrary quaternion
    ]
    
    iterations = 5000
    
    # Benchmark quaternion normalization
    total_time = 0
    for quat in test_quats:
        avg_time, _ = time_function(quat_normalize, quat, iterations=iterations)
        total_time += avg_time
    
    print(f"✓ quat_normalize: {total_time*1e6:.2f} µs/call (avg)")
    
    # Benchmark quaternion conjugate
    total_time = 0
    for quat in test_quats:
        avg_time, _ = time_function(quat_conjugate, quat, iterations=iterations)
        total_time += avg_time
    
    print(f"✓ quat_conjugate: {total_time*1e6:.2f} µs/call (avg)")
    
    # Benchmark quaternion multiplication
    q1, q2 = test_quats[0], test_quats[1]
    mult_time, _ = time_function(quat_product, q1, q2, iterations=iterations)
    print(f"✓ quat_product: {mult_time*1e6:.2f} µs/call")
    
    # Compare with SciPy if available
    if SCIPY_AVAILABLE:
        # SciPy quaternion operations
        q1_scipy = np.array([q1[1], q1[2], q1[3], q1[0]])  # Convert to [x,y,z,w]
        q2_scipy = np.array([q2[1], q2[2], q2[3], q2[0]])
        
        # Multiplication using SciPy
        def scipy_quat_mult(q1, q2):
            r1 = R.from_quat(q1)
            r2 = R.from_quat(q2)
            result = (r1 * r2).as_quat()
            return result
        
        scipy_mult_time, _ = time_function(scipy_quat_mult, q1_scipy, q2_scipy, iterations=iterations//10)
        print(f"📊 SciPy quaternion multiplication: {scipy_mult_time*1e6:.2f} µs/call")


def benchmark_conversions():
    """Benchmark conversion functions"""
    print("\n🏃 Benchmarking Conversion Functions")
    print("-" * 42)
    
    if not QUATMATH_AVAILABLE:
        print("❌ QuatMath functions not available")
        return
    
    test_quat = np.array([0.707, 0.5, 0.3, 0.2])
    test_quat = test_quat / np.linalg.norm(test_quat)  # Normalize
    
    iterations = 2000
    
    # Benchmark quaternion to Euler
    euler_time, euler_result = time_function(quat_to_euler_angles, test_quat, iterations=iterations)
    print(f"✓ quat_to_euler_angles: {euler_time*1e6:.2f} µs/call")
    
    # Benchmark quaternion to DCM
    dcm_time, dcm_result = time_function(quat_to_dcm, test_quat, iterations=iterations)
    print(f"✓ quat_to_dcm: {dcm_time*1e6:.2f} µs/call")
    
    # Compare with SciPy if available
    if SCIPY_AVAILABLE:
        test_quat_scipy = np.array([test_quat[1], test_quat[2], test_quat[3], test_quat[0]])
        
        def scipy_to_euler(q):
            return R.from_quat(q).as_euler('xyz')
        
        def scipy_to_matrix(q):
            return R.from_quat(q).as_matrix()
        
        scipy_euler_time, _ = time_function(scipy_to_euler, test_quat_scipy, iterations=iterations//5)
        scipy_matrix_time, _ = time_function(scipy_to_matrix, test_quat_scipy, iterations=iterations//5)
        
        print(f"📊 SciPy to Euler: {scipy_euler_time*1e6:.2f} µs/call")
        print(f"📊 SciPy to matrix: {scipy_matrix_time*1e6:.2f} µs/call")


def benchmark_array_operations():
    """Benchmark with array inputs (vectorized operations)"""
    print("\n🏃 Benchmarking Array Operations")
    print("-" * 40)
    
    if not QUATMATH_AVAILABLE:
        print("❌ QuatMath functions not available")
        return
    
    # Create arrays of test data
    n_samples = 1000
    angles_deg = np.random.uniform(0, 360, n_samples)
    angles_rad = np.random.uniform(0, 2*np.pi, n_samples)
    
    iterations = 100
    
    print(f"Testing with {n_samples} samples...")
    
    # Test if our functions can handle arrays (vectorized)
    try:
        # Test deg_to_rad with array
        start_time = time.perf_counter()
        for _ in range(iterations):
            results = [deg_to_rad(angle) for angle in angles_deg[:100]]  # Sample first 100
        array_time = (time.perf_counter() - start_time) / iterations
        
        print(f"✓ deg_to_rad (100 elements): {array_time*1000:.2f} ms")
        
        # Compare with numpy vectorized
        start_time = time.perf_counter()
        for _ in range(iterations):
            results = np.deg2rad(angles_deg[:100])
        numpy_time = (time.perf_counter() - start_time) / iterations
        
        print(f"📊 NumPy np.deg2rad (100 elements): {numpy_time*1000:.2f} ms")
        print(f"   Speedup factor: {array_time/numpy_time:.1f}x slower")
        
    except Exception as e:
        print(f"⚠️  Array operations not fully vectorized: {e}")


def generate_performance_report():
    """Generate a comprehensive performance report"""
    print("📊 QuatMath Performance Report")
    print("=" * 50)
    print(f"Python version: {sys.version}")
    print(f"NumPy version: {np.__version__}")
    
    if SCIPY_AVAILABLE:
        import scipy
        print(f"SciPy version: {scipy.__version__}")
    else:
        print("SciPy: Not available")
    
    print(f"System: {os.name}")
    print()
    
    if not QUATMATH_AVAILABLE:
        print("❌ Cannot run benchmarks - QuatMath functions not available")
        return
    
    # Run all benchmarks
    benchmark_angle_conversions()
    benchmark_quaternion_operations() 
    benchmark_conversions()
    benchmark_array_operations()
    
    print("\n🏁 Performance Analysis Complete")
    print("=" * 40)
    print("💡 Tips for optimization:")
    print("   • Use NumPy vectorized operations for large arrays")
    print("   • Consider SciPy for complex quaternion sequences")
    print("   • Our functions are optimized for single-value operations")
    print("   • Cache results when possible for repeated calculations")


if __name__ == "__main__":
    generate_performance_report()