#!/usr/bin/env python3
"""
Test Vector Generator for SensorFusion Unit Tests

Generates mathematically verified test vectors for:
- Quaternion operations (using scipy.spatial.transform.Rotation)
- Matrix operations (using numpy)

Output: C header files with test data arrays
Author: Vikas Yadav
Date: 2025-10-11
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
from pathlib import Path

def format_quaternion_c(q, var_name):
    """Format quaternion as C initializer (w, x, y, z format)"""
    return f"quaternion_double_t {var_name} = {{{q[0]:.17e}, {q[1]:.17e}, {q[2]:.17e}, {q[3]:.17e}}};"

def format_matrix_c(m, var_name):
    """Format 3x3 matrix as C initializer"""
    lines = [f"double {var_name}[3][3] = {{"]
    for i in range(3):
        row = ", ".join([f"{m[i,j]:.17e}" for j in range(3)])
        lines.append(f"    {{{row}}},")
    lines[-1] = lines[-1][:-1]  # Remove trailing comma
    lines.append("};")
    return "\n".join(lines)

def format_angles_c(angles, var_name):
    """Format Euler angles as C array (degrees)"""
    return f"double {var_name}[3] = {{{angles[0]:.17e}, {angles[1]:.17e}, {angles[2]:.17e}}}; // roll, pitch, yaw in degrees"

def generate_quaternion_test_vectors():
    """Generate verified quaternion test vectors"""

    test_cases = []

    # Test 1: Identity quaternion
    q_identity = np.array([1.0, 0.0, 0.0, 0.0])
    test_cases.append({
        'name': 'identity',
        'quat': q_identity,
        'description': 'Identity quaternion (no rotation)'
    })

    # Test 2: 90° rotation around X-axis
    r_x90 = R.from_euler('x', 90, degrees=True)
    q_x90 = r_x90.as_quat(scalar_first=True)  # [w, x, y, z]
    test_cases.append({
        'name': 'rot_x_90deg',
        'quat': q_x90,
        'description': '90° rotation around X-axis'
    })

    # Test 3: 90° rotation around Y-axis
    r_y90 = R.from_euler('y', 90, degrees=True)
    q_y90 = r_y90.as_quat(scalar_first=True)
    test_cases.append({
        'name': 'rot_y_90deg',
        'quat': q_y90,
        'description': '90° rotation around Y-axis'
    })

    # Test 4: 90° rotation around Z-axis
    r_z90 = R.from_euler('z', 90, degrees=True)
    q_z90 = r_z90.as_quat(scalar_first=True)
    test_cases.append({
        'name': 'rot_z_90deg',
        'quat': q_z90,
        'description': '90° rotation around Z-axis'
    })

    # Test 5: 45° rotation around arbitrary axis [1, 1, 1]
    axis = np.array([1.0, 1.0, 1.0])
    axis = axis / np.linalg.norm(axis)
    angle_rad = np.radians(45)
    r_arb = R.from_rotvec(angle_rad * axis)
    q_arb = r_arb.as_quat(scalar_first=True)
    test_cases.append({
        'name': 'rot_arbitrary_45deg',
        'quat': q_arb,
        'description': '45° rotation around [1,1,1] axis'
    })

    # Test 6: Quaternion multiplication (X90 * Y90)
    r_combined = r_x90 * r_y90
    q_combined = r_combined.as_quat(scalar_first=True)
    test_cases.append({
        'name': 'product_x90_y90',
        'quat1': q_x90,
        'quat2': q_y90,
        'result': q_combined,
        'description': 'Product of X90 and Y90 rotations'
    })

    # Test 7: Quaternion to rotation matrix (X90)
    rot_matrix_x90 = r_x90.as_matrix()
    test_cases.append({
        'name': 'quat_to_rotmtx_x90',
        'quat': q_x90,
        'rotmtx': rot_matrix_x90,
        'description': 'X90 quaternion to rotation matrix'
    })

    # Test 8: Rotation matrix to quaternion (Y90)
    rot_matrix_y90 = r_y90.as_matrix()
    test_cases.append({
        'name': 'rotmtx_to_quat_y90',
        'rotmtx': rot_matrix_y90,
        'quat': q_y90,
        'description': 'Y90 rotation matrix to quaternion'
    })

    # Test 9: Euler angles to quaternion (30° roll, 45° pitch, 60° yaw)
    r_euler = R.from_euler('xyz', [30, 45, 60], degrees=True)
    q_euler = r_euler.as_quat(scalar_first=True)
    test_cases.append({
        'name': 'euler_30_45_60',
        'angles': np.array([30.0, 45.0, 60.0]),  # roll, pitch, yaw
        'quat': q_euler,
        'description': 'Euler angles (30°, 45°, 60°) to quaternion'
    })

    # Test 10: Quaternion integration (angular velocity over time)
    omega = np.array([0.0, 0.0, 10.0])  # 10 rad/s around Z
    dt = 0.1  # 100ms
    angle = np.linalg.norm(omega) * dt
    if angle > 1e-9:
        axis_norm = omega / np.linalg.norm(omega)
        r_integrated = R.from_rotvec(angle * axis_norm)
        q_integrated = r_integrated.as_quat(scalar_first=True)
    else:
        q_integrated = q_identity

    test_cases.append({
        'name': 'integrate_omega_z10',
        'omega': omega,
        'dt': dt,
        'result': q_integrated,
        'description': 'Integrate ω=[0,0,10] rad/s for 0.1s'
    })

    return test_cases

def generate_matrix_test_vectors():
    """Generate verified matrix operation test vectors"""

    test_cases = []

    # Test 1: Identity matrix
    I = np.eye(3)
    test_cases.append({
        'name': 'identity',
        'matrix': I,
        'description': '3x3 Identity matrix'
    })

    # Test 2: Matrix multiplication (A * B)
    A = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], dtype=float)
    B = np.array([[9, 8, 7], [6, 5, 4], [3, 2, 1]], dtype=float)
    C = A @ B
    test_cases.append({
        'name': 'multiply_AB',
        'matrix1': A,
        'matrix2': B,
        'result': C,
        'description': 'Matrix multiplication A*B'
    })

    # Test 3: Matrix transpose
    A_T = A.T
    test_cases.append({
        'name': 'transpose_A',
        'matrix': A,
        'result': A_T,
        'description': 'Transpose of matrix A'
    })

    # Test 4: Matrix addition
    D = A + B
    test_cases.append({
        'name': 'add_AB',
        'matrix1': A,
        'matrix2': B,
        'result': D,
        'description': 'Matrix addition A+B'
    })

    # Test 5: Matrix subtraction
    E = A - B
    test_cases.append({
        'name': 'subtract_AB',
        'matrix1': A,
        'matrix2': B,
        'result': E,
        'description': 'Matrix subtraction A-B'
    })

    # Test 6: Scalar multiplication
    F = 2.5 * A
    test_cases.append({
        'name': 'scalar_multiply_A',
        'matrix': A,
        'scalar': 2.5,
        'result': F,
        'description': 'Scalar multiplication 2.5*A'
    })

    # Test 7: Rotation matrix from X-axis rotation (90°)
    angle = np.radians(90)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])
    test_cases.append({
        'name': 'rotation_x_90deg',
        'matrix': R_x,
        'description': 'Rotation matrix for 90° around X-axis'
    })

    # Test 8: Rotation matrix from Y-axis rotation (90°)
    R_y = np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])
    test_cases.append({
        'name': 'rotation_y_90deg',
        'matrix': R_y,
        'description': 'Rotation matrix for 90° around Y-axis'
    })

    # Test 9: Rotation matrix from Z-axis rotation (90°)
    R_z = np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])
    test_cases.append({
        'name': 'rotation_z_90deg',
        'matrix': R_z,
        'description': 'Rotation matrix for 90° around Z-axis'
    })

    # Test 10: Combined rotation (Rx * Ry)
    R_xy = R_x @ R_y
    test_cases.append({
        'name': 'rotation_combined_xy',
        'matrix1': R_x,
        'matrix2': R_y,
        'result': R_xy,
        'description': 'Combined rotation Rx(90°) * Ry(90°)'
    })

    return test_cases

def generate_quaternion_header():
    """Generate C header file with quaternion test vectors"""

    test_cases = generate_quaternion_test_vectors()

    lines = []
    lines.append("/*******************************************************************************")
    lines.append(" * Verified Quaternion Test Vectors")
    lines.append(" * ")
    lines.append(" * Generated by: scripts/generate_test_vectors.py")
    lines.append(" * Using: scipy.spatial.transform.Rotation")
    lines.append(" * Date: 2025-10-11")
    lines.append(" * ")
    lines.append(" * Quaternion format: [q0, q1, q2, q3] = [w, x, y, z]")
    lines.append(" ******************************************************************************/")
    lines.append("")
    lines.append("#ifndef TEST_VECTORS_QUAT_H")
    lines.append("#define TEST_VECTORS_QUAT_H")
    lines.append("")
    lines.append('#include "algo_sf_quatmath.h"')
    lines.append("")

    # Generate test data
    for tc in test_cases:
        lines.append(f"// {tc['description']}")

        if 'quat' in tc and 'quat1' not in tc:
            lines.append(format_quaternion_c(tc['quat'], f"test_quat_{tc['name']}"))

        if 'quat1' in tc:
            lines.append(format_quaternion_c(tc['quat1'], f"test_quat_{tc['name']}_input1"))
            lines.append(format_quaternion_c(tc['quat2'], f"test_quat_{tc['name']}_input2"))
            lines.append(format_quaternion_c(tc['result'], f"test_quat_{tc['name']}_expected"))

        if 'rotmtx' in tc and 'quat' in tc:
            lines.append(format_matrix_c(tc['rotmtx'], f"test_rotmtx_{tc['name']}"))
            lines.append(format_quaternion_c(tc['quat'], f"test_quat_{tc['name']}_expected"))

        if 'angles' in tc:
            lines.append(format_angles_c(tc['angles'], f"test_angles_{tc['name']}"))
            lines.append(format_quaternion_c(tc['quat'], f"test_quat_{tc['name']}_expected"))

        if 'omega' in tc:
            lines.append(f"double test_omega_{tc['name']}[3] = {{{tc['omega'][0]:.17e}, {tc['omega'][1]:.17e}, {tc['omega'][2]:.17e}}}; // rad/s")
            lines.append(f"double test_dt_{tc['name']} = {tc['dt']:.17e}; // seconds")
            lines.append(format_quaternion_c(tc['result'], f"test_quat_{tc['name']}_expected"))

        lines.append("")

    lines.append("#endif // TEST_VECTORS_QUAT_H")
    lines.append("")

    return "\n".join(lines)

def generate_matrix_header():
    """Generate C header file with matrix test vectors"""

    test_cases = generate_matrix_test_vectors()

    lines = []
    lines.append("/*******************************************************************************")
    lines.append(" * Verified Matrix Test Vectors")
    lines.append(" * ")
    lines.append(" * Generated by: scripts/generate_test_vectors.py")
    lines.append(" * Using: numpy")
    lines.append(" * Date: 2025-10-11")
    lines.append(" ******************************************************************************/")
    lines.append("")
    lines.append("#ifndef TEST_VECTORS_MATRIX_H")
    lines.append("#define TEST_VECTORS_MATRIX_H")
    lines.append("")

    # Generate test data
    for tc in test_cases:
        lines.append(f"// {tc['description']}")

        if 'matrix' in tc and 'matrix1' not in tc and 'scalar' not in tc:
            lines.append(format_matrix_c(tc['matrix'], f"test_matrix_{tc['name']}"))

        if 'matrix1' in tc:
            lines.append(format_matrix_c(tc['matrix1'], f"test_matrix_{tc['name']}_A"))
            lines.append(format_matrix_c(tc['matrix2'], f"test_matrix_{tc['name']}_B"))
            lines.append(format_matrix_c(tc['result'], f"test_matrix_{tc['name']}_expected"))

        if 'scalar' in tc:
            lines.append(format_matrix_c(tc['matrix'], f"test_matrix_{tc['name']}_input"))
            lines.append(f"double test_scalar_{tc['name']} = {tc['scalar']:.17e};")
            lines.append(format_matrix_c(tc['result'], f"test_matrix_{tc['name']}_expected"))

        if 'result' in tc and 'matrix1' not in tc and 'scalar' not in tc:
            lines.append(format_matrix_c(tc['result'], f"test_matrix_{tc['name']}_expected"))

        lines.append("")

    lines.append("#endif // TEST_VECTORS_MATRIX_H")
    lines.append("")

    return "\n".join(lines)

def main():
    print("Generating verified test vectors...")
    print("=" * 70)

    # Generate quaternion test vectors
    print("\n1. Generating quaternion test vectors...")
    quat_header = generate_quaternion_header()
    quat_path = Path(__file__).parent.parent / "test" / "fixtures" / "test_vectors_quat.h"
    with open(quat_path, 'w') as f:
        f.write(quat_header)
    print(f"   ✓ Written to: {quat_path}")

    # Generate matrix test vectors
    print("\n2. Generating matrix test vectors...")
    matrix_header = generate_matrix_header()
    matrix_path = Path(__file__).parent.parent / "test" / "fixtures" / "test_vectors_matrix.h"
    with open(matrix_path, 'w') as f:
        f.write(matrix_header)
    print(f"   ✓ Written to: {matrix_path}")

    print("\n" + "=" * 70)
    print("✓ Test vector generation complete!")
    print("\nNext steps:")
    print("  1. Include headers in test files:")
    print('     #include "test_vectors_quat.h"')
    print('     #include "test_vectors_matrix.h"')
    print("  2. Use test vectors in unit tests")
    print("  3. Build and run: make && ./bin/test_quatmath")

    return 0

if __name__ == "__main__":
    sys.exit(main())
