#!/usr/bin/env python3
"""
Convert test data from INVENSENSE (MPU9250) format to FREESCALE (FXOS8700CQ+FXAS21000) format

The raw ADC counts are different between platforms due to different:
- Bit depths (16-bit vs 14-bit for accelerometer)
- Full-scale ranges (±4800µT vs ±1200µT for magnetometer)
- Sensitivities

This script converts by:
1. Reading INVENSENSE raw counts
2. Converting to physical units (g, dps, µT)
3. Converting to FREESCALE raw counts

Author: Vikas Yadav
Date: 2025-10-12
"""

import sys
import re

# INVENSENSE MPU9250 specifications (current test data)
INV_ACCEL_RANGE_G = 4.0
INV_ACCEL_MAX_COUNT = 32767
INV_ACCEL_SCALE = INV_ACCEL_RANGE_G / INV_ACCEL_MAX_COUNT  # g per count

INV_GYRO_RANGE_DPS = 1000.0
INV_GYRO_MAX_COUNT = 32767
INV_GYRO_SCALE = INV_GYRO_RANGE_DPS / INV_GYRO_MAX_COUNT  # dps per count

INV_MAG_RANGE_UT = 4800.0
INV_MAG_MAX_COUNT = 32767
INV_MAG_SCALE = INV_MAG_RANGE_UT / INV_MAG_MAX_COUNT  # µT per count

# FREESCALE FRDM-STBC-AGM01 specifications
FSL_ACCEL_RANGE_G = 4.0
FSL_ACCEL_MAX_COUNT = 8191  # 14-bit
FSL_ACCEL_SCALE = FSL_ACCEL_RANGE_G / FSL_ACCEL_MAX_COUNT  # g per count

FSL_GYRO_RANGE_DPS = 1000.0
FSL_GYRO_MAX_COUNT = 32767  # 16-bit
FSL_GYRO_SCALE = FSL_GYRO_RANGE_DPS / FSL_GYRO_MAX_COUNT  # dps per count

FSL_MAG_RANGE_UT = 1200.0  # Different range!
FSL_MAG_MAX_COUNT = 32767  # 16-bit
FSL_MAG_SCALE = FSL_MAG_RANGE_UT / FSL_MAG_MAX_COUNT  # µT per count

def convert_sample(sensor_id, x, y, z):
    """
    Convert one sensor sample from INVENSENSE to FREESCALE format

    Returns:
        Tuple of (x_fsl, y_fsl, z_fsl) in FREESCALE raw counts
    """
    if sensor_id == 0:  # Accelerometer
        # Convert to physical units (g)
        x_g = x * INV_ACCEL_SCALE
        y_g = y * INV_ACCEL_SCALE
        z_g = z * INV_ACCEL_SCALE

        # Convert to FREESCALE counts
        x_fsl = int(round(x_g / FSL_ACCEL_SCALE))
        y_fsl = int(round(y_g / FSL_ACCEL_SCALE))
        z_fsl = int(round(z_g / FSL_ACCEL_SCALE))

    elif sensor_id == 1:  # Gyroscope
        # Convert to physical units (dps)
        x_dps = x * INV_GYRO_SCALE
        y_dps = y * INV_GYRO_SCALE
        z_dps = z * INV_GYRO_SCALE

        # Convert to FREESCALE counts (same scale, just recalculate)
        x_fsl = int(round(x_dps / FSL_GYRO_SCALE))
        y_fsl = int(round(y_dps / FSL_GYRO_SCALE))
        z_fsl = int(round(z_dps / FSL_GYRO_SCALE))

    elif sensor_id == 2:  # Magnetometer
        # Convert to physical units (µT)
        x_ut = x * INV_MAG_SCALE
        y_ut = y * INV_MAG_SCALE
        z_ut = z * INV_MAG_SCALE

        # Convert to FREESCALE counts
        # Note: FREESCALE has smaller range, so values might clip
        x_fsl = int(round(x_ut / FSL_MAG_SCALE))
        y_fsl = int(round(y_ut / FSL_MAG_SCALE))
        z_fsl = int(round(z_ut / FSL_MAG_SCALE))

        # Clip to valid range
        max_mag = FSL_MAG_MAX_COUNT
        x_fsl = max(-max_mag, min(max_mag, x_fsl))
        y_fsl = max(-max_mag, min(max_mag, y_fsl))
        z_fsl = max(-max_mag, min(max_mag, z_fsl))
    else:
        raise ValueError(f"Unknown sensor ID: {sensor_id}")

    return x_fsl, y_fsl, z_fsl

def main():
    input_file = 'test_input_output_0922.h'
    output_file = 'test_input_output_0922_freescale.h'

    print(f"Converting test data from INVENSENSE to FREESCALE format...")
    print(f"Input:  {input_file}")
    print(f"Output: {output_file}")

    # Read input file
    with open(input_file, 'r') as f:
        content = f.read()

    # Extract header and footer
    header_end = content.find('static const test_sensor_sample_t sensor_input_data[] = {')
    if header_end == -1:
        print("Error: Could not find data array")
        return 1

    header_end += len('static const test_sensor_sample_t sensor_input_data[] = {')
    header = content[:header_end]

    # Find closing brace
    footer_start = content.rfind('};')
    if footer_start == -1:
        print("Error: Could not find end of array")
        return 1

    footer = content[footer_start:]

    # Parse sensor data
    pattern = r'\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+)ULL\}'
    matches = re.findall(pattern, content)

    print(f"Found {len(matches)} sensor samples")

    # Convert samples
    converted_lines = []
    for match in matches:
        sensor_id = int(match[0])
        x_inv = int(match[1])
        y_inv = int(match[2])
        z_inv = int(match[3])
        timestamp = match[4]

        # Convert to FREESCALE
        x_fsl, y_fsl, z_fsl = convert_sample(sensor_id, x_inv, y_inv, z_inv)

        # Format output line
        line = f"    {{{sensor_id}, {x_fsl}, {y_fsl}, {z_fsl}, {timestamp}ULL}},"
        converted_lines.append(line)

    # Update header comment
    header = header.replace(
        '/* Auto-generated from test_input_output_0922.xlsx */',
        '/* Auto-generated from test_input_output_0922.xlsx - FREESCALE sensor format */'
    )
    header = header.replace(
        '#ifndef TEST_INPUT_OUTPUT_0922_H',
        '#ifndef TEST_INPUT_OUTPUT_0922_FREESCALE_H'
    )
    header = header.replace(
        '#define TEST_INPUT_OUTPUT_0922_H',
        '#define TEST_INPUT_OUTPUT_0922_FREESCALE_H\n\n' +
        '// Sensor data converted for FREESCALE platform (FXOS8700CQ + FXAS21000)\n' +
        '// - Accelerometer: 14-bit, ±4g range\n' +
        '// - Gyroscope: 16-bit, ±1000dps range\n' +
        '// - Magnetometer: 16-bit, ±1200µT range'
    )

    # Write output file
    output = header + '\n' + '\n'.join(converted_lines) + '\n' + footer

    with open(output_file, 'w') as f:
        f.write(output)

    print(f"Conversion complete!")
    print(f"Converted {len(matches)} samples")
    print(f"\nScale factor comparison:")
    print(f"  Accelerometer: {INV_ACCEL_MAX_COUNT} → {FSL_ACCEL_MAX_COUNT} counts (for same ±4g)")
    print(f"  Gyroscope: {INV_GYRO_MAX_COUNT} → {FSL_GYRO_MAX_COUNT} counts (same)")
    print(f"  Magnetometer: ±{INV_MAG_RANGE_UT}µT → ±{FSL_MAG_RANGE_UT}µT range")

    # Show a few sample conversions
    print(f"\nSample conversions:")
    for i in [0, 1, 2, 10, 11, 12]:
        match = matches[i]
        sensor_id = int(match[0])
        x_inv, y_inv, z_inv = int(match[1]), int(match[2]), int(match[3])
        x_fsl, y_fsl, z_fsl = convert_sample(sensor_id, x_inv, y_inv, z_inv)

        sensor_name = ['Accel', 'Gyro', 'Mag'][sensor_id]
        print(f"  {sensor_name}: INV[{x_inv:6d}, {y_inv:6d}, {z_inv:6d}] → "
              f"FSL[{x_fsl:6d}, {y_fsl:6d}, {z_fsl:6d}]")

    return 0

if __name__ == '__main__':
    sys.exit(main())
