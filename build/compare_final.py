"""Compare C and Python outputs after fixing gyro scale"""
import numpy as np
import csv

def load_csv(filename):
    data = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'ts': int(row['timestamp']),
                'q': np.array([float(row['q0']), float(row['q1']), float(row['q2']), float(row['q3'])]),
                'roll': float(row['roll']),
                'pitch': float(row['pitch']),
                'yaw': float(row['yaw'])
            })
    return data

def quat_distance(q1, q2):
    """Quaternion distance: 1 - |q1 · q2|"""
    dot = np.abs(np.dot(q1, q2))
    return 1.0 - dot

def angle_error(q1, q2):
    """Angular error in degrees"""
    dot = np.clip(np.abs(np.dot(q1, q2)), 0.0, 1.0)
    return 2.0 * np.arccos(dot) * 180.0 / np.pi

print("Loading outputs...")
c_data = load_csv('c_outputs_0922.csv')
py_data = load_csv('python_outputs_0922.csv')

print(f"C outputs: {len(c_data)}")
print(f"Python outputs: {len(py_data)}")

# Match by timestamp
quat_dists = []
angle_errs = []
roll_errs = []
pitch_errs = []

for c_row in c_data:
    # Find matching Python output
    py_row = next((p for p in py_data if p['ts'] == c_row['ts']), None)
    if py_row:
        qd = quat_distance(c_row['q'], py_row['q'])
        ae = angle_error(c_row['q'], py_row['q'])
        quat_dists.append(qd)
        angle_errs.append(ae)
        roll_errs.append(abs(c_row['roll'] - py_row['roll']))
        pitch_errs.append(abs(c_row['pitch'] - py_row['pitch']))

print(f"\n{'='*80}")
print("COMPARISON RESULTS (after fixing gyro scale)")
print(f"{'='*80}")
print(f"Matched samples: {len(quat_dists)}")
print(f"\nQuaternion Distance:")
print(f"  Mean:   {np.mean(quat_dists):.6f}")
print(f"  Median: {np.median(quat_dists):.6f}")
print(f"  Max:    {np.max(quat_dists):.6f}")
print(f"\nAngular Error (degrees):")
print(f"  Mean:   {np.mean(angle_errs):.3f}°")
print(f"  Median: {np.median(angle_errs):.3f}°")
print(f"  Max:    {np.max(angle_errs):.3f}°")
print(f"\nEuler Angle Errors:")
print(f"  Roll:  mean={np.mean(roll_errs):.3f}°, max={np.max(roll_errs):.3f}°")
print(f"  Pitch: mean={np.mean(pitch_errs):.3f}°, max={np.max(pitch_errs):.3f}°")

# Samples within tolerance
good_quat = sum(1 for d in quat_dists if d < 0.001)
good_angle = sum(1 for e in angle_errs if e < 1.0)

print(f"\nSamples within tolerance:")
print(f"  Quat distance < 0.001: {good_quat}/{len(quat_dists)} ({100*good_quat/len(quat_dists):.1f}%)")
print(f"  Angle error < 1.0°:    {good_angle}/{len(angle_errs)} ({100*good_angle/len(angle_errs):.1f}%)")

if np.mean(quat_dists) < 0.001 and np.mean(angle_errs) < 1.0:
    print(f"\n✅ SUCCESS: C and Python implementations match within tolerance!")
else:
    print(f"\n❌ Still have differences - need further investigation")
