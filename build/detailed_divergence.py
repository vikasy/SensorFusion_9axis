"""Detailed analysis of first divergence point"""
import csv
import numpy as np

def load_data(filename):
    data = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'ts': int(row['timestamp']),
                'q0': float(row['q0']),
                'q1': float(row['q1']),
                'q2': float(row['q2']),
                'q3': float(row['q3']),
                'roll': float(row['roll']),
                'pitch': float(row['pitch']),
                'yaw': float(row['yaw'])
            })
    return data

c_data = load_data('c_outputs_0922.csv')
py_data = load_data('python_outputs_0922.csv')

print(f"Checking samples 260-266 around first divergence point...")
print(f"\n{'='*100}")

for idx in range(260, 267):
    c = c_data[idx]
    p = py_data[idx]

    # Compute quaternion distance
    c_q = np.array([c['q0'], c['q1'], c['q2'], c['q3']])
    p_q = np.array([p['q0'], p['q1'], p['q2'], p['q3']])
    q_dist = 1.0 - np.abs(np.dot(c_q, p_q))

    print(f"\n**Sample {idx}** (ts={c['ts']})")
    print(f"  C:      [{c['q0']:.8f}, {c['q1']:.8f}, {c['q2']:.8f}, {c['q3']:.8f}]")
    print(f"  Python: [{p['q0']:.8f}, {p['q1']:.8f}, {p['q2']:.8f}, {p['q3']:.8f}]")
    print(f"  Quat distance: {q_dist:.8f} {'✓' if q_dist < 0.001 else '❌'}")
    print(f"  C angles:      Roll={c['roll']:7.3f}°, Pitch={c['pitch']:7.3f}°, Yaw={c['yaw']:7.3f}°")
    print(f"  Python angles: Roll={p['roll']:7.3f}°, Pitch={p['pitch']:7.3f}°, Yaw={p['yaw']:7.3f}°")
    print(f"  Angle diffs:   Roll={abs(c['roll']-p['roll']):7.3f}°, Pitch={abs(c['pitch']-p['pitch']):7.3f}°")
