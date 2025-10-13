"""Find where C and Python outputs start to diverge"""
import csv
import numpy as np

def load_timestamps(filename):
    """Load just timestamps and quaternions"""
    data = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'ts': int(row['timestamp']),
                'q': np.array([float(row['q0']), float(row['q1']), float(row['q2']), float(row['q3'])])
            })
    return data

def quat_distance(q1, q2):
    """Quaternion distance: 1 - |q1 · q2|"""
    dot = np.abs(np.dot(q1, q2))
    return 1.0 - dot

print("Loading outputs...")
c_data = load_timestamps('c_outputs_0922.csv')
py_data = load_timestamps('python_outputs_0922.csv')

print(f"C outputs: {len(c_data)}")
print(f"Python outputs: {len(py_data)}")

# Create timestamp sets
c_timestamps = set(d['ts'] for d in c_data)
py_timestamps = set(d['ts'] for d in py_data)

# Find missing timestamps
missing_in_py = sorted(c_timestamps - py_timestamps)
extra_in_py = sorted(py_timestamps - c_timestamps)

print(f"\n{'='*80}")
print("TIMESTAMP ANALYSIS")
print(f"{'='*80}")
print(f"Timestamps in C but not Python: {len(missing_in_py)}")
print(f"Timestamps in Python but not C: {len(extra_in_py)}")

if missing_in_py:
    print(f"\nFirst 10 missing timestamps in Python:")
    for ts in missing_in_py[:10]:
        c_idx = next(i for i, d in enumerate(c_data) if d['ts'] == ts)
        print(f"  ts={ts}, C index={c_idx}")

    print(f"\nLast 10 missing timestamps in Python:")
    for ts in missing_in_py[-10:]:
        c_idx = next(i for i, d in enumerate(c_data) if d['ts'] == ts)
        print(f"  ts={ts}, C index={c_idx}")

# Check matched samples for divergence
print(f"\n{'='*80}")
print("MATCHED SAMPLES - QUATERNION DIVERGENCE ANALYSIS")
print(f"{'='*80}")

divergences = []
for i, c_row in enumerate(c_data):
    py_row = next((p for p in py_data if p['ts'] == c_row['ts']), None)
    if py_row:
        qd = quat_distance(c_row['q'], py_row['q'])
        if qd > 0.001:  # Significant divergence
            divergences.append({
                'c_idx': i,
                'ts': c_row['ts'],
                'dist': qd,
                'c_q': c_row['q'],
                'py_q': py_row['q']
            })

if divergences:
    print(f"Found {len(divergences)} samples with quat distance > 0.001")
    print(f"\nFirst divergence:")
    d = divergences[0]
    print(f"  C index: {d['c_idx']}, timestamp: {d['ts']}")
    print(f"  Quat distance: {d['dist']:.8f}")
    print(f"  C quat:  {d['c_q']}")
    print(f"  Py quat: {d['py_q']}")
else:
    print("All matched samples have quat distance < 0.001 ✓")

# Check if Python outputs stop at a certain point
if py_data:
    last_py_ts = py_data[-1]['ts']
    last_py_idx = next(i for i, d in enumerate(c_data) if d['ts'] == last_py_ts)
    print(f"\n{'='*80}")
    print("PYTHON OUTPUT RANGE")
    print(f"{'='*80}")
    print(f"Last Python timestamp: {last_py_ts}")
    print(f"Corresponds to C index: {last_py_idx}/{len(c_data)-1}")
    print(f"Python generated first {last_py_idx+1} outputs, then stopped or diverged")
