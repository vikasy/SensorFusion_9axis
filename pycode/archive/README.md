# Archived Development Scripts

This directory contains debug, analysis, and experimental scripts used during development and optimization of the sensor fusion implementation. These files are preserved for historical reference but are not part of the production codebase.

## Directory Structure

- **debug_scripts/**: Debugging and experimental test scripts
- **analysis_scripts/**: Accuracy analysis and comparison scripts
- **matlab_port/**: Original Matlab-to-Python port files (superseded by optimized implementation)

## Production Files (in parent pycode/ directory)

### Core Implementation
- `sensor_fusion_6axis.py` - 6-axis (accel + gyro) EKF sensor fusion
- `sensor_fusion_9axis.py` - 9-axis (accel + gyro + mag) EKF sensor fusion with magnetometer
- `sensor_platform_config.py` - Sensor platform configurations (Invensense, Freescale)
- `example_usage.py` - Simple usage examples
- `sensor_fusion_main.py` - Main entry point

### Testing & Validation (Production Quality)
- `test_all_synthetic_datasets.py` - Comprehensive validation on 10 synthetic datasets
- `test_all_simple_rotations.py` - Simple rotation validation tests
- `regression_test.py` - Automated regression testing with baseline comparison
- `regression_baseline.json` - Baseline metrics (Phase 1 optimization results)

### Optimization Tools
- `optimize_bias_parameters_focused.py` - Phase 1 grid search optimization
- `parameter_optimization_results_focused.json` - Optimization results

### Utilities
- `plot_all_synthetic_datasets.py` - Visualization of test results
- `generate_python_reference.py` - Generate reference quaternion outputs
- `generate_python_freescale.py` - Generate Freescale platform outputs

## Optimization History

The production implementation includes:

**Iteration 1**: Motion-adaptive bias rate limiting (ENABLED)
- Prevents tracking motion as gyro bias during rotation
- Three-tier threshold system based on rotation magnitude

**Phase 1**: Grid search parameter optimization (ENABLED)
- Optimized thresholds: slow=12.0 dps, fast=45.0 dps
- Optimized rates: moderate=0.07, fast=0.015
- Results: -13.4% error, -44.1% bias on complex motion

**Iteration 2**: Adaptive process noise scaling (DISABLED)
- Investigated but found to cause regressions
- Implementation present but disabled in code

See documentation:
- `ITERATION1_RESULTS.md`
- `PHASE1_OPTIMIZATION_RESULTS.md`
- `ITERATION2_RESULTS.md`
- `IMPROVEMENT_ROADMAP.md`
