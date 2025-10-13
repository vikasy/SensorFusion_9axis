function Generate_6Axis_Reference_Freescale()
% Generate reference quaternion output from 6-axis MATLAB implementation
% Using FREESCALE sensor specifications (FXOS8700CQ + FXAS21000)
% for comparison with INVENSENSE

global DEG2RAD QW_FIXED G2MPSECSQ EPSILON NUM_GYRO_SAMP;
global Cacc dt;

% Add QuatMath directory to path
addpath(fullfile(fileparts(mfilename('fullpath')), 'QuatMath'));

% Initialize 6-axis sensor fusion
state_var = SF_Init_State('6X_AG');
state_var.reset = 1;

% Set noise parameters (same as INVENSENSE for fair comparison)
QvA = 2e-6;
QwA = 1e-4;
Cacc = 0.5;
QvG = 0.01;
Qwb = 1e-9;
DEG2RAD = pi/180;
G2MPSECSQ = 9.80665;
NUM_GYRO_SAMP = 4;  % SF_OVERSAMPLE_RATIO
dt = 0.04;  % 4 * 0.01 = 0.04 seconds
EPSILON = 1e-12;

Rt = DEG2RAD*DEG2RAD*dt*dt*(QvG + Qwb);
state_var.MeasNoiseVar = (QvA + QwA + Rt)*eye(3);

QW_FIXED = zeros(6,6);
QW_FIXED(1:3,1:3) = (QvG+Qwb)*dt*dt*eye(3);
QW_FIXED(4:6,4:6) = Qwb*eye(3);
QW_FIXED(1:3,4:6) = -dt*Qwb*eye(3);
QW_FIXED(4:6,1:3) = QW_FIXED(1:3,4:6)';

state_var.ProcNoiseVar = QW_FIXED;

% FREESCALE Sensor Specifications
% FXOS8700CQ: 14-bit accelerometer, ±4g range
FSL_ACCEL_RANGE_G = 4.0;
FSL_ACCEL_MAX_COUNT = 8191;  % 14-bit: 2^13 - 1
FSL_ACC_SCALE = FSL_ACCEL_RANGE_G / FSL_ACCEL_MAX_COUNT;

% FXAS21000: 16-bit gyroscope, ±1000 dps range
FSL_GYRO_RANGE_DPS = 1000.0;
FSL_GYRO_MAX_COUNT = 32767;  % 16-bit
FSL_GYRO_SCALE = FSL_GYRO_RANGE_DPS / FSL_GYRO_MAX_COUNT;

fprintf('=================================================================\n');
fprintf('FREESCALE Sensor Specifications:\n');
fprintf('  Accelerometer (FXOS8700CQ): 14-bit, ±%.1fg, scale=%.10f g/count\n', ...
    FSL_ACCEL_RANGE_G, FSL_ACC_SCALE);
fprintf('  Gyroscope (FXAS21000): 16-bit, ±%.1fdps, scale=%.10f dps/count\n', ...
    FSL_GYRO_RANGE_DPS, FSL_GYRO_SCALE);
fprintf('=================================================================\n\n');

% Load test data from header file
fprintf('Parsing test data...\n');
fid = fopen('../test/data/testdata/fusion/test_input_output_0922.h', 'r');
if fid == -1
    error('Cannot open test data file');
end

% Read entire file
file_content = fread(fid, '*char')';
fclose(fid);

% Parse sensor data using regex
pattern = '\{(\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(\d+)ULL\}';
matches = regexp(file_content, pattern, 'tokens');

sensor_data = [];
for i = 1:length(matches)
    sensor_data(i).id = str2double(matches{i}{1});
    sensor_data(i).x = str2double(matches{i}{2});
    sensor_data(i).y = str2double(matches{i}{3});
    sensor_data(i).z = str2double(matches{i}{4});
    sensor_data(i).ts = str2double(matches{i}{5});
end

fprintf('Loaded %d sensor samples\n', length(sensor_data));

% Buffers for oversampling
acc_buffer = zeros(3, NUM_GYRO_SAMP);
gyro_buffer = zeros(3, NUM_GYRO_SAMP);
acc_count = 0;
gyro_count = 0;
acc_ts = 0;
gyro_ts = 0;

% Output file
out_fid = fopen('matlab_freescale_quaternions_0922.csv', 'w');
fprintf(out_fid, 'sample,timestamp,sensor_id,q0,q1,q2,q3\n');

fusion_count = 0;

fprintf('Processing samples with FREESCALE specs...\n');

for i = 1:length(sensor_data)
    sensor_id = sensor_data(i).id;

    % Skip magnetometer (id=2)
    if sensor_id == 2
        continue;
    end

    sensor_counts = [sensor_data(i).x; sensor_data(i).y; sensor_data(i).z];
    timestamp = sensor_data(i).ts;

    if sensor_id == 0  % Accelerometer
        acc_count = acc_count + 1;
        acc_buffer(:, acc_count) = sensor_counts;
        if acc_count == 1
            acc_ts = timestamp;
        end

        if acc_count == NUM_GYRO_SAMP
            % Buffer full - ready to process
            acc_count = 0;
            acc_ready = 1;
        else
            acc_ready = 0;
        end

    elseif sensor_id == 1  % Gyroscope
        gyro_count = gyro_count + 1;
        gyro_buffer(:, gyro_count) = sensor_counts;
        if gyro_count == 1
            gyro_ts = timestamp;
        end

        if gyro_count == NUM_GYRO_SAMP
            % Buffer full - ready to process
            gyro_count = 0;
            gyro_ready = 1;
        else
            gyro_ready = 0;
        end
    end

    % Run fusion when both sensors ready
    if exist('acc_ready', 'var') && exist('gyro_ready', 'var') && acc_ready && gyro_ready
        acc_ready = 0;
        gyro_ready = 0;

        % Prepare sensor inputs using FREESCALE scale factors
        acc_in.ts = acc_ts;
        acc_in.data = mean(acc_buffer, 2) * FSL_ACC_SCALE * G2MPSECSQ;

        gyro_in.ts = gyro_ts;
        gyro_in.data = mean(gyro_buffer, 2) * FSL_GYRO_SCALE;
        gyro_in.hist = gyro_buffer * FSL_GYRO_SCALE;

        % Run fusion
        prev_orient_init = state_var.orient_init;
        prev_reset = state_var.reset;
        state_var = SF_Update_6X_AG(state_var, acc_in, gyro_in);

        % Only write output if not in reset or initialization
        if prev_reset == 0 && prev_orient_init == 1
            % Write output
            fprintf(out_fid, '%d,%d,%d,%.15f,%.15f,%.15f,%.15f\n', ...
                fusion_count, timestamp, sensor_id, ...
                state_var.QuatPost(1), state_var.QuatPost(2), ...
                state_var.QuatPost(3), state_var.QuatPost(4));
            fusion_count = fusion_count + 1;
        end

        if mod(fusion_count, 100) == 0
            fprintf('  Fusion runs: %d...\n', fusion_count);
        end
    end
end

fclose(out_fid);

fprintf('\nComplete! Generated %d fusion outputs\n', fusion_count);
fprintf('Output saved to: matlab_freescale_quaternions_0922.csv\n');

end
