function SF_Main( input_fname )

% The main entry point 
% input file is the spreadsheet or text file containing the input sensor
% data


%%  Initialize the following sub-Process noise matrices (constant global):
%   QvA          : accel sensor measurement error covariance
%   QwA          : linear accel process model error covariance
%   Cacc         : linear acceleration time constant
%   QvG          : gyro sensor measurement error covaraince
%   Qwb          : zero-rate gyro offset process model error covariance
%   QvM          : mag sensor measurement error covariance
%   QwM          : mag disturbance process model error covariance
%   Cmd          : mag disturbance time constant
% 	dt           : KF update interval (sec)
%   fs_acc       : acc sampling rate
%   fs_mag       : mag sampling rate
%   fs_gyro      : gyro sampling rate
%   NUM_GYRO_SAMP:
%   QW_INIT      :
%   EPSILON      : 
    global DEG2RAD QV_INIT_AGM QW_INIT_AGM EPSILON NUM_GYRO_SAMP QW_FIXED;
    global B G2MPSECSQ Cacc Cmd dt;
 
  
%% Set up the 9axis sensor fusion algorithm
    state_var = SF_Init_State( '9X_AGM' );
    state_var.reset = 1;

%% Setup filter parameters
method = 2;
nl = 0;
fs = 1;
    
%% Open input data file

if( method == 1 )
%% METHOD 1
     %input_fname = 'Data\RawInData1.xlsx';
     Data = xlsread(input_fname);
     sensor_list = unique(Data(:,1));
     [Data_len, dim] = size(Data);
     dim = dim - 2;
     num_sensor = length(sensor_list);
     N = round(Data_len/num_sensor)-1;
     first_ts = Data(1,2);
     ts = first_ts + [0:0.2:0.2*N];
     acc_data = Data(Data(:,1)==0,2:5)';
     gyro_data = Data(Data(:,1)==1,2:5)';
     mag_data = Data(Data(:,1)==2,2:5)';
     quat_data = Data(Data(:,1)==3,2:6)';
     orient_data = Data(Data(:,1)==4,2:6)';
     dt = 0.02; %20msec

elseif( method == 2)
%% METHOD 2
     Acc_Data = xlsread(input_fname, 'Acc');
     Gyro_Data = xlsread(input_fname, 'Gyro');
     Mag_Data = xlsread(input_fname, 'Mag');
     Quat_Data = xlsread(input_fname, 'Quat');
     Orient_Data = xlsread(input_fname, 'Orient');
     sampling_rate = 100; %Hz
     sampling_interval = 1/sampling_rate; 
     dt = sampling_interval;
     N = min([size(Acc_Data,1),size(Gyro_Data,1),size(Mag_Data,1)]);
     ts = zeros(N,1);
     for i=1:N,
         ts(i) = i*sampling_interval;
     end

elseif(method == 3)
%% METHOD 3
     %[ts, gyro_input, accel_input, mag_input, quat_output, OrientAng] = generate_input();
     [ts, gyro_input, accel_input, mag_input, quat_output, OrientAng] = generate_input_noisy(1);
     N = length(ts);
     %dt = 0.01; % 10msec
     dt = ts(2) - ts(1);
     
elseif(method == 4 )
%% METHOD 4
     Acc_Data1 = xlsread(input_fname, 'Acc');
     Gyro_Data = xlsread(input_fname, 'Gyro');
     Mag_Data = xlsread(input_fname, 'Mag');
     Quat_Data = xlsread(input_fname, 'Quat');
     Orient_Data = xlsread(input_fname, 'Orient');
     sampling_rate = 25; %Hz
     sampling_interval = 1/sampling_rate; 
     dt = sampling_interval;
     ts = Gyro_Data(:,1);
     N = length(Quat_Data);
     Acc_Data = zeros(N,3);
     k = 1;
     for i=1:N,
         while( ts(i) > Acc_Data1(k,1) )
             k = k + 1;
         end
         Acc_Data(i,:) = Acc_Data1(k,:);
     end
     ts = zeros(N,1);
     for i=1:N,
         ts(i) = i*sampling_interval;
     end
     figure;
     plot(ts);
     figure;
     plot(Gyro_Data(:,2:4));
     figure;
     plot(Acc_Data(:,2:4));
     figure;
     plot(Mag_Data(:,2:4));
     figure;
     plot(Quat_Data(:,2:5));
     
elseif(method == 5)
%% METHOD 5
    Data = xlsread(input_fname, 'Input');
    dt =  4/50;
    N = size(Data,1);

elseif(method == 6)
%% METHOD 6
    ExampleScript;
    load('ExampleData.mat');
    % creates time, Gyroscope, Accelerometer, Magnetometer, quaternions,
    % euler
    % Modify Mag - swap x and y data axis
    temp = Magnetometer(:,1);
    Magnetometer(:,1) = Magnetometer(:,2);
    Magnetometer(:,2) = temp;
    N = length(time);
    dt = time(2) - time(1);
end

%% Set Noise parameters for Algorithm
if( nl == 1 )
    %% NOISELESS CASE
    QvA = 3e-12; % accel measurement noise g^2 (from RMS error of 1.4mg)
    QwA = 2e-12; % accel drift g^2
    Cacc = 0.5; % accel sensor noise time constant
    QvG = 0.03e-12;  % gyro measurement noise (deg/s)^2
    Qwb = 1e-12; % gyro offset drift (deg/s)^2
    QvM = 0.1e-12;  % mag measurement noise uT^2
    Qwd = 0.5e-12;  % mag distrubance drift uT^2
    Cmd = 0.5;  % mag disturbance noise time constant
    %dt = 100e-3; % 100 msec
    DEG2RAD = pi/180; 
    B = 50.0;  %uT
    NUM_GYRO_SAMP = 1;
    Rt = DEG2RAD*DEG2RAD*dt*dt*(QvG + Qwb);
    QV_INIT_AGM = blkdiag((QvA + QwA + Rt)*eye(3), (QvM + Qwd + B*B*Rt)*eye(3));
    Qt1 = 2000e-16;
    Qt2 = 300e-16;
    Qt3 = 1000e-12;
    Qt4 = 1000e-12;
    Qt5 = 500e-12;
    QW_INIT_AGM = blkdiag(Qt1*eye(3), Qt2*eye(3), Qt3*eye(3), Qt4*eye(3));
    QW_INIT_AGM(1:3,4:6) = Qt5*eye(3);
    QW_INIT_AGM(4:6,1:3) = QW_INIT_AGM(1:3,4:6)';
    EPSILON = 1e-6;
    QW_FIXED = blkdiag((QvG+Qwb)*dt*dt*eye(3), Qwb*eye(3), QwA*eye(3), Qwd*eye(3));
    QW_FIXED(1:3,4:6) = -dt*Qwb*eye(3);
    QW_FIXED(4:6,1:3) = QW_FIXED(1:3,4:6)';
    G2MPSECSQ = 9.8; %1g = 9.8m/s2
    
 elseif( fs == 1 )
    %% OPENSOURCE 
    QvA = 2e-6; % accel measurement noise g^2 (from RMS error of 1.4mg)
    QwA = 1e-4; % accel drift g^2
    Cacc = 0.5; % accel sensor noise time constant
    QvG = 0.01;  % gyro measurement noise (deg/s)^2
    Qwb = 1e-9; % gyro offset drift (deg/s)^2
    QvM = 0.1;  % mag measurement noise uT^2
    Qwd = 0.5;  % mag distrubance drift uT^2
    Cmd = 0.5;  % mag disturbance noise time constant
    %dt = 100e-3; % 100 msec
    DEG2RAD = pi/180; 
    B = 50.0;  %uT
    NUM_GYRO_SAMP = 1;
    Rt = DEG2RAD*DEG2RAD*dt*dt*(QvG + Qwb);
    QV_INIT_AGM = blkdiag((QvA + QwA + Rt)*eye(3), (QvM + Qwd + B*B*Rt)*eye(3));
    Qt1 = 2000e-5;
    Qt2 = 250e-3;
    Qt3 = 10e-5;
    Qt4 = 600e-3;
    Qt5 = 0;
    QW_INIT_AGM = blkdiag(Qt1*eye(3), Qt2*eye(3), Qt3*eye(3), Qt4*eye(3));
    QW_INIT_AGM(1:3,4:6) = Qt5*eye(3);
    QW_INIT_AGM(4:6,1:3) = QW_INIT_AGM(1:3,4:6)';
    EPSILON = 1e-6;
    QW_FIXED = blkdiag((QvG+Qwb)*dt*dt*eye(3), Qwb*eye(3), QwA*eye(3), Qwd*eye(3));
    QW_FIXED(1:3,4:6) = -dt*Qwb*eye(3);
    QW_FIXED(4:6,1:3) = QW_FIXED(1:3,4:6)';
    G2MPSECSQ = 9.8; %1g = 9.8m/s2 
    
elseif( fs == 2 )
    %% MODIFIED
    QvA = 3e-3; % accel measurement noise g^2 (from RMS error of 1.4mg)
    QwA = 1e-8; % accel drift g^2
    Cacc = 0.5; % accel sensor noise time constant
    QvG = 1e-1;  % gyro measurement noise (deg/s)^2
    Qwb = 1e-8; % gyro offset drift (deg/s)^2
    QvM = 10;  % mag measurement noise uT^2
    Qwd = 1;  % mag distrubance drift uT^2
    Cmd = 0.5;  % mag disturbance noise time constant
    %dt = 100e-3; % 100 msec
    DEG2RAD = pi/180; 
    B = 50.0;  %uT
    NUM_GYRO_SAMP = 1;
    Rt = DEG2RAD*DEG2RAD*dt*dt*(QvG + Qwb);
    QV_INIT_AGM = blkdiag((QvA + QwA + Rt)*eye(3), (QvM + Qwd + B*B*Rt)*eye(3));
    Qt1 = 2000e-16;
    Qt2 = 300e-16;
    Qt3 = 1000e-12;
    Qt4 = 1000e-12;
    Qt5 = 500e-12;
    QW_INIT_AGM = blkdiag(Qt1*eye(3), Qt2*eye(3), Qt3*eye(3), Qt4*eye(3));
    QW_INIT_AGM(1:3,4:6) = Qt5*eye(3);
    QW_INIT_AGM(4:6,1:3) = QW_INIT_AGM(1:3,4:6)';
    EPSILON = 1e-6;
    QW_FIXED = blkdiag((QvG+Qwb)*dt*dt*eye(3), Qwb*eye(3), QwA*eye(3), Qwd*eye(3));
    QW_FIXED(1:3,4:6) = -dt*Qwb*eye(3);
    QW_FIXED(4:6,1:3) = QW_FIXED(1:3,4:6)';
    G2MPSECSQ = 9.8; %1g = 9.8m/s2 
    
end
 
%% Setup output buffers
    Qp = zeros(N,4);
    Qc = zeros(N,4);
    Qe = zeros(N,4);
    Phi_calc = zeros(N,1);
    Theta_calc = zeros(N,1);
    Psi_calc = zeros(N,1);
    Phi_exp = zeros(N,1);
    Theta_exp = zeros(N,1);
    Psi_exp = zeros(N,1);
    
%% Run the algo on input data
     for i=1:N,
        if( method == 1 )
           %% METHOD 1
           acc_in = [];
           if( acc_data(1,i) < ts(i)+0.4 && acc_data(1,i) > ts(i)-0.4 )
               acc_in.ts = ts(i);
               acc_in.data = acc_data(2:4,i)*G2MPSECSQ;
           end
           gyro_in = [];
           if( gyro_data(1,i) < ts(i)+0.4 && gyro_data(1,1) > ts(i)-0.4 )
               gyro_in.ts = ts(i);
               gyro_in.data = gyro_data(2:4,i)/DEG2RAD;
               gyro_in.hist = gyro_in.data;
           end
           mag_in = [];
           if( mag_data(1,i) < ts(i)+0.4 && mag_data(1,i) > ts(i)-0.4 )
               mag_in.ts = ts(i);
               mag_in.data = mag_data(2:4,i);
           end 
           if( isempty(acc_in) && isempty(gyro_in) && isempty(mag_in) )
               continue;
           end

        elseif( method == 2 )
            
        %% METHOD 2
            acc_in.ts = ts(i);
            acc_in.data = Acc_Data(i,3:5)';
            gyro_in.ts = ts(i);
            gyro_in.data = Gyro_Data(i,3:5)'/DEG2RAD;
            gyro_in.hist = gyro_in.data;
            mag_in.ts = ts(i);
            mag_in.data = Mag_Data(i,3:5)';
            quat_output(:,i) = Quat_Data(i,3:6)'; 
            Phi_exp(i) = Orient_Data(i,3);
            Theta_exp(i) = Orient_Data(i,4);
            Psi_exp(i) = Orient_Data(i,5);
            
        elseif( method == 3 )
        %% METHOD 3
            acc_in.ts = ts(i);
            acc_in.data = accel_input(:,i);
            gyro_in.ts = ts(i);
            gyro_in.data = gyro_input(:,i);
            gyro_in.hist = gyro_in.data;
            mag_in.ts = ts(i);
            mag_in.data = mag_input(:,i);
            Phi_exp(i) = OrientAng(1,i);
            Theta_exp(i) = OrientAng(2,i);
            Psi_exp(i) = OrientAng(3,i);
        elseif(method == 4)
        %% METHOD 4
            acc_in.ts = ts(i);
            acc_in.data = Acc_Data(i,2:4)';
            gyro_in.ts = ts(i);
            gyro_in.data = Gyro_Data(i,2:4)'/DEG2RAD;
            gyro_in.hist = gyro_in.data;
            mag_in.ts = ts(i);
            mag_in.data = Mag_Data(i,2:4)';
            quat_output(:,i) = Quat_Data(i,2:5)';
            Phi_exp(i) = Orient_Data(i,2);
            Theta_exp(i) = Orient_Data(i,3);
            Psi_exp(i) = Orient_Data(i,4);
            
        elseif(method == 5)
        %% METHOD 5            
            if( mod(i,4) == 0 )
                ts(i/4) = Data(i,1);
                gyro_in.ts = ts(i/4);
                gyro_in.data = mean(Data(i-3:i,5:7),1)';
                gyro_in.hist = Data(i-3:i,5:7)';
                acc_in.ts = ts(i/4);
                acc_in.data = mean(Data(i-3:i,2:4),1)';
                mag_in.ts = ts(i/4);
                mag_in.data = mean(Data(i-3:i,8:10),1)';
                state_var = SF_Update_AGM(state_var, acc_in, gyro_in, mag_in);
                Qc(i/4,:) = state_var.QuatPost;
                Qp(i/4,:) = state_var.QuatPri;
                Gyro_Data_Fast(i-3:i,1:3) = Data(i-3:i,5:7);
                %Gyro_Data(i/4,1:3) = mean(Data(i-3:i,5:7));
                Gyro_Data(i/4,1:3) = Data(i,5:7);
                %Acc_Data(i/4,1:3) = mean(Data(i-3:i,2:4));
                Acc_Data(i/4,1:3) = Data(i,2:4);
                %Mag_Data(i/4,1:3) = mean(Data(i-3:i,8:10));
                Mag_Data(i/4,1:3) = Data(i,8:10);
                quat_output(i/4,1:4) = Data(i/4,11:14)';
                %fprintf('Q Pri:i=%d, q0=%f, q1=%f, q2=%f, q3=%f \n', i, Qp(i,1), Qp(i,2), Qp(i,3), Qp(i,4));
                %fprintf('Calc Q:i=%d, q0=%f, q1=%f, q2=%f, q3=%f \n', i, Qc(i,1), Qc(i,2), Qc(i,3), Qc(i,4));
                Phi_exp(i/4) = Data(i/4,15);
                Theta_exp(i/4) = Data(i/4,16);
                Psi_exp(i/4) = Data(i/4,17);
            end
        
        elseif(method == 6)
        %% METHOD 6
        % creates time, Gyroscope, Accelerometer, Magnetometer
            acc_in.ts = time(i);
            acc_in.data = Accelerometer(i,:)'*G2MPSECSQ;
            gyro_in.ts = time(i);
            gyro_in.data = Gyroscope(i,:)';
            gyro_in.hist = gyro_in.data;
            mag_in.ts = time(i);
            mag_in.data = Magnetometer(i,:)'*100; % convert Gauss to uTesla
            quat_output(:,i) = quaternion(i,:)';
            Phi_exp(i) = -euler(i,2);
            Theta_exp(i) = -euler(i,1);
            Psi_exp(i) = mod(euler(i,3),360);
        end % of various methods
        
        if(mod(i,1000)==0)
            disp(i);
        end
        
        if(method ~= 5 )
            state_var = SF_Update_AGM(state_var, acc_in, gyro_in, mag_in);
            Qc(i,:) = state_var.QuatPost;
            Qe(i,:) = quat_output(:,i)';
            Qp(i,:) = state_var.QuatPri;
            fprintf('Q Pri:i=%d, q0=%f, q1=%f, q2=%f, q3=%f \n', i, Qp(i,1), Qp(i,2), Qp(i,3), Qp(i,4));
            fprintf('Calc Q:i=%d, q0=%f, q1=%f, q2=%f, q3=%f \n', i, Qc(i,1), Qc(i,2), Qc(i,3), Qc(i,4));
            fprintf('Exp Q:i=%d, q0=%f, q1=%f, q2=%f, q3=%f \n', i, Qe(i,1), Qe(i,2), Qe(i,3), Qe(i,4));
            Phi_calc(i) = state_var.PhiPost;
            Theta_calc(i) = state_var.ThetaPost;
            Psi_calc(i) = state_var.PsiPost;
        end        
     end % of running algo for i=1:N,
     
     
%% Output Analysis

     if(method == 5 )
         figure;
         plot(Gyro_Data_Fast(:,1:3));
         figure;
         plot(Gyro_Data(:,1:3));
         figure;
         plot(Acc_Data(:,1:3));
         figure;
         plot(Mag_Data(:,1:3));
     end
     
     figure;
     plot(Qc(:,1),'b');
     hold on;
     plot(Qc(:,2),'r');
     plot(Qc(:,3),'g');
     plot(Qc(:,4),'k');
     title('Calculated Q');
     xlabel('Time (s)');
     ylabel('Quaternion');
     legend('q0', 'q1', 'q2', 'q3');
     figure;
     plot(Qe(:,1),'b');
     hold on;
     plot(Qe(:,2),'r');
     plot(Qe(:,3),'g');
     plot(Qe(:,4),'k');
     title('Expected Q');
     xlabel('Time (s)');
     ylabel('Quaternion');
     legend('q0', 'q1', 'q2', 'q3');
     figure;
     plot(Qc(:,1) - Qe(:,1),'b');
     hold on;
     plot(Qc(:,2) - Qe(:,2),'r');
     plot(Qc(:,3) - Qe(:,3),'g');
     plot(Qc(:,4) - Qe(:,4),'k');
     title('Error in Q');
     xlabel('Time (s)');
     ylabel('Quaternion diff');
     legend('q0_err', 'q1_err', 'q2_err', 'q3_err');
     
     figure;
     plot(Phi_calc,'b');
     hold on;
     plot(Theta_calc,'r');
     plot(Psi_calc,'g');
     xlabel('Time (s)');
     ylabel('Angle (deg)');
     legend('\phi', '\theta', '\psi');
     title('Calculated Orientaton Angles');
     
     figure;
     plot(Phi_calc - Phi_exp,'b');
     hold on;
     plot(Theta_calc - Theta_exp,'r');
     plot(Psi_calc - Psi_exp,'g');
     xlabel('Time (s)');
     ylabel('Angle error (deg)');
     legend('\phi_err', '\theta_err', '\psi_err');
     title('Error in Orientaton Angles');
