function q = SF_Orient_Mag(mag_in, frame)

% Calculate orientation matrix based on magnetometer sensor data
% 
% 

R = eye(3);
mag_mag_xy = norm(mag_in.data(1:2));

switch frame
    case 'AND'
        if( mag_mag_xy > 0 )
            R(1,1) = mag_in.data(2)/mag_mag_xy;
            R(2,2) = Rz(1,1);
            R(1,2) = mag_in.data(1)/mag_mag_xy;
            R(2,1) = -Rz(1,2);
        end
    %otherwise
end

RotMtx = R;
q = RodMat2Qat( RotMtx );
