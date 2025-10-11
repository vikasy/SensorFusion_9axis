function q = SF_Orient_Mag_Mod(mag_in, frame)

% Calculate orientation matrix based on magnetometer sensor data
% 
% In this modified algorithm, only assumption is Roll angle is 0
% unlike other algo where both roll and pith are assumed to be 0.
% 

Rx = eye(3);
Ry = eye(3);
Rz = eye(3);
mag_mag_xy = norm(mag_in.data(1:2));
mag_mag_yz_sq = norm(mag_in.data(2:3))^2;

switch frame
    case 'AND'
        if( mag_mag_xy > 0 )
            Rz(1,1) = mag_in.data(2)/mag_mag_xy;
            Rz(2,2) = Rz(1,1);
            Rz(1,2) = mag_in.data(1)/mag_mag_xy;
            Rz(2,1) = -Rz(1,2);
        end
        if( mag_mag_yz_sq > 0 )
            Rx(2,2) = (mag_in.data(2)^2 - mag_in.data(3)^2)/mag_mag_yz_sq;
            Rx(3,3) = Rx(2,2);
            Rx(2,3) = 2*mag_in.data(2)*mag_in.data(3)/mag_mag_yz_sq;
            Rx(3,2) = -Rx(2,3);
        end
    %otherwise
end

RotMtx = Rx*Ry*Rz;
q = RodMat2Qat( RotMtx );
