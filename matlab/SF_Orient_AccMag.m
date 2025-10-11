function q = SF_Orient_AccMag(acc_in, mag_in, frame)

% Calculate orientation matrix based on accelerometer sensor and magnetometer sensor data
% 
% 

mag_grav = norm(acc_in.data);
mag_mag = norm(mag_in.data);

V3 = [0; 0; 1];
V2 = [0; 1; 0];
V1 = [1; 0; 0];

switch frame
    case 'AND'
        if( mag_grav > 0 )
            V3 = acc_in.data/mag_grav;
        end
        if( mag_mag > 0 );
            temp = cross(mag_in.data/mag_mag, V3);
            temp_mag = norm(temp);
            if( temp_mag > 0 )
                V1 = temp/temp_mag;
            end
        end
    %otherwise
end

temp = cross(V3, V1);
temp_mag = norm(temp);
if( temp_mag > 0 )
    V2 = temp/temp_mag;
end

RotMtx = [V1, V2, V3];
q = RodMat2Quat( RotMtx );

