function q = SF_Orient_Acc(acc_in, frame)

% Calculate orientation matrix based on accelerometer sensor data
% 
% 


mag_grav = norm(acc_in.data);
V3 = [0; 0; 1];
V2 = [0; 1; 0];
V1 = [1; 0; 0];
mag_grav_yz = norm(acc_in.data(2:3));
alpha = 1;

if( mag_grav > 0 && mag_grav_yz > 0 )
    alpha = mag_grav/mag_grav_yz;
end

switch frame
    case 'AND'
        if( mag_grav > 0 )
            V3 = acc_in.data/mag_grav;
        end
        V2 = [0; alpha*V3(3); -alpha*V3(2)];
        V1 = [1/alpha; -alpha*V3(1)*V3(2); -alpha*V3(1)*V3(3)]; 
    %otherwise
end

RotMtx = [V1, V2, V3];
q = RodMat2Qat( RotMtx );


