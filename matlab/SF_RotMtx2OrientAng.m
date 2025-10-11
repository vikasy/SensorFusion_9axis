function [theta, phi, psi, rho, chi] = SF_RotMtx2OrientAng( RotMtx, theta_prev, psi_prev, frame )
% Update the orientation angles, compass heading, and tilt angles (in Deg)
% based on the updated rotation matrix
% Input: Rotation Matrix, cordinate frame, prev theta and psi angles
% Output: -90 <= phi <= 90 
%        -180 <= theta < 180
%         0 <= psi, rho, tilt < 360
%
% Use prev values of theta and psi for resolving Gimbal lock condition
%

persistent flag;
if( isempty(flag) )
    flag = 0;
end

switch frame
    case 'AND',        
        phi = asind( RotMtx(1,3) ); % roll angle [-90,90]
        theta = atan2d( -RotMtx(2,3), RotMtx(3,3) ); % pitch angle (-180,180)
        if( theta < -179 )
            theta = 180;
        end
        psi = atan2d( -RotMtx(1,2), RotMtx(1,1) ); % yaw angle [0, 360)  
        if(psi < 0 )
            psi = psi + 360;
        end
        if(psi > 359.55 )
            psi = 0;
        end
        % Gimbal Lock resolution at roll = 90 or -90 (+/-2)
        if( phi > 88 )
            sum = atan2d( RotMtx(2,1), RotMtx(2,2) ); % psi+theta           
            if( flag == 1 )
                theta = theta_prev;              
                psi = sum - theta_prev;
                psi = mod( psi, 360 );
                flag = 0;
            else
                psi = psi_prev;
                theta = sum - psi_prev;
                theta = mod(theta+180, 360) - 180;
                flag = 1;
            end
        elseif ( phi < -88 )
            diff = atan2d( RotMtx(2,1), RotMtx(2,2) ); % psi-theta
            if( flag == 1 )
                theta = theta_prev;
                psi = theta_prev + diff;
                psi = mod( psi, 360 );
                flag = 0;
            else
                psi = psi_prev;
                theta = psi_prev - diff;
                theta = mod(theta+180, 360) - 180;
                flag = 1;
            end
        end        
end
rho = psi;
chi = acosd( RotMtx(3,3) );

