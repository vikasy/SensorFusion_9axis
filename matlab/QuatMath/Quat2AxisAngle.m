function [axis, ang_rad] = Quat2AxisAngle(q)
%Quat2AxisAngle Converts a quaternion orientation to a rotation matrix
%
%   [axis, ang_rad] =  Quat2AxisAngle(q)
%   Angle is in radians
%
%   Converts a quaternion orientation vector to its components
%   in terms of a 3D vector (axis of rotation) and an angle in rad
%   as per: q = [cos(ang/2), axis*sin(ang/2)] = [q1, {q2,q3,q4}]
%
%   NOTE: Here ang_rad represent rotation of the cordinate frame, which is
%   negative of rotaiton of a vector in a cordinate frame.

    if( norm(q) == 0 )
        error('q is a null quaternion!');
    end
    q = QuatNormal(q);
    ang_rad = -2*acos(q(1));
    if( norm(q(2:4)) > 0 )
        axis = q(2:4)/norm(q(2:4));
    else
        axis = [0, 0, 0];
    end
    
end

