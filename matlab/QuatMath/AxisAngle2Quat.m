function q = AxisAngle2Quat(axis, ang_rad)
%AxisAngle2Quat Converts an axis-angle orientation to a rotation quaternion
%
%   q = AxisAngle2Quat(axis, ang_rad)
%   angle in radians
%
%   Converts an axis angle orientation pair to a quaternion rotation
%   NOTE: This is cordinate frame rotation (opposite of vector rotation)
%

    if( norm(axis) > 0 )
        axis = axis/norm(axis);
    end
    q0 = cos(-ang_rad/2);
    q1 = -axis(1)*sin(-ang_rad/2);
    q2 = -axis(2)*sin(-ang_rad/2);
    q3 = -axis(3)*sin(-ang_rad/2);
    q = [q0, q1, q2, q3]';
    q = QuatNormal( q );
    
end

