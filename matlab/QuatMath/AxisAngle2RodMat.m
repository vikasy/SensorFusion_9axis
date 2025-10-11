function R = AxisAngle2RodMat(axis, ang_rad)
%AxisAngle2RodMat Converts an axis-angle orientation to a rotation matrix
%
%   R = AxisAngle2RodMat(axis, ang_rad)
%   Angle is in radians
%
%   Converts and axis-angle orientation to a Rodregues rotation matrix 
%   axis = n
%   angle = a
%   v is before rotation and w is after rotation by angle a about axis n, 
%   such that w = Rv, then as per Rodrigues formula:
%   w = v + (sin(a))(nxv) + (1 - cos(a))(nx(nxv))
%   i.e. w = [I + (sin(a))N + (1 - cos(a))N^2] v = Rv
%

    if( norm(axis) > 0 )
        axis = axis/norm(axis);
    end
    N = CPMat(axis);
    R = eye(3) + (sin(ang_rad)*N + (1 - cos(ang_rad)))*N;

end

