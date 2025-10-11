function C = Quat2DCMat(q)
%Quat2DCMat Converts a quaternion orientation to ZYX Euler angles
%
%   C = Quat2DCMat(q)
%
%   Converts a orientation quaternion (4x1) to corresponding direction 
%   cosine matrix C (3x3) for the 3D rotation
%
    
    CPq = CPMat(q(2:4));
    C = eye(3) - 2*q(1)*CPq + 2*CPq*CPq;

end

