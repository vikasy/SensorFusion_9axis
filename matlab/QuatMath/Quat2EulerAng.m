function [phi, theta, psi, gimbal] = Quat2EulerAng(q)
%Quat2EulerAng Converts a quaternion orientation to ZYX Euler angles
%
%   [phi, theta, psi, gimbal] = Quat2EulerAng(q)
%
%   Use Quat2RodMat and then RodMat2EulerAng to get the answer
%

    C = Quat2RodMat( q );
    %disp(C)
    [phi, theta, psi, gimbal] = RodMat2EulerAng(C);
    %disp([phi, theta, psi, gimbal]);
    
end

