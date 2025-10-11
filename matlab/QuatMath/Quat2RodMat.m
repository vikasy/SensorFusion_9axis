function R = Quat2RodMat(q)
%Quat2RodMat Converts a quaternion orientation to a Rodriguez orientation matrix
%
%   R = Quat2RodMat(q)
%
%   R is actually cordination frame rotation matrix.
%   such that w = Rv is same as w = q'*v*q
%   
%   Converts a quaternion orientation to a Rodriguez rotation matrix.
%

    q = QuatNormal(q);
    
    R(1,1) = 2*( q(1)^2 + q(2)^2 ) - 1;
    R(1,2) = 2*( q(2)*q(3) + q(1)*q(4) );
    R(1,3) = 2*( q(2)*q(4) - q(1)*q(3) );
    R(2,1) = 2*( q(2)*q(3) - q(1)*q(4) );
    R(2,2) = 2*( q(1)^2 + q(3)^2 ) - 1;
    R(2,3) = 2*( q(3)*q(4) + q(1)*q(2) );
    R(3,1) = 2*( q(2)*q(4) + q(1)*q(3) );
    R(3,2) = 2*( q(3)*q(4) - q(1)*q(2) );
    R(3,3) = 2*( q(1)^2 + q(4)^2 ) - 1;
    
end

