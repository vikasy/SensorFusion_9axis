function R = EulerAng2RodMat(phi, theta, psi)
%EulerAng2RodMat Converts a ZYX Euler angle (Android) orientation to a rotation matrix
%
%   R = EulerAng2RodMat(phi, theta, psi)
%
%   Angles are in radians
%
%   Converts a rotation matrix orientation to ZYX Euler angle (Android) where phi is
%   a rotation around X, theta around Y and psi around Z.
%
% 
% For Android cordinate system, rotation matrix can be obtained as follows:
% R = Rx(theta)Ry(phi)Rz(psi)
% where theta is pitch angle about x axis, phi is roll angle about y axis, 
% and psi is heading angle about z axis. Rx is rotation matrix about x axis, 
% Ry is rotation matrix about y axis, and Rz is rotation matrix about z axis
% 
% Rx(theta) = [1    0    0;
%              0    ct  -st;
%              0    st   ct]
% ct=cos(theta), st=sin(theta)
% 
% Ry(phi) = [cp   0  -sp;
%            0    1   0;
%            sp   0   cp]
% cp=cos(phi), sp=sin(phi)
% 
% Rz(psi) = [cs  -ss   0;
%            ss   cs   0;
%            0     0   1]
% cs=cos(psi), ss=sin(psi)                           
% 
% R = Rx(theta)Ry(phi)Rz(psi)
% => R = [R11 R12 R13;
%         R21 R22 R23;
%         R31 R32 R33];
%      = [ cp*cs             -cp*ss           sp;
%          ct*ss+cs*sp*st cs*ct-sp*ss*st  -cp*st;
%         -cs*ct*sp+ss*st ct*sp*ss+cs*st   cp*ct]

    cs = cos(psi);
    ss = sin(psi);
    cp = cos(phi);
    sp = sin(phi);
    ct = cos(theta);
    st = sin(theta);

    R = zeros(3,3);
    R(1,1) = cp*cs;
    R(1,2) = -cp*ss;
    R(1,3) = sp;
    R(2,1) = ct*ss+cs*sp*st;
    R(2,2) = cs*ct-sp*ss*st;
    R(2,3) = -cp*st;
    R(3,1) = -cs*ct*sp+ss*st;
    R(3,2) = ct*sp*ss+cs*st;
    R(3,3) = cp*ct;
    
end

