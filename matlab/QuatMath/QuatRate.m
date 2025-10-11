function qdot = QuatRate(q, w)
%QuatRate Computes rate of change in quaternion wrt time
%   Input q and w are column vectors
%
%   qdot = QuatRate(q) (rad/sec)
%
%   qdot = 0.5*q*W 
%       where W is rate of rotation quaterion (angular rate)
%        W = [0; wx; wy; wz]
% 
    W = [0; w];
    
    qdot = 0.5*QuatProduct(q, W);
    
end

