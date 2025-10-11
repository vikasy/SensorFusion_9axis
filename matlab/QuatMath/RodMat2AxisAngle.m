function [axis, ang] = RodMat2AxisAngle(R)
%RodMat2AxisAngle Converts a Rodriguess rotation matrix to axis angle
%
%   [axis, ang] = RodMat2AxisAngle(R)
%

    q = RodMat2Quat( R );
    [axis, ang] = Quat2AxisAngle( q );
    
end

