function q = RodMat2Quat(R)
%RodMat2Quat Converts a Rodriguez rotation matrix to quaternion orientation
%
%   q = RodMat2Quat(R)
%
%   Converts a 3x3 rotation matrix to quaternion [q0; q1; q2; q3]
%   Matches C implementation RotMtx2Quat (algo_sf_quatmath.c lines 57-102)
%

    % Matrix A from C code (lines 65-68)
    A = [0.25,  0.25,  0.25,  0.25;
         0.25, -0.25, -0.25,  0.25;
        -0.25,  0.25, -0.25,  0.25;
        -0.25, -0.25,  0.25,  0.25];

    % Vector b from rotation matrix diagonal + 1 (lines 72-75)
    b = [R(1,1); R(2,2); R(3,3); 1.0];

    % Compute quaternion components (lines 76-85)
    q = A * b;  % Matrix multiplication
    q = max(q, 0.0);  % Clamp negative values to zero
    q = sqrt(q);  % Take square root

    % Determine signs based on off-diagonal elements (lines 87-95)
    if R(2,3) < R(3,2)
        q(2) = -q(2);
    end
    if R(3,1) < R(1,3)
        q(3) = -q(3);
    end
    if R(1,2) < R(2,1)
        q(4) = -q(4);
    end

    % Normalize quaternion
    q = QuatNormal(q);

end
