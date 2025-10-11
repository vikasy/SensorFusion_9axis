function q_rotated = QuatRotate(q, ang_deg)

% Rotate given quaternion by provided angle
% Angle is in degree
%

    ang_mag = norm(ang_deg);
    
    EPSILON = 1e-6;
    
    if( ang_mag > EPSILON )
        rot_axis_unit = ang_deg/ang_mag;
        rot_ang = Deg2Rad(ang_mag);
        rot_q = AxisAngle2Quat(rot_axis_unit, rot_ang);
    else
        rot_q = [1; 0.5*ang_deg];
    end
    
    rot_q = QuatNormal(rot_q);
    q_rotated = QuatProduct(q, rot_q);
    q_rotated = QuatNormal(q_rotated);
    