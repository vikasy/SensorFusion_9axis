function qnorm = QuatNormal(q)
%QuatNormalize Normalize a quaternion so that its mag = 1
%
%   qnorm = QuatNormal(q)
%   qnorm = q/qmag
%
%   If q(0) is negative, it makes it positive without changing the
%   functionality of quaternion

    qmag = norm(q);
    qnorm = [1; 0; 0; 0];
    if( qmag>0 )
        qnorm = q./qmag;
    end
    if( qnorm(1) < 0 )
        qnorm = -qnorm;
    end
    
end
