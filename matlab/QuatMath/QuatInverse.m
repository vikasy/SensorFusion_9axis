function qi = QuatInverse(q)
%QuatInverse Computes inverse quaternion
%
%   qi = QuatInverse(q)
%
%   q*qi = qi*q = 1.
%   => qi = qc/qmag_sq
%   where qc is conjugate quaternion and qmag_sq is square of its magnitude
%

    qmag = norm(q);
    qi = q;
    if(qmag>0)
        qi = QuatConjugate(q)./(qmag^2);
    end
    
end
