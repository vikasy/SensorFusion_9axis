function qc = QuatConjugate(q)
%QuatConjugate Converts a quaternion to its conjugate
%
%   qc = QuatConjugate(q)
%
%   Converts a quaternion to its conjugate.
%

    qc(1) = q(1);
    qc(2:4) = -q(2:4);
    
end
