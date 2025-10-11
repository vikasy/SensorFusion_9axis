function w = QuatRotVec(q, v)
%QuatRotVec Rotates a vector 'in opposite direction' by a quaternion
% It basically rotates the cordinate frame and computes representation
% of v in the rotated frame.
%
%   w = QuatRot(q, v) = q'*V*q
%
%   where q = [cos(ang/2), axis*sin(ang/2)]
%   and V = [0; v]
%   Rotates the 3D column vector v in opposite direction by ang
%   about 'axis'
%
%   q and v should be column vectors

    if(( size(v,2) ~= 1 ) || (size(q,2) ~= 1))
        error('q and v should be column vectors');
    end
    if( norm(q) == 0 )
        error('q is a null quaternion!');
    end
    
    q = QuatNormal(q);
    qrot = QuatProduct(QuatProduct(QuatConjugate(q), [0; v]), q);
    w = qrot(2:4);

end

