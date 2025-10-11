function pq = QuatProduct(p, q)
%QuatProduct Calculates product of to quaternions
%
%   pq = QuatProduct(p, q)
%
%   pq = [p0q0-P.Q, PxQ + p0Q + q0P] 
%   where q = [p0, P] and q = [q0, Q]
%

    p = p(:);
    q = q(:);
    if(length(q) == 3)
        q = [0; q];
    end
  
    P = p(2:4);
    Q = q(2:4);
    
    pq(1) = p(1)*q(1) - dot(P,Q);
    pq(2:4) = cross(P,Q) + p(1)*Q + q(1)*P;
    
end

