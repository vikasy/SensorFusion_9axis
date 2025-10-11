function CP = CPMat( v )
%CPMAT Creates 3x3 cross product matrix for a vector cross product
%
%  if v and w are (3x1) vectos, then there cross product:
%  v x w = [v x] w
%  where [v x] is a (3x3) matrix given by:
%     [v x] = [  0  -v(3)  v(2);
%               v(3)   0  -v(1);
%              -v(2) v(1)     0;]
%
%  
%  Note that [v x] = -[v x]'
%            [a x] + [b x] = [(a+b) x] ( axc+bxc = (a+b)xc) )
%
%            
    CP = [  0  -v(3)  v(2);
          v(3)     0 -v(1);
         -v(2)  v(1)     0;];
 
end

