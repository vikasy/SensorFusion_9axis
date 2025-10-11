function deg_out = Rad2Deg( rad_in )
% convert angle in radian to angle in degree
% input: angle in radian
% output: angle in degree 
% if rad_in >= 0, then 0 <= deg_out < 360
% if rad_in < 0, then -360 < deg_out <= 0

deg_out = 180*(rad_in/pi);
if( deg_out >= 0 )
    deg_out = mod( deg_out, 360 );
else
    deg_out = mod( deg_out, -360 );
end


