function T = get_T( x, y, theta)
% Build transformation matrix upon x,y and theta.

T= zeros(4,4);
T (1:2,1:2) = [cos(theta), -sin(theta); sin(theta), cos(theta)];
T(3,3) = 1;
T(4,4) = 1;
T(1,4) = x;
T(2,4) = y;

end

