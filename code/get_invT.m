function invT = get_invT( x, y, theta)
% Build transformation matrix upon x,y and theta.

invT= zeros(4,4);
invT (1:2,1:2) = [cos(theta), sin(theta); -sin(theta), cos(theta)];
invT(3,3) = 1;
invT(4,4) = 1;
invT(1:2,4) = -invT(1:2,1:2)*[x; y];

end
