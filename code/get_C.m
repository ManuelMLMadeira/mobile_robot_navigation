function [C] = get_C(x_curr,y_curr,theta_curr)
% Build matrix that allows the transformation from odometry coordinates to
% the "real" coordinates. It is intended to be updated in every lidar scan.

odo_coordinates = pioneer_read_odometry();
x_odo = odo_coordinates(1);
y_odo = odo_coordinates(2);
theta_odo = MinRad(OdoToRad(odo_coordinates(3)));

T1_inv = get_invT(x_odo, y_odo, theta_odo);
T2 = get_T(x_curr, y_curr, theta_curr);
C = T2*T1_inv;

end

