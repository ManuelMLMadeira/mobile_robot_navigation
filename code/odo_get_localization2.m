function [n_position] = odo_get_localization2 (correction_matrix)
%corrects odometry measurements according to correction matrix

temp_pos = pioneer_read_odometry();
temp_pos(3) = MinRad2(OdoToRad(temp_pos(3)));

T1 = get_T (temp_pos(1), temp_pos(2), temp_pos(3));
T2 = correction_matrix*T1;

figure(1)
axis([0 20000 0 20000])
plot(temp_pos(1),temp_pos(2),'o','MarkerSize',10,'MarkerFaceColor',[0 1 0])
hold on

theta = acos(T2(1,1));
if T2(1,2)>0
    theta = - theta;
end
theta = MinRad2(theta);

x = T2(1,4);
y = T2(2,4);

plot(x,y,'o','MarkerSize',10,'MarkerFaceColor',[1 0 0])
hold on
drawnow

n_position = [x y theta];
end