function [ y_robot, angle_robot ] = LidarCorrection( scan_filtered, angles_filtered, int)
% int = 0 => medicao feita a direita 
% int = 1 => medicao feita a esquerda

   % Required constants:
corridor_width = 1650;
angle_threshold = pi/4;

    % mede a esquerda
    if int   
        left_side = abs(angles_filtered-pi/2) < angle_threshold;
        angles_left_wall = angles_filtered(left_side);
        scan_left_wall = scan_filtered(left_side);
        len = length (scan_left_wall);

        x = [];
        y = [];
        for j = 1:len
            scan_j = scan_left_wall(j);
            angle_j = angles_left_wall(j);
            x = [x scan_j*cos(angle_j)];
            y = [y scan_j*sin(angle_j)];
        end
       

        p_wall = polyfit(x,y,1);
        y_predicted = polyval(p_wall,x);
        y_0 = polyval(p_wall,0); 
        y_robot = corridor_width/2 - y_0;
        angle_robot =  - atan (p_wall(1));


    % mede a direita
    else
        right_side = abs(angles_filtered+pi/2) < angle_threshold;
        angles_right_wall = angles_filtered(right_side);
        scan_right_wall = scan_filtered(right_side);
        len = length (scan_right_wall);
        x = [];
        y = [];

        for j = 1:len
            scan_j = scan_right_wall(j);
            angle_j = angles_right_wall(j);
            x = [x scan_j*cos(angle_j)];
            y = [y scan_j*sin(angle_j)];
        end
        

        p_wall = polyfit(x,y,1);
        y_predicted = polyval(p_wall,x);      
        y_0 = polyval(p_wall,0); 
        y_robot = -corridor_width/2 + abs(y_0);
        angle_robot = - atan (p_wall(1));
    end

    
end

