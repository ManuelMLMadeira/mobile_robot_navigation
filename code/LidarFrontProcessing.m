function [ door_status, robot_coord ] = LidarFrontProcessing( scan, door_width, int_reg, max_y_dist, bool_correction)
% Convert a lidar scan into a spatial representation + outputs door_status
% and the coordinates in relation to the beginning (left) of the door. 
% (Assuming frontal relevance)

% scan - Lidar scan;
% door_width - width of the scanned door;
% int_reg - 0 -> so ver parede do lado direito, 1 -> so ver parede do lado esquerdo, 2-> ve de ambos os lados
% max_y_dist - max distance to be considered on the (local) y axis
% bool_correction - corrects the position of the robot in global axis if
% true. If false, robot_coord = NaN.

 try 
     % Constants initialization
    threshold_sides = 100; % comprimento que desprezo junto ao inicio e fim da porta
    ombreira = 150; % e cerca de 80, mas dar um desconto 
    angle_threshold = 45 * pi/180;  % angulo a partir do qual a porta passa de entreaberta para aberta
    detection_threshold = 20; % variacao na derivada onde deteta
    robot_coord = NaN;

    % Processing to space
    angles = (-120:((240)/(682-1)):120)*(pi/180);
    range_max = 4000;
    range_min = 10;
    x = [];
    y = [];
    for j = 1:682
        scan_j = scan(j);
        angle_j = angles(j);
        if scan_j > range_max || scan_j < range_min 
            scan_j = range_max;
        end
        y_j = scan_j*sin(angle_j);
        x_j = scan_j*cos(angle_j);
        
        if int_reg == 0
            if y_j > -max_y_dist && y_j < door_width/2  && x_j>0
                x = [x scan_j*cos(angle_j)];
                y = [y y_j];
            end
 
        
        elseif int_reg == 1
            if y_j > -door_width/2 && y_j < max_y_dist && x_j>0
                x = [x scan_j*cos(angle_j)];
                y = [y y_j];
            end
            
        else 
            if y_j > -max_y_dist && y_j < max_y_dist && x_j>0
                x = [x scan_j*cos(angle_j)];
                y = [y y_j];
            end
        end   
    end
    
    figure
    plot (x,y,'o')
    axis ([ -200 4000 -2000 2000])
    axis 'auto y'
    
    if int_reg == 0
        peaks = [];
        k=7;
        sm_x = smooth(diff(x),k).';
        while isempty(peaks)
            peaks = find(sm_x > detection_threshold);
            detection_threshold = detection_threshold-1;    
        end
        end_index = peaks(1);
        door_end = y(end_index);
        if bool_correction
            robot_coord =  door_width + door_end;
        end
        
        x_right = x(1:end_index);
        y_right = y(1:end_index);
        
        x_door_indexes1 = find(y > y(end_index)+ threshold_sides);
        x_door_indexes2 = find(y < (y(end_index) + door_width - threshold_sides));
        x_door_indexes = intersect(x_door_indexes1, x_door_indexes2);
        x_door = x(x_door_indexes);
        y_door = y(x_door_indexes);
        
            % normalizing to a linear wall
        p_wall = polyfit(y_right,x_right,1);
        x_to_wall_right = x_right - polyval(p_wall, y_right,1);
        x_to_wall_door = x_door - polyval(p_wall, y_door,1);
       
        max_x = max(x_to_wall_door);
        
        if max_x < ombreira
            door_status = 0;
        elseif max_x < ombreira + door_width * sin(angle_threshold)
            door_status = 1;
        else 
            door_status = 2;
        end
        
        figure
        plot(x_to_wall_right,y_right,'o')
        hold on
        plot(x_to_wall_door, y_door,'o')
        axis ([ -200 4000 -2000 2000])
        axis 'auto y'
        hold on
 
             
    elseif int_reg == 1
        x = fliplr(x);
        y = fliplr(y);
        k=7;
        sm_x = smooth(diff(x),k).';
        peaks = [];
        while isempty(peaks)
            peaks = find(sm_x > detection_threshold);
            detection_threshold = detection_threshold-1;    
        end
        beg_index = peaks(1);
        door_beg = y(beg_index);
        if bool_correction
            robot_coord = door_beg;
        end
        
        x_left = x(1:beg_index);
        y_left = y(1:beg_index);
        
        x_door_indexes1 = find(y < y(beg_index) - threshold_sides);
        x_door_indexes2 = find(y > (y(beg_index) - door_width + threshold_sides));
        x_door_indexes = intersect(x_door_indexes1, x_door_indexes2);
        x_door = x(x_door_indexes);
        y_door = y(x_door_indexes);
        
        % normalizing to a linear wall
        p_wall = polyfit(y_left,x_left,1);
        x_to_wall_left = x_left - polyval(p_wall, y_left,1);
        x_to_wall_door = x_door - polyval(p_wall, y_door,1);
        
        max_x = max(x_to_wall_door);
        
        if max_x < ombreira
            door_status = 0;
        elseif max_x < ombreira + door_width * sin(angle_threshold)
            door_status = 1;
        else 
            door_status = 2;
        end
        
        figure
        plot(x_to_wall_left,y_left,'o')
        hold on
        plot(x_to_wall_door, y_door,'o')
        axis ([ -200 4000 -1000 1000])
        axis 'auto y'
        hold on


    else
        %ver pela direita
        peaks = [];
        k=7;
        sm_x = smooth(diff(x),k).';
        while isempty(peaks)
            peaks = find(sm_x > detection_threshold);
            detection_threshold = detection_threshold-1; 
        end
        end_index = peaks(1);
        door_end = y(end_index);
        
        %ver pela esquerda
        detection_threshold = 20;
        end_index = length(x)-end_index+1;
        x = fliplr(x);
        y = fliplr(y);
        k=7;
        sm_x = smooth(diff(x),k).';
        peaks = [];
        while isempty(peaks)
            peaks = find(sm_x > detection_threshold);
            detection_threshold = detection_threshold-1; 
        end
        beg_index = peaks(1);
        door_beg = y(beg_index);
        if bool_correction
            robot_coord = door_beg;
        end
        
        x_left = x(1:beg_index);
        y_left = y(1:beg_index);
        
        x_right = x(end_index:length(x));
        y_right = y(end_index:length(x));
        
        x_door_indexes1 = find(y < y(beg_index) - threshold_sides);
        x_door_indexes2 = find(y > (y(end_index) + threshold_sides));
        x_door_indexes = intersect(x_door_indexes1, x_door_indexes2);
        x_door = x(x_door_indexes);
        y_door = y(x_door_indexes);
        
        % normalizing to a linear wall
        p_wall = polyfit([y_left, y_right],[x_left, x_right],1);
        x_to_wall_left = x_left - polyval(p_wall, y_left,1);
        x_to_wall_right = x_right - polyval(p_wall, y_right,1);
        x_to_wall_door = x_door - polyval(p_wall, y_door,1);
        
        max_x = max(x_to_wall_door);
        
        if max_x < ombreira
            door_status = 0;
        elseif max_x < ombreira + door_width * sin(angle_threshold)
            door_status = 1;
        else 
            door_status = 2;
        end  
        
        
        figure
        plot(x_to_wall_left,y_left,'o')
        hold on
        plot(x_to_wall_door, y_door,'o')
        hold on 
        plot(x_to_wall_right,y_right,'o')
        axis ([ -200 4000 -2000 2000])
        axis 'auto y'
        hold on
    end
 
    
catch
    disp('Door not properly detected!')
    door_status = NaN;
 end

end

