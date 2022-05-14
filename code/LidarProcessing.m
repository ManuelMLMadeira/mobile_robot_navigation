function [scan_filtered, angles_filtered]  = LidarProcessing (scan)
% Convert a lidar scan into a spatial representation 
% (Assuming lateral relevance)

% Processing to space
range_max = 4000;
range_min = 10;
angles = (-120:((240)/(682-1)):120)*(pi/180);
% x = [];
% y = [];
scan_filtered=[];
angles_filtered=[];

for j = 1:682
    scan_j = scan(j);
    angle_j = angles(j);
    if scan_j < range_max && scan_j > range_min 
%         x = [x scan_j*cos(angle_j)];
%         y = [y scan_j*sin(angle_j)];
        scan_filtered=[scan_filtered scan_j];
        angles_filtered=[angles_filtered angle_j]; % em rad
    end 
end

% figure
% plot(x,y)

end 






