function [ harm_scan,harm_angles ] = HarmScans( N,lidar )
% Harmoniza N medidas de scans do lidar.
scans = zeros(N,682);
angles = (-120:((240)/(682-1)):120)*(pi/180);
RangeMax = 4000;
RangeMin = 10;

% figure
for i = 1:N
    pause(1)
    scan = LidarScan(lidar);
    scans (i,:) = scan;
%     plot(angles,scan)
%     hold on 
end

harm_scan = [];
harm_angles = [];

for j = 1:682
    col_j = scans(:,j);
    new_col = [];
    
    for k = 1:N
       if col_j(k)< RangeMax && col_j(k)>RangeMin
           new_col = [new_col; col_j(k)];
       end
    end
    
    harm_value = harmmean(new_col); 
    if ~isnan(harm_value)
        harm_scan = [harm_scan harm_value];
        harm_angles = [harm_angles angles(j)];
    end
end 
% plot(harm_angles,harm_scan)
% legend({'1','2','3','4','final'})

end




