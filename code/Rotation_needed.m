function angle_diff = Rotation_needed (curr_loc, desired_loc)

temp_diff = desired_loc -curr_loc;
while abs(temp_diff)>360
    if  temp_diff > 360
        temp_diff = temp_diff - 360;
    else 
        temp_diff = temp_diff + 360;
    end
  
end

if temp_diff > 180
    temp_diff = -(360 - temp_diff);
elseif temp_diff <-180
    temp_diff = 360 + temp_diff;   
end

angle_diff = temp_diff;
    
end