function new_w = controller_orientation(current_orientation,desired_orientation)
% current_p = current (x,y,theta) 
% next_p = desired (x,y)


alpha = desired_orientation-current_orientation;


%alterar empiricamente?
K_ro = 1;
K_alpha = 0.5;

%definir novo w
w = K_ro*cos(alpha)*sin(alpha) + K_alpha*alpha;
 
 
% limitador de velocidade angular
w_limit = 30 *pi/180;
if w>w_limit
    w=w_limit;
elseif w<-w_limit
    w=-w_limit;
end

new_w = w;
end
 