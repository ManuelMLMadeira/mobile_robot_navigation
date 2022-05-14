
function control_nl = controller_translation(current_p,next_p)
% current_p = current (x,y,theta) 
% next_p = desired (x,y)


desired_next_pos = next_p; %updatar com os valores sucessivos do path
x = current_p(1);
y = current_p(2);
t = current_p(3);

ro = sqrt((desired_next_pos(1) - x)^2 + (desired_next_pos(2) - y)^2);
beta = atan2(desired_next_pos(2)-y,desired_next_pos(1) - x);
alpha = beta-t;

%alterar empiricamente?
K_ro = 1;
K_alpha = 0.5;

%definir novos v,w
v = K_ro*ro*cos(alpha);
w = K_ro*cos(alpha)*sin(alpha) + K_alpha*alpha;
 
%limitador de velocidade linear
v_limit = 250;
if v>v_limit
    v=v_limit;
elseif v<-v_limit
    v=-v_limit;
end
 
% limitador de velocidade angular
% w_limit = 40 *pi/180;
% if w>w_limit
%     w=w_limit;
% elseif w<-w_limit
%     w=-w_limit;
% end

control_nl = [v,w];
end
 