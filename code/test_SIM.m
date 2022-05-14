%%Definicao do path (point tracking)

path = [1.00    1.00  0;
        1.00    16.00 sym(pi)/2;
        14.8    16.00 0;
        14.80   1.00  -sym(pi)/2;
        1.00    1.00  -sym(pi)];
    
path = double(path);    
robotCurrentLocation = path(1:2);%admitindo o robot no primeiro ponto do path
initialOrientation = path(1,3);

robotCurrentPose = [robotCurrentLocation initialOrientation];

robotRadius = 0.16;
robot = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);

plot(path(:,1), path(:,2),'k--d')
xlim([0 20])
ylim([0 20])


 %% seguir trajetoria
 v_profile = [];
 w_profile = [];
 goalRadius = 0.05;
 
 goalOrientation = 0.05;


controlRate = robotics.Rate(10);
for i = 2:length(path)

    desired_position = path(i,1:2);
    distanceToGoal = norm(robotCurrentPose(1:2) - desired_position);
    
    desired_orientation = path(i,3);
    
    rotation_necessary = robotCurrentPose(1,3) - desired_orientation;
    
    while (rotation_necessary > goalOrientation||rotation_necessary < -goalOrientation)
        if(distanceToGoal <= goalRadius)
            break
        end

        u  = controller_nl(robotCurrentPose,desired_position);
        v = u(1);
        v_profile = [v_profile, v];
        omega = u(2);
        w_profile = [w_profile, omega];
        %disp (u)
        % Simulate the robot using the controller outputs.
        drive(robot, 0, omega);
    
        % Extract current location information ([X,Y]) from the current pose of the
        % robot
        robotCurrentPose = robot.getRobotPose;

        % Re-compute the distance to the goal
        rotation_necessary = robotCurrentPose(1,3) - desired_orientation;
        distanceToGoal = norm(robotCurrentPose(1:2) - desired_position);

        % Wait for control rate to ensure 10 Hz rate.
        waitfor(controlRate);
    end
    
    while( distanceToGoal > goalRadius )
    % Compute the controller outputs, i.e., the inputs to the robot
    u  = controller_nl(robotCurrentPose,desired_position);
    v = u(1);
    v_profile = [v_profile, v];
    omega = u(2);
    w_profile = [w_profile, omega];
    
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);
    
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.getRobotPose;
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - desired_position);
    
    % Wait for control rate to ensure 10 Hz rate.
    waitfor(controlRate);
    
    end
    
end
 %% plotar velocidades para ver estabilidade do controlador
figure
x=1:(length(v_profile));
plot (x,v_profile,x,w_profile)
title('Velocity Profiles During Trajectory K_r_o=1, K_a_l_p_h_a=0.5')
xlabel('Controller iterations') 
ylabel('Velocity')
legend({'Linear Velocity(v)','Angular Velocity (w)'},'Location','northeast')

%% Terminar simulacao

delete(robot)

%% Variaveis geometricas

function control_nl = controller_nl(current_p,next_p)

desired_next_pos = next_p; %updatar com os valores sucessivos do path
x = current_p(1);
y = current_p(2);
t = current_p(3);

ro = sqrt((desired_next_pos(1) - x)^2 + (desired_next_pos(2) - y)^2);
alpha = atan2((desired_next_pos(2) - y), (desired_next_pos(1) - x)) - t;
beta = alpha + t;



%alterar empiricamente
K_ro = 1;
K_alpha = 0.5;

%definir novos v,w
 v = K_ro*ro*cos(alpha);
 w = K_ro*cos(alpha)*sin(alpha) + K_alpha*alpha;
 
 control_nl = [v,w];
end
 
