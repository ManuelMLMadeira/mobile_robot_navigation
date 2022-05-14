%% Inicializacao
clear all 
close all

%timer
global semaphore  
t_lidar = timer('Period', 0.3,'ExecutionMode', 'fixedRate');
t_lidar.TimerFcn = @myCallback;
start(t_lidar)

%Sons
load('sounds.mat');

%Inicializar robot
sp  = serial_port_start;
pioneer_init(sp);
SetupLidar;
soundsc(arranca, Fs_arranca);
pause(3)


%Inicializar variaveis
goalRadius = 50;
goalOrientation = 1;
%accumulated_error = zeros(1,3);
path = [3000    0      
        3000    3800
        3000    6915;%PORTA 1
        3000    9910;%PORTA 2
        3000    11345; %PORTA 3
        3000    12905; %PORTA 4
        3000    15645; %PORTA 5
        3000    17500; %PORTA 6 E FINAL CORREDOR 1
        5365    18130; %PORTA 7
        8115    18130; %PORTA 8
        10130   18130; %PORTA 9
        14890   18130; %PORTA 10
        17070   18130; %PORTA 11
        17230   18130; %FIM CORREDOR 2
        17430   14665; %PORTA 12
        17430   13410; %PORTA 13
        17430   11715; %PORTA 14
        17430   10255; %PORTA 15
        17430   10210; %PORTA 16
        17430   6215; %PORTA 17
        17430   3800; %FINAL DO CORREDOR 3 E PORTA 18
        12740   3800; %PORTA 19
        7250    3800; %PORTA 20
        3200    3800; %PORTA 21 
        3200    0     
        0       0 ];

% info portas 
doors = [3000 6480; %1
         3000  9475; %2
         3000  10895; %3 - RIGHT
         3000  12470; %4
         3000  15210; %5
         3000  17500; %6 - porta medida a 80 cm do inicio do corredor
         4930  18130; %7
         7680  18130; %8
         9680  18130; %9 - RIGHT
         14300 18130;%10
         16680 18130;%11
         17430 15050;%12 - RIGHT
         17430 14000;%13
         17430 12100;%14 - RIGHT
         17430 10690;%15
         17430 10800;%16 - RIGHT
         17430 6805;%17
         17430 3800;%18 - porta medida a 60 cm do inicio do corredor
         13330 3800; %19
         7840  3800; %20
         3695  3800]; %21
right_doors = [3 9 12 14 16];
double_door = [6 10 11 13 17 18 19 20 21];
stairs = [3 9 16];
wc = [12 14];


% Observaveis
v_profile = [];
w_profile = [];
positions = zeros(1*10^6,3);
counter_positions = 1;
C = get_T(0,0,0);

figure(1)
axis([0 20000 0 20000])
plot(path(:,1), path(:,2),'k--d')
hold on
drawnow

%% Saida da Sala

for i = 1:2

% INCIALIZACAO
    curr_pos = odo_get_localization(C);
    positions(counter_positions,:) = curr_pos;
    counter_positions = counter_positions +1;
    
    desired_position = path(i,1:2);
    if i == 1
        desired_orientation = 0;
    else
        desired_orientation = pi/2;
    end
        
    distanceToGoal = norm(curr_pos(1:2) - desired_position);
    rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);
    

% ENDIREITAR
    while ~(abs(rotation_necessary)<goalOrientation)
            if(distanceToGoal <= goalRadius)
                break
            end

            % Compute the controller outputs, i.e., the inputs to the robot
            omega  = controller_orientation(curr_pos(3),desired_orientation)*180/pi;
            w_profile = [w_profile, omega];
            v_profile = [v_profile, 0];


            pioneer_set_controls(sp,0,round(omega))

            curr_pos = odo_get_localization(C);
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;

            distanceToGoal = norm(curr_pos(1:2) - desired_position);
            rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);
    end
       
    
% PERCORRER SAIDA
    while( distanceToGoal > goalRadius )
        

   % Compute the controller outputs, i.e., the inputs to the robot
        u  = controller_translation(curr_pos,desired_position);
        v = u(1);
        v_profile = [v_profile, v];
        omega = double(u(2)*180/double(pi));
        w_profile = [w_profile, omega];

        pioneer_set_controls(sp,round(v),round(omega))
        curr_pos = odo_get_localization(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;

        distanceToGoal = norm(curr_pos(1:2) - desired_position);       
    end          
end

% FIM SAIDA DA SALA
disp('saida terminada')
% pioneer_set_controls(sp,0,0)
% v_profile = [v_profile, 0];
% w_profile = [w_profile, 0];


%% Corredor 1

% INICIALIZACAO

% preparacao corredor 1

%CHEGAR A PONTO FINAL
curr_pos = odo_get_localization(C);
desired_position_f = path(8,1:2);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;
distanceToGoal_f = norm(curr_pos(1:2) - desired_position_f);

corridor = 1;

desired_position = path(3,1:2);
desired_orientation = pi/2;
distanceToGoal = norm(curr_pos(1:2) - desired_position);
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);

% porta 1
door = 1;
door_coord = doors(door,:);
dist_detect = 435;
int_x = 1;
rot = double(pi);
int_ori = 0;
door_pause = 8;
i=3;

while( distanceToGoal_f > goalRadius )

    if semaphore && i~=8
        sonar_detect(sp, sai_da_frente, Fs_guedes);
        semaphore = 0;
    end
    if i<8   
        %INICIALIZACAO
        curr_pos = odo_get_localization(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;

        desired_position = path(i,1:2);

        distanceToGoal = norm(curr_pos(1:2) - desired_position);
        rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);

        %CHEGAR A PONTO
        if( (desired_position(2) - curr_pos(2)) < goalRadius )

            pioneer_set_controls(sp,0,0)
            v_profile = [v_profile, 0];
            w_profile = [w_profile, 0];
            curr_pos = odo_get_localization(C);                         
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;
           


            % DOOR STATUS
            if door~=6
                pioneer_set_controls(sp,0,0)
                v_profile = [v_profile, 0];
                w_profile = [w_profile, 0];

                % VIRAR P PORTA
                diff_rot = Rotation_needed(curr_pos(3)*180/pi,rot*180/pi);
                while abs(diff_rot) > goalOrientation  %verificar esta tolerancia
                   omega = controller_orientation (curr_pos(3), rot)*180/pi;
                   pioneer_set_controls(sp,0,round(omega));
                   v_profile = [v_profile, 0];
                   w_profile = [w_profile, omega];
                   curr_pos = odo_get_localization(C);
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;
                   diff_rot = Rotation_needed(curr_pos(3)*180/pi,rot*180/pi);
                end  
                pioneer_set_controls(sp,0,0)
                v_profile = [v_profile, 0];
                w_profile = [w_profile, 0];

                disp('rodar terminado')

                pause(1)
                scan = LidarScan(lidar);
                if door==1
                     disp(['position antes de correcao x local: ' num2str(curr_pos)]) 
                     [door_status,x_dist] = LidarFrontProcessing(scan, 2*dist_detect,2,1000,1);
                     if ~isnan(x_dist) && abs(curr_pos(2)-(door_coord(2)+x_dist))<250
                        curr_pos(2) = door_coord(2) + x_dist;
                        C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));
                        curr_pos = odo_get_localization(C); 
                        disp(['position depois de correcao x local: ' num2str(curr_pos)])
                     end
                else
                    door_status = LidarFrontProcessing(scan, 2*dist_detect,2,1000,0);
                end
                disp(['door_status:' num2str(door_status)])
                if door_status ==0
                    soundsc(fechada,Fs_fechada);
                elseif door_status ==1
                    soundsc(entreaberta,Fs_entreaberta);
                elseif door_status ==2
                    soundsc(aberta,Fs_aberta);
                end
                        

                % ENDIREITAR DE NOVO

                beta = MinRad(atan2(path(8,2)-curr_pos(2),path(8,1)-curr_pos(1)));
                rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

                while ~(abs(rotation_necessary)<goalOrientation)
                       omega = controller_orientation (curr_pos(3),beta)*180/pi;

                       pioneer_set_controls(sp,0,round(omega))
                       w_profile = [w_profile, omega];
                       v_profile = [v_profile, 0];

                       curr_pos = odo_get_localization(C);                         
                       positions(counter_positions,:) = curr_pos;
                       counter_positions = counter_positions +1;
                       rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
                end


                pioneer_set_controls(sp,0,0)
                v_profile = [v_profile, 0];
                w_profile = [w_profile, 0];
                curr_pos = odo_get_localization(C);                         
                positions(counter_positions,:) = curr_pos;
                counter_positions = counter_positions +1;

                %CORRECAO Y E THETA
                if door == 1 || door == 4 

                    [harm_scan harm_angles] = HarmScans(4,lidar);
                    [y_robot, angle_robot] = LidarCorrection(harm_scan, harm_angles, int_ori);

                    disp(['posicao antes correcao: ' num2str(curr_pos)])

                    new_theta = MinRad(pi/2 + angle_robot);
                    diff_theta = new_theta - curr_pos(3);
                    if abs(diff_theta) > 1 * pi/180 %THETA THRESHOLD
                            curr_pos(3) = new_theta;
                    end 

                    new_y  = 2330 + 1650/2 - y_robot;
                    diff_y = new_y - curr_pos(1);
                    if abs(diff_y) > 30                    %Y THRESHOLD
                        curr_pos(1) = new_y;
                    end 

                    C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));


                    curr_pos = odo_get_localization(C);
                    disp(['position depois de correcao y local: ' num2str(curr_pos)])                        
                    positions(counter_positions,:) = curr_pos;
                    counter_positions = counter_positions +1;
                    
                    % ENDIREITAR DE NOVO
                    beta = MinRad(atan2(path(8,2)-curr_pos(2),path(8,1)-curr_pos(1)));
                    rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

                   while ~(abs(rotation_necessary)<goalOrientation)
                           omega = controller_orientation (curr_pos(3),beta)*180/pi;

                           pioneer_set_controls(sp,0,round(omega))
                           w_profile = [w_profile, omega];
                           v_profile = [v_profile, 0];

                           curr_pos = odo_get_localization(C);                         
                           positions(counter_positions,:) = curr_pos;
                           counter_positions = counter_positions +1;
                           distanceToGoal = norm(curr_pos(1:2) - desired_position);
                           rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
                   end

                   pioneer_set_controls(sp,0,0)
                   v_profile = [v_profile, 0];
                   w_profile = [w_profile, 0];
                   curr_pos = odo_get_localization(C);                         
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;


                end

                %ATUALIZAR PORTA
                door = door + 1;
                disp(['next door:' num2str(door)])
                door_pause = 0;
                door_coord = doors(door,:);         

                if ismember(door,double_door)
                   dist_detect = 590;
                elseif ismember(door,wc)
                   dist_detect = 385;
                elseif ismember(door,stairs)
                   dist_detect=450;
                else 
                   dist_detect = 435;
                end

                if ismember(door,right_doors)
                   int_x = 0;
                else
                   int_x = 1;
                end 

                if int_x
                  rot = double(pi);
                else
                  rot = 0;
                end

                if door == 2
                    int_ori = 0;
                elseif door == 6
                    int_ori = 1;
                end
            end
            % FIM DOOR STATUS
             i =i+1;
             disp(['new i:' num2str(i)])
        end
    end

   % Compute the controller outputs, i.e., the inputs to the robot
    u  = controller_translation(curr_pos,desired_position_f);
    v = u(1);
    v_profile = [v_profile, v];
    omega = double(u(2)*180/double(pi));
    w_profile = [w_profile, omega];

    pioneer_set_controls(sp,round(v),round(omega))
    curr_pos = odo_get_localization(C);
    positions(counter_positions,:) = curr_pos;
    counter_positions = counter_positions +1;

    distanceToGoal_f = norm(curr_pos(1:2) - desired_position_f);       
end          

    
% FIM CORREDOR 1    
c
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

curr_pos = odo_get_localization(C);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;
    
%% Curva 1 
disp('Curva 1')

%Rodar para pi/2 (controlador nao garante isto antes)
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,90);
curr_pos = odo_get_localization(C);
while ~(abs(rotation_necessary)<goalOrientation)
       omega = controller_orientation (curr_pos(3),pi/2)*180/pi;

       pioneer_set_controls(sp,0,round(omega))
       w_profile = [w_profile, omega];
       v_profile = [v_profile, 0];

       curr_pos = odo_get_localization(C);                         
       positions(counter_positions,:) = curr_pos;
       counter_positions = counter_positions +1;
       rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,90);
end

pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];
curr_pos = odo_get_localization(C);                         
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;

door = 6;
door_coord = doors(door,:);
dist_detect = 590;

pause(1)
scan = LidarScan(lidar);
door_status = LidarFrontProcessing(scan, 2*dist_detect,0,1000,0);
disp(['door_status:' num2str(door_status)])
if door_status ==0
    soundsc(fechada,Fs_fechada);
elseif door_status ==0
    soundsc(entreaberta,Fs_entreaberta);
elseif door_status ==0
    soundsc(aberta,Fs_aberta);
end

%CORRECAO Y E THETA

[harm_scan harm_angles] = HarmScans(4,lidar);
[y_robot, angle_robot] = LidarCorrection(harm_scan, harm_angles, int_ori);

disp(['posicao antes correcao: ' num2str(curr_pos)])

new_theta = MinRad(pi/2 + angle_robot);
diff_theta = new_theta - curr_pos(3);
if abs(diff_theta) > 1 * pi/180 %THETA THRESHOLD
        curr_pos(3) = new_theta;
end 

new_y  = 2330 + 1650/2 - y_robot;
diff_y = new_y - curr_pos(1);
if abs(diff_y) > 30                    %Y THRESHOLD
    curr_pos(1) = new_y;
end 

C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));

curr_pos = odo_get_localization(C);
disp(['position depois de correcao y local: ' num2str(curr_pos)])                        
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;

% ENDIREITAR UM POUCO
point_orientation = [3000 18130];

 while point_orientation(2) - curr_pos(2) > goalRadius  %verificar esta tolerancia
     
    u = controller_translation (curr_pos, [curr_pos(1) point_orientation(2)]);
    v = u(1);
    omega = double(u(2)*180/pi);
    pioneer_set_controls(sp,round(v),round(omega));
    v_profile = [v_profile, v];
    w_profile = [w_profile, omega];
    curr_pos = odo_get_localization(C);
    positions(counter_positions,:) = curr_pos;
    counter_positions = counter_positions +1;
    d_toGoal = norm(curr_pos(1:2) - point_orientation);
 end

pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

curr_pos = odo_get_localization(C);
X = curr_pos(1);
Y = curr_pos(2);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;

%RODAR PARA ORIENTACAO 0
beta = MinRad(atan2(path(9,2)-curr_pos(2),path(9,1)-curr_pos(1)));
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
while ~(abs(rotation_necessary)<goalOrientation)
            
            % Compute the controller outputs, i.e., the inputs to the robot
            omega = controller_orientation(curr_pos(3),beta)*180/pi; 
            
            w_profile = [w_profile, omega];
            v_profile = [v_profile, 0];
            pioneer_set_controls(sp,0,round(omega))

            curr_pos = odo_get_localization(C);
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;
            rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
end
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

C = get_C (X,Y,curr_pos(3));


curr_pos = odo_get_localization(C);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;

%% Corredor 2
% 
disp ('Corredor 2')
% INICIALIZACAO
% CHEGAR A PONTO FINAL
desired_position_f = path(13,1:2);
curr_pos = odo_get_localization(C);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;
distanceToGoal_f = norm(curr_pos(1:2) - desired_position_f);

% preparacao corredor 2
corridor = 0;

curr_pos = odo_get_localization(C);
desired_position = path(9,1:2);
desired_orientation = 0;
distanceToGoal = norm(curr_pos(1:2) - desired_position);
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);

%ATUALIZAR PORTA
door = door + 1;
disp(['next door:' num2str(door)])
door_pause = 8;
door_coord = doors(door,:);
int_ori = 1;
dist_detect = 435;
int_x = 1;
rot = pi/2;
i = 9;
bool_78 = 0;

while( i~=14 )
 
    if semaphore && i ~= 13
        sonar_detect(sp, sai_da_frente, Fs_guedes);
        semaphore = 0;
    end
    
    if i<14

        %INICIALIZACAO
        curr_pos = odo_get_localization(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;

        desired_position = path(i,1:2);
        distanceToGoal = norm(curr_pos(1:2) - desired_position);
        rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);

    % PERCORRER CORREDOR
       if( (desired_position(1) - curr_pos(1)) < goalRadius )

            pioneer_set_controls(sp,0,0)
            v_profile = [v_profile, 0];
            w_profile = [w_profile, 0];
            curr_pos = odo_get_localization(C);                         
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;

            % DOOR STATUS

            if door~=12
               diff_rot = Rotation_needed(curr_pos(3)*180/pi,rot*180/pi);
               while abs(diff_rot) > goalOrientation  %verificar esta tolerancia
                   omega = controller_orientation (curr_pos(3), rot)*180/pi;
                   pioneer_set_controls(sp,0,round(omega));
                   v_profile = [v_profile, 0];
                   w_profile = [w_profile, omega];
                   curr_pos = odo_get_localization(C);
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;
                   diff_rot = Rotation_needed(curr_pos(3)*180/pi,rot*180/pi);
               end  
               pioneer_set_controls(sp,0,0)
               v_profile = [v_profile, 0];
               w_profile = [w_profile, 0];
               disp('rodar terminado')

               pause(1)
               scan = LidarScan(lidar);
               if door==7
                    disp(['position antes de correcao x local: ' num2str(curr_pos)]) 
                    [door_status,x_dist] = LidarFrontProcessing(scan, 2*dist_detect,1,1400,1);
                    if ~isnan(x_dist) && (door_coord(1)+x_dist)-curr_pos(1)>0 && (door_coord(1)+x_dist)-curr_pos(1)<250
                        curr_pos(1) = door_coord(1) + x_dist;
                        C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));
                        curr_pos = odo_get_localization(C); 
                        disp(['position depois de correcao x local: ' num2str(curr_pos)])
                    end
               elseif door == 11
                   door_status = LidarFrontProcessing(scan, 2*dist_detect,1,1200,0);
               elseif door == 10 
                   door_status = LidarFrontProcessing(scan, 2*dist_detect,0,500,0);
               else
                    door_status = LidarFrontProcessing(scan, 2*dist_detect,2,1000,0);
                end
                disp(['door_status:' num2str(door_status)])
                if door_status ==0
                    soundsc(fechada,Fs_fechada);
                elseif door_status ==1
                    soundsc(entreaberta,Fs_entreaberta);
                elseif door_status ==2
                    soundsc(aberta,Fs_aberta);
                end

               % ENDIREITAR DE NOVO

               beta = MinRad(atan2(path(14,2)-curr_pos(2),path(14,1)-curr_pos(1)));
               rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
                curr_pos = odo_get_localization(C);
                positions(counter_positions,:) = curr_pos;
                counter_positions = counter_positions +1;

               while ~(abs(rotation_necessary)<goalOrientation)
                       omega = controller_orientation (curr_pos(3),beta)*180/pi;

                       pioneer_set_controls(sp,0,round(omega))
                       w_profile = [w_profile, omega];
                       v_profile = [v_profile, 0];

                       curr_pos = odo_get_localization(C);                         
                       positions(counter_positions,:) = curr_pos;
                       counter_positions = counter_positions +1;
                       distanceToGoal = norm(curr_pos(1:2) - desired_position);
                       rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
               end

               pioneer_set_controls(sp,0,0)
               v_profile = [v_profile, 0];
               w_profile = [w_profile, 0];
               curr_pos = odo_get_localization(C);                         
               positions(counter_positions,:) = curr_pos;
               counter_positions = counter_positions +1;


               %CORRECAO Y E THETA
               if door == 9
                    [harm_scan harm_angles] = HarmScans(4,lidar);
                    [y_robot, angle_robot] = LidarCorrection(harm_scan, harm_angles, int_ori);

                    disp(['posicao antes correcao: ' num2str(curr_pos)])

                    new_theta = MinRad(angle_robot);
                    diff_theta = new_theta - curr_pos(3);
                    if abs(diff_theta) > 1 * pi/180 %THETA THRESHOLD
                            curr_pos(3) = new_theta;
                    end  

                    new_y  = 18730 - 1650/2 + y_robot;
                    diff_y = new_y - curr_pos(2);
                    if abs(diff_y) > 30                    %Y THRESHOLD
                        curr_pos(2) = new_y;
                    end 

                    C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));

                    curr_pos = odo_get_localization(C);
                    disp(['position depois de correcao y local: ' num2str(curr_pos)])                        
                    positions(counter_positions,:) = curr_pos;
                    counter_positions = counter_positions +1;

                    % ENDIREITAR DE NOVO
                    beta = MinRad(atan2(path(13,2)-curr_pos(2),path(13,1)-curr_pos(1)));
                    rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

                   while ~(abs(rotation_necessary)<goalOrientation)
                           omega = controller_orientation (curr_pos(3),beta)*180/pi;

                           pioneer_set_controls(sp,0,round(omega))
                           w_profile = [w_profile, omega];
                           v_profile = [v_profile, 0];

                           curr_pos = odo_get_localization(C);                         
                           positions(counter_positions,:) = curr_pos;
                           counter_positions = counter_positions +1;
                           rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
                   end

                   pioneer_set_controls(sp,0,0)
                   v_profile = [v_profile, 0];
                   w_profile = [w_profile, 0];
                   curr_pos = odo_get_localization(C);                         
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;

               end

               %ATUALIZAR PORTA
               door = door + 1;
               disp(['next door:' num2str(door)])
               door_pause = 0;
               door_coord = doors(door,:);         

               if ismember(door,double_door)
                   dist_detect = 590;
               elseif ismember(door,wc)
                   dist_detect = 385;
               elseif ismember(door,stairs)
                   dist_detect=450;
               else 
                   dist_detect = 435;
               end

                if ismember(door,right_doors)
                   int_x = 0;
                else
                   int_x = 1;
                end 

                if int_x
                  rot = pi/2;
                else
                  rot = -pi/2;
                end
            end
            % FIM DOOR STATUS  
       i = i+1;   
       disp(['new i:' num2str(i)])
       end
       %FIM CHEGAR A PORTA
       
    end
    %FIM PORTAS NO CORREDOR
    
    %CORRECAO Y E THETA A MEIO
       if curr_pos(1) > 6400 && ~bool_78 
           int_ori = 1;
           
            pioneer_set_controls(sp,0,0)
            v_profile = [v_profile, 0];
            w_profile = [w_profile, 0];

           
            [harm_scan harm_angles] = HarmScans(4,lidar);
            [y_robot, angle_robot] = LidarCorrection(harm_scan, harm_angles, int_ori);

            disp(['posicao antes correcao: ' num2str(curr_pos)])

            new_theta = MinRad(angle_robot);
            diff_theta = new_theta - curr_pos(3);
            if abs(diff_theta) > 1 * pi/180 %THETA THRESHOLD
                    curr_pos(3) = new_theta;
            end  

            new_y  = 18730 - 1650/2 + y_robot;
            diff_y = new_y - curr_pos(2);
            if abs(diff_y) > 30                    %Y THRESHOLD
                curr_pos(2) = new_y;
            end 

            C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));

            curr_pos = odo_get_localization(C);
            disp(['position depois de correcao y local: ' num2str(curr_pos)])                        
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;

            % ENDIREITAR DE NOVO
            beta = MinRad(atan2(path(13,2)-curr_pos(2),path(13,1)-curr_pos(1)));
            rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

           while ~(abs(rotation_necessary)<goalOrientation)
                   omega = controller_orientation (curr_pos(3),beta)*180/pi;

                   pioneer_set_controls(sp,0,round(omega))
                   w_profile = [w_profile, omega];
                   v_profile = [v_profile, 0];

                   curr_pos = odo_get_localization(C);                         
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;
                   rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
           end

           pioneer_set_controls(sp,0,0)
           v_profile = [v_profile, 0];
           w_profile = [w_profile, 0];
           curr_pos = odo_get_localization(C);                         
           positions(counter_positions,:) = curr_pos;
           counter_positions = counter_positions +1;
           bool_78 = 1;

       end

    if i~=14
       % Compute the controller outputs, i.e., the inputs to the robot
        u  = controller_translation(curr_pos,desired_position_f);
        v = u(1);
        v_profile = [v_profile, v];
        omega = double(u(2)*180/double(pi));
        w_profile = [w_profile, omega];

        pioneer_set_controls(sp,round(v),round(omega))
        curr_pos = odo_get_localization(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;

        distanceToGoal_f = norm(curr_pos(1:2) - desired_position_f);
    end
end          


% FIM CORREDOR 2    
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];


%% Curva 2
 
disp('Curva 2')

%Rodar para -pi/2
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,-90);
curr_pos = odo_get_localization(C);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;

X = curr_pos(1);
Y = curr_pos(2);

while ~(abs(rotation_necessary)<goalOrientation)
       omega = controller_orientation (curr_pos(3),-pi/2)*180/pi;

       pioneer_set_controls(sp,0,round(omega))
       w_profile = [w_profile, omega];
       v_profile = [v_profile, 0];

       curr_pos = odo_get_localization(C);                         
       positions(counter_positions,:) = curr_pos;
       counter_positions = counter_positions +1;
       distanceToGoal = norm(curr_pos(1:2) - desired_position);
       rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,-90);
end
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

C = get_C (X,Y,curr_pos(3));

%% Corredor 3
% 
disp ('Corredor 3')

% INICIALIZACAO

% preparacao corredor 3
corridor = 1;

desired_position = path(15,1:2);
desired_orientation = -pi/2;
curr_pos = odo_get_localization2(C);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;
distanceToGoal = norm(curr_pos(1:2) - desired_position);
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);


%PORTA JA ATUALIZADA
door_pause = 0;
dist_detect = 385;
int_x = 0;
rot = -pi;
bool21 = 0;
int_ori = 1;

% ENDIREITAR UM POUCO
point_orientation = [17430 16000];
d_toGoal = curr_pos(2) - point_orientation(2);
 while d_toGoal > goalRadius  %verificar esta tolerancia
     if semaphore 
        sonar_detect(sp, sai_da_frente, Fs_guedes);
        semaphore = 0;
    end
    u = controller_translation2 (curr_pos, [curr_pos(1) point_orientation(2)]);
    v = u(1);
    omega = double(u(2)*180/pi);
    pioneer_set_controls(sp,round(v),round(omega));
    v_profile = [v_profile, v];
    w_profile = [w_profile, omega];
    curr_pos = odo_get_localization2(C);
    positions(counter_positions,:) = curr_pos;
    counter_positions = counter_positions +1;
    d_toGoal = curr_pos(2) - point_orientation(2);
 end

pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];
curr_pos = odo_get_localization2(C);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;

%RODAR PARA -pi/2
 diff_rot = Rotation_needed(curr_pos(3)*180/pi,-90);
 while abs(diff_rot) > goalOrientation  %verificar esta tolerancia
   omega = controller_orientation (curr_pos(3), -pi/2)*180/pi;
   pioneer_set_controls(sp,0,round(omega));
   v_profile = [v_profile, 0];
   w_profile = [w_profile, omega];
   curr_pos = odo_get_localization2(C);
   positions(counter_positions,:) = curr_pos;
   counter_positions = counter_positions +1;
   diff_rot = Rotation_needed(curr_pos(3)*180/pi,-90);
 end 
 
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];
 


% CORRECCAO Y E THETA

[harm_scan harm_angles] = HarmScans(4,lidar);
[y_robot, angle_robot] = LidarCorrection(harm_scan, harm_angles, int_ori);

disp(['posicao antes correcao: ' num2str(curr_pos)])

new_theta = MinRad2(-pi/2 + angle_robot);
diff_theta = new_theta - curr_pos(3);
if abs(diff_theta) > 1 * pi/180 %THETA THRESHOLD
        curr_pos(3) = new_theta;
end  

new_y  =  18030 - 1650/2 + y_robot;
diff_y = new_y - curr_pos(1);
if abs(diff_y) > 30                    %Y THRESHOLD
    curr_pos(1) = new_y;
end 

C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));

curr_pos = odo_get_localization2(C);
disp(['position depois de correcao y local: ' num2str(curr_pos)])                        
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;

% ENDIREITAR DE NOVO
beta = MinRad2(atan2(path(21,2)-curr_pos(2),path(21,1)-curr_pos(1)));
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

while ~(abs(rotation_necessary)<goalOrientation)
       omega = controller_orientation (curr_pos(3),beta)*180/pi;

       pioneer_set_controls(sp,0,round(omega))
       w_profile = [w_profile, omega];
       v_profile = [v_profile, 0];

       curr_pos = odo_get_localization2(C);                         
       positions(counter_positions,:) = curr_pos;
       counter_positions = counter_positions +1;
       distanceToGoal = norm(curr_pos(1:2) - desired_position);
       rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
end

pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

%CHEGAR A PONTO FINAL
desired_position_f = path(21,1:2);
curr_pos = odo_get_localization2(C);                         
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;
distanceToGoal_f = norm(curr_pos(1:2) - desired_position_f);
i = 15;

while( distanceToGoal_f > goalRadius )

    if semaphore && i~=21
        sonar_detect(sp, sai_da_frente, Fs_guedes);
        semaphore = 0;
    end
    
    if i<21

        %INICIALIZACAO
        curr_pos = odo_get_localization2(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;

        desired_position = path(i,1:2);
        distanceToGoal = norm(curr_pos(1:2) - desired_position);
        rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);

        %ATE PONTO DESEJADO
       if( (curr_pos(2) - desired_position(2)) < goalRadius )

            pioneer_set_controls(sp,0,0)
            v_profile = [v_profile, 0];
            w_profile = [w_profile, 0];

            % DOOR STATUS
            if door~=18

               diff_rot = Rotation_needed(curr_pos(3)*180/pi,rot*180/pi);
               while abs(diff_rot) > goalOrientation  %verificar esta tolerancia
                   omega = controller_orientation (curr_pos(3), rot)*180/pi;
                   pioneer_set_controls(sp,0,round(omega));
                   v_profile = [v_profile, 0];
                   w_profile = [w_profile, omega];
                   curr_pos = odo_get_localization2(C);
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;
                   diff_rot = Rotation_needed(curr_pos(3)*180/pi,rot*180/pi);
               end  
               pioneer_set_controls(sp,0,0)
               v_profile = [v_profile, 0];
               w_profile = [w_profile, 0];

               disp('rodar terminado')

               pause(1)
               scan = LidarScan(lidar);
%                if door==13 
%                     disp(['position antes de correcao x local: ' num2str(curr_pos)]) 
%                     [door_status,x_dist] = LidarFrontProcessing(scan, 2*dist_detect,1,1000,1);
%                     if ~isnan(x_dist)
%                         curr_pos(2) = door_coord(2) - x_dist;
%                         C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));
%                         curr_pos = odo_get_localization2(C); 
%                         disp(['position depois de correcao x local: ' num2str(curr_pos)])
%                     end
                if door==13
                    door_status = LidarFrontProcessing(scan, 2*dist_detect,1,1000,0);
               elseif door ==14 || door == 17 
                   door_status = LidarFrontProcessing(scan, 2*dist_detect,0,1000,0);
               else
                    door_status = LidarFrontProcessing(scan, 2*dist_detect,2,1000,0);
                end
                disp(['door_status:' num2str(door_status)])
                if door_status ==0
                    soundsc(fechada,Fs_fechada);
                elseif door_status ==1
                    soundsc(entreaberta,Fs_entreaberta);
                elseif door_status ==2
                    soundsc(aberta,Fs_aberta);
                end

               % CASO ESPECIAL 15/16
               if door==15
                   rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,-180);
                   door = 16;
                   i=i+1;
                   disp(['new i:' num2str(i)])
                   while ~(abs(rotation_necessary)<goalOrientation)
                           omega = controller_orientation (curr_pos(3),-pi)*180/pi;

                           pioneer_set_controls(sp,0,round(omega))
                           w_profile = [w_profile, omega];
                           v_profile = [v_profile, 0];

                           curr_pos = odo_get_localization2(C);                         
                           positions(counter_positions,:) = curr_pos;
                           counter_positions = counter_positions +1;
                           rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,-180);
                   end

                   pioneer_set_controls(sp,0,0)
                   v_profile = [v_profile, 0];
                   w_profile = [w_profile, 0];
                   disp('rodar terminado')

                   pause(1)
                   scan = LidarScan(lidar);
                   door_status = LidarFrontProcessing(scan, 2*dist_detect,1,500,0);
                   if door_status ==0
                        soundsc(fechada,Fs_fechada);
                    elseif door_status ==1
                        soundsc(entreaberta,Fs_entreaberta);
                    elseif door_status ==2
                        soundsc(aberta,Fs_aberta);
                    end
               end


               % ENDIREITAR DE NOVO
                beta = MinRad2(atan2(path(21,2)-curr_pos(2),path(21,1)-curr_pos(1)));
                rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

               while ~(abs(rotation_necessary)<goalOrientation)
                       omega = controller_orientation (curr_pos(3),beta)*180/pi;

                       pioneer_set_controls(sp,0,round(omega))
                       w_profile = [w_profile, omega];
                       v_profile = [v_profile, 0];

                       curr_pos = odo_get_localization2(C);                         
                       positions(counter_positions,:) = curr_pos;
                       counter_positions = counter_positions +1;

                       rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
               end

               pioneer_set_controls(sp,0,0)
               v_profile = [v_profile, 0];
               w_profile = [w_profile, 0];


               %ATUALIZAR PORTA
               door = door + 1;
               door_pause = 0;
               disp(['next door:' num2str(door)])
               door_coord = doors(door,:);         

               if ismember(door,double_door)
                   dist_detect = 590;
               elseif ismember(door,wc)
                   dist_detect = 385;
               elseif ismember(door,stairs)
                   dist_detect=450;
               else 
                   dist_detect = 435;
               end

               if ismember(door,right_doors)
                   int_x = 0;
               else
                   int_x = 1;
               end 

               if int_x
                  rot = 0;
               else
                  rot = -double(pi);
               end

             end
            % FIM DOOR STATUS 
             i = i+1;
             disp(['new i:' num2str(i)])
       end
       
       %CORRECAO Y E THETA A MEIO
       if curr_pos(2) < 8500 && ~bool21 
           
            pioneer_set_controls(sp,0,0)
            v_profile = [v_profile, 0];
            w_profile = [w_profile, 0];

           
            [harm_scan harm_angles] = HarmScans(4,lidar);
            [y_robot, angle_robot] = LidarCorrection(harm_scan, harm_angles, int_ori);

            disp(['posicao antes correcao: ' num2str(curr_pos)])

            new_theta = MinRad2(-pi/2 + angle_robot);
            diff_theta = new_theta - curr_pos(3);
            if abs(diff_theta) > 1 * pi/180 %THETA THRESHOLD
                    curr_pos(3) = new_theta;
            end  

            new_y  =  18030 - 1650/2 + y_robot;
            diff_y = new_y - curr_pos(1);
            if abs(diff_y) > 30                    %Y THRESHOLD
                curr_pos(1) = new_y;
            end 

            C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));

            curr_pos = odo_get_localization2(C);
            disp(['position depois de correcao y local: ' num2str(curr_pos)])                        
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;

            % ENDIREITAR DE NOVO
            beta = MinRad2(atan2(path(21,2)-curr_pos(2),path(21,1)-curr_pos(1)));
            rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

           while ~(abs(rotation_necessary)<goalOrientation)
                   omega = controller_orientation (curr_pos(3),beta)*180/pi;

                   pioneer_set_controls(sp,0,round(omega))
                   w_profile = [w_profile, omega];
                   v_profile = [v_profile, 0];

                   curr_pos = odo_get_localization2(C);                         
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;
                   rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
           end

           pioneer_set_controls(sp,0,0)
           v_profile = [v_profile, 0];
           w_profile = [w_profile, 0];
           bool21 = 1;

       end
      
    end
    
   
               

   % Compute the controller outputs, i.e., the inputs to the robot
        u  = controller_translation2(curr_pos,desired_position_f);
        v = u(1);
        v_profile = [v_profile, v];
        omega = double(u(2)*180/double(pi));
        w_profile = [w_profile, omega];

        pioneer_set_controls(sp,round(v),round(omega))
        curr_pos = odo_get_localization2(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;

        distanceToGoal_f = norm(curr_pos(1:2) - desired_position_f);       
end          


% FIM CORREDOR 3    
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

%% Curva 3 
disp('Curva 3')

%Rodar para -pi/2 (controlador nao garante isto antes)
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,-90);

while ~(abs(rotation_necessary)<goalOrientation)
       omega = controller_orientation (curr_pos(3),-pi/2)*180/pi;

       pioneer_set_controls(sp,0,round(omega))
       w_profile = [w_profile, omega];
       v_profile = [v_profile, 0];

       curr_pos = odo_get_localization2(C);                         
       positions(counter_positions,:) = curr_pos;
       counter_positions = counter_positions +1;
       distanceToGoal = norm(curr_pos(1:2) - desired_position);
       rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,-90);
end

pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

door = 18;
door_coord = doors(door,:);
dist_detect = 590;

pause(1)
scan = LidarScan(lidar);
door_status = LidarFrontProcessing(scan, 2*dist_detect,0,1200,0);
disp(['door_status:' num2str(door_status)])
if door_status ==0
    soundsc(fechada,Fs_fechada);
elseif door_status ==1
    soundsc(entreaberta,Fs_entreaberta);
elseif door_status ==2
    soundsc(aberta,Fs_aberta);
end

X = curr_pos(1);
Y = curr_pos(2);

%RODAR PARA ORIENTACAO -pi

curr_pos = odo_get_localization2(C);
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;

rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,-pi*180/pi);

while ~(abs(rotation_necessary)<goalOrientation)
            
            % Compute the controller outputs, i.e., the inputs to the robot
            omega = controller_orientation(curr_pos(3),-pi)*180/pi; 
            
            w_profile = [w_profile, omega];
            v_profile = [v_profile, 0];
            pioneer_set_controls(sp,0,round(omega))

            curr_pos = odo_get_localization2(C);
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;
            rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,-pi*180/pi);
end

%FIM CURVA
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

C = get_C (X,Y,curr_pos(3));

%% Corredor 4
% 

disp ('Corredor 4')

% INICIALIZACAO

% preparacao corredor 4
corridor = 0;

curr_pos = odo_get_localization2(C);
desired_position = path(22,1:2);
desired_orientation = -pi;
distanceToGoal = norm(curr_pos(1:2) - desired_position);
rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);

%ATUALIZAR PORTA
door = door + 1;
door_pause = 0;
door_coord = doors(door,:);
int_ori = 0;
dist_detect = 590;
int_x = 1;
rot = -pi/2;
bool21 = 0;

%CHEGAR A PONTO FINAL
desired_position_f = path(24,1:2);
curr_pos = odo_get_localization2(C);                         
positions(counter_positions,:) = curr_pos;
counter_positions = counter_positions +1;
distanceToGoal_f = norm(curr_pos(1:2) - desired_position_f);
i = 22;

while( distanceToGoal_f > goalRadius )

    if semaphore && i~=24
        sonar_detect(sp, sai_da_frente, Fs_guedes);
        semaphore = 0;
    end
    
    if i <24

        %INICIALIZACAO
        curr_pos = odo_get_localization2(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;
        desired_position = path(i,1:2);
        distanceToGoal = norm(curr_pos(1:2) - desired_position);
        rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);

        % ATE PONTO
       if( (curr_pos(1) - desired_position(1)) < goalRadius )
            pioneer_set_controls(sp,0,0)
            v_profile = [v_profile, 0];
            w_profile = [w_profile, 0];

            % DOOR STATUS
            if door~=21
                if door == 20
                    dist_detect = 790;
                end

               diff_rot = Rotation_needed(curr_pos(3)*180/pi,rot*180/pi);
               while abs(diff_rot) > goalOrientation  %verificar esta tolerancia
                   omega = controller_orientation (curr_pos(3), rot)*180/pi;
                   pioneer_set_controls(sp,0,round(omega));
                   v_profile = [v_profile, 0];
                   w_profile = [w_profile, omega];
                   curr_pos = odo_get_localization2(C);
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;
                   diff_rot = Rotation_needed(curr_pos(3)*180/pi,rot*180/pi);
               end  
               pioneer_set_controls(sp,0,0)
               v_profile = [v_profile, 0];
               w_profile = [w_profile, 0];
               disp('rodar terminado')

               pause(1)
               scan = LidarScan(lidar);
               if door==19 
                    disp(['position antes de correcao x local: ' num2str(curr_pos)]) 
                    [door_status,x_dist] = LidarFrontProcessing(scan, 2*dist_detect,0,800,1);
                    if ~isnan(x_dist) && curr_pos(1)-(door_coord(1)-x_dist)<250 && curr_pos(1)-(door_coord(1)-x_dist)>0
                        curr_pos(1) = door_coord(1) - x_dist;
                        C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));
                        curr_pos = odo_get_localization2(C); 
                        disp(['position depois de correcao x local: ' num2str(curr_pos)]) 
                    end
               else

                    door_status = LidarFrontProcessing(scan,dist_detect,2,1200,0);
                end
                disp(['door_status:' num2str(door_status)])
                if door_status ==0
                    soundsc(fechada,Fs_fechada);
                elseif door_status ==1
                    soundsc(entreaberta,Fs_entreaberta);
                elseif door_status ==2
                    soundsc(aberta,Fs_aberta);
                end


               % ENDIREITAR DE NOVO

               beta = MinRad2(atan2(path(24,2)-curr_pos(2),path(24,1)-curr_pos(1)));
               rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

               while ~(abs(rotation_necessary)<goalOrientation)
                       omega = controller_orientation (curr_pos(3),beta)*180/pi;

                       pioneer_set_controls(sp,0,round(omega))
                       w_profile = [w_profile, omega];
                       v_profile = [v_profile, 0];

                       curr_pos = odo_get_localization2(C);                         
                       positions(counter_positions,:) = curr_pos;
                       counter_positions = counter_positions +1;
                       distanceToGoal = norm(curr_pos(1:2) - desired_position);
                       rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
               end

               pioneer_set_controls(sp,0,0)
               v_profile = [v_profile, 0];
               w_profile = [w_profile, 0];


               %CORRECAO Y E THETA
               if door == 19   
                   [harm_scan harm_angles] = HarmScans(4,lidar);
                   [y_robot, angle_robot] = LidarCorrection(harm_scan, harm_angles, int_ori);

                   disp(['posicao antes correcao: ' num2str(curr_pos)])

                    new_theta = MinRad2(angle_robot-pi);
                    diff_theta = new_theta - curr_pos(3);
                    if abs(diff_theta) > 1 * pi/180 %THETA THRESHOLD
                            curr_pos(3) = new_theta;
                    end  

                    new_y  = 3000 + 1650/2 - y_robot;
                    diff_y = new_y - curr_pos(2);
                    if abs(diff_y) > 30                    %Y THRESHOLD
                        curr_pos(2) = new_y;
                    end 

                    C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));

                    curr_pos = odo_get_localization2(C);
                    disp(['position depois de correcao y local: ' num2str(curr_pos)])                        
                    positions(counter_positions,:) = curr_pos;
                    counter_positions = counter_positions +1;

                    % ENDIREITAR DE NOVO
                    beta = MinRad2(atan2(path(24,2)-curr_pos(2),path(24,1)-curr_pos(1)));
                    rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

                   while ~(abs(rotation_necessary)<goalOrientation)
                           omega = controller_orientation (curr_pos(3),beta)*180/pi;

                           pioneer_set_controls(sp,0,round(omega))
                           w_profile = [w_profile, omega];
                           v_profile = [v_profile, 0];

                           curr_pos = odo_get_localization2(C);                         
                           positions(counter_positions,:) = curr_pos;
                           counter_positions = counter_positions +1;
                           rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
                   end

                   pioneer_set_controls(sp,0,0)
                   v_profile = [v_profile, 0];
                   w_profile = [w_profile, 0];

               end

               %ATUALIZAR PORTA
               door = door + 1;
               disp(['next door:' num2str(door)])
               door_pause = 0;
               door_coord = doors(door,:);         

               if ismember(door,double_door)
                   dist_detect = 590;
               elseif ismember(door,wc)
                   dist_detect = 385;
               elseif ismember(door,stairs)
                   dist_detect=450;
               else 
                   dist_detect = 435;
               end

            end
            % FIM DOOR STATUS
            i = i+1;
            disp(['new i:' num2str(i)])
       end
       %FIM CHEGAR A PORTA
    end
    %FIM PORTAS DO CORREDOR


    
    %CORRECAO Y E THETA PRE ENTRADA
       if curr_pos(1) < 5300 && ~bool21 
           pioneer_set_controls(sp,0,0)
           v_profile = [v_profile, 0];
           w_profile = [w_profile, 0];
           int_ori = 1;
           [harm_scan harm_angles] = HarmScans(4,lidar);
           [y_robot, angle_robot] = LidarCorrection(harm_scan, harm_angles, int_ori);
            
           disp(['posicao antes correcao: ' num2str(curr_pos)])
            
            new_theta = MinRad2(angle_robot-pi);
            diff_theta = new_theta - curr_pos(3);
            if abs(diff_theta) > 1 * pi/180 %THETA THRESHOLD
                    curr_pos(3) = new_theta;
            end  

            new_y  = 3000 + 1650/2 - y_robot;
            diff_y = new_y - curr_pos(2);
            if abs(diff_y) > 30                    %Y THRESHOLD
                curr_pos(2) = new_y;
            end 

            C = get_C (curr_pos(1),curr_pos(2),curr_pos(3));
            
            curr_pos = odo_get_localization2(C);
            disp(['position depois de correcao y local: ' num2str(curr_pos)])                        
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;
            
            % ENDIREITAR DE NOVO
            beta = MinRad2(atan2(path(24,2)-curr_pos(2),path(24,1)-curr_pos(1)));
            rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);

           while ~(abs(rotation_necessary)<goalOrientation)
                   omega = controller_orientation (curr_pos(3),beta)*180/pi;

                   pioneer_set_controls(sp,0,round(omega))
                   w_profile = [w_profile, omega];
                   v_profile = [v_profile, 0];

                   curr_pos = odo_get_localization2(C);                         
                   positions(counter_positions,:) = curr_pos;
                   counter_positions = counter_positions +1;
                   rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,beta*180/pi);
           end

           pioneer_set_controls(sp,0,0)
           v_profile = [v_profile, 0];
           w_profile = [w_profile, 0];
            
            bool21 = 1;
       end
       
   % Compute the controller outputs, i.e., the inputs to the robot
        u  = controller_translation2(curr_pos,desired_position_f);
        v = u(1);
        v_profile = [v_profile, v];
        omega = double(u(2)*180/double(pi));
        w_profile = [w_profile, omega];

        pioneer_set_controls(sp,round(v),round(omega))
        curr_pos = odo_get_localization2(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;

        distanceToGoal_f = norm(curr_pos(1:2) - desired_position_f);       
end          


% FIM CORREDOR 4    
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

%% Curva 4

 %RODAR PARA -pi/2
 curr_pos = odo_get_localization2(C);
 positions(counter_positions,:) = curr_pos;
 counter_positions = counter_positions +1;
 
 X = curr_pos(1);
 Y = curr_pos(2);
 
 diff_rot = Rotation_needed(curr_pos(3)*180/pi,-90);
 while abs(diff_rot) > goalOrientation  %verificar esta tolerancia
   omega = controller_orientation (curr_pos(3), -pi/2)*180/pi;
   pioneer_set_controls(sp,0,round(omega));
   v_profile = [v_profile, 0];
   w_profile = [w_profile, omega];
   curr_pos = odo_get_localization2(C);
   positions(counter_positions,:) = curr_pos;
   counter_positions = counter_positions +1;
   diff_rot = Rotation_needed(curr_pos(3)*180/pi,-90);
 end 
 
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

C = get_C (X,Y,curr_pos(3));
                   
%% Entrada na Sala

soundsc(champions,Fs_champions)
for i = 25:26

% INCIALIZACAO
    curr_pos = odo_get_localization2(C);
    positions(counter_positions,:) = curr_pos;
    counter_positions = counter_positions +1;
    
    desired_position = path(i,1:2);
    if i == 25
        desired_orientation = -pi/2;
    else
        desired_orientation = -pi;
    end
    distanceToGoal = norm(curr_pos(1:2) - desired_position);
    rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);
    
    X = curr_pos(1);
    Y = curr_pos(2);
    
% ENDIREITAR
    while ~(abs(rotation_necessary)<goalOrientation)
            if(distanceToGoal <= goalRadius)
                break
            end
            
            
            % Compute the controller outputs, i.e., the inputs to the robot
            omega  = controller_orientation(curr_pos(3),desired_orientation)*180/pi;
            w_profile = [w_profile, omega];
            v_profile = [v_profile, 0];

            pioneer_set_controls(sp,0,round(omega))

            curr_pos = odo_get_localization2(C);
            positions(counter_positions,:) = curr_pos;
            counter_positions = counter_positions +1;

            distanceToGoal = norm(curr_pos(1:2) - desired_position);
            rotation_necessary = Rotation_needed(curr_pos(3)*180/pi,desired_orientation*180/pi);
    end
    
    C = get_C (X,Y,curr_pos(3));
       
    
% PERCORRER SAIDA
    while( distanceToGoal > goalRadius )
        
        if semaphore 
            sonar_detect(sp, sai_da_frente, Fs_guedes);
            semaphore = 0;
        end
    
   % Compute the controller outputs, i.e., the inputs to the robot
        u  = controller_translation2(curr_pos,desired_position);
        v = u(1);
        v_profile = [v_profile, v];
        omega = double(u(2)*180/double(pi));
        w_profile = [w_profile, omega];

        pioneer_set_controls(sp,round(v),round(omega))
        curr_pos = odo_get_localization2(C);
        positions(counter_positions,:) = curr_pos;
        counter_positions = counter_positions +1;

        distanceToGoal = norm(curr_pos(1:2) - desired_position);       
    end          
end

% FIM SAIDA DA SALA
disp('entrada terminada')
pioneer_set_controls(sp,0,0)
v_profile = [v_profile, 0];
w_profile = [w_profile, 0];

%% Fecho 
delete(timerfindall)
delete(instrfindall)
serial_port_stop(sp)
pioneer_close(sp)