# Basic Navigation Strategies for Mobile Robots

This repository contains the implementation of basic navigation strategies for mobile robots. 

In particular, this implementation was designed to deal with the Pioneer P3-DX robots. 

This implementation intends to make the robot navigate through a prescribed blueprint. Moreover, whenever the robot encounters doors, it has to turn its front towards the door and determine if it is fully open, half-open or closed.

The file ***report.pdf*** provides an extensive and detailed description of our approach to solving this problem.

### Code:

Inside the directory ***code***, you can find the computational implementation of the aforementioned navigation strategies in MATLAB.

+ ***BoboHobbit.m***: main script; calls the auxiliary functions described below;
+ ***controller_orientation.m***: control the robot orientation (compute only angular velocity, not linear velocity);
+ ***controller_translation.m*** and ***controller_translation2.m***: controls the robot navigation (computes both angular and linear velocities);
+ ***Fecho.m***: terminate all the ongoing processes in the computer at the end of the run;
+ ***get_C.m***, ***get_invT.m*** and ***get_invT***: compute auxiliary matrices (for coordinates conversion and rotation);
+ ***HarmScans.m***: "harmonizes" (filters out noisy measurements through harmonic mean) lidar scans;
+ ***LidarFrontProcessing.m***: converts a lidar scan into a spatial representation and outputs door_status and the coordinates concerning the beginning (left) of the door;
+ ***LidarProcessing.m***: convert a lidar scan into a spatial representation;
+ ***MinRad.m*** and ***MinRad2.m***: convert an angle (in radians) to the conventional angle branch;
+ ***odo_get_localization.m*** and ***odo_get_localization2.m***: correct odometry measurements according to correction matrix;
+ ***OdoToRad.m***: receives an angle from odometry and transforms it into radians.
+ ***Rotation_needed.m***: computes the rotation required to go from the current orientation to the desired one;
+ ***sonar_detect.m***: detect the sonar values from the Pioneer;
+ ***test_SIM***: computational simulation of the run (no real data), to check the correctness of some parts of the implementation;
+ ***sounds.mat***: file with the different sounds used throughout the run (to assert the door state).
  


 We apologize for having some of the code comments in portuguese. 