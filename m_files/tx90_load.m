%% Load and display robot
clear
clc
robot = importrobot('tx90.urdf')
axes = show(robot);
axes.CameraPositionMode = 'auto';