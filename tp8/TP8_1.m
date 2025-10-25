%TP 8 EJ1 
clear; clc; close all; 
T = 3; 
delta = 0.1; 
t = 0:delta:T; % Vector de tiempo 
q0 = [0, -pi/2, 0, 0, 0, 0]; 
q1 = [-pi/3, pi/10, -pi/5, pi/2, pi/4, 0]; 
% Generación de la trayectoria usando jtraj 
[q, qd, qq] = jtraj(q0, q1, t); 
disp(q) 
dh = [
    0, 0.45,  0.075, -pi/2, 0;
    0, 0,     0.3,    0,    0;
    0, 0,     0.075, -pi/2, 0;
    0, 0.32,  0,      pi/2,  0;
    0, 0,     0,     -pi/2,  0;
    0, 0.008, 0,      0,    0
];
fanuc_robot = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA'); 
fanuc_robot.plot(q); 
qplot(t, q) 