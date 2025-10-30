% FANUC Ejercicio 2 - IK posición fija + jtraj
clear; clc; close all;
dh=[ 
0, 0.45, 0.075, -pi/2, 0; 
0, 0, 0.3, 0, 0;
0, 0, 0.075, -pi/2, 0;  
0, 0.32, 0,pi/2, 0;  
0,0,0,-pi/2,0;
0, 0.008, 0, 0, 0;      
];
fanuc_robot = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');
P1 = [0, 0, 0.95];
P2 = [0.4, 0, 0.95];
qq = [0, -pi/2, -pi/4, 0, pi/4, 0]; % semilla/orientación
N = 100;

T_orientacion = fanuc_robot.fkine(qq);
R = T_orientacion.R;
T1 = [R, P1'; 0, 0, 0, 1];
T2 = [R, P2'; 0, 0, 0, 1];

q1 = fanuc_robot.ikine(T1, 'q0', qq, 'mask', [1 1 1 0 0 0]);
q2 = fanuc_robot.ikine(T2, 'q0', qq, 'mask', [1 1 1 0 0 0]);

q = jtraj(q1, q2, N);
fanuc_robot.plot(q);
t_index = 1:N;
qplot(t_index, q);

