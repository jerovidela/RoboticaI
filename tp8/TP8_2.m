%TP 8 EJ 2 
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
qq = [0, -pi/2, -pi/4, 0, pi/4, 0]; % Vector semilla y de orientación 
N = 100; 
% Matriz de rotación R del vector semilla 'qq' (FK) 
T_orientacion = fanuc_robot.fkine(qq); 
R = T_orientacion.R; 
% Crear matrices de transformación homogénea T1 y T2 
T1 = [R, P1'; 0, 0, 0, 1]; 
T2 = [R, P2'; 0, 0, 0, 1]; 
% Resolver IK para T1 y T2 usando qq como vector semilla ('q0') 
% 'mask', [1 1 1 0 0 0] indica que solo se fuerza la posición (x, y, z) 
q1 = fanuc_robot.ikine(T1, 'q0', qq, 'mask', [1 1 1 0 0 0]); 
q2 = fanuc_robot.ikine(T2, 'q0', qq, 'mask', [1 1 1 0 0 0]); 
% Interpolación en el espacio articular 
q = jtraj(q1, q2, N)  
fanuc_robot.plot(q); 
t_index = 1:N; % Vector de "tiempo" o índice de discretización 
qplot(t_index, q); 