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
q1 = [0, -pi/2, -pi/2, 0, pi/2, 0]; 
q2 = [pi/2, -pi/2, -pi/2, 0, pi/2, 0]; 
q3 = [pi/6, -pi/6, 0, 0, pi/6, 0]; 
q4 = [pi/3, -pi/4, pi/6, 0, pi/3, 0]; 
% Parámetros de tiempo 
N = 100;  % cantidad de puntos por segmento 
t_total = linspace(0, 3*(N-1), 3*N); % eje temporal para graficar 
%Generar trayectorias con frenado entre segmentos  
[q12, qd12, qdd12] = jtraj(q1, q2, N); 
[q23, qd23, qdd23] = jtraj(q2, q3, N); 
[q34, qd34, qdd34] = jtraj(q3, q4, N); 
% Unir las trayectorias 
q_total   = [q12; q23; q34]; %Posiciones 
qd_total  = [qd12; qd23; qd34]; %Velocidades 
qdd_total = [qdd12; qdd23; qdd34]; %Aceleraciones
%Graficar posición, velocidad y aceleración 
t = 1:length(q_total); 
figure; 
plot(t, q_total); 
title('Posición articular con frenado'); 
xlabel('Tiempo'); ylabel('Ángulo'); 
legend('q1','q2','q3','q4','q5','q6'); 
grid on; 
figure; 
plot(t, qd_total); 
title('Velocidad articular con frenado'); 
xlabel('Tiempo'); ylabel('Velocidad');
legend('q1','q2','q3','q4','q5','q6'); 
grid on; 
figure;
plot(t, qdd_total); 
title('Aceleración articular con frenado'); 
xlabel('Tiempo'); ylabel('Aceleración'); 
legend('q1','q2','q3','q4','q5','q6'); 
grid on;