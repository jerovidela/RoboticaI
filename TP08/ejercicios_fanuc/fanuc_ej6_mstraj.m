% FANUC Ejercicio 6 - mstraj multipunto con límites de velocidad
clear; clc; close all; 
dh = [ 
0, 0.45, 0.075, -pi/2, 0; 
0, 0, 0.3, 0, 0; 
0, 0, 0.075, -pi/2, 0; 
0, 0.32, 0, pi/2, 0; 
0, 0, 0, -pi/2, 0; 
0, 0.008, 0, 0, 0 
]; 
fanuc_robot = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA'); 
q1 = [0, -pi/2, -pi/2, 0, pi/2, 0]; 
q2 = [pi/2, -pi/2, -pi/2, 0, pi/2, 0]; 
q3 = [pi/6, -pi/6, 0, 0, pi/6, 0]; 
q4 = [pi/3, -pi/4, pi/6, 0, pi/3, 0];
puntos = [q1; q2; q3; q4]; 
dt = 0.1; 
qdmax = [0.2 0.2 0.2 0.2 0.2 0.2]; 

q_total = mstraj(puntos, qdmax, [], q1, dt, 0); 
qd_total = [zeros(1,6); diff(q_total)/dt]; 
qdd_total = [zeros(1,6); diff(qd_total)/dt];

t = 1:length(q_total); 
figure; plot(t, q_total); title('Posición articular'); xlabel('Tiempo'); ylabel('Ángulo'); grid on; legend('q1','q2','q3','q4','q5','q6'); 
figure; plot(t, qd_total); title('Velocidad articular'); xlabel('Tiempo'); ylabel('Velocidad'); grid on; legend('q1','q2','q3','q4','q5','q6'); 
figure; plot(t, qdd_total); title('Aceleración articular'); xlabel('Tiempo'); ylabel('Aceleración'); grid on; legend('q1','q2','q3','q4','q5','q6');

