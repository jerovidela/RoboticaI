clear; clc; close all;
robot      


% Matriz de posición y orientación deseada 
T = transl(0.5, -0.2, 0.2) * trotx(0);   
% Configuración inicial 
q0 = zeros(1,6);   


Q = cin_inv_Faro(R, T, q0, true); 

disp('Solución encontrada:')
disp(Q)   

% --- Comprobación gráfica ---
R.plot(Q');
