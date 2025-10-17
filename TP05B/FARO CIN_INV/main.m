clear; clc; close all;
robot      


q=rand(1,6);
T = R.fkine(q).double  
q0 = zeros(1,6);   
Q = cin_inv_Faro(R, T, q0, true); 
disp('Solución encontrada:')
disp(Q)   
R.plot(Q');
