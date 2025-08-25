% Requiere Robotics Toolbox for MATLAB (Peter Corke)
% https://petercorke.com/toolboxes/robotics-toolbox/

clear; clc; close all

% Utilidades (grados a rad)
d2r = @(d) deg2rad(d);

% ----- Ejercicio 5: TH para cada secuencia (movimientos en el marco M -> postmultiplicar)
TMa = troty(d2r(45)) * transl(0,0,1);  % (a) rot(Y_M,45) luego trasl p_M
TMb = transl(0,0,1) * troty(d2r(45));  % (b) trasl p_M luego rot(Y_M,45)

disp('T_O_M (a) ='); disp(TMa)
disp('T_O_M (b) ='); disp(TMb)

% ----- Gráfico para apreciar la diferencia
figure; hold on; grid on; axis equal
trplot(eye(4), 'frame','O', 'length',0.6);
trplot(TMa, 'frame','M_a','color','r','length',0.6);
trplot(TMb, 'frame','M_b','color','g','length',0.6);
view(135,20); xlabel('X'); ylabel('Y'); zlabel('Z');
title('Ej.5: Diferencia entre T_O_M (a) y (b)')

% ----- Ejercicio 6: expresar p_O en cada M
pO = [0.5; 0; 1; 1];  % punto homogéneo

pMa = TMa\pO;
pMb = TMb\pO;

fprintf('\nEj.6: p_O = [0.5 0 1]^T\n');
fprintf('  p_M^(a) = [%.6f  %.6f  %.6f]^T\n', pMa(1:3));
fprintf('  p_M^(b) = [%.6f  %.6f  %.6f]^T\n', pMb(1:3));

% Visual rápido del punto en O y sus proyecciones en M_a y M_b
plot3(pO(1), pO(2), pO(3), 'ko', 'MarkerSize',8,'LineWidth',1.5);
text(pO(1), pO(2), pO(3), '  p_O','VerticalAlignment','bottom');

% reconstruyo el punto visto desde O que corresponde a cada p_M (debería volver a p_O)
pO_from_a = TMa * pMa; %#ok<NASGU>
pO_from_b = TMb * pMb; %#ok<NASGU>
