% Definir los parámetros DH del robot
L(1) = Link('d', 0.340, 'a', 0, 'alpha', -pi/2);
L(2) = Link('d', 0, 'a', 0, 'alpha', pi/2);
L(3) = Link('d', 0.400, 'a', 0, 'alpha', pi/2);
L(4) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
L(5) = Link('d', 0.400, 'a', 0, 'alpha', -pi/2);
L(6) = Link('d', 0, 'a', 0, 'alpha', pi/2);
L(7) = Link('d', 0, 'a', 0, 'alpha', 0);

% Creamos el modelo del robot:
iiwa = SerialLink(L, 'name', 'KUKA LBR iiwa 7 R800');

%  Matriz de transformacion deseada
T = [1 0 0 0.23;
     0 1 0 0.70;
     0 0 1 0.60;
     0 0 0 1];

%Definimos 3 semillas distintas
q0_1 = zeros(1,7);         %todas en cero
q0_2 = pi/4 * ones(1,7);  %todas a 45
q0_3 = -pi/4 * ones(1,7);  %todas a -45


%usamos ikine para calcular la solucion
q1 = iiwa.ikine(T, q0_1, [1 1 1 0 0 0]);
q2 = iiwa.ikine(T, q0_2, [1 1 1 0 0 0]);
q3 = iiwa.ikine(T, q0_3, [1 1 1 0 0 0]);



% mostramos las soluciones:
disp('Solución 1:');
disp(q1);
disp('Solución 2:');
disp(q2);
disp('Solución 3:');
disp(q3);

% Verificar usando fkine
T1 = iiwa.fkine(q1);
T2 = iiwa.fkine(q2);
T3 = iiwa.fkine(q3);

disp('Matriz de transformación para q1:');
disp(T1);
disp('Matriz de transformación para q2:');
disp(T2);
disp('Matriz de transformación para q3:');
disp(T3);
