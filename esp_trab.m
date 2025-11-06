clear; clc; close all;

%% Notas

% Tanto en las graficas de los planos XZ como XY, se realizan saltos entre
% el punto de inicio y fin, pero estos se encuentran en una zona de trabajo
% no deseada y son mínimos, por lo que se ignoran las discontinuidades.

% Aclaraciones sobre el area de trabajo del robot: como usamos el simulador
% plot_robot para ver hasta donde puede llegar la punta del efector final,
% este nos permite poner el actuador final sobre los ejes, permitiendo que 
% llegue incluso al centro de la base. En la realidad esto no es posible y
% se debe tener cuidado de no dar coordenadas articulares que causen una
% colisión del robot

% En el plano XZ, se realizó un ultimo movimiento el cual permite 
% visualizar mejor la zona de trabajo efectiva del robot, pero no
% representa la capacidad de movimiento total del robot. Si se desea verla,
% setear el trigger de salto a 0.

% Se recomienda setear el factor de seguridad para limites articulares a
% 0.9 tal que no se fuerze a los motores cerca de su limites. 

%% --- Cargar definición del robot
robot; % crea el objeto R

%% Factor de seguridad para limites articulares
f_seg=1;

%% Trigger de salto
trigger=1;

%% --- Workspace XZ (tramos separados)
qlim = f_seg * qlim;

q6 = 0;
q5 = 0;
q1 = 0;

% Tramo 1 XZ (variamos q4 lim_inf->0)
q2 = qlim(2,1);
q3 = qlim(3,1);
q4 = qlim(4,1); 
step = -q4/20;
q4_range = (q4:step:0);
ptsXZ_tramo1 = [];
for q4 = q4_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo1(end+1,:) = T.t';
end

% Tramo 2 XZ (variamos q3 lim_inf->0)
step = -q3/30;
q3_range = (q3:step:-0);
ptsXZ_tramo2 = [];
for q3 = q3_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo2(end+1,:) = T.t';
end
% Tramo 3 XZ (variamos q2 lim_inf->lim_sup)
q2_end = qlim(2,2);
step = (q2_end-q2)/100;
q2_range = (q2:step:q2_end);
ptsXZ_tramo3 = [];
for q2 = q2_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo3(end+1,:) = T.t';
end

% Tramo 4 XZ (variamos q3 0->lim_sup)
q3_end = qlim(3,2);
step = (q3_end-q3)/30;
q3_range = (q3:step:q3_end);
ptsXZ_tramo4 = [];
for q3 = q3_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo4(end+1,:) = T.t';
end
% Tramo 5 XZ (variamos q4 0->lim_sup)
q4_end = qlim(4,2);
step = (q4_end-q4)/20;
q4_range = (q4:step:q4_end);
ptsXZ_tramo5 = [];
for q4 = q4_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo5(end+1,:) = T.t';
end

% Tramo 6 XZ (variamos q2 lim_sup->lim_inf)
q2_end = qlim(2,1);
step = (q2_end-q2)/60;
q2_range = (q2:step:q2_end);
ptsXZ_tramo6 = [];
for q2 = q2_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo6(end+1,:) = T.t';
end

% Unir tramos sin duplicar puntos
ptsXZ = [ptsXZ_tramo1; ptsXZ_tramo2; ptsXZ_tramo3; ptsXZ_tramo4; ptsXZ_tramo5];
if trigger == 1
    ptsXZ = [ptsXZ;ptsXZ_tramo6];
end    

% Graficar XZ
figure;
fill(ptsXZ(:,1), ptsXZ(:,3), 'r', 'FaceAlpha',0.3, 'EdgeColor','r','LineWidth',1.2);
grid on; axis equal;
xlabel('X [m]'); ylabel('Z [m]');
title('Area de trabajo del workspace XZ');

%% --- Workspace XY (tramos separados)

q1 = qlim(1,2);
q2 = 0;
q3 = 0;
q4 = 0;

q6 = 0;

% Tramo 1 XY (variamos q5 lim_sup->0)
q5 = qlim(5,2);
step = -q5/20;
q5_range = (q5:step:0);
ptsXY_tramo1 = [];
for q5 = q5_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXY_tramo1(end+1,:) = T.t';
end

% Tramo 2 XY (variamos q1 lim_sup->lim_inf)
q1_end = qlim(1,1);
step = (q1_end-q1)/120;
q1_range = (q1:step:q1_end);
ptsXY_tramo2 = [];
for q1 = q1_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXY_tramo2(end+1,:) = T.t';
end

% Tramo 3 XY (variamos q5 0->lim_inf)
q1 = q1_end;
q5_end = qlim(5,1);
step = (q5_end-q5)/20;
q5_range = (q5:step:q5_end);
ptsXY_tramo3 = [];

for q5 =  q5_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXY_tramo3(end+1,:) = T.t';
end

% Al igual que en el plano XZ, se realiza un salto entre el punto de inicio
% y fin, pero este se encuentra en una zona de trabajo no deseada y es 
% mínimo, por lo que se ignora la discontinuidad

% Unir tramos
ptsXY = [ptsXY_tramo1;ptsXY_tramo2;ptsXY_tramo3];


% Graficar área XY 
figure;
fill(ptsXY(:,1), ptsXY(:,2), 'b', 'FaceAlpha',0.3, 'EdgeColor','b','LineWidth',1.2); hold on;
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Área de trabajo XY ');


