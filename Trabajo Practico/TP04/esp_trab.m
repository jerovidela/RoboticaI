clear; clc; close all;

%% --- Cargar definición del robot
robot; % crea el objeto R

%% --- Workspace XZ (tramos separados)
q6 = deg2rad(180);
q5 = deg2rad(-90);
q4 = deg2rad(0);
q3 = deg2rad(0);
q2= deg2rad(120);
q1 = deg2rad(0);

% Tramo 1: q3 de -10 a -90
q3_range = deg2rad(-10:-5:-90);
ptsXZ_tramo1 = [];
for q3 = q3_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo1(end+1,:) = T.t';
end

% Tramo 2: q2 de 120 a 0
q2_range = deg2rad(120:-5:-15);
ptsXZ_tramo2 = [];
for q2 = q2_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo2(end+1,:) = T.t';
end
% Tramo 3: q6 de -180 a 0
q6_range = deg2rad(-180:15:0);
ptsXZ_tramo3 = [];
for q6 = q6_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo3(end+1,:) = T.t';
end

% Tramo 4: q2 de 0 a -120
q2_range = deg2rad(0:-5:-120);
ptsXZ_tramo4 = [];
for q2 = q2_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo4(end+1,:) = T.t';
end
% Tramo 5: q3 de -90 a -170
q3_range = deg2rad(-90:-5:-170);
ptsXZ_tramo5 = [];
for q3 = q3_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXZ_tramo5(end+1,:) = T.t';
end
% Unir tramos sin duplicar puntos
ptsXZ = [ptsXZ_tramo1; ptsXZ_tramo2; ptsXZ_tramo3; ptsXZ_tramo4; ptsXZ_tramo5];

% Graficar XZ
figure;
fill(ptsXZ(:,1), ptsXZ(:,3), 'r', 'FaceAlpha',0.3, 'EdgeColor','r','LineWidth',1.2);
grid on; axis equal;
xlabel('X [m]'); ylabel('Z [m]');
title('Area de trabajo del workspace XZ');

%% --- Workspace XY (tramos separados)
q2 = deg2rad(90);
q3 = deg2rad(-90);
q4 = deg2rad(0);
q5 = deg2rad(-90);
q6 = deg2rad(270);

%Tramo 0: q4 de -50 a 0
q4_range = deg2rad(-50:5:0);
ptsXY_tramo4 = [];
for q4 = q4_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXY_tramo4(end+1,:) = T.t';
end

% Tramo 1: q1 de 170 a -170
q1_range = deg2rad(170:-5:-170);
ptsXY_tramo1 = [];
for q1 = q1_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXY_tramo1(end+1,:) = T.t';
end

% Tramo 2: q6=270 a 90
q6_range = deg2rad(270:-5:90);
ptsXY_tramo2 = [];
for q6 = q6_range
    q = [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXY_tramo2(end+1,:) = T.t';
end

% Tramo 3: q4 de 0 50
q4_range = deg2rad(0:5:50);
ptsXY_tramo3 = [];
for q4 = q4_range
    q= [q1 q2 q3 q4 q5 q6];
    T = R.fkine(q);
    ptsXY_tramo3(end+1,:)= T.t';
end
% Unir tramos
ptsXY = [ptsXY_tramo1;ptsXY_tramo2];

%No se unen los tramos 4 y 3 porque distorsionan el codigo, pero es
%practcamente la misma fomrma

% --- Definir hueco circular ---
%dam = 0.1; % diámetro [m]
%r = dam/2;
%theta = linspace(0,2*pi,200);
%circle = [r*cos(theta') r*sin(theta')];

% Graficar área XY con hueco
figure;
fill(ptsXY(:,1), ptsXY(:,2), 'b', 'FaceAlpha',0.3, 'EdgeColor','b','LineWidth',1.2); hold on;
%fill(circle(:,1), circle(:,2), 'w', 'EdgeColor','k','LineWidth',1); % hueco central
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Área de trabajo XY ');

% Aclaraciones sobre el area de trabajo del robot: como usamos el simulador
% plot_robot para ver hasta donde puede llegar la punta del efector final,
% este nos permite sobre poner los ejes, permitiendo que llegue incluso al
% centro de la base, en la realidad esto no es posible, por lo que tomando
% en cuenta las dimenciones de diametro de los ejes hemos extraido un area
% de trabajo que no es posible en la realidad.
