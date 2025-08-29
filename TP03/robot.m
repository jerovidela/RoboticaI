% robot.m — Definición del brazo 6R (DH estándar, RTB clásico)
clear; clc; close all;
assert(exist('SerialLink','class')==8, 'Falta el Robotics Toolbox en el path.');

% === Longitudes genéricas (cambia por las del robot modelo real) ===
L1 = 0.35;   % altura columna/base
L2 = 0.30;   % brazo 1
L3 = 0.25;   % brazo 2
L4 = 0.20;   % desplazamiento muñeca
L5 = 0.10;   % espaciador muñeca
L6 = 0.10;   % brida/herramienta

% === Links: sintaxis clásica Link([theta d a alpha]) ===
L(1) = Link([0    L1   0     -pi/2]);
L(2) = Link([0    0    L2     0   ]);
L(3) = Link([0    0    L3     pi/2]);
L(4) = Link([0    L4   0     -pi/2]);
L(5) = Link([0    0    0      pi/2]);
L(6) = Link([0    L6   0      0   ]);

% Forzar DH estándar
for i=1:6, L(i).mdh = 0; end

% Tipo de articulación (0 = rotacional)
% for i=1:6, L(i).sigma = 0; end

% Offsets (ajusta si querés que q=0 sea “pose home” distinta)
for i=1:6, L(i).offset = 0; end

% Límites de ejemplo
L(1).qlim = deg2rad([-170 170]);
L(2).qlim = deg2rad([-170 170]);
L(3).qlim = deg2rad([-170 170]);
L(4).qlim = deg2rad([-185 185]);
L(5).qlim = deg2rad([-120 120]);
L(6).qlim = deg2rad([-350 350]);

% Crear robot
R = SerialLink(L, 'name', 'Escaner6R');

% Workspace para ploteo
workspace = [-0.9 0.9 -0.9 0.9 -0.1 1.6];

% Pose de prueba
q_home = deg2rad([0 -30 60 0 45 0]);
% figure; 
% R.plot(q_home, 'workspace', workspace, 'scale', 0.8, 'jointdiam', 1.1, 'notiles');
% title('Escaner6R - Pose de prueba');
% view(135,25); grid on; axis equal

% Variables al workspace
assignin('base','R',R);
assignin('base','workspace',workspace);
assignin('base','q_home',q_home);
