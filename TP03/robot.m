% robot.m — Definición del brazo 6R (DH estándar, RTB clásico)
clear; clc; close all;
assert(exist('SerialLink','class')==8, 'Falta el Robotics Toolbox en el path.');

% === Longitudes Genéricos ===

% L1 = 0.283;   % altura columna/base
% L2 = 0.629;   % brazo 1 (En el datasheet es L1)
% L3 = 0.444;   % brazo 2 (En el datasheet es L2)
% L4 = 0.220;   % desplazamiento muñeca
% L5 = 0.10;   % espaciador muñeca
% L6 = 0.10;   % brida/herramienta
% 
% dh = [
%     0   L1   0    -pi/2   0  
%     0   0    L2    0      0
%     0   0    L3    pi/2   0
%     0   L4   0    -pi/2   0
%     0   0    0     pi/2   0
%     0   L6   0     0      0
% ];


% === Longitudes del ScanArm (Quantum 3.0 m) === %
%%%%%%% MUÑECA ESFÉRICA %%%%%%%%
% L1 = 0.755;   % tubo 1 (brazo superior)
% L2 = 0.569;   % tubo 2 (antebrazo)
% L3 = 0;       % no hay un "brazo 3" largo
% L4 = 0;       % offset muñeca (pequeño, lo podés dejar en 0)
% L5 = 0;       % espaciador muñeca
% L6 = 0.10;    % herramienta (palpador)
% 
% dh = [
%    0   0   0     +pi/2
%    0   0   L1    0
%    0   0   L2    0
%    0   0   0     -pi/2
%    0   0   0     +pi/2
%    0   L6  0     0
% ];

%%%%% MUÑECA CON 2 DOF %%%%%%

L1 = 0.755;    % tubo 1 (brazo)
L2 = 0.569;    % tubo 2 (antebrazo)
d_base = 0.0;  % opcional
d_tool = 0.12; % opcional (palpador)

% --- DH 6R: 2 ejes en muñeca (θ5, θ6) ---
dh = [
   0   d_base   0     +pi/2
   0   0   L1    0
   0   0   0    +pi/2
   0   0   L2     0
   0   0   0     -pi/2
   0   d_tool  0     0
];

% % % === Links: sintaxis clásica Link([theta d a alpha]) ===
% % L(1) = Link([0    L1   0     -pi/2]);
% % L(2) = Link([0    0    L2     0   ]);
% % L(3) = Link([0    0    L3     pi/2]);
% % L(4) = Link([0    L4   0     -pi/2]);
% % L(5) = Link([0    0    0      pi/2]);
% % L(6) = Link([0    L6   0      0   ]);
% % 
% % %%% Forzar DH estándar %%%
% % for i=1:6, L(i).mdh = 0; end
% % 
% % % Tipo de articulación (0 = rotacional)
% % % Por alguna razón crashea
% % % for i=1:6, L(i).sigma = 0; end
% % 
% % % === Offsets === %
% % for i=1:6, L(i).offset = 0; end


% === Construcción de Links DESDE dh (RTB clásico) ===
n = size(dh,1); L(1,n)=Link();
for i=1:n
    r = dh(i,:);
    if numel(r)==5 && r(5)==1
        L(i) = Link([r(1) r(2) r(3) r(4) 1]);   % prismática
    else
        L(i) = Link([r(1) r(2) r(3) r(4)]);     % rotacional
    end
    L(i).mdh = 0;
    L(i).offset = 0;             % ajustá si tu cero mecánico es otro
end

% === Límites === %

L(1).qlim = deg2rad([-170 170]);
L(2).qlim = deg2rad([-170 170]);
L(3).qlim = deg2rad([-170 170]);
L(4).qlim = deg2rad([-185 185]);
L(5).qlim = deg2rad([-120 120]);
L(6).qlim = deg2rad([-350 350]);

% === Crear robot === %
R = SerialLink(L, 'name', 'Escaner6R');

% Workspace para ploteo
workspace = [-0.9 0.9 -0.9 0.9 -0.1 1.6];

% === Pose de prueba === %

q_home = deg2rad([0 -30 60 0 45 0]);
% figure; 
% R.plot(q_home, 'workspace', workspace, 'scale', 0.8, 'jointdiam', 1.1, 'notiles');
% title('Escaner6R - Pose de prueba');
% view(135,25); grid on; axis equal

% Variables al workspace

assignin('base','R',R);
assignin('base','workspace',workspace);
assignin('base','dh',dh);
