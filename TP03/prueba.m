% robot.m — Definición del brazo 6GDL (DH estándar, RTB clásico)
clear; clc; close all;
% ===== 0) RTB en el path (ajusta carpeta si hace falta) =====
if exist('SerialLink','class') ~= 8
    % ejemplo: si tenés RVCTools en una carpeta vecina
    baseDir = fileparts(mfilename('fullpath'));
    rtbGuess = fullfile(baseDir, 'rvctools'); % cambia esto si tu ruta es otra
    if exist(rtbGuess, 'dir'), addpath(genpath(rtbGuess)); end
end
assert(exist('SerialLink','class')==8, 'No encuentro el Robotics Toolbox. Agrégalo al path.');

% ===== 1) Geometría genérica (cambiá por la del robot modelo real) =====
L1 = 0.35;   % altura columna
L2 = 0.30;   % brazo 1
L3 = 0.25;   % brazo 2
L4 = 0.20;   % desplaz. muñeca
L6 = 0.10;   % brida / TCP

% ===== 2) Matriz DH estándar: [theta d a alpha sigma] =====
% sigma = 0 (rotacional) por defecto; pon 1 si alguna es prismática.
dh = [
% theta   d     a     alpha    sigma
   0      L1    0     -pi/2    0    % 1
   0      0     L2     0       0    % 2
   0      0     L3     pi/2    0    % 3
   0      L4    0     -pi/2    0    % 4
   0      0     0      pi/2    0    % 5
   0      L6    0      0       0    % 6
];

% ===== 3) Construcción de Links (RTB clásico: Link([theta d a alpha [sigma [offset]]])) =====
n = size(dh,1);
L(1,n) = Link();
for i = 1:n
    row = dh(i,:);
    if numel(row) == 5
        L(i) = Link(row(1:5));     % incluye sigma
    else
        L(i) = Link(row(1:4));     % sin sigma -> rotacional
    end
    L(i).mdh = 0;                  % forzar DH estándar
    L(i).offset = 0;               % ajustá si tu “cero” mecánico es otro
    % Límites articulares genéricos (rad); cambiá por los reales del modelo
    switch i
        case {1,2,3}
            L(i).qlim = deg2rad([-170 170]);
        case 4
            L(i).qlim = deg2rad([-185 185]);
        case 5
            L(i).qlim = deg2rad([-120 120]);
        case 6
            L(i).qlim = deg2rad([-350 350]);
    end
end

% ===== 4) Base y Tool =====
% Base: por ejemplo, 0.8 m sobre el suelo
base = transl(0,0,0.8);
% Tool: sensor ubicado a 0.12 m de la brida, mirando “hacia abajo” (ajusta a tu sensor real)
tool = transl(0,0,0.12) * troty(pi);

% ===== 5) Objeto SerialLink =====
R = SerialLink(L, 'name', 'Escaner6R', 'base', base, 'tool', tool);

% ===== 6) Workspace =====
% workspace = [-0.9 0.9  -0.9 0.9  -0.1 1.6];
workspace = [-3 3  -3 3  0 3];


% ===== 7) Visualización rápida opcional =====
q_home = deg2rad([0 -30 60 0 45 0]);
% figure('Name','Robot - Pose de prueba');
% R.plot(q_home, 'workspace', workspace, 'scale', 0.8, 'jointdiam', 1.1, 'notiles');
% view(135,25); grid on; axis equal

% ===== 8) Dejar variables accesibles =====
assignin('base','R',R);
assignin('base','dh',dh);
assignin('base','workspace',workspace);
assignin('base','q_home',q_home);
