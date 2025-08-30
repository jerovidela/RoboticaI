% robot.m — Definición del ScanArm 6GDL (DH estándar, RTB clásico)
clear; clc; close all;

%% ===== 0) RTB en el path (ajusta carpeta si hace falta) =====
if exist('SerialLink','class') ~= 8
    baseDir = fileparts(mfilename('fullpath'));
    rtbGuess = fullfile(baseDir, 'rvctools'); % cambia si tu ruta es otra
    if exist(rtbGuess, 'dir'), addpath(genpath(rtbGuess)); end
end
assert(exist('SerialLink','class')==8, 'No encuentro el Robotics Toolbox. Agrégalo al path.');

%% ===== 1) Selección de tamaño y geometría desde datasheet FARO =====
% Elegí el tamaño del brazo (en metros de diámetro nominal) y setea L1/L2:
size_m = 3.0;  % <-- 1.5, 2.5, 3.0, 3.5, 4.0, etc. según tu datasheet

switch size_m
    case 1.5
        L1 = 0.398; L2 = 0.213;
    case 2.5
        L1 = 0.629; L2 = 0.444;
    case 3.0
        L1 = 0.755; L2 = 0.569;   % ejemplo 3.0 m
    case 3.5
        L1 = 0.879; L2 = 0.694;
    case 4.0
        L1 = 1.004; L2 = 0.819;
    otherwise
        error('Tamaño %.1f m no mapeado. Completa L1 y L2 a mano.', size_m);
end

% Offsets opcionales
d_base = 0.0;   % m (altura de la base si querés elevar el origen)
d_tool = 0.0;   % m (longitud efectiva del palpador/soporte herramienta)

%% ===== 2) DH clásica propuesta (Link([theta d a alpha])) =====
% Convención didáctica consistente con hombro–codo–muñeca:
% a2 = L1, a3 = L2; resto de a_i = 0; todos σ=0 (rotacionales).
% α: {+90, 0, 0, -90, +90, 0} para desacoplar la muñeca.
L(1) = Link([0   d_base  0     +pi/2]);   % θ1
L(2) = Link([0   0       L1    0     ]);  % θ2
L(3) = Link([0   0       L2    0     ]);  % θ3
L(4) = Link([0   0       0     -pi/2 ]);  % θ4
L(5) = Link([0   0       0     +pi/2 ]);  % θ5
L(6) = Link([0   d_tool  0     0     ]);  % θ6

% Matriz DH (útil para inspección/guardar)
dh = [...
% i  theta   d         a      alpha      sigma
  1,  0,     d_base,   0,     +pi/2,     0;
  2,  0,     0,        L1,    0,         0;
  3,  0,     0,        L2,    0,         0;
  4,  0,     0,        0,     -pi/2,     0;
  5,  0,     0,        0,     +pi/2,     0;
  6,  0,     d_tool,   0,     0,         0];

%% ===== 3) Límites articulares (ajusta a tu uso/según hoja de datos) =====
% Valores genéricos razonables para simular (puedes afinarlos):
L(1).qlim = deg2rad([-170  170]);
L(2).qlim = deg2rad([-170  170]);
L(3).qlim = deg2rad([-170  170]);
L(4).qlim = deg2rad([-185  185]);
L(5).qlim = deg2rad([-120  120]);
L(6).qlim = deg2rad([-350  350]);

%% ===== 4) Base y Tool =====
% Base: por ejemplo, 0.8 m sobre el suelo
base = transl(0,0,0.8);
% Tool: sensor a 0.12 m de la brida, mirando “hacia abajo”
tool = transl(0,0,0.12) * troty(pi);

%% ===== 5) Objetos SerialLink =====
arm6 = SerialLink(L, 'name', sprintf('FARO_%0.1fm_6R', size_m), 'base', base, 'tool', tool);

% Variante 7 ejes: añade roll final puro
L7 = Link([0 0 0 0]);           % θ7, d=0, a=0, α=0
arm7 = SerialLink([L L7], 'name', sprintf('FARO_%0.1fm_7R', size_m), 'base', base, 'tool', tool);

%% ===== 6) Workspace =====
workspace = [-3 3  -3 3  0 3];

%% ===== 7) Poses de ejemplo =====
q_home = deg2rad([0 -30 60 0 45 0]);         % home ilustrativa
q_demo = deg2rad([30 -20 40 45 -45 90]);     % demo

T_home = arm6.fkine(q_home);
T_demo = arm6.fkine(q_demo);
disp('T_home ='); disp(T_home);
disp('T_demo ='); disp(T_demo);

figure('Name','ScanArm 6R - Pose demo');
arm6.plot(q_demo, 'workspace', workspace, 'scale', 0.8, 'jointdiam', 0.25, 'notiles');
view(135,25); grid on; axis equal

%% ===== 8) Exportar variables al base workspace =====
assignin('base','arm6',arm6);
assignin('base','arm7',arm7);
assignin('base','dh',dh);
assignin('base','workspace',workspace);
assignin('base','q_home',q_home);
assignin('base','L1',L1);
assignin('base','L2',L2);
assignin('base','size_m',size_m);
