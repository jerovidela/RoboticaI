% main.m - Script principal de simulacion del escaneo
%
% Resumen:
% - Configura el robot (robot_G11) y parametros del escaneo.
% - Agrega un movimiento de posicionamiento desde una pose "home"
%   hasta el inicio del escaneo para suavizar aceleraciones iniciales.
% - Corrige la cinematica elevando la placa a z_scan = 0.35 m.
% - Genera la trayectoria completa (posicionamiento + escaneo).
% - Renderiza graficos de verificacion y produce una animacion opcional.
%
% Entradas: (ninguna; parametrizacion definida en este script)
% Salidas:  (ninguna; genera figuras/archivos en la carpeta Pic)

clear functions; rehash; clc;

proj = fileparts(mfilename('fullpath'));  % carpeta del proyecto
addpath(proj);                            % agrega SOLO el padre de +traj y +graphs
rehash;


% ====================================================================
% VERSIÓN 5
%
% Esta función soluciona DOS problemas:
% 1. AÑADE UN MOVIMIENTO DE POSICIONAMIENTO:
%    Genera un 'mstraj' desde una pose 'home' hasta el inicio
%    del escaneo ('q_init') para eliminar el pico de aceleración inicial.
% 2. CORRIGE LA CINEMÁTICA: Sube la placa a z_scan = 0.35 m.
%    La altura anterior (0.20 m) era inalcanzable y causaba la
%    oscilación en Z.
% ====================================================================

% --- Setup mínimo
[R, plotopt] = robot_G11();
R.base = eye(4);

% --- Pose "Home" (usaremos la de la demo de robot_G11.m)
q_home = deg2rad([0 90 170 90 30 70 ]); %

% --- Parámetros del scan (CORRECCIÓN DE ALTURA)
cfg = struct( ...
    'W',       0.40, ...
    'H',       0.30, ...
    'm',       0.01, ...
    'delta',   0.02, ...
    'z_scan',  0.35, ... 
    'h_clear', 0.03, ...
    'v_scan',  0.06, ...
    'a_scan',  0.60, ...
    'dt',      0.01, ...
    'R_orient', trotx(pi), ...
    'Vj_ut',   [0.7 0.7 0.7 1.4 1.4 1.4], ...
    'tacc_ut', 0.15 ...
);

% --- Posición de la placa
xc = 0.45; yc = 0.00; zc = cfg.z_scan; % zc ahora es 0.35
cfg.T_plate = transl(xc - cfg.W/2, yc - cfg.H/2, 0);

% --- Calcular q_init (el inicio del escaneo)
xs_0 = cfg.m; y_0  = cfg.m;
T_start = cfg.T_plate * transl(xs_0, y_0, cfg.z_scan) * cfg.R_orient;
q_init = R.ikcon(T_start, q_home); % Usamos q_home como semilla

% --- 1) GENERAR MOVIMIENTO DE POSICIONAMIENTO
fprintf('Generando movimiento de posicionamiento (Home -> q_init)...\n');
t_position = 2.0; % s (tiempo para el movimiento inicial)
tacc_pos = 0.5;   % s (aceleración suave)
V_pos = abs(q_init - q_home) / t_position * 1.5; % Velocidad estimada
V_pos = max(V_pos, [0.7 0.7 0.7 1.4 1.4 1.4]);  % Mínima velocidad

% Genera el tramo de "posicionamiento"
[q_position, t_position] = mstraj([q_home; q_init], V_pos, [], q_home, cfg.dt, tacc_pos);
t_now = t_position(end);

% --- 2) GENERAR TRAYECTORIA DE ESCANEO
fprintf('Calculando trayectoria de escaneo (orquestador)...\n');
% Llamamos al orquestador PASÁNDOLE el q_init
[q_scan, t_scan, scan_log] = traj.scan_orchestrator(R, cfg, q_init);

% --- 3) UNIR TODO
q_total = [q_position; q_scan];
t_total = [t_position; t_now + t_scan]; % Importante sumar el offset de tiempo

fprintf('Trayectoria completa generada. Duración: %.2f s\n', t_total(end));

% --- Ajustar 'scan_log' para reflejar el offset de tiempo y puntos
k_offset = size(q_position, 1);
for i = 1:numel(scan_log)
    scan_log(i).k0 = scan_log(i).k0 + k_offset;
    scan_log(i).k1 = scan_log(i).k1 + k_offset;
end

% ========= G R Á F I C O S  D E  C H E Q U E O =========
outdir = fullfile(proj,'Pic'); if ~exist(outdir,'dir'), mkdir(outdir); end
saveopts = struct('save',true,'dpi',300,'close',false,'output_dir', outdir);
files = graphs.render_profiles(R, q_total, t_total, "Placa", saveopts);

graphs.panelA_global(R, q_total, t_total, scan_log, cfg.v_scan, saveopts);
fprintf('Gráficos generados.\n');

% ========= A N I M A C I Ó N =========
cfgsim = struct( ...
    'ws', [-0.5 0.8 -0.5 0.8 0 1], ...
    'fps', 30, ...
    'trail_color', [0 0 0], ...
    'trail_width', 1.5, ...
    'plate', struct('center',[xc yc zc], 'size',[cfg.W cfg.H], 'color',[0.72 0.75 0.78]), ...
    'speed_ref', cfg.v_scan, ...
    'jac_metric', 'cond', ...
    'jac_cond_max', 1000, ...
    'maximize', true, ...
    'save_video', true, ...
    'video_file', fullfile(outdir,'scan') ...   % sin extensión, el helper decide
);
% Usá el plotopt "oficial" y solo agregá flags que quieras:
cfgsim.plotopt = [plotopt, {'nowrist','noshadow','noname','delay',0}];

% Blindaje extra por si las moscas
if ~isscalar(cfgsim), cfgsim = cfgsim(1); end

fprintf('Iniciando animación...\n');
simulation.animate_path(R, q_total, t_total, cfgsim, scan_log);

fprintf('Simulación completada.\n');


