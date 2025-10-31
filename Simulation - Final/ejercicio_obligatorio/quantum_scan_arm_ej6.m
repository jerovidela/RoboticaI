% TP8 - Ejercicio Obligatorio: Planificación y Generación de Trayectorias
% Escaneo de una placa rectangular (plano XZ) con dos enfoques:
%   Solución A: Interpolación en espacio articular (jtraj).
%   Solución B: Interpolación cartesiana en el tramo de escaneo (ctraj) + IK.
%
% Requisitos del entorno: MATLAB + Robotics Toolbox (Peter Corke)
% Integraciones: usa tp8/robot.m, tp8/cin_inv_Faro.m y tp8/helpers/RoboticaUtils.m

clear; clc; close all;

%% Paths y carga de robot
thisDir = fileparts(mfilename('fullpath'));
tpDir   = fileparts(thisDir);              % carpeta padre
addpath(tpDir, fullfile(tpDir,'helpers')); % helpers para IK (si existen en padre)

% Cargar robot definido para el grupo (evita inconsistencias de offset)
if exist(fullfile(thisDir,'robot_G11.m'),'file') == 2
    R = robot_G11();
else
    error('No se encuentra robot_G11.m en ejercicio_obligatorio');
end
fprintf('Robot "%s" cargado. GDL=%d\n', R.name, R.n);

% Fix mínimo: asegurar offset como columna 6x1 para IK
R.offset = R.offset(:);

%% Parámetros de tarea y discretización
dt = 0.05;              % paso temporal [s]
t_aprox   = 3.0;        % duración del acercamiento [s]
t_scan    = 4.0;        % duración del escaneo [s]
t_retract = 3.0;        % duración del retorno [s]

N1 = max(2, round(t_aprox/dt));
N2 = max(2, round(t_scan/dt));
N3 = max(2, round(t_retract/dt));

%% Definir waypoints cartesianos de la placa (plano XZ, Y constante)
% Elegidos dentro del alcance geométrico del modelo
y_plate = 0.00;              % placa en plano XZ (Y≈0)
standoff = 0.00;             % sin offset extra

% Orientación: eje z de la herramienta hacia -Y (normal al plano XZ)
Rtool = rotx(-pi/2);

% Poses
T_approach = rt2tr(Rtool, [0.30; 0.02; 0.25]);   % por encima, leve +Y
T_start    = rt2tr(Rtool, [0.25; y_plate+standoff; 0.15]);
T_end      = rt2tr(Rtool, [0.45; y_plate+standoff; 0.15]);
T_retract  = rt2tr(Rtool, [0.35; 0.03; 0.25]);

% Semilla inicial para IK
q_seed = ones(6,1);

% IK para waypoints (A y extremos B)
q_app    = cin_inv_Faro(R, T_approach, q_seed, true);
q_start  = cin_inv_Faro(R, T_start,    q_app,  true);
q_end    = cin_inv_Faro(R, T_end,      q_start,true);
q_retr   = cin_inv_Faro(R, T_retract,  q_end,  true);

%% SOLUCIÓN A: Interpolación articular (quintica)
t1 = linspace(0, t_aprox,   N1);
t2 = linspace(0, t_scan,    N2);
t3 = linspace(0, t_retract, N3);

[qA1, qdA1, qddA1] = jtraj(q_app.',   q_start.', N1);
[qA2, qdA2, qddA2] = jtraj(q_start.', q_end.',   N2);
[qA3, qdA3, qddA3] = jtraj(q_end.',   q_retr.',  N3);

qA   = [qA1;  qA2(2:end,:);  qA3(2:end,:)];
qdA  = [qdA1; qdA2(2:end,:); qdA3(2:end,:)];
qddA = [qddA1;qddA2(2:end,:);qddA3(2:end,:)];
tA   = [t1, (t1(end)+t2(2:end)), (t1(end)+t2(end)+t3(2:end))];

posA = zeros(size(qA,1),3);
for i=1:size(qA,1)
    Tcurr = R.fkine(qA(i,:));
    posA(i,:) = Tcurr.t';
end
velA = RoboticaUtils.derivada(posA, tA');
accA = RoboticaUtils.derivada(velA, tA');

%% SOLUCIÓN B: Escaneo cartesiano + IK, transiciones articulares
[qB1, ~, ~] = jtraj(q_app.', q_start.', N1);
TB = ctraj(SE3(T_start), SE3(T_end), N2);
qB2 = zeros(N2, R.n); q_prev = q_start;
for i = 1:N2
    Ti = TB(i);
    qi = cin_inv_Faro(R, Ti.T, q_prev, true);
    qB2(i,:) = qi.'; q_prev = qi;
end
[qB3, ~, ~] = jtraj(q_end.', q_retr.', N3);
qB  = [qB1; qB2(2:end,:); qB3(2:end,:)];
tB  = tA;
qdB = RoboticaUtils.derivada(qB,  tB');
qddB= RoboticaUtils.derivada(qdB, tB');

posB = zeros(size(qB,1),3);
for i=1:size(qB,1)
    Tcurr = R.fkine(qB(i,:));
    posB(i,:) = Tcurr.t';
end
velB = RoboticaUtils.derivada(posB, tB');
accB = RoboticaUtils.derivada(velB, tB');

%% Animaciones
figA = figure('Name','Animación Método A (jtraj)');
try, R.plot(qA, 'delay', dt, 'trail', '-'); catch, R.plot(qA); end
maximize_and_wait(figA);

figB = figure('Name','Animación Método B (ctraj + IK)');
try, R.plot(qB, 'delay', dt, 'trail', '-'); catch, R.plot(qB); end
maximize_and_wait(figB);

%% Gráficos (resumen corto)
outDir = fullfile(thisDir,'figs'); if ~exist(outDir,'dir'); mkdir(outDir); end
cols = lines(R.n); legq = arrayfun(@(k) sprintf('q%d',k), 1:R.n, 'UniformOutput', false);
f1 = figure('Name','Evolución articular (grados)'); hold on; grid on;
for k=1:R.n
    plot(tA, rad2deg(qA(:,k)), '--', 'Color', cols(k,:), 'DisplayName',[legq{k} ' A']);
    plot(tB, rad2deg(qB(:,k)), '-',  'Color', cols(k,:), 'DisplayName',[legq{k} ' B']);
end
xlabel('Tiempo [s]'); ylabel('Ángulo [deg]'); title('Evolución Articular'); legend('Location','bestoutside');
maximize_and_wait(f1);

%% Utilidad local: maximizar y esperar botón
function maximize_and_wait(fig)
    try, set(fig, 'WindowState','maximized'); catch, set(fig, 'Units','normalized', 'OuterPosition',[0 0 1 1]); end
    btn = uicontrol('Parent',fig, 'Style','pushbutton', 'String','Continuar', ...
                    'Units','normalized', 'Position',[0.86 0.02 0.12 0.06], ...
                    'FontSize',12, 'Callback',@(src,evt) uiresume(fig)); %#ok<NASGU>
    try, uiwait(fig); catch, end
end

