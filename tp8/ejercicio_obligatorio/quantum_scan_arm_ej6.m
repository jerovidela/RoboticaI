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
tp8Dir  = fileparts(thisDir);              % .../tp8
addpath(tp8Dir, fullfile(tp8Dir,'helpers'));   % helpers para IK

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
% Elegidos dentro del alcance geométrico del modelo (L2≈0.40, L3≈0.21, d6≈0.22)
y_plate = 0.00;              % placa en plano XZ (Y≈0)
standoff = 0.00;             % sin offset extra (se puede ajustar)

% Orientación: eje z de la herramienta hacia -Y (normal al plano XZ)
Rtool = rotx(-pi/2);

% Poses escogidas dentro del alcance (L2+L3≈0.611 m)
T_approach = rt2tr(Rtool, [0.30; 0.02; 0.25]);   % por encima, leve +Y
T_start    = rt2tr(Rtool, [0.25; y_plate+standoff; 0.15]);
T_end      = rt2tr(Rtool, [0.45; y_plate+standoff; 0.15]);
T_retract  = rt2tr(Rtool, [0.35; 0.03; 0.25]);

% Semilla inicial para IK
q_seed = ones(6,1);

% IK para waypoints (para Solución A y para extremos B)
q_app    = cin_inv_Faro(R, T_approach, q_seed, true);
q_start  = cin_inv_Faro(R, T_start,    q_app,  true);
q_end    = cin_inv_Faro(R, T_end,      q_start,true);
q_retr   = cin_inv_Faro(R, T_retract,  q_end,  true);

%% SOLUCIÓN A: Interpolación en espacio articular (quintica)
t1 = linspace(0, t_aprox,   N1);
t2 = linspace(0, t_scan,    N2);
t3 = linspace(0, t_retract, N3);

[qA1, qdA1, qddA1] = jtraj(q_app.',   q_start.', N1);  % acercamiento
[qA2, qdA2, qddA2] = jtraj(q_start.', q_end.',   N2);  % escaneo (articular)
[qA3, qdA3, qddA3] = jtraj(q_end.',   q_retr.',  N3);  % retracción

% Evitar duplicar muestras en fronteras: alinear tamaños con tA
qA   = [qA1;  qA2(2:end,:);  qA3(2:end,:)];
qdA  = [qdA1; qdA2(2:end,:); qdA3(2:end,:)];
qddA = [qddA1;qddA2(2:end,:);qddA3(2:end,:)];
tA   = [t1, (t1(end)+t2(2:end)), (t1(end)+t2(end)+t3(2:end))];

% Trayectoria cartesiana para A (para comparación)
posA = zeros(size(qA,1),3);
for i=1:size(qA,1)
    Tcurr = R.fkine(qA(i,:));
    posA(i,:) = Tcurr.t';
end
velA = RoboticaUtils.derivada(posA, tA');
accA = RoboticaUtils.derivada(velA, tA');

%% SOLUCIÓN B: Escaneo cartesiano + IK, transiciones articulares
% Segmento 1: jtraj hasta inicio
[qB1, ~, ~] = jtraj(q_app.', q_start.', N1);

% Segmento 2: ctraj entre T_start y T_end con IK punto a punto
TB = ctraj(SE3(T_start), SE3(T_end), N2);
qB2 = zeros(N2, R.n);
q_prev = q_start;  % semilla inicial
for i = 1:N2
    Ti = TB(i);
    qi = cin_inv_Faro(R, Ti.T, q_prev, true);  % solución más cercana
    qB2(i,:) = qi.';
    q_prev = qi;
end

% Segmento 3: jtraj desde fin hasta retracción
[qB3, ~, ~] = jtraj(q_end.', q_retr.', N3);

% Igual criterio de muestreo en fronteras para que coincida con tB
qB  = [qB1; qB2(2:end,:); qB3(2:end,:)];
tB  = tA;                             % mismas marcas temporales totales
qdB = RoboticaUtils.derivada(qB,  tB');
qddB= RoboticaUtils.derivada(qdB, tB');

% Trayectoria cartesiana para B
posB = zeros(size(qB,1),3);
for i=1:size(qB,1)
    Tcurr = R.fkine(qB(i,:));
    posB(i,:) = Tcurr.t';
end
velB = RoboticaUtils.derivada(posB, tB');
accB = RoboticaUtils.derivada(velB, tB');

%% Animaciones previas a los gráficos
fprintf('\n=== Animación Método A (jtraj) ===\n');
figA = figure('Name','Animación Método A (jtraj)');
try
    R.plot(qA, 'delay', dt, 'trail', '-');
catch
    % En caso de versión diferente del toolbox, usar plot simple
    R.plot(qA);
end
title('Animación Método A (jtraj)');
maximize_and_wait(figA);

fprintf('\n=== Animación Método B (ctraj + IK) ===\n');
figB = figure('Name','Animación Método B (ctraj + IK)');
try
    R.plot(qB, 'delay', dt, 'trail', '-');
catch
    R.plot(qB);
end
title('Animación Método B (ctraj + IK)');
maximize_and_wait(figB);

%% Gráficos y guardado
outDir = fullfile(thisDir,'figs');
if ~exist(outDir,'dir'); mkdir(outDir); end

% Paleta por articulación
cols = lines(R.n);
legq = arrayfun(@(k) sprintf('q%d',k), 1:R.n, 'UniformOutput', false);

% Figura combinada con subplots 3x2: A a la izquierda, B a la derecha %{
%{
f0 = figure('Name','Articular: A vs B (subplots)');

% Fila 1: posición
subplot(3,2,1); hold on; grid on;
for k=1:R.n, plot(tA, rad2deg(qA(:,k)), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Ángulo [deg]'); title('Posición articular (A)'); legend('Location','bestoutside');

subplot(3,2,2); hold on; grid on;
for k=1:R.n, plot(tB, rad2deg(qB(:,k)), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Ángulo [deg]'); title('Posición articular (B)'); legend('Location','bestoutside');

% Fila 2: velocidad
subplot(3,2,3); hold on; grid on;
for k=1:R.n, plot(tA, rad2deg(qdA(:,k)), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Velocidad [deg/s]'); title('Velocidad articular (A)'); legend('Location','bestoutside');

subplot(3,2,4); hold on; grid on;
for k=1:R.n, plot(tB, rad2deg(qdB(:,k)), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Velocidad [deg/s]'); title('Velocidad articular (B)'); legend('Location','bestoutside');

% Fila 3: aceleración
subplot(3,2,5); hold on; grid on;
for k=1:R.n, plot(tA, rad2deg(qddA(:,k)), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Aceleración [deg/s^2]'); title('Aceleración articular (A)'); legend('Location','bestoutside');

subplot(3,2,6); hold on; grid on;
for k=1:R.n, plot(tB, rad2deg(qddB(:,k)), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Aceleración [deg/s^2]'); title('Aceleración articular (B)'); legend('Location','bestoutside');

saveas(f0, fullfile(outDir,'articular_grid_A_B.png'));
maximize_and_wait(f0);
%}
%}

% Figuras individuales como antes: A y B superpuestos por magnitud
% Posición articular
f1 = figure('Name','Evolución articular (grados)'); hold on; grid on;
for k=1:R.n
    plot(tA, rad2deg(qA(:,k)), '--', 'Color', cols(k,:), 'DisplayName',[legq{k} ' A']);
    plot(tB, rad2deg(qB(:,k)), '-',  'Color', cols(k,:), 'DisplayName',[legq{k} ' B']);
end
xlabel('Tiempo [s]'); ylabel('Ángulo [deg]'); title('Evolución Articular'); legend('Location','bestoutside');
saveas(f1, fullfile(outDir,'q_compare.png'));
maximize_and_wait(f1);

% Velocidad articular
f2 = figure('Name','Velocidad articular (deg/s)'); hold on; grid on;
for k=1:R.n
    plot(tA, rad2deg(qdA(:,k)), '--', 'Color', cols(k,:), 'DisplayName',[legq{k} 'd A']);
    plot(tB, rad2deg(qdB(:,k)), '-',  'Color', cols(k,:), 'DisplayName',[legq{k} 'd B']);
end
xlabel('Tiempo [s]'); ylabel('Velocidad [deg/s]'); title('Velocidad Articular'); legend('Location','bestoutside');
saveas(f2, fullfile(outDir,'qd_compare.png'));
maximize_and_wait(f2);

% Aceleración articular
f3 = figure('Name','Aceleración articular (deg/s^2)'); hold on; grid on;
for k=1:R.n
    plot(tA, rad2deg(qddA(:,k)), '--', 'Color', cols(k,:), 'DisplayName',[legq{k} 'dd A']);
    plot(tB, rad2deg(qddB(:,k)), '-',  'Color', cols(k,:), 'DisplayName',[legq{k} 'dd B']);
end
xlabel('Tiempo [s]'); ylabel('Aceleración [deg/s^2]'); title('Aceleración Articular'); legend('Location','bestoutside');
saveas(f3, fullfile(outDir,'qdd_compare.png'));
maximize_and_wait(f3);

% Figura combinada con subplots 3x2: A a la izquierda, B a la derecha
f0 = figure('Name','Articular: A vs B (subplots)');

% Fila 1: posición
subplot(3,2,1); hold on; grid on;
for k=1:R.n, plot(tA, qA(:,k), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Ángulo [rad]'); title('Posición articular (A)'); legend('Location','bestoutside');

subplot(3,2,2); hold on; grid on;
for k=1:R.n, plot(tB, qB(:,k), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Ángulo [rad]'); title('Posición articular (B)'); legend('Location','bestoutside');

% Fila 2: velocidad
subplot(3,2,3); hold on; grid on;
for k=1:R.n, plot(tA, qdA(:,k), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Velocidad [rad/s]'); title('Velocidad articular (A)'); legend('Location','bestoutside');

subplot(3,2,4); hold on; grid on;
for k=1:R.n, plot(tB, qdB(:,k), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Velocidad [rad/s]'); title('Velocidad articular (B)'); legend('Location','bestoutside');

% Fila 3: aceleración
subplot(3,2,5); hold on; grid on;
for k=1:R.n, plot(tA, qddA(:,k), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Aceleración [rad/s^2]'); title('Aceleración articular (A)'); legend('Location','bestoutside');

subplot(3,2,6); hold on; grid on;
for k=1:R.n, plot(tB, qddB(:,k), '-', 'Color', cols(k,:), 'DisplayName',legq{k}); end
xlabel('Tiempo [s]'); ylabel('Aceleración [rad/s^2]'); title('Aceleración articular (B)'); legend('Location','bestoutside');

saveas(f0, fullfile(outDir,'articular_grid_A_B.png'));
maximize_and_wait(f0);

% Posición cartesiana: 3 plots (fila superior combinada, abajo A y B)
fP = figure('Name','Posición cartesiana (grid)');
subplot(2,2,[1 2]); hold on; grid on;
plot(tA, posA(:,1), '--', 'Color', [0.85 0.33 0.10], 'DisplayName','X A');
plot(tA, posA(:,2), '--', 'Color', [0.47 0.67 0.19], 'DisplayName','Y A');
plot(tA, posA(:,3), '--', 'Color', [0.00 0.45 0.74], 'DisplayName','Z A');
plot(tB, posB(:,1), '-',  'Color', [0.85 0.33 0.10], 'DisplayName','X B');
plot(tB, posB(:,2), '-',  'Color', [0.47 0.67 0.19], 'DisplayName','Y B');
plot(tB, posB(:,3), '-',  'Color', [0.00 0.45 0.74], 'DisplayName','Z B');
title('A vs B (X,Y,Z)'); ylabel('Posición [m]'); legend('Location','bestoutside');
subplot(2,2,3); hold on; grid on;
plot(tA, posA(:,1), '-', 'Color', [0.85 0.33 0.10], 'DisplayName','X');
plot(tA, posA(:,2), '-', 'Color', [0.47 0.67 0.19], 'DisplayName','Y');
plot(tA, posA(:,3), '-', 'Color', [0.00 0.45 0.74], 'DisplayName','Z');
title('A (X,Y,Z)'); xlabel('Tiempo [s]'); ylabel('Posición [m]'); legend('Location','best');
subplot(2,2,4); hold on; grid on;
plot(tB, posB(:,1), '-', 'Color', [0.85 0.33 0.10], 'DisplayName','X');
plot(tB, posB(:,2), '-', 'Color', [0.47 0.67 0.19], 'DisplayName','Y');
plot(tB, posB(:,3), '-', 'Color', [0.00 0.45 0.74], 'DisplayName','Z');
title('B (X,Y,Z)'); xlabel('Tiempo [s]'); ylabel('Posición [m]'); legend('Location','best');
saveas(fP, fullfile(outDir,'pos_cart_grid.png')); maximize_and_wait(fP);

% Velocidad cartesiana: 3 plots (fila superior combinada, abajo A y B)
fV = figure('Name','Velocidad cartesiana (grid)');
subplot(2,2,[1 2]); hold on; grid on;
plot(tA, velA(:,1), '--', 'DisplayName','X A');
plot(tA, velA(:,2), '--', 'DisplayName','Y A');
plot(tA, velA(:,3), '--', 'DisplayName','Z A');
plot(tB, velB(:,1), '-',  'DisplayName','X B');
plot(tB, velB(:,2), '-',  'DisplayName','Y B');
plot(tB, velB(:,3), '-',  'DisplayName','Z B');
title('A vs B (X,Y,Z)'); ylabel('Velocidad [m/s]'); legend('Location','bestoutside');
subplot(2,2,3); hold on; grid on;
plot(tA, velA(:,1), '-', 'DisplayName','X');
plot(tA, velA(:,2), '-', 'DisplayName','Y');
plot(tA, velA(:,3), '-', 'DisplayName','Z');
title('A (X,Y,Z)'); xlabel('Tiempo [s]'); ylabel('Velocidad [m/s]'); legend('Location','best');
subplot(2,2,4); hold on; grid on;
plot(tB, velB(:,1), '-', 'DisplayName','X');
plot(tB, velB(:,2), '-', 'DisplayName','Y');
plot(tB, velB(:,3), '-', 'DisplayName','Z');
title('B (X,Y,Z)'); xlabel('Tiempo [s]'); ylabel('Velocidad [m/s]'); legend('Location','best');
saveas(fV, fullfile(outDir,'vel_cart_grid.png')); maximize_and_wait(fV);

% Aceleración cartesiana: 3 plots (fila superior combinada, abajo A y B)
fAcc = figure('Name','Aceleración cartesiana (grid)');
subplot(2,2,[1 2]); hold on; grid on;
plot(tA, accA(:,1), '--', 'DisplayName','X A');
plot(tA, accA(:,2), '--', 'DisplayName','Y A');
plot(tA, accA(:,3), '--', 'DisplayName','Z A');
plot(tB, accB(:,1), '-',  'DisplayName','X B');
plot(tB, accB(:,2), '-',  'DisplayName','Y B');
plot(tB, accB(:,3), '-',  'DisplayName','Z B');
title('A vs B (X,Y,Z)'); ylabel('Aceleración [m/s^2]'); legend('Location','bestoutside');
subplot(2,2,3); hold on; grid on;
plot(tA, accA(:,1), '-', 'DisplayName','X');
plot(tA, accA(:,2), '-', 'DisplayName','Y');
plot(tA, accA(:,3), '-', 'DisplayName','Z');
title('A (X,Y,Z)'); xlabel('Tiempo [s]'); ylabel('Aceleración [m/s^2]'); legend('Location','best');
subplot(2,2,4); hold on; grid on;
plot(tB, accB(:,1), '-', 'DisplayName','X');
plot(tB, accB(:,2), '-', 'DisplayName','Y');
plot(tB, accB(:,3), '-', 'DisplayName','Z');
title('B (X,Y,Z)'); xlabel('Tiempo [s]'); ylabel('Aceleración [m/s^2]'); legend('Location','best');
saveas(fAcc, fullfile(outDir,'acc_cart_grid.png')); maximize_and_wait(fAcc);

% Trayectoria geométrica en plano XZ
f7 = figure('Name','Trayectoria XZ'); hold on; grid on; axis equal
plot(posA(:,1), posA(:,3), '--', 'LineWidth',1.2, 'DisplayName','A (jtraj)');
plot(posB(:,1), posB(:,3), '-',  'LineWidth',1.2, 'DisplayName','B (ctraj+IK)');
xlabel('X [m]'); ylabel('Z [m]'); title('Trayectoria del TCP en plano XZ'); legend('Location','best');
saveas(f7, fullfile(outDir,'xz_trayectoria.png'));
maximize_and_wait(f7);

fprintf('Figuras guardadas en: %s\n', outDir);

%% Utilidad de presentación: maximizar y esperar botón
function maximize_and_wait(fig)
    % Maximiza la ventana y muestra un botón "Continuar" que desbloquea la ejecución
    try
        set(fig, 'WindowState','maximized');
    catch
        try
            set(fig, 'Units','normalized', 'OuterPosition',[0 0 1 1]);
        catch
        end
    end
    % Botón para continuar
    btn = uicontrol('Parent',fig, 'Style','pushbutton', 'String','Continuar', ...
                    'Units','normalized', 'Position',[0.86 0.02 0.12 0.06], ...
                    'FontSize',12, 'Callback',@(src,evt) uiresume(fig));
    try
        uiwait(fig);
    catch
        % si la figura se cerró a mano, continuar
    end
    if isvalid(btn)
        try, delete(btn); end
    end
    % Cerrar automáticamente la figura para mantener la pantalla limpia
    if ishghandle(fig)
        try, close(fig); end
    end
end
