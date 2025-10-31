% TP8 - Escaneo en placa: comparación A (jtraj) vs B (ctraj+IK)
% Versión reubicada en scenarios/, ajustada para usar robots/ y TP08/helpers.

clear; clc; close all;

%% Paths y carga de robot
scnDir  = fileparts(mfilename('fullpath'));     % .../Simulation - Final/scenarios
simRoot = fileparts(scnDir);                    % .../Simulation - Final
addpath(fullfile(simRoot,'robots'));

% Helpers (TP08)
tp8Dir  = fullfile(simRoot,'..','TP08');
addpath(tp8Dir, fullfile(tp8Dir,'helpers'));

% Cargar robot del grupo
R = robot_G11();
fprintf('Robot "%s" cargado. GDL=%d\n', R.name, R.n);
R.offset = R.offset(:);

%% Parámetros de tarea y discretización
dt = 0.05; t_aprox = 3.0; t_scan = 4.0; t_retract = 3.0;
N1 = max(2, round(t_aprox/dt));
N2 = max(2, round(t_scan/dt));
N3 = max(2, round(t_retract/dt));

%% Waypoints en placa (plano XZ, Y constante)
y_plate = 0.00; standoff = 0.00;
Rtool = rotx(-pi/2);
T_approach = rt2tr(Rtool, [0.30; 0.02; 0.25]);
T_start    = rt2tr(Rtool, [0.25; y_plate+standoff; 0.15]);
T_end      = rt2tr(Rtool, [0.45; y_plate+standoff; 0.15]);
T_retract  = rt2tr(Rtool, [0.35; 0.03; 0.25]);

% IK para waypoints
q_seed = ones(6,1);
q_app   = cin_inv_Faro(R, T_approach, q_seed, true);
q_start = cin_inv_Faro(R, T_start,    q_app,  true);
q_end   = cin_inv_Faro(R, T_end,      q_start,true);
q_retr  = cin_inv_Faro(R, T_retract,  q_end,  true);

%% Método A: articular
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

%% Método B: cartesiano + IK
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

%% Animaciones (con botón continuar)
disp('=== Animación Método A (jtraj) ===');
figA = figure('Name','Animación Método A (jtraj)');
try, R.plot(qA, 'delay', dt, 'trail', '-'); catch, R.plot(qA); end
maximize_and_wait(figA);

disp('=== Animación Método B (ctraj + IK) ===');
figB = figure('Name','Animación Método B (ctraj + IK)');
try, R.plot(qB, 'delay', dt, 'trail', '-'); catch, R.plot(qB); end
maximize_and_wait(figB);

%% Utilidad local
function maximize_and_wait(fig)
    try, set(fig, 'WindowState','maximized'); catch, set(fig, 'Units','normalized','OuterPosition',[0 0 1 1]); end
    uicontrol('Parent',fig, 'Style','pushbutton', 'String','Continuar', ...
             'Units','normalized', 'Position',[0.86 0.02 0.12 0.06], ...
             'FontSize',12, 'Callback',@(src,evt) uiresume(fig));
    try, uiwait(fig); catch, end
    if ishghandle(fig), try, close(fig); end, end
end

