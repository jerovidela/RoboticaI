
close all;
FLAG = 0;  %Para Activar simulacion 3D dejar en 1, Para desactivarla dejar en 0
dh =  [ ...
    0       0.283   0.000   -pi/2   0;   % Joint 1
    0       0.000   0.398    0      0;   % Joint 2
    0       0.000   0.213    0      0;   % Joint 3
    0       0.000   0.000    pi/2   0;   % Joint 4
    0       0.000   0.000    pi/2   0;   % Joint 5
    0       0.166   0.000    0      0];  % Joint 6
name = 'Robot Scan Arm';
qlim = deg2rad([ ...
   -170  170;    % q1
   -150   80;    % q2
   -110  140;    % q3
   -140  140;    % q4
   -120  120;    % q5
   -360  360]);  % q6
offset = deg2rad([0; 0; 0; 0; 90; 0]);
base = transl(0,0,0);
tool = transl(0,0,0);

R = SerialLink(dh);
R.name = name;
R.qlim = qlim;
R.offset = offset;
R.base = base;
R.tool = tool;

% ===== Parámetros del barrido =====
M         = [1 1 1 1 1 0];         % 5-DOF (mantener vertical Z)
R_vertical = SE3.rpy(0, pi, 0);     % herramienta vertical hacia abajo
z0        = 0.15;                   % altura de palpado
x0        = 0.40; x1 = 0.60;        % extremos en X de cada pasada
y0        = 0.00;                   % inicio en Y
num_pases = 5;                      % cantidad de líneas
dy        = 0.02;                   % separación entre líneas (en Y)
lift      = 0.02;                   % alzamiento durante cruce de carril

% Discretización por segmento
N_line  = 200;   % puntos por línea recta de medición
N_lift  = 60;    % puntos para subir/bajar y desplazamiento corto
N_lower = 40;    % puntos para “bajar” al plano (z0)

% ===== Waypoints SE3 (trayectoria serpentina) =====
% primer punto y configuración
P_inicio = [x0 ; y0 ; z0];
P_final  = [x1 ; y0 ; z0];
T0 = SE3(P_inicio) * R_vertical;
Tf = SE3(P_final)  * R_vertical;

q0_hint = deg2rad([10, 0, 0, 0, 0, 0]);
q0 = R.ikcon(T0, q0_hint);
qf = R.ikcon(Tf);

% armamos una lista de poses SE3 concatenando segmentos ctraj
T_all = SE3.empty(0,0);

y = y0;
dir = +1;   % +1: x0->x1, -1: x1->x0
for i = 1:num_pases
    % 1) Línea de barrido sobre el plano z0
    if dir > 0
        A = SE3([x0; y; z0]) * R_vertical;
        B = SE3([x1; y; z0]) * R_vertical;
    else
        A = SE3([x1; y; z0]) * R_vertical;
        B = SE3([x0; y; z0]) * R_vertical;
    end
    T_all = [T_all, ctraj(A, B, N_line)];

    % 2) Si no es la última pasada: transición al carril siguiente
    if i < num_pases
        y_next = y + dy;

        % 2a) subir y desplazar en Y (manteniendo X del extremo actual)
        if dir > 0
            P_end = [x1; y; z0];
            P_up  = [x1; y_next; z0+lift];
        else
            P_end = [x0; y; z0];
            P_up  = [x0; y_next; z0+lift];
        end
        T_end = SE3(P_end) * R_vertical;
        T_up  = SE3(P_up)  * R_vertical;
        T_all = [T_all, ctraj(T_end, T_up, N_lift)];

        % 2b) bajar al plano en el mismo X del extremo (inicio de nueva línea)
        if dir > 0
            P_down = [x1; y_next; z0];   % comenzar próxima línea desde x1
        else
            P_down = [x0; y_next; z0];   % comenzar próxima línea desde x0
        end
        T_down = SE3(P_down) * R_vertical;
        T_all  = [T_all, ctraj(T_up, T_down, N_lower)];

        % actualizar carril y dirección para la próxima línea
        y   = y_next;
        dir = -dir;
    end
end

% ===== IK de seguimiento con continuidad (igual a tu método) =====
N = numel(T_all);
Q_lineal = jtraj(q0, qf, max(N,2));     % semilla suave (solo para q0..qf)
Q_lineal = repmat(Q_lineal(1,:), N, 1); % semilla constante simple

Q_trayectoria = zeros(N,6);
q_prev = Q_lineal(1,:);

for k = 1:N
    Tk = T_all(k).T; % hace que T_all pase de SE3 a matriz 4*4 para que no falle cin_inv_Faro
    q_try = cin_inv_Faro(R, Tk, q_prev, true);
    if ~isempty(q_try) && ~any(isnan(q_try))
        q_try = q_try.'; % corrigo a 1*6
    end
    if isempty(q_try) || any(isnan(q_try))
        q_try = cin_inv_Faro(R, Tk, Q_lineal(k,:), true)
    end
  
    if isempty(q_try) || any(isnan(q_try))
        q_try = q_prev   % último recurso, mantiene continuidad
    end
    % evitar salto mayor a pi
    dq_diff = q_try - q_prev;
    dq_diff = mod(dq_diff + pi, 2*pi) - pi;
    q_try = q_prev + dq_diff;

    Q_trayectoria(k,:) = q_try;
    q_prev = q_try;
end

if (FLAG==1)
    % R.plot (animación)
    %R.plot(Q_trayectoria);
    R.plot(Q_trayectoria, 'trail', {'r', 'LineWidth', 2});
end

% ===== Cálculo de la posición del efector para graficar =====
P = zeros(size(Q_trayectoria,1),3);
for i = 1:size(Q_trayectoria,1)
    T = R.fkine(Q_trayectoria(i,:));
    P(i,:) = transl(T);   % extrae [x,y,z]
end


% ===== Tiempo total y derivadas (para tu bloque Jacobiano) =====
T_total = 2.0 * num_pases;                 % ajustá el tiempo total si querés
t  = linspace(0, T_total, N).';
dt = t(2)-t(1);

q  = movmean(Q_trayectoria, 3);
dq = gradient(q, dt);
ddq = gradient(dq, dt);

% ===== Vel/Acc con Jacobiano (igual a tu esquema robusto) =====
qN = size(q,1);
v_lin = zeros(qN,3);  w_ang = zeros(qN,3);
a_lin = zeros(qN,3);  alpha = zeros(qN,3);
have_jdot = ismethod(R,'jacob_dot') || ismethod(R,'jdot');

for k = 1:qN
    qk = q(k,:);  dqk = dq(k,:).';  ddqk = ddq(k,:).';
    J = R.jacob0(qk);
    v_lin(k,:) = (J(1:3,:)*dqk).';
    w_ang(k,:) = (J(4:6,:)*dqk).';

    Jdot = [];
    if have_jdot
        try
            if ismethod(R,'jacob_dot'), Jdot = R.jacob_dot(qk, dq(k,:));
            else,                      Jdot = R.jdot(qk, dq(k,:));
            end
            if ndims(Jdot) > 2, Jdot = squeeze(Jdot); end
        catch
            Jdot = [];
        end
    end
    if isempty(Jdot) || ~isequal(size(Jdot),[6 6])
        k_prev = max(1,k-1);  k_next = min(qN,k+1);
        J_prev = R.jacob0(q(k_prev,:));
        J_next = R.jacob0(q(k_next,:));
        dt_eff = (k_next-k_prev)*dt;  if dt_eff==0, Jdot = zeros(6,6);
        else, Jdot = (J_next - J_prev)/dt_eff; end
    end

    acc6 = J*ddqk + Jdot*dqk;
    a_lin(k,:) = acc6(1:3).';
    alpha(k,:) = acc6(4:6).';
end

v_norm = sqrt(sum(v_lin.^2,2));
a_norm = sqrt(sum(a_lin.^2,2));

% ====== PLOTS básicos (si querés verlos) ======
tJ = t;  % mismo tiempo

figure('Name','End-effector position (from fkine)','NumberTitle','off');
subplot(4,1,1); 
plot(t, P(:,1), 'LineWidth', 1.2); grid on;
ylabel('X [m]');
subplot(4,1,2); 
plot(t, P(:,2), 'LineWidth', 1.2); grid on;
ylabel('Y [m]');
subplot(4,1,3); 
plot(t, P(:,3), 'LineWidth', 1.2); grid on;
ylabel('Z [m]');
subplot(4,1,4);
plot(t, sqrt(sum(P.^2,2)), 'LineWidth', 1.2); grid on;
ylabel('||P|| [m]'); xlabel('t [s]');
sgtitle('End-effector position (from fkine)');

figure('Name','End-effector velocity (Jacobian)','NumberTitle','off');
subplot(4,1,1); plot(tJ, v_lin(:,1)); grid on; ylabel('Vx [m/s]');
subplot(4,1,2); plot(tJ, v_lin(:,2)); grid on; ylabel('Vy [m/s]');
subplot(4,1,3); plot(tJ, v_lin(:,3)); grid on; ylabel('Vz [m/s]');
subplot(4,1,4); plot(tJ, v_norm);     grid on; ylabel('||V|| [m/s]'); xlabel('t [s]');
sgtitle('End-effector velocity (from J·dq)');

figure('Name','End-effector acceleration (Jacobian)','NumberTitle','off');
subplot(4,1,1); plot(tJ, a_lin(:,1)); grid on; ylabel('Ax [m/s^2]');
subplot(4,1,2); plot(tJ, a_lin(:,2)); grid on; ylabel('Ay [m/s^2]');
subplot(4,1,3); plot(tJ, a_lin(:,3)); grid on; ylabel('Az [m/s^2]');
subplot(4,1,4); plot(tJ, a_norm);     grid on; ylabel('||A|| [m^2/s]'); xlabel('t [s]');
sgtitle('End-effector acceleration (J·ddq + Jdot·dq)');


%======================================================================
% === Plot de Posiciones, Velocidades y Aceleraciones Articulares ===
%======================================================================

% Definición de colores y leyendas para las 5 primeras articulaciones
q_indices = 1:5;
q_names = {'q1', 'q2', 'q3', 'q4', 'q5'};
% Colores para q1 a q5 (puedes ajustarlos)
colors = [
    0.0 0.447 0.741; % Azul
    0.850 0.325 0.098; % Naranja
    0.929 0.694 0.125; % Amarillo
    0.494 0.184 0.556; % Violeta
    0.466 0.674 0.188  % Verde
];

figure('Name','Variables Articulares (q, dq, ddq)','NumberTitle','off','Position',[100 100 1000 800]);

% --- 1. Subplot de POSICIONES (Ángulos q) ---
subplot(3,1,1);
hold on;
for i = 1:length(q_indices)
    idx = q_indices(i);
    plot(t, Q_trayectoria(:, idx), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', q_names{i});
end
hold off;
title('Posiciones Angulares q (Articulaciones 1 a 5)');
ylabel('Ángulo (rad)');
grid on;
legend('show', 'Location', 'best');

% --- 2. Subplot de VELOCIDADES (dq) ---
subplot(3,1,2);
hold on;
for i = 1:length(q_indices)
    idx = q_indices(i);
    plot(t, dq(:, idx), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', q_names{i});
end
hold off;
title('Velocidades Angulares \dot{q}');
ylabel('Velocidad (rad/s)');
grid on;
legend('show', 'Location', 'best');

% --- 3. Subplot de ACELERACIONES (ddq) ---
subplot(3,1,3);
hold on;
for i = 1:length(q_indices)
    idx = q_indices(i);
    plot(t, ddq(:, idx), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', q_names{i});
end
hold off;
title('Aceleraciones Angulares \ddot{q}');
ylabel('Aceleración (rad/s^2)');
xlabel('Tiempo (s)');
grid on;
legend('show', 'Location', 'best');

% Ajuste general de la ventana
sgtitle('Evolución Temporal de las Variables Articulares q1-q5');