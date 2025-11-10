close all;
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

M = [1 1 1 1 1 0];
R_vertical = SE3.rpy(0, pi, 0);
P_inicio = [0.4 ; 0.0 ; 0.15];
P_final = [0.6 ; 0.0 ; 0.15];
T0 = SE3(P_inicio) * R_vertical;
Tf = SE3(P_final) * R_vertical;
q0_hint = deg2rad([10, 0, 0, 0, 0, 0]);
q0 = R.ikcon(T0, q0_hint);
qf = R.ikcon(Tf);
N = 200;
T_trayectoria = ctraj(T0, Tf, N);
Q_lineal = jtraj(q0, qf, N);
Q_trayectoria = zeros(N,6);
q_prev = Q_lineal(1,:);

for k = 1:N
    Tk = T_trayectoria(k);
    q_try = R.ikine(Tk, 'q0', Q_lineal(k,:), 'mask', M, 'qlim',1);
    % si no converge, usar la solución previa (continuidad)
    if isempty(q_try) || any(isnan(q_try))
        q_try = R.ikine(Tk, 'q0', q_prev, 'mask', M, 'qlim',1);
    end
    % si aún falla, caer a ikcon cerca de la previa
    if isempty(q_try) || any(isnan(q_try))
        q_try = R.ikcon(Tk, q_prev);
    end
    % último recurso: usar el punto lineal (mantener continuidad)
    if isempty(q_try) || any(isnan(q_try))
        q_try = Q_lineal(k,:);
    end
    % evitar salto mayor a π (cambio de rama)
    dq_diff = q_try - q_prev;
    dq_diff = mod(dq_diff + pi, 2*pi) - pi;
    q_try = q_prev + dq_diff;

    Q_trayectoria(k,:) = q_try;
    q_prev = q_try;
end
% Q_trayectoria = movmean(Q_trayectoria, 7);
R.plot(Q_trayectoria);

q  = Q_trayectoria;
q  = movmean(q, 3);
dq = gradient(q, dt);
ddq = gradient(dq, dt);

qN = size(v_lin, 1);              % largo "real" de las derivadas
tJ = t;                           % tiempo para jacobiano
if numel(tJ) ~= qN
    tJ = linspace(0, T_total, qN).';
end

v_lin = zeros(qN,3);  w_ang = zeros(qN,3);
a_lin = zeros(qN,3);  alpha = zeros(qN,3);

have_jdot = ismethod(R,'jacob_dot') || ismethod(R,'jdot');

for k = 1:qN
    qk       = q(k,:); 
    dqk_col  = dq(k,:).';
    ddqk_col = ddq(k,:).';

    J = R.jacob0(qk);

    v_lin(k,:) = (J(1:3,:)*dqk_col).';
    w_ang(k,:) = (J(4:6,:)*dqk_col).';

    Jdot = [];
    if have_jdot
        try
            if ismethod(R,'jacob_dot')
                Jdot = R.jacob_dot(qk, dq(k,:));
            else
                Jdot = R.jdot(qk, dq(k,:));
            end
            if ndims(Jdot) > 2, Jdot = squeeze(Jdot); end
        catch
            Jdot = [];
        end
    end

    if isempty(Jdot) || ~isequal(size(Jdot),[6 6])
        k_prev = max(1, k-1);
        k_next = min(qN, k+1);

        J_prev = R.jacob0(q(k_prev,:));
        J_next = R.jacob0(q(k_next,:));

        dt_eff = (k_next - k_prev) * dt;     % 2*dt salvo en bordes
        if dt_eff == 0
            Jdot = zeros(6,6);
        else
            Jdot = (J_next - J_prev) / dt_eff;
        end
    end

    acc6 = J*ddqk_col + Jdot*dqk_col;   % [ax ay az αx αy αz]^T
    a_lin(k,:) = acc6(1:3).';
    alpha(k,:) = acc6(4:6).';
end

v_norm = sqrt(sum(v_lin.^2,2));
a_norm = sqrt(sum(a_lin.^2,2));

% Velocidad (cartesiana)
h2 = figure('Name','End-effector velocity (Jacobian)','NumberTitle','off','Visible','on');
subplot(4,1,1); plot(tJ, v_lin(:,1)); grid on; ylabel('Vx [m/s]');
subplot(4,1,2); plot(tJ, v_lin(:,2)); grid on; ylabel('Vy [m/s]');
subplot(4,1,3); plot(tJ, v_lin(:,3)); grid on; ylabel('Vz [m/s]');
subplot(4,1,4); plot(tJ, v_norm);     grid on; ylabel('||V|| [m/s]'); xlabel('t [s]');
sgtitle('End-effector velocity (from J·dq)'); shg; drawnow;

% Aceleración (cartesiana)
h3 = figure('Name','End-effector acceleration (Jacobian)','NumberTitle','off','Visible','on');
subplot(4,1,1); plot(tJ, a_lin(:,1)); grid on; ylabel('Ax [m/s^2]');
subplot(4,1,2); plot(tJ, a_lin(:,2)); grid on; ylabel('Ay [m/s^2]');
subplot(4,1,3); plot(tJ, a_lin(:,3)); grid on; ylabel('Az [m/s^2]');
subplot(4,1,4); plot(tJ, a_norm);     grid on; ylabel('||A|| [m/s^2]'); xlabel('t [s]');
sgtitle('End-effector acceleration (J·ddq + Jdot·dq)'); shg; drawnow;

figure; plot(tJ, w_ang); grid on; title('\omega (rad/s)'); legend('\omega_x','\omega_y','\omega_z');
figure; plot(tJ, alpha); grid on; title('\alpha (rad/s^2)'); legend('\alpha_x','\alpha_y','\alpha_z');

