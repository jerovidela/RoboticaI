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
    120  350;   % q2
   -170  170;    % q3
   -180  180;    % q4
   -210   30;    % q5
   -360  360]);  % q6
offset = deg2rad([0; -270; 0; 0; 90; 0]);
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
q0_hint = deg2rad([80, 0, 0, 0, 10, 0]);
q0 = R.ikcon(T0, q0_hint);
qf = R.ikcon(Tf);
N = 50; 
T_trayectoria = ctraj(T0, Tf, N);
Q_trayectoria = R.ikine(T_trayectoria, 'q0', q0, 'mask', M, 'qlim', 1, 'search', 1);
R.plot(Q_trayectoria);


%% ====== ANALISIS TRAYECTORIA (cartesiano) ======
drawnow; pause(0.1);                 % soltar animación
set(0,'DefaultFigureWindowStyle','normal');  % evitar 'docked'

Npts = size(Q_trayectoria, 1);
T_total = 2.0;                        % [s] ajustá si querés
t = linspace(0, T_total, Npts).';
dt = t(2)-t(1);

% Posición deseada vs real
pos_des = zeros(Npts,3);
pos_act = zeros(Npts,3);
% --- Posición deseada (de T_trayectoria) ---
if isa(T_trayectoria, 'SE3')
    % array de SE3 -> transl devuelve [N x 3]
    pos_des = transl(T_trayectoria);           % [N x 3]
else
    % stack numérico 4x4xN
    pos_des = zeros(N,3);
    for i = 1:N
        pos_des(i,:) = transl(T_trayectoria(:,:,i));
    end
end

% --- Posición real (fkine de Q_trayectoria) ---
Ta_all = R.fkine(Q_trayectoria);               % puede ser SE3 array o 4x4xN
if isa(Ta_all, 'SE3')
    pos_act = transl(Ta_all);                   % [N x 3]
else
    pos_act = zeros(N,3);
    for i = 1:N
        pos_act(i,:) = transl(Ta_all(:,:,i));
    end
end


% Error, vel, acel
err = pos_des - pos_act;                         % [ex ey ez]
err_norm = sqrt(sum(err.^2,2));
vel = gradient(pos_act, dt);                     % [vx vy vz]
acc = gradient(vel, dt);                         % [ax ay az]
vel_norm = sqrt(sum(vel.^2,2));
acc_norm = sqrt(sum(acc.^2,2));

%% ====== PLOTS ======
% Posición
h1 = figure('Name','End-effector position','NumberTitle','off','Visible','on');
subplot(3,1,1); plot(t, pos_des(:,1),'--', t, pos_act(:,1),'-'); grid on; ylabel('X [m]'); legend('desired','actual','Location','best');
subplot(3,1,2); plot(t, pos_des(:,2),'--', t, pos_act(:,2),'-'); grid on; ylabel('Y [m]');
subplot(3,1,3); plot(t, pos_des(:,3),'--', t, pos_act(:,3),'-'); grid on; ylabel('Z [m]'); xlabel('t [s]');
sgtitle('End-effector position'); shg; drawnow;

% Velocidad
h2 = figure('Name','End-effector velocity','NumberTitle','off','Visible','on');
subplot(4,1,1); plot(t, vel(:,1)); grid on; ylabel('Vx [m/s]');
subplot(4,1,2); plot(t, vel(:,2)); grid on; ylabel('Vy [m/s]');
subplot(4,1,3); plot(t, vel(:,3)); grid on; ylabel('Vz [m/s]');
subplot(4,1,4); plot(t, vel_norm); grid on; ylabel('||V|| [m/s]'); xlabel('t [s]');
sgtitle('End-effector velocity'); shg; drawnow;

% Aceleración
h3 = figure('Name','End-effector acceleration','NumberTitle','off','Visible','on');
subplot(4,1,1); plot(t, acc(:,1)); grid on; ylabel('Ax [m/s^2]');
subplot(4,1,2); plot(t, acc(:,2)); grid on; ylabel('Ay [m/s^2]');
subplot(4,1,3); plot(t, acc(:,3)); grid on; ylabel('Az [m/s^2]');
subplot(4,1,4); plot(t, acc_norm); grid on; ylabel('||A|| [m/s^2]'); xlabel('t [s]');
sgtitle('End-effector acceleration'); shg; drawnow;

% Error de posición
h4 = figure('Name','Position error','NumberTitle','off','Visible','on');
subplot(4,1,1); plot(t, err(:,1)); grid on; ylabel('e_x [m]');
subplot(4,1,2); plot(t, err(:,2)); grid on; ylabel('e_y [m]');
subplot(4,1,3); plot(t, err(:,3)); grid on; ylabel('e_z [m]');
subplot(4,1,4); plot(t, err_norm); grid on; ylabel('||e|| [m]'); xlabel('t [s]');
sgtitle('Position error'); shg; drawnow;

%% ====== (Opcional) espacio articular ======
q = Q_trayectoria;
dq = gradient(q, dt);
ddq = gradient(dq, dt);
h5 = figure('Name','Joint space','NumberTitle','off','Visible','on');
subplot(3,1,1); plot(t, q);   grid on; ylabel('q [rad]');     legend('q1','q2','q3','q4','q5','q6','Location','bestoutside');
subplot(3,1,2); plot(t, dq);  grid on; ylabel('dq [rad/s]');
subplot(3,1,3); plot(t, ddq); grid on; ylabel('ddq [rad/s^2]'); xlabel('t [s]');
sgtitle('Joint space'); shg; drawnow;

% (Opcional) guardar como PNG para verificar que se generaron:
% saveas(h1,'pos.png'); saveas(h2,'vel.png'); saveas(h3,'acc.png'); saveas(h4,'err.png'); saveas(h5,'joint.png');
