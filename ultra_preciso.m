close all; clc; clear;
dh = [ 0 0.283 0    -pi/2 0;
       0 0     0.398  0   0;
       0 0     0.213  0   0;
       0 0     0      pi/2 0;
       0 0     0      pi/2 0;
       0 0.166 0      0    0 ];
R = SerialLink(dh, 'name','Robot Scan Arm');
R.qlim   = deg2rad([ -170 170; -150 80; -110 140; -140 140; -120 120; -360 360 ]);
R.offset = deg2rad([0 0 0 0 0 0]).';
R.base   = transl(0,0,0);
R.tool   = transl(0,0,0);

M = [1 1 1 1 1 0];
R_vertical = SE3.rpy(0,pi,0);

num_lines = 4;      % Número total de líneas a escanear
dy = 0.05;          % Separación entre líneas en Y (metros)
dz = 0.02;          % Altura de elevación/descenso en Z (metros)
vel_scan = 0.05;    % Velocidad para las líneas de escaneo (m/s)
vel_move = 0.05;
dt = 0.02;          % Paso de tiempo (s) -> 50 Hz

% Puntos base de la primera línea
P_start_base = [0.3; 0; 0.15];
P_end_base   = [0.5; 0; 0.15];
scan_dist = norm(P_end_base - P_start_base);

% Inicializar arreglos totales
Q_all = []; t_all = []; t_current = 0;

% Punto de partida inicial
T0_init = SE3(P_start_base) * R_vertical;
q_current = R.ikcon(T0_init, deg2rad([10 0 0 0 0 0])); % q inicial
P_current = P_start_base;

% Añadimos el primer punto para que los arreglos no estén vacíos
Q_all = [q_current];
t_all = [0];

for i = 1:num_lines
    fprintf('Generando Línea %d...\n', i);
    
    % --- Segmento 1: LÍNEA DE ESCANEO ---
    y_val = (i-1) * dy;
    
    % Definir P0 y Pf para la línea actual (dirección alterna)
    if mod(i, 2) == 1 % Líneas impares: +X
        P0 = P_start_base + [0; y_val; 0];
        Pf = P_end_base   + [0; y_val; 0];
    else % Líneas pares: -X
        P0 = P_end_base   + [0; y_val; 0];
        Pf = P_start_base + [0; y_val; 0];
    end
    
    T0 = SE3(P0) * R_vertical;
    Tf = SE3(Pf) * R_vertical;
    
    T_total = scan_dist / vel_scan;
    t_seg = (dt:dt:T_total).';
    N = length(t_seg);
    
    if N > 0
        q0 = q_current;
        qf = R.ikcon(Tf, q0); % q final del segmento

        % Trayectoria cartesiana recta + refinamiento por IK (tu código)
        T_path = ctraj(T0, Tf, N);
        Q_seed = jtraj(q0, qf, N); % Semilla suave
        Q_seg = zeros(N,6);
        q_prev = q0;
        for k = 1:N
            Tk = T_path(k);
            q_try = R.ikine(Tk, 'q0', Q_seed(k,:), 'mask', M, 'qlim', 1);
            if isempty(q_try)
                q_try = R.ikine(Tk, 'q0', q_prev, 'mask', M, 'qlim', 1);
            end
            if isempty(q_try)
                q_try = R.ikcon(Tk, q_prev);
            end
            if isempty(q_try)
                q_try = q_prev;
            end
            dqdiff = mod(q_try - q_prev + pi, 2*pi) - pi;
            q_try  = q_prev + dqdiff;
            Q_seg(k,:) = q_try;
            q_prev = q_try;
        end

        % Añadir a los arreglos totales
        Q_all = [Q_all; Q_seg];
        t_all = [t_all; t_seg + t_current];
        
        t_current = t_all(end);
        q_current = Q_all(end, :); % El próximo segmento empieza desde el final de este
        P_current = Pf;
    end
    
    
    % --- Segmentos 2, 3, 4: TRANSICIÓN (si no es la última línea) ---
    if i < num_lines
        P_lift   = P_current + [0; 0; dz];      % 2. Elevar
        P_shift  = P_lift + [0; dy; 0];         % 3. Desplazar en Y
        
        y_next = i * dy;
        if mod(i+1, 2) == 1 % Próxima línea es impar (+X)
            P_descend = P_start_base + [0; y_next; 0];
        else % Próxima línea es par (-X)
            P_descend = P_end_base + [0; y_next; 0];
        end
        
        Waypoints_T = {P_lift, P_shift, P_descend};
        Distances = [dz, dy, dz]; % Distancias aproximadas
        
        for j = 1:3 % Loop para los 3 movimientos de transición
            P0 = P_current;
            Pf = Waypoints_T{j};
            T0 = SE3(P0) * R_vertical;
            Tf = SE3(Pf) * R_vertical;
            
            T_total = Distances(j) / vel_move;
            t_seg = (dt:dt:T_total).';
            N = length(t_seg);
            
            if N > 0
                q0 = q_current;
                qf = R.ikcon(Tf, q0);

                T_path = ctraj(T0, Tf, N);
                Q_seed = jtraj(q0, qf, N);
                Q_seg = zeros(N,6);
                q_prev = q0;
                for k = 1:N
                    Tk = T_path(k);
                    q_try = R.ikine(Tk, 'q0', Q_seed(k,:), 'mask', M, 'qlim', 1);
                    if isempty(q_try), q_try = R.ikine(Tk, 'q0', q_prev, 'mask', M, 'qlim', 1); end
                    if isempty(q_try), q_try = R.ikcon(Tk, q_prev); end
                    if isempty(q_try), q_try = q_prev; end
                    dqdiff = mod(q_try - q_prev + pi, 2*pi) - pi;
                    q_try  = q_prev + dqdiff;
                    Q_seg(k,:) = q_try;
                    q_prev = q_try;
                end
                
                Q_all = [Q_all; Q_seg];
                t_all = [t_all; t_seg + t_current];
                
                t_current = t_all(end);
                q_current = Q_all(end, :);
                P_current = Pf;
            end
        end 
    end 
end

R.plot(Q_all, 'delay', 0.01, 'trail', 'r-');

Q = Q_all;
t = t_all;
N = length(t);
dt = mean(diff(t));

Qd = gradient(Q.', dt).';
Qdd = gradient(Qd.', dt).';

v    = zeros(N,3);    w = zeros(N,3);
a    = zeros(N,3);    alpha = zeros(N,3);

Jdot = zeros(6,6,N);
for k = 1:N
    k_prev = max(1, k-1);
    k_next = min(N, k+1);
    J_prev = R.jacob0(Q(k_prev,:));
    J_next = R.jacob0(Q(k_next,:));
    Jdot(:,:,k) = (J_next - J_prev) / ( (k_next - k_prev)*dt );
end

for k = 1:N
    J = R.jacob0(Q(k,:));
    v(k,:) = (J(1:3,:)*Qd(k,:).').';
    w(k,:) = (J(4:6,:)*Qd(k,:).').';

    acc6 = J*Qdd(k,:).' + Jdot(:,:,k)*Qd(k,:).';
    a(k,:)     = acc6(1:3).';
    alpha(k,:) = acc6(4:6).';
end

vnorm = vecnorm(v,2,2);
anorm = vecnorm(a,2,2);

kappa = zeros(N,1);
w_yosh = zeros(N,1);
for k = 1:N
    J = R.jacob0(Q(k,:));
    try
        s = svd(J);
        kappa(k) = max(s)/min(s);
        w_yosh(k) = sqrt(det(J*J.'));
    catch
        kappa(k) = NaN;
        w_yosh(k) = 0;
    end
end

%% --- Plots rápidos ---
figure('Name','Jacobian conditioning');  
yyaxis left;  plot(t, kappa, 'LineWidth',1.2); grid on; ylabel('\kappa(J)'); set(gca, 'YScale', 'log');
yyaxis right; plot(t, w_yosh, 'LineWidth',1.2); ylabel('w = \surd det(JJ^T)');
xlabel('t [s]'); title('Condicionamiento del Jacobiano');

figure('Name','End-effector velocity (J·qd)');
subplot(4,1,1); plot(t, v(:,1)); grid on; ylabel('Vx [m/s]');
subplot(4,1,2); plot(t, v(:,2)); grid on; ylabel('Vy [m/s]');
subplot(4,1,3); plot(t, v(:,3)); grid on; ylabel('Vz [m/s]');
subplot(4,1,4); plot(t, vnorm);  grid on; ylabel('||V||'); xlabel('t [s]');

figure('Name','End-effector acceleration (J·qdd + Jdot·qd)');
subplot(4,1,1); plot(t, a(:,1)); grid on; ylabel('Ax [m/s^2]');
subplot(4,1,2); plot(t, a(:,2)); grid on; ylabel('Ay [m/s^2]');
subplot(4,1,3); plot(t, a(:,3)); grid on; ylabel('Az [m/s^2]');
subplot(4,1,4); plot(t, anorm);  grid on; ylabel('||A||'); xlabel('t [s]');

figure('Name','Joint space');
subplot(3,1,1); plot(t, Q);   grid on; ylabel('q [rad]');
subplot(3,1,2); plot(t, Qd);  grid on; ylabel('qd [rad/s]');
subplot(3,1,3); plot(t, Qdd); grid on; ylabel('qdd [rad/s^2]'); xlabel('t [s]');