close all; clc; clear;

[R, plotopt] = robot_G11();

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
q_current = R.ikcon(T0_init, deg2rad([10 180 0 0 0 0])); % q inicial
P_current = P_start_base;

% Añadimos el primer punto para que los arreglos no estén vacíos
%Q_all = [q_current];
%t_all = [0];

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

R.plot(Q_all, plotopt{:},'delay', 0.01, 'trail', 'r-');

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




function [R, plotopt, q_home] = robot_G11(vis)
% robot_G11  Crea y visualiza el manipulador del grupo G11 (SerialLink)
% - Define parametros DH, limites, base y herramienta; devuelve SerialLink.
% - Si se pasa 'vis', ajusta opciones de visualizacion y puede plotear (vis.plot=true).
% Entradas: vis (struct opcional: ws, linkcolor, jointcolor, jointlen, jointdiam, plot)
% Salidas:  R (SerialLink), plotopt (cell de opciones para R.plot), q_home (1xN)
%% Definicion de los parametros del robot
    d_tool = 0.0; % Definición explícita del desplazamiento de la herramienta puede cambiar segun la herramienta
    dh = [ ...
        0     0.283     0.000    -pi/2   0;   % Joint 1
        0     0.000     0.398    0       0;   % Joint 2
        0     0.000     0.213    0       0;   % Joint 3
        0     0.000     0.000    pi/2    0;   % Joint 4
        0     0.000     0.000    pi/2    0;   % Joint 5
        0     0.166     0.000    0       0    % Joint 6
        ];  

    name = 'Robot Scan Arm';
    qlim = deg2rad([ ...
       -170  170;    % q1
       3.667  240;   % q2
       -170  170;    % q3
       -180  180;    % q4
       -120  120;    % q5
       -360  360]);  % q6
    offset = deg2rad([0 -270 0 0 0 0]);  % el offset de q2 representa el codo en nuestro robot
    base = transl(0,0,0);
    tool = transl(0,0,-d_tool);
    
    %% Definicion del robot
    R = SerialLink(dh);
    R.name = name;
    R.qlim = qlim;
    R.offset = offset;
    R.base = base;
    R.tool = tool;

    % Opciones de estilo por defecto para plot (no dibuja ahora)
    if nargin < 1 || isempty(vis), vis = struct(); end
    if ~isfield(vis,'ws'),         vis.ws         = [-0.5 0.8 -0.5 0.8 0 1]; end
    if ~isfield(vis,'linkcolor'),  vis.linkcolor  = [0.35 0.36 0.38];      end
    if ~isfield(vis,'jointcolor'), vis.jointcolor = [0.95 0.45 0.10];      end
    % Escalas: links mas "finos" y juntas mas notorias
    ws = vis.ws; dims = [ws(2)-ws(1), ws(4)-ws(3), ws(6)-ws(5)];
    baseScale = max(1e-3, min(dims));
    % Juntas mucho más grandes por defecto
    if ~isfield(vis,'jointlen'),   vis.jointlen   = 0.95 * baseScale;      end
    if ~isfield(vis,'jointdiam'),  vis.jointdiam  = 1.15 * baseScale;      end
    if ~isfield(vis,'scale'),      vis.scale      = 0.55;                  end

    % Guardar estilo para aplicar en futuros R.plot(...)
    plotopt = {
        'workspace', vis.ws, ...
        'scale', vis.scale, ...
        'linkcolor', vis.linkcolor, ...
        'jointcolor', vis.jointcolor, ...
        'jointlen', vis.jointlen, ...
        'jointdiam', vis.jointdiam};

    % Pose "home" para ploteo por defecto (diferente a cero)
    % Pose de home mas "estirada" (puede ajustarse con vis.home_deg)
    if isfield(vis,'home_deg') && numel(vis.home_deg) == 6
        q_home_deg = vis.home_deg(:).';
    else
        q_home_deg = [25 -165 45 -90 -90 0];
    end
    q_home = deg2rad(q_home_deg);

    % Plot opcional si se solicita explicitamente
    if isfield(vis,'plot') && vis.plot
        R.plot(q_home, plotopt{:});
        axis(vis.ws); grid on; box on; axis vis3d; view(135,25);
        camlight headlight; lighting gouraud;
        title('Robot Scan Arm');
    end
end
