% FANUC Ejercicio 5 - Multisegmento con frenado (jtraj) + animación
clear; clc; close all;

% Definición del robot (FANUC Paint Mate 200iA)
dh=[
0, 0.45, 0.075, -pi/2, 0;
0, 0, 0.3, 0, 0;
0, 0, 0.075, -pi/2, 0;
0, 0.32, 0, pi/2, 0;
0, 0, 0, -pi/2, 0;
0, 0.008, 0, 0, 0;
];
fanuc_robot = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');

% Puntos articulares propuestos
q1 = [0, -pi/2, -pi/2, 0, pi/2, 0];
q2 = [pi/2, -pi/2, -pi/2, 0, pi/2, 0];
q3 = [pi/6, -pi/6, 0, 0, pi/6, 0];
q4 = [pi/3, -pi/4, pi/6, 0, pi/3, 0];
N  = 100;         % puntos por segmento
dt = 0.05;        % paso temporal nominal para animación

% Trayectorias con frenado entre segmentos
[q12, qd12, qdd12] = jtraj(q1, q2, N);
[q23, qd23, qdd23] = jtraj(q2, q3, N);
[q34, qd34, qdd34] = jtraj(q3, q4, N);

% Unir segmentos
q_total   = [q12; q23; q34];
qd_total  = [qd12; qd23; qd34];
qdd_total = [qdd12; qdd23; qdd34];
q_deg     = rad2deg(q_total);
T = size(q_total,1);
t = linspace(0, dt*(T-1), T);  % vector de tiempo

% Figura maximizada con subplots 1x2 (animación + q(t))
fig = figure('Color','w');
set(fig, 'Renderer', 'opengl');
try
    set(fig, 'WindowState', 'maximized');
catch
    set(fig, 'Units','normalized','Position',[0 0 1 1]);
end

% Subplot (1,2,1): animación
ax1 = subplot(1,2,1);
title(ax1, 'Animación por segmentos (pulse una tecla o el botón)');
grid(ax1,'on'); axis(ax1,'equal');
plot_opts = {'noname','nojaxes','noshadow','scale',0.8};
fanuc_robot.plot(q_total(1,:), plot_opts{:});

% Botón para continuar al próximo segmento
btn = uicontrol('Style','pushbutton','String','Continuar', ...
    'Units','normalized','Position',[0.46 0.93 0.1 0.05], ...
    'Callback',@(s,e) uiresume(fig)); %#ok<NASGU>

% Subplot (1,2,2): q(t) en grados con líneas animadas
ax2 = subplot(1,2,2);
hold(ax2,'on'); grid(ax2,'on');
title(ax2, 'Trayectorias articulares q(t)');
xlabel(ax2, 'Tiempo [s]'); ylabel(ax2, 'q [deg]');
colors = lines(6);
al = gobjects(1,6);
for j = 1:6
    al(j) = animatedline(ax2, 'Color', colors(j,:), 'LineWidth', 1.2);
    addpoints(al(j), t(1), q_deg(1,j));
end
legend(ax2, {'q1','q2','q3','q4','q5','q6'}, 'Location','best');

% Animar cada segmento por separado, con key/button interrupt
segments = {q12, q23, q34};
segStarts = [1, N+1, 2*N+1];
segEnds   = [N, 2*N, 3*N];
tic;
for s = 1:numel(segments)
    iStart = segStarts(s);
    iEnd   = segEnds(s);
    for i = iStart:iEnd
        fanuc_robot.animate(q_total(i,:));
        for j = 1:6
            addpoints(al(j), t(i), q_deg(i,j));
        end
        if mod(i,2)==0 || i==iEnd
            drawnow limitrate nocallbacks;
        end
        % Sincronizar a tiempo aproximado
        if i < iEnd
            target = t(i) + dt;
            while toc < target, pause(0.001); end
        end
    end
    % Pausa hasta tecla o botón
    try
        wasKey = waitforbuttonpress; %#ok<NASGU>
        uiwait(fig, 1); % si no fue tecla, permitir botón
    catch
        break; % si cerraron la figura, terminar
    end
end

% Figura 2: perfiles completos q, qd, qdd
fig2 = figure('Color','w');
try, set(fig2,'WindowState','maximized'); catch, set(fig2,'Units','normalized','Position',[0 0 1 1]); end
subplot(3,1,1); plot(t, q_deg); title('Posición articular (deg)'); ylabel('deg'); grid on; legend('q1','q2','q3','q4','q5','q6','Location','best');
subplot(3,1,2); plot(t, qd_total); title('Velocidad articular (rad/s)'); ylabel('rad/s'); grid on;
subplot(3,1,3); plot(t, qdd_total); title('Aceleración articular (rad/s^2)'); ylabel('rad/s^2'); xlabel('Tiempo [s]'); grid on;

