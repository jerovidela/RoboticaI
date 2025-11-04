% FANUC Ejercicio 1 - jtraj básico
clear; clc; close all;

T = 3;
delta = 0.1;
t = 0:delta:T; % Vector de tiempo

q_inicial = [0, -pi/2, 0, 0, 0, 0];
q_final   = [-pi/3, pi/10, -pi/5, pi/2, pi/4, 0];

[q, qd, qq] = jtraj(q_inicial, q_final, t);
q_deg = rad2deg(q); % para graficar en grados

dh = [
    0, 0.45,  0.075, -pi/2, 0;
    0, 0,     0.3,    0,    0;
    0, 0,     0.075, -pi/2, 0;
    0, 0.32,  0,      pi/2,  0;
    0, 0,     0,     -pi/2,  0;
    0, 0.008, 0,      0,    0
];

fanuc_robot = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');

% Figura con subplots 1x2 y ajustes para fluidez
fig = figure('Color','w');
set(fig, 'Renderer', 'opengl');

% Maximizar ventana (compatibilidad con versiones viejas)
try
    set(fig, 'WindowState', 'maximized');
catch
    set(fig, 'Units', 'normalized', 'Position', [0 0 1 1]);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Subplot izquierdo: animación del robot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ax1 = subplot(1,2,1);
title(ax1, 'Animación del robot'); grid(ax1,'on'); axis(ax1,'equal');

%  Opciones de plot para aligerar 
plot_opts = {'noname','nojaxes','noshadow','scale',0.8};
fanuc_robot.plot(q(1,:), plot_opts{:});


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Subplot derecho: q(t) en grados
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ax2 = subplot(1,2,2);
hold(ax2,'on'); grid(ax2,'on');
title(ax2, 'Trayectorias articulares q(t)');
xlabel(ax2, 'Tiempo [s]'); ylabel(ax2, 'q [deg]');
colors = lines(6);
al = gobjects(1,6);

% Grafica todas las "curvas" en el mismo plot con diferentes colores
for j = 1:6
    al(j) = animatedline(ax2, 'Color', colors(j,:), 'LineWidth', 1.2);
    addpoints(al(j), t(1), q_deg(1,j));
end
legend(ax2, {'q1','q2','q3','q4','q5','q6'}, 'Location','best');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               ANIMACIÓN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = numel(t);
tic;
updateEvery = 2; % actualizar render cada 2 muestras para aumentar FPS percibidos
for i = 1:N
    % Actualizar robot (más liviano que replot)
    fanuc_robot.animate(q(i,:));

    % Agregar un punto por articulación (q en grados)
    for j = 1:6
        addpoints(al(j), t(i), q_deg(i,j));
    end

    % Refrescar a tasa limitada
    if mod(i, updateEvery) == 0 || i == N
        drawnow limitrate nocallbacks;
    end

    % Sincronizar con tiempo real (respetar paso delta)
    if i < N
        target = t(i+1);
        while toc < target
            pause(0.001);
        end
    end
end
