% FANUC Ejercicio 3 - Interpolación cartesiana (ctraj) + IK y animación
clear; clc; close all; 

% Definición del robot (FANUC Paint Mate 200iA)
dh=[ 
0, 0.45, 0.075, -pi/2, 0; 
0, 0, 0.3, 0, 0;
0, 0, 0.075, -pi/2, 0;  
0, 0.32, 0,pi/2, 0;  
0,0,0,-pi/2,0;
0, 0.008, 0, 0, 0;      
];
fanuc_robot = SerialLink(dh, 'name', 'FANUC Paint Mate 200iA');

% Puntos cartesianos y semilla/orientación
P1 = [0, 0, 0.95]; 
P2 = [0.4, 0, 0.95]; 
qq = [0, -pi/2, -pi/4, 0, pi/4, 0];
N = 100;
t = linspace(0, 1, N);

% Orientación fija tomada de la semilla
T_orientacion = fanuc_robot.fkine(qq); 
R = T_orientacion.R; 
T1 = [R, P1'; 0, 0, 0, 1]; 
T2 = [R, P2'; 0, 0, 0, 1]; 

% Trayectoria cartesiana recta con ctraj
Ttray = ctraj(T1, T2, N);   % trayectoria cartesiana

% IK punto a punto (posición solamente)
q_tray = zeros(N, 6);
for i = 1:N
    q_tray(i,:) = fanuc_robot.ikine(Ttray(:,:,i), 'q0', qq, 'mask', [1 1 1 0 0 0]);
end
q_deg = rad2deg(q_tray);

% Figura maximizada con subplots 1x2
fig = figure('Color','w');
set(fig, 'Renderer', 'opengl');
try
    set(fig, 'WindowState', 'maximized');
catch
    set(fig, 'Units','normalized','Position',[0 0 1 1]);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Subplot izquierdo: animación del robot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ax1 = subplot(1,2,1);
title(ax1, 'Animación del robot'); grid(ax1,'on'); axis(ax1,'equal');
plot_opts = {'noname','nojaxes','noshadow','scale',0.8};
fanuc_robot.plot(q_tray(1,:), plot_opts{:});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Subplot derecho: q(t) en grados
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ax2 = subplot(1,2,2);
hold(ax2,'on'); grid(ax2,'on');
title(ax2, 'Trayectorias articulares q(t)');
xlabel(ax2, 'Índice (0..1)'); ylabel(ax2, 'q [deg]');
colors = lines(6);
al = gobjects(1,6);

for j = 1:6
    plot(ax2, t, q_deg(:,j), 'x', 'Color', colors(j,:), 'HandleVisibility','off');
    al(j) = animatedline(ax2, 'Color', colors(j,:), 'LineWidth', 1.2);
    addpoints(al(j), t(1), q_deg(1,j));
end
legend(ax2, {'q1','q2','q3','q4','q5','q6'}, 'Location','best');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               ANIMACIÓN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

updateEvery = 2;
for i = 1:N
    fanuc_robot.animate(q_tray(i,:));
    for j = 1:6
        addpoints(al(j), t(i), q_deg(i,j));
    end
    if mod(i, updateEvery) == 0 || i == N
        drawnow limitrate nocallbacks;
    end
end
