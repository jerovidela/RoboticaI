% plot_robot_frames.m — RTB clásico, sin autoscale tóxico y sin 'thick'
clear; clc; close all;

% 1) Cargar el robot SIN dibujar nada en robot.m
run('robot.m');                 % deja R, q_home, workspace en memoria
q = evalin('base','q_home');

% 2) Figura + axes y cámara decentes
fig = figure('Name','Escaner6R - Frames');
ax  = axes('Parent', fig); hold(ax,'on'); grid(ax,'on');

axis(ax, 'manual');             % congelar autoscale
daspect(ax, [1 1 1]);           % proporción 1:1:1
pbaspect(ax, [1 1 1]);
axis(ax, 'vis3d');
camproj(ax, 'orthographic');
view(ax, 60, 25);
camup(ax, [0 0 1]);
camtarget(ax, [0 0 0.8]);

% Límites visibles (ajusta a gusto)
xlim(ax, [-0.5 1.2]);
ylim(ax, [-0.5 0.5]);
zlim(ax, [-0.3 1.0]);

xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
title(ax,'Escaner6R - Frames');

% 3) Dibujar el robot en ESTE axes
axes(ax);
R.plot(q, ...
    'jointdiam', 0.5, ...
    'noname','notiles','delay',0,'scale',0.8);

% 4) Dibujar frames {0..n}
sistemas = ones(1, R.n+1);  % todos
LEN = 0.18;                 % tamaño de ejes de los frames
LW  = 1.5;                  % grosor de línea numérico (no 'thick')

T = R.base;
if sistemas(1)
    axes(ax);
    trplot(T, 'frame','0', 'length',LEN, 'rgb', 'arrow', 'LineWidth', LW);
end

for i = 1:R.n
    T = T * R.links(i).A(q(i));
    if sistemas(i+1)
        axes(ax);
        trplot(T, 'frame', num2str(i), 'length',LEN, 'rgb', 'arrow', 'LineWidth', LW);
    end
end

% Frame de la TOOL
axes(ax);
trplot(T*R.tool, 'frame','T', 'length',LEN, 'rgb', 'arrow', 'LineWidth', LW);

% 5) Reafirmar límites
xlim(ax, [-0.5 1.2]); ylim(ax, [-0.5 0.5]); zlim(ax, [-0.3 1.0]);
drawnow;
