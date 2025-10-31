% Simulation - Final: main
% Ejecuta simulaciones de placa (raster) y cilindro (hélice) usando
% utilidades separadas para animación y gráficos.

clear; clc; close all;

% Generadores
addpath(fullfile(pwd, 'generators'));
addpath(fullfile(pwd, 'utils'));
addpath(fullfile(pwd, 'robots'));

%% 1) Placa raster
plate = generate_plate_raster();

% Escena y animación
figP = figure('Name','Placa - Animación y q(t)');
try, set(figP,'WindowState','maximized'); catch, set(figP,'Units','normalized','OuterPosition',[0 0 1 1]); end
subplot(1,2,1); ax = gca; grid(ax,'on'); axis(ax,'equal'); view(ax,135,25); hold(ax,'on');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Placa - Animación');
draw_plate(ax, plate.geom.x_min, plate.geom.x_max, plate.geom.y_min, plate.geom.y_max, plate.geom.z0, 0.006);
plot3(plate.pos(:,1), plate.pos(:,2), plate.pos(:,3), 'k--');
animate_robot(plate.R, plate.q_path, plate.t, 'Placa - Animación', ...
    'stride', 3, 'targetSeconds', 45, 'ax', ax, 'showTrail', true, 'trailColor', [0 0 0]);

subplot(1,2,2);
plot_q_only(plate.t, plate.q_path, 'Placa - q(t) [deg]');

% Perfiles
[posP, velP, accP] = compute_cartesian_profiles(plate.R, plate.q_path, plate.t);
plot_xyz_profiles(plate.t, posP, velP, accP, 'Placa');
plot_q_qd_qdd(plate.t, plate.q_path, 'Placa - Perfiles articulares');

%% 2) Cilindro hélice
cyl = generate_cylinder_helix();

figC = figure('Name','Cilindro - Animación y q(t)');
try, set(figC,'WindowState','maximized'); catch, set(figC,'Units','normalized','OuterPosition',[0 0 1 1]); end
subplot(1,2,1); ax = gca; grid(ax,'on'); axis(ax,'equal'); view(ax,135,25); hold(ax,'on');
xlabel('X'); ylabel('Y'); zlabel('Z'); title('Cilindro - Animación');
draw_cylinder(ax, cyl.geom.R, cyl.geom.z_min, cyl.geom.z_max, cyl.geom.cx, cyl.geom.cy);
plot3(cyl.pos(:,1), cyl.pos(:,2), cyl.pos(:,3), 'k--');
animate_robot(cyl.R, cyl.q_path, cyl.t, 'Cilindro - Animación', ...
    'stride', 3, 'targetSeconds', 45, 'ax', ax, 'showTrail', true, 'trailColor', [0 0 0]);

subplot(1,2,2);
plot_q_only(cyl.t, cyl.q_path, 'Cilindro - q(t) [deg]');

[posC, velC, accC] = compute_cartesian_profiles(cyl.R, cyl.q_path, cyl.t);
plot_xyz_profiles(cyl.t, posC, velC, accC, 'Cilindro');
plot_q_qd_qdd(cyl.t, cyl.q_path, 'Cilindro - Perfiles articulares');

disp('Simulaciones completadas.');
