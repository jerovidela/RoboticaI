% verify_arm_design.m
% Verifica que el URDF + STLs carguen bien en MATLAB y dibuja el brazo.
% Requisitos: Robotics System Toolbox.
%
% Ubicación: guardar este archivo dentro de:
%   Simulation - Final/Design/
%
% Uso:
%   >> cd('.../Simulation - Final/Design')
%   >> verify_arm_design

close all; clc;

%% 0) Config general
designFolder = fileparts(mfilename('fullpath'));
cd(designFolder);
expectedFiles = ["articulated_arm_auto.urdf", ...
                 "link_part_00.stl","link_part_01.stl","link_part_02.stl", ...
                 "link_part_03.stl","link_part_04.stl","link_part_05.stl","link_part_06.stl"];

%% 1) Chequeo de archivos
missing = expectedFiles(arrayfun(@(f) exist(fullfile(designFolder,f),'file')~=2, expectedFiles));
if ~isempty(missing)
    error("Faltan archivos en %s:\n%s", designFolder, strjoin(missing,newline));
end
fprintf("OK: %d archivos encontrados.\n", numel(expectedFiles));

%% 2) Importar el robot desde URDF
try
    robot = importrobot(fullfile(designFolder,'articulated_arm_auto.urdf'), ...
        "DataFormat","column","MeshPath", designFolder);
catch ME
    if contains(ME.message, "Undefined function 'importrobot'")
        error("Falta Robotics System Toolbox. Instálalo o activa la licencia.");
    else
        rethrow(ME);
    end
end

%% 3) Sanidad básica del árbol
nj = numel(homeConfiguration(robot));
assert(nj==6, "Este URDF trae %d juntas, esperaba 6.", nj);
fprintf("OK: %d juntas encontradas.\n", nj);
showdetails(robot);

%% 4) Plot inicial
fig = figure('Name','Arm Design Check','Color','w'); 
show(robot, "Collisions","off","Visuals","on"); 
axis equal; grid on; view(135,20); camlight; title('Arm at home configuration');

%% 5) Chequear ejes uno por uno (mueve 30° y vuelve)
config = zeros(nj,1);
for i = 1:nj
    config(:) = 0;
    config(i) = deg2rad(30);
    show(robot, config, "PreservePlot", false, "Collisions","off","Visuals","on"); 
    drawnow; pause(0.15);
    config(i) = 0;
end
title('Axes sanity check done');

%% 6) Trayectoria cortita para ver si todo se articula sin locuras
frames = 120;
for t = linspace(0, 1, frames)
    q = zeros(nj,1);
    q(1) = (t-0.5)*pi/2;     % base yaw
    q(2) = sin(2*pi*t)*pi/6; % hombro
    q(3) = cos(2*pi*t)*pi/8; % codo
    q(4) = sin(4*pi*t)*pi/5; % muñeca roll
    q(5) = cos(4*pi*t)*pi/6; % muñeca pitch
    q(6) = sin(6*pi*t)*pi/3; % roll final
    show(robot, q, "PreservePlot", false, "Collisions","off","Visuals","on"); 
    drawnow;
end
title('Quick motion sweep');

%% 7) Guardar snapshot (sin UI, solo el eje 3D)
fname = fullfile(designFolder, 'arm_design_preview.png');

try
    ax = gca;                                   % exportar SOLO el eje
    exportgraphics(ax, fname, 'Resolution', 180);
catch
    % Plan B: captura de pantalla del eje si exportgraphics no coopera
    frame = getframe(gca);
    imwrite(frame.cdata, fname);
end

if exist(fname,'file')
    fprintf("OK: snapshot guardado en %s\n", fname);
else
    warning("No se pudo guardar el snapshot. Probá 'saveas(gcf, fname)'.");
end

