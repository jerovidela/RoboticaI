function fix_and_verify_arm()
% Ejecutar desde: Simulation - Final/Design
% Arregla rutas/escala del URDF, verifica que existan los STL, muestra el robot y guarda PNG.

clc; close all;
designFolder = pwd;
urdfIn  = fullfile(designFolder,'articulated_arm_auto.urdf');
urdfOut = fullfile(designFolder,'articulated_arm_auto_patched.urdf');

% --- 0) Sanidad de archivos esperados
expected = ["link_part_00.stl","link_part_01.stl","link_part_02.stl", ...
            "link_part_03.stl","link_part_04.stl","link_part_05.stl","link_part_06.stl"];
miss = expected(arrayfun(@(f) exist(fullfile(designFolder,f),'file')~=2, expected));
assert(isfile(urdfIn), 'No encuentro %s', urdfIn);
assert(isempty(miss),  "Faltan STL: \n%s", strjoin(miss,newline));

% --- 1) Parchear URDF: rutas absolutas + escala (ajusta aquí si tu STL está en mm)
SCALE = 1;         % pon 0.001 si tus STL están en milímetros
patchURDF(urdfIn, urdfOut, designFolder, SCALE);

% --- 2) Importar y verificar que carguen VISUALES
warning('on','robotics:robot:urdfimporter:MeshFileNotFound');
robot = importrobot(urdfOut, "DataFormat","column", "MeshPath", designFolder);

% ¿Tienen visuals todos los cuerpos?
nb = numel(robot.Bodies);
hasVisual = false(nb,1);
for i=1:nb
    V = robot.Bodies{i}.Visuals;
    hasVisual(i) = ~isempty(V);
end
assert(all(hasVisual), "Algún body está sin Visuals (malla). Revisa nombres y escala.");

% --- 3) Mostrar con ejes decentes
fig = figure('Name','Arm (patched)','Color','w','Renderer','opengl');
ax = axes('Parent',fig); hold(ax,'on');
show(robot, "Parent", ax, "Visuals","on","Collisions","off");
axis(ax,'equal'); grid(ax,'on'); view(ax, 135,20); camlight(ax); lighting(ax,'gouraud');
title(ax, 'Arm at home configuration (with meshes)');

% --- 4) Mover 6 DoF para confirmar articulación
nj = numel(homeConfiguration(robot));
assert(nj==6, 'Este URDF tiene %d DoF; esperaba 6.', nj);
for i=1:nj
    q = zeros(nj,1); q(i) = deg2rad(30);
    show(robot,q,"Parent",ax,"PreservePlot",false,"Visuals","on"); drawnow; pause(0.15);
end

% --- 5) Guardar snapshot sin UI
png = fullfile(designFolder,'arm_design_preview.png');
exportgraphics(ax, png, 'Resolution', 200);
fprintf('OK: preview guardado en %s\n', png);
end

function patchURDF(fin, fout, meshPath, scale)
% Reescribe todos los <mesh> a ruta absoluta y aplica <scale>
doc = xmlread(fin);
meshes = doc.getElementsByTagName('mesh');
for k=0:meshes.getLength-1
    m = meshes.item(k);
    fname = char(m.getAttribute('filename'));
    [~,n,e] = fileparts(fname);
    absname = fullfile(meshPath, [n e]);   % asegura nombre exacto
    m.setAttribute('filename', absname);
    if nargin>=4 && ~isempty(scale)
        s = sprintf('%g %g %g', scale, scale, scale);
        m.setAttribute('scale', s);
    end
end
xmlwrite(fout, doc);
end
