% Demo: Hélice sobre cilindro (usa generator)
addpath(fullfile(fileparts(mfilename('fullpath')),'..','generators'));
addpath(fullfile(fileparts(mfilename('fullpath')),'..','utils'));
addpath(fullfile(fileparts(mfilename('fullpath')),'..','robots'));

cyl = generate_cylinder_helix();
fig = animate_robot(cyl.R, cyl.q_path, cyl.t, 'Cilindro - Animación'); %#ok<NASGU>
plot_q_only(cyl.t, cyl.q_path, 'Cilindro - q(t) [deg]');

