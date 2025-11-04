% Demo: Raster sobre placa (usa generator)
addpath(fullfile(fileparts(mfilename('fullpath')),'..','generators'));
addpath(fullfile(fileparts(mfilename('fullpath')),'..','utils'));
addpath(fullfile(fileparts(mfilename('fullpath')),'..','robots'));

plate = generate_plate_raster();
fig = animate_robot(plate.R, plate.q_path, plate.t, 'Placa - Animación');
plot_q_only(plate.t, plate.q_path, 'Placa - q(t) [deg]');

