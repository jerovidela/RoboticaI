function out = generate_cylinder_helix()
% generate_cylinder_helix  Genera una trayectoria helicoidal sobre un cilindro.
% Devuelve struct con: R, q_path (Nx6), t (Nx1), poses (1xN SE3), pos (Nx3), y geom.

baseDir = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(baseDir),'robots'));
tp08Dir = fullfile(fileparts(fileparts(baseDir)), 'TP08');
addpath(tp08Dir, fullfile(tp08Dir,'helpers'));

R = robot_G11(); R.offset = R.offset(:);

% Parámetros
WD = 0.12; v_scan = 0.12; f_line = 200; rho = v_scan/f_line;
R_cyl = 0.060; z_min = 0.29; z_max = 0.51; cx = 0.60; cy = 0.00;
pitch = 0.0475;

H = z_max - z_min; turns = H/pitch; nPerTurn = ceil(2*pi*(R_cyl+WD)/rho);
N = max(2, ceil(turns * nPerTurn)); tlin = linspace(0, 2*pi*turns, N).'; z = linspace(z_min, z_max, N).';

poses = SE3.empty;
Rtool = rotx(pi); % z_tool = -Z absoluto
for i=1:N
    th = tlin(i); zr = z(i);
    nrm = [cos(th); sin(th); 0];
    ps = [cx; cy; zr] + R_cyl*nrm;        % punto sobre superficie del cilindro
    pt = ps + WD * Rtool(1:3,3);          % desplaza -WD en Z
    poses(end+1) = SE3(rt2tr(Rtool, pt)); %#ok<SAGROW>
end

q_path = zeros(N, R.n); q_seed = zeros(R.n,1);
for i=1:N, qi = cin_inv_Faro(R, poses(i).T, q_seed, true); q_path(i,:) = qi.'; q_seed = qi; end

P = zeros(N,3); for i=1:N, P(i,:) = poses(i).t'; end
S = [0; cumsum(vecnorm(diff(P),2,2))]; t = S / v_scan;

out = struct('R',R,'q_path',q_path,'t',t,'poses',{poses},'pos',P, ...
             'geom',struct('R',R_cyl,'z_min',z_min,'z_max',z_max,'cx',cx,'cy',cy));
end
