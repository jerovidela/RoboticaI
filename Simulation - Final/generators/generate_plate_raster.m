function out = generate_plate_raster()
% generate_plate_raster  Raster en U por pares sobre una placa plana.
% Devuelve struct con: R, q_path (Nx6), t (Nx1), poses (1xN SE3), pos (Nx3), geom

% Carga robot y helpers (relativos a esta carpeta)
baseDir = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(baseDir),'robots'));
tp08Dir = fullfile(fileparts(fileparts(baseDir)), 'TP08');
addpath(tp08Dir, fullfile(tp08Dir,'helpers'));

R = robot_G11(); R.offset = R.offset(:);

% Parámetros del sensor y región útil
WD = 0.12;          % standoff
W_nom = 0.080;      % ancho útil nominal
overlap = 0.40;     % solapado lateral
W_eff = W_nom;      % sin tilt -> W_eff = W_nom
Delta = W_eff * (1 - overlap);     % separación entre franjas

v_scan = 0.12; f_line = 200; rho = v_scan / f_line; % muestreo longitudinal

% Dimensiones de placa y marco de escaneo {S}
H = 0.28; L = 0.38;     % alto (transversal c) y largo (longitudinal s)
z0 = 0.35; cx = 0.60; cy = 0.00;  % centro y cota
s_hat = [1;0;0]; c_hat = [0;1;0]; n_hat = [0;0;1];
s_min = -L/2; s_max = L/2; c_min = -H/2; c_max = H/2;
p0 = [cx; cy; z0];               % origen mundo de {S}

% Orientación de herramienta: z_tool = -Z absoluto (sin tilt)
Rtool = rotx(pi);

% Generación de U por pares
c_vals = c_min:Delta:c_max; if c_vals(end) < c_max, c_vals(end+1) = c_max; end
poses = SE3.empty;  %#ok<NASGU>
rb = Delta/2;       % radio de blend superior
lift = 0.05;        % elevación para saltos entre pares

to_world = @(s,c,h) p0 + s*s_hat + c*c_hat + h*n_hat; % (s,c,h)->R^3

for j = 1:2:numel(c_vals)-1
    cl = c_vals(j); cr = c_vals(j+1); midc = (cl+cr)/2; r = (cr-cl)/2; if r<=0, r = rb; end
    % 1) Subida por franja izquierda (s: s_min->s_max, c=cl)
    for s = s_min:rho:s_max
        p_surf = to_world(s, cl, 0); p_tcp = p_surf + WD * Rtool(1:3,3);
        poses(end+1) = SE3(rt2tr(Rtool, p_tcp)); %#ok<AGROW>
    end
    % 2) Codo superior (semicírculo en (s,c) hacia +s)
    thetas = linspace(-pi/2, pi/2, max(8,ceil(pi*r/rho)));
    for th = thetas
        s = s_max + r*cos(th); c = midc + r*sin(th);
        p_surf = to_world(s, c, 0); p_tcp = p_surf + WD * Rtool(1:3,3);
        poses(end+1) = SE3(rt2tr(Rtool, p_tcp));
    end
    % 3) Bajada por franja derecha (s: s_max->s_min, c=cr)
    for s = s_max:-rho:s_min
        p_surf = to_world(s, cr, 0); p_tcp = p_surf + WD * Rtool(1:3,3);
        poses(end+1) = SE3(rt2tr(Rtool, p_tcp));
    end
    % 4) Retiro elevado y salto al siguiente par (si existe)
    if j+2 <= numel(c_vals)
        cnxt = c_vals(j+2);
        % elevar
        for h = linspace(0, lift, max(3,ceil(lift/rho)))
            p_surf = to_world(s_min, cr, h); p_tcp = p_surf + WD * Rtool(1:3,3);
            poses(end+1) = SE3(rt2tr(Rtool, p_tcp));
        end
        % mover en c (transversal) elevado
        for c = linspace(cr, cnxt, max(5,ceil(abs(cnxt-cr)/rho)))
            p_surf = to_world(s_min, c, lift); p_tcp = p_surf + WD * Rtool(1:3,3);
            poses(end+1) = SE3(rt2tr(Rtool, p_tcp));
        end
        % bajar
        for h = linspace(lift, 0, max(3,ceil(lift/rho)))
            p_surf = to_world(s_min, cnxt, h); p_tcp = p_surf + WD * Rtool(1:3,3);
            poses(end+1) = SE3(rt2tr(Rtool, p_tcp));
        end
    end
end

% IK con continuidad
q_path = zeros(numel(poses), R.n); q_seed = zeros(R.n,1);
for i=1:numel(poses)
    qi = cin_inv_Faro(R, poses(i).T, q_seed, true);
    q_path(i,:) = qi.'; q_seed = qi;
end

% Tiempo y posiciones
P = zeros(numel(poses),3); for i=1:numel(poses), P(i,:) = poses(i).t'; end
S = [0; cumsum(vecnorm(diff(P),2,2))]; t = S / v_scan;

% Geometría en mundo para dibujar la placa
x_min = cx - L/2; x_max = cx + L/2; y_min = cy - H/2; y_max = cy + H/2;

out = struct('R',R,'q_path',q_path,'t',t,'poses',{poses},'pos',P, ...
             'geom',struct('x_min',x_min,'x_max',x_max,'y_min',y_min,'y_max',y_max,'z0',z0));
end

