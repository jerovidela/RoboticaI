function [q_total, t_total, scan_log] = scan_orchestrator(R, cfg, q_init)
% scan_orchestrator  Orquesta escaneo en pasadas rectas con U-turns
%
% Resumen:
% - Genera N pasadas rectas (LSPB+IK) sobre una placa con separacion delta.
% - Entre pasadas, sube, cruza y baja (U-turn) con mstraj.
% - Acepta q_init para arrancar desde pose inicial ya posicionada.
%
% Genera N pasadas rectas (LSPB+IK) con U-turns (mstraj) y log por pasada.
% R  : SerialLink
% cfg: struct con campos:
%   W,H,m,delta,z_scan,h_clear,v_scan,a_scan,dt,R_orient,Vj_ut,tacc_ut
%
% Devuelve:
%   q_total [K x dof], t_total [K x 1], scan_log(1..Nscan) con:
%     k0,k1,T0,T1,ta,tc,dt

    W = cfg.W; H = cfg.H; m = cfg.m; delta = cfg.delta;
    z = cfg.z_scan; h = cfg.h_clear;
    v = cfg.v_scan; a = cfg.a_scan; dt = cfg.dt;
    Rz   = cfg.R_orient; 
    Vj   = cfg.Vj_ut; 
    tacc = cfg.tacc_ut;

    if isfield(cfg,'T_plate'), Tplate = cfg.T_plate; else, Tplate = eye(4); end

    % Geometría útil
    x0 = m; x1 = W - m; Lx = x1 - x0;
    Hutil = H - 2*m; jmax = floor(Hutil/delta);
    ylist = m + (0:jmax) * delta;
    Nscan = numel(ylist);

    q_total = zeros(0,R.n);
    t_total = zeros(0,1);
    scan_log = struct('k0',{},'k1',{},'T0',{},'T1',{},'ta',{},'tc',{},'dt',{});
    
    
    % --- INICIO DE LA MODIFICACIÓN ---
    if nargin < 3 || isempty(q_init)
        % Fallback si no se pasa q_init (comportamiento anterior)
        fprintf('ADVERTENCIA: No se proveyó q_init. Usando q_last = zeros().\n');
        q_last = zeros(1,R.n);
    else
        % Comportamiento deseado: empezar desde el q_init provisto
        q_last = q_init;
    end
    % --- FIN DE LA MODIFICACIÓN ---
    
    
    t_now = 0.0;

    for i = 1:Nscan
        y = ylist(i);
        sigma = 1 - 2*mod(i-1,2);
        if sigma > 0, xs=x0; xe=x1; else, xs=x1; xe=x0; end

        T0 = Tplate * transl(xs, y, z) * Rz;
        T1 = Tplate * transl(xe, y, z) * Rz;


        [q_scan, t_scan] = traj.make_scan_segment_inv(R, T0, T1, v, a, dt, q_last);

        k0 = size(q_total,1) + 1;
        k1 = k0 + size(q_scan,1) - 1;

        t_seg = t_now + t_scan;
        t_now = t_seg(end);

        q_total = [q_total; q_scan];
        t_total = [t_total; t_seg];

        % tiempos LSPB de esta fila
        ta = v/a;
        if Lx < v^2/a, ta = sqrt(Lx/a); tc = 0;
        else, tc = (Lx - v^2/a)/v;
        end

        scan_log(end+1) = struct('k0',k0,'k1',k1,'T0',T0,'T1',T1, ...
                                 'ta',ta,'tc',tc,'dt',dt);
        q_last = q_scan(end,:);

        if i < Nscan
            y_next = ylist(i+1);
            if sigma > 0, x_start_next = x1; else, x_start_next = x0; end
            T_next = Tplate * transl(x_start_next, y_next, z) * Rz;

            Tup   = Tplate * transl(xe,          y,       z+h) * Rz;
            Tmid  = Tplate * transl(x_start_next, y_next, z+h) * Rz;
            Tdown = T_next;

            poses_ut = cat(3, Tup, Tmid, Tdown);
            q_ut = traj.make_uturn_segment(R, poses_ut, q_scan(end,:), Vj, dt, tacc);
            q_ut(end,:) = R.ikcon(T_next, q_ut(end,:));

            Nu   = size(q_ut,1);
            t_ut = t_now + (0:Nu-1).' * dt;
            t_now = t_ut(end);

            q_total = [q_total; q_ut];
            t_total = [t_total; t_ut];
            q_last  = q_ut(end,:);
        end
    end
end
