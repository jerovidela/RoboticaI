function fig = animate_path(R, q_total, t_total, cfg, scan_log)
% animate_path  Animacion 3D de la trayectoria con placa y medidores
%
% Resumen:
% - Anima el robot siguiendo q_total vs t_total y traza del TCP.
% - Dibuja placa, colorea pasadas (scan_log) y muestra barras de |v_tip| y metrica jacobiana.
% - Puede grabar video si cfg.save_video=true.
%
% Entradas:
%   R: SerialLink; q_total [K x dof]; t_total [K x 1]
%   cfg: struct con campos de visual (ws,fps,trail,plate,video,...)
%   scan_log: struct de segmentos de escaneo (opcional)
% Salidas:
%   fig: handle de la figura creada
% Animación con traza del TCP y placa dibujada.
% cfg:
%   .ws           = [-1.5 1 -1.5 1 0 1]   % workspace
%   .fps          = 30                    % frames por segundo destino
%   .trail_color  = [0 0 0]               % color de la traza
%   .trail_width  = 1.5
%   .plate.center = [0.45 0.00 0.20]      % opción B (centro de placa, TCP)
%   .plate.size   = [W H]                 % dimensiones de la placa
%   .plate.color  = [0.6 0.8 1.0]
%   .save_video   = false                 % o true
%   .video_file   = 'scan.mp4'

    arguments
        R
        q_total double
        t_total double
        cfg struct = struct()
        scan_log struct = struct('k0',{},'k1',{})
    end

    % defaults
    if ~isfield(cfg,'ws'),          cfg.ws = [-0.5 0.8 -0.5 0.8 0 1]; end
    if ~isfield(cfg,'fps'),         cfg.fps = 30; end
    if ~isfield(cfg,'trail_color'), cfg.trail_color = [0 0 0]; end
    if ~isfield(cfg,'trail_width'), cfg.trail_width = 1.5; end
    if ~isfield(cfg,'plate') || ~isfield(cfg.plate,'center')
        cfg.plate.center = [0.45 0 0.20];  % tu opción B
    end
    if ~isfield(cfg.plate,'size'),  cfg.plate.size = [0.40 0.30]; end
    if ~isfield(cfg.plate,'color'), cfg.plate.color = [0.72 0.75 0.78]; end  % metallic gray
    if ~isfield(cfg,'save_video'),  cfg.save_video = false; end
    if ~isfield(cfg,'video_file'),  cfg.video_file = 'scan.mp4'; end

    dt = mean(diff(t_total));
    step = max(1, round(1/(cfg.fps*dt)));   % salto de muestras para animar

    % Precompute punta
    N = size(q_total,1);
    P = zeros(N,3);
    for k=1:N
        T = R.fkine(q_total(k,:));
        P(k,:) = transl(T);
    end

    % Precompute tip speed magnitude and reference
    try
        V = graphs.compute_derivatives(t_total, P, 1, 0.35, 3);
    catch
        V = [zeros(1,3); diff(P,1,1) ./ max(diff(t_total), eps)];
        if size(V,1) < N, V(end+1:N,:) = 0; end
    end
    vmag = sqrt(sum(V.^2,2));
    if ~isfield(cfg,'speed_ref') || isempty(cfg.speed_ref)
        cfg.speed_ref = max(1e-6, prctile(vmag, 99));
    end

    % Figura y robot
    fig = figure('Name','ScanArm Simulation','Color','w','Visible','on');
    % Fijar tamaño de figura para capturas consistentes
    try
        set(fig,'Units','pixels');
        pos = get(fig,'Position');
        if isfield(cfg,'video_size') && numel(cfg.video_size)==2
            sz = cfg.video_size; w = sz(1); h = sz(2);
        else
            w = 1280; h = 720;
        end
        set(fig,'Position',[pos(1) pos(2) w h]);
        set(fig,'Resize','off');
    catch
    end
    R.plot(q_total(1,:), 'workspace', cfg.ws, 'nowrist', 'noshadow', 'noname', 'delay', 0);
    hold on; grid on; grid minor; box on; axis equal; axis vis3d; view(45,25);
    lighting gouraud; camlight headlight; material metal;

    % Placa
    simulation.draw_plate(cfg.plate.center, cfg.plate.size, cfg.plate.color);

    % Traza completa (gris tenue al fondo) y traza “viva”
    plot3(P(:,1), P(:,2), P(:,3), '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
    htrail = plot3(NaN,NaN,NaN, '-', 'Color', cfg.trail_color, 'LineWidth', cfg.trail_width);

    % Si te pasaron scan_log, marcá cada pasada con color
    if ~isempty(scan_log)
        for s = 1:numel(scan_log)
            k0 = scan_log(s).k0; k1 = scan_log(s).k1;
            plot3(P(k0:k1,1), P(k0:k1,2), P(k0:k1,3), ...
                '-', 'LineWidth', 1.6, 'Color', [0.2 0.5 1.0 0.9]);
        end
    end

    % Gauges for |v_tip| and Jacobian metric
    axSpeed = axes('Position',[0.10 0.06 0.80 0.045],'Units','normalized');
    axis(axSpeed,[0 1 0 1]); axSpeed.XTick=[]; axSpeed.YTick=[]; box(axSpeed,'on'); hold(axSpeed,'on');
    rectangle(axSpeed,'Position',[0 0 1 1],'FaceColor',[0.9 0.9 0.9],'EdgeColor',[0.7 0.7 0.7]);
    hBarSpeed = patch(axSpeed,[0 0 0 0],[0 1 1 0],[0.20 0.60 0.25],'EdgeColor','none');
    hTxtSpeed = text(axSpeed,0.01,0.5,'|v_{tip}| = 0.000 m/s','HorizontalAlignment','left','VerticalAlignment','middle','FontSize',9);
    text(axSpeed,0.99,0.5,sprintf('ref=%.3f m/s', cfg.speed_ref),'HorizontalAlignment','right','VerticalAlignment','middle','FontSize',9,'Color',[0 0 0]+0.3);

    axJac = axes('Position',[0.10 0.01 0.80 0.045],'Units','normalized');
    axis(axJac,[0 1 0 1]); axJac.XTick=[]; axJac.YTick=[]; box(axJac,'on'); hold(axJac,'on');
    rectangle(axJac,'Position',[0 0 1 1],'FaceColor',[0.92 0.92 0.92],'EdgeColor',[0.7 0.7 0.7]);
    hBarJac = patch(axJac,[0 0 0 0],[0 1 1 0],[0.25 0.45 0.85],'EdgeColor','none');
    hTxtJac = text(axJac,0.01,0.5,'J metric = n/a','HorizontalAlignment','left','VerticalAlignment','middle','FontSize',9);
    if ~isfield(cfg,'jac_metric') || isempty(cfg.jac_metric), cfg.jac_metric = 'cond'; end
    if ~isfield(cfg,'jac_cond_max') || isempty(cfg.jac_cond_max), cfg.jac_cond_max = 1000; end
    if strcmpi(cfg.jac_metric,'cond')
        jacLabel = sprintf('cond(J) max %.0f', cfg.jac_cond_max);
    else
        jacLabel = 'manipulability w(J)';
    end
    text(axJac,0.99,0.5,jacLabel,'HorizontalAlignment','right','VerticalAlignment','middle','FontSize',9,'Color',[0 0 0]+0.3);

    % Video opcional
    if cfg.save_video
        % asegurar carpeta de salida
        try
            [vidDir,~,~] = fileparts(cfg.video_file);
            if ~isempty(vidDir) && ~exist(vidDir,'dir'), mkdir(vidDir); end
        catch
        end
        vw = VideoWriter(cfg.video_file, 'MPEG-4');
        vw.FrameRate = cfg.fps;
        open(vw);
        % Capturar un primer frame para fijar el tamaño del video
        try
            writeVideo(vw, getframe(fig));
        catch
        end
    else
        vw = [];
    end

    % Animación
    for k = 1:step:N
        R.animate(q_total(k,:));
        set(htrail,'XData',P(1:k,1),'YData',P(1:k,2),'ZData',P(1:k,3));
        % update speed bar
        v = vmag(k);
        w = min(max(v / max(cfg.speed_ref,1e-9), 0), 1);
        set(hBarSpeed,'XData',[0 w w 0]);
        set(hTxtSpeed,'String',sprintf('|v_{tip}| = %.3f m/s', v));
        % update Jacobian metric bar
        try
            J = R.jacob0(q_total(k,:));
            if strcmpi(cfg.jac_metric,'manip')
                val = sqrt(max(real(det(J*J.')),0));
                vmax_prev = getappdata(fig,'jac_val_max'); if isempty(vmax_prev), vmax_prev = val; end
                vmax = max(vmax_prev, val + eps);
                setappdata(fig,'jac_val_max', vmax);
                wj = min(max(val / max(vmax,1e-9), 0), 1);
                set(hTxtJac,'String',sprintf('w(J) = %.3g', val));
            else
                val = cond(J);
                wj = min(max(val / max(cfg.jac_cond_max,1e-9), 0), 1);
                set(hTxtJac,'String',sprintf('cond(J) = %.1f', val));
            end
        catch
            wj = 0; set(hTxtJac,'String','J metric = n/a');
        end
        set(hBarJac,'XData',[0 wj wj 0]);

        drawnow limitrate
        if ~isempty(vw)
            writeVideo(vw, getframe(fig));
        end
    end

    if ~isempty(vw), close(vw); end
end
