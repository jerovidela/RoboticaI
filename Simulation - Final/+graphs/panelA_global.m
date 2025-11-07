function stats = panelA_global(R, Q, t, scan_log, vref, opts)
% panelA_global  Panel de chequeo global de velocidad y error lateral
%
% Resumen:
% - Calcula |v_tip|(t) en toda la simulacion.
% - Sobre segmentos de scan: proyecta velocidad y mide desviacion lateral.
% - Dibuja tres subplots y sombrea periodos de escaneo.
%
% Entradas:
%   R: SerialLink, Q: [K x dof], t: [K x 1]
%   scan_log: struct con campos por pasada (k0,k1,T0,T1,...)
%   vref: velocidad de referencia (opcional)
%   opts: struct de guardado/formatos/directorio (opcional)
% Salidas:
%   stats: struct con estadisticas por pasada (cv, mean, std)
% Gráfico global: |v_tip|(t) (todo), v_proj(t) y desviación lateral (solo scans)
% stats: CV de v_proj por fila

    if nargin < 5, vref = []; end
    if nargin < 6 || isempty(opts), opts = struct(); end
    if ~isfield(opts,'save'),       opts.save = false; end
    if ~isfield(opts,'formats'),    opts.formats = {'png'}; end
    if ~isfield(opts,'output_dir') || isempty(opts.output_dir)
        here   = fileparts(mfilename('fullpath'));
        simDir = fileparts(here);
        opts.output_dir = fullfile(simDir,'Pic');
    end

    K = size(Q,1);
    % Posición de la punta
    P = zeros(K,3);
    for k=1:K
        Tk = R.fkine(Q(k,:));
        P(k,:) = transl(Tk);
    end
    % Velocidad vectorial por diferencias
    % V = [zeros(1,3); diff(P,1,1) ./ max(diff(t), eps)];
    V = graphs.compute_derivatives(t, P, 1, 0.35, 3);
    vmag = sqrt(sum(V.^2,2));

    % Buffers segmentados (NaN fuera de scans)
    vproj = nan(K,1);
    elat  = nan(K,1);
    stats = struct('cv',[],'mean',[],'std',[]);

    for i=1:numel(scan_log)
        s  = scan_log(i);
        idx = s.k0:s.k1;

        % extremos como FILAS 1x3, no columnas
        p0 = transl(s.T0);  p0 = p0(:).';      % 1x3
        p1 = transl(s.T1);  p1 = p1(:).';      % 1x3

        % dirección de la recta
        d = p1 - p0;  L = norm(d);
        if L < eps, d = [1 0 0]; else, d = d / L; end   % 1x3

        % velocidad proyectada sobre la recta
        vproj(idx) = V(idx,:) * d(:);          % (Nx3)*(3x1) -> Nx1

        % desviación lateral: distancia punto-recta
        E    = P(idx,:) - p0;                  % Nx3  (resta fila a todas las filas)
        proj = (E * d(:)) * d;                 % (Nx1)*(1x3) -> Nx3  (outer product)
        elat(idx) = vecnorm(E - proj, 2, 2);   % Nx1

    end

    % Plot
    figure('Color','w'); tl = tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

    nexttile; plot(t, vmag, 'LineWidth',1.2); grid on; ylabel('|v_{tip}| [m/s]');
    if nargin>=5 && ~isempty(vref), yline(vref,'--','Color',[0 0 0]+0.5); end
    title('Velocidad de la punta (toda la simulación)');

    nexttile; plot(t, vproj, 'LineWidth',1.2); grid on; ylabel('v_{proj} [m/s]');
    if nargin>=5 && ~isempty(vref), yline(vref,'--','Color',[0 0 0]+0.5); end
    title('Velocidad proyectada sobre cada recta de scan');

    nexttile; plot(t, elat*1e3, 'LineWidth',1.2); grid on; ylabel('Desv. lateral [mm]'); xlabel('t [s]');
    title('Desviación lateral (NaN fuera de scans)');

    % Bandas para scans
    ax = findall(gcf,'Type','axes');
    for a = ax.'
        hold(a,'on');
        for i=1:numel(scan_log)
            s = scan_log(i);
            patch(a, t([s.k0 s.k1 s.k1 s.k0]), a.YLim([1 1 2 2]), ...
                  [0.85 0.92 1], 'EdgeColor','none', 'FaceAlpha',0.25);
        end
        uistack(findobj(a,'Type','line'), 'top');
    end

    % Resumen rápido en consola
    if ~isempty(stats)
        cv_all = [stats.cv];
        fprintf('CV v_proj por fila (%%): '); fprintf('%.2f ', 100*cv_all); fprintf('\n');
    end

    % Guardado opcional
    if exist('opts','var') && isstruct(opts) && isfield(opts,'save') && opts.save
        if ~isfield(opts,'formats') || isempty(opts.formats), opts.formats = {'png'}; end
        if ~isfield(opts,'output_dir') || isempty(opts.output_dir)
            here   = fileparts(mfilename('fullpath'));
            simDir = fileparts(here);
            opts.output_dir = fullfile(simDir,'Pic');
        end
        if ~exist(opts.output_dir,'dir'), mkdir(opts.output_dir); end
        base = fullfile(opts.output_dir, 'panelA_global');
        fig = gcf;
        for f = 1:numel(opts.formats)
            fmt = lower(opts.formats{f});
            file = sprintf('%s.%s', base, fmt);
            try
                switch fmt
                    case 'png'
                        exportgraphics(fig, file, 'Resolution', 300);
                    case 'pdf'
                        exportgraphics(fig, file, 'ContentType','vector');
                    case 'svg'
                        print(fig, file, '-dsvg');
                    case 'fig'
                        savefig(fig, file);
                    otherwise
                        saveas(fig, file);
                end
            catch
                saveas(fig, file);
            end
        end
    end
end
