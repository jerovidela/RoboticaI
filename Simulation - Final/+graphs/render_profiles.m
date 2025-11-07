function outfiles = render_profiles(R, q_total, t_total, titlePrefix, opts)
%RENDER_PROFILES  Render cartesiano (3 figs) y articular (3 subplots) y guarda en Sim/Pic.
% Uso (compatible con el original):
%   graphs.render_profiles(R, q_total, t_total)                    % sin guardar explícito (usa defaults)
%   graphs.render_profiles(R, q_total, t_total, 'Placa')
%   files = graphs.render_profiles(R, q_total, t_total, 'Placa', struct('save',true,'formats',{'png','pdf'}))
%
% Entradas:
%   R, q_total, t_total, titlePrefix (opcional)
%   opts (opcional) struct con campos:
%       .save        logical (default: true)    Guardar figuras creadas por esta función
%       .output_dir  char   (default: 'Sim/Pic' relativo a este archivo)
%       .formats     char|cellstr (default: 'png')  p.ej. {'png','pdf','fig'}
%       .dpi         double (default: 300)      Resolución para raster
%       .close       logical (default: false)   Cerrar las figuras luego de guardar
%
% Salida:
%   outfiles  string array con rutas de archivos generados (si .save==true); vacío si no se guardó.

if nargin < 4 || isempty(titlePrefix), titlePrefix = 'Placa'; end
if nargin < 5 || isempty(opts), opts = struct; end

% -------------------- Defaults --------------------
if ~isfield(opts,'save');       opts.save = true;        end
if ~isfield(opts,'formats');    opts.formats = "png";    end
if ~isfield(opts,'dpi');        opts.dpi = 300;          end
if ~isfield(opts,'close');      opts.close = false;      end
% Determinar output_dir sin usar isempty (evita sombras de builtin)
needsDefaultOut = ~isfield(opts,'output_dir');
if ~needsDefaultOut
    od = opts.output_dir;
    if (isstring(od) && all(strlength(od)==0)) || (ischar(od) && size(od,2)==0)
        needsDefaultOut = true;
    end
end
if needsDefaultOut
    here   = fileparts(mfilename('fullpath'));   % .../Sim/+graphs
    simDir = fileparts(here);                    % .../Sim
    opts.output_dir = fullfile(simDir, 'Pic');   % .../Sim/Pic
end
if ischar(opts.formats) || isstring(opts.formats)
    opts.formats = cellstr(opts.formats);
end

outfiles = strings(0,1);
if opts.save && ~exist(opts.output_dir,'dir')
    mkdir(opts.output_dir);
end

% Capturar figuras abiertas antes (para identificar las nuevas)
preFigs = findall(0,'Type','figure');

% ==================== TUS CÁLCULOS Y PLOTS (SIN CAMBIOS) ====================
% 1) Perfiles cartesianos
[pos, vel, acc] = graphs.compute_cartesian_profiles(R, q_total, t_total);
graphs.plot_xyz_profiles(t_total, pos, vel, acc, titlePrefix);

% 2) Perfiles articulares (deg)
graphs.plot_q_qd_qdd(t_total, q_total, [titlePrefix ' - perfiles articulares']);
% ============================================================================

% Identificar solo las figuras creadas por esta llamada
postFigs = findall(0,'Type','figure');
isOld    = ismember(postFigs, preFigs);
newFigs  = postFigs(~isOld);

% Ordenar por Number para nombres consistentes
if ~isempty(newFigs)
    [~,ord] = sort([newFigs.Number]);
    newFigs = newFigs(ord);
end

% Nombrar si no tienen Name, para que los archivos salgan prolijos
for k = 1:numel(newFigs)
    if isempty(get(newFigs(k),'Name'))
        set(newFigs(k),'Name', sprintf('%02d_%s', k, sanitize(titlePrefix)));
    end
end

% Guardado
if opts.save && ~isempty(newFigs)
    outfiles = saveFigures(newFigs, opts.output_dir, opts.formats, opts.dpi, opts.close);
end

end

% -------------------- Helpers locales --------------------
function s = sanitize(txt)
    s = regexprep(char(txt), '[^\w\-]+', '_');
end

function files = saveFigures(figs, outdir, formats, dpi, closeAfter)
    files = strings(0,1);
    for k = 1:numel(figs)
        fig = figs(k);
        base = get(fig,'Name');
        if isempty(base), base = sprintf('fig_%02d', k); end
        base = sanitize(base);
        for f = 1:numel(formats)
            fmt = lower(formats{f});
            file = fullfile(outdir, sprintf('%s.%s', base, fmt));
            try
                switch fmt
                    case 'png'
                        exportgraphics(fig, file, 'Resolution', dpi);
                    case 'pdf'
                        exportgraphics(fig, file, 'ContentType','vector');
                    case 'svg'
                        print(fig, file, '-dsvg');
                    case 'fig'
                        savefig(fig, file);
                    otherwise
                        print(fig, file, ['-d' fmt], sprintf('-r%d', dpi));
                end
            catch
                % Fallback amplio si exportgraphics no está o falla
                switch fmt
                    case {'png','jpg','jpeg','tiff','bmp'}
                        print(fig, file, ['-d' fmt], sprintf('-r%d', dpi));
                    case 'fig'
                        savefig(fig, file);
                    otherwise
                        saveas(fig, file);
                end
            end
            files(end+1,1) = string(file);
        end
        if closeAfter, close(fig); end
    end
end
