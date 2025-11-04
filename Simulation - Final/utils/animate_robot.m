function fig = animate_robot(R, q_path, t, axTitle, varargin)
% animate_robot  Crea una figura con la animación del robot para una
% trayectoria articular q_path (N x 6). Si se provee t (N x 1) se respeta
% el espaciado temporal aproximado.
% Opcionales (Name,Value):
%   - 'stride' (int, default 2): salta frames para reducir pasos de animación
%   - 'speedup' (double, default 4): comprime la escala temporal física
%   - 'targetSeconds' (double, opcional): duración objetivo de la animación;
%       si se provee, se ignora 'speedup' y se usa t(end)/targetSeconds
%   - 'ax' (axes handle, opcional): si se provee, dibuja en ese axes sin
%       crear una figura nueva
%   - 'showTrail' (logical, default true): dibuja la trayectoria TCP en tiempo real
%   - 'trailColor' (1x3 double, default [0 0 0]): color de la curva TCP
%   - 'fps' (double, default 30): fps objetivo cuando se usa targetSeconds

p = inputParser; p.KeepUnmatched = true;
addParameter(p,'stride',2,@(x)isnumeric(x)&&isscalar(x)&&x>=1);
addParameter(p,'speedup',4,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'targetSeconds',[],@(x) isempty(x) || (isnumeric(x)&&isscalar(x)&&x>0));
addParameter(p,'ax',[],@(x) isempty(x) || isgraphics(x,'axes'));
addParameter(p,'showTrail',true,@(x)islogical(x)||ismember(x,[0 1]));
addParameter(p,'trailColor',[0 0 0],@(x)isnumeric(x)&&numel(x)==3);
addParameter(p,'fps',30,@(x)isnumeric(x)&&isscalar(x)&&x>0);
parse(p,varargin{:});
stride = round(p.Results.stride); speedup = p.Results.speedup;
targetSeconds = p.Results.targetSeconds;
ax = p.Results.ax;
showTrail = p.Results.showTrail; trailColor = p.Results.trailColor;
fps = p.Results.fps;

if nargin < 4, axTitle = 'Animación'; end
if nargin < 3 || isempty(t)
    t = (0:size(q_path,1)-1)';
    dt = 0.02 * ones(size(t));
else
    dt = [diff(t); mean(diff(t))];
end

% Ajustar compresión temporal según targetSeconds
totalT = t(end) - t(1);
idxStride = 1:stride:size(q_path,1);
if ~isempty(targetSeconds) && targetSeconds > 0
    nTarget = min(numel(idxStride), round(fps * targetSeconds));
    frameIdx = round(linspace(1, numel(idxStride), nTarget));
    idx = idxStride(frameIdx);
    perFrame = 1/fps;
else
    idx = idxStride;
    perFrame = mean(dt)/max(speedup,1);
end

if isempty(ax)
    fig = figure('Name', axTitle, 'Color','w');
    try, set(fig,'WindowState','maximized'); catch, set(fig,'Units','normalized','OuterPosition',[0 0 1 1]); end
    ax = axes('Parent',fig);
    grid(ax,'on'); axis(ax,'equal'); view(ax,135,25);
    xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z [m]'); title(ax, axTitle);
else
    fig = ancestor(ax,'figure');
end
hold(ax,'on'); axes(ax);

% Hacer juntas "monedas": más cortas y diámetro consistente
R.plot(q_path(1,:), 'noname','nojaxes','noshadow','scale',0.8, ...
    'jointlen', 0.05, 'jointdiam', 0.7);

% Trail TCP
if showTrail
    % calcular posición TCP de forma incremental (FK)
    tcpLine = animatedline(ax, 'Color', trailColor, 'LineWidth', 1.5);
end
for ii = 1:numel(idx)
    i = idx(ii);
    R.animate(q_path(i,:));
    if mod(ii,2)==0 || ii==numel(idx)
        drawnow limitrate nocallbacks;
    end
    % agregar punto TCP al trail
    if showTrail
        Tfk = R.fkine(q_path(i,:));
        addpoints(tcpLine, Tfk.t(1), Tfk.t(2), Tfk.t(3));
    end
    pause(max(perFrame, 0.001));
end

end
