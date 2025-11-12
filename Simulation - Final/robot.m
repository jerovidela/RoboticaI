[R, opt] = robot_G11();

q = zeros(6,1)

R.plot(q, opt{:})

function [R, plotopt, q_home] = robot_G11(vis)
% robot_G11  Crea y visualiza el manipulador del grupo G11 (SerialLink)
% - Define parametros DH, limites, base y herramienta; devuelve SerialLink.
% - Si se pasa 'vis', ajusta opciones de visualizacion y puede plotear (vis.plot=true).
% Entradas: vis (struct opcional: ws, linkcolor, jointcolor, jointlen, jointdiam, plot)
% Salidas:  R (SerialLink), plotopt (cell de opciones para R.plot), q_home (1xN)
%% Definicion de los parametros del robot
    d_tool = 0.0; % Definición explícita del desplazamiento de la herramienta puede cambiar segun la herramienta
    dh = [ ...
        0     0.283     0.000    -pi/2   0;   % Joint 1
        0     0.000     0.398    0       0;   % Joint 2
        0     0.000     0.213    0       0;   % Joint 3
        0     0.000     0.000    pi/2    0;   % Joint 4
        0     0.000     0.000    pi/2    0;   % Joint 5
        0     0.166     0.000    0       0    % Joint 6
        ];  

    name = 'Robot Scan Arm';
    qlim = deg2rad([ ...
       -170  170;    % q1
       -150  80;   % q2
       -110  140;    % q3
       -140  140;    % q4
       -120  120;    % q5
       -360  360]);  % q6

    offset = deg2rad([0 -270 0 0 0 0]);  % el offset de q2 representa el codo en nuestro robot
    base = transl(0,0,0);
    tool = transl(0,0,-d_tool);
    
    %% Definicion del robot
    R = SerialLink(dh);
    R.name = name;
    R.qlim = qlim;
    R.offset = offset;
    R.base = base;
    R.tool = tool;

    % Opciones de estilo por defecto para plot (no dibuja ahora)
    if nargin < 1 || isempty(vis), vis = struct(); end
    if ~isfield(vis,'ws'),         vis.ws         = [-0.5 0.8 -0.5 0.8 0 1]; end
    if ~isfield(vis,'linkcolor'),  vis.linkcolor  = [0.35 0.36 0.38];      end
    if ~isfield(vis,'jointcolor'), vis.jointcolor = [0.95 0.45 0.10];      end
    % Escalas: links mas "finos" y juntas mas notorias
    ws = vis.ws; dims = [ws(2)-ws(1), ws(4)-ws(3), ws(6)-ws(5)];
    baseScale = max(1e-3, min(dims));
    % Juntas mucho más grandes por defecto
    if ~isfield(vis,'jointlen'),   vis.jointlen   = 0.95 * baseScale;      end
    if ~isfield(vis,'jointdiam'),  vis.jointdiam  = 1.15 * baseScale;      end
    if ~isfield(vis,'scale'),      vis.scale      = 0.55;                  end

    % Guardar estilo para aplicar en futuros R.plot(...)
    plotopt = {
        'workspace', vis.ws, ...
        'scale', vis.scale, ...
        'linkcolor', vis.linkcolor, ...
        'jointcolor', vis.jointcolor, ...
        'jointlen', vis.jointlen, ...
        'jointdiam', vis.jointdiam};

    % Pose "home" para ploteo por defecto (diferente a cero)
    % Pose de home mas "estirada" (puede ajustarse con vis.home_deg)
    if isfield(vis,'home_deg') && numel(vis.home_deg) == 6
        q_home_deg = vis.home_deg(:).';
    else
        q_home_deg = [25 -270 45 -90 -90 0];
    end
    q_home = deg2rad(q_home_deg);

    % Plot opcional si se solicita explicitamente
    if isfield(vis,'plot') && vis.plot
        R.plot(q_home, plotopt{:});
        axis(vis.ws); grid on; box on; axis vis3d; view(135,25);
        camlight headlight; lighting gouraud;
        title('Robot Scan Arm');
    end
end
