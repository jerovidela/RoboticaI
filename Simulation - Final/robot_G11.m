function R = robot_G11(vis)
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
       3.667  240;   % q2
       -170  170;    % q3
       -180  180;    % q4
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
    q = zeros(1, R.n);
    q = [0 90 170 90 30 70 ];
    %% Opciones visuales (parametricas)
    if nargin < 1 || isempty(vis), vis = struct(); end
    if ~isfield(vis,'ws'),         vis.ws         = [-0.5 0.8 -0.5 0.8 0 1]; end
    if ~isfield(vis,'linkcolor'),  vis.linkcolor  = [0.58 0.60 0.63];        end
    if ~isfield(vis,'jointcolor'), vis.jointcolor = [0.20 0.55 1.00];        end
    if ~isfield(vis,'jointlen'),   vis.jointlen   = 0.08;                    end
    if ~isfield(vis,'jointdiam'),  vis.jointdiam  = 0.06;                    end
    %% Ploteo Estético (Opción 1: plot)
    figure(1);
    R.plot(q, ...
        'workspace', [-1.5 1 -1.5 1 0 1], ... % Define el tamaño de la caja
        'scale', 1, ...                  % Escala del gráfico
        'linkcolor', [0.4 0.4 0.4], ...   % Eslabones grises
        'jointcolor', [0.90 0.35 0.10], ... % Juntas naranjas
        'notiles', ...                     % Sin piso
        'jointlen', 0.05, ...                % Largo cosmético del eje (0.05m)
        'jointdiam', 0.05);                % Diámetro cosmético del eje (0.05m)
    
    % Reconfigurar visual segun parametros 'vis'
    R.plot(q, 'workspace', vis.ws, 'scale', 1, 'linkcolor', vis.linkcolor, ...
        'jointcolor', vis.jointcolor, 'notiles', 'jointlen', vis.jointlen, 'jointdiam', vis.jointdiam);
    axis(vis.ws); grid on; box on; axis vis3d; view(135,25);
    camlight headlight; lighting gouraud;

    title('Robot Scan Arm');
end

function h = plot_pretty_G11(R, Q, diam, workspace, opts)
% plot_pretty_G11  Dibuja el SerialLink con “tambores” cilíndricos por junta
% y un workspace visible. Si Q es Nx6, anima el movimiento.
%
% R: SerialLink (RTB Corke)
% Q: [N x R.n] posiciones articulares (rad)
% diam: diámetro de tambores (m)  [por defecto 0.06]
% workspace: [xmin xmax ymin ymax zmin zmax]  (ej. [-1.5 1 -1.5 1 0 1])
% opts: struct opcional con campos:
%   .octant = true  % dibuja cubo translúcido del primer octante dentro del workspace
%   .plate  = []    % [x0 x1 y0 y1 z] dibuja una placa rectangular opcional
%   .delay  = 0.02  % delay de animación (s)

    if nargin < 3 || isempty(diam),      diam = 0.06;          end
    if nargin < 4 || isempty(workspace), workspace = [-1 1 -1 1 0 1]; end
    if nargin < 5, opts = struct; end
    if ~isfield(opts,'octant'), opts.octant = true; end
    if ~isfield(opts,'delay'),  opts.delay  = 0.02; end

    % --- Figura y ejes
    h.fig = figure('Color','w'); hold on; grid on; box on;
    axis(workspace); axis vis3d; view(135,25);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

    % --- Workspace: caja del primer octante (opcional) y wireframe general
    draw_workspace_wireframe(workspace, 0.25); % líneas finas
    if opts.octant
        w = workspace;
        draw_octant_box([max(0,w(1)) w(2) max(0,w(3)) w(4) max(0,w(5)) w(6)], 0.06);
    end
    if isfield(opts,'plate') && ~isempty(opts.plate)
        draw_plate(opts.plate); % [x0 x1 y0 y1 z]
    end
    camlight headlight; lighting gouraud;

    % --- Robot inicial
    q1 = Q(1,:); 
    R.plot(q1, 'workspace', workspace, 'noname', 'jointaxis', 'off', ...
               'shadow', 'off', 'delay', 0, 'ortho', 'on');

    % --- Construcción de tambores por junta (cilindros a lo largo de +z_i)
    n  = R.n;
    r  = diam/2;
    L  = diam;                    % largo = diámetro (tu pedido)
    [CX,CY,CZ] = cylinder(r, 32); % malla base (unidad en z)
    CZ = (CZ - 0.5) * L;          % centered at z=0 with length L
    % Parametric adjustment of drum length (cylinders)
    L_new = L;
    if exist('opts','var') && isstruct(opts)
        if isfield(opts,'drum_length') && ~isempty(opts.drum_length)
            L_new = opts.drum_length;
        else
            L_new = 1.5*diam;  % default 1.5x diameter
        end
        if L > 0
            CZ = CZ * (L_new / L);
            L  = L_new;
        end
    end

    h.tg = gobjects(n,1);
    h.surf = gobjects(n,1);
    ax = gca;
    for i = 1:n
        h.tg(i) = hgtransform('Parent', ax);
        h.surf(i) = surf(CX, CY, CZ, 'Parent', h.tg(i), ...
            'EdgeColor','none', 'FaceAlpha',0.35);
    end
    if exist('opts','var') && isstruct(opts) && isfield(opts,'material') && ~isempty(opts.material)
        try, material(opts.material); catch, material dull; end
    else
        material metal;
    end

    % --- Posicionar tambores para q1
    Ti = link_transforms(R, q1);           % {Ti} de base a frame i
    for i = 1:n
        set(h.tg(i), 'Matrix', Ti{i});
    end
    drawnow;

    % --- Animación (si Q tiene múltiples filas)
    if size(Q,1) > 1
        for k = 1:size(Q,1)
            R.plot(Q(k,:), 'delay', 0);    % dibuja la pose k
            Ti = link_transforms(R, Q(k,:));
            for i = 1:n, set(h.tg(i), 'Matrix', Ti{i}); end
            drawnow; pause(opts.delay);
        end
    end
end

function Ti = link_transforms(R, q)
% Devuelve cell Ti{1..n} con transformadas 4x4 de base a cada frame i
    Ti = cell(R.n,1);
    T  = eye(4);
    for i = 1:R.n
        Ai = R.links(i).A(q(i));  % DH local i-1 -> i
        T  = T * Ai;
        Ti{i} = T;
    end
end

function draw_workspace_wireframe(w, a)
% Dibuja un wireframe de la caja del workspace
    if nargin<2, a = 0.3; end
    [x0,x1,y0,y1,z0,z1] = deal(w(1),w(2),w(3),w(4),w(5),w(6));
    X = [x0 x1 x1 x0 x0 x1 x1 x0];
    Y = [y0 y0 y1 y1 y0 y0 y1 y1];
    Z = [z0 z0 z0 z0 z1 z1 z1 z1];
    edges = [1 2;2 3;3 4;4 1; 5 6;6 7;7 8;8 5; 1 5;2 6;3 7;4 8];
    for e = edges.'
        plot3(X(e), Y(e), Z(e), 'Color',[0 0 0]+0.2, 'LineWidth',0.5, 'LineStyle','-');
    end
end

function draw_octant_box(w, alphaFace)
% Caja translúcida del primer octante delimitada por w
    [x0,x1,y0,y1,z0,z1] = deal(w(1),w(2),w(3),w(4),w(5),w(6));
    if x1<=x0 || y1<=y0 || z1<=z0, return; end
    verts = [x0 y0 z0; x1 y0 z0; x1 y1 z0; x0 y1 z0; ...
             x0 y0 z1; x1 y0 z1; x1 y1 z1; x0 y1 z1];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices',verts,'Faces',faces,'FaceAlpha',alphaFace, ...
          'EdgeAlpha',0.15,'FaceColor',[0.6 0.6 0.6]);
end

function draw_plate(p)
% Dibuja una placa rectangular en z fija: p = [x0 x1 y0 y1 z]
    x0=p(1); x1=p(2); y0=p(3); y1=p(4); z=p(5);
    verts = [x0 y0 z; x1 y0 z; x1 y1 z; x0 y1 z];
    faces = [1 2 3 4];
    patch('Vertices',verts,'Faces',faces,'FaceAlpha',0.15, ...
          'EdgeAlpha',0.25,'FaceColor',[0.3 0.5 0.9]);
end
