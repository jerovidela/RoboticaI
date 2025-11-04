function R = robot_G11()
% robot_G11
% Modelo DH (estándar) del FARO Quantum Max ScanArm (6 GDL) para TP8.
% Unidades: metros. Todos los ejes rotacionales (sigma=0).
% Nota: offsets mecánicos no definidos -> se toman 0 (columna 6x1).

% DH estándar: [theta, d, a, alpha, sigma]
dh = [ ...
    0, 0.283, 0.000, -pi/2, 0;   % J1
    0, 0.000, 0.398,  0,     0;   % J2
    0, 0.000, 0.213,  0,     0;   % J3
    0, 0.000, 0.000,  pi/2,  0;   % J4
    0, 0.025, 0.213,  pi/2,  0;   % J5
    0, 0.166, 0.000,  0,     0];  % J6

% Crear robot
R = SerialLink(dh, 'name', 'FARO Quantum Scan Arm G11');

% Límites articulares: [-180°, +180°]
R.qlim = deg2rad(repmat([-180 180], 6, 1));

% Offsets mecánicos: 0 (como columna 6x1 para evitar inconsistencias)
R.offset = zeros(6,1);

% Marcos base y herramienta (pueden ajustarse más adelante)
R.base = transl(0,0,0);
R.tool = transl(0,0,0);
end

