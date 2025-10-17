%% Definicion de los parametros del robot
d_tool = 0.0; % Definición explícita del desplazamiento de la herramienta puede cambiar segun la herramienta
dh = [ ...
    0       0.283   0.000    -pi/2   0;   % Joint 1
    0       0.000   0.398    0      0;   % Joint 2
    0       0.000   0.213    0      0;   % Joint 3
    0       0.000   0.000    pi/2   0;   % Joint 4
    0       0.000   0.000    pi/2      0;   % Joint 5
    0       0.166   0.000    0      0];  % Joint 6
name = 'Robot Scan Arm';
qlim = deg2rad([ ...
   -170  170;    % q1
   3.667  240;    % q2
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
