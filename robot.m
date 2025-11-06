%% Robot corregido con limites articulares
% Revisado 06/11

%% Definicion de los parametros del robot
dh =  [ ...
    0       0.283   0.000   -pi/2   0;   % Joint 1
    0       0.000   0.398    0      0;   % Joint 2
    0       0.000   0.213    0      0;   % Joint 3
    0       0.000   0.000    pi/2   0;   % Joint 4
    0       0.025   0.000    pi/2   0;   % Joint 5
    0       0.166   0.000    0      0];  % Joint 6
name = 'Robot Scan Arm';
qlim = deg2rad([ ...
   -170  170;    % q1
   -150   80;    % q2
   -110  140;    % q3
   -140  140;    % q4
   -120  120;    % q5
   -360  360]);  % q6
offset = deg2rad([0; 0; 0; 0; 90; 0]);
R.offset = offset;
base = transl(0,0,0);
tool = transl(0,0,0);

%% Definicion del robot
R = SerialLink(dh);
R.name = name;
R.qlim = qlim;
R.offset = offset;
R.base = base;
R.tool = tool;
