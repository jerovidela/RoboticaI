
clc, clear, close all
%=========================================================================%
fprintf('Graficador Robot TP Robotica')

%=========================================================================%
dh =  [ ...
    0       0.283   0.000   -pi/2   0;   % Joint 1
    0       0.000   0.398    0      0;   % Joint 2
    0       0.000   0.213    0      0;   % Joint 3
    0       0.000   0.000    pi/2   0;   % Joint 4
    0       0.025   0.000    pi/2   0;   % Joint 5
    0       0.166   0.000    0      0];  % Joint 6

R = SerialLink(dh,'name','Robot');
q = [0,0,0,0,0,0];

qlim = deg2rad([ ...
   -170  170;    % q1
   -150   80;    % q2
   -110  140;    % q3
   -140  140;    % q4
   -120  120;    % q5
   -360  360]);  % q6
offset = deg2rad([0; 0; 0; 0; 90; 0]);
R.qlim = qlim;

%R.base = transl(-0.5,0,0); % * trotx(pi);
%R.tool = transl(.2, .2, 0);
R.offset = [0 0 0 0 pi/2 0];

% R.plot(q,'workspace',[-1 1 -1 1 -1 1])
R.plot(q,'scale',0.8,'trail',{'r', 'LineWidth', 2})
R.teach()
