
clc, clear, close all
%=========================================================================%
fprintf('Graficador Robot TP Robotica')

%=========================================================================%
dh = [ ...
    0       0.283   0.000   -pi/2   0;   % Joint 1
    0       0.000   0.398    0      0;   % Joint 2
    0       0.000   0.213    0      0;   % Joint 3
    0       0.000   0.000    pi/2   0;   % Joint 4
    0       0.000   0.000    pi/2   0;   % Joint 5
    0       0.166   0.000    0      0];  % Joint 6

R = SerialLink(dh,'name','Robot');
q = [0,0,0,0,0,0];

R.qlim(1,1:2) = [-180,  180]*pi/180;
R.qlim(2,1:2) = [-180,  180]*pi/180;
R.qlim(3,1:2) = [-180,  180]*pi/180;
R.qlim(4,1:2) = [-180,  180]*pi/180;
R.qlim(5,1:2) = [-180,  180]*pi/180;
R.qlim(6,1:2) = [-180,  180]*pi/180;

%R.base = transl(-0.5,0,0); % * trotx(pi);
%R.tool = transl(.2, .2, 0);
R.offset = [0 0 0 0 pi/2 0];

% R.plot(q,'workspace',[-1 1 -1 1 -1 1])
R.plot(q,'scale',0.8,'trail',{'r', 'LineWidth', 2})
R.teach()
