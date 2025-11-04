% demo_articulated_import.m
close all; clc;
thisFolder = fileparts(mfilename('fullpath'));
robot = importrobot(fullfile(thisFolder, 'articulated_arm_auto.urdf'), ...
    "DataFormat","column","MeshPath", thisFolder);
figure('Name','Auto-segmented Arm');
show(robot); axis equal; view(135,20); camlight; drawnow;
% Animate
config = homeConfiguration(robot);
q = zeros(numel(config),1);
for t = linspace(0,1,160)
    q(1) = (t-0.5)*2*pi/4;
    q(2) = sin(t*2*pi)*pi/6;
    q(3) = cos(t*2*pi)*pi/8;
    q(4) = sin(t*4*pi)*pi/5;
    q(5) = cos(t*4*pi)*pi/6;
    q(6) = sin(t*6*pi)*pi/3;
    show(robot, q, "PreservePlot", false, "Collisions","off","Visuals","on"); drawnow;
end
