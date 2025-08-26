clear; clc; close all;
%% 1.2 DH parameters matrix: [theta  d   a   alpha  sigma]
% Units: meters and radians. sigma = 0 (revolute), 1 (prismatic).
% Replace these generic values with your TF's definitive parameters.
dh = [ ...
    0       0.350   0.050    -pi/2   0;   % Joint 1
    0       0.000   0.300     0      0;   % Joint 2
    0       0.000   0.250    -pi/2   0;   % Joint 3
    0       0.300   0.000     pi/2   0;   % Joint 4
    0       0.000   0.000    -pi/2   0;   % Joint 5
    0       0.120   0.000     0      0];  % Joint 6 (flange)

%% 1.3 Create SerialLink object R and set properties
% Name
name = 'Robot Scan Arm';

% Joint limits (rad) — generic symmetric limits you can adjust
qlim = deg2rad([ ...
   -170  170;    % q1
   -120  120;    % q2
   -170  170;    % q3
   -180  180;    % q4
   -120  120;    % q5
   -360  360]);  % q6

% Offsets (rad) — if your zero marks are not at DH zero, set here
offset = deg2rad([0 0 0 0 0 0]);  % row or column vector is fine

% Base transform (SE(3)) — set non-identity to test robustness
% Ex
