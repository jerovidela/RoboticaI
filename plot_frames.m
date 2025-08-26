% plot_frames.m
% Utility to plot the robot at a chosen configuration and display selected
% DH frames {0}..{n} (and optionally the tool frame {T}).
% -------------------------------------------------------------------------

%% 2.1 Load the robot definition
run('robot.m');  % brings R, dh, workspace into memory

%% 2.2 Define a joint configuration to analyze (within limits)
% Example: mid-range of each joint
q = mean(R.qlim, 2)';        % 1x6 row vector
% Alternatively:
% q = zeros(1, R.n);

%% 2.3 Define which frames to display (boolean vector)
% Convention:
% - Length == R.n + 1  -> plot {0}, {1}, ..., {n}
% - Length == R.n + 2  -> plot {0}, {1}, ..., {n}, {T} (tool)
% Examples (for 6-DOF):
%   sistemas = [1 1 1 1 1 1 1];       % {0}..{6}
%   sistemas = [0 0 0 0 1 1 1];       % only {4},{5},{6}
%   sistemas = [1 0 0 0 0 0 0 1];     % {0} and {T}
sistemas = [1 1 1 1 1 1 1 1];  % default: show all incl. tool

% Normalize length to either (n+1) or (n+2)
if ~(numel(sistemas) == R.n + 1 || numel(sistemas) == R.n + 2)
    error('Length of "sistemas" must be R.n+1 (%d) or R.n+2 (%d).', R.n+1, R.n+2);
end

%% 2.4 Plot the robot
figure('Name', [R.name ' — Frames Viewer'], 'NumberTitle', 'off');
R.plot(q, 'workspace', workspace, 'scale', 0.8, 'jointdiam', 0.6, 'notiles');
hold on; grid on; axis equal;

%% 2.5 Loop over frames and draw according to "sistemas"
% Visual style parameters for frames
frameLen   = 0.12;   % axis length in meters
lineWidth  = 1.5;    % axis line thickness
labelOpts  = {'fontSize', 10, 'fontweight', 'bold'};

% {0}: base frame (includes R.base)
if sistemas(1)
    trplot(R.base, 'frame', '0', 'length', frameLen, 'thick', lineWidth, labelOpts{:});
end

% {1}..{n}: intermediate DH frames
T = R.base;
for i = 1:R.n
    % RTB provides R.A(i, q): transform from base to frame {i}
    Ti = R.A(i, q);                 % SE3 of frame {i} in world
    if sistemas(i+1)
        trplot(Ti, 'frame', num2str(i), 'length', frameLen, 'thick', lineWidth, labelOpts{:});
    end
    T = Ti; %#ok<NASGU>  % keep variable for clarity; not strictly needed
end

% Optional tool frame {T}: fkine(q) already includes base and tool
if numel(sistemas) == R.n + 2 && sistemas(end)
    Ttool = R.fkine(q);
    trplot(Ttool, 'frame', 'T', 'length', frameLen, 'thick', lineWidth, labelOpts{:});
end

title([R.name ' — q = [' sprintf('%.1f° ', rad2deg(q)) ']']);
hold off;
