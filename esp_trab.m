function esp_trab(opts)
% ESP_TRAB  Workspace plots (2D) for the robot defined in robot.m
% Usage:
%   esp_trab()
% default: with trajectories between extremes
%   esp_trab(struct('traj',false,'nPerJoint',7))
%
% Requires:
%   - robot.m -> must define SerialLink R with fields: R.qlim (joint limits)
%
% Options:
%   opts.traj       (bool) draw jtraj curves between extreme configs (default: true)
%   opts.nPerJoint  (int)  samples per joint for dense cloud (default: 7)
%   opts.showTeach  (bool) open teach pendant at mid config (default: false)

    if nargin == 0, opts = struct; end
    if ~isfield(opts, 'traj'), opts.traj = true; end
    if ~isfield(opts, 'nPerJoint'), opts.nPerJoint = 7; end
    if ~isfield(opts, 'showTeach'), opts.showTeach = false; end

    % 1) Load robot definition ------------------------------------------------
    % robot.m MUST set a SerialLink named R (with correct DH, qlim, base, tool).
    run('robot.m');
    assert(exist('R','var')==1 && isa(R,'SerialLink'), ...
        'robot.m must create a SerialLink variable named R');

    % Sanity checks
    assert(~isempty(R.qlim), 'R.qlim (joint limits) is empty. Set R.qlim in robot.m');
    n = R.n;  qlim = R.qlim;      % [n x 2] (min,max)

    % 2) Build extreme joint configurations (corners of the joint box) -------
    % All 2^n corners (n=7 => 128). This is fine and small.
    corners = corners_of_limits(qlim);
    Pc = fk_points(R, corners);   % [Nc x 3] Cartesian points of corners (TCP)

    % 3) Optional: add per-joint sweeps & a dense cloud for better shape -----
    sweeps = per_joint_sweeps(qlim);                 % one joint at min/max, others mid
    Ps = fk_points(R, sweeps);

    cloud = grid_cloud(qlim, opts.nPerJoint);        % coarse grid cloud
    Pcloud = fk_points(R, cloud);

    % 4) Plot: two representative 2D views (XY and XZ) -----------------------
    figure('Name','Workspace - XY & XZ','Color','w');

    % ---- XY view (top) ----
    subplot(1,2,1); hold on; grid on; axis equal;
    title('Workspace (Top view XY)');
    xlabel('X [m]'); ylabel('Y [m]');
    plot(Pcloud(:,1), Pcloud(:,2), '.');                    % cloud
    plot(Pc(:,1),     Pc(:,2),     'ko','MarkerSize',4,'MarkerFaceColor','k');  % corners
    plot(Ps(:,1),     Ps(:,2),     'rx','MarkerSize',5,'LineWidth',1);          % sweeps

    % Convex hull to outline reachable XY region approximately
    if size(Pcloud,1) >= 3
        try
            Kxy = convhull(Pcloud(:,1), Pcloud(:,2));
            plot(Pcloud(Kxy,1), Pcloud(Kxy,2), '-', 'LineWidth', 1.2);
        catch
        end
    end

    % ---- XZ view (side) ----
    subplot(1,2,2); hold on; grid on; axis equal;
    title('Workspace (Side view XZ)');
    xlabel('X [m]'); ylabel('Z [m]');
    plot(Pcloud(:,1), Pcloud(:,3), '.');
    plot(Pc(:,1),     Pc(:,3),     'ko','MarkerSize',4,'MarkerFaceColor','k');
    plot(Ps(:,1),     Ps(:,3),     'rx','MarkerSize',5,'LineWidth',1);

    if size(Pcloud,1) >= 3
        try
            % convhull in XZ plane
            Kxz = convhull(Pcloud(:,1), Pcloud(:,3));
            plot(Pcloud(Kxz,1), Pcloud(Kxz,3), '-', 'LineWidth', 1.2);
        catch
        end
    end

    % 5) Optional trajectories between corners (joint-space curves) ----------
    if opts.traj
        % connect corners in a loop in joint space, plot TCP traces
        for i = 1:size(corners,1)
            q1 = corners(i,:);
            q2 = corners(mod(i,size(corners,1))+1,:);
            Q  = jtraj(q1, q2, 50);       % 50 steps along the joint-space edge
            P  = fk_points(R, Q);
            subplot(1,2,1); plot(P(:,1), P(:,2), '-', 'LineWidth', 0.75);
            subplot(1,2,2); plot(P(:,1), P(:,3), '-', 'LineWidth', 0.75);
        end
    end
    subplot(1,2,1); hold off;
    subplot(1,2,2); hold off;

    % 6) Report quick metrics for TP (max reach, height, box) ----------------
    allP = [Pcloud; Pc; Ps];
    xmin = min(allP(:,1)); xmax = max(allP(:,1));
    ymin = min(allP(:,2)); ymax = max(allP(:,2));
    zmin = min(allP(:,3)); zmax = max(allP(:,3));
    rxy  = vecnorm(allP(:,1:2),2,2); rmax = max(rxy);

    fprintf('\n=== Workspace quick metrics ===\n');
    fprintf('Max planar reach (XY): %.1f mm\n', 1000*rmax);
    fprintf('X range: [%.1f, %.1f] mm\n', 1000*xmin, 1000*xmax);
    fprintf('Y range: [%.1f, %.1f] mm\n', 1000*ymin, 1000*ymax);
    fprintf('Z range: [%.1f, %.1f] mm\n\n', 1000*zmin, 1000*zmax);

    % 7) Optional: teach pendant at mid config -------------------------------
    if opts.showTeach
        qmid = mean(qlim,2)';    % mid-range config
        R.teach(qmid);           % use to probe poses before plotting
    end
end

% ------------ helpers -------------------------------------------------------
function Q = corners_of_limits(qlim)
    % Returns all 2^n corners of the joint-limit hyper-rectangle
    n = size(qlim,1);
    mins = qlim(:,1)';  maxs = qlim(:,2)';
    M = dec2bin(0:(2^n-1)) - '0';        % [2^n x n] 0/1 mask
    Q = M .* maxs + (1-M) .* mins;       % pick min/max per joint
end

function Q = per_joint_sweeps(qlim)
    % For each joint: two configs where that joint at min/max, others at mid
    n = size(qlim,1);
    qmid = mean(qlim,2)';
    Q = zeros(2*n, n);
    for i=1:n
        q1 = qmid; q1(i) = qlim(i,1);
        q2 = qmid; q2(i) = qlim(i,2);
        Q(2*i-1,:) = q1;
        Q(2*i  ,:) = q2;
    end
end

function Q = grid_cloud(qlim, k)
    % Coarse joint grid around limits for a scatter "cloud"
    % k points per joint (e.g., 7) -> k^n total (can explode).
    % So we build a random Latin-hypercube style sample capped at Nmax.
    n = size(qlim,1);
    Nmax = 4000;                    % cap for performance
    target = min(Nmax, max(200, k^min(n,5))); % keep it reasonable
    % Latin hypercube sample in [0,1], map to limits:
    u = lhsdesign(target, n);
    Q = qlim(:,1)' + u .* (qlim(:,2)' - qlim(:,1)');
end

function P = fk_points(R, Q)
    % FK for many configs -> Nx3 (TCP position)
    if size(Q,1)==1, Q = Q(:)'; end
    N = size(Q,1);  P = zeros(N,3);
    for i=1:N
        T = R.fkine(Q(i,:));
        P(i,:) = transl(T);
    end
end
