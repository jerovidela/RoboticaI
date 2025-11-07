function Q = tf_ik(R, target, q_mejor, q0, opts)
    if nargin < 5, opts = struct(); end
    Tgoal = normalize_target(target);

    usedAnalytic = false;
    if R.n == 6
        try
            % muñeca esférica: a4≈0 y a5≈0
            if abs(R.links(4).a) < 1e-9 && abs(R.links(5).a) < 1e-9
                seeds = local_generateSeeds(R, q0, 20);
                sols = [];
                for i=1:size(seeds,1)
                    qi = seeds(i,:);
                    q  = R.ikine6s(Tgoal, 'q0', qi);
                    if ~isempty(q) && all(isfinite(q))
                        qn = local_normAng(q);
                        if local_isNew(sols, qn, 1e-3), sols = [sols; qn]; end %#ok<AGROW>
                    end
                end
                if ~isempty(sols)
                    usedAnalytic = true;
                    sols = local_orderByProximity(sols, q0);
                    Q = sols(1,:);               % por defecto
                    if ~q_mejor, Q = sols; end   % devolver todas
                end
            end
        catch
            % cae al numérico
        end
    end

    if ~usedAnalytic
        Q = tf_ik_dls(R, Tgoal, q_mejor, q0, opts);
    end
end

% -------- helpers internos --------

function Tgoal = normalize_target(target)
    if isstruct(target)
        if isfield(target,'T'), Tgoal = target.T;
        else
            Tgoal = eye(4);
            if isfield(target,'R'), Tgoal(1:3,1:3) = target.R; end
            if isfield(target,'p'), Tgoal(1:3,4)   = target.p(:); end
        end
    else
        Tgoal = target;
    end
end

function seeds = local_generateSeeds(R, q0, N)
    n = R.n; L = R.qlim; hasLim = ~isempty(L);
    seeds = zeros(N,n);
    for i=1:N
        jitter = randn(1,n)*(pi/12);     % ~±15°
        qi = q0 + jitter;
        if hasLim
            qi = max(min(qi, L(:,2).'), L(:,1).');
            if mod(i,4)==0
                qi = L(:,1).' + rand(1,n).*(L(:,2).'-L(:,1).');
            end
        end
        seeds(i,:) = local_normAng(qi);
    end
end

function qn = local_normAng(q), qn = atan2(sin(q),cos(q)); end

function tf = local_isNew(S, q, tol)
    if isempty(S), tf = true; return; end
    D = sqrt(sum(atan2(sin(S-q),cos(S-q)).^2,2));
    tf = all(D > tol);
end

function S = local_orderByProximity(S, q0)
    D = vecnorm(atan2(sin(S-q0),cos(S-q0)),2,2);
    [~,ix] = sort(D,'ascend'); S = S(ix,:);
end
% TF_IK
% Variante de cinemática inversa basada en el método numérico ikine().
% Características:
%   - Usa iteraciones (método numérico) en lugar de fórmulas analíticas.
%   - Puede depender fuertemente de la postura inicial (q0).
%   - A veces no converge si el objetivo es difícil.
% En pocas palabras:
%   Encuentra UNA solución articular aproximada usando
%   un método iterativo estándar.


