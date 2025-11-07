function Q = tf_ik_dls(R, target, q_mejor, q0, opts)
    if nargin < 5, opts = struct(); end
    nSeeds = getf(opts,'nSeeds',24);
    itmax  = getf(opts,'itmax',300);
    tol    = getf(opts,'tol',1e-5);
    lambda = getf(opts,'lambda',1e-2);
    alpha  = getf(opts,'alpha',0.25);
    mask   = getf(opts,'mask',[1 1 1 1 1 1]);

    Tgoal = mat4(target);
    n = R.n; assert(numel(q0)==n, 'q0 debe ser 1x%d', n);

    seeds = generateSeeds(R, q0, nSeeds);   % incluye q0 como primera
    sols = []; costs = [];

    W = diag(mask(:));                       % pesos 6x6

    for s = 1:size(seeds,1)
        q = seeds(s,:);
        e = Inf;

        for k = 1:itmax
            T = mat4(R.fkine(q));
            twist = tr2delta(T, Tgoal);      % 6x1
            e = W * twist;                   % ponderado

            if norm(e) < tol, break; end

            J  = R.jacob0(q);                % 6xn
            Jm = W * J;                      % ponderado

            H  = (Jm.'*Jm) + (lambda^2)*eye(n);
            dq = H \ (Jm.'*e);

            % Resolución de redundancia hacia q0 (espacio nulo)
            Jpinv = H \ (Jm.');
            N = eye(n) - Jpinv*Jm;
            dq = dq + alpha * (N * (q0(:) - q(:)));

            q = q + dq(:).';
            q = clampToLimits(R, q);
        end

        if norm(e) < tol
            qn = normalizeAngles(q);
            [isnew, idxSame] = isNew(sols, qn, 1e-3);
            c = norm(atan2(sin(qn-q0), cos(qn-q0)));
            if isnew
                sols  = [sols;  qn]; %#ok<AGROW>
                costs = [costs; c];  %#ok<AGROW>
            else
                if c < costs(idxSame), sols(idxSame,:) = qn; costs(idxSame)=c; end
            end
        end
    end

    if isempty(sols)
        error('TF_IK_DLS: no se hallaron soluciones (revisá alcanzabilidad/mask/semillas).');
    end

    if q_mejor
        [~,i] = min(costs); Q = sols(i,:);
    else
        Q = orderByProximity(sols, q0);
    end
end

%% ===== Helpers =====
function v = getf(s,f,def), if isfield(s,f), v = s.(f); else, v = def; end, end

function M = mat4(X)
    if isa(X,'SE3'), M = X.T;
    elseif isstruct(X) && isfield(X,'T'), M = X.T;
    else, M = X;
    end
end

function seeds = generateSeeds(R,q0,N)
    n = R.n; L = R.qlim; hasLim = ~isempty(L);
    seeds = zeros(N,n);
    seeds(1,:) = normalizeAngles(q0);   % primera = q0
    for i=2:N
        jitter = randn(1,n)*(pi/12);    % ~±15°
        qi = q0 + jitter;
        if hasLim
            qi = max(min(qi, L(:,2).'), L(:,1).');
            if mod(i,3)==0                               % algunas uniformes
                qi = L(:,1).' + rand(1,n).*(L(:,2).'-L(:,1).');
            end
        end
        seeds(i,:) = normalizeAngles(qi);
    end
end

function q = clampToLimits(R,q)
    if isempty(R.qlim), return; end
    q = max(min(q, R.qlim(:,2).'), R.qlim(:,1).');
end

function qn = normalizeAngles(q), qn = atan2(sin(q),cos(q)); end

function [isnew,idx] = isNew(S,q,tol)
    if isempty(S), isnew=true; idx=0; return; end
    D = sqrt(sum(atan2(sin(S-q),cos(S-q)).^2,2));
    [m,idx] = min(D); isnew = (m>tol);
end

function S = orderByProximity(S,q0)
    D = vecnorm(atan2(sin(S-q0),cos(S-q0)),2,2);
    [~,ix]=sort(D,'ascend'); S=S(ix,:);
end
% TF_IK_DLS
% Variante de cinemática inversa numérica usando
% "Mínimos Cuadrados Amortiguados" (Damped Least Squares).
% Ventajas:
%   - Más estable que ikine simple cerca de singularidades.
%   - Tolera objetivos difíciles de alcanzar.
%   - Puede explorar varias semillas para hallar soluciones.
% En pocas palabras:
%   Método numérico robusto que encuentra configuraciones articulares
%   con menor riesgo de fallar por singularidades.

