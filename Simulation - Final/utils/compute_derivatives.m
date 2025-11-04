function [qd, qdd] = compute_derivatives(t, q)
% compute_derivatives  Deriva q(t) por diferencias centrales (tiempo uniforme aprox.)
dt = mean(diff(t));
qd  = gradient(q, dt);
qdd = gradient(qd, dt);
end

