function [pos, vel, acc] = compute_cartesian_profiles(R, q, t)
N = size(q,1);
pos = zeros(N,3);
for i=1:N
    Ti = R.fkine(q(i,:));      % SE3
    pos(i,:) = transl(Ti);     % 1x3
end
% Derivadas robustas (ajusta win_sec si querés más/menos suavizado)
vel = graphs.compute_derivatives(t, pos, 1, 0.35, 3);
acc = graphs.compute_derivatives(t, vel, 2, 0.35, 3);
end
