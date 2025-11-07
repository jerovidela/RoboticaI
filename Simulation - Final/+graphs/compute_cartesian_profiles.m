function [pos, vel, acc] = compute_cartesian_profiles(R, q, t)
% compute_cartesian_profiles  Perfiles cartesianos de la punta (TCP)
%
% Resumen:
% - Calcula posicion [X Y Z] del TCP a partir de q(k) via fkine.
% - Estima velocidad y aceleracion por derivadas temporales suavizadas.
%
% Entradas:
%   R: SerialLink del robot
%   q: [N x dof] trayectoria articular (rad)
%   t: [N x 1] tiempos (s)
% Salidas:
%   pos: [N x 3] posicion cartesiana (m)
%   vel: [N x 3] velocidad cartesiana (m/s)
%   acc: [N x 3] aceleracion cartesiana (m/s^2)
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
