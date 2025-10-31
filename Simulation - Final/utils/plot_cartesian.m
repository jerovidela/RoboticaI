function [pos, vel, acc] = compute_cartesian_profiles(R, q, t)
% compute_cartesian_profiles  Calcula posición, velocidad y aceleración del TCP.
% pos: Nx3; vel/acc por diferencias centrales (tiempo uniforme aproximado).
N = size(q,1);
pos = zeros(N,3);
for i=1:N
    T = R.fkine(q(i,:));
    pos(i,:) = T.t';
end
dt = mean(diff(t));
vel = gradient(pos, dt);
acc = gradient(vel, dt);
end

function plot_xyz_profiles(t, pos, vel, acc, titlePrefix)
% plot_xyz_profiles  Grafica perfiles cartesianos (pos/vel/acc) en 3 figuras.
if nargin < 5, titlePrefix = 'Cartesiano'; end

% Posición
figure('Color','w'); hold on; grid on;
plot(t, pos, 'LineWidth',1.0);
legend({'X','Y','Z'}, 'Location','best'); xlabel('Tiempo [s]'); ylabel('Posición [m]');
title([titlePrefix ' - Posición']);

% Velocidad
figure('Color','w'); hold on; grid on;
plot(t, vel, 'LineWidth',1.0);
legend({'X','Y','Z'}, 'Location','best'); xlabel('Tiempo [s]'); ylabel('Velocidad [m/s]');
title([titlePrefix ' - Velocidad']);

% Aceleración
figure('Color','w'); hold on; grid on;
plot(t, acc, 'LineWidth',1.0);
legend({'X','Y','Z'}, 'Location','best'); xlabel('Tiempo [s]'); ylabel('Aceleración [m/s^2]');
title([titlePrefix ' - Aceleración']);
end

