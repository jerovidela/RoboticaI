function plot_xyz_profiles(t, pos, vel, acc, titlePrefix)
% plot_xyz_profiles  Grafica perfiles cartesianos X,Y,Z de pos/vel/acc
%
% Entradas:
%   t: [N x 1] tiempo (s)
%   pos, vel, acc: [N x 3]
%   titlePrefix: prefijo para titulos (opcional)
% Salidas:
%   (ninguna) crea 3 figuras (pos, vel, acc)
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
