function plot_q_qd_qdd(t, q, suptitleText)
% plot_q_qd_qdd  Grafica perfiles q, qd, qdd (en grados)
%
% Entradas:
%   t: [N x 1] tiempo (s)
%   q: [N x dof] posiciones articulares (rad)
%   suptitleText: texto de titulo superior (opcional)
% Salidas:
%   (ninguna) crea una figura con 3 subplots
if nargin < 3, suptitleText = 'Perfiles articulares'; end
qd  = graphs.compute_derivatives(t, q);
qdd = graphs.compute_derivatives(t, qd);

figure('Color','w');
subplot(3,1,1); hold on; grid on; plot(t, rad2deg(q),  'LineWidth',1.0);
title('Posición [deg]');   ylabel('deg');

subplot(3,1,2); hold on; grid on; plot(t, rad2deg(qd), 'LineWidth',1.0);
title('Velocidad [deg/s]'); ylabel('deg/s');

subplot(3,1,3); hold on; grid on; plot(t, rad2deg(qdd),'LineWidth',1.0);
title('Aceleración [deg/s^2]'); ylabel('deg/s^2'); xlabel('Tiempo [s]');

sgtitle(suptitleText);
end
