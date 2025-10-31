function plot_q_only(t, q, suptitleText)
% plot_q_only  Grafica q(t) en grados en una sola figura.
if nargin < 3, suptitleText = 'q(t) [deg]'; end
figure('Color','w'); hold on; grid on;
plot(t, rad2deg(q), 'LineWidth', 1.1);
legend(arrayfun(@(k) sprintf('q%d',k), 1:size(q,2), 'UniformOutput', false), 'Location','best');
xlabel('Tiempo [s]'); ylabel('Grados'); title(suptitleText);
end

