function draw_plate(ax, x_min, x_max, y_min, y_max, z0, thickness)
% draw_plate  Dibuja una placa rectangular (bloque delgado) en z=z0.
% ax: axes; x_min/x_max, y_min/y_max: límites; z0: cota superior; thickness: espesor.
if nargin < 7 || isempty(thickness), thickness = 0.006; end
vol = [x_min x_max y_min y_max (z0 - thickness) z0];
axes(ax);
try
    plotvol(vol, 'alpha',0.25, 'facecolor',[0.75 0.75 0.75], 'edgecolor','none');
catch
    [X,Y] = meshgrid([x_min x_max],[y_min y_max]);
    Z = z0*ones(size(X));
    surf(ax, X, Y, Z, 'FaceAlpha',0.25, 'EdgeColor','none', 'FaceColor',[0.75 0.75 0.75]);
end
end

