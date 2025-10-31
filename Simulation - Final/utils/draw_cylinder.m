function draw_cylinder(ax, R, z_min, z_max, cx, cy)
% draw_cylinder  Dibuja un cilindro (superficie) para referencia visual.
if nargin < 5 || isempty(cx), cx = 0; end
if nargin < 6 || isempty(cy), cy = 0; end
[xc,yc,zc] = cylinder(R, 100);
zc = z_min + (z_max - z_min) * zc;
surf(ax, cx+xc, cy+yc, zc, 'FaceAlpha',0.15, 'EdgeColor','none', 'FaceColor',[0.7 0.7 0.7]);
end

