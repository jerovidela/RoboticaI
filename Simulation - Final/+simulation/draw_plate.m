function h = draw_plate(center, sizeWH, faceColor)
% Dibuja una placa horizontal en z = center(3), centrada en (x,y) = center(1:2)
    cx=center(1); cy=center(2); cz=center(3);
    W=sizeWH(1); H=sizeWH(2);
    if nargin < 3 || isempty(faceColor)
        faceColor = [0.72 0.75 0.78]; % metallic gray por defecto
    end
    x = [cx-W/2, cx+W/2, cx+W/2, cx-W/2];
    y = [cy-H/2, cy-H/2, cy+H/2, cy+H/2];
    z = cz * ones(1,4);
    h = patch('XData',x,'YData',y,'ZData',z, ...
        'FaceColor',faceColor,'EdgeColor',[0.2 0.2 0.25],'LineWidth',1.0, ...
        'FaceAlpha',0.30, 'FaceLighting','gouraud', ...
        'SpecularStrength',0.6,'SpecularExponent',30);
end
