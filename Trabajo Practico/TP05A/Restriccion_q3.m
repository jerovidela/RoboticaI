clear; clc; close all;

flag1 = 1;
flag2 = 1;

a = [0 0 0];
p = [0 0];
q = [0 0 0];

while flag1
    fprintf("Ingrese las dimensiones de los vínculos (largo)\n");
    a(1) = input("a1: ");
    a(2) = input("a2: ");
    a(3) = input("a3: ");
    aa = a(1) + a(2) + a(3);

    fprintf("Vector de articulaciones: "); a %#ok<NOPTS>

    while flag2
        fprintf("Ingrese un punto en el plano:\n");
        p(1) = input("Coordenada x: ");
        p(2) = input("Coordenada y: ");

        fprintf("\nIngrese valor de restricción q1:\n");
        q(1) = input("q1 [rad]: ");
        
        % Vector desde el origen hasta el punto pero con complejos (porque
        % me gustan)
        r  = (p*p')^(1/2)*exp(1i*atan2(p(2),p(1)));     
        % Vector posición de la articulación 1 (en complejo)
        C1 = a(1)*exp(1i*q(1));
        % Vector entre el extremo y la articulación 1
        d  = r - C1;                       
        c  = abs(d);
        % dirección de d en el frame de la junta 1
        phi = angle(d) - q(1);              
        phi = wrapToPi(phi);

        fprintf("\nr: %.4f / %.4f ", abs(r), angle(r))
        fprintf("\nC1: %.4f / %.4f ", abs(C1), angle(C1))
        fprintf("\nd: %.4f / %.4f \n", abs(d), angle(d))

        tol = 1e-9;
        A23 = a(2) + a(3);

        if c > (A23 + tol) || c < abs(a(2)-a(3)) - tol
            fprintf("\n|------------------------------------------------------------------------------------------|\n")
            fprintf("\nNo existe solución para esa restricción. \nElija otra terna de valores (x, y, q1).\n")
            fprintf("\n|------------------------------------------------------------------------------------------|\n")
            
             % ====== GRAFICO EXPLICATIVO ======
            figure; clf; ax = gca; hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
            xlabel(ax,'x'); ylabel(ax,'y'); title(ax,'Objetivo inalcanzable');
        
            q1 = q(1);
            z0 = 0;
            z1 = a(1)*exp(1i*q1);
        
            % eslabón 1
            plot(ax, real([z0 z1]), imag([z0 z1]), 'b-', 'LineWidth', 2);
            plot(ax, real([z0 z1]), imag([z0 z1]), 'ro', 'MarkerFaceColor','r');
        
            % objetivo
            plot(ax, p(1), p(2), 'mx', 'MarkerSize', 10, 'LineWidth', 2);
            text(ax, p(1), p(2), '  objetivo', 'Color','m', 'FontSize',9);
        
            % círculos de alcanzabilidad del 2R
            Ain = abs(a(2)-a(3));
            draw_circle(ax, z1, A23, 'k--', 'radio a2+a3');
            draw_circle(ax, z1, Ain, 'r-.',  'radio |a2-a3|');
        
            % Dirección al objetivo en el marco de la junta 1
            phi = angle(d) - q1;
        
            % ===== "Resto del brazo" =====
            if c > (A23 + tol)
                % Demasiado lejos: brazo totalmente extendido hacia el objetivo
                z2_tip = z1 + a(2)*exp(1i*(q1+phi));
                z3_tip = z1 + (a(2)+a(3))*exp(1i*(q1+phi));
                pose_lbl = 'brazo 2R estirado';
            else
                % Demasiado cerca: alcance mínimo |a2-a3|, eslabones en oposición
                if a(2) >= a(3)
                    z2_tip = z1 + a(2)*exp(1i*(q1+phi));
                    z3_tip = z2_tip + a(3)*exp(1i*(q1+phi+pi));
                else
                    z2_tip = z1 + a(2)*exp(1i*(q1+phi+pi));
                    z3_tip = z2_tip + a(3)*exp(1i*(q1+phi));
                end
                pose_lbl = 'brazo 2R a alcance mínimo';
            end
        
            % Dibujar eslabones 2 y 3 en la pose ilustrativa
            plot(ax, real([z1 z2_tip]), imag([z1 z2_tip]), 'c-', 'LineWidth', 1.8);
            plot(ax, real([z2_tip z3_tip]), imag([z2_tip z3_tip]), 'c-', 'LineWidth', 1.8);
            plot(ax, real([z2_tip z3_tip]), imag([z2_tip z3_tip]), 'ko', 'MarkerFaceColor','c', 'MarkerSize',5);
            text(ax, real(z3_tip), imag(z3_tip), ['  ' pose_lbl], 'Color',[0 0.5 0.5], 'FontSize',9);
        
            % ventana
            aa = a(1)+a(2)+a(3);
            xlim(ax, [-aa-1 aa+1]); ylim(ax, [-aa-1 aa+1]);
        
            legend(ax, {'eslabón 1','juntas','objetivo','alcance máx a2+a3','alcance mín |a2-a3|','2R ilustrativo'}, ...
                'Location','bestoutside');

            fprintf("\n(Toca cualquier tecla para continuar)\n\n")
            pause

        elseif abs(c - A23) <= tol
            % Tangencia: q3 = 0 y q2 = phi
            q2 =  phi;
            q3 =  0;
            fprintf("\n|------------------------------|\nExiste solución única.\n|------------------------------|\n")
            fprintf("Para la terna (%.4f, %.4f, %.4f)", p(1), p(2), q(1))
            fprintf("\nLa solución (q2, q3) es: (%.4f, %.4f)", q2, q3)

            figure; clf;
            plot_ej5([q(1) q2 q3], a, gca, struct(), 'showAngles', true);
            % xlim([-aa-1 aa+1]); ylim([-aa-1 aa+1]); 
            grid on
            fprintf("\n(Toca cualquier tecla para continuar)\n\n")
            pause

        else
            % Dos soluciones con 2R clásico
            alpha = acos( clamp( (a(2)^2 + c^2 - a(3)^2)/(2*a(2)*c), -1, 1) );
            theta = acos( clamp( (a(2)^2 + a(3)^2 - c^2)/(2*a(2)*a(3)), -1, 1) );
            % codo-up
            q2u =  phi - alpha;
            q3u =  (pi - theta);
            % codo-down
            q2d =  phi + alpha;
            q3d = -(pi - theta);

            fprintf("\n|------------------------------|\nExisten 2 pares de soluciones.\n|------------------------------|\n")
            fprintf("\nSolución (q2, q3) es: (%.4f, %.4f)", q2u, q3u)
            fprintf("\nSolución (q2, q3) es: (%.4f, %.4f)", q2d, q3d)

            figure; clf;

            % S1
            subplot(1,2,1);
            plot_ej5([q(1) q2u q3u], a, gca, struct(), 'showAngles', true);
            % xlim([-aa-1 aa+1]); ylim([-aa-1 aa+1]); 
            grid on
            title('Solución 1');

            % S2
            subplot(1,2,2);
            plot_ej5([q(1) q2d q3d], a, gca, struct(), 'showAngles', true);
            % xlim([-aa-1 aa+1]); ylim([-aa-1 aa+1]); 
            grid on
            title('Solución 2');

            fprintf("\n(Toca cualquier tecla para continuar)\n\n")
            pause
        end
    end
end

%% ===================== UTILIDADES =====================

% Función para que no me explote el acos()
function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

% sirve para normalizar ángulos y dejarlos siempre dentro del rango [−π,π].
function ang = wrapToPi(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

% Dibujo RRR en el plano con complejos + arcos de q
function h = plot_ej5(q, a, ax, h, varargin)
% h = plot_ej5(q, a, ax, h, 'showAngles', true)

    opts = struct('showAngles', false);
    if ~isempty(varargin)
        for k = 1:2:numel(varargin)
            opts.(varargin{k}) = varargin{k+1};
        end
    end

    if nargin < 3 || isempty(ax), ax = gca; end
    if nargin < 4, h = struct(); end

    q1 = q(1); q2 = q(2); q3 = q(3);
    a1 = a(1); a2 = a(2); a3 = a(3);

    % Posiciones (complejos)
    z0 = 0;
    z1 = a1 * exp(1i*q1);
    z2 = z1 + a2 * exp(1i*(q1+q2));
    z3 = z2 + a3 * exp(1i*(q1+q2+q3));

    hold(ax, 'on'); axis(ax, 'equal');

    if ~isfield(h,'l1') || ~isgraphics(h.l1)
        h.l1 = plot(ax, real([z0 z1]), imag([z0 z1]), 'b-', 'LineWidth', 2);
        h.l2 = plot(ax, real([z1 z2]), imag([z1 z2]), 'b-', 'LineWidth', 2);
        h.l3 = plot(ax, real([z2 z3]), imag([z2 z3]), 'b-', 'LineWidth', 2);
        h.p  = plot(ax, real([z0 z1 z2 z3]), imag([z0 z1 z2 z3]), ...
                    'ro', 'MarkerFaceColor','r', 'MarkerSize',6);
        xlabel(ax,'x'); ylabel(ax,'y'); title(ax,'Planar RRR (complejos)');
    else
        set(h.l1, 'XData', real([z0 z1]), 'YData', imag([z0 z1]));
        set(h.l2, 'XData', real([z1 z2]), 'YData', imag([z1 z2]));
        set(h.l3, 'XData', real([z2 z3]), 'YData', imag([z2 z3]));
        set(h.p,  'XData', real([z0 z1 z2 z3]), 'YData', imag([z0 z1 z2 z3]));
    end

    if opts.showAngles
        % radio de arco como fracción de cada eslabón
        r1 = 0.35*a1 + 0.1*(a2+a3);
        r2 = 0.35*a2;
        r3 = 0.35*a3;

        % q1: desde eje x a eslabón 1
        draw_arc(ax, z0, 0, q1, r1, sprintf('q1=%.3f', q1));

        % q2: relativo al eslabón 1, arco en z1, desde q1 hasta q1+q2
        draw_arc(ax, z1, q1, q2, r2, sprintf('q2=%.3f', q2));

        % q3: relativo al eslabón 2, arco en z2, desde q1+q2 hasta q1+q2+q3
        draw_arc(ax, z2, q1+q2, q3, r3, sprintf('q3=%.3f', q3));
    end
end

function draw_arc(ax, center, startAng, deltaAng, radius, labelStr)
% Dibuja un arco de ángulo 'deltaAng' desde 'startAng' con radio 'radius'
% y rotula en el medio del arco.
    if radius <= 0 || abs(deltaAng) < 1e-6, return; end
    n = max(12, ceil(abs(deltaAng)*30));        % densidad decente
    t0 = startAng;
    t1 = startAng + deltaAng;
    ts = linspace(t0, t1, n);
    xx = real(center) + radius*cos(ts);
    yy = imag(center) + radius*sin(ts);
    plot(ax, xx, yy, 'k-', 'LineWidth', 1);

    % marca de sentido
    mid = (t0 + t1)/2;
    xm = real(center) + (radius+0.06*radius)*cos(mid);
    ym = imag(center) + (radius+0.06*radius)*sin(mid);
    plot(ax, xm, ym, 'k.', 'MarkerSize', 8);

    if nargin >= 6 && ~isempty(labelStr)
        text(ax, xm, ym, ['  ' labelStr], 'FontSize', 9, 'Color', 'k');
    end
end

function draw_circle(ax, center, radius, style, lbl)
    if radius <= 0, return; end
    if nargin < 4 || isempty(style), style = 'k--'; end
    if nargin < 5, lbl = ''; end

    t = linspace(0, 2*pi, 300);
    z = center + radius*exp(1i*t);

    % dibujar y quedarme con el handle para recuperar un color válido
    hline = plot(ax, real(z), imag(z), style, 'LineWidth', 1.25);

    if ~isempty(lbl)
        ang = pi/6;
        col = get(hline, 'Color');  % color RGB válido
        text(ax, real(center)+1.05*radius*cos(ang), ...
                 imag(center)+1.05*radius*sin(ang), lbl, ...
                 'Color', col, 'FontSize', 9, 'Interpreter','none');
    end
end

