clear; clc; close all;

flag1 = 1;
flag2 = 1;

a = [0 0 0];
p = zeros(3, 3);
p(3, :) = 1;
gamma = 0;
z1 = 0;


q = zeros(3, 2);


fprintf("Ingrese las dimensiones de los vínculos (largo)\n");
a(1) = input("a1: ");
a(2) = input("a2: ");
a(3) = input("a3: ");

% Constantes

A13 = a(1) + a(2) + a(3);
A23 = a(2) + a(3);
tol = 1e-3;

fprintf("Vector de articulaciones: "); a %#ok<NOPTS>

while flag1
    fprintf("Ingrese un punto en el plano:\n");
    p(1, 3) = input("Coordenada x: ");
    p(2, 3) = input("Coordenada y: ");
    gamma = input("Ángulo de orientación [rad]: ");
    gamma = wrapToPi(gamma);

    % Vector desde el origen hasta el punto pero con complejos (porque
    % me gustan)
    z4  = (p(1,3)^2 + p(2,3)^2)^(1/2)*exp(1i*atan2(p(2, 3),p(1, 3))); 

    % Vector desde el origen hasta (x2, y2)
    p(1, 2) = p(1, 3) - a(3)*cos(gamma);
    p(2, 2) = p(2, 3) - a(3)*sin(gamma);
    z3 = (p(1,2)^2 + p(2,2)^2)^(1/2)*exp(1i*atan2(p(2, 2),p(1, 2)));

    % Calculo la primer variable articular q1
    alpha = TdC(a(1), abs(z3), a(2));
    q(1, 1) = angle(z3) - alpha;
    q(1, 2) = angle(z3) + alpha;



    if abs(z4) > A13 + tol
        % Punto fuera del área de trabajo
        fprintf("\n|-----------------------------------------|\n")
        fprintf("No existe solución para esa restricción. \nPunto fuera del área de trabajo.")
        fprintf("\n|-----------------------------------------|\n")
        
        % Definir función para graficar
        FueraAT(p, a);
        pause;
    elseif abs(abs(z4) - A13) < tol
        if abs(gamma - angle(z4)) < tol
            % Hay solución única
            % Brazo estirado
            fprintf("\n|------------------------|\n")
            fprintf("| Existe Solución Única. |")
            fprintf("\n|------------------------|\n")

            q(1, 1) = gamma;
            q(1, 2) = gamma;

            for i = 2:3
                for j = 1:2
                    q(j, i) = 0;
                end
            end

            

            fprintf("\nSolución (q1, q2, q3): (%.4f, %.4f, %.4f)\n", q(1,1), q(2,1), q(3,1))
            SolucionUnica(p, a)

            pause;
        else
            % No hay solución debido al ángulo
            fprintf("\n|-----------------------------------------|\n")
            fprintf("|  No existe solución para esa restricción.  |")
            fprintf("\n|-----------------------------------------|\n")

            fprintf("El punto esta en la frontera del área de traba\n" + ...
                "por lo que no puede tener el ángulo pedido.")

            SolucionUnica(p, a); ax = gca; hold(ax,'on');

            T = findall(ax,'Type','text');
            idx = strcmp(get(T,'String'),'Dentro del Area de Trabajo');  % busca el que diga "\theta"
            htxt = T(idx); pos = get(htxt,'Position');
            
            set(htxt,'Position', pos + [real(0.5*exp(1i*deg2rad(130))), imag(0.5*exp(1i*deg2rad(130))), 0]);


            Lseg  = 0.5;          % largo de los segmentos extra
            r1    = 0.40;         % radio del primer arco (entre X y dirección z3-z4)
            r2    = 0.55;         % radio del segundo arco (entre X y zaux)
            col   = '#7A7A7A';
            
            % === LINEAS ===
            u_hat = z4;
            u_hat = Lseg * u_hat / abs(u_hat); % Cambio el largo

            v_hat = Lseg*exp(1i*gamma);
            

            % Paralela al eje X que pasa por z4
            plot(ax, [real(z4), real(z4)+Lseg], [imag(z4), imag(z4)], ...
                '--', 'Color', col, 'LineWidth', 1.5);
            
            % Para el segmento con dirección z3 -> z4
            plot(ax, [real(z4), real(z4+u_hat)], [imag(z4), imag(z4+u_hat)], ...
                '--', 'Color', col, 'LineWidth', 1.5)

            % Para la referencia de gamma
            plot(ax, [real(z4), real(z4+v_hat)], [imag(z4), imag(z4+v_hat)], ...
                '--', 'Color', col, 'LineWidth', 1.5)

            draw_arc(ax, z4, 0, angle(u_hat), 0.25, 1, sprintf('$\\theta = %.2f^{\\circ}$', rad2deg(angle(u_hat))))
            draw_arc(ax, z4, 0, gamma, 0.25, 1, sprintf('$\\gamma = %.2f^{\\circ}$', rad2deg(gamma)))

            
            pause;
        end
    elseif abs(z4) < (a(1) + a(2) - a(3))
        fprintf("\n|------------------------------|\n")
        fprintf("| Existe un Par de Soluciones.   |")
        fprintf("\n|------------------------------|\n")

        % Soluciones
        for s = 1:2
            p(1, 1) = a(1)*cos(q(1, s));
            p(2, 1) = a(1)*sin(q(1, s));
            T1 = homog2d(q(1,s), p(1, 1), p(2, 1));
    
            p1_2 = T1\p(:, 2);
            q(2, s) = atan2(p1_2(2), p1_2(1));
            q(3, s) = wrapToPi(gamma - (q(1, s) + q(2, s)));
        end
        
        fprintf("En formato (q1, q2, q3): \n")
        fprintf("\nSolución 1: (%.4f, %.4f, %.4f) [rad]", q(:, 1))
        fprintf("\nSolución 2: (%.4f, %.4f, %.4f) [rad]", q(:, 2))

        p1 = ParSol(a, q(:, 1));
        p2 = ParSol(a, q(:, 2));

        figure; clf;
        
        subplot(1,2,1);
        alcanzable(q(:,1), a, gamma, gca, 'showAngles');
        title('Solución 1');
        
        subplot(1,2,2);
        alcanzable(q(:,2), a, gamma, gca, 'showAngles');
        title('Solución 2');
        
        fprintf("\n(Toca cualquier tecla para continuar)\n\n")
        pause;

    else

        % Caso intermedio: a1+a2-a3 < ||p|| < a1+a2+a3, pero hay que chequear gamma
        rho   = abs(z4);             % ||p||
        theta = angle(z4);           % arg(p)

         % --- Filtro de compatibilidad de circunferencias (alcanzabilidad de gamma) ---
        A = (rho^2 + a(3)^2 - (a(1) + a(2))^2) / (2*rho*a(3));
        B = (rho^2 + a(3)^2 - (a(1) - a(2))^2) / (2*rho*a(3));

        % Clamp para que no me explote el acos()
        A = clamp(A,-1,1); B = clamp(B,-1,1);

        dmin = acos(B);    % límite inferior de |gamma - theta|
        dmax = acos(A);    % límite superior de |gamma - theta|

        % Normalizo el desfasaje pedido a [0,pi]
        Delta = abs(wrapToPi(gamma - theta));
        
        if Delta > pi, Delta = 2*pi - Delta; end

        if (Delta < dmin - tol) || (Delta > dmax + tol)
            fprintf('\nγ=%.2f° fuera del rango permitido: [%.2f°, %.2f°]. Sin solución.\n', ...
                rad2deg(Delta), rad2deg(dmin), rad2deg(dmax));
            % Podés graficar un aviso si querés.
            figure; clf; ax=gca; hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); grid minor;
            plot(ax, p(1,3), p(2,3), 'g.', 'MarkerSize', 18);
            title(ax, 'γ fuera de rango en el caso intermedio');
        else
            % --- γ es compatible: resolver 2R hasta z3 y cerrar con q3 ---
            z3_vec = [p(1,3); p(2,3)] - a(3)*[cos(gamma); sin(gamma)];
            c = norm(z3_vec);

            % IK 2R (dos ramas) para alcanzar z3
            phi   = atan2(z3_vec(2), z3_vec(1));
            alpha = acos( clamp( (a(1)^2 + c^2 - a(2)^2)/(2*a(1)*c), -1, 1) );

            q(1, 1) = wrapToPi(phi - alpha);    % codo-up
            q(1, 2) = wrapToPi(phi + alpha);    % codo-down

            % q2 relativo en cada rama (vector z1->z3 en el marco base)
            q(2, 1) = wrapToPi( atan2(z3_vec(2) - a(1)*sin(q(1, 1)), z3_vec(1) - a(1)*cos(q(1, 1))) - q(1, 1) );
            q(2, 2) = wrapToPi( atan2(z3_vec(2) - a(1)*sin(q(1, 2)), z3_vec(1) - a(1)*cos(q(1, 2))) - q(1, 2) );

            % q3 cierra la orientación: q1+q2+q3 = gamma
            q(3, 1) = wrapToPi( gamma - (q(1, 1) + q(2, 1)) );
            q(3, 2) = wrapToPi( gamma - (q(1, 2) + q(2, 2)) );

            q(:,1) = [q(1, 1); q(2, 1); q(3, 1)];
            q(:,2) = [q(1, 2); q(2, 2); q(3, 1)];

            if abs( abs(z4) - (a(1) + a(2) - a(3)) ) < tol
                % Caso donde existe 1 solución
                fprintf("\n|------------------------|\n")
                fprintf("| Existe Solución Única. |")
                fprintf("\n|------------------------|\n")

                fprintf("\nSolución 1: (%.4f, %.4f, %.4f) [rad]", q(:,1))
                
                % --- Gráfica ---
                figure; clf;
                alcanzable(q(:,1), a, gamma, gca, 'showAngles');
                title('Solución 1'); grid on;

            else
                % Caso donde existen 2 soluciones
                fprintf("\n|------------------------------|\n")
                fprintf("| Existe un Par de Soluciones.   |")
                fprintf("\n|------------------------------|\n")

                fprintf("\nSolución 1: (%.4f, %.4f, %.4f) [rad]", q(:,1))
                fprintf("\nSolución 2: (%.4f, %.4f, %.4f) [rad]\n", q(:,2))
    
                % --- Gráfica ---
                figure; clf;
    
                subplot(1,2,1);
                alcanzable(q(:,1), a, gamma, gca, 'showAngles');
                title('Solución 1'); grid on;
    
                subplot(1,2,2);
                alcanzable(q(:,2), a, gamma, gca, 'showAngles');
                title('Solución 2'); grid on;
            end
        end

        fprintf("\n(Toca cualquier tecla para continuar)\n\n")
        pause;
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

function h = FueraAT(p, a)
    figure; clf; ax = gca; hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); grid minor;
    xlabel(ax,'x'); ylabel(ax,'y'); title(ax,'Objetivo inalcanzable');
    
    A13 = a(1)+a(2)+a(3);
    z = zeros(4, 1);
    
    for s = 1:3
        M = 0;
        for n = 1:s
            M = a(n) + M;
        end
        z(s+1) = M*exp(1i*atan2(p(2, 3),p(1, 3)));
    end
    
    % Brazo totalmente estirado
    plot(ax, real([z(1) z(4)]), imag([z(1) z(4)]), 'b-', 'LineWidth', 2);
    
    % Puntos de articulaciones
    for s = 1:4
        plot(ax, real(z(s)), imag(z(s)), 'Or', 'MarkerSize', 7, 'LineWidth', 1);
    end
    
    % Punto objetivo
    plot(ax, p(1, 3), p(2, 3), 'x', 'Color', '#77AC30', 'MarkerSize', 15, 'LineWidth', 3);
    text(ax, p(1, 3), p(2, 3), '  objetivo', 'Color','r', 'FontSize',9);
    draw_circle(ax, z(1), A13, 'k--', '$\rm Radio del brazo$');
    
    
    xlim(ax, [-p(1, 3)-1 p(1, 3)+1]); ylim(ax, [-p(2, 3)-1 p(2, 3)+1]);
    
    fprintf("\n(Toca cualquier tecla para continuar)\n\n")

end

function h = SolucionUnica(p, a)

    figure; clf; ax = gca; hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); grid minor;
    xlabel(ax,'x'); ylabel(ax,'y'); title(ax,'Objetivo en la frontera del area de trabajo');

    A13 = a(1)+a(2)+a(3);
    z = zeros(4, 1);

    for s = 1:3
        M = 0;
        for n = 1:s
            M = a(n) + M;
        end
        z(s+1) = M*exp(1i*atan2(p(2, 3),p(1, 3)));
    end

    % Brazo totalmente estirado
    plot(ax, real([z(1) z(4)]), imag([z(1) z(4)]), 'b-', 'LineWidth', 2);

    % Puntos de articulaciones
    for s = 1:4
        plot(ax, real(z(s)), imag(z(s)), 'Or', 'MarkerSize', 7, 'LineWidth', 1);
    end

    % Punto objetivo
    plot(ax, p(1, 3), p(2, 3), '.', 'Color', '#77AC30', 'MarkerSize', 15, 'LineWidth', 2);
    text(ax, p(1, 3), p(2, 3), '  objetivo', 'Color','#77AC30', 'FontSize',9);

    q1 = rad2deg(angle(z(2)));
    draw_arc(ax, z(1), 0, angle(z(2)), a(1)/3, 0, sprintf('$q_{1} = %.4f^{\\circ}$', q1))

    q2 = 0;
    draw_arc(ax, z(2), angle(z(2)), 1e-5, a(2)/3, 0, sprintf('$q_{2} = %.4f^{\\circ}$', q2))
    q3 = 0;
    draw_arc(ax, z(3), angle(z(2)), 1e-5, a(3)/3, 0, sprintf('$q_{3} = %.4f^{\\circ}$', q3))

    angle_start = angle(z(4)) - pi/9;
    draw_arc(ax, z(1), angle_start, 2*pi/9, A13, 1, sprintf('Dentro del Area de Trabajo'));
    
    x_lim = abs(real(z(4))) + 1; y_lim = abs(imag(z(4))) + 1;
    xlim(ax, [-x_lim x_lim]); ylim(ax, [-y_lim y_lim]);

    fprintf("\n(Toca cualquier tecla para continuar)\n\n")

end

function alcanzable(q, a, gamma, ax, varargin)
% alcanzable  Dibuja un manipulador RRR planar en 2D
%
% @input:
%       q = [q1;q2;q3]   ángulos articulares (rad)
%       a = [a1 a2 a3]  longitudes de los eslabones
%       ax              eje de dibujo (usa gca si no se pasa)

    if nargin < 3 || isempty(ax)
        ax = gca;
    end

    % posiciones de cada junta (usando números complejos)
    z0 = 0;
    z1 = a(1)*exp(1i*q(1));
    z2 = z1 + a(2)*exp(1i*(q(1)+q(2)));
    z3 = z2 + a(3)*exp(1i*(q(1)+q(2)+q(3)));

    % limpiar y setear el eje
    cla(ax);
    hold(ax,'on'); axis(ax,'equal'); grid(ax,'on'); grid minor;
    xlabel(ax,'x'); ylabel(ax,'y');

    % eslabones
    plot(ax, real([z0 z1]), imag([z0 z1]), 'b-', 'LineWidth', 2);
    plot(ax, real([z1 z2]), imag([z1 z2]), 'b-', 'LineWidth', 2);
    plot(ax, real([z2 z3]), imag([z2 z3]), 'b-', 'LineWidth', 2);

    % pequeños segmentos para ver los ángulos

    % juntas
    plot(ax, real([z0 z1 z2 z3]), imag([z0 z1 z2 z3]), ...
        'Or', 'MarkerSize', 7, 'LineWidth', 1);

    % opcional: dibujar arcos de ángulos
    if any(strcmp(varargin,'showAngles'))
        r1 = 0.1*a(1);  r2 = 0.1*a(2);  r3 = 0.1*a(3);
        
        % Arco q1
        draw_arc(ax, z0, 0, q(1), r1, 0, sprintf('$q_{1} = %.4f^{\\circ}$', rad2deg(q(1))));
        
        % Arco q2
        plot(ax, [real(z1), real(z1 + 0.3*a(1)*exp(1i*angle(z1)))], [imag(z1), imag(z1 + 0.3*a(1)*exp(1i*angle(z1)))], '--', 'Color', '#77AC30', 'LineWidth', 1)
        draw_arc(ax, z1, q(1), q(2), r2, 0, sprintf('$q_{2} = %.4f^{\\circ}$', rad2deg(q(2))));
        
        % Arco q3
        plot(ax, [real(z2), real(z2 + 0.3*a(2)*exp(1i*angle(z2-z1)))], [imag(z2), imag(z2 + 0.3*a(2)*exp(1i*angle(z2-z1)))], '--', 'Color', '#77AC30', 'LineWidth', 1)
        draw_arc(ax, z2, q(1)+q(2), q(3), r3, 0, sprintf('$q_{3} = %.4f^{\\circ}$', rad2deg(q(3))));
        
        % Arco gamma
        plot(ax, [real(z3), real(z3 + 0.3*a(3)*exp(1i*angle(z3-z2)))], [imag(z3), imag(z3 + 0.3*a(3)*exp(1i*angle(z3-z2)))], '--', 'Color', '#77AC30', 'LineWidth', 1)
        plot(ax, [real(z3), real(z3 + 0.3*a(3))], [imag(z3), imag(z3)], '--', 'Color', '#77AC30', 'LineWidth', 1)
        draw_arc(ax, z3, 0, gamma, 1.2*r3, 0, sprintf('$\\gamma = %.4f^{\\circ}$', rad2deg(gamma)));
    end

    % ajustar límites
     alcance = max([abs(z1), abs(z2), abs(z3)]);
     xlim(ax, [-alcance*1.1, alcance*1.1]);
     ylim(ax, [-alcance*1.1, alcance*1.1]);
end


function draw_arc(ax, center, startAng, deltaAng, radius, as, labelStr)

         % @inputs:
%         ax
%         center: complejo
%         startAng [rad]
%         deltaAng [rad]
%         radius
%         dot: 0 / 1
%         labelStr: sprintf('Texto')
% 
% Dibuja un arco de ángulo 'deltaAng' desde 'startAng' con radio 'radius'
% y rotula en el medio del arco.
%
    if radius <= 0, return; end
    
    n = max(12, ceil(abs(deltaAng)*30));        % densidad decente
    t0 = startAng;
    t1 = startAng + deltaAng;
    ts = linspace(t0, t1, n);
    xx = real(center) + radius*cos(ts);
    yy = imag(center) + radius*sin(ts);

    % marca de sentido
    mid = (t0 + t1)/2;
    xm = real(center) + (radius+0.06*radius)*cos(mid);
    ym = imag(center) + (radius+0.06*radius)*sin(mid);

    if abs(deltaAng) < 1e-6
        if nargin >= 7 && ~isempty(labelStr)
            interp = 'tex';
            if contains(labelStr, '$'), interp = 'latex'; end
            text(ax, xm, ym, labelStr, 'FontSize', 12, 'Color', 'k', 'Interpreter', interp);
        end
        return;
    end

    if (nargin >= 6) && (as == 1)
        plot(ax, xm, ym, '--', 'MarkerSize', 8);
        plot(ax, xx, yy, '--', 'LineWidth', 1);
    else
        plot(ax, xm, ym, 'k.', 'MarkerSize', 8);
        plot(ax, xx, yy, 'k-', 'LineWidth', 1);
    end

    if nargin >= 7 && ~isempty(labelStr)
        interp = 'tex';
        if contains(labelStr, '$'), interp = 'latex'; end
        text(ax, xm, ym, labelStr, 'FontSize', 12, 'Color', 'k', 'Interpreter', interp);
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
                 'Color', col, 'FontSize', 9, 'Interpreter','latex');
    end
end

function T = homog2d(theta, dx, dy)
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];
    t = [dx; dy];
    T = [R t; 0 0 1];
end

function p = ParSol(a, q)
% 
% @inputs
%     a: vector de eslabones 3x1
%     q: vector de coordenadas modales 3x1
% 
% @outpus:
%     p: matriz 3x3 de todos los puntos
%         las coordenadas son en R2, solo que tienen
%         el 1 como tercer valor para hacer multiplicaciones
%         por matrices de transformación homogenea

    p = ones(3, 3);

    % Sinceramente, con complejos es más facil
    
    z0 = 0;
    z1 = a(1)*exp(1i*q(1));
    z2 = z1 + a(2)*exp(1i*(q(1)+q(2)));
    z3 = z2 + a(3)*exp(1i*(q(1)+q(2)+q(3)));


    % Segunda Articulación
    p(1, 1) = real(z1);
    p(2, 1) = imag(z1);

    % Tercer Articulación
    p(1, 2) = real(z2);
    p(2, 2) = imag(z2);

    % Extremo
    p(1, 3) = real(z3);
    p(2, 3) = imag(z3);


end

function gamma = TdC(a, b, c)
    % devuelve el ángulo gamma (en radianes)
    val = (a^2 + b^2 - c^2) / (2*a*b);
    val = min(max(val, -1), 1); % clamp para evitar problemas numéricos
    gamma = acos(val);
end 