function Q = cin_inv_Faro(R, T, q0, mejor)
    T = invHomog(R.base) * T * invHomog(R.tool);

    % --- Paso 1: centro de la muñeca ---
    d6 = R.links(6).d;          % distancia DH del efector
    p  = T(1:3,4) - d6 * T(1:3,3);  % posición del centro de la muñeca

    % --- Paso 2: cálculo de q1 ---
    q1a = atan2(p(2), p(1));
    q1b = q1a + pi;
    q1  = [q1a, q1b];

    % --- Paso 3: cálculo de q2 y q3 (geométrico) ---
    % Usamos los parámetros DH
    L2 = R.links(2).a;   % longitud a2
    L3 = R.links(4).d;   % distancia d4
    soluciones = [];

    for i=1:2
        % rotamos el punto al sistema {1}
        T1 = R.links(1).A(q1(i)).double;
        pc = invHomog(T1) * [p; 1];
        px = pc(1); py = pc(2);

        B = atan2(py, px);
        r = sqrt(px^2 + py^2);

        % ley de cosenos
        G = acos((L2^2 + r^2 - L3^2) / (2*L2*r));
        q2a = B - G;
        q2b = B + G;

        % q3 (corregido por -pi/2)
        for q2 = [q2a, q2b]
            T2 = T1 * R.links(2).A(q2).double;
            pc2 = invHomog(T2) * [p; 1];
            q3 = atan2(pc2(2), pc2(1)) - pi/2;

            % --- Paso 4: orientación (q4,q5,q6) ---
            T3  = T2 * R.links(3).A(q3).double;
            T36 = invHomog(T3) * T;
            
            % q4
            q4a = atan2(T36(2,3), T36(1,3));
            q4b = q4a + pi;

            for q4 = [q4a, q4b]
                T4 = R.links(4).A(q4).double;
                T46 = invHomog(T4) * T36;

                % q5
                q5 = atan2(T46(2,3), T46(1,3)) - pi/2;

                % q6
                T5 = R.links(5).A(q5).double;
                T56 = invHomog(T5) * T46;
                q6 = atan2(T56(2,1), T56(1,1));

                % guardar solución
                soluciones = [soluciones, [q1(i); q2; q3; q4; q5; q6]];
            end
        end
    end

    % --- Paso 5: salida ---
    if mejor
        % elegir la más cercana a q0
        difs = soluciones - q0(:)*ones(1,size(soluciones,2));
        normas = vecnorm(difs);
        [~,idx] = min(normas);
        Q = soluciones(:,idx);
    else
        Q = soluciones;
    end
end
% CIN_INV_FARO
% Función principal de CINEMÁTICA INVERSA para el robot Faro.
% Recibe:
%   R   -> robot (SerialLink con parámetros DH)
%   T   -> matriz homogénea objetivo (posición + orientación)
%   q0  -> postura inicial (semilla)
%   mejor -> true  = devuelve la solución más cercana a q0
%            false = devuelve todas las soluciones
% Devuelve:
%   Q -> conjunto de ángulos articulares que llevan al efector a T
%
% En pocas palabras:
%   A partir de una posición/orientación deseada del extremo,
%   calcula los ángulos de las articulaciones que permiten alcanzarla.

