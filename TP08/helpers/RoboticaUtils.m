classdef RoboticaUtils
    % RoboticaUtils
    % Utilidades comunes para robótica usadas en TP8.
    %
    % Métodos públicos estáticos:
    %   - invHomog(T): inversa de una transformación homogénea 4x4/SE3.
    %   - ikFaro(R, T, q0, mejor): cinemática inversa geométrica del Faro.
    %   - derivada(vz, t): derivada numérica (primera) de series discretas.
    %
    % Notas:
    %   - Unidades en radianes.
    %   - Para ikFaro, la semilla q0 puede ser fila o columna; se usa para
    %     seleccionar la solución más cercana si 'mejor' es true.
    %   - Se valida que R.offset tenga dimensiones compatibles con R.n.
    %
    methods (Static)
        function Tinv = invHomog(T)
            % invHomog
            % Devuelve la inversa de una matriz homogénea.
            % Acepta: SE3, struct con campo T, o double 4x4.

            if isa(T,'SE3')
                M = T.T;
            elseif isstruct(T) && isfield(T,'T')
                M = T.T;
            else
                M = T;  % asumimos double(4x4)
            end

            Rm = M(1:3,1:3);
            p = M(1:3,4);
            Rt = Rm.';
            Tinv = [Rt, -Rt*p; 0 0 0 1];
        end

        function Q = ikFaro(R, T, q0, mejor)
            % ikFaro
            % Cinemática inversa geométrica para el robot Faro (6 GDL).
            % Q = ikFaro(R, T, q0, mejor)
            %   R     -> SerialLink del robot
            %   T     -> objetivo homogéneo (SE3 o 4x4)
            %   q0    -> semilla (1x6 o 6x1)
            %   mejor -> true: devuelve 6x1 más cercana a q0
            %            false: devuelve todas las 8 soluciones 6x8

            % Guardar y anular temporalmente offsets
            offsets = R.offset;
            R.offset = zeros(6,1);

            % Normalizar T al marco del robot
            T = RoboticaUtils.invHomog(R.base) * T * RoboticaUtils.invHomog(R.tool);

            % Centro de la muñeca respecto de S0
            d6 = R.links(6).d;
            p  = T(1:3,4) - d6 * T(1:3,3);

            % q1 por proyección en XY (dos posibilidades separadas por pi)
            q1a = atan2(p(2), p(1));
            if q1a > 0, q1b = q1a - pi; else, q1b = q1a + pi; end
            q1 = [q1a, q1b];

            soluciones = zeros(6,8);
            soluciones(1,:) = [q1a q1a q1a q1a q1b q1b q1b q1b];
            idx = 1;
            for i = 1:length(q1)
                L2 = R.links(2).a;
                L3 = R.links(3).a;

                % p respecto S1
                T1 = R.links(1).A(q1(i)).double;
                p_1 = RoboticaUtils.invHomog(T1) * [p; 1];

                % Proyección al plano X1Y1
                r = sqrt(p_1(1)^2 + p_1(2)^2);
                if r < 1e-8
                    warning('r muy pequeño: posible singularidad geométrica');
                    continue;
                end
                B = atan2(p_1(2), p_1(1));
                G = acos((L2^2 + r^2 - L3^2) / (2 * r * L2));

                q2a = B - real(G);
                q2b = B + real(G);
                q2 = [q2a, q2b] - 2*pi;

                for k = 1:2
                    for j = 1:2 %#ok<FXSET>
                        soluciones(2,idx) = q2(k);
                        T2 = R.links(2).A(q2(k)).double;
                        p_2 = RoboticaUtils.invHomog(T2) * p_1;  % p respecto de S2
                        q3 = atan2(p_2(2), p_2(1));
                        soluciones(3,idx) = q3;
                        idx = idx + 1;
                    end
                end
            end

            % Calcular q4, q5 y q6 para cada rama (de a pares)
            for i = 1:2:7
                q1v = soluciones(1, i);
                q2v = soluciones(2, i);
                q3v = soluciones(3, i);
                [q4, q5, q6] = RoboticaUtils.calcular_qm(R, q1v, q2v, q3v, T, q0);
                soluciones(4:6, i:i+1) = [q4; q5; q6];
            end

            % Restaurar offsets y validar dimensiones
            R.offset = offsets;
            offset_local = R.offset;
            if size(offset_local, 1) == 1
                offset_local = offset_local.';
            end
            if size(offset_local, 1) ~= R.n
                error('Dimensiones de offset %s no coinciden con N=%d.', mat2str(size(offset_local)), R.n);
            end

            % Compensar offset del modelo
            % soluciones = soluciones - repmat(R.offset, 1, 8);
            soluciones = soluciones - repmat(offset_local, 1, size(soluciones,2));

            if mejor
                Qaux = soluciones - repmat(q0, 1, 8);
                normas = zeros(1,8);
                for i = 1:8
                    normas(i) = norm(Qaux(:,i));
                end
                [~, pos] = min(normas);
                Q = soluciones(:, pos);
            else
                Q = soluciones;
            end
        end

        function d = derivada(vz, t)
            % derivada
            % Derivada numérica de una señal (por columnas) en tiempos t.
            % Soporta malla temporal constante o variable.

            [m, n] = size(vz);
            if size(t, 2) > 1
                t = t';
            end
            if m <= 2
                warning('Pocos puntos para derivada.');
                d = zeros(m, n);
                return;
            end
            if length(t) ~= m
                error('Tiempo y datos no coinciden.');
            end
            dt_vec  = diff(t);
            dt_mean = mean(dt_vec);
            d = zeros(m, n);
            if any(abs(dt_vec - dt_mean) > 1e-6 * abs(dt_mean))
                % Tiempo no constante
                for j = 1:n
                    d(1,j) = (-3*vz(1,j) + 4*vz(2,j) - vz(3,j)) / (t(3) - t(1));
                    for i = 2:m-1
                        d(i,j) = (vz(i+1,j) - vz(i-1,j)) / (t(i+1) - t(i-1));
                    end
                    d(m,j) = (3*vz(m,j) - 4*vz(m-1,j) + vz(m-2,j)) / (t(m) - t(m-2));
                end
            else
                % Tiempo constante
                dt = dt_mean;
                if abs(dt) < eps
                    warning('dt casi cero');
                    return;
                end
                for j = 1:n
                    d(1,j) = (-3*vz(1,j) + 4*vz(2,j) - vz(3,j)) / (2*dt);
                    for i = 2:m-1
                        d(i,j) = (vz(i+1,j) - vz(i-1,j)) / (2*dt);
                    end
                    d(m,j) = (3*vz(m,j) - 4*vz(m-1,j) + vz(m-2,j)) / (2*dt);
                end
            end
        end
    end

    methods (Static, Access = private)
        function [q4,q5,q6] = calcular_qm(R, q1, q2, q3, T, q0)
            T1 = R.links(1).A(q1).double;
            T2 = R.links(2).A(q2).double;
            T3 = R.links(3).A(q3).double;

            T36 = RoboticaUtils.invHomog(T3) * RoboticaUtils.invHomog(T2) * RoboticaUtils.invHomog(T1) * T;
            tol = 1e-6;
            if abs(T36(3,3) - 1) < tol
                % Solución degenerada: z3 y z5 alineados
                warning('Caso degenerado');
                q4(1) = q0(4);
                q5(1) = 0;
                q6(1) = atan2(T36(2,1), T36(1,1)) - q4(1);
                q4(2) = q4(1);
                q5(2) = 0;
                q6(2) = q6(1);
            else
                % Solución normal
                q4(1) = atan2(T36(2,3), T36(1,3));
                if q4(1) > 0, q4(2) = q4(1) - pi; else, q4(2) = q4(1) + pi; end
                q5 = zeros(1,2);
                q6 = q5;
                for i = 1:2
                    T4 = R.links(4).A(q4(i)).double;
                    T6 = RoboticaUtils.invHomog(T4) * T36;
                    q5(i) = atan2(T6(2,3), T6(1,3)) + pi/2;

                    T5 = R.links(5).A(q5(i)).double;
                    T6 = RoboticaUtils.invHomog(T5) * T6;
                    q6(i) = atan2(T6(2,1), T6(1,1));
                end
            end
        end
    end
end

