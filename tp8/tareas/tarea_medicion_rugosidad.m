% --- EJERCICIO FINAL FARO SCANARM ---
clear; clc; close all;
clear functions;
%% --- 0. Cargar Robot y Parámetros ---
try
    robot; % Carga R desde robots/robot_faro.m mediante el wrapper
    fprintf('Robot "%s" cargado correctamente.\n', R.name);
catch ME
    error('No se pudo cargar robot.m: %s', ME.message);
end

%% --- 1. Definir Puntos Cartesianos y Articulares Clave ---
% --- Puntos Articulares (q) ---
q_start = deg2rad([0, 0, -54.4, 3.8, -55.2, 0]);    
q_approach = deg2rad([0, 0, -3.8, -26.4, 0, 0]);    

% --- Parámetros de la Tarea ---
delta_z = 0.001; % Distancia vertical para simular contacto (1 mm)
delta_x = 0.05;  % Distancia de medición en X (5 cm)
t_seg1 = 4.0;    % Duración acercamiento (s)
t_seg2 = 5.0;    % Duración medición (s)
t_seg3 = 4.0;    % Duración retorno (s)
t_touch = 0.5;   % Duración corta para contacto/retracción (s)
dt = 0.05;       % Paso de tiempo para discretización (s)

% --- Calcular Matrices de Transformación Homogénea (T) ---
T_start = R.fkine(q_start);
T_approach_obj = R.fkine(q_approach); % Guardar objeto SE3
T_approach_mat = T_approach_obj.T;    % Obtener matriz 4x4

% Calcular Orientación del Palpador (45 grados)
R_palpador = roty(-45 * pi/180);
R_approach_original = T_approach_mat(1:3, 1:3);

% Reconstruir base ortonormal para R_palpador_final
x_vec_orig = R_approach_original(:,1);
y_vec_orig = R_approach_original(:,2);
z_vec_nuevo = R_palpador * [0; 0; 1];
z_vec_nuevo = z_vec_nuevo / norm(z_vec_nuevo);
y_vec_temp = y_vec_orig - dot(y_vec_orig, z_vec_nuevo) * z_vec_nuevo;
y_vec_nuevo = y_vec_temp / norm(y_vec_temp);
x_vec_nuevo = cross(y_vec_nuevo, z_vec_nuevo);
x_vec_nuevo = x_vec_nuevo / norm(x_vec_nuevo);
R_palpador_final = [x_vec_nuevo, y_vec_nuevo, z_vec_nuevo];

if abs(det(R_palpador_final) - 1) > 1e-5
    error('R_palpador_final no es una matriz de rotación válida.');
end
disp('Matriz de Orientación del Palpador (45 deg):');
disp(R_palpador_final);

% Actualizar T_approach con la orientación correcta (usar pos original)
T_approach = rt2tr(R_palpador_final, T_approach_mat(1:3, 4)); % rt2tr necesita columna

% Calcular Poses de Contacto, Fin y Retracción
vec_z_tool = T_approach(1:3, 3);   % Eje Z de la herramienta
T_contact_start_pos = T_approach(1:3, 4) - delta_z * vec_z_tool;
T_contact_start = rt2tr(R_palpador_final, T_contact_start_pos);
T_contact_end_pos = T_contact_start_pos + [delta_x; 0; 0]; % Mover 5cm en X global
T_contact_end = rt2tr(R_palpador_final, T_contact_end_pos);
T_retract_pos = T_contact_end_pos + delta_z * vec_z_tool;
T_retract = rt2tr(R_palpador_final, T_retract_pos);

% Calcular Puntos Articulares (q) faltantes usando Cinemática Inversa
try
    % VERIFICAR DIMENSIONES DE LA SEMILLA ANTES DE LLAMAR
    q_approach_col = q_approach.'; % Asegurar que la semilla es columna
    disp(['Dimensiones de q_approach pasada a cin_inv_Faro (llamada 1): ', num2str(size(q_approach_col))]);

    q_contact_start = cin_inv_Faro(R, T_contact_start, q_approach_col, true); % Pasar columna
    disp(['Dimensiones de q_contact_start devuelta: ', num2str(size(q_contact_start))]); % Debería ser 6x1

    q_contact_end = cin_inv_Faro(R, T_contact_end, q_contact_start, true); % Semilla debe ser 6x1
    disp(['Dimensiones de q_contact_end devuelta: ', num2str(size(q_contact_end))]); % Debería ser 6x1

    q_retract = cin_inv_Faro(R, T_retract, q_contact_end, true); % Semilla debe ser 6x1
    disp(['Dimensiones de q_retract devuelta: ', num2str(size(q_retract))]); % Debería ser 6x1

catch ME
    fprintf('!!! Error capturado DENTRO de cin_inv_Faro !!!\n');
    fprintf('Mensaje Original: %s\n', ME.message); % El error de dimensiones

    % Imprimir el Stack Trace para ver la línea exacta
    fprintf('--- Stack Trace del Error ---\n');
    for k = 1:length(ME.stack)
        fprintf('Archivo: %s, Función: %s, Línea: %d\n', ...
                ME.stack(k).file, ME.stack(k).name, ME.stack(k).line);
        % Opcional: Detenerse después de la primera entrada relevante si es muy largo
        % if contains(ME.stack(k).name, 'cin_inv_Faro') || contains(ME.stack(k).name, 'calcular_qm')
        %     break;
        % end
    end
    fprintf('-----------------------------\n');

    % Detener ejecución con un error más informativo
    error('Falló la cinemática inversa (ver stack trace arriba). Posible pose inalcanzable o singularidad cerca.');
end

% --- Vectores de Tiempo y Pasos ---
N1 = round(t_seg1 / dt);
N_touch = round(t_touch / dt);
N2 = round(t_seg2 / dt);
N3 = round(t_seg3 / dt);
t1 = linspace(0, t_seg1, N1);
t_touch_vec = linspace(0, t_touch, N_touch);
t2 = linspace(0, t_seg2, N2);
t3 = linspace(0, t_seg3, N3);
% Tiempo total para gráficas
t_total = [t1, t1(end)+t_touch_vec(2:end), t1(end)+t_touch+t2(2:end), t1(end)+t_touch+t2(end)+t_touch_vec(2:end), t1(end)+t_touch+t2(end)+t_touch+t3(2:end)];

fprintf('--- Puntos Articulares Calculados ---\n');
fprintf('q_start:         %s\n', mat2str(rad2deg(q_start), 3));
fprintf('q_approach:      %s\n', mat2str(rad2deg(q_approach), 3));
fprintf('q_contact_start: %s\n', mat2str(rad2deg(q_contact_start'), 3)); % Transponer para mostrar como fila
fprintf('q_contact_end:   %s\n', mat2str(rad2deg(q_contact_end'), 3)); % Transponer para mostrar como fila
fprintf('q_retract:       %s\n', mat2str(rad2deg(q_retract'), 3));     % Transponer para mostrar como fila

%% --- 2. Implementación A: Interpolación Articular (jtraj) ---
fprintf('\n--- Calculando Método A (Todo jtraj) ---\n');
% Transponer (.') los resultados de cin_inv_Faro para asegurar que sean FILAS (1x6) para jtraj
q_contact_start_fila = q_contact_start.';
q_contact_end_fila   = q_contact_end.';
q_retract_fila       = q_retract.';

[qA1, qdA1, qddA1] = jtraj(q_start, q_approach, N1);
[qA2, qdA2, qddA2] = jtraj(q_approach, q_contact_start_fila, N_touch);
[qA3, qdA3, qddA3] = jtraj(q_contact_start_fila, q_contact_end_fila, N2);
[qA4, qdA4, qddA4] = jtraj(q_contact_end_fila, q_retract_fila, N_touch);
[qA5, qdA5, qddA5] = jtraj(q_retract_fila, q_start, N3);

% Concatenar (quitando puntos repetidos entre segmentos)
qA = [qA1; qA2(2:end,:); qA3(2:end,:); qA4(2:end,:); qA5(2:end,:)];
qdA = [qdA1; qdA2(2:end,:); qdA3(2:end,:); qdA4(2:end,:); qdA5(2:end,:)];
qddA = [qddA1; qddA2(2:end,:); qddA3(2:end,:); qddA4(2:end,:); qddA5(2:end,:)];

% Calcular trayectoria cartesiana resultante
total_puntos_A = size(qA, 1);
posA = zeros(total_puntos_A, 3);
for i = 1:total_puntos_A
    T_curr = R.fkine(qA(i,:));
    posA(i,:) = T_curr.t';
end
velA = derivada(posA, t_total');
accA = derivada(velA, t_total');

%% --- 3. Implementación B: Interpolación Mixta (jtraj + ctraj) ---
% Comentado para probar primero la Implementación A  no mw funciona :(
% Descomenta esta sección una vez que la Implementación A funcione
%{
fprintf('--- Calculando Método B (Mixto jtraj/ctraj) ---\n');
% Segmento 1: jtraj (Start -> Approach)
[qB1, qdB1, qddB1] = jtraj(q_start, q_approach, N1);
% Segmento 2: jtraj (Approach -> ContactStart)
[qB2, qdB2, qddB2] = jtraj(q_approach, q_contact_start.', N_touch); % Usar fila q_contact_start_fila

% Segmento 3: ctraj (ContactStart -> ContactEnd) - Medición
T_segmento3 = ctraj(T_contact_start, T_contact_end, N2);
qB3 = zeros(N2, R.n);
q_seed_B = q_contact_start.'; % Semilla inicial es el final del segmento anterior (fila)
for i = 1:N2
    try
        q_sol = cin_inv_Faro(R, T_segmento3(:,:,i), q_seed_B.', true); % Pasar semilla como COLUMNA
        if ~isempty(q_sol) && ~any(isnan(q_sol))
             if size(q_sol, 1) > 1; q_sol = q_sol.'; end % Asegurar q_sol sea FILA para guardar y semilla
             qB3(i,:) = q_sol;
             q_seed_B = q_sol; % Actualizar semilla (fila)
        else
             warning('IK B: No se encontró solución válida en el paso %d. Repitiendo anterior.', i);
             if i>1; qB3(i,:) = qB3(i-1,:); else; qB3(i,:) = q_seed_B; end % Usar semilla si falla el 1ro
        end
    catch ME_ikine
        warning('IK B: Error en cin_inv_Faro en el paso %d: %s. Repitiendo anterior.', i, ME_ikine.message);
        if i>1; qB3(i,:) = qB3(i-1,:); else; qB3(i,:) = q_seed_B; end % Usar semilla si falla el 1ro
    end
end
% Calcular qd, qdd numéricamente para el segmento 3
t_seg2_vec = linspace(0, t_seg2, N2)';
qdB3 = derivada(qB3, t_seg2_vec);
qddB3 = derivada(qdB3, t_seg2_vec);

% Segmento 4: jtraj (ContactEnd -> Retract)
q_end_seg3 = qB3(end,:); % Posición final del segmento ctraj (fila)
[qB4, qdB4, qddB4] = jtraj(q_end_seg3, q_retract.', N_touch); % Usar fila q_retract_fila

% Segmento 5: jtraj (Retract -> Start)
[qB5, qdB5, qddB5] = jtraj(q_retract.', q_start, N3); % Usar fila q_retract_fila

% Concatenar (quitando puntos repetidos)
qB = [qB1; qB2(2:end,:); qB3(2:end,:); qB4(2:end,:); qB5(2:end,:)];
qdB = [qdB1; qdB2(2:end,:); qdB3(2:end,:); qdB4(2:end,:); qdB5(2:end,:)];
qddB = [qddB1; qddB2(2:end,:); qddB3(2:end,:); qddB4(2:end,:); qddB5(2:end,:)];

% Calcular trayectoria cartesiana resultante
total_puntos_B = size(qB, 1);
posB = zeros(total_puntos_B, 3);
for i = 1:total_puntos_B
    T_curr = R.fkine(qB(i,:));
    posB(i,:) = T_curr.t';
end
velB = derivada(posB, t_total');
accB = derivada(velB, t_total');
%}

%% --- 4. Animación ---
figure('Name', 'Animación Método A (Articular)');
R.plot(qA);
title('Animación Método A (Todo jtraj)');

% Descomenta cuando Método B funcione
%{
figure('Name', 'Animación Método B (Mixto)');
R.plot(qB);
title('Animación Método B (jtraj + ctraj)');
%}

%% --- 5. Gráficas Comparativas ---
% Ajustar vector de tiempo si B está comentado
if ~exist('qB', 'var') % Si qB no existe (porque B está comentado)
    qB = qA; qdB = qdA; qddB = qddA; % Copiar A para evitar errores en plot
    posB = posA; velB = velA; accB = accA;
    t_total_B = t_total_A;
    warning('La sección de Implementación B está comentada. Mostrando A vs A en comparativas.');
else
    % Asegurarse que t_total tenga la misma longitud que las trayectorias
    if length(t_total) ~= size(qA, 1) || length(t_total) ~= size(qB, 1)
       warning('Discrepancia en longitud de vectores de tiempo y trayectorias. Ajustando t_total.');
       t_total_A = linspace(t_total(1), t_total(end), size(qA,1));
       t_total_B = linspace(t_total(1), t_total(end), size(qB,1));
    else
       t_total_A = t_total;
       t_total_B = t_total;
    end
end

% --- Espacio Articular ---
figure('Name', 'Comparación Articular');
% Posición
subplot(3,1,1); plot(t_total_A, qA, '--'); hold on; plot(t_total_B, qB, '-'); grid on; title('Comparación Posición Articular'); ylabel('Ángulo [rad]'); legend_q = arrayfun(@(x) sprintf('q%d', x), 1:R.n, 'UniformOutput', false); legend([strcat(legend_q, '-A'), strcat(legend_q, '-B')], 'Location', 'bestoutside');
% Velocidad
subplot(3,1,2); plot(t_total_A, qdA, '--'); hold on; plot(t_total_B, qdB, '-'); grid on; title('Comparación Velocidad Articular'); ylabel('Velocidad [rad/s]'); legend([strcat(legend_q, '-A'), strcat(legend_q, '-B')], 'Location', 'bestoutside');
% Aceleración
subplot(3,1,3); plot(t_total_A, qddA, '--'); hold on; plot(t_total_B, qddB, '-'); grid on; title('Comparación Aceleración Articular'); ylabel('Aceleración [rad/s^2]'); xlabel('Tiempo [s]'); legend([strcat(legend_q, '-A'), strcat(legend_q, '-B')], 'Location', 'bestoutside');

% --- Espacio Cartesiano ---
figure('Name', 'Comparación Cartesiana');
legend_xyz = {'X', 'Y', 'Z'};
% Posición
subplot(3,1,1); plot(t_total_A, posA, '--'); hold on; plot(t_total_B, posB, '-'); grid on; title('Comparación Posición Cartesiana'); ylabel('Posición [m]'); legend([strcat(legend_xyz, '-A'), strcat(legend_xyz, '-B')], 'Location', 'bestoutside');
% Velocidad
subplot(3,1,2); plot(t_total_A, velA, '--'); hold on; plot(t_total_B, velB, '-'); grid on; title('Comparación Velocidad Cartesiana'); ylabel('Velocidad [m/s]'); legend([strcat(legend_xyz, '-A'), strcat(legend_xyz, '-B')], 'Location', 'bestoutside');
% Aceleración
subplot(3,1,3); plot(t_total_A, accA, '--'); hold on; plot(t_total_B, accB, '-'); grid on; title('Comparación Aceleración Cartesiana'); ylabel('Aceleración [m/s^2]'); xlabel('Tiempo [s]'); legend([strcat(legend_xyz, '-A'), strcat(legend_xyz, '-B')], 'Location', 'bestoutside');

% --- Gráfico Z vs X (Geometría) ---
figure('Name', 'Trayectoria Geométrica Z vs X');
plot(posA(:,1), posA(:,3), '--', 'LineWidth', 1.5, 'DisplayName', 'Método A (jtraj)'); hold on;
plot(posB(:,1), posB(:,3), '-', 'LineWidth', 1.5, 'DisplayName', 'Método B (Mixto)'); grid on; % Cambiado ctraj a Mixto
axis equal; xlabel('X [m]'); ylabel('Z [m]');
title('Comparación Trayectoria Geométrica en Plano XZ');
legend('Location', 'best');

%% --- 6. Selección y Justificación (Inciso 5) ---
fprintf('\n--- Selección del Método (Inciso 5) ---\n');
fprintf('El método más adecuado para la aplicación de medición de rugosidad es el Método B (Mixto).\n');
fprintf('Justificación:\n');
fprintf('1. Fase de Medición (P3 a P4): Requiere que el palpador siga una LÍNEA RECTA precisa sobre la superficie y mantenga una ORIENTACIÓN CONSTANTE (45 grados). Esto SOLO se garantiza con la interpolación cartesiana (`ctraj`). El Método A (solo `jtraj`) generaría un arco, haciendo la medición incorrecta.\n');
fprintf('2. Fases de Acercamiento y Retorno (P1->P3 y P4->P6): El camino exacto no es crítico. La interpolación articular (`jtraj`) es preferible porque genera movimientos más suaves para las articulaciones del robot, reduce vibraciones y es computacionalmente menos costosa.\n');
fprintf('El Método B combina lo mejor de ambos: precisión cartesiana donde es necesaria (medición) y suavidad articular donde es posible (movimientos de transición).\n');

%% --- Función Derivada (Recreada del TP) ---
function d = derivada(vz, t)
    % Wrapper: delega en RoboticaUtils.derivada para centralizar lógica
    d = RoboticaUtils.derivada(vz, t);
end
