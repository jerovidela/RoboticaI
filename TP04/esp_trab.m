function esp_trab()
% ESP_TRAB - Calcula y grafica la superficie de trabajo del robot.
% Genera una representación precisa basada en el barrido de los límites articulares.
% Requiere que el robot esté definido en el workspace (ejecutar 'robot.m' antes).

    % === Cargar el robot del workspace ===
    R = evalin('base', 'R');

    % === Límites articulares ===
    qlims = R.qlim;

    % === Parámetros de barrido ===
    numSamples = 300; % Más puntos para mayor suavidad

    % === Superficie XY (Vista Superior) ===
    % Generar la forma de dona

    % 1. Alcance Máximo (Contorno exterior)
    q_ext = [0, qlims(2,2), qlims(3,1), 0, 0, 0]; 
    pos_ext_xy = zeros(numSamples, 2);
    q1_range = linspace(qlims(1,1), qlims(1,2), numSamples);
    
    for i = 1:numSamples
        q_ext(1) = q1_range(i);
        T = R.fkine(q_ext);
        pos = transl(T)';
        pos_ext_xy(i,:) = [pos(1), pos(2)];
    end

    % 2. Alcance Mínimo (Contorno interior)
    q_min = [0, qlims(2,1), qlims(3,2), 0, 0, 0]; 
    pos_min_xy = zeros(numSamples, 2);
    
    for i = 1:numSamples
        q_min(1) = q1_range(i);
        T = R.fkine(q_min);
        pos = transl(T)';
        pos_min_xy(i,:) = [pos(1), pos(2)];
    end

    % --- Gráfico XY ---
    figure('Name','Espacio de Trabajo - Vista Superior (XY)');
    hold on;
    fill(pos_ext_xy(:,1), pos_ext_xy(:,2), [0.7 0.7 0.7], 'FaceAlpha', 0.5); % Gris
    fill(pos_min_xy(:,1), pos_min_xy(:,2), 'w', 'FaceAlpha', 1);
    plot(pos_ext_xy(:,1), pos_ext_xy(:,2), 'k-', 'LineWidth', 1.5);
    plot(pos_min_xy(:,1), pos_min_xy(:,2), 'k--', 'LineWidth', 1);
    xlabel('X [m]'); ylabel('Y [m]');
    title('Espacio de Trabajo - Plano XY (Anillo)');
    grid on; axis equal;
    hold off;
    
    % === Superficie XZ (Vista Lateral - como la imagen de referencia) ===
    figure('Name','Espacio de Trabajo - Vista Lateral (XZ)');
    hold on;

    % Fijar q1, q4, q5, q6 para una vista lateral (normalmente q1=0 o q1=pi/2)
    % Se asume q1=0 para este perfil. q4, q5, q6 en su posición neutra o extendida.
    q_fixed_xz = [0, 0, 0, 0, 0, 0]; 
    q_fixed_xz(4) = 0; % q4 a 0 para simplificar el perfil, o extenderlo
    q_fixed_xz(5) = 0; % q5 a 0
    q_fixed_xz(6) = 0; % q6 a 0

    % Generar un mallado de puntos variando q2 y q3
    q2_range = linspace(qlims(2,1), qlims(2,2), numSamples);
    q3_range = linspace(qlims(3,1), qlims(3,2), numSamples);
    
    pos_all_xz = [];

    for i = 1:numSamples
        for j = 1:numSamples
            q_fixed_xz(2) = q2_range(i);
            q_fixed_xz(3) = q3_range(j);
            T = R.fkine(q_fixed_xz);
            pos = transl(T)';
            pos_all_xz = [pos_all_xz; pos(1), pos(3)];
        end
    end

    % Eliminar puntos duplicados o muy cercanos (opcional, para convhull)
    % pos_all_xz = unique(pos_all_xz, 'rows'); % Descomentar si hay problemas de rendimiento

    % Calcular el envoltorio convexo de todos los puntos generados en XZ
    k_xz = convhull(pos_all_xz(:,1), pos_all_xz(:,2));
    
    % Rellenar el área de trabajo XZ
    fill(pos_all_xz(k_xz, 1), pos_all_xz(k_xz, 2), [0.7 0.7 0.7], 'FaceAlpha', 0.5); % Gris
    plot(pos_all_xz(k_xz, 1), pos_all_xz(k_xz, 2), 'k-', 'LineWidth', 1.5); % Contorno negro
    
    xlabel('X [m]'); ylabel('Z [m]');
    title('Espacio de Trabajo - Plano XZ (Vista Lateral)');
    grid on; axis equal;
    hold off;
end