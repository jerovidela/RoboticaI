function esp_trab()
% ESP_TRAB - Calcula y grafica el espacio de trabajo del robot.
% Genera una representación de la superficie más precisa en 2D.
% Requiere que el robot esté definido en el workspace (ejecutar 'robot.m' antes).
    
    % === Cargar el robot del workspace ===
    R = evalin('base', 'R');
    
    % === Límites articulares ===
    n = R.n;
    qlims = zeros(n,2);
    for i = 1:n
        qlims(i,:) = R.links(i).qlim; % Copiar límites a una matriz
    end
    
    % === Generar configuraciones aleatorias de alta densidad ===
    % Aumentamos significativamente el número de puntos para delinear bien la superficie.
    numPoints = 50000; 
    qSamples = zeros(numPoints, n);
    for i = 1:n
        qmin = qlims(i,1); 
        qmax = qlims(i,2);
        qSamples(:,i) = qmin + (qmax - qmin) .* rand(numPoints, 1);
    end
    
    % === Calcular posiciones de la herramienta (end-effector) ===
    pos = zeros(numPoints, 3);
    for i = 1:numPoints
        T = R.fkine(qSamples(i,:));
        pos(i,:) = transl(T); % Obtener [x y z]
    end
    
    % --- Graficar la superficie de trabajo en planos 2D ---
    
    % === Superficie en el plano XY (Vista superior) ===
    figure('Name','Espacio de Trabajo - Vista Superior (XY)');
    hold on;
    % Calcular el envoltorio convexo para la nube de puntos en el plano XY.
    k_xy = convhull(pos(:,1), pos(:,2));
    % Graficar el contorno de la superficie.
    plot(pos(k_xy, 1), pos(k_xy, 2), 'b-', 'LineWidth', 2);
    xlabel('X [m]'); ylabel('Y [m]');
    title('Superficie de Trabajo - Plano XY');
    grid on; axis equal;
    hold off;
    
    % === Superficie en el plano XZ (Vista lateral) ===
    figure('Name','Espacio de Trabajo - Vista Lateral (XZ)');
    hold on;
    % Calcular el envoltorio convexo para la nube de puntos en el plano XZ.
    k_xz = convhull(pos(:,1), pos(:,3));
    % Graficar el contorno de la superficie.
    plot(pos(k_xz, 1), pos(k_xz, 3), 'b-', 'LineWidth', 2);
    xlabel('X [m]'); ylabel('Z [m]');
    title('Superficie de Trabajo - Plano XZ');
    grid on; axis equal;
    hold off;
    
end