function esp_trab()
% ESP_TRAB - Calcula y grafica el espacio de trabajo del robot.

    R = evalin('base', 'R');
    
    % === Límites articulares ===
    n = R.n;
    qlims = zeros(n,2);
    for i = 1:n
        qlims(i,:) = R.links(i).qlim; 
    end
    
    % === Generar configuraciones aleatorias de alta densidad ===
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
    
    figure('Name','Espacio de Trabajo - Vista Superior (XY)');
    hold on;
   
    k_xy = convhull(pos(:,1), pos(:,2));
   
    plot(pos(k_xy, 1), pos(k_xy, 2), 'b-', 'LineWidth', 2);
    xlabel('X [m]'); ylabel('Y [m]');
    title('Superficie de Trabajo - Plano XY');
    grid on; axis equal;
    hold off;
    
  
    figure('Name','Espacio de Trabajo - Vista Lateral (XZ)');
    hold on;
  
    k_xz = convhull(pos(:,1), pos(:,3));
    
    plot(pos(k_xz, 1), pos(k_xz, 3), 'b-', 'LineWidth', 2);
    xlabel('X [m]'); ylabel('Z [m]');
    title('Superficie de Trabajo - Plano XZ');
    grid on; axis equal;
    hold off;
    
end