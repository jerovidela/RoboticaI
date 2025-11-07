function q_uturn = make_uturn_segment(R, Twaypoints, q0, V, dt, tacc)
% make_uturn_segment  Genera maniobra de retorno (U-turn) fuera de la placa
%
% Resumen:
% - Resuelve IK en pocos waypoints (levantar, cruzar, bajar).
% - Usa mstraj para suavizar entre waypoints con velocidades articulares V.
%
% Entradas:
%   R: SerialLink
%   Twaypoints: 4x4xM poses clave para el giro
%   q0: junta inicial (1 x dof)
%   V: velocidades por junta [1 x dof] (rad/s)
%   dt: paso temporal (s), tacc: tiempo de blending (s)
% Salidas:
%   q_uturn: [K x dof] trayectoria articular del U-turn
% U-turn fuera de la placa: IK en pocos waypoints y mstraj para suavizar.
% - Twaypoints: 4x4xM (levantar, cruzar, bajar...)
% - q0: junta inicial (1x6) para continuidad
% - V: velocidades por junta [1x6] (rad/s)
% - dt: paso temporal, tacc: blend de aceleración (s)

    M = size(Twaypoints,3);
    Qw = zeros(M, R.n);
    q_prev = q0;
    for i = 1:M
        q_prev = R.ikcon(Twaypoints(:,:,i), q_prev);
        Qw(i,:) = q_prev;
    end

    % mstraj: arranca desde q0 y sigue los waypoints con blend
    q_uturn = mstraj(Qw, V, [], q0, dt, tacc);

    % Quita duplicado del primer punto si coincide con q0
    if norm(q_uturn(1,:) - q0) < 1e-12
        q_uturn = q_uturn(2:end,:);
    end
end
