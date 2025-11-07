function [Q, t, S, Sdot] = make_scan_segment_inv(R, T0, T1, vmax, amax, dt, q0)
% make_scan_segment_inv  Segmento recto de escaneo via IK por puntos
%
% Resumen:
% - Genera perfil LSPB cartesiano entre T0 y T1 (s, s_dot).
% - Para cada punto, resuelve IK (ikcon) y construye Q.
%
% Entradas:
%   R: SerialLink; T0,T1: SE3 (4x4); vmax (m/s); amax (m/s^2); dt (s)
%   q0: semilla de IK (1 x dof, opcional)
% Salidas:
%   Q: [N x dof] trayectoria articular
%   t: [N x 1] tiempos
%   S: [N x 1] avance acumulado (m)
%   Sdot: [N x 1] velocidad a lo largo del segmento (m/s)
% Genera una trayectoria articular para un segmento de escaneo recto.
% Este método calcula la trayectoria Cartesiana (LSPB) primero y luego
% resuelve la cinemática inversa (IK) para cada punto.
%
% R:    SerialLink
% T0:   Pose de inicio (4x4)
% T1:   Pose final (4x4)
% vmax: Velocidad crucero (m/s)
% amax: Aceleración (m/s^2)
% dt:   Paso de tiempo (s)
% q0:   Posición articular inicial (1xR.n) [para semilla de IK]

if nargin<7 || isempty(q0)
    q0 = zeros(1,R.n); % Semilla por defecto si no se provee
end

% --- 1. Geometría del Segmento
p0 = T0(1:3,4);     % Posición inicial [x;y;z]
p1 = T1(1:3,4);     % Posición final [x;y;z]
L = norm(p1-p0);    % Longitud total del segmento
d = (p1-p0)/max(L,eps); % Vector director unitario (en frame {0})
R_mat = T0(1:3,1:3);    % Orientación (constante)

% --- 2. Perfil de Movimiento (LSPB) para 's(t)'
% s(t) es la distancia recorrida a lo largo de la línea, de 0 a L
ta = vmax/amax; % Tiempo de aceleración
if L < vmax^2/amax
    % Caso triangular (no se alcanza vmax)
    ta = sqrt(L/amax);
    tc = 0;
    vmax = amax*ta; % vmax real es menor
else
    % Caso trapezoidal
    tc = (L - vmax^2/amax)/vmax; % Tiempo de crucero
end

Ttot = 2*ta + tc; % Tiempo total
t = (0:dt:Ttot)'; % Vector de tiempo
if t(end)<Ttot
    t=[t;Ttot]; % Asegura que el último punto esté incluido
end
N = numel(t);
S = zeros(N,1);
Sdot = zeros(N,1);

% Calcular s(t) y s_dot(t) para cada punto en el tiempo
for k=1:N
    tk=t(k);
    if tk<=ta
        % Fase de aceleración
        S(k)    = 0.5*amax*tk^2;
        Sdot(k) = amax*tk;
    elseif tk<=ta+tc
        % Fase de crucero
        S(k)    = 0.5*amax*ta^2 + vmax*(tk-ta);
        Sdot(k) = vmax;
    else
        % Fase de desaceleración
        tp    = Ttot - tk; % tiempo restante
        S(k)    = L - 0.5*amax*tp^2;
        Sdot(k) = amax*tp;
    end
end

% --- 3. Cálculo de IK en cada punto
Q = zeros(N, R.n);  % Pre-alocar memoria para las juntas
q_prev = q0;        % Usar q0 como la primera semilla

for k=1:N
    % Calcular la posición cartesiana deseada en este paso
    p_k = p0 + S(k)*d;
    
    % Ensamblar la pose cartesiana completa (posición + orientación)
    % rt2tr: "Rotation" and "translation" to "transform"
    T_k = rt2tr(R_mat, p_k);
    
    % Resolver la cinemática inversa para esta pose, usando la
    % solución anterior como semilla para continuidad
    q_k = R.ikcon(T_k, q_prev);
    
    % Guardar la solución y actualizar la semilla
    Q(k,:) = q_k;
    q_prev = q_k;
end

end
