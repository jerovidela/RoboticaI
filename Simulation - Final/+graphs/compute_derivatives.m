function dy = compute_derivatives(t, y, order, win_sec, poly_order)
% compute_derivatives  Derivada temporal robusta (y' o y'') con suavizado
%
% Resumen:
% - Aplica, si disponible, filtro de Savitzky-Golay para suavizar.
% - Usa diferencias centrales considerando t no uniforme.
%
% Entradas:
%   t: [N x 1] tiempo (s)
%   y: [N x D] señal
%   order: 1 (primera derivada) o 2 (segunda)
%   win_sec: ventana SG en segundos (opcional, default 0.35)
%   poly_order: grado del SG (opcional, default 3)
% Salidas:
%   dy: [N x D] derivada temporal estimada
% Derivada robusta: SG para suavizar -> diferencia central (t no uniforme).
% y: [N x D], t: [N x 1]; order: 1 (vel) o 2 (acel)
if nargin < 3 || isempty(order),      order = 1;      end
if nargin < 4 || isempty(win_sec),    win_sec = 0.35; end
if nargin < 5 || isempty(poly_order), poly_order = 3; end

N = size(y,1); D = size(y,2);
dy = zeros(N,D);
if N < 3, return; end

dt_mean = mean(diff(t));
frame   = max(3, 2*floor((win_sec/dt_mean)/2)+1);   % impar
frame   = min(frame, 2*floor((N-1)/2)+1);           % no mayor que N

% 1) Suavizado SG (si existe), si no, pasa directo
if exist('sgolayfilt','file') == 2
    ys = sgolayfilt(y, poly_order, frame);
else
    ys = y;
end

% 2) Derivada 1: diferencia central con t no uniforme
d1 = zeros(N,D);
d1(1,:)   = (ys(2,:)   - ys(1,:)  ) ./ max(t(2)-t(1),eps);
d1(end,:) = (ys(end,:) - ys(end-1,:)) ./ max(t(end)-t(end-1),eps);
for k = 2:N-1
    d1(k,:) = (ys(k+1,:) - ys(k-1,:)) ./ max(t(k+1)-t(k-1),eps);
end

if order == 1
    dy = d1;
    return
end

% 3) Derivada 2: opcional re-suavizado y otra central
if exist('sgolayfilt','file') == 2
    d1s = sgolayfilt(d1, poly_order, frame);
else
    d1s = d1;
end
d2 = zeros(N,D);
d2(1,:)   = (d1s(2,:)   - d1s(1,:)  ) ./ max(t(2)-t(1),eps);
d2(end,:) = (d1s(end,:) - d1s(end-1,:)) ./ max(t(end)-t(end-1),eps);
for k = 2:N-1
    d2(k,:) = (d1s(k+1,:) - d1s(k-1,:)) ./ max(t(k+1)-t(k-1),eps);
end
dy = d2;
end


% function dy = compute_derivatives(t, y, order, win_sec, poly_order)
% Derivada temporal robusta: Savitzky–Golay si hay toolbox, sino central.
% y: [N x D], t: [N x 1]
% order: 1 -> 1ra deriv (vel), 2 -> 2da (acc)
% win_sec: ventana en segundos (ej 0.35), poly_order: grado (ej 3)
%if nargin < 3 || isempty(order),      order = 1;       end
%if nargin < 4 || isempty(win_sec),    win_sec = 0.35;  end
%if nargin < 5 || isempty(poly_order), poly_order = 3;  end
%
%N = size(y,1); D = size(y,2);
%dy = zeros(N,D);
%if N < 3, return; end
%
%dt_mean = mean(diff(t));
%frame = max(3, 2*floor((win_sec/dt_mean)/2)+1);  % impar
%
%use_sg = exist('sgolayfilt','file') == 2;
%
%if use_sg
%    % sgolayfilt admite derivadas con (order, Ts)
%    for d = 1:D
%       dy(:,d) = sgolayfilt(y(:,d), poly_order, frame, order, dt_mean);
%    end
%else
%    % Fallback: diferencias (forward/backward/extremos, central interior)
%    if order == 1
%        dy(1,:)   = (y(2,:)   - y(1,:)  ) / max(t(2)-t(1),eps);
%        dy(end,:) = (y(end,:) - y(end-1,:)) / max(t(end)-t(end-1),eps);
%        for k=2:N-1
%            dy(k,:) = (y(k+1,:) - y(k-1,:)) / max(t(k+1)-t(k-1),eps);
%        end
%    else
%        % 2da derivada: derivar la 1ra con el mismo esquema
%        y1 = compute_derivatives(t, y, 1);
%        dy = compute_derivatives(t, y1, 1);
%    end
%end
%end
