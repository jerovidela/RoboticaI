clear; clc; close all;
workspace = [-0.9 0.9 -0.9 0.9 -0.1 1.6];
robot
q = zeros(1, R.n);
%q = [1,1,1,1,1,1];
ejes = ones(1, R.n);
%ejes = [1,0,0,0,0,1];
sistemas = [1, ones(1, R.n)];   % 1 = mostrar {0..n}; ej: [0 0 1 1 ...] para últimos
L = 0.12;                   % largo de los ejes dibujados

figure('Color','w'); grid on; axis equal;
view(135,25);
R.plot(q, 'notiles', 'scale',0.7, 'jointdiam',0.6);
R.teach();
hold on;

% ----- Sistema {0} (base) si se pide -----
if numel(sistemas) >= 1 && sistemas(1) == 1
    trplot(R.base, 'frame','0', 'length', L);
end

% ----- Recorrer y dibujar frames {1..n} -----
T = R.base;
for i = 1:R.n
    Ai = R.A(i, q);   % <<< IMPORTANTE: pasar q COMPLETO
    T  = T * Ai;

    % Mostrar frame {i} si lo pide "ejes" (máscara de articulaciones)
    if i <= numel(ejes) && ejes(i) == 1
        trplot(T, 'frame', num2str(i), 'length', L);
    end

    % Mostrar frame {i} si lo pide "sistemas" (máscara de {0..n})
    if i+1 <= numel(sistemas) && sistemas(i+1) == 1
        trplot(T, 'frame', num2str(i), 'length', L);
    end
end

% (Opcional) Mostrar el tool como {T}
trplot(T * R.tool, 'frame','T', 'length', L);
title('Frames seleccionados');

