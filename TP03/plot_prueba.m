% plot_robot_frames.m — Plot del robot + sistemas de referencia seleccionados


% 1) Cargar definición del robot del TF
run('prueba.m');   % deja R, workspace y dh en memoria

% 2) Posición articular que querés analizar (puedes usar q_home del robot.m)
q = evalin('base','q_home');

% 3) Vector de booleanos para frames {0}..{n} (n = 6).
%    1 = dibujar, 0 = omitir. Aquí muestro todos.
sistemas = [1 1 1 1 1 1 1];  % {0} {1} {2} {3} {4} {5} {6}
assert(length(sistemas)==(R.n+1), 'La longitud de "sistemas" debe ser n+1.');

% 4) Plot del robot
figure('Name','Robot + Sistemas');
R.plot(q, 'workspace', workspace, 'scale', 0.8, 'jointdiam', 1.0, 'notiles');
hold on; grid on; axis equal;
title('Escaner6R - Frames DH seleccionados');

% 5) Dibujar los sistemas {0}..{n} a partir de los A_i(q)
%    Lo hacemos multiplicando Link.A(qi) acumulativamente (independiente de versión RTB)
T = R.base;
L = R.links;

% {0}
if sistemas(1)
    trplot(T, 'frame','0', 'length', 0.12, 'rgb', 'thick', 'arrow');
end

% {1}..{n}
for i = 1:R.n
    T = T * L(i).A(q(i));
    if sistemas(i+1)
        trplot(T, 'frame', num2str(i), 'length', 0.12, 'rgb', 'thick', 'arrow');
    end
end

% Opcional: dibujar también el frame de la TOOL (TCP)
T_tool = T * R.tool;
trplot(T_tool, 'frame', 'T', 'length', 0.12, 'rgb', 'thick', 'arrow');

view(135, 25);
