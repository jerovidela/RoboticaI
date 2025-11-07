%% TP4 - Ejercicio 2: SCARA ABB IRB-910SC (comparación y validación)
% - Compara la DH del enunciado vs una DH SCARA típica (R-R-P-R).
% - Verifica T por transformaciones elementales vs fkine().
% - Valida A_i enlace-a-enlace con R.links(i).A(q(i)).
% - Dibuja posturas corrigiendo el error de qlim/workspace del plot.

clc; clear; close all
fprintf('=== TP4 - Ej.2: SCARA ABB IRB-910SC ===\n');

%% ---------------- (A) Código de ejemplo del enunciado ----------------
% Formato RTB: [theta  d  a  alpha  sigma], sigma=0 rotacional, sigma=1 prismática
% OJO: el enunciado usa alpha3 = pi (invierte el eje z del 3er eslabón)
DH_demo = [ ...
    0.000   0.195   0.300   0.000   0;   % R
    0.000   0.000   0.250   0.000   0;   % R
    0.000   0.000   0.000   pi      1;   % P (alpha=pi)
    0.000   0.000   0.000   0.000   0];  % R

Rdemo = SerialLink(DH_demo, 'name', 'SCARA_demo');

q_demo = [0 0 0 0];             % q = [q1 q2 d3 q4], aquí d3=0
T_demo = Rdemo.fkine(q_demo).T;
fprintf('\n(A) T_demo (enunciado) con q=[0 0 0 0]:\n'); disp(T_demo)

%% ------- (B) Nuestra DH "propia" coherente con la teoría del SCARA ------
% Variante 0.55 m -> a1=0.300, a2=0.250; prismática d3 a lo largo de +z
% (Si tu robot es de 0.45 m o 0.65 m, cambia a1 por 0.200 o 0.400)
a1 = 0.300; 
a2 = 0.250;  

DH_ours = [ ...
    0     0      a1   0   0;   % R  (theta1 variable)
    0     0      a2   0   0;   % R  (theta2 variable)
    0     0      0    0   1;   % P  (d3 variable)
    0     0      0    0   0];  % R  (theta4 variable)

Rours = SerialLink(DH_ours, 'name', 'SCARA_ours');

% Conjunto de posturas para comparar
Q     = { [0 0 0 0], [pi/6 -pi/3 -0.08 pi/4], [pi/4 pi/6 -0.15 -pi/2] };
names = {'q0','qA','qB'};

% Transformación elemental DH estándar: Rz(th)*Tz(d)*Tx(a)*Rx(al)
Ai = @(th,d,a,al) trotz(th) * transl(0,0,d) * transl(a,0,0) * trotx(al);

fprintf('\n(B) Comparación T (fkine vs elementales) y contra el demo:\n');
for k = 1:numel(Q)
    q = Q{k};   % [q1 q2 d3 q4]

    % --- fkine con nuestra DH ---
    T_fk = Rours.fkine(q).T;

    % --- fkine con DH del enunciado (mismos q) ---
    % NOTA: puede diferir por alpha3=pi en DH_demo
    T_demo_q = Rdemo.fkine(q).T;

    % --- CD por transformaciones elementales (nuestra DH) ---
    T_elem = eye(4);
    for i = 1:4
        sigma = DH_ours(i,5);
        a  = DH_ours(i,3);
        al = DH_ours(i,4);

        if sigma == 0          % rotacional: theta variable, d fijo
            th = q(i);
            d  = DH_ours(i,2);
        else                   % prismática: theta fijo (=0), d variable (=q(i))
            th = 0;
            d  = DH_ours(i,2) + q(i);
        end

        T_elem = T_elem * Ai(th,d,a,al);
    end

    fprintf('\n%s) q = [% .3f % .3f % .3f % .3f]\n', names{k}, q);
    fprintf('   T_ours (fkine) =\n');  disp(T_fk)
    fprintf('   T_elem (elementales) =\n'); disp(T_elem)
    fprintf('   ||T_ours - T_elem||_F = %.3e  (debe ser ~1e-12)\n', norm(T_fk - T_elem,'fro'))
    fprintf('   T_demo (enunciado) =\n'); disp(T_demo_q)
    fprintf('   ||T_ours - T_demo||_F = %.3e  (puede ser grande por alpha3=pi)\n', norm(T_fk - T_demo_q,'fro'))
end

%% ------ (C) Validación enlace-a-enlace con R.links(i).A(q(i)) ------
% IMPORTANTE:
% R.links(i).A(q) devuelve un SE3. Para componer con doubles usamos .T (4x4).
% Eso evita el error "operands to * cannot be composed".
fprintf('\n(C) Validación de cada A_i con R.links(i).A(q(i)):\n');

qtest = [pi/6, -pi/4, -0.10, pi/3];  % [q1 q2 d3 q4]
fprintf('   qtest = [% .3f % .3f % .3f % .3f]\n', qtest);

% -- Acumulado por RTB (con matrices 4x4 "double") --
Aacc_rtb = eye(4);
for i = 1:4
    % La clase Link gestiona internamente si es R o P; simplemente paso q(i)
    Ai_rtb = Rours.links(i).A(qtest(i)).T;   % convertir SE3 -> 4x4 double
    Aacc_rtb = Aacc_rtb * Ai_rtb;
end

% -- Acumulado por transformaciones elementales (nuestra DH) --
Aacc_ele = eye(4);
for i = 1:4
    sigma = DH_ours(i,5);
    a  = DH_ours(i,3);
    al = DH_ours(i,4);

    if sigma == 0
        th = qtest(i); d = DH_ours(i,2);
    else
        th = 0;          d = DH_ours(i,2) + qtest(i);
    end

    Aacc_ele = Aacc_ele * Ai(th,d,a,al);
end

fprintf('   ||Aacc_rtb - Aacc_ele||_F = %.3e  (debe ser ~1e-12)\n', norm(Aacc_rtb - Aacc_ele,'fro'));


%% --------------- (D) Gráfico de posturas (opcional) ----------------
% Robot de DIBUJO (no toca Rours de cálculo)
Rplot = SerialLink(Rours, 'name', 'SCARA_plot');

% La prismática NO puede tener q<0 para plot -> límites no negativos
Rplot.qlim = [ -pi   pi;        % q1
               -pi   pi;        % q2
                0    0.25;      % d3 (m)  AJUSTÁ si tu SCARA tiene más carrera
               -pi   pi ];      % q4

% Offset SOLO para lo que recibe el plot (si algún d3 de Q es negativo)
q3vals = cellfun(@(v) v(3), Q);
off3   = max(0, -min(q3vals)) + 1e-3;   % ej: si min(d3)=-0.15 -> off3~0.151

% Caja de trabajo amplia (m) y ventana grande
ws = [-0.9 0.9  -0.9 0.9  -0.1 0.7];
figure('Name','SCARA - Posturas (nuestra DH)','Color','w','Position',[100 100 1200 420])

for k = 1:numel(Q)
    subplot(1,numel(Q),k)
    hold on

    % q que se le manda al plot (la prismática queda >= 0)
    q_plot    = Q{k};
    q_plot(3) = q_plot(3) + off3;

    % Dibujo: más grande, sin sombras, sin nombre
    Rplot.plot(q_plot, ...
        'workspace', ws, ...
        'scale', 1.35, ...         % <<< más grande
        'jointdiam', 0.05, ...     % diámetro de juntas (estético)
        'noshadow', 'noname');

    % Ejes de referencia y ajustes de cámara/estética
    trplot(eye(4),'length',0.25,'frame','0','color','k','thick',1.5)
    axis equal vis3d
    grid on
    camup([0 0 1])                 % Z hacia arriba
    campos([0.6 0.6 0.45])         % posición de cámara (x,y,z) en metros aprox.
    lighting gouraud

    title(names{k},'FontSize',12,'FontWeight','bold')
end