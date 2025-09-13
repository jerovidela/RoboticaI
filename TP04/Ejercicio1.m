%% TP4 - Ejercicio 1 
% Cinemática directa del FANUC Paint Mate 200iA
% usando TRANSFORMACIONES ELEMENTALES (trotz, transl, trotx)
% y verificación con SerialLink.fkine()

clc; clear; close all
fprintf('TP4 - Ej.1: CD con transformaciones elementales + verificación fkine\n');

% ---------- DH del enunciado (estándar) ----------
%    i :   d       a       alpha
DHconst = [ 0.45   0.075   -pi/2;   % 1
            0      0.300    0;      % 2
            0      0.075   -pi/2;   % 3
            0.32   0        pi/2;   % 4
            0      0       -pi/2;   % 5
            0.008  0        0];     % 6

% ---------- Transformación elemental A_i ----------
Ai = @(th,d,a,al) trotz(th) * transl(0,0,d) * transl(a,0,0) * trotx(al);

% ---------- Vectores articulares ----------
q1 = [0,          0,          0,      0,      0,      0];
q2 = [pi/4,      -pi/2,       0,      0,      0,      0];
q3 = [pi/5,   -2*pi/5,   -pi/10,   pi/2,  3*pi/10,  -pi/2];
q4 = [-0.61,     -0.15,     -0.30,  1.40,   1.90,   -1.40];

Q      = {q1,q2,q3,q4};
Qnames = {'q1','q2','q3','q4'};

% ---------- Robot RTB para verificación ----------
% (mismo DH pasado al formato de SerialLink: [theta d a alpha sigma])
dh_rtb = [ zeros(6,1), DHconst(:,1), DHconst(:,2), DHconst(:,3), zeros(6,1) ];
Rrtb   = SerialLink(dh_rtb, 'name', 'PaintMate200iA');

% ---------- (opcional) Subplots como en el ejemplo del TP ----------
figure('Name','Posturas q1..q4')
for k = 1:numel(Q)
    subplot(2,2,k), Rrtb.plot(Q{k}); campos([10 15 10]); hold on
    trplot(eye(4),'length',1.5,'frame','0')
    title(Qnames{k})
end

% ---------- Bucle principal: CD elemental + verificación ----------
for k = 1:numel(Q)
    q = Q{k};

    % CD por transformaciones elementales
    T = eye(4);
    for i = 1:6
        d = DHconst(i,1); a = DHconst(i,2); al = DHconst(i,3);
        T = T * Ai(q(i), d, a, al);
    end
    T_elem = T;                         %% >>> CAMBIO (nombrar explícitamente T_elem)

    % Salida formateada
    fprintf('\n%s = [% .4f % .4f % .4f % .4f % .4f % .4f]\n', Qnames{k}, q);
    fprintf('T_elem ({}^{0}T_{6}) =\n'), disp(T_elem)
    p = T_elem(1:3,4);  R = T_elem(1:3,1:3);
    fprintf('  p = [%.4f  %.4f  %.4f]^T (m)\n', p);
    fprintf('  ||R^T R - I||_F = %.2e\n', norm(R.'*R - eye(3), 'fro'));

    % -------- Verificación correcta con fkine --------
    T_fk = Rrtb.fkine(q).T;             %% >>> CAMBIO (antes se comparaba con I)
    dif  = norm(T_elem - T_fk, 'fro');  %% >>> CAMBIO (métrica correcta)
    fprintf('[check fkine] %s, ||T_elem - T_fk||_F = %.3e\n', Qnames{k}, dif);
end