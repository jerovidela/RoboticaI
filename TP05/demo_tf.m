
clear; clc; close all;

% 1) Cargar robot
run('robot.m');                      % crea R
q0 = zeros(1, R.n);

% 2) Objetivo con solución “segura” (CD de una q dentro de límites)
q_des = ((R.qlim(:,1) + R.qlim(:,2))/2).';     % centro de cada eje
Tgoal = R.fkine(q_des); if isa(Tgoal,'SE3'), Tgoal = Tgoal.T; end

% 3) Resolver CI (analítico) y, si falla, DLS directo
q_mejor = true;
try
    Qbest = cin_inv_Faro(R, Tgoal, q0, q_mejor);    % intenta Pieper (+fallback interno)
catch
    % Si cin_inv_Faro no pudo caer al DLS, lo llamo directo acá:
    opts.mask = [1 1 1 0.7 0.7 0.7];
    Qbest = tf_ik_dls(R, Tgoal, q_mejor, q0, opts);
end

% 4) (Opcional) todas las soluciones analíticas si existieran
try
    Qall = cin_inv_Faro(R, Tgoal, q0, false);
catch
    Qall = Qbest;   % al menos la del DLS
end

% 5) Verificación + visual
Tbest = R.fkine(Qbest); if isa(Tbest,'SE3'), Tbest = Tbest.T; end
err = norm(tr2delta(Tbest, Tgoal));
fprintf('Qbest:\n'); disp(Qbest);
fprintf('Error twist: %.3e\n', err);

figure('Color','w'); grid on; axis equal; view(135,25);
R.plot(q0,'notiles','noname','scale',0.7); hold on;
trplot(Tgoal,'frame','goal','length',0.12,'color','k');
R.plot(Qbest,'notiles','noname');
title(sprintf('TF — Mejor solución | err = %.2e', err));
% DEMO_TF
% Script de prueba para la CINEMÁTICA INVERSA.
%
% Pasos que hace:
%   1) Carga el robot (R) definido en robot.m
%   2) Define un objetivo Tgoal (posición + orientación deseada)
%   3) Llama a cin_inv_Faro (y opcionalmente tf_ik / tf_ik_dls)
%   4) Muestra en consola las soluciones articulares encontradas
%   5) Grafica el robot en la postura solución
% En pocas palabras:
%   Permite probar de manera rápida la cinemática inversa del robot
%   sin tener que escribir comandos largos en la terminal.


