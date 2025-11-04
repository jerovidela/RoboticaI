function Q = cin_inv_Faro(R, T, q0, mejor)
% cin_inv_Faro (compatibilidad)
% Wrapper que delega en RoboticaUtils.ikFaro para centralizar utilidades.
if exist('RoboticaUtils','class') ~= 8
    addpath(fullfile(fileparts(mfilename('fullpath')), 'helpers'));
end
Q = RoboticaUtils.ikFaro(R, T, q0, mejor);
end
