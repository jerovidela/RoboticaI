function Tinv = invHomog(T)
% invHomog (compatibilidad)
% Wrapper que delega en RoboticaUtils.invHomog para centralizar utilidades.
if exist('RoboticaUtils','class') ~= 8
    addpath(fullfile(fileparts(mfilename('fullpath')), 'helpers'));
end
Tinv = RoboticaUtils.invHomog(T);
end
