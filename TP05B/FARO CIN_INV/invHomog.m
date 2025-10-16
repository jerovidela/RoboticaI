function Tinv = invHomog(T)

    if isa(T,'SE3')
        M = T.T;                    % extrae 4x4
    elseif isstruct(T) && isfield(T,'T')
        M = T.T;                    % extrae 4x4
    else
        M = T;                      % asumimos double(4x4)
    end

    R = M(1:3,1:3);
    p = M(1:3,4);
    Rt = R.';
    Tinv = [Rt, -Rt*p; 0 0 0 1];

end
% INVHOMOG
% Función auxiliar que calcula la INVERSA de una matriz homogénea.
% Una matriz homogénea combina ROTACIÓN y TRASLACIÓN en 3D.
% Esta función "da vuelta" esa transformación.
% Se usa dentro de cin_inv_Faro para simplificar las cuentas
% de transformaciones (por ejemplo, pasar de base a herramienta).
