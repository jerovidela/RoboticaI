function Q = cin_inv_Faro(R, T, q0, mejor)
offsets=R.offset;
R.offset=zeros(6,1);
    T = invHomog(R.base) * T * invHomog(R.tool); 

    % centro de la muñeca respecto de S0
    d6=R.links(6).d;  
    p  = T(1:3,4) -d6* T(1:3,3);  
   

    % como q1 rota toda la esctructura hasta la muñeca, no imoprta q2 ni
    % q3, por lo que basta solo con la proyeccion de p en el sistema S0
    q1a = atan2(p(2),p(1));
    if q1a>0, q1b=q1a-pi; else, q1b=q1a+pi; end
    q1=[q1a,q1b];

   soluciones = zeros(6,8); 
   soluciones(1,:)=[q1a q1a q1a q1a q1b q1b q1b q1b];
   idx=1;
    for i=1:length(q1) % Bucle para las 4 soluciones q2, 2 por cada q1
        
        L2 = R.links(2).a;   % a2 = 0.398
        L3 = R.links(3).a;   % a3 = 0.213
        
        % p respecto S1
        T1 = R.links(1).A(q1(i)).double;
        p_1 = invHomog(T1) * [p; 1]; 
        
        % 2. Proyección al plano de movimiento X1Y1
        r =sqrt(p_1(1)^2 + p_1(2)^2);
        if r < 1e-8
        warning('r muy pequeño: posible singularidad geométrica');
        continue;
        end
        B=atan2(p_1(2),p_1(1));
        G=acos((L2^2 + r^2 - L3^2) / (2 * r * L2));

        q2a=B-real(G);
        q2b=B+real(G);
        q2=[q2a,q2b];

       for k=1:2 
           for j=1:2
               soluciones(2,idx)=q2(k);
               T2=R.links(2).A(q2(k)).double;
               p_2 = invHomog(T2)*[p;1];  %p respecto de S2
               q3 = atan2(p_2(2),p_2(1));
               soluciones(3,idx)=q3;
               idx=idx+1;
           end 
       end
       
    end



%calculo de q4, q5 y q6
for i=1:2:7
    q1 = soluciones(1, i);
    q2 = soluciones(2, i);
    q3 = soluciones(3, i);
    [q4, q5, q6] = calcular_qm(R, q1, q2, q3, T, q0);
    soluciones(4:6,i:i+1) = [q4; q5; q6];
end

R.offset = offsets;
soluciones = soluciones - R.offset' * ones(1,8);

if mejor
    Qaux = soluciones  - q0' * ones(1,8);
    normas = zeros(1,8);
    for i=1:8
        normas(i) = norm(Qaux(:,i));
    end
    [~,pos] = min(normas);
    Q = soluciones(:, pos);
else
    Q = soluciones;
end
end

function [q4,q5,q6] = calcular_qm(R, q1, q2, q3, T, q0)
T1 = R.links(1).A(q1).double;
T2 = R.links(2).A(q2).double;
T3 = R.links(3).A(q3).double;

T36 = invHomog(T3) * invHomog(T2) * invHomog(T1) * T;
tol = 1e-6;
if abs(T36(3,3) - 1) < tol
    % solución degenerada:
    %   > z3 y z5 alineados (y z6)
    %   > q4 y q6 generan el mismo movimiento
    %   > q5 = 0 (o q5 = 180º)
    %   > se asume q4 = q4_anterior
    warning('Caso degenerado')
    q4(1) = q0(4);
    q5(1) = 0;
    q6(1) = atan2(T36(2,1), T36(1,1)) - q4(1);
    q4(2) = q4(1);
    q5(2) = 0;
    q6(2) = q6(1);
else
    % solución normal:
    q4(1) = atan2(T36(2,3), T36(1,3)); %atan2(-T36(2,3), -T36(1,3))
    if q4(1) > 0, q4(2) = q4(1) - pi; else, q4(2) = q4(1) + pi; end
    q5 = zeros(1,2);
    q6 = q5;
    for i=1:2
        T4 = R.links(4).A(q4(i)).double;
        T6 = invHomog(T4) * T36;
        q5(i) = atan2(T6(2,3), T6(1,3)) + pi/2;
        
        T5 = R.links(5).A(q5(i)).double;
        T6 = invHomog(T5) * T6;
        q6(i) = atan2(T6(2,1),T6(1,1));
    end
end
end

% CIN_INV_FARO
% Función principal de CINEMÁTICA INVERSA para el robot Faro.
% Recibe:
%   R   -> robot (SerialLink con parámetros DH)
%   T   -> matriz homogénea objetivo (posición + orientación)
%   q0  -> postura inicial (semilla)
%   mejor -> true  = devuelve la solución más cercana a q0
%            false = devuelve todas las soluciones
% Devuelve:
%   Q -> conjunto de ángulos articulares que llevan al efector a T
%
% En pocas palabras:
%   A partir de una posición/orientación deseada del extremo,
%   calcula los ángulos de las articulaciones que permiten alcanzarla.

