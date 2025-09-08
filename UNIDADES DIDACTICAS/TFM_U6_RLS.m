
function [Numerador,Denominador,Matriz_P,Vector_X] = TFM_U6_RLS(Entrada_sistema,Salida_sistema,Numerador,Denominador,Matriz_P,Vector_X,Factor_olvido)
% RLS para identificar un modelo de 2º orden: G(z) = b0*z^-1 / (1 + a1*z^-1 + a2*z^-2)
% La ecuación en diferencias es: y(k) = -a1*y(k-1) - a2*y(k-2) + b0*u(k-1)

if ~exist('Factor_olvido','var')
    Factor_olvido=0.995; % Un valor más conservador (lento pero estable) es mejor para sistemas reales
end

% Vector de parámetros a estimar: Theta = [b0; a1; a2]
Theta=[Numerador; Denominador(2:end)']; % Asumimos Numerador = [0 b0]

% El vector X (regresor) se forma como: [u(k-1), -y(k-1), -y(k-2)]
Vector_X(1) = Entrada_sistema; % Este es u[k-1]

Matriz_K = (Matriz_P*Vector_X)/(Factor_olvido+Vector_X'*Matriz_P*Vector_X);
Error = Salida_sistema - Vector_X'*Theta;
Theta = Theta + Matriz_K*Error;
Theta(abs(Theta)<1e-5) = 0;

Numerador = Theta(1);
Denominador = [1 Theta(2:3)'];

Matriz_P = (1/Factor_olvido)*(Matriz_P - (Matriz_K*Vector_X'*Matriz_P));

% Actualizar el regresor con la salida actual para la siguiente iteración
Vector_X(3) = Vector_X(2);
Vector_X(2) = -Salida_sistema;
end
