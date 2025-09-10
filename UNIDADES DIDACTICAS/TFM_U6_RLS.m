function [Numerador,Denominador,Matriz_P,Vector_X] = TFM_U6_RLS(Entrada_sistema,Salida_sistema,Numerador,Denominador,Matriz_P,Vector_X,Factor_olvido)
% RLS PARA IDENTIFICAR UN MODELO DE 2º ORDEN: G(Z) = B0*Z^-1 / (1 + A1*Z^-1 + A2*Z^-2)
% LA ECUACION EN DIFERENCIAS ES: Y(K) = -A1*Y(K-1) - A2*Y(K-2) + B0*U(K-1)

% SI NO SE DEFINE EL FACTOR DE OLVIDO, SE USA UN VALOR POR DEFECTO
if ~exist('Factor_olvido','var')
    Factor_olvido = 0.995; % UN VALOR MÁS CONSERVADOR (LENTO PERO ESTABLE) ES MEJOR PARA SISTEMAS REALES
end

% VECTOR DE PARAMETROS A ESTIMAR: THETA = [B0; A1; A2]
Theta = [Numerador; Denominador(2:end)']; % SE ASUME NUMERADOR = [0 B0]

% EL VECTOR X (REGRESOR) SE FORMA COMO: [U(K-1), -Y(K-1), -Y(K-2)]
Vector_X(1) = Entrada_sistema; % ESTE ES U(K-1)

% CALCULO DE LA GANANCIA DE ADAPTACION
Matriz_K = (Matriz_P * Vector_X) / (Factor_olvido + Vector_X' * Matriz_P * Vector_X);

% CÁLCULO DEL ERROR DE PREDICCIÓN
Error = Salida_sistema - Vector_X' * Theta;

% ACTUALIZACIÓN DE PARÁMETROS
Theta = Theta + Matriz_K * Error;

% FILTRO PARA EVITAR PARÁMETROS DEMASIADO PEQUEÑOS (REDUCE RUIDO)
Theta(abs(Theta) < 1e-5) = 0;

% ACTUALIZAR NUMERADOR Y DENOMINADOR ESTIMADOS
Numerador = Theta(1);
Denominador = [1 Theta(2:3)'];

% SE ACTUALIZA  LA MATRIZ DE COVARIANZA
Matriz_P = (1/Factor_olvido) * (Matriz_P - (Matriz_K * Vector_X' * Matriz_P));

%SE ACTUALIZA EL REGRESOR CON LA SALIDA ACTUAL PARA LA SIGUIENTE ITERACIÓN
Vector_X(3) = Vector_X(2);
Vector_X(2) = -Salida_sistema;

end
