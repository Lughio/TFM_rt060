function [Num,Den,P_rls,X_rls]=TFM_U6_IRLS(Input,Output,Num,Den,P_rls,X_rls)
%Función que implementa el método de mínimos cuadrados recursivos
%Como parámetros de entrada usa 
%   Input: entrada al sistema en el instante actual
%   Output: salida del sistema en el instante actual
%   Num: Vector que contiene los valores actuales de los coeficientes del numerador de la función de transferencia a identificar
%   Den: Vector que contiene los valores actuales de los coeficientes del denominador de la función de transferencia a identificar
%   P_rls y X_rls: Parámetros internos que hay que reutilizar en cada llamada, la primera vez se pasará una matriz vacía [], en posteriores llamadas a la función se usarán los valores devueltos por iterative_RLS

Den=[1 Den];

if isempty(X_rls)
    order_N=length(Num);
    order_D=length(Den);
    % Crear los datos iniciales para la función RLS
    P_rls=diag(rand(1,order_N+order_D-1)*1000);
    X_rls=rand(order_N+order_D-1,1)/1000;
end

% Calculo el sistema identificado en esta iteración
    [Num,Den,P_rls,X_rls]=TFM_U6_RLS(Input,Output,Num,Den,P_rls,X_rls);
    Den=Den(2:end);
