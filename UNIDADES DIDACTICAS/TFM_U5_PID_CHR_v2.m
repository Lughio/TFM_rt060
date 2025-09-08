clc;
clear;
close all;
% ========================================================================
%  UNIDAD 5: CONTROL PID SINTONIZACIÓN CHR
%  Este script implementa un control de un PID mediante sintonizacion CHR:
%  1. Mediante la prueba y error se establecen los valores del PID
%  2. Mediante el calculo de la Unidad 1 se puede establecer un valor de
%  conversión aproximado
%  3. El modelo grafica en tiempo real la posición de la maqueta y la potencia empleada.
% =======================================================================================

%% PASO 1: INICIALIZACIÓN DE VARIABLES Y PARÁMETROS

% --- CARGA DE LIBRERIA---
%ESTE DRIVER EMPLEADO REQUIERE UNA INSTALACIÓN PREVIA
%SE EMPLEA ESTE DRIVER SOBRE OTROS CON MAS OPCIONES POR LA COMPATIBILIDAD CON SOFTWARE DE 64BITS%
%OTROS DRIVERS TIENEN PROBLEMAS CON EL SOFTWARE DE 64 BITS

% --- PARÁMETROS DE PLANTA CALCULADOS EXPERIMENTALMENTE---
K = 0.500;   % GANANCIA DE LA PLANTA [ (cm/s) / V ] 
T = 0.9;     % CONSTANTE DE TIEMPO (Tb) [s]
Tt = 1.05;    % TIEMPO MUERTO (Te) [s]

% --- CÁLCULO DE PARÁMETROS PID USANDO LAS FÓRMULAS CHR ---
Kp = (0.6 * T) / (K * (Tt + 1e-9));
Ti = T;
Td = 0.5 * (Tt);

fprintf('--- Parámetros del Controlador PID (Calculados con CHR) ---\n');
fprintf('Kp = %.3f\n', Kp);
fprintf('Ti = %.3f s\n', Ti);
fprintf('Td = %.3f s\n', Td);
fprintf('----------------------------------------------------------\n');

dt = 0.01;   % paso de integración [s]

% --- Consigna inicial ---
consigna_cm = 3.5; 

% --- Inicialización de Variables---
integral_error = 0;
posicion_anterior_cm = 0;
idnum = -1;
demo = 0;
canal = 0;
gain_conversion = 7.36;
gain_direction = 0.2;
bias_direction = 1;
bias_motor = 0.9;

% --- Preparación de Gráfica en Tiempo Real ---
figureHandle = figure('Name', 'Respuesta PID con Sintonización CHR', 'NumberTitle', 'off');

%CUADRO DE TEXTO DE CONSIGNA
uicontrol("Style","text",'Units','normalized',"Position",[0.01 0.97 0.3 0.03],'FontWeight','bold','String','INTRODUCE CONSIGNA:','Parent',figureHandle);
% INICIALIZACION DEL VALOR DE CONSIGNA
variable_consigna = uicontrol('Style','edit','Units','normalized','Position',[0.30 0.97 0.20 0.04],'String',num2str(consigna_cm),'Parent',figureHandle);
% BOTON DE ACTUALIZACION DE CONSIGNA
uicontrol('Style','pushbutton','Units','normalized','Position',[0.80 0.97 0.2 0.04],'FontWeight','bold','String','ACTUALIZAR','BackgroundColor',[0 1 0],'Callback',@(src,event) actualizarConsigna(variable_consigna,figureHandle),'Parent',figureHandle);
% VISUALIZACION ACTUAL DEL VALOR DE CONSIGNA
uicontrol('Style','text','Units','normalized','Position',[0.55 0.97 0.20 0.03],'Tag','valorActual','FontWeight','bold','ForegroundColor',[1 0 0],'String',['VALOR ACTUAL: ',num2str(consigna_cm)],'Parent',figureHandle);

%GRAFICAS
subplot(2,2,[1 2]);
hPosicion = plot(NaN, NaN, 'b-', 'LineWidth',2); hold on;
hConsigna = plot(NaN, NaN, 'r--'); grid on;
title('Posición'); ylabel('cm'); legend('Posición Real', 'Consigna');

subplot(2,2,3);
hControl = plot(NaN, NaN, 'k-'); grid on; title('Control Motor'); ylabel('Potencia (%)');

subplot(2,2,4);
hError = plot(NaN,NaN); grid on; title('Error'); ylabel('cm');

disp('Iniciando bucle de control...');
pause(1);

%BUCLE DE CONTROL
try
    tic; 
    while ishandle(figureHandle)  %CUANDO SE CIERRA SE DETIENE
        tiempo_iteracion_inicio = toc;

        % LEE EL VALOR DE LA INTERFAZ GRAFICA
        variable_consigna_cm = str2double(get(variable_consigna,'String'));
        if isnan(variable_consigna_cm)
            variable_consigna_cm = consigna_cm; % POR SI HAY ALGUN ERROR
        end
        %AJUSTE DE LA VARIABLE
        variable_consigna_cm = max(0.0, min(30.0, variable_consigna_cm));

        % SE LEE EL VOLTAJE
        [voltaje_actual, ~, errorCode, idnum] = EAnalogIn(idnum, demo, canal, 0);
        if errorCode ~= 0
            disp(['Error en EAnalogIn: ' num2str(errorCode)]);
            break;
        end

        posicion_actual_cm = gain_conversion * voltaje_actual + 0.22;
        posicion_actual_cm = max(0, min(30.0, posicion_actual_cm));

        if posicion_anterior_cm == 0
            posicion_anterior_cm = posicion_actual_cm;
        end

        % --- Calcular Error ---
        error_actual = variable_consigna_cm - posicion_actual_cm;

        % --- PID ---
        P = Kp * error_actual;
        integral_error_temp = integral_error + (error_actual * dt);
        I = 0;
        if Ti > 0
            I = (Kp / Ti) * integral_error_temp;
        end
        derivada_pos = (posicion_actual_cm - posicion_anterior_cm) / dt;
        D = - (Kp * Td) * derivada_pos;
        u_esfuerzo = P + I + D;

        % CONTROL DE SOBRESATURACION
        u_saturado = max(-100, min(100, u_esfuerzo));
        if u_esfuerzo ~= u_saturado && Ti > 0
             integral_error = (u_saturado - P - D) / (Kp / Ti);
        else
             integral_error = integral_error_temp;
        end

        % LOGICA DE CONTROL
        control_senal = ((u_esfuerzo - 0.22) / gain_conversion)*100 / 5.0;
        control_senal = max(-100, min(100, control_senal));
        voltaje_salida_motor = (control_senal) * 5.00 / 100;
        voltaje_salida_motor = max(-5, min(5, voltaje_salida_motor));

        [~, idnum] = EDigitalOut(idnum, demo, 1, 0, gain_direction * voltaje_salida_motor + bias_direction);
        [errorCode, idnum] = EAnalogOut(idnum, demo, abs(voltaje_salida_motor), 0.0);
        if errorCode ~= 0
            error('Error en eDAC: %d', errorCode);
        end
        [~, idnum] = EDigitalOut(idnum, demo, canal, 1, bias_motor + abs(voltaje_salida_motor));

        %ACTUALIZACIÓN GRAFICA
        posicion_anterior_cm = posicion_actual_cm; 
        t_now = toc;
        set(hConsigna, 'XData', [get(hConsigna, 'XData') t_now], 'YData', [get(hConsigna, 'YData') variable_consigna_cm]);
        set(hPosicion, 'XData', [get(hPosicion, 'XData') t_now], 'YData', [get(hPosicion, 'YData') posicion_actual_cm]);
        set(hControl, 'XData', [get(hControl, 'XData') t_now], 'YData', [get(hControl, 'YData') control_senal]);
        set(hError, 'XData', [get(hError, 'XData') t_now], 'YData', [get(hError, 'YData') error_actual]);
        drawnow limitrate;

        % MANTENER PERIODO 
        pause(max(0, dt - (toc - tiempo_iteracion_inicio)));
    end

catch e
    disp('¡DETENCIÓN DE EMERGENCIA O ERROR!');
    rethrow(e)
end

%%  APAGADO SEGURO
disp('Apagando el motor...');
try
    [errorCode, ~] = EAnalogOut(idnum, demo, 0.0, 0.0);
    if errorCode ~= 0
        error('Error en eDAC: %d', errorCode);
    end
catch
    disp('Error durante el apagado seguro.');
end

%% FUNCION DE ACTUALIZACIÓN 
function actualizarConsigna(variable_consigna,f)
    valor = str2double(get(variable_consigna,'String'));
    if isnan(valor)
        errordlg('Introduce un número válido','Error');
    else
        % Actualizar cuadro de visualización
        valorBox = findobj(f,'Tag','valorActual');
        valor = max(0.0, min(30.0, valor));
        set(variable_consigna,'String',num2str(valor));
        set(valorBox,'String',['VALOR ACTUAL: ',num2str(valor)]);
    end
end
