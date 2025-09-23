clc; 
clear;
close all;

% ========================================================================
%  UNIDAD 3: COMPARATIVA DE CONTROL P, PI Y PD MEDIANTE SINTONIZCION
%  ESTANDAR
%  Este script implementa varios controladores para comprender como
%  afectan los diferentes valores de los coeficientes 
%  1. Se inicia siempre desde la posicion inicial de 0 cm
%  2. Se ejecuta cada modelo por separado en orden secuencial
%  3. Se realiza una prueba con un valor de coeficiente para probar su
%  resultado
%  4. Se grafica la prueba realizada permitiendo guardar una comparativa de
%  los valores asignados a cada prueba pues todos son dependientes de Kp
% =========================================================================

% --- CARGA DE LIBRERIA---
% ESTE DRIVER EMPLEADO REQUIERE UNA INSTALACION PREVIA
% SE EMPLEA ESTE DRIVER SOBRE OTROS CON MAS OPCIONES POR LA COMPATIBILIDAD
% CON SOFTWARE DE 64BITS
% OTROS DRIVERS TIENEN PROBLEMAS CON EL SOFTWARE DE 64 BITS

%% PASO 1: INICIALIZACION Y CONFIGURACION

% ---PARAMETROS DEL CONTROLADOR CHR---
Kp = 1.2;
Ti = 2.0;
Td = 1.0;

% --- PARAMETROS DE CONFIGURACION ---
tiempo_exp_individual = 20;  %SEGUNDOS POR PRUEBA
dt = 0.05;                   % PERIODO DE MUESTREO
consigna_cm = 20.0;
posicion_inicio_cm = -3.0;   % ES UN CERO LOGICO, LUEGO SE APLICA ESTA 
%CORRECION
offset_inicio = 3.0;         % CORRECCION DE OFFSET PARA ASEGURAR EL CERO
iniobjetivo = posicion_inicio_cm + offset_inicio;
idnum = -1;
demo = 0;
canal = 0;
gain_conversion = 7.36;   % cm por V aprox
gain_direction = 0.2;
bias_direction = 1;
bias_motor = 0.9;

% ---INTERFAZ GRAFICA---
figureHandle = figure('Name', 'Comparativa de Controladores P, PI, y PD', 'NumberTitle', 'off');
ax1 = subplot(3, 1, 1); title(ax1,'Respuesta del Controlador Proporcional (P)'); ylabel(ax1,'Posicion (cm)'); grid(ax1,'on'); hold(ax1,'on');
ax2 = subplot(3, 1, 2); title(ax2,'Respuesta del Controlador Proporcional-Integral (PI)'); ylabel(ax2,'Posicion (cm)'); grid(ax2,'on'); hold(ax2,'on');
ax3 = subplot(3, 1, 3); title(ax3,'Respuesta del Controlador Proporcional-Derivativo (PD)'); xlabel(ax3,'Tiempo (s)'); ylabel(ax3,'Posicion (cm)'); grid(ax3,'on'); hold(ax3,'on');

%% PASO 2: EJECUCION SECUENCIAL
try
    % INICIALIZACION DEL MOVIMIENTO
    disp('--- FASE DE INICIALIZACION ---');
    fprintf('Moviendo el carro a la posicion de inicio universal (%.2f cm)...\n', iniobjetivo);
    mover_posicion(iniobjetivo, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);
    disp('Sistema inicializado. Comenzando pruebas...');
    
    % --- Experimento 1: Controlador P ---
    disp('--- Iniciando Prueba del Controlador P ---');
    [hist_t_P, hist_x_P] = test_controlador('P', Kp, Ti, Td, tiempo_exp_individual, dt, consigna_cm, posicion_inicio_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);
    plot(ax1, hist_t_P, hist_x_P, 'b-', 'LineWidth', 2, 'DisplayName', 'Posicion Real');
    plot(ax1, [0 hist_t_P(end)], [consigna_cm consigna_cm], 'r--', 'LineWidth',1,'DisplayName', 'Consigna');
    legend(ax1,'show','Location','southeast');
    
    disp('Prueba P finalizada. Volviendo al inicio...');
    mover_posicion(iniobjetivo, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);

    % --- Experimento 2: Controlador PI ---
    disp('--- Iniciando Prueba del Controlador PI ---');
    [hist_t_PI, hist_x_PI] = test_controlador('PI', Kp, Ti, Td, tiempo_exp_individual, dt, consigna_cm, posicion_inicio_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);
    plot(ax2, hist_t_PI, hist_x_PI, 'b-', 'LineWidth', 2, 'DisplayName', 'Posicion Real');
    plot(ax2, [0 hist_t_PI(end)], [consigna_cm consigna_cm], 'r--', 'LineWidth',1,'DisplayName', 'Consigna');
    legend(ax2,'show','Location','southeast');

    disp('Prueba PI finalizada. Volviendo al inicio...');
    mover_posicion(iniobjetivo, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);

    % --- Experimento 3: Controlador PD ---
    disp('--- Iniciando Prueba del Controlador PD ---');
    [hist_t_PD, hist_x_PD] = test_controlador('PD', Kp, Ti, Td, tiempo_exp_individual, dt, consigna_cm, posicion_inicio_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);
    plot(ax3, hist_t_PD, hist_x_PD, 'b-', 'LineWidth', 2, 'DisplayName', 'Posicion Real');
    plot(ax3, [0 hist_t_PD(end)], [consigna_cm consigna_cm], 'r--', 'LineWidth',1,'DisplayName', 'Consigna');
    legend(ax3,'show','Location','southeast');
    
    disp('Prueba PD finalizada. Volviendo al inicio por ultima vez...');
    mover_posicion(iniobjetivo, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);

catch e
    disp('ERROR DURANTE LA EJECUCION DE LAS PRUEBAS!');
    rethrow(e)
end

%%  PASO 3: APAGADO SEGURO FINAL
disp('Apagando el motor...');

try
    % SE APAGA EL MOTOR PARA UN APAGADO SEGURO
    [errorCode, idnum] = EAnalogOut(idnum, demo, 0.0, 0.0);
    if exist('errorCode','var') && errorCode ~= 0
        warning('EAnalogOut devolvio codigo de error: %d', errorCode);
    end
    disp('Sistema detenido de forma segura.');
catch
    disp('Error durante el apagado seguro.');
end


%%  FUNCIONES AUXILIARES

function [hist_tiempo, hist_posicion] = test_controlador(tipo_control, Kp, Ti, Td, tiempo_exp, dt, consigna_cm, posicion_inicio_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor)
    % INICALIZACION
    num_pasos = floor(tiempo_exp / dt);
    integral_error = 0;
    posicion_anterior_cm = posicion_inicio_cm;
    hist_tiempo = zeros(1, num_pasos);
    hist_posicion = zeros(1, num_pasos);
    tic;
    
    % LIMITES DE CONTROL
    maxPercent = 100;
    minPercent = -100;
    
    for k = 1:num_pasos
        tiempo_iteracion_inicio = toc;
        % --- Leer sensor ---
        [voltaje_sensor, ~, errorCode, idnum] = EAnalogIn(idnum, demo, canal, 0);
        if exist('errorCode','var') && errorCode ~= 0
            warning('Error en EAnalogIn: %d. Interrumpiendo prueba.', errorCode);
            hist_tiempo = hist_tiempo(1:k-1);
            hist_posicion = hist_posicion(1:k-1);
            break;
        end
        
        % CONVERSION DE CM A VOLTIOS
        posicion_actual_cm = gain_conversion * voltaje_sensor + 0.22;
        posicion_actual_cm = max(0, min(30.0, posicion_actual_cm));
        
        %ERROR
        error_actual = consigna_cm - posicion_actual_cm;
        
        % PARAMETROS P, I , D
        P = Kp * error_actual;
        I = 0;
        D = 0;
        
        % PARA PI
        if strcmpi(tipo_control, 'PI')
            integral_error_temp = integral_error + error_actual * dt;
            if Ti > eps
                I = (Kp / Ti) * integral_error_temp;
            else
                I = 0;
            end
        end
        
        % PARA SOLO PD
        if strcmpi(tipo_control, 'PD')
            derivada_pos = (posicion_actual_cm - posicion_anterior_cm) / dt;
            D = - (Kp * Td) * derivada_pos;
        end
        
        % SEGUN PID
        if strcmpi(tipo_control,'PI')
            u_esfuerzo = P + I;
        elseif strcmpi(tipo_control,'PD')
            u_esfuerzo = P + D;
        else
            u_esfuerzo = P;
        end
        
        % CONVERTIR DE ESFUERZO A VOLTAJE
        % Interpretación: ((u - offset)/gain_conv) => fracción en V; convertimos a %
        control_senal_percent = ((u_esfuerzo - 0.22) / gain_conversion) * 100;   % en %
        control_senal_percent = max(minPercent, min(maxPercent, control_senal_percent));
        voltaje_salida_motor = (control_senal_percent / 100) * 5.0;  % en V, entre -5 y 5
        voltaje_salida_motor = max(-5, min(5, voltaje_salida_motor));
        
        %CONTROL CONTRA SATURACION
        if strcmpi(tipo_control,'PI')
            % RECALCULAR LA I
            I_check = (Kp / max(Ti, eps)) * integral_error_temp;
            u_check = P + I_check;
            control_percent_check = ((u_check - 0.22) / gain_conversion) * 100;
            if control_percent_check >= minPercent && control_percent_check <= maxPercent
                integral_error = integral_error_temp;
                I = I_check;
            else
               %NO SE ACTUALIZA EL ERROR INTEGRAL
            end
            u_esfuerzo = P + I;
        end
        %CONTROL DE LA RT060
        try
            %CONTROL DE DIRECCION SEGUN VOLTAJE
            [~, idnum] = EDigitalOut(idnum, demo, 1, 0, gain_direction * voltaje_salida_motor + bias_direction);
            [errorCodeOut, idnum] = EAnalogOut(idnum, demo, abs(voltaje_salida_motor), 0.0);
            if exist('errorCodeOut','var') && errorCodeOut ~= 0
                warning('EAnalogOut devolvió código de error: %d', errorCodeOut);
            end
            [~, idnum] = EDigitalOut(idnum, demo, canal, 1, bias_motor + abs(voltaje_salida_motor));
        catch hwErr
            warning('Error al escribir salidas HW: %s', hwErr.message);
        end
        
        % REGISTRO HISTORICO
        hist_tiempo(k) = toc;
        hist_posicion(k) = posicion_actual_cm;
        
        %GUARDAR EL ANTERIOR VALOR DE LA POSICION
        posicion_anterior_cm = posicion_actual_cm;
        
        %SE MANTIENE EL CICLO
        pause(max(0, dt - (toc - tiempo_iteracion_inicio)));
    end
    
    % SE APAGA EL MOTOR EN CUANTO FINALIZA 
    try
        [errStop, ~] = EAnalogOut(idnum, demo, 0.0, 0.0);
        if exist('errStop','var') && errStop ~= 0
            warning('Error al apagar EAnalogOut: %d', errStop);
        end
    catch
        % SI FALLA PUES SALTA AUTOMATICAMENTE
    end
end

function mover_posicion(posicion_objetivo_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor)
   %SE MUEVE EL CARRO A LA POSICION OBJETIVO
    fprintf('Moviendo a la posición objetivo (%.2f cm)... ', posicion_objetivo_cm);
    
    Kp_retorno = 0.8; 
    tolerancia_cm = 0.1;
    timeout_s = 40; % s
    
    tic;
    while toc < timeout_s
        [voltaje_sensor, ~, errorCode, idnum] = EAnalogIn(idnum, demo, canal, 0);
        if exist('errorCode','var') && errorCode ~= 0
            warning('Error en EAnalogIn durante posicionamiento: %d', errorCode);
            break;
        end
        posicion_actual_cm = gain_conversion * voltaje_sensor + 0.22;
        posicion_actual_cm = max(0, min(30.0, posicion_actual_cm));
        
        error_retorno = posicion_objetivo_cm - posicion_actual_cm;
        
        if abs(error_retorno) < tolerancia_cm
            fprintf('Posición alcanzada.\n');
            try, EAnalogOut(idnum, demo, 0.0, 0.0); end
            pause(0.5);
            return;
        end
        
        y_total = Kp_retorno * error_retorno;
        control_percent = ((y_total - 0.22) / gain_conversion) * 100;
        control_percent = max(-100, min(100, control_percent));
        voltaje_salida_motor = (control_percent / 100) * 5.0;
        voltaje_salida_motor = max(-5, min(5, voltaje_salida_motor));
        
        try
            [~, idnum] = EDigitalOut(idnum, demo, 1, 0, gain_direction * voltaje_salida_motor + bias_direction);
            [errOut, idnum] = EAnalogOut(idnum, demo, abs(voltaje_salida_motor), 0.0);
            if exist('errOut','var') && errOut ~= 0
                warning('EAnalogOut (go_to_position) devolvió código: %d', errOut);
            end
            [~, idnum] = EDigitalOut(idnum, demo, canal, 1, bias_motor + abs(voltaje_salida_motor));
        catch
            warning('Error al escribir salidas HW en go_to_position.');
        end
        
        pause(0.05);
    end
    
    fprintf('Tiempo de posicionamiento agotado.\n');
    try, EAnalogOut(idnum, demo, 0.0, 0.0); end
    pause(0.5);
end