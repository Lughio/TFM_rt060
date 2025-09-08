clc;
clear;
close all;

% ========================================================================
%  UNIDAD 3: COMPARATIVA DE CONTROL P, PI Y PD MEDIANTE SINTONIZCION
%  ESTANDAR
%  Este script implementan varios controladores para comprender como
%  afectan los diferentes valores de los coeficientes 
%  1. Se inicia siempre desde la posición inicial de 0 cm
%  2. Se ejecuta cada modelo por separado en orden secuencial
%  3. Se realiza una prueba con un valor de coeficiente para probar su
%  resultado
%  4. Se grafica la prueba realizada permitiendo guardar una comparativa de
%  los valores asignados a cada prueba pues todos son dependientes de Kp
% =========================================================================

% --- CARGA DE LIBRERIA---
%ESTE DRIVER EMPLEADO REQUIERE UNA INSTALACIÓN PREVIA
%SE EMPLEA ESTE DRIVER SOBRE OTROS CON MAS OPCIONES POR LA COMPATIBILIDAD CON SOFTWARE DE 64BITS%
%OTROS DRIVERS TIENEN PROBLEMAS CON EL SOFTWARE DE 64 BITS

% ---PARAMETROS DEL CONTROLADOR CHR---
Kp = 1.2;
Ti = 2.0;
Td = 1.0;

% --- PARÁMETROS DE CONFIGURACIÓN ---
tiempo_exp_individual = 20;  % s por cada prueba
dt = 0.05;                   % periodo de muestreo
consigna_cm = 20.0;
posicion_inicio_cm = -3.0;   % "cero" lógico: luego aplicamos corrección
offset_inicio = 3.0;         % corrección que usabas en el script
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
ax1 = subplot(3, 1, 1); title(ax1,'Respuesta del Controlador Proporcional (P)'); ylabel(ax1,'Posición (cm)'); grid(ax1,'on'); hold(ax1,'on');
ax2 = subplot(3, 1, 2); title(ax2,'Respuesta del Controlador Proporcional-Integral (PI)'); ylabel(ax2,'Posición (cm)'); grid(ax2,'on'); hold(ax2,'on');
ax3 = subplot(3, 1, 3); title(ax3,'Respuesta del Controlador Proporcional-Derivativo (PD)'); xlabel(ax3,'Tiempo (s)'); ylabel(ax3,'Posición (cm)'); grid(ax3,'on'); hold(ax3,'on');

%% PASO 2: EJECUCIÓN SECUENCIAL
try
    %INICIALIZACION DEL MOVIMIENTO
    disp('--- FASE DE INICIALIZACIÓN ---');
    fprintf('Moviendo el carro a la posición de inicio universal (%.2f cm)...\n', iniobjetivo);
    go_to_position(iniobjetivo, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);
    disp('Sistema inicializado. Comenzando pruebas...');
    
    % --- Experimento 1: Controlador P ---
    disp('--- Iniciando Prueba del Controlador P ---');
    [hist_t_P, hist_x_P] = run_controller_test('P', Kp, Ti, Td, tiempo_exp_individual, dt, consigna_cm, posicion_inicio_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);
    plot(ax1, hist_t_P, hist_x_P, 'b-', 'LineWidth', 2, 'DisplayName', 'Posición Real');
    plot(ax1, [0 hist_t_P(end)], [consigna_cm consigna_cm], 'r--', 'LineWidth',1,'DisplayName', 'Consigna');
    legend(ax1,'show','Location','southeast');
    
    disp('Prueba P finalizada. Volviendo al inicio...');
    go_to_position(iniobjetivo, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);

    % --- Experimento 2: Controlador PI ---
    disp('--- Iniciando Prueba del Controlador PI ---');
    [hist_t_PI, hist_x_PI] = run_controller_test('PI', Kp, Ti, Td, tiempo_exp_individual, dt, consigna_cm, posicion_inicio_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);
    plot(ax2, hist_t_PI, hist_x_PI, 'b-', 'LineWidth', 2, 'DisplayName', 'Posición Real');
    plot(ax2, [0 hist_t_PI(end)], [consigna_cm consigna_cm], 'r--', 'LineWidth',1,'DisplayName', 'Consigna');
    legend(ax2,'show','Location','southeast');

    disp('Prueba PI finalizada. Volviendo al inicio...');
    go_to_position(iniobjetivo, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);

    % --- Experimento 3: Controlador PD ---
    disp('--- Iniciando Prueba del Controlador PD ---');
    [hist_t_PD, hist_x_PD] = run_controller_test('PD', Kp, Ti, Td, tiempo_exp_individual, dt, consigna_cm, posicion_inicio_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);
    plot(ax3, hist_t_PD, hist_x_PD, 'b-', 'LineWidth', 2, 'DisplayName', 'Posición Real');
    plot(ax3, [0 hist_t_PD(end)], [consigna_cm consigna_cm], 'r--', 'LineWidth',1,'DisplayName', 'Consigna');
    legend(ax3,'show','Location','southeast');
    
    disp('Prueba PD finalizada. Volviendo al inicio por última vez...');
    go_to_position(iniobjetivo, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor);

catch e
    disp('¡ERROR DURANTE LA EJECUCIÓN DE LAS PRUEBAS!');
    rethrow(e)
end

%%  PASO 3: APAGADO SEGURO FINAL
disp('Apagando el motor...');

try
    % Intento apagar eDAC y comprobar error
    [errorCode, idnum] = EAnalogOut(idnum, demo, 0.0, 0.0);
    if exist('errorCode','var') && errorCode ~= 0
        warning('EAnalogOut devolvió código de error: %d', errorCode);
    end
    disp('Sistema detenido de forma segura.');
catch
    disp('Error durante el apagado seguro.');
end

%%  FUNCIONES AUXILIARES (subfunciones locales)

function [hist_tiempo, hist_posicion] = run_controller_test(tipo_control, Kp, Ti, Td, tiempo_exp, dt, consigna_cm, posicion_inicio_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor)
    % Inicializaciones
    num_pasos = floor(tiempo_exp / dt);
    integral_error = 0;
    posicion_anterior_cm = posicion_inicio_cm;
    hist_tiempo = zeros(1, num_pasos);
    hist_posicion = zeros(1, num_pasos);
    tic;
    
    % Parámetros anti-windup / límites
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
        
        % --- Conversión a cm y saturación física ---
        posicion_actual_cm = gain_conversion * voltaje_sensor + 0.22;
        posicion_actual_cm = max(0, min(30.0, posicion_actual_cm));
        
        % --- Error ---
        error_actual = consigna_cm - posicion_actual_cm;
        
        % --- Componentes P, I, D ---
        P = Kp * error_actual;
        I = 0;
        D = 0;
        
        % Integral solo para PI
        if strcmpi(tipo_control, 'PI')
            integral_error_temp = integral_error + error_actual * dt;
            if Ti > eps
                I = (Kp / Ti) * integral_error_temp;
            else
                I = 0;
            end
        end
        
        % Derivada solo para PD
        if strcmpi(tipo_control, 'PD')
            derivada_pos = (posicion_actual_cm - posicion_anterior_cm) / dt;
            D = - (Kp * Td) * derivada_pos;
        end
        
        % Señal total del controlador
        if strcmpi(tipo_control,'PI')
            u_esfuerzo = P + I;
        elseif strcmpi(tipo_control,'PD')
            u_esfuerzo = P + D;
        else
            u_esfuerzo = P;
        end
        
        % --- Convertir esfuerzo a señal de % y voltaje ---
        % Interpretación: ((u - offset)/gain_conv) => fracción en V; convertimos a %
        control_senal_percent = ((u_esfuerzo - 0.22) / gain_conversion) * 100;   % en %
        control_senal_percent = max(minPercent, min(maxPercent, control_senal_percent));
        voltaje_salida_motor = (control_senal_percent / 100) * 5.0;  % en V, entre -5 y 5
        voltaje_salida_motor = max(-5, min(5, voltaje_salida_motor));
        
        % Anti-windup simple: si no estamos saturados, aceptar integral_temp
        if strcmpi(tipo_control,'PI')
            % Recalcular I con integral_error_temp y comprobar saturación teórica
            I_check = (Kp / max(Ti, eps)) * integral_error_temp;
            u_check = P + I_check;
            control_percent_check = ((u_check - 0.22) / gain_conversion) * 100;
            if control_percent_check >= minPercent && control_percent_check <= maxPercent
                integral_error = integral_error_temp;
                I = I_check;
            else
                % no actualizar integral_error (anti-windup)
                % opcional: integral_error remains the same
            end
            u_esfuerzo = P + I;
        end
        
        % --- Enviar a hardware (con comprobación de errores) ---
        try
            % Digital outputs (dirección) - adaptado al orden de tus funciones
            [~, idnum] = EDigitalOut(idnum, demo, 1, 0, gain_direction * voltaje_salida_motor + bias_direction);
            [errorCodeOut, idnum] = EAnalogOut(idnum, demo, abs(voltaje_salida_motor), 0.0);
            if exist('errorCodeOut','var') && errorCodeOut ~= 0
                warning('EAnalogOut devolvió código de error: %d', errorCodeOut);
            end
            [~, idnum] = EDigitalOut(idnum, demo, canal, 1, bias_motor + abs(voltaje_salida_motor));
        catch hwErr
            warning('Error al escribir salidas HW: %s', hwErr.message);
        end
        
        % --- Registro histórico ---
        hist_tiempo(k) = toc;
        hist_posicion(k) = posicion_actual_cm;
        
        % --- Preparación siguiente paso ---
        posicion_anterior_cm = posicion_actual_cm;
        
        % Mantener periodo de muestreo
        pause(max(0, dt - (toc - tiempo_iteracion_inicio)));
    end
    
    % Apagar la salida al terminar la prueba (seguro)
    try
        [errStop, ~] = EAnalogOut(idnum, demo, 0.0, 0.0);
        if exist('errStop','var') && errStop ~= 0
            warning('Error al apagar EAnalogOut: %d', errStop);
        end
    catch
        % si falla no rompemos el script
    end
end

function go_to_position(posicion_objetivo_cm, idnum, demo, canal, gain_conversion, gain_direction, bias_direction, bias_motor)
    % Mueve el carro a posicion_objetivo_cm (valor absoluto en cm)
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
