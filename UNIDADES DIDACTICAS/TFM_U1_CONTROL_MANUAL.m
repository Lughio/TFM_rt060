clc;
clear;
close all;
% ========================================================================
%  UNIDAD 1: CONTROL MANUAL DE LA MAQUETA 
%  Este script implementa el control manual para determinar la relacion
%  voltaje-posición:
%  1. Se controla con la:
%    - Tecla '+': Mueve el carro a la derecha.
%    - Tecla '-': Mueve el carro a la izquierda.
%    - Tecla 'ESC': Detiene el programa de forma segura.
%  2. Al finalizar puede visualizarse la tension y posición calculada
%  3. Fisicamente se comprueba si corresponde con la realidad
%  4. Se prueba valores de conversión hasta sacar una relación entre los
%  resultados y la realidad.
% =============================================================================================

%% PASO 1: DECLARACIONES E INICIALIZACIONES
% --- CARGA DE LIBRERIA---
%ESTE DRIVER EMPLEADO REQUIERE UNA INSTALACIÓN PREVIA
%SE EMPLEA ESTE DRIVER SOBRE OTROS CON MAS OPCIONES POR LA COMPATIBILIDAD CON SOFTWARE DE 64BITS%
%OTROS DRIVERS TIENEN PROBLEMAS CON EL SOFTWARE DE 64 BITS

% --- PARAMETROS DE CONFIGURACIÓN ---
voltaje_paso =1.0 ; % VOLTAJE PARA APLICAR MOVIMIENTO(+VOLTAJE +VELOCIDAD)
duracion_paso = 0.1; % TIEMPO EN SEGUNDOS QUE EL MOTOR ESTARA ENCENDIDOgain_conversion=7.36;
offset_conversion=0.22;
gain_direction = 0.2;%EXTRAIDA DE EJEMPLO DE SIMULINK DEL FABRICANTE
bias_direction = 1;%EXTRAIDA DE EJEMPLO DE SIMULINK DEL FABRICANTE
bias_motor = 0.9;%EXTRAIDA DE EJEMPLO DE SIMULINK DEL FABRICANTE
pos_max_cm=30.0;
% --- PARAMETROS DE CONEXIONADO LABJACK RT060---
idnum = -1;
demo = 0;
canal_lectura_pos = 0;
canal_do_1 = 1; %CANAL DIGITAL 1
canal_ao_motor = 0;
canal_do_2 = 0; %CANAL DIGITAL 0

% INICIALIZACIÓN DE VARIABLES
keep_running = true;

%% ------------INTERFAZ GRÁFICA---------------
fig = figure('Name', 'Control Manual RT060', 'KeyPressFcn', 'uiresume(gcbf)');
ax = axes('Position', [0.0 0.4 1.0 0.3]);
titleHandle = title(ax, 'Esperando tecla... (+ para derecha, - para izquierda, ESC para salir)');
textHandle = text(0.5, 0.5, 'Posición: -- cm | Voltaje: -- V', 'FontSize', 24, ...
    'HorizontalAlignment', 'center', 'Parent', ax);
set(ax, 'XTick', [], 'YTick', []);
%CUADRO DE TEXTO VALOR GAIN_CONVERSION
uicontrol("Style","text",'Units','normalized',"Position",[0.01 0.95 0.3 0.03],'FontWeight','bold','String','GAIN_CONVERSION: ','Parent',fig);
%INICIALIZACION VALOR OFFSET_CONVERSION
variable_gain=uicontrol('Style','edit','Units','normalized','Position',[0.30 0.95 0.20 0.04],'String',num2str(gain_conversion),'Parent',fig);
%CUADRO DE TEXTO VALOR OFFSET_CONVERSION
uicontrol("Style","text",'Units','normalized',"Position",[0.01 0.90 0.3 0.03],'FontWeight','bold','String','OFFSET_CONVERSION: ','Parent',fig);
%INICIALIZACION VALOR GAIN_CONVERSION
variable_offset=uicontrol('Style','edit','Units','normalized','Position',[0.30 0.90 0.20 0.04],'String',num2str(offset_conversion),'Parent',fig);
%CUADRO DE TEXTO VALOR VOLTAJE_PASO
uicontrol("Style","text",'Units','normalized',"Position",[0.01 0.85 0.3 0.03],'FontWeight','bold','String','VOLTAJE DE MOVIMIENTO: ','Parent',fig);
%INICIALIZACION VALOR GAIN_CONVERSION
variable_paso=uicontrol('Style','edit','Units','normalized','Position',[0.30 0.85 0.20 0.04],'String',num2str(voltaje_paso),'Parent',fig);
%BOTON DE ACTUALIZACION
total_variable=[variable_paso variable_offset variable_gain];
uicontrol('Style','pushbutton','Units','normalized','Position',[0.80 0.85 0.2 0.04],'FontWeight','bold','String','ACTUALIZAR','BackgroundColor',[0 1 0],'Callback',@(src,event) actualizarValores(total_variable,textHandle,idnum,demo,canal_lectura_pos,pos_max_cm),'Parent',fig);
disp('Ventana de control activa.');
disp('Presiona "+", "-" o "ESC".');

%%  PASO 2: BUCLE DE CONTROL MANUAL 
while keep_running
    %ACTUALIZACIÓN VARIABLES POR INTERFAZ GRAFICA
    variable_gain_conv=str2double(get(variable_gain,'String'));
    variable_gain_conv = max(0.0, min(50.0, variable_gain_conv));
    variable_offset_conv=str2double(get(variable_offset,'String'));
    variable_offset_conv = max(0.0, min(1.0, variable_offset_conv));
    voltaje_salida=str2double(get(variable_paso,'String'));
    voltaje_salida = max(0.480, min(5.0, voltaje_salida));

    % SE LEE LA POSICION ACTUAL
    [voltaje_actual, ~, ~, ~] = EAnalogIn(idnum, demo, canal_lectura_pos, 0);
    
    % EMPLEO LA FORMULA DE CONVERSION
    posicion_actual_cm = variable_gain_conv*voltaje_actual+variable_offset_conv;
    posicion_actual_cm = max(0, min(pos_max_cm, posicion_actual_cm));
    
    % ACTUALIZO CONSTANTEMENTE EL TEXTO PARA MOSTRAR LOS VALORES
    set(textHandle, 'String', sprintf('Posición: %.2f cm | Voltaje: %.4f V', posicion_actual_cm, voltaje_actual));
    
    % ESPERAR A QUE EL USUARIO PRESIONE UNA TECLA 
    uiwait(fig);
    
    % AL CERRARSE LA FIGURA SE CIERRA EL PROGRAMA
    if ~ishghandle(fig)
        break;
    end
    
    % LEO LA TECLA PULSADA
    key = get(fig, 'CurrentCharacter');
    
    
    % SEGÚN LA TECLA  PULSADA SE REALIZA LAS SIGUIENTES ACCIONES
    switch key
        case '+'
            %ACTUALIZACION DE INTERFAZ A MOVIMIENTO
            set(titleHandle, 'String', 'MOVIENDO A LA DERECHA...');
            
            % --- MOVER MOTOR A LA DERECHA ---
            dir_dcha=gain_direction*voltaje_salida +bias_direction; % DCHA ES VALOR POSITIVO, SG U2 UNOS 0.480V
            % CONTROL DE MOVIMIENTO DEL MOTOR 
            [~,idnum]=EDigitalOut(idnum,demo,canal_do_1,0,dir_dcha);%SEÑAL DE DIRECCIÓN
            [~, idnum] = EAnalogOut(idnum, demo, abs(voltaje_salida), 0.0);
            [~,idnum]=EDigitalOut(idnum,demo,canal_do_2,1,bias_motor+abs(voltaje_salida));
            
            pause(duracion_paso); % SE DA UN TIEMPO DE ESTABILIZACIÓN
            
            % SE PARA EL MOTOR PARA GARANTIZAR QUE LA POSICIÓN NO VARIA SIN
            % INTERACCIÓN DEL USUARIO.
            [~, ~] = EAnalogOut(idnum, demo, 0.0, 0.0);

        case '-'
             %ACTUALIZACION DE INTERFAZ A MOVIMIENTO
            set(titleHandle, 'String', 'MOVIENDO A LA IZQUIERDA...');

            % --- MOVER MOTOR A LA IZQUIERDA ---
            dir_izq = -gain_direction * voltaje_salida + bias_direction; %IZQ ES VALOR PEQ O NEGATIVO
            
            [~,idnum]=EDigitalOut(idnum,demo,canal_do_1,0, dir_izq); % SEÑAL DE DIRECCIÓN
            [~, idnum] = EAnalogOut(idnum, demo, abs(voltaje_salida), 0.0);       % MAGNITUD DE VELOCIDAD
            [~,idnum]=EDigitalOut(idnum,demo,canal_do_2,1,bias_motor+abs(voltaje_salida));
            
            pause(duracion_paso); % TIEMPO DE ESTABILIZACIÓN DE POSICIÓN
            [~, ~] = EAnalogOut(idnum, demo, 0.0, 0.0);% SE PAUSA MOTOR PARA GARANTIZAR POSICIÓN
            
        case char(27) % TECLA ESC EN CODIGO ASCII
            set(titleHandle, 'String', 'Saliendo...');
            keep_running = false;
    end
    
    % ACTUALIZACION DE INTERFAZ A ESPERA
    set(titleHandle, 'String', 'Esperando tecla... (+ para derecha, - para izquierda, ESC para salir)');
end

%%  PASO 3: APAGADO SEGURO
if ishghandle(fig)
    close(fig);
end
disp('Finalizando y apagando el motor...');
try
    % APAGADO DE MOTOR POR SEGURIDAD
    [errorCode, ~] = EAnalogOut(idnum, demo, 0.0, 0.0);
    if errorCode ~= 0, error('Error en EAnalogOut durante el apagado: %d', errorCode); end
    disp('Sistema detenido de forma segura.');
catch ex
    disp('Error durante el apagado seguro.');
    disp(ex.message);
end

  % FUNCION DE ACTUALIZACIÓN DE INTERFAZ
 function actualizarValores(total_variable,textHandle,idnum,demo,canal_lectura_pos,pos_max_cm)
    valores = zeros(1,length(total_variable));
    for i=1:length(total_variable)
         valor = str2double(get(total_variable(i),'String'));
         if isnan(valor)
             errordlg('Introduce un número válido','Error');
             return;
         else
             if i==1
                 valor=max(0.480,min(5,valor));
             elseif i==2
                 valor=max(0.0,min(1.0,valor));
             else
                 valor=max(0,min(50,valor));
             end
             set(total_variable(i),'String',num2str(valor));
             valores(i)=valor;
         end
    end
   
    %REACTUALIZACION INMEDIATA
    voltaje_paso       = valores(1);
    offset_conversion  = valores(2);
    gain_conversion    = valores(3);

    % SE LEE LA POSICION DE NUEVO CON LOS NUEVOS VALORES
    [voltaje_actual, ~, ~, ~] = EAnalogIn(idnum, demo, canal_lectura_pos, 0);
    posicion_actual_cm = gain_conversion * voltaje_actual + offset_conversion;
    posicion_actual_cm = max(0, min(pos_max_cm, posicion_actual_cm));

    % SE ACTUALIZA EL TEXTO DE LA INTERFAZ
    set(textHandle, 'String',sprintf('Posición: %.2f cm | Voltaje: %.4f V', posicion_actual_cm, voltaje_actual));
    disp("Valores Actualizados");
    set(gcbf, 'CurrentObject', []);  %DEJA DE OBSERVAR LA UI PARA LEER DE NUEVO LA TECLAS
end

       
