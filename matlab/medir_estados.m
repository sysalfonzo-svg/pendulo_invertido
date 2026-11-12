function resultado = medir_estados(puerto, duracion_s, carpeta, mostrar)
% MEDIR_ESTADOS Visualiza y registra PENDULO_STATES_V1, sin accionar motores.
%   R = medir_estados("COM15",60)
%   R = medir_estados("COM15",20,"registros",false) % sin ventana
%   R = medir_estados() % seleccion automatica SOLO con un puerto disponible
% El COM es variable: consultar serialportlist("available") antes de ejecutar.
% Cerrar PlatformIO Serial Monitor y cualquier aplicacion que use ese COM.
% No envia Z ni D automaticamente. Los botones requieren una accion manual.
arguments
    puerto (1,1) string = ""
    duracion_s (1,1) double {mustBePositive,mustBeFinite} = 60
    carpeta (1,1) string = ""
    mostrar (1,1) logical = true
end
if strlength(puerto) == 0
    puertos = serialportlist("available");
    if numel(puertos) ~= 1
        error('Pendulo:Puerto', ...
            'Indique el COM de la STM32. Disponibles: %s', strjoin(puertos,', '));
    end
    puerto = puertos(1);
end
if strlength(carpeta) == 0
    carpeta = string(fullfile(fileparts(fileparts(mfilename('fullpath'))),'registros'));
end
if ~isfolder(carpeta)
    mkdir(carpeta);
end
inicio = datetime('now','TimeZone','America/Bogota');
base = fullfile(carpeta,"estados_"+string(inicio,'yyyyMMdd_HHmmss_SSS'));
archivos = struct('csv',base+".csv",'mat',base+".mat",'eventos',base+"_eventos.txt");
nombres = esquema_estados();
capacidad = max(2000,ceil(min(duracion_s,600)*220));
datos = NaN(capacidad,27);
n = 0; lineasInvalidas = 0; fragmento = ''; detener = false;
identificado = false; terminado = false; fallo = "";
s = []; fig = []; estadoUI = []; trazos = gobjects(4,1);
selectores = gobjects(3,1); signosUIInicializados = false;
fidCSV = -1; fidEventos = -1; comentarios = strings(0,1);
meta = struct('protocolo',"PENDULO_STATES_V1",'puerto',puerto, ...
    'inicio',inicio,'duracion_solicitada_s',duracion_s, ...
    'Ts_objetivo_s',0.005,'unidades_estados',{{'m','m/s','rad','rad/s'}}, ...
    'izquierda_fisica',"encoder R, CPR=1974", ...
    'derecha_fisica',"encoder L, CPR=1969.6", ...
    'nota',"Sin motores. Z y D solo por accion manual; calibracion RAM del MCU.");
T = array2table(zeros(0,27),'VariableNames',nombres);
resumen = struct(); tiempo_s = zeros(0,1);
try
    fidCSV = fopen(archivos.csv,'wt');
    fidEventos = fopen(archivos.eventos,'wt');
    if fidCSV < 0 || fidEventos < 0
        error('Pendulo:Archivo','No se pueden abrir los archivos de registro.');
    end
    % Recursos capturados por valor; un cleanup no depende del workspace
    % compartido de funciones anidadas durante una interrupcion con Ctrl+C.
    limpiezaArchivos = onCleanup(@() cerrarArchivos(fidCSV,fidEventos));
    fprintf(fidCSV,'%s\n',strjoin(nombres,','));
    s = serialport(puerto,115200,'Timeout',0.25);
    limpiezaSerial = onCleanup(@() cerrarSerial(s));
    configureTerminator(s,'LF');
    setDTR(s,true);
    flush(s);
    reloj = tic;
    ultimoPregunta = -1;
    while ~identificado && toc(reloj) < 6
        if toc(reloj)-ultimoPregunta >= 0.5
            writeline(s,'?');
            ultimoPregunta = toc(reloj);
        end
        consumir(true);
        pause(0.005);
    end
    if ~identificado
        error('Pendulo:Firmware', ...
            'No responde PENDULO_STATES_V1. No se inicia G: comprobar carga y COM.');
    end
    if mostrar
        crearVentana();
        limpiezaFigura = onCleanup(@() cerrarFigura(fig));
    end
    enviar('G');
    reloj = tic; ultimoDato = 0; ultimoDibujo = -1;
    while ~detener && toc(reloj) < duracion_s
        anterior = n;
        consumir(false);
        if n > anterior
            ultimoDato = toc(reloj);
        end
        if toc(reloj)-ultimoDato > 5
            error('Pendulo:SinDatos','Mas de 5 s sin muestras: conexion interrumpida.');
        end
        if mostrar && isgraphics(fig) && toc(reloj)-ultimoDibujo >= 0.1
            dibujar();
            ultimoDibujo = toc(reloj);
        end
        if s.NumBytesAvailable == 0
            pause(0.002); % Solo el PC; la adquisicion de 5 ms ocurre en el MCU.
        end
    end
catch ME
    fallo = string(getReport(ME,'basic','hyperlinks','off'));
    warning('Pendulo:Captura','Captura terminada: %s',ME.message);
end
finalizar();
resultado = struct('datos',T,'tiempo_s',tiempo_s,'resumen',resumen, ...
    'meta',meta,'archivos',archivos);
fprintf('\n%u muestras guardadas en %s\n',n,archivos.csv);
disp(resumen);
fprintf('El resumen valida adquisicion y flags; la escala mecanica requiere prueba fisica.\n');

    function consumir(soloIdentidad)
        disponibles = s.NumBytesAvailable;
        if disponibles == 0
            return
        end
        % read se limita a bytes ya recibidos: no espera una linea incompleta.
        bytes = read(s,min(disponibles,65536),'uint8');
        fragmento = [fragmento, char(bytes(:).')];
        finales = find(fragmento == newline);
        if isempty(finales)
            if numel(fragmento) > 4096
                lineasInvalidas = lineasInvalidas+1;
                fragmento = '';
            end
            return
        end
        primero = 1;
        for j = 1:numel(finales)
            linea = strtrim(fragmento(primero:finales(j)-1));
            primero = finales(j)+1;
            [fila,tipo] = decodificar_estado(linea);
            if tipo == "comentario"
                if ~isempty(regexp(linea,'^#\s+PENDULO_STATES_V1(\s|$|;)','once'))
                    identificado = true;
                end
                anotar(linea);
            elseif tipo == "muestra" && ~soloIdentidad
                n = n+1;
                if n > size(datos,1)
                    datos(end+20000,27) = NaN; %#ok<AGROW>
                end
                datos(n,:) = fila;
                fprintf(fidCSV,'%s\n',linea);
            elseif tipo == "invalida"
                lineasInvalidas = lineasInvalidas+1;
                anotar(['RX_INVALIDA ',linea]);
            end
        end
        fragmento = fragmento(primero:end);
    end

    function anotar(texto)
        marca = char(datetime('now','TimeZone','America/Bogota', ...
            'Format','yyyy-MM-dd HH:mm:ss.SSS XXX'));
        registro = string(marca)+" "+string(texto);
        comentarios(end+1,1) = registro;
        if fidEventos >= 0
            fprintf(fidEventos,'%s\n',registro);
        end
        if isscalar(estadoUI) && isgraphics(estadoUI) && ~startsWith(string(texto),"# CSV:")
            estadoUI.String = char(texto);
        end
    end

    function enviar(comando)
        % Esta lista impide incorporar accidentalmente un comando de motor.
        permitido = any(strcmp(comando,{'G','H','S','N','Z'})) || ...
            ~isempty(regexp(comando,'^D -?1 -?1 -?1$','once'));
        assert(identificado && permitido,'Comando no permitido.');
        writeline(s,comando);
        anotar(['TX ',comando]);
    end

    function crearVentana()
        fig = figure('Name','Cuatro estados - motores deshabilitados', ...
            'NumberTitle','off','Color','w','Position',[100,100,1100,760], ...
            'CloseRequestFcn',@cerrar);
        titulos = {'Posicion del carro','Velocidad filtrada', ...
            'Angulo: 0 = vertical arriba','Velocidad angular filtrada'};
        unidades = {'m','m/s','grados','grados/s'};
        for q = 1:4
            eje = axes(fig,'Position',[0.08,0.81-(q-1)*0.165,0.88,0.125]);
            trazos(q) = plot(eje,NaN,NaN,'LineWidth',1);
            grid(eje,'on'); ylabel(eje,unidades{q}); title(eje,titulos{q});
            if q == 4
                xlabel(eje,'Tiempo MCU (s)');
            end
        end
        uicontrol(fig,'Style','pushbutton','String','Z: vertical ARRIBA ahora', ...
            'Units','normalized','Position',[0.03,0.17,0.24,0.045], ...
            'Callback',@(~,~) enviar('Z'));
        uicontrol(fig,'Style','pushbutton','String','N: origen del carro', ...
            'Units','normalized','Position',[0.29,0.17,0.18,0.045], ...
            'Callback',@(~,~) enviar('N'));
        uicontrol(fig,'Style','pushbutton','String','S: EMERGENCIA', ...
            'Units','normalized','Position',[0.49,0.17,0.2,0.045], ...
            'ForegroundColor',[0.8,0,0],'Callback',@(~,~) enviar('S'));
        uicontrol(fig,'Style','pushbutton','String','Terminar registro', ...
            'Units','normalized','Position',[0.71,0.17,0.24,0.045], ...
            'Callback',@terminar);
        etiquetas = {'Signo L (rueda derecha)','Signo R (rueda izquierda)','Signo theta'};
        for q = 1:3
            xx = 0.03+(q-1)*0.24;
            uicontrol(fig,'Style','text','String',etiquetas{q}, ...
                'Units','normalized','Position',[xx,0.12,0.23,0.035], ...
                'BackgroundColor','w','HorizontalAlignment','left');
            selectores(q) = uicontrol(fig,'Style','popupmenu','String',{'+1','-1'}, ...
                'Units','normalized','Position',[xx,0.08,0.15,0.04]);
        end
        uicontrol(fig,'Style','pushbutton','String','D: confirmar sentidos', ...
            'Units','normalized','Position',[0.75,0.08,0.22,0.05], ...
            'Callback',@aplicarSignos);
        estadoUI = uicontrol(fig,'Style','text','String', ...
            'Mover a mano. Para Z: sostener quieto y vertical ARRIBA. Ver README.', ...
            'Units','normalized','Position',[0.03,0.015,0.94,0.05], ...
            'BackgroundColor','w','HorizontalAlignment','left');
        drawnow;

        function aplicarSignos(~,~)
            signos = 3-2*[selectores.Value];
            enviar(sprintf('D %d %d %d',signos));
        end
    end

    function dibujar()
        if n == 0
            drawnow limitrate; return
        end
        % Solo ventana de 10 s; el CSV conserva todas las muestras recibidas.
        idx = max(1,n-1999):n;
        xx = datos(idx,2);
        tt = [0;cumsum(mod(diff(xx),2^32))]*1e-6;
        valores = datos(idx,6:9);
        flags = uint32(datos(idx,12));
        angularInvalido = bitand(flags,uint32(3)) ~= 0;
        valores(angularInvalido,3:4) = NaN;
        valores(:,3:4) = rad2deg(valores(:,3:4));
        for q = 1:4
            set(trazos(q),'XData',tt,'YData',valores(:,q));
        end
        if ~signosUIInicializados
            for q = 1:3
                selectores(q).Value = 1+(datos(n,22+q)<0);
            end
            signosUIInicializados = true;
        end
        ultimoFlag = uint32(datos(n,12));
        sensorTexto = 'valido'; ceroTexto = 'definido'; signosTexto = 'declarados';
        if bitand(ultimoFlag,uint32(1)), sensorTexto = 'INVALIDO'; end
        if bitand(ultimoFlag,uint32(2)), ceroTexto = 'PENDIENTE: sujetar ARRIBA y Z'; end
        if bitand(ultimoFlag,uint32(4)), signosTexto = 'PENDIENTES'; end
        estadoUI.String = sprintf(['Sensor %s | Cero %s | Sentidos %s\n' ...
            'Activos: L=%+d, R=%+d, theta=%+d | Motores bloqueados | Escala: verificar con regla'], ...
            sensorTexto,ceroTexto,signosTexto,datos(n,23:25));
        fig.Name = sprintf(['Cuatro estados | n=%d | L=%d R=%d raw=%d ' ...
            '| flags=%d | PWM=%d,%d'],n,datos(n,4:5),datos(n,10), ...
            datos(n,12:14));
        drawnow limitrate;
    end

    function terminar(~,~)
        detener = true;
    end

    function cerrar(~,~)
        detener = true;
        delete(fig);
    end

    function finalizar()
        if terminado
            return
        end
        terminado = true;
        if ~isempty(s)
            cerrarSerial(s);
            s = [];
        end
        if fidCSV >= 0
            fclose(fidCSV); fidCSV = -1;
        end
        if fidEventos >= 0
            fclose(fidEventos); fidEventos = -1;
        end
        if isgraphics(fig)
            delete(fig);
        end
        datos = datos(1:n,:);
        T = array2table(datos,'VariableNames',nombres);
        [resumen,tiempo_s] = validar_estados(datos);
        resumen.lineas_invalidas = lineasInvalidas;
        if lineasInvalidas ~= 0
            resumen.aceptacion_temporal = false;
            resumen.cuatro_estados_listos = false;
        end
        meta.fin = datetime('now','TimeZone','America/Bogota');
        meta.firmware_identificado = identificado;
        meta.error = fallo;
        meta.comentarios_y_comandos = comentarios;
        try
            save(archivos.mat,'T','datos','tiempo_s','resumen','meta');
        catch MEguardar
            warning('Pendulo:Guardar','No se pudo guardar MAT: %s',MEguardar.message);
        end
    end
end

function cerrarSerial(s)
% Esta funcion no depende del workspace anidado de medir_estados.
try
    if ~isempty(s) && isvalid(s)
        writeline(s,'H');
        writeline(s,'S');
    end
catch
    % Desconexion USB: el paro por software no puede entregarse.
end
try
    if ~isempty(s) && isvalid(s)
        delete(s);
    end
catch
end
end

function cerrarArchivos(varargin)
for k = 1:nargin
    try
        if varargin{k} >= 0 && ~isempty(fopen(varargin{k}))
            fclose(varargin{k});
        end
    catch
    end
end
end

function cerrarFigura(fig)
if isgraphics(fig)
    delete(fig);
end
end
