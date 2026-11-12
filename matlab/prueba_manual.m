function resultado = prueba_manual(puerto,duracion_s,carpeta)
% PRUEBA_MANUAL Registro y pulsos manuales protegidos, sin equilibrio automatico.
% R = prueba_manual("COM15",300)
% Antes: marcar frente, verificar sentidos a mano, elevar ruedas y sujetar carro.
% Z/D/V/E/A/C requieren clic explicito. Nunca se rearman ni repiten pulsos solos.
arguments
    puerto (1,1) string
    duracion_s (1,1) double {mustBePositive,mustBeFinite} = 300
    carpeta (1,1) string = ""
end
if strlength(carpeta)==0
    carpeta=string(fullfile(fileparts(fileparts(mfilename('fullpath'))),'registros'));
end
reg=RegistroManual(puerto,duracion_s,carpeta);
limpieza=limpiezaRegistro(reg); %#ok<NASGU> % onCleanup owns safe shutdown.
reloj=tic;
identificado=false; detener=false; fragmento='';
ultimoKeep=-Inf; ultimoP=-Inf; ultimoDato=0; ultimoDibujo=-Inf;
motor=struct('state',"DESCONOCIDO",'faults',NaN,'verify',0,'good',0, ...
    'pwmL',0,'pwmR',0,'max_loop_us',NaN,'max_cycle_us',NaN,'F_N',NaN,'manual_build',0);
ultimoEstado=-Inf; ultimaNoticia="Esperando identidad del firmware manual";
trazos=gobjects(4,1); selectores=gobjects(3,1);
checkFrente=[]; botonA=[]; botonC=[]; botonV=[]; estadoUI=[]; avisoUI=[];
campoL=[]; campoR=[]; campoDuracion=[]; signosInicializados=false;
try
    reg.Serial=serialport(puerto,115200,'Timeout',0.1);
    configureTerminator(reg.Serial,'LF'); setDTR(reg.Serial,true); flush(reg.Serial);
    ultimoPregunta=-Inf;
    while ~identificado && toc(reloj)<6
        if toc(reloj)-ultimoPregunta>=0.5
            writeline(reg.Serial,comando_manual("?")); ultimoPregunta=toc(reloj);
        end
        consumir(true); pause(0.002);
    end
    assert(identificado,'No responde PENDULO_STATES_V1 y MOTOR_MANUAL_V1 en la misma identidad.');
    reg.Meta.firmware_identificado=true;
    crearVentana();
    enviar("G",[],false);
    inicioCaptura=toc(reloj); ultimoDato=inicioCaptura;
    while ~detener && toc(reloj)-inicioCaptura<duracion_s
        mantenerConexion();
        consumir(false);
        mantenerConexion();
        if toc(reloj)-ultimoDato>2
            error('Pendulo:SinDatos','Dos segundos sin muestras; se cierra y se solicita parada.');
        end
        if toc(reloj)-ultimoDibujo>=0.1
            dibujar(); ultimoDibujo=toc(reloj);
        end
        if reg.Serial.NumBytesAvailable==0, pause(0.002); end
    end
catch ME
    reg.Meta.error=string(getReport(ME,'basic','hyperlinks','off'));
    reg.evento("ERROR "+string(ME.message));
    warning('Pendulo:Manual','Prueba terminada: %s',ME.message);
end
reg.cerrar();
resultado=reg.Resultado;
fprintf('%d muestras manuales guardadas en %s\n',reg.N,reg.Archivos.csv);
disp(resultado.resumen);
clear limpieza

    function mantenerConexion()
        ahora=toc(reloj);
        if ahora-ultimoKeep>=0.05
            enviar("K",[],false); ultimoKeep=toc(reloj);
        end
        if ahora-ultimoP>=0.5
            enviar("P",[],false); ultimoP=toc(reloj);
        end
    end
    function consumir(soloIdentidad)
        nbytes=reg.Serial.NumBytesAvailable;
        if nbytes==0, return; end
        bytes=read(reg.Serial,min(nbytes,8192),'uint8');
        fragmento=[fragmento,char(bytes(:).')];
        finales=find(fragmento==newline);
        if isempty(finales)
            if numel(fragmento)>4096
                reg.Invalidas=reg.Invalidas+1; fragmento='';
            end
            return
        end
        primero=1;
        for j=1:numel(finales)
            linea=strtrim(fragmento(primero:finales(j)-1)); primero=finales(j)+1;
            [fila,tipo]=decodificar_estado(linea);
            if tipo=="comentario"
                [dato,tipoMotor]=decodificar_motor(linea);
                if tipoMotor=="identidad", identificado=true; end
                if tipoMotor=="motor"
                    motor=dato; ultimoEstado=toc(reloj);
                elseif tipoMotor=="invalido"
                    reg.Invalidas=reg.Invalidas+1;
                    ultimaNoticia="Estado MOTOR mal formado; habilitacion bloqueada";
                    ultimoEstado=-Inf;
                elseif ~startsWith(linea,'# CSV:')
                    ultimaNoticia=string(linea);
                end
                reg.evento(linea);
            elseif tipo=="muestra" && ~soloIdentidad
                reg.muestra(fila,linea); ultimoDato=toc(reloj);
            elseif tipo=="invalida"
                reg.Invalidas=reg.Invalidas+1; reg.evento("RX_INVALIDA "+string(linea));
            end
        end
        fragmento=fragmento(primero:end);
    end
    function enviar(tipo,valores,anotar)
        assert(identificado,'No se envian comandos sin identidad manual verificada.');
        comando=comando_manual(tipo,valores);
        if tipo=="!"
            write(reg.Serial,uint8('!'),'uint8');
        else
            writeline(reg.Serial,comando);
        end
        if anotar
            reg.evento("TX "+string(comando)); ultimaNoticia="Enviado: "+string(comando);
        end
    end
    function accion(tipo,valores)
        % Solo se llama desde callbacks de una accion explicita del usuario.
        try
            if tipo=="A"
                assert(motor.manual_build==1 && checkFrente.Value==1 && motor.verify==1 && motor.state=="DISARMED" && ...
                    motor.faults==0 && toc(reloj)-ultimoEstado<1, ...
                    'Marque/verifique frente (V 1), libere fallo y espere estado DISARMED reciente.');
                motor.state="ARM_REQUESTED";
            elseif tipo=="C"
                assert(motor.manual_build==1 && checkFrente.Value==1 && motor.verify==1 && motor.state=="ARMED" && ...
                    motor.faults==0 && toc(reloj)-ultimoEstado<1, ...
                    'Se requiere estado ARMED reciente. Cada pulso necesita nueva habilitacion.');
                % Validar antes de marcar pendiente; un dato invalido no envia nada.
                comando_manual(tipo,valores);
                motor.state="PULSE_REQUESTED"; botonC.Enable='off';
            elseif tipo=="V" && valores==1
                assert(checkFrente.Value==1,'Marque primero la verificacion fisica del frente.');
            elseif ismember(tipo,["Z","D"])
                checkFrente.Value=0; motor.verify=0;
            end
            enviar(tipo,valores,true);
            if ismember(tipo,["S","!","E"]), ultimoEstado=-Inf; end
        catch ME
            ultimaNoticia=string(ME.message); reg.evento("ACCION_RECHAZADA "+ultimaNoticia);
        end
        actualizarPermisos();
    end
    function crearVentana()
        reg.Figura=figure('Name','Prueba manual L298N: sin equilibrio automatico', ...
            'NumberTitle','off','Color','w','Position',[50,40,1200,900], ...
            'CloseRequestFcn',@cerrarVentana);
        estadoUI=uicontrol(reg.Figura,'Style','text','Units','normalized', ...
            'Position',[0.03,0.945,0.94,0.04],'String','Motores sin habilitar', ...
            'FontWeight','bold','FontSize',11,'BackgroundColor','w');
        titulos={'Posicion (m)','Velocidad (m/s)','Angulo (grados)','Velocidad angular (grados/s)'};
        for q=1:4
            eje=axes(reg.Figura,'Position',[0.07,0.825-(q-1)*0.125,0.88,0.09]);
            trazos(q)=plot(eje,NaN,NaN); grid(eje,'on'); ylabel(eje,titulos{q});
        end
        xlabel(eje,'Tiempo MCU de la ventana (s)');
        boton('S: PARADA ENCLAVADA',[.03,.375,.22,.045],@(~,~) accion("S",[]),[.8,0,0]);
        boton('! inmediato',[.27,.375,.14,.045],@(~,~) accion("!",[]),[.8,0,0]);
        boton('E CLEAR: liberar fallo',[.43,.375,.23,.045],@(~,~) accion("E",[]),[0,0,0]);
        boton('Terminar y guardar',[.69,.375,.27,.045],@terminar,[0,0,0]);
        boton('Z: vertical ARRIBA ahora',[.03,.315,.25,.04],@(~,~) accion("Z",[]),[0,0,0]);
        boton('N: origen del carro',[.30,.315,.20,.04],@(~,~) accion("N",[]),[0,0,0]);
        etiquetas={'Signo L (rueda derecha)','Signo R (rueda izquierda)','Signo theta'};
        for q=1:3
            xx=.03+(q-1)*.20;
            uicontrol(reg.Figura,'Style','text','Units','normalized','Position',[xx,.28,.19,.025], ...
                'String',etiquetas{q},'BackgroundColor','w','HorizontalAlignment','left');
            selectores(q)=uicontrol(reg.Figura,'Style','popupmenu','Units','normalized', ...
                'Position',[xx,.25,.15,.03],'String',{'+1','-1'});
        end
        boton('D: aplicar signos',[.66,.25,.30,.04],@aplicarSignos,[0,0,0]);
        checkFrente=uicontrol(reg.Figura,'Style','checkbox','Units','normalized', ...
            'Position',[.03,.20,.66,.035],'BackgroundColor','w','Value',0, ...
            'String','Frente marcado; avance, retroceso y angulo comprobados A MANO', ...
            'Callback',@cambiarVerificacion);
        botonV=boton('V 1: confirmar verificacion',[.70,.20,.26,.035], ...
            @(~,~) accion("V",1),[0,0,0]);
        campoL=entrada('PWM izquierdo (%)',.03,'-35');
        campoR=entrada('PWM derecho (%)',.23,'-35');
        campoDuracion=entrada('Duracion (50..150 ms)',.43,'100');
        botonA=boton('A: habilitar 5 s',[.64,.12,.15,.04],@(~,~) accion("A",[]),[0,0,0]);
        botonC=boton('C: UN pulso',[.81,.12,.15,.04],@pulso,[.7,.2,0]);
        uicontrol(reg.Figura,'Style','text','Units','normalized','Position',[.03,.08,.94,.03], ...
            'String','PWM negativo = avance historico. Cada pulso se desarma; esperar 1 s antes de habilitar otro.', ...
            'BackgroundColor','w','HorizontalAlignment','left');
        avisoUI=uicontrol(reg.Figura,'Style','text','Units','normalized','Position',[.03,.015,.94,.06], ...
            'String','Sin habilitacion ni pulsos automaticos. Comenzar con ruedas levantadas y sujecion.', ...
            'BackgroundColor','w','HorizontalAlignment','left');
        actualizarPermisos(); drawnow;
    end
    function h=boton(texto,pos,callback,color)
        h=uicontrol(reg.Figura,'Style','pushbutton','String',texto,'Units','normalized', ...
            'Position',pos,'Callback',callback,'ForegroundColor',color, ...
            'Interruptible','off','BusyAction','cancel');
    end
    function h=entrada(texto,x,valor)
        uicontrol(reg.Figura,'Style','text','Units','normalized','Position',[x,.155,.19,.025], ...
            'String',texto,'BackgroundColor','w','HorizontalAlignment','left');
        h=uicontrol(reg.Figura,'Style','edit','Units','normalized','Position',[x,.12,.16,.03], ...
            'String',valor,'BackgroundColor','w');
    end
    function aplicarSignos(~,~)
        accion("D",3-2*[selectores.Value]);
    end
    function pulso(~,~)
        accion("C",[str2double(campoL.String),str2double(campoR.String),str2double(campoDuracion.String)]);
    end
    function cambiarVerificacion(~,~)
        if checkFrente.Value==0
            accion("S",[]); accion("V",0); motor.verify=0;
        end
        actualizarPermisos();
    end
    function actualizarPermisos()
        if isempty(checkFrente) || ~isgraphics(checkFrente), return; end
        reciente=toc(reloj)-ultimoEstado<1;
        verificado=checkFrente.Value==1 && motor.verify==1;
        botonV.Enable=onoff(checkFrente.Value==1);
        botonA.Enable=onoff(motor.manual_build==1 && reciente && verificado && motor.state=="DISARMED" && motor.faults==0);
        botonC.Enable=onoff(motor.manual_build==1 && reciente && verificado && motor.state=="ARMED" && motor.faults==0);
    end
    function dibujar()
        if ~isgraphics(reg.Figura), detener=true; return; end
        if reg.N>0
            idx=max(1,reg.N-1999):reg.N;
            tiempos=reg.Datos(idx,2); tt=[0;cumsum(mod(diff(tiempos),2^32))]*1e-6;
            estados=reg.Datos(idx,6:9);
            estados(bitand(uint32(reg.Datos(idx,12)),uint32(3))~=0,3:4)=NaN;
            estados(:,3:4)=rad2deg(estados(:,3:4));
            for q=1:4, set(trazos(q),'XData',tt,'YData',estados(:,q)); end
            if ~signosInicializados
                for q=1:3, selectores(q).Value=1+(reg.Datos(reg.N,22+q)<0); end
                signosInicializados=true;
            end
            reg.Figura.Name=sprintf('Manual | n=%d | nL=%d nR=%d | flags=%d | PWM=%d,%d', ...
                reg.N,reg.Datos(reg.N,4:5),reg.Datos(reg.N,12:14));
        end
        actualizarPermisos();
        estadoUI.String=sprintf('Motor: %s | Perfil manual: %d | Fallos: %.0f | Verificacion: %d | F ideal: %.3f N', ...
            motor.state,motor.manual_build,motor.faults,motor.verify,motor.F_N);
        avisoUI.String=sprintf('%s\nMax. lazo %.0f us / ciclo %.0f us. F ideal NO es PWM. Control de equilibrio deshabilitado.', ...
            ultimaNoticia,motor.max_loop_us,motor.max_cycle_us);
        drawnow limitrate;
    end
    function terminar(~,~)
        detener=true;
    end
    function cerrarVentana(~,~)
        detener=true;
        try
            enviar("!",[],true);
        catch
            % USB may already be disconnected; MCU lease and watchdog remain.
        end
        delete(reg.Figura);
    end
end

function limpieza=limpiezaRegistro(reg)
% Captura el objeto por valor en un scope sin callbacks anidados compartidos.
limpieza=onCleanup(@() reg.cerrar());
end

function texto=onoff(valor)
if valor, texto='on'; else, texto='off'; end
end
