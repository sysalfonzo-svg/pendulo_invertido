classdef RegistroManual < handle
    % Registro y cierre independiente del workspace de callbacks de la GUI.
    properties
        Serial = []
        Figura = []
        FidCSV = -1
        FidEventos = -1
        Datos
        N = 0
        Invalidas = 0
        Comentarios = strings(0,1)
        Archivos
        Meta
        Nombres
        Cerrado = false
        Resultado = struct()
    end
    methods
        function obj=RegistroManual(puerto,duracion,carpeta)
            if ~isfolder(carpeta), mkdir(carpeta); end
            inicio=datetime('now','TimeZone','America/Bogota');
            base=fullfile(carpeta,"manual_"+string(inicio,'yyyyMMdd_HHmmss_SSS'));
            obj.Archivos=struct('csv',base+".csv",'mat',base+".mat",'eventos',base+"_eventos.txt");
            obj.Nombres=esquema_estados();
            obj.Datos=NaN(max(2000,ceil(min(duracion,600)*220)),27);
            obj.Meta=struct('protocolo',"PENDULO_STATES_V1; MOTOR_MANUAL_V1", ...
                'puerto',puerto,'inicio',inicio,'duracion_solicitada_s',duracion, ...
                'firmware_identificado',false,'error',"",'Ts_objetivo_s',0.005, ...
                'orden_estados',{{'x_m','v_m_s','theta_rad','omega_rad_s'}}, ...
                'PWM_limite_interfaz',50,'duracion_pulso_max_ms',150, ...
                'heartbeat_objetivo_s',0.05, ...
                'nota',"A, C, E, V, Z, N y D solo por accion explicita; no equilibrio automatico.");
            try
                obj.FidCSV=fopen(obj.Archivos.csv,'wt');
                obj.FidEventos=fopen(obj.Archivos.eventos,'wt');
                assert(obj.FidCSV>=0 && obj.FidEventos>=0,'No se pueden crear registros.');
                fprintf(obj.FidCSV,'%s\n',strjoin(obj.Nombres,','));
            catch ME
                obj.Meta.error=string(ME.message);
                obj.cerrar();
                rethrow(ME);
            end
        end
        function muestra(obj,fila,linea)
            obj.N=obj.N+1;
            if obj.N>size(obj.Datos,1), obj.Datos(end+20000,27)=NaN; end
            obj.Datos(obj.N,:)=fila;
            fprintf(obj.FidCSV,'%s\n',linea);
        end
        function evento(obj,texto)
            marca=string(datetime('now','TimeZone','America/Bogota', ...
                'Format','yyyy-MM-dd HH:mm:ss.SSS XXX'));
            linea=marca+" "+string(texto);
            obj.Comentarios(end+1,1)=linea;
            if obj.FidEventos>=0, fprintf(obj.FidEventos,'%s\n',linea); end
        end
        function cerrar(obj)
            if obj.Cerrado, return; end
            obj.Cerrado=true;
            % Los comandos de salida solo detienen; nunca habilitan ni mueven.
            try
                if ~isempty(obj.Serial) && isvalid(obj.Serial)
                    writeline(obj.Serial,'H'); writeline(obj.Serial,'S');
                    obj.evento('TX H y S al cerrar');
                end
            catch ME
                obj.evento("Cierre serie: "+string(ME.message));
            end
            try
                if ~isempty(obj.Serial) && isvalid(obj.Serial), delete(obj.Serial); end
            catch
            end
            obj.Serial=[];
            for fid=[obj.FidCSV,obj.FidEventos]
                try
                    if fid>=0 && ~isempty(fopen(fid)), fclose(fid); end
                catch
                end
            end
            obj.FidCSV=-1; obj.FidEventos=-1;
            if isscalar(obj.Figura) && isgraphics(obj.Figura), delete(obj.Figura); end
            datos=obj.Datos(1:obj.N,:);
            T=array2table(datos,'VariableNames',obj.Nombres);
            [resumen,tiempo_s]=validar_estados(datos);
            resumen.lineas_invalidas=obj.Invalidas;
            resumen.pwm_fuera_limite_manual=any(abs(datos(:,13:14))>50,'all');
            resumen.adquisicion_manual_valida=resumen.aceptacion_temporal && ...
                obj.Invalidas==0 && resumen.muestras_sensor_invalido==0 && ...
                resumen.incremento_i2c_errors==0 && ~resumen.pwm_fuera_limite_manual;
            % cuatro_estados_listos de validar_estados exige PWM cero; aqui los
            % pulsos son intencionales. La aceptacion mecanica sigue siendo manual.
            obj.Meta.fin=datetime('now','TimeZone','America/Bogota');
            obj.Meta.comentarios_y_comandos=obj.Comentarios;
            meta=obj.Meta;
            try
                save(obj.Archivos.mat,'T','datos','tiempo_s','resumen','meta');
            catch ME
                warning('Pendulo:Guardar','No se pudo guardar MAT: %s',ME.message);
            end
            obj.Resultado=struct('datos',T,'tiempo_s',tiempo_s,'resumen',resumen, ...
                'meta',meta,'archivos',obj.Archivos);
        end
    end
end
