function R = verificar_gui(puerto)
% Prueba real de visualizacion: espera datos antes de capturar la ventana.
% No envia Z/D ni comandos de movimiento. Requiere motores sin alimentacion.
addpath('matlab');
capturada = false;
t = timer('ExecutionMode','fixedSpacing','Period',0.5,'TimerFcn',@capturar);
limpieza = onCleanup(@() cerrarTimer(t));
start(t);
R = medir_estados(puerto,10,'registros',true);
cerrarTimer(t);
assert(capturada,'No se pudo capturar la ventana con datos.');
assert(R.resumen.aceptacion_temporal,'No cumple la temporizacion con graficas.');
disp(R.archivos);

    function capturar(~,~)
        if capturada
            return
        end
        figuras = findall(groot,'Type','figure');
        for f = figuras'
            if ~startsWith(string(f.Name),"Cuatro estados |")
                continue
            end
            trazos = findall(f,'Type','line');
            if isempty(trazos) || ~all(arrayfun(@(l) numel(l.XData)>20,trazos))
                continue
            end
            imagen = getframe(f);
            imwrite(imagen.cdata,'registros/ventana_estados_con_datos.png');
            capturada = true;
            return
        end
    end
end

function cerrarTimer(t)
if isvalid(t)
    stop(t);
    delete(t);
end
end
