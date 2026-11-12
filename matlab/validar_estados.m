function [resumen, tiempo_s] = validar_estados(datos)
% Validacion offline del transporte, tiempos y estados. No valida la mecanica.
% Acepta una tabla del esquema o una matriz numerica N x 27.
if istable(datos)
    datos = datos{:, esquema_estados()};
end
assert(isnumeric(datos) && size(datos,2) == 27, 'Se requieren 27 columnas.');
n = size(datos,1);
tiempo_s = zeros(n,1);
resumen = struct('muestras',n,'intervalos_perdidos_secuencia',0, ...
    'secuencia_duplicada_o_reiniciada',false,'reloj_reiniciado',false, ...
    'dt_mediana_us',NaN,'dt_min_us',NaN,'dt_max_us',NaN, ...
    'dt_p99_error_us',NaN,'diferencia_timestamp_dt_max_us',NaN, ...
    'muestras_sensor_invalido',0,'muestras_sin_cero',0, ...
    'muestras_sin_sentidos',0,'muestras_emergencia',0, ...
    'muestras_no_finitas_con_sensor_valido',0,'pwm_no_cero',false, ...
    'incremento_invalidL',0,'incremento_invalidR',0, ...
    'incremento_missed',0,'incremento_tx_drop',0,'incremento_i2c_errors',0, ...
    'contadores_finales',[],'acq_max_us',NaN,'late_max_us',NaN, ...
    'aceptacion_temporal',false,'cuatro_estados_listos',false);
if n == 0
    return
end
flags = uint32(datos(:,12));
sensorOK = bitand(flags,uint32(1)) == 0;
ceroOK = bitand(flags,uint32(2)) == 0;
sentidosOK = bitand(flags,uint32(4)) == 0;
emergencia = bitand(flags,uint32(8)) ~= 0;
dt = datos(:,3);
if n > 1
    ds = mod(diff(datos(:,1)),2^32);
    du = mod(diff(datos(:,2)),2^32);
    valido = ds > 0 & ds < 2^31;
    resumen.intervalos_perdidos_secuencia = sum(ds(valido)-1);
    resumen.secuencia_duplicada_o_reiniciada = any(~valido);
    resumen.reloj_reiniciado = any(du == 0 | du >= 2^31);
    % Un reinicio no se interpreta como una captura de 71 minutos.
    du(du >= 2^31 | du == 0) = NaN;
    tiempo_s = [0; cumsum(du)] * 1e-6;
    contiguo = ds == 1 & isfinite(du);
    dif = abs(du(contiguo)-dt([false;contiguo]));
    if ~isempty(dif)
        resumen.diferencia_timestamp_dt_max_us = max(dif);
    end
end
% La primera fila puede ser la primera adquisicion tras arrancar el MCU.
dt = dt(2:end);
if ~isempty(dt)
    resumen.dt_mediana_us = median(dt);
    resumen.dt_min_us = min(dt);
    resumen.dt_max_us = max(dt);
    err = sort(abs(dt-5000));
    resumen.dt_p99_error_us = err(max(1,ceil(0.99*numel(err))));
end
resumen.muestras_sensor_invalido = sum(~sensorOK);
resumen.muestras_sin_cero = sum(~ceroOK);
resumen.muestras_sin_sentidos = sum(~sentidosOK);
resumen.muestras_emergencia = sum(emergencia);
resumen.muestras_no_finitas_con_sensor_valido = ...
    sum(sensorOK & ceroOK & any(~isfinite(datos(:,6:9)),2));
resumen.pwm_no_cero = any(datos(:,13:14) ~= 0,'all');
etiquetas = {'incremento_invalidL','incremento_invalidR','incremento_missed', ...
    'incremento_tx_drop','incremento_i2c_errors'};
for k = 1:5
    incremento = mod(diff(datos(:,14+k)),2^32);
    % Un descenso distinto de rollover se marca como discontinuidad.
    if any(incremento >= 2^31)
        resumen.(etiquetas{k}) = NaN;
    else
        resumen.(etiquetas{k}) = sum(incremento);
    end
end
resumen.contadores_finales = datos(end,15:19);
resumen.acq_max_us = max(datos(:,20));
resumen.late_max_us = max(datos(:,21));
% Umbral de esta etapa: +/-100 us en todos los periodos de 5 ms, adquisicion
% <1 ms y cero perdidas. Ajustarlo exige analizar los datos, no ocultarlos.
resumen.aceptacion_temporal = n >= 1000 && all(abs(dt-5000) <= 100) && ...
    resumen.intervalos_perdidos_secuencia == 0 && ...
    ~resumen.secuencia_duplicada_o_reiniciada && ~resumen.reloj_reiniciado && ...
    resumen.diferencia_timestamp_dt_max_us <= 2 && ...
    resumen.incremento_missed == 0 && resumen.incremento_tx_drop == 0 && ...
    resumen.acq_max_us < 1000 && resumen.late_max_us <= 100;
resumen.cuatro_estados_listos = resumen.aceptacion_temporal && ...
    all(sensorOK & ceroOK & sentidosOK) && ...
    all(isfinite(datos(:,6:9)),'all') && ~resumen.pwm_no_cero && ...
    resumen.incremento_invalidL == 0 && resumen.incremento_invalidR == 0 && ...
    resumen.incremento_i2c_errors == 0;
end
