function prueba_estados()
% Pruebas sinteticas del protocolo, rollover y criterios de aceptacion.
% No abre puertos ni modifica hardware.
N = 1001;
d = zeros(N,27);
d(:,1) = mod(2^32-3+(0:N-1)',2^32);
d(:,2) = mod(2^32-8000+(0:N-1)'*5000,2^32);
d(:,3) = 5000;
d(:,10) = 1000;
d(:,11) = 32;
d(:,20) = 250;
d(:,21) = 2;
d(:,22) = 1000;
d(:,23:25) = 1;
linea = strjoin(arrayfun(@(x) sprintf('%.17g',x),d(1,:), ...
    'UniformOutput',false),',');
[r,tipo] = decodificar_estado(linea);
assert(tipo == "muestra" && isequal(r,d(1,:)));
[~,tipo] = decodificar_estado('# PENDULO_STATES_V1 motores bloqueados');
assert(tipo == "comentario");
[~,tipo] = decodificar_estado('1,2,3');
assert(tipo == "invalida");
sinSensor = d(1,:); sinSensor([10,11,22]) = -1; sinSensor(12) = 3;
sinSensor(8:9) = NaN;
[r,tipo] = decodificar_estado(strjoin(arrayfun(@(x) sprintf('%.17g',x), ...
    sinSensor,'UniformOutput',false),','));
assert(tipo == "muestra" && isnan(r(8)) && r(22) == -1);
[~,tipo] = decodificar_estado(strrep(linea,',1000,',',1x,'));
assert(tipo == "invalida");
[q,t] = validar_estados(d);
assert(q.cuatro_estados_listos && abs(t(end)-5) < 1e-12);
assert(q.intervalos_perdidos_secuencia == 0);
roto = d; roto(10,:) = [];
q = validar_estados(roto);
assert(q.intervalos_perdidos_secuencia == 1 && ~q.aceptacion_temporal);
roto = d; roto(50,1:2) = roto(49,1:2);
q = validar_estados(roto);
assert(q.secuencia_duplicada_o_reiniciada && q.reloj_reiniciado);
roto = d; roto(:,12) = 2;
q = validar_estados(roto);
assert(q.aceptacion_temporal && ~q.cuatro_estados_listos && q.muestras_sin_cero == N);
roto = d; roto(30,12) = 1; roto(30,8:9) = NaN;
q = validar_estados(roto);
assert(q.muestras_sensor_invalido == 1 && ~q.cuatro_estados_listos);
roto = d; roto(30,8) = NaN;
q = validar_estados(roto);
assert(q.muestras_no_finitas_con_sensor_valido == 1 && ~q.cuatro_estados_listos);
roto = d; roto(30:end,18) = 1;
q = validar_estados(roto);
assert(q.incremento_tx_drop == 1 && ~q.aceptacion_temporal);
roto = d; roto(30,13) = 1;
q = validar_estados(roto);
assert(q.pwm_no_cero && ~q.cuatro_estados_listos);
fprintf('prueba_estados: parser, rollover, perdidas, flags, NaN y PWM correctos.\n');
end
