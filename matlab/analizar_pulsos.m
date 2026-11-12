function R = analizar_pulsos(origen, carpetaSalida, mostrar)
% ANALIZAR_PULSOS Analisis offline de CSV/MAT manual de 27 columnas. No usa COM.
% R=analizar_pulsos("registros/manual_....csv")
% R=analizar_pulsos("registros/manual_....mat","registros/analisis",false)
% Tambien acepta tabla/matriz numerica para pruebas. No identifica fuerza,
% ganancia estatica ni tau. Los bordes PWM se observan a resolucion de 5 ms.
arguments
    origen
    carpetaSalida (1,1) string = ""
    mostrar (1,1) logical = true
end
nombres=esquema_estados();
[d,fuente]=leerOrigen(origen,nombres);
R=struct('estado',"datos insuficientes",'fuente',fuente,'causas',strings(0,1), ...
    'pulsos',table(),'series',table(),'giro_libre',{{}},'figuras',gobjects(0), ...
    'archivos',strings(0,1),'nota', ...
    "Pulsos cortos no validan ganancia estatica, tau ni conversion N/PWM. Giro libre no es fuerza de traccion.");
if size(d,1)<3
    R.causas="Se requieren al menos tres muestras."; informar(R); return
end
N=size(d,1); U=2^32;
enteros=[1:5,10:25];
if any(~isfinite(d(:,enteros)),'all') || any(fix(d(:,enteros))~=d(:,enteros),'all')
    R.estado="datos rechazados";R.causas="Campos enteros invalidos.";informar(R);return
end
ds=mod(diff(d(:,1)),U); du=mod(diff(d(:,2)),U);
causas=strings(0,1);
if any(ds~=1), causas(end+1)="Secuencia con huecos, duplicados o reinicio."; end
if any(du<4900 | du>5100) || any(abs(du-d(2:end,3))>2)
    causas(end+1)="Temporizacion fuera de 5000 +/-100 us o dt incoherente.";
end
if any(d(:,20)>1500 | d(:,20)<0 | d(:,21)>100 | d(:,21)<0)
    causas(end+1)="Adquisicion o retraso del tick fuera del limite.";
end
if any(mod(diff(d(:,15:19)),U)~=0,'all')
    causas(end+1)="Aumentaron errores de encoder/I2C o perdidas missed/tx_drop.";
end
flags=uint32(d(:,12));
if any(bitand(flags,uint32(1))~=0) || any(d(:,10)<0 | d(:,10)>4095) || ...
        any(d(:,11)<0 | d(:,11)>255) || any(bitand(uint32(d(:,11)),uint32(56))~=32)
    causas(end+1)="Lecturas AS5600 invalidas.";
end
if any(~ismember(d(:,23:25),[-1,1]),'all') || any(abs(d(:,13:14))>50,'all')
    causas(end+1)="Signos fuera de +/-1 o PWM fuera del limite manual.";
end
if ~isempty(causas)
    R.estado="datos rechazados";R.causas=causas;informar(R);return
end
t=[0;cumsum(du)]*1e-6;
% Diferencia modular int32; evita saltos en 2^31 y 2^32 de los contadores.
dn=mod(diff(d(:,4:5))+2^31,U)-2^31;
cuentas=[zeros(1,2);cumsum(dn,1)];
dt=du*1e-6;
rpmL=[NaN;dn(:,1).*d(2:end,23)./dt]*60/1969.6; % rueda DERECHA
rpmR=[NaN;dn(:,2).*d(2:end,24)./dt]*60/1974;   % rueda IZQUIERDA
paso=2*pi*0.0325./[1969.6,1974];
vxRuedas=[NaN;sum(dn.*d(2:end,23:24).*paso,2)./(2*dt)];
R.series=table(t,d(:,13),d(:,14),rpmR,rpmL,d(:,6),d(:,7),vxRuedas, ...
    'VariableNames',{'t_s','pwm_izq','pwm_der','rpm_izq_R','rpm_der_L', ...
    'x_m','vx_filtrada_m_s','vx_ruedas_m_s'});
activo=any(d(:,13:14)~=0,2);
inicios=find(activo & [true;~activo(1:end-1)]);
finales=find(activo & [~activo(2:end);true])+1; % primera muestra con PWM cero
if isempty(inicios)
    R.causas="No se observaron pulsos PWM; no se ajusta ningun modelo.";
    informar(R);return
end
filas=cell(numel(inicios),1); curvas=cell(numel(inicios),1);
for k=1:numel(inicios)
    i=inicios(k); j=finales(k); completo=i>1 && j<=N;
    f=struct('pulso',k,'valido',false,'motivo',"",'t_inicio_observado_s',t(i), ...
        'pwm_izq',d(i,13),'pwm_der',d(i,14),'duracion_observada_ms',NaN, ...
        'duracion_min_ms',NaN,'duracion_max_ms',NaN,'reposo_previo_100ms',false, ...
        'delta_nL',NaN,'delta_nR',NaN,'recorrido_izq_R_m',NaN, ...
        'recorrido_der_L_m',NaN,'delta_x_m',NaN,'retardo_izq_ms',NaN, ...
        'retardo_der_ms',NaN,'rpm_pico_abs_izq',NaN,'rpm_pico_abs_der',NaN, ...
        'vx_pico_abs_m_s',NaN,'giro_libre_observado_s',NaN, ...
        'recorrido_giro_libre_m',NaN,'fallo_en_ventana',false);
    if ~completo
        f.motivo="Pulso parcial: falta muestra cero anterior o posterior.";
        filas{k}=f;continue
    end
    ultimo=find(t<=t(j)+1.0,1,'last');
    if k<numel(inicios), ultimo=min(ultimo,inicios(k+1)-1); end
    w=(i-1):ultimo;
    if t(j)-t(i)>0.155+1e-9
        f.motivo="Duracion superior al limite manual de 150 ms mas una muestra.";
    elseif any(d(i:j-1,13:14)~=d(i,13:14),'all')
        f.motivo="Cambio de PWM dentro del tramo activo sin intervalo cero.";
    elseif any(bitand(flags(w),uint32(6))~=0) || ...
            any(~isfinite(d(w,6:9)),'all') || any(d(w,22)<0 | d(w,22)>4095)
        f.motivo="Faltan cero/signos o estados finitos en la ventana del pulso.";
    elseif any(d(w,22:25)~=d(i,22:25),'all')
        f.motivo="Cambio de calibracion dentro del pulso/giro libre.";
    else
        dxEsperado=sum(dn(w(1):w(end)-1,:).*d(w(2:end),23:24).*paso,2)/2;
        if any(abs(diff(d(w,6))-dxEsperado)>2e-5)
            f.motivo="Discontinuidad del origen x o conversion de ruedas incoherente.";
        end
    end
    if strlength(f.motivo)>0, filas{k}=f;continue;end
    f.valido=true;f.motivo="Datos validos; no identifica dinamica ni fuerza.";
    f.duracion_observada_ms=(t(j)-t(i))*1000;
    f.duracion_min_ms=(t(j-1)-t(i))*1000;
    f.duracion_max_ms=(t(j)-t(i-1))*1000;
    deltas=cuentas(j,:)-cuentas(i-1,:);
    f.delta_nL=deltas(1);f.delta_nR=deltas(2);
    recorridos=deltas.*d(i,23:24).*paso;
    f.recorrido_izq_R_m=recorridos(2);f.recorrido_der_L_m=recorridos(1);
    f.delta_x_m=d(j,6)-d(i-1,6);
    pre=max(1,i-21):(i-1);
    f.reposo_previo_100ms=t(i-1)-t(pre(1))>=0.099 && ...
        all(d(pre,13:14)==0,'all') && all(max(cuentas(pre,:),[],1)-min(cuentas(pre,:),[],1)<=2);
    if f.reposo_previo_100ms
        if f.pwm_izq~=0
            f.retardo_izq_ms=retardo(cuentas(:,2),i,j,ultimo,t);
        end
        if f.pwm_der~=0
            f.retardo_der_ms=retardo(cuentas(:,1),i,j,ultimo,t);
        end
    end
    f.rpm_pico_abs_izq=max(abs(rpmR(i:j)));
    f.rpm_pico_abs_der=max(abs(rpmL(i:j)));
    f.vx_pico_abs_m_s=max(abs(vxRuedas(i:j)));
    f.giro_libre_observado_s=t(ultimo)-t(j);
    f.recorrido_giro_libre_m=d(ultimo,6)-d(j,6);
    f.fallo_en_ventana=any(bitand(flags(w),uint32(8))~=0);
    c=j:ultimo;
    curvas{k}=table(t(c)-t(j),rpmR(c),rpmL(c),vxRuedas(c),d(c,7),d(c,6)-d(j,6), ...
        'VariableNames',{'t_desde_corte_s','rpm_izq_R','rpm_der_L', ...
        'vx_ruedas_m_s','vx_filtrada_m_s','x_desde_corte_m'});
    filas{k}=f;
end
R.pulsos=struct2table(vertcat(filas{:}));R.giro_libre=curvas;
validos=R.pulsos.valido;
if ~any(validos)
    R.estado="datos insuficientes";R.causas=unique(R.pulsos.motivo);
elseif all(validos)
    R.estado="pulsos analizados";
else
    R.estado="analisis parcial";R.causas=unique(R.pulsos.motivo(~validos));
end
if any(validos) && (mostrar || strlength(carpetaSalida)>0)
    R.figuras=graficas(R,mostrar);
end
if strlength(carpetaSalida)>0
    if ~isfolder(carpetaSalida),mkdir(carpetaSalida);end
    tabla=fullfile(carpetaSalida,'resumen_pulsos.csv');writetable(R.pulsos,tabla);
    R.archivos(end+1)=tabla;
    for k=1:numel(R.figuras)
        nombre=fullfile(carpetaSalida,sprintf('pulsos_%d.png',k));
        exportgraphics(R.figuras(k),nombre,'Resolution',150);
        R.archivos(end+1)=nombre;
    end
    if ~mostrar,close(R.figuras);R.figuras=gobjects(0);end
end
informar(R);
end

function valor=retardo(cuentas,i,~,ultimo,t)
% Umbral >2 cuentas acumuladas desde muestra previa, no retardo electrico puro.
q=find(abs(cuentas(i:ultimo)-cuentas(i-1))>2,1,'first');
valor=NaN;
if ~isempty(q),valor=(t(i+q-1)-t(i))*1000;end
end

function [d,fuente]=leerOrigen(origen,nombres)
fuente="datos en memoria";
if ischar(origen) || isstring(origen)
    fuente=string(origen);[~,~,ext]=fileparts(fuente);
    switch lower(ext)
        case '.csv'
            origen=readtable(fuente,'VariableNamingRule','preserve');
        case '.mat'
            a=load(fuente);
            if isfield(a,'T')
                origen=a.T;
            elseif isfield(a,'datos')
                origen=a.datos;
            else
                error('Pendulo:Formato','El MAT necesita T o datos.');
            end
        otherwise,error('Pendulo:Formato','Se requiere CSV, MAT, tabla o matriz.');
    end
end
if istable(origen)
    assert(all(ismember(nombres,origen.Properties.VariableNames)), ...
        'Pendulo:Formato','Faltan columnas del protocolo de 27 campos.');
    origen=origen{:,nombres};
end
assert(isnumeric(origen)&&ismatrix(origen)&&size(origen,2)==27, ...
    'Pendulo:Formato','Se requieren 27 columnas numericas.');
d=double(origen);
end

function f=graficas(R,mostrar)
visible='off';if mostrar,visible='on';end
s=R.series;
f(1)=figure('Name','Pulsos manuales: medidas, sin modelo de fuerza','Color','w','Visible',visible);
tiledlayout(f(1),4,1);
nexttile;stairs(s.t_s,[s.pwm_izq,s.pwm_der]);ylabel('% PWM');legend('Izquierdo','Derecho');grid on;
nexttile;plot(s.t_s,[s.rpm_izq_R,s.rpm_der_L]);ylabel('RPM');legend('Izq: encoder R','Der: encoder L');grid on;
nexttile;plot(s.t_s,[s.vx_ruedas_m_s,s.vx_filtrada_m_s]);ylabel('m/s');legend('Ruedas sin filtro','vx filtrada');grid on;
nexttile;plot(s.t_s,s.x_m);ylabel('x (m)');xlabel('Tiempo MCU (s)');grid on;
f(2)=figure('Name','Giro libre: curvas observadas, no fuerza de traccion','Color','w','Visible',visible);
ax=axes(f(2));hold(ax,'on');
for k=1:numel(R.giro_libre)
    c=R.giro_libre{k};if isempty(c),continue;end
    plot(ax,c.t_desde_corte_s,c.vx_ruedas_m_s,'DisplayName',sprintf('Pulso %d',k));
end
xlabel(ax,'Tiempo desde primera muestra PWM cero (s)');ylabel(ax,'vx de ruedas (m/s)');
title(ax,'Hasta 1 s de giro libre; no se infiere fuerza');grid(ax,'on');legend(ax,'show');
end

function informar(R)
fprintf('analizar_pulsos: %s\n',R.estado);
if ~isempty(R.pulsos),disp(R.pulsos);end
for k=1:numel(R.causas),fprintf('  %s\n',R.causas(k));end
fprintf('%s\n',R.nota);
end
