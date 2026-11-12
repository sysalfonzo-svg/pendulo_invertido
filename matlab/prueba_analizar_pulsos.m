function prueba_analizar_pulsos()
% Datos sinteticos: no abre COM ni solicita acciones del firmware.
N=300;d=zeros(N,27);
d(:,1)=mod(2^32-30+(0:N-1)',2^32);
d(:,2)=mod(2^32-155000+(0:N-1)'*5000,2^32);
d(:,3)=5000;d(:,10)=2000;d(:,11)=32;d(:,20)=150;d(:,21)=4;
d(:,22)=2000;d(:,23:25)=repmat([1,-1,-1],N,1);
d(31:50,13:14)=-40; % 100 ms observados, ambos motores
dl=zeros(N,1);dr=zeros(N,1);
dl(35:50)=4;dr(37:50)=-6;
dl(51:60)=2;dr(51:60)=-3; % giro libre tras quitar PWM
cl=cumsum(dl);cr=cumsum(dr);
% Cruza int32 positivo->negativo, y cero->uint32 max en rueda opuesta.
d(:,4)=mod(2^31-20+cl+2^31,2^32)-2^31;
d(:,5)=mod(10+cr+2^31,2^32)-2^31;
d(:,6)=0.5*(cl*2*pi*.0325/1969.6-cr*2*pi*.0325/1974);
d(:,7)=[0;diff(d(:,6))/.005];d(:,26)=d(:,7);
R=analizar_pulsos(d,"",false);p=R.pulsos;
assert(R.estado=="pulsos analizados" && height(p)==1 && p.valido);
assert(abs(p.duracion_observada_ms-100)<1e-9);
assert(abs(p.duracion_min_ms-95)<1e-9 && abs(p.duracion_max_ms-105)<1e-9);
assert(p.delta_nL==66 && p.delta_nR==-87);
assert(abs(p.recorrido_izq_R_m-87*2*pi*.0325/1974)<1e-12);
assert(abs(p.recorrido_der_L_m-66*2*pi*.0325/1969.6)<1e-12);
assert(abs(p.retardo_izq_ms-30)<1e-9 && abs(p.retardo_der_ms-20)<1e-9);
assert(abs(R.series.rpm_izq_R(37)-6/.005*60/1974)<1e-9);
assert(abs(R.series.rpm_der_L(35)-4/.005*60/1969.6)<1e-9);
assert(p.recorrido_giro_libre_m>0 && ~isempty(R.giro_libre{1}));
assert(max(abs(R.series.rpm_izq_R),[],'omitnan')<100);
% No pulsos: respuesta explicita, sin intentar ajustar nada.
sin=d;sin(:,13:14)=0;R=analizar_pulsos(sin,"",false);
assert(R.estado=="datos insuficientes" && isempty(R.pulsos));
% Perdida de fila, error I2C, sensor invalido y tiempo incorrecto se rechazan.
mal=d;mal(42,:)=[];R=analizar_pulsos(mal,"",false);assert(R.estado=="datos rechazados");
mal=d;mal(40:end,19)=1;R=analizar_pulsos(mal,"",false);assert(R.estado=="datos rechazados");
mal=d;mal(40,12)=1;R=analizar_pulsos(mal,"",false);assert(R.estado=="datos rechazados");
mal=d;mal(40,3)=6000;R=analizar_pulsos(mal,"",false);assert(R.estado=="datos rechazados");
% Cero/signos pendientes en la ventana impiden evaluar x del pulso.
mal=d;mal(40,12)=2;R=analizar_pulsos(mal,"",false);assert(~R.pulsos.valido);
mal=d;mal(40,12)=4;R=analizar_pulsos(mal,"",false);assert(~R.pulsos.valido);
% Calibracion inicial pendiente, anterior a la ventana, no invalida pulso sano.
mal=d;mal(1:10,12)=6;mal(1:10,8)=NaN;R=analizar_pulsos(mal,"",false);assert(R.pulsos.valido);
% Un N entre el pulso y el giro libre crea una discontinuidad detectada.
mal=d;mal(70:end,6)=mal(70:end,6)-.01;R=analizar_pulsos(mal,"",false);assert(~R.pulsos.valido);
% Corte del archivo mientras aun hay PWM: no medir duracion completa ficticia.
R=analizar_pulsos(d(1:45,:),"",false);assert(~R.pulsos.valido);
% Si ya habia movimiento, no atribuirle un retardo de arranque.
mal=d;mal(20:30,4)=mal(20:30,4)+3;
R=analizar_pulsos(mal,"",false);assert(~R.pulsos.reposo_previo_100ms && isnan(R.pulsos.retardo_der_ms));
% Comprobar lectura de archivos en una carpeta temporal propia.
carpeta=string(tempname);mkdir(carpeta);limpieza=onCleanup(@() borrarTemporales(carpeta));
T=array2table(d,'VariableNames',esquema_estados());
writetable(T,fullfile(carpeta,'pulso.csv'));save(fullfile(carpeta,'pulso.mat'),'T');
RC=analizar_pulsos(fullfile(carpeta,'pulso.csv'),"",false);
RM=analizar_pulsos(fullfile(carpeta,'pulso.mat'),"",false);
assert(isequal(RC.pulsos.Properties.VariableNames,RM.pulsos.Properties.VariableNames));
for nombre=string(RC.pulsos.Properties.VariableNames)
    a=RC.pulsos.(nombre);b=RM.pulsos.(nombre);
    if isnumeric(a)
        % CSV decimal serialization may round the final floating-point bit.
        assert(all((isnan(a)&isnan(b)) | abs(a-b)<1e-10));
    else
        assert(isequaln(a,b));
    end
end
clear limpieza
fprintf('prueba_analizar_pulsos: movimiento, mapeo, rollover, calidad y CSV/MAT correctos.\n');
end

function borrarTemporales(carpeta)
% Only two files created by this test; no recursive deletion.
for nombre=["pulso.csv","pulso.mat"]
    archivo=fullfile(carpeta,nombre);
    if isfile(archivo), delete(archivo); end
end
if isfolder(carpeta), rmdir(carpeta); end
end
