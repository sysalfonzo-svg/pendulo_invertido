function archivo = prueba_lqr(puerto, signos)
% PRUEBA_LQR Primer intento EXPERIMENTAL de equilibrio maximo 1.2 segundos.
% Se requiere MATLAB con serialport y USB CDC operativo en la Black Pill.
% Uso: archivo=prueba_lqr("COM15",[signL signR signTheta]);
% IMPORTANTE: no colocar signos supuestos. Verificar fisicamente y conservar
% los signos D que ya funcionaron durante la etapa de adquisicion.
% El firmware exige A AUTO + T y latido K cada <250 ms, ademas de condiciones
% de sensor, centrado, angulo, signos y seguridad. No es swing-up.

if nargin < 2
    error('Uso: prueba_lqr("COM15",[signL signR signTheta]); no inventar signos.');
end
assert(isnumeric(signos) && numel(signos)==3 && all(abs(signos(:))==1), ...
    'Los signos deben ser [1/-1, 1/-1, 1/-1] confirmados experimentalmente.');
puerto = char(string(puerto));

s = serialport(puerto,115200,'Timeout',0.05);
limpieza = onCleanup(@() detenerSeguro(s)); %#ok<NASGU>
configureTerminator(s,'LF');
pause(0.5); flush(s);
writeline(s,'?');
[identidad,~] = recibirHasta(s,'LQR_EXPERIMENTAL_V1',1.0);
assert(identidad,'El dispositivo no anuncia LQR_EXPERIMENTAL_V1. No se enviaron movimientos.');
writeline(s,'H');pause(0.1);

fprintf('Mantenga la varilla vertical ARRIBA, inmovil, con el carro centrado.\n');
input('Presione ENTER para aplicar el cero y los signos ya verificados...','s');
writeline(s,sprintf('D %d %d %d',signos(1),signos(2),signos(3)));
pause(0.15);
writeline(s,'N');
pause(0.15);
writeline(s,'Z');
[cero,~] = recibirHasta(s,'ZERO_OK',1.0);
assert(cero,'No se confirmo ZERO_OK. Revise el AS5600 y la inmovilidad.');

respuesta = input(['Confirme que D coincide con la comprobacion fisica previa ' ...
    'de las ruedas y theta. Escriba SI: '],'s');
if ~strcmpi(strtrim(respuesta),'SI')
    error('Cancelado: signos no confirmados. Motores apagados.');
end
writeline(s,'V 1');
[verificado,~] = recibirHasta(s,'VERIFY_OK',1.0);
assert(verificado,'No se confirmo VERIFY_OK. Motores apagados.');
pause(0.65); % al menos 100 muestras a 5 ms con sensores sanos
writeline(s,'P');[~,estado] = recibirHasta(s,'# AUTO state=',0.6);
fprintf('%s\n',estado);

fprintf('\nAVISO: el mapa fuerza->PWM es provisional; el control puede ser inestable.\n');
fprintf('El modo corta al superar 10 grados, 6 cm, 40%% PWM o 1.2 s.\n');
fprintf('Use topes, retencion del pendulo y corte fisico de 12 V al alcance.\n');
respuesta = input('Si el carro esta centrado y la vertical confirmada, escriba INICIAR: ','s');
if ~strcmpi(strtrim(respuesta),'INICIAR')
    fprintf('Cancelado: no se habilitaron los motores.\n');
    archivo="";return;
end

archivo = string(fullfile(pwd,['lqr_intento_' datestr(now,'yyyymmdd_HHMMSS') '.csv']));
fid = fopen(char(archivo),'w');
assert(fid>0,'No fue posible crear el registro CSV');
limpiaArchivo=onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'# Prueba LQR experimental, no validada como equilibrio estable\n');
fprintf(fid,'# Puerto=%s D=%d %d %d\n',puerto,signos(1),signos(2),signos(3));

% A y T se envian juntos para no caducar el latido antes de iniciar.
writeline(s,'A AUTO');writeline(s,'K');writeline(s,'T');writeline(s,'G');
lastKeepalive=tic;
startTrial=tic;
runOK=false;
armOK=false;
while toc(startTrial)<1.7  % incluye 0.5s para diagnostico posterior al corte
    if toc(lastKeepalive)>=0.05
        writeline(s,'K');lastKeepalive=tic;
    end
    for j=1:7
        if s.NumBytesAvailable==0, break; end
        try
            line = char(readline(s));
        catch
            break;
        end
        if contains(line,'AUTO_ARM_OK'), armOK=true; end
        if contains(line,'AUTO_RUN:') , runOK=true; end
        if contains(line,'# AUTO state=FAULT') || contains(line,'AUTO_RUN_REJECT')
            fprintf('AVISO firmware: %s\n',strtrim(line));
        end
        fprintf(fid,'%s\n',strtrim(line));
    end
    pause(0.005);
end
writeline(s,'H');
pause(0.1);
while s.NumBytesAvailable>0
    try
        fprintf(fid,'%s\n',strtrim(char(readline(s))));
    catch
        break;
    end
end
if ~armOK || ~runOK
    warning('No se recibieron ambas confirmaciones AUTO_ARM_OK y AUTO_RUN. Revisar log y P.');
end
fprintf('Terminada y desarmada. Registro guardado: %s\n',char(archivo));
end

function [encontrado,registro] = recibirHasta(s,patron,duracion)
encontrado = false;registro='';
t=tic;
while toc(t)<duracion
    if s.NumBytesAvailable>0
        try
            mensaje=char(readline(s));
        catch
            continue;
        end
        registro=[registro, sprintf('%s\n',strtrim(mensaje))]; %#ok<AGROW>
        if contains(mensaje,patron), encontrado=true;return;end
    else
        pause(0.01);
    end
end
end
function detenerSeguro(s)
try
    writeline(s,'H');
catch
end
end
