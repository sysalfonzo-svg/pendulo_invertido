function [dato,tipo] = decodificar_motor(linea)
% Identidad manual y estado de motores en comentarios; sin operaciones serie.
dato = struct(); tipo = "otro";
texto = strtrim(char(linea));
if ~isempty(regexp(texto,'^#\s+PENDULO_STATES_V1(\s|;|$)','once')) && ...
        ~isempty(regexp(texto,'(^|[ ;])MOTOR_MANUAL_V1([ ;]|$)','once'))
    tipo = "identidad"; return
end
if ~startsWith(texto,'# MOTOR ')
    return
end
tipo = "invalido";
dato.manual_build=0; % Missing capability field never enables the GUI.
perfil=regexp(texto,'(?:^|\s)manual_build=([^\s]+)','tokens');
if numel(perfil)>1, return; end
if isscalar(perfil)
    dato.manual_build=str2double(perfil{1}{1});
    if ~ismember(dato.manual_build,[0 1]), return; end
end
nombres = {'state','faults','verify','good','pwmL','pwmR','max_loop_us','max_cycle_us','F_N'};
for k=1:numel(nombres)
    token = regexp(texto,['(?:^|\s)',nombres{k},'=([^\s]+)'],'tokens');
    if numel(token)~=1
        return
    end
    if k==1
        dato.state = string(token{1}{1});
    else
        dato.(nombres{k}) = str2double(token{1}{1});
    end
end
if ~ismember(dato.state,["DISARMED","ARMED","PULSE","FAULT"])
    return
end
numeros = [dato.faults,dato.verify,dato.good,dato.pwmL,dato.pwmR,dato.max_loop_us,dato.max_cycle_us];
if any(~isfinite(numeros)) || any(numeros~=fix(numeros)) || ...
        dato.faults<0 || dato.faults>2^32-1 || ~ismember(dato.verify,[0 1]) || ...
        dato.good<0 || dato.max_loop_us<0 || dato.max_cycle_us<0 || ...
        abs(dato.pwmL)>100 || abs(dato.pwmR)>100 || isinf(dato.F_N)
    return
end
tipo = "motor";
end
