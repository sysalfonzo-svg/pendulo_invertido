function comando = comando_manual(tipo,valores)
% Construye solo comandos permitidos. No abre puertos ni envia nada.
arguments
    tipo (1,1) string
    valores double = []
end
switch tipo
    case {"?","G","H","S","!","K","P","N","Z"}
        assert(isempty(valores),'Este comando no admite argumentos.');
        comando = char(tipo);
    case "E"
        assert(isempty(valores),'E CLEAR no admite otros argumentos.');
        comando = 'E CLEAR';
    case "A"
        assert(isempty(valores),'A MANUAL no admite otros argumentos.');
        comando = 'A MANUAL';
    case "V"
        assert(isscalar(valores) && ismember(valores,[0 1]),'V requiere 0 o 1.');
        comando = sprintf('V %d',valores);
    case "D"
        assert(numel(valores)==3 && all(ismember(valores(:),[-1 1])), ...
            'D requiere tres signos +1/-1.');
        comando = sprintf('D %d %d %d',valores(:));
    case "C"
        assert(numel(valores)==3 && all(isfinite(valores(:))) && ...
            all(valores(:)==fix(valores(:))) && all(abs(valores(1:2))<=50) && ...
            valores(3)>=50 && valores(3)<=150, ...
            'C requiere PWM enteros -50..50 y duracion 50..150 ms.');
        comando = sprintf('C %d %d %d',valores(:));
    otherwise
        error('Pendulo:Comando','Comando manual no permitido: %s',tipo);
end
end
