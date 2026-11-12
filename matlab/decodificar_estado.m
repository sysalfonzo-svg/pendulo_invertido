function [fila, tipo] = decodificar_estado(linea)
% Parser estricto: no confunde avisos, cabeceras o mensajes con una muestra.
% tipo: "muestra", "comentario", "vacia" o "invalida".
fila = [];
texto = strtrim(char(linea));
if isempty(texto)
    tipo = "vacia"; return
end
if texto(1) == '#'
    tipo = "comentario"; return
end
tipo = "invalida";
partes = strsplit(texto, ',', 'CollapseDelimiters', false);
if numel(partes) ~= 27
    return
end
% str2double acepta NaN/Inf; solo NaN en estados derivados es admisible.
patron = '^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$|^[+-]?[Nn][Aa][Nn]$';
if any(cellfun(@(x) isempty(regexp(x, patron, 'once')), partes))
    return
end
fila = str2double(partes);
enteros = [1:5, 10:25];
if any(~isfinite(fila(enteros))) || any(fix(fila(enteros)) ~= fila(enteros))
    fila = []; return
end
sinSigno = [1:3,12,15:21];
if any(fila(sinSigno) < 0) || any(fila(sinSigno) > 2^32-1) || ...
        any(fila(4:5) < -2^31 | fila(4:5) > 2^31-1) || ...
        fila(10) < -1 || fila(10) > 4095 || fila(11) < -1 || fila(11) > 255 || ...
        fila(22) < -1 || fila(22) > 4095 || any(~ismember(fila(23:25), [-1,1]))
    fila = []; return
end
if (any(fila(10:11) == -1) && bitand(uint32(fila(12)),uint32(1)) == 0) || ...
        (fila(22) == -1 && bitand(uint32(fila(12)),uint32(2)) == 0)
    fila = []; return
end
if any(isinf(fila))
    fila = []; return
end
tipo = "muestra";
end
