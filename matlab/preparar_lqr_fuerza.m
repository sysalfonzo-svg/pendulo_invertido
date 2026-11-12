function modelo = preparar_lqr_fuerza(fuente)
% PREPARAR_LQR_FUERZA Exporta el modelo original de fuerza, sin usar hardware.
%   modelo = preparar_lqr_fuerza()
%   modelo = preparar_lqr_fuerza('ruta/modelo_inicial_pendulo.m')
% Ejecuta el archivo MATLAB de confianza en el workspace de esta funcion.
% Conserva el orden [x; vx; theta; omega], theta=0 arriba, entrada F en N.
% Escribe modelos/modelo_fuerza.mat e include/lqr_force_model.h.
% El header sirve SOLO para calcular F=-K_F*X como diagnostico; NO produce PWM.
arguments
    fuente (1,1) string = ""
end
prepRoot = fileparts(fileparts(mfilename('fullpath')));
if strlength(fuente) == 0
    fuente = string(fullfile(prepRoot,'matlab','modelo_inicial_pendulo.m'));
end
assert(isfile(fuente),'No existe el archivo del modelo: %s',fuente);
[prepOk,prepAttrs] = fileattrib(fuente);
assert(prepOk,'No se puede resolver la ruta de la fuente.');
prepFuente = string(prepAttrs.Name);
prepHash = sha256Archivo(prepFuente);
prepPrevVisible = get(groot,'DefaultFigureVisible');
prepPrevFigures = findall(groot,'Type','figure');
prepFigCleanup = onCleanup(@() restaurarFiguras(prepPrevVisible,prepPrevFigures));
set(groot,'DefaultFigureVisible','off');

% Se ejecuta la fuente intacta: no se vuelven a inventar ni sintonizar ganancias.
% Prefijo prep evita colisiones con las variables del script original.
% NaN detecta salidas ausentes y hace explicitas las variables creadas por run.
M=NaN; m=NaN; L=NaN; l=NaN; g=NaN; b=NaN; c=NaN; Jp=NaN; Ts=NaN;
A=NaN(4); B=NaN(4,1); C=NaN(2,4); Ad=NaN(4); Bd=NaN(4,1);
K_F=NaN(1,4); Q=NaN(4); R=NaN; polos=NaN(4,1);
run(char(prepFuente));

assert(exist('Jp','var')==1 && isscalar(Jp) && abs(Jp-0.00253432)<1e-12, ...
    'Jp debe ser la inercia experimental 0.00253432 kg*m^2.');
assert(exist('Ts','var')==1 && isscalar(Ts) && abs(Ts-0.005)<1e-12, ...
    'El modelo debe usar Ts=0.005 s.');
assert(isequal(size(A),[4 4]) && isequal(size(B),[4 1]) && ...
    isequal(size(Ad),[4 4]) && isequal(size(Bd),[4 1]) && isequal(size(K_F),[1 4]), ...
    'Dimensiones incompatibles con cuatro estados y una fuerza.');
assert(isreal(A) && isreal(B) && isreal(Ad) && isreal(Bd) && isreal(K_F) && ...
    all(isfinite([A(:);B(:);Ad(:);Bd(:);K_F(:)])), ...
    'Matrices/ganancias no finitas o complejas.');
assert(isequal(A(1,:),[0 1 0 0]) && isequal(A(3,:),[0 0 0 1]) && ...
    B(1)==0 && B(3)==0 && isequal(C,[1 0 0 0;0 0 1 0]), ...
    'La estructura no coincide con [x; vx; theta; omega].');

% Reconciliar A/B con las convenciones mecanicas y la entrada F en newtons.
prepH = [M+m,m*l;m*l,Jp];
prepE = prepH\[0,-b,0,0;0,0,m*g*l,-c];
prepU = prepH\[1;0];
prepA = [0,1,0,0;prepE(1,:);0,0,0,1;prepE(2,:)];
prepB = [0;prepU(1);0;prepU(2)];
assert(det(prepH)>0 && Jp-m*l^2>0 && ...
    norm(A-prepA,'inf')<1e-10 && norm(B-prepB,'inf')<1e-10, ...
    'El modelo no coincide con las convenciones mecanicas documentadas.');
prepDiscrete = c2d(ss(A,B,eye(4),zeros(4,1)),Ts,'zoh');
assert(norm(Ad-prepDiscrete.A,'inf')<1e-10 && norm(Bd-prepDiscrete.B,'inf')<1e-10, ...
    'Ad/Bd no corresponden al ZOH de A/B con Ts=5 ms.');
prepK = dlqr(Ad,Bd,Q,R);
assert(norm(K_F-prepK,'inf')<1e-10,'K_F no coincide con dlqr del modelo fuente.');
assert(numel(polos)==4 && all(isfinite(polos)) && all(abs(polos)<1) && ...
    norm(poly(polos)-poly(Ad-Bd*K_F),'inf')<1e-10, ...
    'Polos incompatibles o modelo nominal inestable en lazo cerrado.');

modelo = struct('A',A,'B',B,'Ad',Ad,'Bd',Bd,'K_F',K_F,'polos',polos, ...
    'Q',Q,'R',R,'Ts',Ts,'Jp',Jp,'Jcm',Jp-m*l^2, ...
    'parametros',struct('M',M,'m',m,'L',L,'l',l,'g',g,'b',b,'c',c), ...
    'estados',{{'x_m','vx_m_s','theta_rad','omega_rad_s'}}, ...
    'entrada','F_N','ley','F_N = -K_F * [x; vx; theta; omega]', ...
    'fuente',prepFuente,'fuente_sha256',prepHash, ...
    'tipo','LQR nominal de fuerza ideal, SOLO diagnostico, sin mapa a PWM', ...
    'generado',datetime('now','TimeZone','America/Bogota'));
prepModelDir = fullfile(prepRoot,'modelos');
if ~isfolder(prepModelDir), mkdir(prepModelDir); end
prepIncludeDir = fullfile(prepRoot,'include');
if ~isfolder(prepIncludeDir), mkdir(prepIncludeDir); end
prepMat = fullfile(prepModelDir,'modelo_fuerza.mat');
save(prepMat,'modelo','A','B','Ad','Bd','K_F','polos','Q','R','Ts','Jp');
prepHeader = fullfile(prepIncludeDir,'lqr_force_model.h');
escribirHeader(prepHeader,modelo);
clear prepFigCleanup
fprintf('\nExportado modelo de FUERZA IDEAL; no es un controlador PWM.\n');
fprintf('SHA256 fuente: %s\nMAT: %s\nHeader: %s\n',prepHash,prepMat,prepHeader);
end

function sha = sha256Archivo(ruta)
fid = fopen(ruta,'rb');
assert(fid>=0,'No se puede leer la fuente.');
limpieza = onCleanup(@() fclose(fid));
bytes = fread(fid,inf,'*uint8');
digest = java.security.MessageDigest.getInstance('SHA-256');
digest.update(typecast(bytes,'int8'));
sha = lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end

function restaurarFiguras(visible,previas)
set(groot,'DefaultFigureVisible',visible);
actuales = findall(groot,'Type','figure');
for k=1:numel(actuales)
    if ~any(actuales(k)==previas)
        delete(actuales(k));
    end
end
end

function escribirHeader(ruta,modelo)
fid = fopen(ruta,'wt');
assert(fid>=0,'No se puede escribir el header.');
limpieza = onCleanup(@() fclose(fid));
fprintf(fid,'#pragma once\n\n');
fprintf(fid,'// Generado por matlab/preparar_lqr_fuerza.m. No editar ganancias a mano.\n');
fprintf(fid,'// SOLO DIAGNOSTICO: F=-K_F*X entrega newtons, nunca PWM.\n');
fprintf(fid,'// X=[x_m, vx_m_s, theta_rad, omega_rad_s]; theta=0 arriba.\n');
fprintf(fid,'// Falta identificar/validar accionamiento, saturacion, zona muerta y retardos.\n');
fprintf(fid,'namespace lqr_force_model {\n');
fprintf(fid,'constexpr char SOURCE_SHA256[] = "%s";\n',modelo.fuente_sha256);
fprintf(fid,'constexpr float TS_S = %s;\n',literalFloat(modelo.Ts));
fprintf(fid,'constexpr float JP_KG_M2 = %s;\n',literalFloat(modelo.Jp));
escribirMatriz(fid,'A',modelo.A);
escribirVector(fid,'B',modelo.B);
escribirMatriz(fid,'AD',modelo.Ad);
escribirVector(fid,'BD',modelo.Bd);
escribirVector(fid,'K_F',modelo.K_F);
escribirVector(fid,'POLES_REAL',real(modelo.polos));
escribirVector(fid,'POLES_IMAG',imag(modelo.polos));
fprintf(fid,['inline float diagnosticForceN(float x, float vx, float theta, float omega) {\n' ...
    '  return -(K_F[0]*x + K_F[1]*vx + K_F[2]*theta + K_F[3]*omega);\n' ...
    '}\n} // namespace lqr_force_model\n']);
end

function escribirMatriz(fid,nombre,matriz)
fprintf(fid,'constexpr float %s[%d][%d] = {\n',nombre,size(matriz,1),size(matriz,2));
for fila=1:size(matriz,1)
    valores = arrayfun(@literalFloat,matriz(fila,:),'UniformOutput',false);
    fprintf(fid,'  {%s}',strjoin(valores,', '));
    if fila<size(matriz,1), fprintf(fid,','); end
    fprintf(fid,'\n');
end
fprintf(fid,'};\n');
end

function escribirVector(fid,nombre,vector)
valores = arrayfun(@literalFloat,vector(:).','UniformOutput',false);
fprintf(fid,'constexpr float %s[%d] = {%s};\n',nombre,numel(vector),strjoin(valores,', '));
end

function valor = literalFloat(x)
valor = sprintf('%.9g',x);
if ~contains(valor,'.') && ~contains(lower(valor),'e')
    valor = [valor,'.0'];
end
valor = [valor,'f'];
end
