% Modelo mecanico preliminar con las medidas del carro.
% Ejecutar en MATLAB con Control System Toolbox. NO cargar en la STM32.
% Entrada F en newtons. K_F NO produce PWM.
% Aproximaciones: varilla uniforme; acople/iman de inercia despreciable;
% sin friccion, deslizamiento, cabeceo ni inercia reflejada de transmision.
% Esta simulacion ideal NO valida el hardware ni reemplaza pendulo_lqr.m.

M = 0.8466;                 % kg, carro sin varilla
m = 0.0273;                 % kg, varilla
L = 0.516;                  % m, longitud de la varilla
l = 0.265;                  % m, pivote a centro de masa medido
g = 9.81;
% Momento de inercia experimental
Jp = 0.00253432;       % kg*m^2 respecto al pivote

% Inercia respecto al centro de masa
Jcm = Jp - m*l^2;
b = 0; c = 0;              % fricciones no identificadas: supuesto nominal
Ts = 0.005;                % s, periodo propuesto, pendiente verificar en STM32

% X=[x; velocidad; theta; velocidad_angular]. Theta=0 vertical arriba.
% x positivo hacia delante; theta positivo inclinado hacia delante.
H0 = [M+m, m*l; m*l, Jp];
E = H0\[0,-b,0,0;0,0,m*g*l,-c];
U = H0\[1;0];
A = [0,1,0,0; E(1,:); 0,0,0,1; E(2,:)];
B = [0;U(1);0;U(2)];
C = [1,0,0,0;0,0,1,0];
assert(det(H0)>0 && rank(ctrb(A,B))==4 && rank(obsv(A,C))==4);
planta_F = ss(A,B,C,zeros(2,1));
planta_Fd = c2d(planta_F,Ts,'zoh');
Ad = planta_Fd.A; Bd = planta_Fd.B;

escalaEstados = [0.10;0.40;deg2rad(5);1.5];
escalaFuerza = 1;           % N, peso de coste; NO es limite del actuador
Q = diag(1./escalaEstados.^2);
R = 1/escalaFuerza^2;
[K_F,~,polos] = dlqr(Ad,Bd,Q,R);
assert(all(abs(polos)<1));

% Realimentacion ideal: todos los estados conocidos; F sin saturacion.
% Comparacion lineal/no lineal con mando constante entre muestras.
t = 0:Ts:8; N = numel(t);
Xlin = zeros(4,N); Xnl = zeros(4,N);
Xlin(:,1) = [0;0;deg2rad(3);0]; Xnl(:,1) = Xlin(:,1);
F = zeros(1,N);
p = struct('M',M,'m',m,'l',l,'Jp',Jp,'b',b,'c',c,'g',g);
for k=1:N-1
    Xlin(:,k+1) = (Ad-Bd*K_F)*Xlin(:,k);
    F(k) = -K_F*Xnl(:,k);
    z = Xnl(:,k); h = Ts/4;
    for j=1:4
        k1 = dinamica(z,F(k),p);
        k2 = dinamica(z+h*k1/2,F(k),p);
        k3 = dinamica(z+h*k2/2,F(k),p);
        k4 = dinamica(z+h*k3,F(k),p);
        z = z+h*(k1+2*k2+2*k3+k4)/6;
    end
    Xnl(:,k+1) = z;
end
F(N) = -K_F*Xnl(:,N);

fprintf('\nMODELO PRELIMINAR: FUERZA IDEAL, NO PWM\n');
fprintf('Jp estimada = %.8f kg*m^2\n',Jp);
disp('A ='); disp(A); disp('B ='); disp(B);
disp('K_F = [N/m, N/(m/s), N/rad, N/(rad/s)]'); disp(K_F);
disp('Polos discretos del modelo nominal ='); disp(polos);
fprintf('Fuerza maxima solicitada en este ejemplo: %.4f N\n',max(abs(F)));
fprintf('Faltan accionamiento real, saturacion, retardos y estimador.\n');

figure('Name','Modelo preliminar: fuerza ideal, no PWM','Color','w');
subplot(3,1,1);
plot(t,rad2deg(Xnl(3,:)),t,rad2deg(Xlin(3,:)),'--');
ylabel('Angulo (grados)'); legend('No lineal','Lineal'); grid on;
title('LQR nominal: estados conocidos y actuador ideal');
subplot(3,1,2);
plot(t,Xnl(1,:),t,Xlin(1,:),'--'); ylabel('Posicion (m)'); grid on;
subplot(3,1,3);
stairs(t,F); ylabel('Fuerza (N)'); xlabel('Tiempo (s)'); grid on;

function dz = dinamica(z,F,p)
th=z(3); w=z(4);
H=[p.M+p.m,p.m*p.l*cos(th);p.m*p.l*cos(th),p.Jp];
a=H\[F-p.b*z(2)+p.m*p.l*w*w*sin(th); ...
      p.m*p.g*p.l*sin(th)-p.c*w];
dz=[z(2);a(1);w;a(2)];
end
