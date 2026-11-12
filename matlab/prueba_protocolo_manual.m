function prueba_protocolo_manual()
% Pruebas sin puertos/hardware: comandos permitidos y parser de estado.
assert(strcmp(comando_manual("A"),'A MANUAL'));
assert(strcmp(comando_manual("E"),'E CLEAR'));
assert(strcmp(comando_manual("V",1),'V 1'));
assert(strcmp(comando_manual("V",0),'V 0'));
assert(strcmp(comando_manual("D",[1,-1,-1]),'D 1 -1 -1'));
assert(strcmp(comando_manual("C",[-40,-40,100]),'C -40 -40 100'));
assert(strcmp(comando_manual("C",[50,50,150]),'C 50 50 150'));
invalidos={@() comando_manual("C",[51,0,100]), ...
    @() comando_manual("C",[-51,0,100]),@() comando_manual("C",[40,40,151]), ...
    @() comando_manual("C",[40,40,49]),@() comando_manual("C",[NaN,0,100]), ...
    @() comando_manual("C",[1.5,0,100]),@() comando_manual("D",[1,0,1]), ...
    @() comando_manual("V",2),@() comando_manual("BALANCE"), ...
    @() comando_manual("G",1)};
for k=1:numel(invalidos)
    fallo=false;
    try
        invalidos{k}();
    catch
        fallo=true;
    end
    assert(fallo,'Se acepto un comando invalido.');
end
[~,tipo]=decodificar_motor('# PENDULO_STATES_V1; MOTORS_LOCKED');
assert(tipo=="otro",'No aceptar el firmware anterior como manual.');
[~,tipo]=decodificar_motor('# MOTOR_MANUAL_V1');
assert(tipo=="otro",'Las dos identidades deben compartir linea.');
[~,tipo]=decodificar_motor('# PENDULO_STATES_V1; MOTOR_MANUAL_V1; Ts_us=5000; LQR_DISABLED;');
assert(tipo=="identidad");
linea='# MOTOR state=ARMED faults=0 verify=1 good=100 pwmL=0 pwmR=0 max_loop_us=83 max_cycle_us=201 F_N=2.3049';
[estado,tipo]=decodificar_motor(linea);
assert(tipo=="motor" && estado.state=="ARMED" && estado.verify==1 && estado.F_N==2.3049);
assert(estado.manual_build==0,'Campo de perfil ausente debe bloquear armado en GUI.');
[estado,tipo]=decodificar_motor([linea,' manual_build=1']);
assert(tipo=="motor" && estado.manual_build==1);
[estado,tipo]=decodificar_motor([linea,' manual_build=0']);
assert(tipo=="motor" && estado.manual_build==0);
[~,tipo]=decodificar_motor([linea,' manual_build=2']);
assert(tipo=="invalido");
[~,tipo]=decodificar_motor([linea,' manual_build=1 manual_build=0']);
assert(tipo=="invalido");
[estado,tipo]=decodificar_motor(strrep(linea,'F_N=2.3049','F_N=nan'));
assert(tipo=="motor" && isnan(estado.F_N));
invalidosEstado={strrep(linea,'state=ARMED','state=RUNNING'), ...
    strrep(linea,'verify=1','verify=2'),strrep(linea,'faults=0','faults=-1'), ...
    strrep(linea,'F_N=2.3049',''),[linea,' state=FAULT']};
for k=1:numel(invalidosEstado)
    [~,tipo]=decodificar_motor(invalidosEstado{k}); assert(tipo=="invalido");
end
fprintf('prueba_protocolo_manual: comandos y parser aprobados, sin abrir puertos.\n');
end
