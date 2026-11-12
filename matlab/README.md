# MATLAB: medicion y pruebas manuales

El proyecto incorpora ahora `prueba_manual.m`, con armado y pulsos por acciones
explicitas, y `analizar_pulsos.m` para revisar los registros sin conectar la placa.
Consultar primero [las instrucciones del proyecto](../README.md) y
[el procedimiento fisico](../docs/pruebas_fisicas.md). El equilibrio automatico
todavia no esta implementado: `preparar_lqr_fuerza.m` exporta exclusivamente el
modelo de fuerza ideal y requiere Control System Toolbox.

```matlab
addpath('matlab')
serialportlist("available")
R = prueba_manual("COM15",300); % Cambiar COM; no mueve motores al abrirse
informe = analizar_pulsos(R.archivos.csv);
```

Las instrucciones restantes describen el cliente de solo medicion
`medir_estados.m`, que se conserva para repetir la validacion de los estados.

## Medicion de cuatro estados

Guardar todos los `.m` de esta carpeta juntos. No requiere Control System Toolbox.
El muestreo de 5 ms lo realiza la STM32; MATLAB recibe por USB, registra todas las
filas recibidas y actualiza las graficas a unos 10 Hz. Una demora de la interfaz
del PC no cambia el reloj del MCU. Las perdidas de transporte quedan visibles
en `seq`, `tx_drop` y el resumen; la ausencia de perdidas debe verificarse.

Cerrar el monitor serie de PlatformIO. Identificar el COM actual; no asumir que
el COM17 con advertencia pertenece a la placa. En esta sesion se detecto COM15.

```matlab
addpath('matlab')                   % Desde la carpeta del proyecto
serialportlist("available")
R = medir_estados("COM15",60);      % Cambiar COM si Windows lo reasigna
```

Sin ventana, para registrar una placa ya calibrada durante la misma alimentacion:

```matlab
R = medir_estados("COM15",30,"registros",false);
```

El programa exige la identidad `PENDULO_STATES_V1` antes de enviar `G`. Si la
STM32 todavia ejecuta identificacion, termina sin iniciar la captura. Nunca
envia `C` ni otro comando de motor. No abre automaticamente varios COM para
buscar una placa. `medir_estados()` solo elige por si mismo cuando hay un unico
puerto disponible.

## Calibracion manual con motores sin alimentacion

1. **N: origen del carro** fija el origen al situar el carro en el punto de
   referencia. No es una medicion de la posicion absoluta sobre el suelo.
2. Empujar el carro a mano hacia delante. Observar los contadores de la ventana.
   El registrado **L** corresponde a la **rueda derecha** (1969,6 cuentas/vuelta);
   el registrado **R** a la **izquierda** (1974 cuentas/vuelta). Seleccionar para
   cada uno el signo que haga positiva su contribucion al avance. Verificar por
   separado cada rueda si sus cuentas se cancelan en el promedio.
3. Sostener el pendulo quieto, **vertical hacia arriba**, y pulsar **Z**. El
   programa no identifica por si mismo cual vertical es la de arriba.
4. Inclinar ligeramente el pendulo hacia delante y observar el angulo. Debe
   ser positivo. Elegir el signo de theta correspondiente. Aplicar los tres
   signos con **D: confirmar sentidos** solo despues de la comprobacion fisica;
   los selectores se inicializan con los signos recibidos al conectar. Cambiar
   una seleccion solo tiene efecto al pulsar D; el pie muestra los signos
   realmente activos. Repetir Z si cambia la referencia fisica del sensor.
5. Registrar una segunda captura quieta y otra moviendo el carro y pendulo a
   mano. Mantener el control fisico del conjunto; no intentar equilibrio.

Los botones Z, N y D son acciones explicitas: no se ejecutan al conectar. Los
reinicios del MCU pierden la calibracion guardada en RAM. Un cambio de origen
durante una captura produce una discontinuidad esperada; queda registrado en
el archivo de eventos y conviene hacer una nueva captura para validacion.
El boton **S** enclava emergencia. Al terminar o cerrar la ventana se envian
**H y S**, se libera el puerto y se guardan los datos. La emergencia enclavada
queda visible en flags, pero no impide medir ni requiere reinicio para iniciar
otro registro en este firmware de medicion con motores bloqueados.

## Archivos y criterios

Cada ejecucion crea en `registros/` un CSV con 27 columnas, un MAT con `T`,
`datos`, `tiempo_s`, `meta`, `resumen`, y un TXT de comentarios y comandos.
La hora de archivo usa `America/Bogota`. El tiempo de adquisicion proviene del
MCU, no del instante de recepcion de Windows. `t_us` y `seq` se desenvuelven
modulo 2^32; un reinicio u orden invalido se marca, no se oculta como rollover.
Los errores conservan el registro parcial disponible. Si se interrumpe con
Ctrl+C, el cleanup cierra el puerto y los archivos; el CSV parcial permanece,
pero el MAT de resumen puede no haberse generado. Cerrar la ventana o usar
"Terminar registro" permite generar tambien el resumen MAT.

```matlab
load(R.archivos.mat)
[resumen, tiempo_s] = validar_estados(T);
disp(resumen)
prueba_estados                    % Solo datos sinteticos, sin abrir puertos
```

Flags: bit 0 = sensor invalido; bit 1 = cero angular pendiente; bit 2 = sentidos
pendientes; bit 3 = emergencia. El angulo se almacena en radianes y la velocidad
angular en rad/s; la grafica convierte ambos a grados para inspeccion. Los
angulos pendientes de cero o con sensor invalido no se dibujan como validos.
`v_raw_m_s` y `omega_raw_rad_s` permiten comparar filtrado y derivacion.

La aceptacion temporal exige al menos 1000 muestras, periodos dentro de
5000 +/- 100 us, latencia de inicio <=100 us, adquisicion <1000 us, coherencia
entre timestamp y dt, y cero perdidas durante el registro. Para marcar
`cuatro_estados_listos`, exige ademas sensor/cero/sentidos validos, estados
finitos, cero PWM y sin incrementos de errores I2C o encoder. La emergencia
enclavada se informa por separado: no invalida las mediciones de esta etapa.
Estos son criterios de esta etapa, no una autorizacion para habilitar LQR.
La exactitud mecanica requiere comparar desplazamiento medido, inclinacion y
sentidos con referencias fisicas; el software no puede certificarla solo.

API oficial: [serialport](https://www.mathworks.com/help/matlab/ref/serialport.html)
y [serialportlist](https://www.mathworks.com/help/matlab/ref/serialportlist.html).
