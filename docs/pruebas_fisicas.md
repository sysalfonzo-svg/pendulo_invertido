# Pruebas progresivas del carro-pendulo fisico

Este procedimiento usa la adquisicion de cuatro estados a 5 ms y el modo
manual de pulsos. No habilita equilibrio automatico. La referencia de motor
se conserva: **C -40 -40 corresponde al avance observado con el firmware
anterior**. Esa observacion debe contrastarse de nuevo con las ruedas
levantadas; no decide por si sola el signo de la odometria.

**Estado de esta entrega:** el modo manual esta implementado en el codigo,
pero esta nueva version no se ha recargado ni ensayado con motores en hardware
durante esta etapa. La compilacion y las pruebas de software se informan
aparte; no sustituyen los ensayos fisicos pendientes de esta guia. La
adquisicion anterior validada se conserva en
`backups/adquisicion_validada_2026-10-31/`.

Los resultados historicos y su ambiguedad de avance/retroceso estan en
`validacion_etapa1.md`. No copiar la ultima combinacion de signos como si
fuera una comprobacion nueva. Antes de alimentar motores se debe completar
la prueba de frente marcado descrita abajo.

## Hardware y convenciones que se conservan

| Elemento | Conexion o significado |
|---|---|
| Motor izquierdo | PWM PA9; direccion PB14/PB15 |
| Motor derecho | PWM PA8; direccion PB12/PB13 |
| Encoder registrado L | PA6/PA7; rueda derecha fisica; 1969,6 cuentas/vuelta |
| Encoder registrado R | PA0/PA1; rueda izquierda fisica; 1974 cuentas/vuelta |
| AS5600 | SCL PB6; SDA PB7; alimentacion y pullups a 3,3 V |
| Estado | `[x; vx; theta; omega]`, unidades m, m/s, rad, rad/s |
| Referencia angular | theta=0 con el pendulo vertical ARRIBA |
| Sentido positivo | x y theta positivos hacia el mismo frente marcado |

Los puentes de ENA/ENB del L298N deben estar retirados para que la STM32
controle las entradas de habilitacion. Mantener GND comun y no conectar
simultaneamente dos fuentes de 5 V sin aislamiento previsto. Un paro de
software pone los motores en giro libre: el carro puede seguir avanzando
por inercia. La distancia libre necesaria incluye esa parada.

Para garantizar enables bajos durante reset o watchdog, colocar resistencias
externas de aproximadamente 10 kohm desde ENA y ENB a GND. Los pull-down
internos configurados por firmware no cubren el intervalo previo a setup.

Preparar un interruptor fisico accesible que corte los 12 V de potencia.
El USB y el programa de PC no sustituyen ese corte. No tocar ruedas,
engranajes ni conexiones de potencia durante un pulso. Al programar o
reiniciar mantener la potencia desconectada: el software no controla los
GPIO durante todo el intervalo de reset previo a setup.

## Identificacion de la version y carga

Con 12 V desconectados y USB/ST-Link conectados, desde PowerShell en la
carpeta del proyecto, empezar con el perfil que bloquea la actuacion:

```powershell
& "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe" run -e solo_medicion
& "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe" run -e solo_medicion -t upload
```

`solo_medicion` define `ENABLE_MANUAL_MOTORS=0`: conserva la adquisicion y
rechaza A y C. Verificar en P `manual_build=0`. Cuando se complete la
comparacion de adquisicion, cargar la version manual, aun con 12 V desconectados:

```powershell
& "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe" run -e genericSTM32F401RC
& "$env:USERPROFILE/.platformio/penv/Scripts/pio.exe" run -e genericSTM32F401RC -t upload
```

En esta version P informa `manual_build=1`. Esto permite solicitar armado
manual, pero el arranque sigue desarmado y nunca inicia movimiento. Ambos
perfiles tienen LQR bloqueado.

Exigir `Verified OK` y `SUCCESS`. Cerrar el monitor serie antes de abrir
MATLAB. Identificar el COM actual con `serialportlist("available")`; COM15
fue el de esta sesion, pero Windows puede reasignarlo. ST-Link no utiliza
ese COM para cargar.

Consultar `?` y `P`. La version debe anunciar `PENDULO_STATES_V1` y la
capacidad `MOTOR_MANUAL_V1`, y P debe mostrar el perfil esperado. No ejecutar
pulsos si responde solo el firmware
viejo de identificacion o si no se reconoce el protocolo. El CSV de los
estados conserva sus 27 columnas; `P` y las lineas que comienzan con `#`
aportan los diagnosticos adicionales.

## Comandos del modo manual

| Comando | Funcion |
|---|---|
| `?` | Identidad y capacidades |
| `P` | Estado de habilitacion, fallos y diagnosticos |
| `G` | Inicia la telemetria; no arma motores |
| `H` | Desarma y termina telemetria; no equivale a habilitar otra prueba |
| `S` o `!` | Parada de emergencia enclavada; `!` no necesita Enter |
| `E CLEAR` | Reconoce/libera un enclavamiento cuando la causa ya desaparecio; sigue desarmado |
| `Z` | Captura cero angular, solo con pendulo inmovil y vertical ARRIBA |
| `D signL signR signTheta` | Declara signos +1/-1 de las conversiones, no cambia cables ni motores |
| `N` | Nuevo origen del carro; no es una medicion de posicion absoluta |
| `V 1` | Confirma explicitamente que se realizo la comprobacion fisica de signos |
| `V 0` | Revoca esa confirmacion; disponible solo sin una sesion de motor activa |
| `A MANUAL` | Habilita por tiempo limitado el modo de pulsos; no inicia movimiento |
| `K` | Latido del cliente de pruebas |
| `C pwmIzq pwmDer duracion_ms` | Solicita UN pulso corto y limitado |

Usar el cliente `matlab/prueba_manual.m`, que recibe y registra datos mientras
envia `K` cada 50 ms. Desde la carpeta del proyecto:

```matlab
addpath('matlab')
prueba_manual("COM15") % Cambiar COM si Windows lo reasigno
```

Sus controles de armado y de pulso requieren acciones explicitas. K mantiene
la comunicacion supervisada, pero no arma ni extiende la ventana absoluta
de armado de 5 s. No usar el monitor serie como cliente de movimiento: no es
practico mantener a mano el latido. Al cerrar el cliente se solicita S.
No encadenar una lista de pulsos sin inspeccion entre ellos.

Los limites iniciales del modo son: magnitud PWM maxima 50%, pulsos de
50 a 150 ms, ventana de armado de 5 s y al menos 1 s entre pulsos. Cada pulso
terminado deja el modo desarmado. No hay inversion directa de un motor
durante un pulso. Los limites de angulo y velocidad angular iniciales son
20 grados y 8 rad/s. Consultar `P` y las constantes del firmware final antes
de ensayar; no aumentar limites para conseguir que una prueba pase.

La confirmacion `V 1` no verifica el movimiento por si misma: registra una
comprobacion fisica realizada por el operador. Reset, D, Z y desconexion USB
invalidan esa confirmacion. Una muestra de sensor invalida tambien la revoca.
N, H y el fin normal de un pulso no la invalidan, aunque N reinicia el contador
de muestras sanas. D, Z, N y V se rechazan durante ARMED o PULSE: desarmar con
H antes de utilizarlos.
Despues de un fallo, comprobar P y volver a declarar la verificacion cuando
lo exija el firmware. E CLEAR requiere al menos 100 muestras consecutivas
sanas, cero, signos y verificacion fisica vigentes; sigue dejando el modo
desarmado. Con salida cero se pueden completar D/Z/V antes de reconocer el
fallo. No usar E CLEAR para ignorar una causa que sigue presente.

## Limites temporales implementados, aun pendientes de comprobar en la placa

| Supervision | Umbral o mecanismo en el codigo |
|---|---|
| Captura y vigilancia independiente | ISR de TIM2 cada 5000 us |
| Duracion de una vuelta de foreground | maximo 2000 us |
| Captura, adquisicion y procesado de un ciclo | maximo 2500 us |
| Intervalo entre muestras | 4900 a 5100 us |
| Retraso de ISR respecto al tick | maximo 100 us |
| Lectura I2C del AS5600 | timeout 1500 us desde el tick |
| Antiguedad del ultimo ciclo completo | maximo 10000 us |
| Falta de progreso del foreground | maximo 10000 us |
| Latido K | caduca a los 250 ms; cliente intenta enviarlo cada 50 ms |
| IWDG | nominal 64 ms si LSI=32 kHz; depende de la tolerancia del oscilador |

La ISR puede ordenar salida cero aunque el foreground tarde demasiado.
El IWDG se refresca solo despues de procesar una adquisicion completa; cubre
la falta de progreso cuando la vigilancia de software no puede ejecutarse.
Su valor nominal de 64 ms no es una promesa de latencia exacta de parada.
Un reset por IWDG deja registrado WATCHDOG y requiere recuperar condiciones
y reconocimiento explicito; no rearma motores. La tolerancia y el efecto
real sobre ENA/ENB deben comprobarse sin potencia.

Los umbrales no significan que todas las paradas ocurran en ese mismo
microsegundo: las comprobaciones tienen su cadencia y deben medirse. P
informa `max_loop_us` y `max_cycle_us`; el CSV conserva dt, late, acq,
missed y errores. Ningun sobretiempo debe resolverse relajando el limite
sin investigar primero el registro.

Los bits de `faults` en P son: 1 ESTOP, 2 SENSOR, 4 STALE, 8 ANGLE
(tambien limite de omega), 16 TIMING, 32 LINK, 64 ENCODER, 128 WATCHDOG,
256 CALIBRATION y 512 recorrido. Pueden acumularse. Son distintos de los
bits de `flags` del CSV; su bit 3 indica que hay un fallo enclavado.

## 0. Revalidar adquisicion y signos, sin potencia de motores

1. Registrar 30 s en reposo con el programa MATLAB de estados. Comparar con
   la adquisicion anterior: dt alrededor de 5000 us, sin incremento de
   `missed`, sin errores I2C, transiciones invalidas ni perdidas de CSV durante
   una captura normal. PWM debe ser 0/0. Conservar el CSV y los diagnosticos.
2. Pegar una flecha **FRENTE** en el chasis y otra marca fija en la mesa o suelo.
   Usar siempre ese mismo lado para avance del carro e inclinacion del pendulo.
3. Calibrar Z con un soporte que mantenga el pendulo quieto vertical ARRIBA.
   No usar Z cuando cuelgue abajo ni para ocultar una inclinacion excesiva.
4. Aplicar N en una marca de origen y registrar las cuentas nL/nR iniciales.
   Empujar el carro a mano 20 cm medidos hacia la flecha, con ambas ruedas
   apoyadas y sin recolocarlo ni girar una rueda antes de empezar. Guardar
   cuentas finales, x, vx y distancia fisica.
5. Regresar a la misma marca sin pulsar N ni D entre los dos movimientos.
   Deben invertirse ambos incrementos de cuentas, vx debe cambiar de signo
   y x debe volver cerca de cero. Registrar el residual en milimetros. Una
   diferencia no se corrige inventando un CPR a partir de una unica pasada.
6. Elegir signL y signR para que cada rueda contribuya positivamente en el
   avance. Aplicar D si hace falta y repetir el recorrido completo con las
   conversiones definitivas. Comprobar cada rueda por separado si se cancelan
   en el promedio, sin confundir L registrado con izquierda fisica.
7. Inclinar el pendulo unos pocos grados hacia la misma flecha: theta debe
   aumentar. Regresarlo al cero y hacia atras: theta debe disminuir. Repetir
   tras cualquier cambio de signo angular.
8. Solo cuando ambas direcciones son coherentes, declarar V 1. Guardar foto
   o croquis de la flecha, signos, cero y registro de las dos pasadas.

El modo manual limita el desplazamiento a 0,10 m desde la posicion al armar.
Fijar N en un centro y armar solo donde quede margen libre a ambos lados,
incluyendo la distancia de giro libre. El limite por encoder es una proteccion
adicional: no detecta deslizamiento ni sustituye topes fisicos y espacio de
parada. Un nuevo armado fija una nueva referencia de este limite; no se debe
usar una sucesion de armados para salir del corredor protegido.

## 1. Comprobar protecciones con los 12 V desconectados

Mantener el pendulo sujeto arriba, lejos de los umbrales de angulo. El
diagnostico debe distinguir DESARMADO, ARMADO, PULSO y FALLO/ESTOP.
Las pruebas de corte se realizan con perfil manual y armado explicito,
siempre sin 12 V. Si una condicion incorrecta ya existe estando desarmado,
puede impedir A sin generar un nuevo enclavamiento; verificar el rechazo y
la salida cero. Durante ARMED/PULSE debe provocar el corte y fallo indicado.

| Prueba | Resultado que debe observarse |
|---|---|
| Arranque/reset normal | PWM 0/0 y desarmado |
| Reset por IWDG | PWM 0/0 y fallo WATCHDOG enclavado |
| Perfil solo_medicion | P informa manual_build=0; A y C se rechazan |
| Solicitar C sin armado | Rechazo y PWM 0/0 |
| S/! y luego intentar A/C | Rechazo hasta reconocer la emergencia y recuperar condiciones |
| E CLEAR | Desarmado; nunca un pulso por si solo |
| Armar y no solicitar C | Desarmado al vencer los 5 s |
| Interrumpir el latido del cliente | Fallo seguro y salida cero dentro del plazo configurado |
| Desconectar USB | Desarmado/fallo, sin rearme al reconectar |
| Sensor desconectado o magneto fuera de rango | Datos invalidos y bloqueo del accionamiento |
| Inclinar mas de 20 grados | Bloqueo/parada por limite angular |
| Parametros C fuera de rango | Rechazo, sin truncar silenciosamente a otro pulso |
| Linea de comando demasiado larga | ESTOP enclavado y salida cero |
| D/Z/N/V durante ARMED o PULSE | Rechazo hasta desarmar con H |
| Exceso de tiempo o muestra vencida | Fallo TIMING/STALE y salida cero |
| Incremento de transicion imposible de encoder | Fallo ENCODER durante una sesion activa |
| Desplazamiento mayor de 0,10 m desde el armado | Fallo por recorrido y salida cero |

Hacer la prueba de sensor inicialmente sin potencia. Restablecer su conexion,
esperar lecturas validas y comprobar la causa registrada antes de E CLEAR.
Un dato de AS5600 solamente es aceptable si hay iman detectado, sin indicadores
de iman demasiado debil/fuerte, y la muestra sigue dentro del plazo vigente.
Un timeout, una muestra vencida o un exceso del tiempo de ejecucion no deben
mantener el ultimo PWM. El watchdog debe reiniciar a estado desarmado si
el software deja de progresar; la verificacion de su reset se hace sin potencia.

No pasar a la prueba siguiente solo porque aparecen cuatro numeros en MATLAB.
Guardar una tabla de resultado observado para cada proteccion. Cualquier
fallo que conserve salida no nula impide energizar el L298N.

Antes del primer armado **con potencia**, completar esta lista con evidencia
fisica de la version cargada. Ninguna casilla esta aprobada por este documento:

- [ ] Identidad y `manual_build` corresponden al perfil cargado; reset no activa ENA/ENB.
- [ ] C sin A no produce salida; A por si solo tampoco la produce.
- [ ] S y ! cortan y enclavan; E CLEAR deja DISARMED y requiere salud recuperada.
- [ ] Fin de pulso y vencimiento de armado desarman; no hay repeticion automatica.
- [ ] Perdida de K y desconexion USB cortan; reconexion no rearma y revoca V.
- [ ] Sensor invalido corta y revoca V; muestra vencida corta; ninguno mantiene el ultimo PWM.
- [ ] Angulo y omega fuera de limite impiden armar o detienen la sesion activa.
- [ ] Fallos de encoder, recorrido y temporizacion detienen la sesion activa.
- [ ] El watchdog reinicia con salidas apagadas y el fallo queda enclavado.
- [ ] El corte fisico de 12 V funciona; las ruedas y el pendulo estan retenidos.

Las pruebas de bloqueo o watchdog que requieran una version instrumentada
deben hacerse con 12 V desconectados y volver despues a la version final,
repitiendo identidad y adquisicion. No improvisar un bloqueo deliberado
durante una prueba energizada.

## 2. Ruedas levantadas: comprobar direccion y parada

Fijar el chasis a un soporte estable con las dos ruedas libres. Sujetar la
varilla vertical ARRIBA mediante soporte; no sostenerla cerca de las ruedas.
Conectar los 12 V solo cuando la prueba sin potencia haya terminado y el
interruptor fisico este al alcance. Empezar desarmado y con telemetria activa.

Probar un unico motor cada vez con 50 ms y el menor PWM que pueda observarse
de forma util. Si no vence la zona muerta, aumentar entre pruebas, manteniendo
pulso corto y sin superar 50%. Ausencia de giro a bajo PWM no demuestra fallo.
Desarmar/esperar parada y cooldown antes de cambiar el sentido.

Despues de los motores individuales, comprobar la referencia historica con
un unico **C -40 -40 100**, armado explicitamente. Ambas ruedas deben girar en
el sentido que desplaza el carro hacia FRENTE al apoyarlas. Los comandos
positivos correspondientes deben producir el sentido opuesto. No cambiar
INVERTIR_L/INVERTIR_R ni pines para corregir un signo de conversion de encoder.

Para cada pulso guardar: mando y tiempo solicitados, PWM observado en CSV,
cuentas de ambas ruedas, tiempo de comienzo y fin, estado de armado/fallo,
sensor, maximos de dt/adquisicion/ejecucion y motivo de parada. Tras vencer
el pulso el PWM debe ser cero sin otra orden del PC. Las ruedas pueden
seguir girando por inercia porque la parada es por giro libre.

Comprobar la parada S durante un pulso corto y la perdida del cliente con
las ruedas levantadas, en pruebas separadas. Una persona permanece junto
al corte de 12 V. Si el pulso no termina o aparece giro inesperado, cortar
potencia, conservar el registro y no repetir aumentando limites.

## 3. Suelo: capacidad de actuacion e identificacion

Delimitar un corredor recto, nivelado y despejado. Usar retencion mecanica
que impida escapar al carro sin introducir una fuerza apreciable durante
el tramo medido; los topes deben quedar fuera de ese tramo. Mantener el
pendulo sujeto cerca de la vertical durante la identificacion del actuador.
Documentar si el soporte altera la masa, rozamiento o movimiento del conjunto.

Fijar el origen en el centro del corredor y configurar/comprobar el limite de
recorrido antes de armar. Ese limite debe dejar margen para el giro libre.
Empezar con un pulso de 50 ms, examinar la distancia real, y aumentar de forma
gradual hasta 100 o 150 ms solo si existe margen. Cada sentido es una prueba
separada, con parada completa y nuevo armado manual.

Repetir pulsos en ambos sentidos alrededor del arranque, por ejemplo 20%,
25%, 30%, 35%, 40%, 45% y 50%, comenzando por los niveles bajos. No automatizar
esta lista como una secuencia continua. Repetir cada punto que sea seguro
para estimar dispersion y diferenciar arranque desde reposo de movimiento
ya iniciado. Registrar tension de bateria antes/despues y condiciones del suelo.

La informacion minima por ensayo es:

- Instante de mando, comienzo de movimiento y retraso hasta producir respuesta.
- Tiempo real con PWM aplicado y saturacion de cada motor.
- Recorrido de cada rueda, velocidad, aceleracion estimada y asimetria.
- PWM minimo que inicia movimiento en cada sentido, y PWM que sostiene giro.
- Desaceleracion tras retirar PWM en giro libre y distancia hasta detenerse.
- Estado de los sensores, temporizacion, caidas de tension y fallos.

P muestra `F_N` como diagnostico de la fuerza ideal que solicita el modelo
MATLAB. `include/lqr_force_model.h`, generado por `matlab/preparar_lqr_fuerza.m`,
contiene A/B, Ad/Bd, K_F y la huella SHA256 de la fuente usada. Calcula
`F_N=-K_F*[x;vx;theta;omega]`, en newtons. Es una magnitud calculada, no un
mando PWM aplicado ni una medicion de fuerza de traccion. El modo manual
no conecta ese diagnostico a los motores y no permite habilitar un LQR.

Las plantas experimentales de ruedas levantadas,
`G_L=1.3355/(0.1688*s+1)` y `G_R=1.1062/(0.1573*s+1)`, tienen unidades
RPM/%PWM. Sirven como punto de partida para velocidad, no como una conversion
directa a fuerza. Bajo carga pueden cambiar ganancia, retardo y dinamica.

La aceleracion en el suelo da **fuerza neta**, no fuerza de traccion aislada.
El valor de desaceleracion en giro libre ayuda a modelar resistencias, pero
no autoriza sumar una constante sin comprobar velocidad, sentido, suelo y
tension. Si se quiere estimar traccion, usar un modelo mecanico que explicite
esas resistencias y las fuerzas del pendulo, con los datos medidos.

No aumentar el PWM mas alla del limite manual para compensar un carro que
no consigue la respuesta necesaria. Ese resultado puede indicar que el
L298N, los motores, la alimentacion o la transmision no tienen margen suficiente.

## 4. LQR real: condiciones para preparar el primer intento

Esta fase queda condicionada a las pruebas anteriores. No existe autorizacion
operativa para activar equilibrio por el simple hecho de haber cargado este
firmware de pulsos. Primero debe existir una version especifica de control
revisada y probada en modo desarmado.

Usar `modelo_inicial_pendulo.m` como referencia de A/B y ganancias, con
`Jp=0.00253432 kg*m^2` respecto al pivote y orden de estados
`[x; vx; theta; omega]`. Confirmar unidades y si cada ganancia corresponde
a modelo continuo o discreto. No copiar K a un multiplicador de PWM:
si la entrada del modelo es F en newtons, `F=-K*X` sigue expresada en newtons.

Antes de generar la ganancia que ejecutara el MCU:

1. Definir una entrada fisica identificada: fuerza estimada o un modelo que
   incluya directamente las dinamicas PWM-motor-transmision-carro. Documentar
   la conversion y sus limites de validez en ambos sentidos.
2. Incorporar la dinamica de actuacion y retardos medidos; aumentar el estado
   si hay estados de motor o retardos discretos. Considerar tambien el filtro
   de velocidad/omega y el AS5600, no solo el periodo nominal de 5 ms.
3. Discretizar con Ts=0,005 s y retencion adecuada al PWM aplicado. Volver a
   verificar controlabilidad, polos del lazo aumentado y margen de actuacion.
4. Incluir saturacion, zona muerta, asimetria y giro libre en las pruebas del
   controlador. Comprobar recuperacion tras saturar; no ocultar el esfuerzo
   requerido limitando solo las graficas.
5. Validar predicciones frente a registros fisicos independientes de los que
   se usaron para identificar. Ejecutar primero el calculo del control con
   salida bloqueada y comprobar signos, tiempos, unidades y limites.
6. Demostrar que el mando disponible puede corregir las perturbaciones
   iniciales previstas sin agotar inmediatamente PWM o recorrido. Si no,
   revisar actuador/modelo antes de energizar un lazo de equilibrio.

## 5. Primer intento fisico de equilibrio, cuando exista esa version validada

Usar carril o guia que mantenga el movimiento en una dimension, topes con
absorcion de energia fuera del intervalo de trabajo y proteccion que limite
la caida de la varilla sin engancharla durante el equilibrio. Mantener a una
persona junto al corte fisico de potencia y otra encargada de la prueba.

Situar el carro centrado y la varilla cerca de la vertical arriba. Confirmar
cero, signos, sensores recientes y margenes de recorrido/angulo. Establecer
un tiempo maximo muy corto para el primer intento y un limite de PWM que ya
haya demostrado capacidad util en las pruebas previas. No hacer swing-up.

El operador habilita explicitamente la version de equilibrio y libera la
varilla solo cuando el estado indica que la prueba esta lista. No usar un
temporizador que arranque equilibrio al conectar USB ni despues de reset.
Registrar todos los estados, mando solicitado/aplicado, saturaciones, tiempos
y motivo de salida. Tras cada intento: salida cero, desarmado e inspeccion
del registro antes de otro armado. Un paro por limite o sensor nunca debe
rearmarse automaticamente.

El criterio inicial es observar que el carro responde en el sentido corrector
con esfuerzo y retardo compatibles con el modelo, y que las protecciones
detienen la prueba. Lograr algunos instantes de equilibrio no valida por si
solo el funcionamiento en todo el recorrido ni frente a bateria descargada.

## Registro de avance de las etapas

| Etapa | Evidencia exigida | Estado a completar con resultados reales |
|---|---|---|
| Adquisicion preservada | CSV sin perdidas; sensor y dt; comparacion con version anterior | Fuente base conservada; nueva carga y medicion fisica pendientes |
| Signos fisicos | Frente marcado; avance y regreso sin cambiar origen; inclinacion concordante | Pendiente de repetir para resolver ambiguedad previa |
| Protecciones sin potencia | Resultado de cada caso y diagnosticos | Pendiente |
| Ruedas levantadas | Pulsos individuales y ambos sentidos; parada y desarmado | Pendiente |
| Suelo | Respuesta por PWM/sentido, retardos, zona muerta, coast, saturacion | Pendiente |
| Modelo y LQR real | Modelo identificado, validacion separada, limites y ganancia discreta | Pendiente |
| Primer equilibrio protegido | Registro del intento fisico y causa de finalizacion | No habilitado en firmware manual |

Actualizar esta tabla solo con ensayos realizados. Compilar, simular o ver
graficas no equivale a haber hecho las pruebas fisicas.
