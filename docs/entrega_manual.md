# Entrega verificada de software — 2026-11-01

El proyecto queda listo para compilar/cargar y efectuar las pruebas de
adquisicion y accionamiento manual. No contiene un controlador PWM de
equilibrio ejecutable: falta identificar y validar el accionamiento fisico.
La exportacion LQR del modelo original se integra exclusivamente como
diagnostico de fuerza, sin una ruta que la aplique a los motores.

## Conservacion de la adquisicion

Antes del trabajo, src/main.cpp habia vuelto a la identificacion antigua.
Se guardo esa version y sus binarios en `backups/antes_modo_manual_2026-11-01/`.
Se recupero la fuente validada anterior y se comprobo su SHA256 exacto:

`ab98b68a8bb79a341342a86ce95d8ae8537222ae66180488a84ad605227a364d`

La copia esta en `backups/adquisicion_validada_2026-10-31/main.cpp`.
`tests/verificar_base.py` verifica esa identidad y que siguen intactos el
estimador state_math.h, decodificacion de encoders, lectura I2C asincrona,
servicio de muestras, salida USB limitada por capacidad y cabecera CSV.
TIM2 mantiene el periodo programado de 5000 us; se agrega su vigilancia de
seguridad y se capturan los PWM junto con las cuentas de cada tick.

## Funciones incorporadas

- PWM TIM1 a 1 kHz, independiente de TIM2. Los pines y direcciones del
  firmware original se conservan; C negativo sigue siendo el avance historico.
- DISARMED al arranque. A MANUAL solo habilita una ventana; C inicia un
  unico pulso, tras validar salud y limites. Fin de pulso vuelve a DISARMED.
- S/! y los fallos activos quedan enclavados. E CLEAR no arma. No hay
  reintentos, inversiones o secuencias de movimiento automaticas.
- Sensor/angulo/omega/encoder, comunicacion, recorrido y tiempos supervisados.
  IWDG se refresca solamente al procesar una adquisicion completa en foreground.
- Los cambios de politica y de salida GPIO se realizan bajo la misma seccion
  critica. El corte fuerza los enables a GPIO LOW, ademas de quitar TIM1 MOE.
- Los timestamps de seguridad dentro de las secciones criticas compensan un
  SysTick pendiente; se conserva el reloj original de la adquisicion.
- Parser ASCII estricto: NUL, controles y exceso de longitud producen parada;
  no se aceptan desbordamientos numericos ni argumentos ocultos.
- Cliente MATLAB con botones explicitos, latido K, registro CSV/MAT y graficas
  de datos reales. Deshabilita botones A/C en el perfil solo_medicion.
- Analisis offline de pulsos: recorridos por rueda, RPM, latencia observada,
  velocidades y giro libre. Rechaza perdidas/errores y pulsos incompletos.
- Fuente MATLAB copiada intacta, A/B/Ad/Bd/K_F exportados y comprobados con
  Jp=0.00253432 y X=[x;vx;theta;omega]. F_N tiene unidades N, nunca porcentaje.

## Comprobaciones ejecutadas

Comando: `powershell -ExecutionPolicy Bypass -File tools/verificar_entrega.ps1`.
Finalizo correctamente. Log conservado en `docs/verificacion_software.log`.

| Comprobacion | Resultado |
|---|---|
| PlatformIO genericSTM32F401RC | SUCCESS; RAM 10020 B, flash 57372 B |
| PlatformIO solo_medicion | SUCCESS; RAM 10020 B, flash 56836 B |
| Fuente de adquisicion recuperada / funciones protegidas / CSV | PASS |
| Matematica C++ | 16.777.216 pares angulares, ambos signos; rollover; CPR; filtros: PASS |
| Politica de motores C++ | Plazos, saturaciones rechazadas, fallos enclavados, lease, cooldown y rollover: PASS |
| Parser C++ | 256 bytes, NUL, limites enteros, desbordes y argumentos invalidos: PASS |
| MATLAB estados y protocolo manual | PASS |
| MATLAB analisis de pulsos | Datos sinteticos, mapeo, rollover, calidad, CSV/MAT: PASS |
| Exportacion del modelo fuente | Jp/Ts/estructura de estados, discretizacion y ganancias comprobadas |
| MATLAB checkcode de todos los .m | Sin avisos |

Estas pruebas no abrieron COM ni enviaron maniobras. **El firmware nuevo no
se cargo ni se probo con motores en esta etapa.** La verificacion fisica de
los 5 ms sin perdidas pertenece a la version anterior; debe repetirse con
el perfil solo_medicion antes de armar la version manual. El cliente manual
no se ha ensayado aun con esta version en una placa real.

## Pendiente antes de equilibrio

1. Cargar sin 12 V y repetir la captura de estados y las pruebas de corte.
2. Marcar un frente fijo y resolver la ambiguedad historica de signos con
   avance/retroceso manuales sin cambiar de origen entre ambos.
3. Probar pulsos por motor, ambos sentidos y parada, con ruedas levantadas.
4. Medir accionamiento sobre suelo, zona muerta, asimetrias, retardos,
   saturacion y recorrido de giro libre. No igualar aceleracion neta a traccion.
5. Construir/validar la planta con el actuador y recalcular/verificar el LQR
   discreto necesario, con filtros, limites y margen real de actuacion.
6. Habilitar una version posterior de equilibrio y ejecutar el primer ensayo
   con proteccion mecanica y corte fisico, segun `pruebas_fisicas.md`.

No basta con cambiar una bandera en este firmware para equilibrar: no se ha
inventado un mapa fuerza-PWM ni conectado K_F directamente a analogWrite.

## Fuentes tecnicas

La implementacion de TIM1/IWDG se apoya en el
[manual STM32F401 RM0368](https://www.st.com/resource/en/reference_manual/DM00096844.pdf).
El comportamiento EN bajo corresponde a parada por giro libre segun el
[datasheet L298](https://www.st.com/resource/en/datasheet/l298.pdf).
Se inspeccionaron tambien las fuentes locales del core STM32duino 2.11:
USBSerial, HardwareTimer, HAL I2C, HAL IWDG y reloj SysTick.
