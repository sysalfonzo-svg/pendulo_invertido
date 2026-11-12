# Evidencia de la etapa de medicion — 2026-10-31

Se conservaron los archivos originales en `backups/antes_integracion_2026-10-31`.
El firmware nuevo se compilo y cargo con ST-Link: `Programming Finished`,
`Verified OK`, `Resetting Target`, `SUCCESS`. No se reinstalaron controladores.
El usuario confirmo que la alimentacion de 12 V estaba desconectada.

## Bloqueo encontrado y corregido

La primera captura real (`estados_20261031_185749_981`) fallo: periodo mediano
10 ms, lecturas AS5600 vencidas y perdidas. Se encontro `delay(10)` dentro de
`USBSerial::operator bool()` del core STM32duino 2.11 instalado. Se reemplazaron
los dos `if(!Serial)` por consultas `Serial.dtr()` sin espera y se recargo.
La captura fallida se conserva; no se usa como evidencia de funcionamiento.

## Resultados reales despues de corregir

| Prueba | Resultado |
|---|---|
| MATLAB sin ventana, 15 s | 3042 muestras; dt minimo/mediano/maximo = 5000 us |
| Sensor en esa captura | 0 lecturas invalidas; acq maximo 154 us |
| Perdidas en esa captura | seq sin huecos; incremento missed/tx_drop/I2C = 0 |
| Retraso de ISR | maximo 4 us |
| Recepcion USB pausada 5 s | 900 filas descartadas y contabilizadas, 0 ciclos de adquisicion perdidos |
| Durante pausa USB | dt 5000 us, sensor valido, acq maximo 155 us |
| MATLAB con ventana final, 10 s | 2036 muestras, dt 5000 us, 0 perdidas o errores |
| Con ventana final | acq maximo 176 us; las cuatro columnas de estado son finitas |
| Motores | PWM siempre 0; sin ruta de habilitacion en este firmware |

Archivos principales:

- `registros/estados_20261031_190135_935.csv/.mat`: captura sin ventana.
- `registros/prueba_usb_pausa5s.json/.csv/.txt`: saturacion de recepcion.
- `registros/estados_20261031_190918_594.csv/.mat`: GUI final comprobada.
- `registros/ventana_estados_con_datos.png`: ventana con datos y controles.

El contador tx_drop acumulado queda en 900 tras la prueba deliberada. Las
capturas MATLAB posteriores citadas tienen incremento cero; no es una averia nueva.
Un registro auxiliar Python con readline y escritura por linea
(`avance_20cm_en_vivo.txt`) descarto otras 196 filas en el transporte: tambien
mantiene dt=5000 us y missed=0. Se conserva como evidencia de un receptor lento,
no como captura de calidad aceptada. Para registrar usar MATLAB, que lee por
bloques, o `prueba_usb.py`, que tambien lee por bloques.
El flag 8 indica S enclavado, esperado por la parada al terminar las capturas.
No invalida la adquisicion; los motores siguen bloqueados permanentemente.

## Calibracion manual realizada con el usuario

- El usuario sostuvo el pendulo vertical ARRIBA. Z acepto `zero_raw=2650`.
- Una inclinacion hacia delante de unos 5–10 grados produjo -0,171806 rad con
  el signo provisional +1. Se establecio `signTheta=-1`.
- Inicialmente se aplico `D -1 1 -1`. La posterior prueba de 20 cm confirmada
  como avance exige corregirlo a **`D 1 -1 -1`**, ya aplicado: signL=+1,
  signR=-1, signTheta=-1. L es encoder registrado de rueda derecha; R, de
  izquierda. No se cambiaron pines ni sentidos de accionamiento de motores.
- La calibracion actual esta en `registros/calibracion_actual.json`: cero 2650,
  signos +1/-1/-1, PWM 0/0. D reinicio x=0 en el punto final de la prueba.
  Estos valores viven en RAM. Tras reiniciar, declarar los signos y fijar Z
  con el pendulo arriba; no enviar Z con el pendulo en otra posicion.

La primera comparacion de 10 cm incluyo un intervalo no controlado entre
referencias y registro 20,1/27,9 cm por rueda; no se uso para cambiar CPR.
Tras fijar N, el usuario confirmo que el siguiente movimiento fue de regreso:
se midieron -10,79 cm y -11,35 cm; media -11,07 cm frente a -10 cm declarados.
El retroceso era coherente con los signos provisionales de entonces. La
discrepancia de magnitud de 1,07 cm motivo la repeticion de 20 cm siguiente.

Se fijo un nuevo N con cuentas L=-899, R=1601 para comprobar un avance de
20 cm: `registros/origen_20cm.json`. El usuario confirmo haber avanzado los
20 cm hacia el mismo frente utilizado para inclinar el pendulo. Resultado:

| Magnitud | Valor |
|---|---|
| Cuentas finales L/R | 1031 / -400 |
| Incrementos L/R | +1930 / -2001 |
| Recorrido rueda derecha (L) | 20,010 cm |
| Recorrido rueda izquierda (R) | 20,700 cm |
| Media de magnitudes | 20,355 cm |
| Exceso frente a regla | 3,55 mm, 1,77% |
| Diferencia entre ruedas | 6,90 mm |
| Transiciones invalidas L/R | 0 / 0 |
| Periodo observado | 5000 us |

Con los signos anteriores x dio -0,203547 m. Tomando la ultima identificacion
de avance como referencia, se corrigieron los signos de conversion L/R a
+1/-1. Con ellos, ese mismo incremento corresponde a +0,203547 m. Se mantuvo
theta=-1. Datos originales y correccion separados en
`registros/avance_20cm_resultado.json`; recepcion en el TXT del mismo nombre.

Existe una ambiguedad fisica: el movimiento anterior declarado como regreso
y este ultimo declarado como avance produjeron incrementos del mismo signo.
No se afirma que ambas pruebas corroboren el sentido. La configuracion actual
se basa en la ultima confirmacion; antes del controlador debe marcarse un
frente fijo y repetir avance/retroceso. La comparacion de 20 cm apoya la escala
nominal en esta pasada, pero una sola prueba no establece exactitud/repetibilidad
ni justifica cambiar radio o CPR. Se conservan los parametros originales.

Datos trazables en `cero_vertical_manual.json`, `signo_angular_manual.json`,
`desplazamiento_manual_10cm.json`, `origen_repeticion_10cm.json` y
`repeticion_10cm.json`, dentro de `registros/`.

## Pruebas de software

- C++ host: 16.777.216 pares de angulos con ambos signos; rollover de contadores,
  mapeo de ruedas y filtro con dt variable. Compilacion sin advertencias.
- MATLAB R2025a: parser, NaN y sentinelas, rollover, huecos, flags y PWM indebido;
  `prueba_estados` pasa. Checkcode sin avisos en la version final comprobada.
- GUI inspeccionada visualmente con datos reales y signos activos mostrados.

`cuatro_estados_listos` del resumen MATLAB certifica coherencia de datos/flags,
no exactitud mecanica ni aptitud para un LQR. Quedan por caracterizar escala y
repetibilidad de odometria, precision del cero manual, filtro interno AS5600 y
retardos. La desconexion fisica del AS5600 y el giro angular completo no se
ensayaron; sus rutas de error/wrap se revisaron y el wrap se probo numericamente.
No se habilito LQR ni se convirtio la fuerza ideal a PWM.
