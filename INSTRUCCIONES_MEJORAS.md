# Control LQR anti-oscilacion v2 — STM32F401RC

Esta version se construyo a partir del main.cpp entregado en `Texto pegado(4).txt` y
los encabezados de la ultima entrega `carro_pendulo_LQR_autoencendido_2026-11-02.zip`.
**No hay cambios de cables ni de pines**. Autoarranque, IWDG y protecciones se conservan.

## Cambios del control

1. **Comando subumbral:** antes se forzaba 25% (L) y 30% (R) ante cualquier
   PWM no nulo. Ahora el mando virtual subumbral se modula en densidad temporal
   de pulsos de 25/30, en la frecuencia de control 200 Hz. El promedio se
   aproxima al requerido. Es experimental; con inercia/rozamiento la fuerza
   NO es necesariamente proporcional al promedio de PWM.
2. **Demanda de aceleracion:** escala inicial de 0.85 (antes 1.0), editable en
   `include/auto_lqr.h` como `FORCE_TO_ACCEL_SCALE`. **No se cambio K**.
3. **Giro inverso:** salida apagada ~10-15 ms tras pedir sentido opuesto;
   previamente se esperaban ~30-35 ms. Se aplica rampa de comandos.
4. **Zona de reposo:** `FORCE_DEADBAND_N=0.04f` y |v|<0.012 m/s.
5. **Trim de cero angular:** `ANGLE_ZERO_TRIM_DEG=0.0f` en `src/main.cpp`.
   Solo ajustar si se conoce la lectura del sensor cuando el eje esta
   exactamente vertical: si el codigo lee +0.5 grados en la vertical,
   configurar +0.5, ya que se resta de theta.

## Instrucciones

1. Situar el carro en una zona libre con topes y un corte fisico de bateria.
2. Abrir la carpeta completa en VS Code con PlatformIO.
3. Para el perfil que conserva el UART de la fuente facilitada (`ENABLE_ESP32_TX=1`):
   `pio run -e lqr_anti_oscilacion`
   `pio run -e lqr_anti_oscilacion -t upload`
4. Si tu montaje utiliza USB CDC en lugar del UART (PA11): utilizar el perfil
   `lqr_anti_oscilacion_usb`, que deja la telemetria USART6 desactivada.
5. La STM32 sigue arrancando el control cuando la placa recibe energia y se
   cumplen 100 lecturas validas con el pendulo casi vertical.
6. Ante oscilacion mayor, fallo o comportamiento inverso, cortar potencia.

## Ajustes de una variable a la vez

- Primera prueba: `ANGLE_ZERO_TRIM_DEG=0.0`, `FORCE_TO_ACCEL_SCALE=0.85`.
- Si el carro responde tarde/no recupera el angulo: probar, con cuidado,
  `FORCE_TO_ACCEL_SCALE=1.00`. NO cambiar K ni PWM maximo simultaneamente.
- Si oscila aun con errores muy pequenos, observar primero los pulsos minimos,
  direccion del carro y posible desplazamiento del cero.
- No desplazar el cero angular a ojo. El sensor AS5600 tiene 4096 posiciones/vuelta:
  aproximadamente 11.38 cuentas por grado.

## Importante

Se probaron los algoritmos portables con g++, pero esta entrega no se ha
compilado con el toolchain STM32 real ni verificado fisicamente. El mapeo
fuerza->PWM sigue siendo empirico y no garantiza estabilidad.

**Telemetria:** el bloque ESP32 UART/DMA de tu archivo se mantiene sin cambios.
No se agregan nuevos protocolos o conexiones en esta etapa.
