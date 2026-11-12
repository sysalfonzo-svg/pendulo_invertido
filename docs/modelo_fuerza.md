# Modelo fuente y exportacion para diagnostico — 2026-11-01

Fuente localizada: `D:/INFO PC/Downloads/modelo_inicial_pendulo.m`.
Se conserva una copia exacta en `matlab/modelo_inicial_pendulo.m`, SHA256:

`1d03253a3c87b435e75649c1d71c77787b2dd4c8d50c6fb36f4a48a10c46b18b`

La fuente declara entrada **F en newtons**, estados
`X=[x; vx; theta; omega]`, x positivo hacia delante y theta positivo inclinado
hacia delante, con cero vertical arriba. Usa Jp=0,00253432 kg*m^2 respecto al
pivote, Jcm=0,0006171775 kg*m^2 y Ts=0,005 s. Su LQR es discreto, con ZOH,
actuacion ideal, fricciones nominales b=c=0 y estados conocidos.

La ejecucion de esa fuente con Control System Toolbox produce:

```text
K_F = [-9.58072412873884, -8.65868827049033,
       -44.0193958551586, -8.04174123970425]
F_N = -K_F * [x_m; vx_m_s; theta_rad; omega_rad_s]
```

Unidades respectivas de K_F: N/m, N/(m/s), N/rad, N/(rad/s).
Los polos nominales son `0.9895739731 +/- 0.0071790694i` y
`0.9680353075 +/- 0.0143097321i`. Esto solo establece estabilidad del modelo
nominal de fuerza; no valida un controlador sobre PWM.

## Regenerar y verificar las exportaciones

Desde la carpeta del proyecto:

```matlab
addpath('matlab')
modelo = preparar_lqr_fuerza();
```

El exportador ejecuta la copia original intacta en el scope de una funcion,
oculta y cierra solo las figuras que esa fuente crea, y restaura la visibilidad
anterior. No abre puertos ni interactua con motores. Requiere MATLAB con Control
System Toolbox. Puede recibir otra ruta explicita al mismo script de confianza.

Antes de exportar verifica Jp y Ts, dimensiones y estructura de los cuatro
estados, A/B contra las ecuaciones mecanicas, discretizacion ZOH, dlqr y polos.
Produce:

- `modelos/modelo_fuerza.mat`: matrices, ganancias y polos en doble precision,
  parametros, unidades, convenciones y SHA256 de la fuente.
- `include/lqr_force_model.h`: constantes `float` para STM32 y la funcion
  `lqr_force_model::diagnosticForceN(x,vx,theta,omega)`. La salida es **solo
  fuerza solicitada para diagnostico**, sin conversion a PWM ni habilitacion.

No editar manualmente el header ni copiar ganancias de una fuente distinta.
Los valores A/B/Ad/Bd/K_F y polos obtenidos durante la auditoria tambien quedan
en `docs/modelo_fuente_salida.txt`.

## Lo que falta para usar el actuador real

1. Fijar un frente fisico unico y verificar avance/retroceso de odometria y el
   signo angular. Para theta positivo, esta ley ideal solicita fuerza positiva:
   el carro debe desplazarse hacia la inclinacion. El avance de los motores
   sigue siendo la referencia historica `C -40 -40`; esto no convierte N a PWM.
2. Medir por separado y juntos ambos motores: umbral de arranque, mantenimiento,
   asimetria por sentido, tiempo de respuesta, parada por giro libre y efecto de
   bateria/carga. Los modelos RPM/%PWM con ruedas levantadas no son modelos de
   fuerza de traccion sobre el suelo.
3. Separar fuerza de traccion y resistencias. Las aceleraciones netas de 40/50%
   y la desaceleracion libre no identifican por si solas un mapa global F-PWM;
   menos aun cerca de la zona muerta, en reposo o en inversion.
4. Validar un modelo con saturacion, zona muerta, retardos del motor, adquisicion
   y filtros. La tau mecanica inestable nominal es aproximadamente 0,187 s,
   comparable a las tau identificadas de motores (~0,16 s); ignorar esa dinamica
   no es una aproximacion ya validada.
5. Determinar si un lazo interno permite realizar la fuerza solicitada por K_F
   con suficiente ancho de banda, o si hace falta aumentar el modelo con los
   estados del actuador y redisenar el LQR discreto. En ambos casos, comprobar
   capacidad, saturacion y region inicial recuperable antes de habilitarlo.

La fuente nominal solicita inicialmente unos 2,30 N para theta=3 grados con
los demas estados en cero. Ese valor no demuestra que el L298N y los motores
puedan producirlo, ni define un limite admisible. En esta etapa K_F se exporta
para comparar demanda con capacidad medida; no se conecta a las salidas PWM.
