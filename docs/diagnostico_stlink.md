# Diagnóstico de ST-Link — 2026-10-31

La conexión ST-Link funciona en la comprobación actual. El error anterior
`OpenOCD init failed / hla_swd / Error: open failed` **no se reprodujo**.
No se conoce la causa del fallo histórico; las comprobaciones actuales no
justifican atribuirlo al controlador de Windows ni al COM17.

## Dispositivos encontrados

| Dispositivo | Identificación | Estado observado |
| --- | --- | --- |
| STM32 STLink | `USB\VID_0483&PID_3748\5&2BB0E0F&0&1` | OK; código de problema 0 |
| USB CDC de la STM32 | `USB\VID_0483&PID_5740\207F356D534D`, COM15 | OK |
| Puerto de comunicaciones COM17 | `ROOT\PORTS\0000` | Error `CM_PROB_NO_SOFTCONFIG` |

COM17 es una instancia distinta del ST-Link y del USB CDC de la STM32.
No se modificó ni eliminó este dispositivo. El número COM15 puede cambiar
al reconectar la placa; hay que volver a identificar el puerto para MATLAB.

El ST-Link tiene instalado el controlador de STMicroelectronics versión
`2.2.0.0`, servicio `WinUSB`, archivo INF `oem128.inf`. Windows informa
`DEVPKEY_Device_HasProblem = False`. No se reinstaló el controlador.
La descripción oficial del paquete está en
[STSW-LINK009, controlador USB de ST-Link](https://www.st.com/resource/en/data_brief/stsw-link009.pdf).

## Evidencia de conexión

Se utilizó el OpenOCD instalado por PlatformIO:

```text
xPack Open On-Chip Debugger 0.12.0-01004-g9ea7f3d64-dirty
STLINK V2J37S7 (API v2) VID:PID 0483:3748
Cortex-M4 r0p1 processor detected
target has 6 breakpoints, 4 watchpoints
```

Las dos comprobaciones terminaron con código de salida 0:

| Reloj SWD | Tensión objetivo informada |
| --- | --- |
| 1000 kHz | 3.285060 V |
| 2000 kHz, configuración predeterminada de `stm32f4x.cfg` | 3.264065 V |

Se abrió y examinó la conexión mediante `init`, sin escribir firmware ni
solicitar `reset` o `halt`, sin instalar controladores y sin enviar comandos
seriales de motores. OpenOCD terminó al finalizar cada comprobación.
La detección de Cortex-M4 confirma comunicación con el núcleo; no verifica
por sí sola el encapsulado, el tamaño de flash o todos los periféricos.

## Repetir la comprobación de conexión

En PowerShell:

```powershell
& 'C:/Users/Sneider/.platformio/packages/tool-openocd/bin/openocd.exe' `
  -s 'C:/Users/Sneider/.platformio/packages/tool-openocd/openocd/scripts' `
  -f interface/stlink.cfg `
  -c 'transport select hla_swd' `
  -f target/stm32f4x.cfg `
  -c 'adapter speed 1000' `
  -c 'gdb_port disabled' `
  -c 'tcl_port disabled' `
  -c 'telnet_port disabled' `
  -c 'init; targets; shutdown'
```

Para comprobar los 2000 kHz predeterminados, omitir la línea
`-c 'adapter speed 1000'`. No hace falta cambiar esta velocidad en el
proyecto según los resultados obtenidos.

Para volver a identificar los dispositivos USB y puertos de interés:

```powershell
Get-PnpDevice -PresentOnly |
  Where-Object {
    $_.FriendlyName -match 'STM32|ST-Link|STLink|COM17' -or
    $_.InstanceId -match 'VID_0483'
  } |
  Format-List Status,Class,FriendlyName,InstanceId,Problem
```

## Carga del firmware integrado

La carga por ST-Link usa su interfaz USB de depuración y los cables SWD
hacia el microcontrolador. **No utiliza COM15 ni COM17**. El USB CDC de la
STM32 se utiliza después para telemetría y comandos desde MATLAB.

Mantener en `platformio.ini`:

```ini
upload_protocol = stlink
```

Desde la carpeta del proyecto, compilar y cargar el firmware preparado:

```powershell
& 'C:/Users/Sneider/.platformio/penv/Scripts/pio.exe' run -e genericSTM32F401RC
& 'C:/Users/Sneider/.platformio/penv/Scripts/pio.exe' run -e genericSTM32F401RC -t upload
```

La segunda orden **sí escribe y reinicia la placa**; no forma parte de las
pruebas de conexión documentadas arriba. El resultado de una carga y de la
medición integrada debe comprobarse por separado. Conservar los motores
deshabilitados durante esta etapa.

Si vuelve a aparecer `open failed`, guardar la salida completa y volver a
comprobar la presencia de `VID_0483&PID_3748`, el estado del controlador,
la alimentación, el cableado SWD y que no haya otro proceso de depuración
usando el adaptador. No cambiar el controlador únicamente por la advertencia
de COM17.
