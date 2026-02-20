# Catálogo de Casos de Error y Mitigaciones

Este documento sirve como bitácora de errores previstos y soluciones encontradas durante el desarrollo de Mouseofono.

## ⚠️ Errores Reales Encontrados (Verificado en campo)

| ID | Error | Contexto | Causa | Solución Correcta |
|---|---|---|---|---|
| R01 | `usbipd: error: The 'wsl' subcommand has been removed` | Windows PowerShell al intentar `usbipd wsl attach --busid X --distribution Ubuntu` | En versiones modernas de `usbipd-win` (≥ v4.x), el subcomando `wsl` fue eliminado. | Usar la nueva sintaxis: `usbipd attach --busid 1-5 --wsl` o `usbipd attach --busid 1-5`. Ver: https://learn.microsoft.com/windows/wsl/connect-usb |
| R02 | `output.wav` vacío (silencio completo) en modo Windows Native | Ejecutar `mouseofono.exe` mientras el dispositivo está en estado `Shared` en usbipd | Cuando el mouse está "Shared", usbipd reemplaza el driver HID de Windows. Raw Input ya no recibe eventos del dispositivo. | Ejecutar `usbipd unbind --busid <X>` como Administrador antes de ejecutar el binario de Windows. |
| R03 | `output.wav` vacío aunque el ratón generate eventos | Renderer descarta todos los eventos del ring buffer | Bug de timestamps: el Reader usaba tiempo QPC absoluto (ej. `1234567.891 s`), mientras que el Renderer usaba `chrono` desde `t=0`. La ventana de comparación `t_current - event.t` era siempre muy negativa. | Unificar ambos lados con QPC. El Renderer debe "bootstrapear" su `t_render` desde el primer evento recibido. |



## Infraestructura (USB & WSL)

| ID | Error | Contexto | Mitigación / Solución |
|---|---|---|---|
| E01 | `usbipd: error: Access is denied` | Windows Host: Al intentar `attach` | Ejecutar PowerShell como Administrador. |
| E02 | `device not found` | Docker: Al iniciar contenedor | Verificar que el `busid` no haya cambiado tras reconexión. El contenedor debe relanzarse si el host cambia el path del device. |
| E03 | Polling Rate inestable | WSL: Jitter en la llegada de eventos | Usar el timestamp del kernel (`input_event.time`) en lugar del clock de la aplicación para mayor precisión temporal. |
| E04 | Permisos `/dev/input/` | WSL/Docker: Permission Denied | Crear regla udev en WSL: `KERNEL=="event*", GROUP="input", MODE="0660"`. O ejecutar Docker con `--privileged`. |

## Aplicación C++

| ID | Error | Contexto | Mitigación / Solución |
|---|---|---|---|
| E05 | WAV Corrupto 0 bytes | Shutdown repentino (Ctrl+C) | Registrar un signal handler para `SIGINT` y `SIGTERM` que llame a la función `close()` del WAV Writer para parchear el header. |
| E06 | Ring Buffer Overflow | Renderer thread demasiado lento | Incrementar el tamaño del buffer (potencia de 2, ej. 65536). Monitorizar y loguear "Buffer Overrun" si ocurre. |
| E07 | Límite 4GB WAV | Capturas muy largas (>6 horas de audio) | Implementar límite de seguridad. El formato WAV RIFF está limitado a 4GB por sus campos de 32 bits. |
| E08 | `read()` interrumpido (EINTR) | Lectura de `/dev/input` | Manejar error `EINTR`: `if (errno == EINTR) continue;` para reintentar la lectura sin abortar. |
| E09 | Latencia de Disco | Escritura de audio en tiempo real | Usar un buffer intermedio antes de llamar a `write()` en disco. Evitar escrituras de 2 bytes (1 sample) cada vez. |

## Audio y Calidad

| ID | Error | Contexto | Mitigación / Solución |
|---|---|---|---|
| E10 | Aliasing Audible | Upsampling a 48 kHz | Diseñar el filtro FIR con un cutoff estricto a la mitad del polling rate (Nyquist). |
| E11 | Clipping / Distorsión | Movimientos rápidos del ratón | Usar funciones de "soft-clipping" como `tanh()` o limitadores automáticos en el mapeo de amplitud. |
| E12 | Silencios imprevistos | Ratón quieto | El Renderer Thread debe emitir samples con valor 0 de forma constante aunque no haya eventos del Reader. |
| E13 | Endianness incorrecto | Audio suena como ruido blanco | Asegurar que los datos se escriben como `Little Endian` (el estándar de WAV), independientemente de la arquitectura del host. |
