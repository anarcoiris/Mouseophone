---
description: Fase de investigación de hardware para validar la cadena de eventos del ratón.
---

# Workflow: Research de Hardware y Cadena de Eventos

Este workflow guía la investigación para validar que los eventos del ratón llegan correctamente desde el hardware físico hasta el entorno WSL2/Docker. Al finalizar, este workflow generará un plan de implementación para la **Phase 0** y **Phase 1**.

## Pasos

1. **Prerrequisitos en Windows**:
   - Verificar instalación de `usbipd-win`. Si no está, instalar vía `winget install usbipd`.
   - Asegurarse de que WSL2 está habilitado y actualizado (`wsl --update`).

2. **Localización del Dispositivo**:
   - Abrir PowerShell como Administrador.
   - Ejecutar `usbipd list` para identificar el `BUSID` del ratón.
   - // turbo
   - **Nota**: Si el ratón es compartido con el host, usbipd podría requerir `--force` o bindeo del dispositivo.

3. **Anclaje a WSL (Attach)**:
   - Ejecutar: `usbipd wsl attach --busid <BUSID> --distribution <NombreDistro>`.
   - Verificar en la terminal de WSL que el kernel ha registrado un nuevo dispositivo.

4. **Identificación de Evento en WSL**:
   - En WSL, instalar herramientas: `sudo apt update && sudo apt install evtest -y`.
   - Ejecutar `sudo evtest` y seleccionar el dispositivo que indique ser el ratón (usualmente bajo `/dev/input/event*`).
   - Mover el ratón y confirmar que se imprimen eventos `REL_X`, `REL_Y`.

5. **Medición del Polling Rate**:
   - Capturar una ráfaga de eventos durante 5 segundos moviendo el ratón de forma agresiva.
   - Contar el número de eventos `EV_SYN` recibidos. `polling_rate = total_syn / total_segundos`.
   - Identificar si el polling es fijo (125, 500, 1000 Hz) o variable.

6. **Validación Docker**:
   - Crear un contenedor de prueba montando el dispositivo: 
     `docker run --rm -it --device /dev/input/eventX ubuntu:latest ls -l /dev/input/`
   - Verificar visibilidad y permisos dentro del contenedor.

7. **Generación de Reporte**:
   - Documentar el path exacto del dispositivo (`/dev/input/eventX`).
   - Documentar el polling rate detectado.
   - Documentar cualquier conflicto de drivers (ej. ratón se deja de mover en Windows al hacer attach).

8. **Output Final**: Crear un archivo `impl_plan_phase_0.md` en el directorio de investigación detallando los pasos exactos para automatizar el attach y la lectura inicial para la Phase 1.
