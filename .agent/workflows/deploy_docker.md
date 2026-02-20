---
description: Containerización y despliegue del sistema Mouseofono.
---

# Workflow: Despliegue en Docker y WSL2

Este workflow automatiza la creación del entorno de ejecución aislado para Mouseofono.

## Pasos

1. **Configuración del Dockerfile**:
   - Usar una imagen base ligera como `ubuntu:22.04` o `debian:bullseye`.
   - Instalar dependencias de compilación: `build-essential`, `cmake`, `libc6-dev`.
   - Multi-stage build: Compilar en la primera fase y copiar solo el binario a la imagen final.

2. **Configuración de Permisos USB**:
   - Documentar la necesidad de la flag `--privileged` o detallar las capabilities mínimas (`SYS_RAWIO`).
   - Mapear el dispositivo mediante el volumen `--device`.

3. **Manejo de Volúmenes**:
   - Configurar un mount para la salida del WAV: `-v $(pwd)/out:/app/out`.

4. **Script de Lanzamiento (run.sh)**:
   - Crear un script que valide la existencia del device antes de lanzar Docker.
   - Detectar automáticamente el path del dispositivo si es posible.

5. **Pruebas de Contenedor**:
   - Lanzar el contenedor y monitorizar los logs.
   - Verificar que el archivo WAV se genera y es accesible desde fuera del contenedor.

6. **Output Final**: Proyecto listo para ser distribuido como una imagen Docker, minimizando la fricción de configuración de dependencias en diferentes máquinas WSL.
