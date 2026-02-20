---
description: Implementación del núcleo C++ para lectura de evdev y escritura WAV.
---

# Workflow: Implementación del Core C++

Este workflow guía la codificación de la aplicación principal de Mouseofono siguiendo las fases del roadmap.

## Pasos

1. **Inicialización de Proyecto**:
   - Crear `CMakeLists.txt` configurando el estándar C++17.
   - Estructurar los directorios `src/core`, `src/driver`, `src/util`.

2. **Clase WavWriter**:
   - Implementar la escritura del header RIFF de 44 bytes.
   - Implementar método `writeSamples(vector<int16_t>)`.
   - Implementar `close()` que realice el `fseek` y parcheo de tamaños.

3. **Clase EvdevReader**:
   - Abrir el archivo de dispositivo pasándole el path por CLI.
   - Implementar bucle de lectura bloqueante sobre `struct input_event`.
   - Filtrar eventos de movimiento y enviarlos a un callback o cola.

4. **Ring Buffer SPSC**:
   - Implementar o integrar un ring buffer lock-free de una sola dirección (Lector -> Renderer).
   - Asegurar que el tamaño sea potencia de 2 para optimizar el operador modulo con bitwise AND.

5. **Renderer Loop**:
   - Implementar el hilo de renderizado que corre a `polling_rate`.
   - Sincronización usando `std::this_thread::sleep_until` para evitar deriva temporal.
   - Aplicar el mapeo de señal seleccionado.

6. **Manejo de Señales**:
   - Utilizar `std::atomic<bool> g_running` para controlar el cierre de los hilos.
   - Capturar `SIGINT` (Ctrl+C).

7. **Build y Ejecución Inicial**:
   - Compilar con `mkdir build && cd build && cmake .. && make`.
   - Ejecutar la Phase 1 (MVP) directamente contra `/dev/input/eventX`.

8. **Output Final**: Una aplicación funcional capaz de generar un archivo WAV audible moviendo el ratón.
