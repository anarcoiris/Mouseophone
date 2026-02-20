---
description: Fase de investigación para definir la estrategia de audio y procesamiento de señal.
---

# Workflow: Research de Audio y Estrategias de Mapeo

Este workflow establece las bases matemáticas y algorítmicas para la conversión de deltas de movimiento en señales de audio coherentes. Al finalizar, generará el plan de implementación para las **Fases 4 y 5**.

## Pasos

1. **Definición de Formato WAV**:
   - Validar el header RIFF. Documentar los offsets exactos de los campos `FileLength` y `DataLength` para el parcheo posterior.
   - Decidir entre Mono y Stéreo según el modo de mapeo.

2. **Modelado de Mapeo de Amplitud**:
   - Investigar funciones de transferencia para deltas. 
   - Probar `sample = gain * dx` versus funciones no lineales para evitar clipping digital (ej. `atan` o `tanh`).
   - Definir el valor de `gain` por defecto basado en un ratón de DPI estándar (ej. 800-1600 DPI).

3. **Modelado de Síntesis de Frecuencia (FM)**:
   - Definir frecuencia base (Carrier, ej. 440 Hz).
   - Definir índice de modulación basado en la velocidad `sqrt(dx^2 + dy^2)`.
   - Establecer límites de frecuencia para evitar subgraves o ultrasonidos inaudibles.

4. **Investigación de Resampling (48 kHz)**:
   - Calcular el factor de sobremuestreo (Upsampling Factor `L`). Si pollhz=1000, `L = 48`.
   - Investigar el diseño de un filtro de FIR (Polyphase Filter) para eficiencia.
   - Determinar el número de "taps" del filtro necesarios para una caída de 60dB en la banda de rechazo.

5. **Simulación de Streaming WAV**:
   - Investigar el comportamiento de las syscalls `write()` en Linux sobre archivos en volúmenes Docker.
   - Validar si el buffer de I/O del SO es suficiente o si la app C++ debe manejar su propio ring buffer de salida.

6. **Output Final**: Crear un archivo `impl_plan_audio_v1.md` con las fórmulas de mapeo finales, coeficientes del filtro FIR (si aplica) y la especificación de la CLI para cambiar entre modos de audio.
