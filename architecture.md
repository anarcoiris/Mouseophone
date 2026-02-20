# Arquitectura de Mouseofono

Este documento detalla la estructura técnica y el flujo de datos de Mouseofono.

## 1. Flujo de Datos General

El camino que sigue un evento desde el hardware hasta el archivo de audio es el siguiente:

1. **Hardware**: El ratón genera eventos USB.
2. **Windows**: `usbipd-win` captura el dispositivo y lo reenvía a WSL2.
3. **WSL2 Kernel**: Registra el dispositivo bajo `/dev/input/eventX` usando el driver `evdev`.
4. **Docker Container**: Monta el dispositivo específico (`--device /dev/input/eventX`) con los permisos necesarios.
5. **C++ App (Reader Thread)**: Lee estructuras `input_event` desde el file descriptor de forma bloqueante.
6. **Ring Buffer**: Los eventos filtrados (REL_X, REL_Y) se colocan en un buffer SPSC (Single Producer Single Consumer) lock-free.
7. **C++ App (Renderer Thread)**: Un bucle disparado por timer (ej. 1ms si 1000Hz) consume los eventos acumulados.
8. **Síntesis/Mapeo**: Transforma deltas de posición en amplitud o frecuencia de audio.
9. **WAV Writer**: Escribe bloques de samples en el archivo `.wav`.
10. **File System**: El archivo está disponible en un volumen compartido para ser reproducido en Windows Host.

## 2. Modelo de Threading e Interfaz de "Micrófono"

Para que el ratón actúe como un sensor de vibración (micrófono), la captura debe ser **determinista y continua**, incluso si no hay movimiento perceptible:

### Reader Thread (Productor)
- **Captura Silenciosa**: Se encarga de recibir cada micro-delta del sensor. En ratones de alto polling (1000Hz), las vibraciones mecánicas se traducen en ráfagas de eventos `dx`/`dy` mínimos.
- **Timestamping**: Crucial para el análisis de ruido (SNR). Cada evento se marca con el tiempo de llegada de alta precisión (QPC en Windows, `timeval` en Linux).

### Renderer Thread (Consumidor)
- **Timeline Continua**: A diferencia de una app de mouse estándar, el Renderer **siempre** genera muestras a la frecuencia de salida (ej. 16kHz o 48kHz). Si no hay eventos, genera muestras de valor cero.
- **Cadena de Procesado Avanzada (v1.2)**: 
  - **Calibración**: Captura del PSD (Power Spectral Density) del ruido del sensor óptico.
  - **Filtro Wiener**: Aplicación de sustracción espectral en tiempo real sobre frames FFT para eliminar el "sensor jitter".
- **Análisis de SNR**: La arquitectura permite grabar el "silencio" del ratón para medir el ruido de fondo del sensor óptico versus la vibración real inducida mecánicamente.



## 3. Formato de Audio WAV

Se utiliza el formato **WAV RIFF PCM 16-bit Little-Endian**.

- **Streaming Header**: Al iniciar, se escribe un header con campos de tamaño en 0.
- **Data Blocks**: Se escriben samples en bloques de N frames (ej. 1024) para minimizar syscalls de disco.
- **Finalización**: Al recibir una señal de cierre (SIGINT), se realiza un `fseek` al inicio del archivo para actualizar los campos `file_size` y `data_size` basándose en los bytes reales escritos.

## 4. Estrategia de Mapeo y Síntesis

| Fase | Alcance | Criterio de Aceptación |
|---|---|---|
| **Phase 0 — Entorno** | Validar WSL2 + usbipd + evdev | `evtest` muestra eventos REL_X/REL_Y incluso por micro-vibraciones. |
| **Phase 1a — MVP** | App C++ dual (Win/Linux) con captura continua y análisis de SNR. | El WAV generado tiene una línea de tiempo real. Se puede distinguir el ruido del sensor (estático) de la vibración inducida. |
| **Phase 1b — Optimización** | Filtro Wiener y calibración espectral (Basado en arXiv:2509.13581v2). | El ruido de sensor ("sensor jitter") se reduce drásticamente tras la calibración, aumentando la claridad de la vibración capturada. |

## 5. Resampling (Upsampling)

Si se requiere una frecuencia de salida estándar (ej. 48 kHz) a partir de un polling de 1 kHz:
- **Interpolación**: Los samples originales se expanden.
- **Filtro Anti-Alias**: Se aplica un filtro FIR (Finite Impulse Response) pasa-bajo diseñado con ventana Hamming para eliminar el ruido de alta frecuencia introducido por el proceso de upsampling.
