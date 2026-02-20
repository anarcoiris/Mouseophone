# Mouseofono Project Roadmap

Este roadmap detalla las fases de desarrollo para Mouseofono, un sistema para convertir eventos de ratón en audio WAV.

| Fase | Alcance | Criterio de Aceptación |
|---|---|---|
| **Phase 0 — Entorno** | Validar WSL2 + usbipd + evdev | `evtest` muestra eventos REL_X/REL_Y del ratón en WSL de forma fluida. |
| **Phase 1 — MVP** | App C++ básica (single-thread) que lee de `/dev/input/event*` y escribe un WAV simple con `fs = polling_rate`. | Al mover el ratón se genera un archivo WAV que contiene la representación directa de los deltas como amplitud. |
| **Phase 2 — Multi-hilo** | Separación en Reader Thread (bloqueante) y Renderer Thread (timer-based) usando un ring buffer lock-free. | Cero pérdida de eventos a 1000 Hz durante sesiones de captura de larga duración (60s+). |
| **Phase 3 — Docker** | Containerización completa. Empaquetar la app C++ en una imagen Docker que accede al dispositivo vía `--device`. | `docker run` genera un audio WAV idéntico al proceso ejecutado de forma nativa en WSL. |
| **Phase 4 — Resampling** | Implementación de upsampling a 48 kHz con filtrado FIR Low-Pass (LPF) para evitar aliasing. | El audio resultante es reproducible en reproductores estándar sin distorsiones por aliasing. |
| **Phase 5 — Síntesis Avanzada** | Mapeos múltiples: amplitud (soft-clip), frecuencia (oscilador) y estéreo (dx=L, dy=R). | Al menos 3 modos de síntesis seleccionables mediante argumentos de línea de comandos. |
| **Phase 6 — Pulido & Doc** | Manejo de señales (SIGINT), métricas de performance, configuración avanzada y README final. | El sistema se cierra de forma limpia (parcheando el header WAV) y está listo para uso general o desarrollo futuro. |

## Futuras Expansiones (Fuera de MVP)
- **Phase 7 — Raw Input Windows**: Alternativa sin WSL usando Raw Input API de Windows y sockets/pipes.
- **Phase 8 — Hardware Directo**: Soporte para lectura de sensor vía SPI con un MCU (Teensy/STM32) para kHz reales.
- **Phase 9 — Interfaz Gráfica**: Panel de control para visualización de formas de onda en tiempo real.
