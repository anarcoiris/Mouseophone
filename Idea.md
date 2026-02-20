Resumen rápido

En Windows: usa WSL2 y publica el ratón en WSL (usb passthrough).

En WSL: monta /dev/input/eventX en el contenedor.

En el contenedor (imagen Docker): compila la app C++ que lee evdev y escribe un WAV.

Guardas el WAV en un volumen compartido y lo reproduces en Windows.

(Entidades técnicas que usarás: WSL, Docker, evdev.)

1) Requisitos y decisiones de diseño

Objetivo: WAV cuya frecuencia de muestreo lógica = polling rate del dispositivo (p. ej. 125/500/1000 Hz). Para compatibilidad con reproductores, también te ofreceré opción de upsample a 48 kHz con anti-aliasing.

Requerimientos mínimos:

Windows 10/11 + WSL2.

Docker Desktop con integración WSL (o Docker en WSL).

usbipd-win (o similar) para reenviar el dispositivo USB desde Windows a WSL si quieres acceder al ratón físico desde WSL.

En WSL: evtest o libinput-tools para identificar /dev/input/eventX.

Privilegios para leer /dev/input/event* (sudo/udev/privileged container).

2) Pasos en Windows → exponer el ratón a WSL

(Opciones: si tu ratón aparece ya como /dev/input/eventX en WSL, saltarte esta parte.)

Instala y habilita WSL2 y Docker Desktop con integración WSL.

Instala usbipd-win en Windows (herramienta Microsoft para USB passthrough).

Desde PowerShell (admin), lista dispositivos USB:

usbipd list

Localiza el busid del ratón.

Adjunta el dispositivo a tu distro WSL:

usbipd wsl attach --busid <BUSID> --distribution <YourDistro>

En WSL, comprueba que aparece:

ls /dev/input
sudo evtest          # o sudo libinput debug-events

Identifica el /dev/input/eventX correcto.

Nota: si no quieres usbipd, otra vía es leer eventos a nivel Windows (Raw Input) y pasarlos al contenedor vía socket; pero para la PoC evdev + WSL es lo más directo.

3) Contenedor Docker: enfoque y permisos

Imagen base: Debian/Ubuntu con g++ y cmake.

Paquetes: libc6-dev, build-essential. No necesitamos librería externa para WAV (escribiremos header WAV simple).

Cuando lances el contenedor, monta /dev/input y un volumen de salida:

docker run --rm -it \
  --device /dev/input/event3:/dev/input/event3 \
  -v /home/user/output:/out \
  --privileged \
  mouse2audio:latest /dev/input/event3 1000 /out/output.wav

--privileged simplifica permisos; puedes refinar con --cap-add/udev rules si prefieres menor privilegio.

Alternativa (más segura): ajustar reglas udev para permitir lectura sin privilegios.

4) Medir el polling efectivo (recomendado antes de generar WAV)

Antes de escribir audio, mide la tasa real de eventos. Pequeño script (bash + evtest / lectura timestamp) o la misma app C++ puede calcular media de eventos por segundo mientras mueves el ratón rápido. Valores típicos: 125, 500, 1000 Hz; pero mejor medir.

5) Pipeline conceptual (datos → audio)

El proceso lee eventos REL_X / REL_Y con timestamp (evdev).

Agrupa/acu mula deltas que caen dentro de cada “tick” de la frecuencia objetivo (ej. 1/1000 s).

Para cada tick produce un sample (mono o estéreo): mapear dx → amplitude o dx→freq para síntesis.

Si necesitas WAV con fs igual al polling (p.ej. 1000 Hz), crea WAV con ese sample rate. Opcionalmente resamplea a 48k (upsample + LPF) para reproducir en players estándar.

Escribe WAV en disco en bloques (streaming) para no usar mucha RAM.

6) Diseño de la app C++ (PoC)

Entrada: path a /dev/input/eventX, polling_rate objetivo (Hz), path salida WAV.

Módulos:

Reader (thread): lee input_event y empuja {time, dx, dy} a un ring buffer thread-safe.

Renderer (thread principal): en un bucle a intervalos 1/polling_rate consume los eventos acumulados hasta ese tiempo y genera un sample por canal.

WAV writer: escribe header WAV y va appending frames (float32 o int16).

Opcional: resampler simple (hold o linear) si se pide salida a 48k.

Mapeos sugeridos:

Simple: sample = gain * dx (range limit con tanh) → estéreo (dx->L, dy->R).

Synthesis: conversion dx->frequency of oscillator and generate sine for that tick (mejor musicalidad).

Buffering: escribe en bloques de N samples (ej. 1024) para eficiencia.

Te adjunto un esqueleto simplificado (resumen, no todo el código) y luego Dockerfile + comandos.

7) Código PoC (esqueleto) — comportamiento: evdev → WAV (fs = polling_rate)
// PoC: mouse2wav.cpp (resumen)
//  - build con: g++ mouse2wav.cpp -o mouse2wav -pthread
//  - usa read() de /dev/input/eventX, escribe WAV PCM 16-bit (mono o stereo)

#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>
#include <vector>
// ... (helpers) ...

// WAV header writer, ring buffer, reader thread, renderer loop:
// - reader: push events {t, dx, dy}
// - renderer: for each tick of duration 1/pollHz accumulate dx/dy and generate sample
// - writer: write samples as int16 little endian

// Idea: if pollHz=1000 -> sample period 1ms

(Te lo puedo pegar completo si quieres; prefiero darlo completo en el siguiente mensaje para no hacerlo demasiado largo aquí.)

8) Dockerfile (ejemplo)
FROM ubuntu:24.04
RUN apt-get update && apt-get install -y build-essential cmake git libssl-dev
WORKDIR /app
COPY . /app
RUN g++ -std=c++17 mouse2wav.cpp -o /usr/local/bin/mouse2wav -pthread
ENTRYPOINT ["/usr/local/bin/mouse2wav"]

Build:

docker build -t mouse2audio:latest .

Run (desde WSL, después de exponer el dispositivo):

docker run --rm -it \
  --device /dev/input/event3:/dev/input/event3 \
  -v $(pwd)/out:/out \
  --privileged \
  mouse2audio:latest /dev/input/event3 1000 /out/output.wav
9) Opciones de salida y reproducción

WAV con fs = polling_rate: te da representación “fiel” temporally. Reproductores aceptan fs arbitrario, pero la señal sonar átono/extraña por baja tasa.

Upsample a 48k: toma cada muestra “sensor” y genera 48k/pollHz muestras por sensor-sample (hold o interpolation) + filtrado LPF para evitar aliasing. Resultado reproducible con fidelidad en audio.

Stereo: mapear dx a canal L y dy a R o usar dx -> amplitude, |dx| -> timbre.

10) Pruebas y validación

Ejecuta con pollHz estimado (125/500/1000) y mueve el ratón con movimientos controlados.

Abre WAV en Audacity/DAW y mira el espectrograma y la tasa efectiva de eventos.

Ajusta gain, filtros y mapping a gusto.

Si necesitas más resolución temporal, considera el modo hardware: leer sensor vía SPI desde un MCU (Teensy/STM32) y transmitir muestras al host vía USB (serial/HID) a alta frecuencia. En ese caso el contenedor leería desde /dev/ttyACM0 o similar.

11) Limitaciones y notas finales

Si sólo lees evdev en el host (sin modificar firmware), el máximo práctico estará limitado por el polling rate del dispositivo o del host (usualmente ≤ 1000 Hz).

Leer el sensor interno vía SPI/MCU te da verdaderas kHz–tens de kHz, pero implica hardware adicional.

Para PoC y facilidad de uso: la ruta evdev → Docker → WAV es la más simple y reproducible en Windows+WSL.