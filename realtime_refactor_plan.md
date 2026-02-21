# Plan de Refactorización: Arquitectura de Tiempo Real (v1.3.1)

## Estado Actual y Diagnóstico
En las pruebas recientes con el G203 (1000Hz), se observa que:
1. El polling rate cae de 1000Hz a 125Hz en cuanto hay movimiento intenso.
2. Aparece `BACKPRESSURE` en la cola PCM incluso sin el filtro Wiener activo.
3. El audio resultante puede sufrir micro-cortes si el hilo de escritura (WAV) no despierta a tiempo.

### Causas Probables:
- **Timer Resolution**: Por defecto, `Sleep(1)` en Windows puede tardar hasta 15.6ms. A 16kHz, procesamos bloques de 256 muestras cada 16ms. Si el hilo duerme de más, la cola se llena.
- **Thread Priority**: Windows puede estar bajando la prioridad de los hilos de captura y DSP al considerarlos "consumidores de fondo", lo que causa que Windows agrupe eventos USB (batching) antes de entregarlos (causando el drop a 125Hz/8ms).

---

## Estrategia de Mejora (Próximos Pasos)

### 1. Sistema de "Reloj Maestro" y Prioridades
- Implementar `timeBeginPeriod(1)` al inicio de la app para forzar la resolución del scheduler a 1ms.
- Elevar la prioridad del hilo de captura (`RawInput`) a `THREAD_PRIORITY_TIME_CRITICAL`.
- Elevar la prioridad del hilo DSP a `THREAD_PRIORITY_HIGHEST`.

### 2. Refinamiento de Diagnóstico (v1.3.1)
- **Contador de Drops**: Añadir `g_events_dropped` para saber si perdemos eventos por saturación de cola.
- **Histograma de Latencia**: Medir el tiempo real que tarda el callback de RawInput para confirmar que el procesado es < 10µs.
- **Backpressure Inteligente**: Cambiar el log de `BACKPRESSURE` para que solo avise si la cola supera el 50% de su capacidad, evitando falsos positivos por buffering normal.

### 3. Loop de Escritura No-Bloqueante
- Optimizar el `writer_thread` para que drene múltiples bloques por cada ciclo de despertar, evitando el cuello de botella del I/O de disco.

---

## Verificación
- **Estabilidad del Poll**: El valor debe mantenerse en ~1000 Hz durante el movimiento.
- **Ausencia de Drops**: `events_dropped` debe ser 0.
- **Análisis SNR**: Con el filtro Wiener activo y la arquitectura estable, el ruido de fondo del sensor debe ser virtualmente indetectable en el archivo WAV resultante.
