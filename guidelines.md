# Guías y Buenas Prácticas de Desarrollo

## Estándares de Código (C++)

- **Versión**: C++17 como base para estabilidad y soporte en contenedores.
- **Bibliotecas**: Priorizar la `std` (standard library). No añadir dependencias externas sin una revisión de arquitectura.
- **Tipos de Datos**: Usar tipos de ancho fijo de `<cstdint>` (`int16_t`, `uint32_t`, etc.) para garantizar portabilidad en estructuras de archivos (WAV) y protocolos.
- **Memory Management**: Evitar `new`/`delete` manual. Usar `std::unique_ptr` o contenedores de la STL. Alocación de memoria dinámica prohibida dentro de los bucles críticos de lectura y renderizado (pre-alocación).
- **Concurrency**: Ring buffer SPSC lock-free para comunicación entre hilos. Evitar `std::mutex` en el camino crítico del audio si es posible.

## Estructura de Proyecto

```text
Mouseofono/
├── src/           # Implementación C++
│   ├── core/      # Lógica de audio y procesamiento
│   ├── driver/    # Interfaz con evdev
│   └── util/      # Helpers de archivos y threads
├── docker/        # Dockerfile y scripts relacionados
├── scripts/       # Utilidades de shell (calibración, setup)
├── tests/         # Tests unitarios y de integración
├── CMakeLists.txt # Sistema de construcción
└── README.md
```

## Convenciones de Naming

- **Archivos**: `snake_case.cpp`, `snake_case.h`.
- **Clases/Structs**: `PascalCase`.
- **Variables/Funciones**: `snake_case`.
- **Constantes/Macros**: `SCREAMING_SNAKE_CASE`.

## Error Handling y Logging

- **No fallar silenciosamente**: Todo error crítico (apertura de device fallida, falta de memoria) debe imprimir a `stderr` y retornar un código de error no-cero.
- **Signals**: Atrapar `SIGINT` para asegurar la integridad de los archivos de salida.
- **Métricas**: Loguear el polling rate medido al inicio para confirmación del usuario.

## Control de Versiones (Git)

- **Mensajes**: Usar el estándar de Conventional Commits (ej. `feat: add ring buffer implementation`, `fix: header patching on shutdown`).
- **Commits**: Mantener los commits atómicos y enfocados en una sola responsabilidad.
