# OpenStrike

Shooter táctico multijugador de código abierto que replica el feeling de CS:GO
con gráficos entre CS 1.6 y CS:Source. Diseñado para correr en hardware de 2010
(GeForce 310M, 4 GB RAM) sin instalación, sin cuenta y sin telemetría.

- **Lenguaje:** C99
- **Renderer:** OpenGL 2.1 (GLSL 1.20)
- **Ventana/Input:** SDL2
- **Licencia:** GPL v3
- **Assets:** propios — sin contenido de Valve

## Estado actual

| Sistema | Estado |
|---|---|
| Ventana + loop 64 Hz + input | ✅ |
| Renderer básico (VBO, lightmaps vertex-bakeados) | ✅ |
| Física pmove (constantes CS:GO, bunny hop nerfeado) | ✅ |
| Mapa JSON (geometría, zonas, spawns, waypoints A*) | ✅ |
| Colisión de suelo (Möller–Trumbore) | ✅ |
| Paredes / step-up / rampas | 🔜 |
| Audio, HUD, armas, rondas, netcode, bots | ❌ planificado |

## Compilar

```bash
# Linux
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build

# Windows (cross-compile desde Linux)
cmake -B build-win -DCMAKE_TOOLCHAIN_FILE=cmake/mingw-w64.cmake -DCMAKE_BUILD_TYPE=Release && cmake --build build-win
```

El binario debe ejecutarse desde la raíz del repo (carga `assets/` con rutas
relativas). En Windows, copiar `SDL2.dll` desde `deps/windows/SDL2/x86_64-w64-mingw32/bin/`.

El CI compila automáticamente Linux y Windows en cada push a `main` y sube los
ejecutables como artefactos de la run de Actions.
