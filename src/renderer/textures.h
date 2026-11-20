// Copyright (c) 2026 OpenStrike Project
// textures.h is part of OpenStrike.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef OPENSTRIKE_TEXTURES_H
#define OPENSTRIKE_TEXTURES_H

#include "types.h"

/* Máximo de materiales/texturas simultáneos (límite fijo por hardware 310M) */
#define TEXTURAS_MAX_MATERIALES 64

/* Índice especial: textura de fallback (checkerboard) */
#define TEXTURA_FALLBACK_IDX 0

/* Inicializa el gestor de texturas.
   Debe llamarse DESPUÉS de renderer_init() (contexto GL activo).
   Retorna 1 OK, 0 fallo. */
int texturas_init(void);

/* Carga todas las texturas referenciadas por el mapa.
   - map: puntero al mapa ya cargado (para leer MapFace.textura)
   - ruta_base: carpeta base de assets (ej. "assets/textures/")
   Retorna 1 OK, 0 fallo (usa fallback para las que falten). */
int texturas_cargar_mapa(const void *map, const char *ruta_base);

/* Selecciona la textura activa en la unidad 0 para dibujar.
   idx: índice devuelto por texturas_cargar_mapa (0 = fallback). */
void texturas_usar(int idx);

/* Libera todas las texturas GL y buffers CPU. */
void texturas_shutdown(void);

/* Obtiene el índice de material para un nombre de textura.
   Retorna índice >= 1, o TEXTURA_FALLBACK_IDX (0) si no existe. */
int texturas_indice_por_nombre(const char *nombre);

#endif /* OPENSTRIKE_TEXTURES_H */