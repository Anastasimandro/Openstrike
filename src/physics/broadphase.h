// Copyright (c) 2026 OpenStrike Project
// broadphase.h is part of OpenStrike.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef OPENSTRIKE_BROADPHASE_H
#define OPENSTRIKE_BROADPHASE_H

#include "types.h"
#include "renderer/math.h"
#include "map/map.h"

/* ── Configuración del grid 3D uniforme ───────────────────────────────────── *
 * Mundo objetivo: mapas CS estilo 512×512×192 unidades (sala_prueba).
 * Grid: 32 × 16 × 32 celdas = 16384 celdas totales.
 * Tamaño celda: 16 u en X/Z, 12 u en Y → cubre 512×192×512 exacto.
 * Cada cara del mapa se registra en las celdas que solapa su AABB.
 * Consulta: dadas min/max de la query, se itera solo celdas relevantes.       *
 * ─────────────────────────────────────────────────────────────────────────── */

#define BP_GRID_X 32
#define BP_GRID_Y 16
#define BP_GRID_Z 32
#define BP_CELL_SIZE_X 16.0f
#define BP_CELL_SIZE_Y 12.0f
#define BP_CELL_SIZE_Z 16.0f
#define BP_WORLD_MIN_X -256.0f
#define BP_WORLD_MIN_Y 0.0f
#define BP_WORLD_MIN_Z -256.0f
#define BP_WORLD_MAX_X 256.0f
#define BP_WORLD_MAX_Y 192.0f
#define BP_WORLD_MAX_Z 256.0f
#define BP_TOTAL_CELLS (BP_GRID_X * BP_GRID_Y * BP_GRID_Z)

/* Máximo de caras por celda (ajustar si mapas mayores lo exigen).
 * sala_prueba: ~20 caras, bien distribuidas → < 10 por celda.
 * Inferno estimado: ~50k caras / 16384 celdas ≈ 3-4 por celda promedio. */
#define BP_MAX_FACES_PER_CELL 64

/* Índice lineal de celda: (cx * BP_GRID_Y + cy) * BP_GRID_Z + cz */
typedef struct {
    u16 face_indices[BP_MAX_FACES_PER_CELL];
    u16 count;
} BroadphaseCell;

typedef struct {
    BroadphaseCell cells[BP_TOTAL_CELLS];
    bool built;
    Vec3 world_min;
    Vec3 world_max;
} Broadphase;

/* ── API pública ──────────────────────────────────────────────────────────── */

/* Construye el broadphase desde el mapa. Llamar UNA vez tras map_cargar().
 * Itera todas las caras, calcula AABB de cada triángulo, registra en celdas.
 * Sin malloc. O(face_count) preprocessing. */
void broadphase_build(Broadphase *bp, const Map *map);

/* Consulta caras que solapan un AABB [mins, maxs].
 * Llena 'out_indices' (array provisto por caller, tamaño max_out).
 * Retorna número de índices escritos (≤ max_out).
 * Si bp no está built o mapa vacío, retorna 0. */
u32 broadphase_query_aabb(const Broadphase *bp, Vec3 mins, Vec3 maxs,
                           u16 *out_indices, u32 max_out);

/* Consulta caras que intersectan un segmento [a, b] (para raycast/swept).
 * Usa DDA 3D (Amanatides-Woo) sobre el grid.
 * Llena 'out_indices' (array caller, tamaño max_out).
 * Retorna número de índices únicos escritos (≤ max_out). */
u32 broadphase_query_segment(const Broadphase *bp, Vec3 a, Vec3 b,
                              u16 *out_indices, u32 max_out);

/* Consulta simple: caras en la celda que contiene 'pos' (para colisión puntual).
 * Retorna count ≤ BP_MAX_FACES_PER_CELL, índices en out_indices. */
u32 broadphase_query_point(const Broadphase *bp, Vec3 pos,
                            u16 *out_indices, u32 max_out);

/* Debug: imprime estadísticas de ocupación del grid. */
void broadphase_debug_stats(const Broadphase *bp);

#endif /* OPENSTRIKE_BROADPHASE_H */