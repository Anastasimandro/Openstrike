// Copyright (c) 2026 OpenStrike Project
// collision.h is part of OpenStrike.
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

#ifndef OPENSTRIKE_COLLISION_H
#define OPENSTRIKE_COLLISION_H

#include "types.h"
#include "renderer/math.h"
#include "map/map.h"

/* ────────────────────────────────────────────────────────────────────────── *
 * Resultado de una consulta de suelo.                                        *
 * ────────────────────────────────────────────────────────────────────────── */
typedef struct {
    bool hit;      /* true si hay geometría de suelo bajo el jugador         */
    f32  y;        /* altura del suelo — posición Y donde apoyar los pies    */
    Vec3 normal;   /* normal de la superficie golpeada (apunta hacia arriba)  */
} FloorHit;

/* Lanza un rayo hacia abajo desde 'pos' y busca la superficie de suelo más
 * cercana dentro de SV_STEPSIZE + 2 unidades.
 * Solo detecta caras cuya normal apunta hacia arriba (Y > 0) — ignora techos
 * y paredes.
 * Seguro llamarlo cada tick: sin malloc, O(vértices del mapa). */
FloorHit collision_detectar_suelo(const Map *map, Vec3 pos);

/* Bloqueo de paredes con slide (5a-2). Tras integrar la posición del tick,
 * lanza un rayo horizontal desde (pos.x, pos.y+36, pos.z) en la dirección
 * XZ de la velocidad. Si golpea una cara casi vertical (|normal.y| < 0.3)
 * más cerca que el avance de este tick + radio 16:
 *   1. recorta pos para dejar 16 u de distancia a la pared,
 *   2. proyecta vel sobre el plano de la pared (vel -= n * dot(vel, n))
 *      → el jugador se resbala a lo largo de ella.
 * Solo actúa si el movimiento va HACIA la pared (dot(vel, normal) < 0).
 * Retorna true si se recortó el movimiento. Sin malloc, O(vértices). */
bool collision_bloquear_pared(const Map *map, Vec3 *pos, Vec3 *vel, f32 dt);

#endif /* OPENSTRIKE_COLLISION_H */
