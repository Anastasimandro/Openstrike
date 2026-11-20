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
// MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef OPENSTRIKE_COLLISION_H
#define OPENSTRIKE_COLLISION_H

#include "types.h"
#include "renderer/math.h"
#include "map/map.h"

/* Altura máxima de escalón para step-up (5a-3).
 * Debe coincidir con SV_STEPSIZE en pmove.h (18 u).
 * Definido aquí para evitar dependencia circular con pmove.h. */
#define SV_STEPSIZE 18.0f

/* Radio del jugador (cápsula aproximada) para el bloqueo de paredes */
#define RADIO_JUGADOR 16.0f

/* Una cara es "pared" si su normal es casi horizontal (|normal.y| < esto) */
#define PARED_NORMAL_Y_MAX 0.3f

/* ────────────────────────────────────────────────────────────────────────── *
 * Resultado de una consulta de suelo.                                        *
 * ────────────────────────────────────────────────────────────────────────── */
typedef struct {
    bool hit;      /* true si hay geometría de suelo bajo el jugador         */
    f32  y;        /* altura del suelo — posición Y donde apoyar los pies    */
    Vec3 normal;   /* normal de la superficie golpeada (apunta hacia arriba)  */
} FloorHit;

/* Lanza un rayo hacia abajo desde 'pos' y busca la superficie de suelo más
 * cercana dentro del rango permitido según crouch_height.
 * Solo detecta caras cuya normal apunta hacia arriba (Y > 0) — ignora techos
 * y paredes.
 * crouch_height: altura actual de la caja del jugador (36 agachado, 72 de pie,
 * o valor interpolado durante transición).
 * Seguro llamarlo cada tick: sin malloc, O(vértices del mapa). */
FloorHit collision_detectar_suelo(const Map *map, Vec3 pos, f32 crouch_height);

/* Bloqueo de paredes con slide (5a-2). Llamar tras integrar la posición del
 * tick. Mide la distancia perpendicular real a cada cara casi vertical
 * (|normal.y| < 0.3) con un rayo desde (pos.x, pos.y + crouch_height/2, pos.z)
 * en dirección opuesta a la normal de la cara — independiente del ángulo de la
 * velocidad.
 * Si estamos a menos de RADIO_JUGADOR (16 u) y nos movemos hacia la cara
 * (dot(vel, normal) < 0):
 *   1. empuja pos perpendicularmente hasta dejar 16 u,
 *   2. proyecta vel sobre el plano de la pared (vel -= n * dot(vel, n))
 *      → el jugador se resbala a lo largo de ella.
 * En esquinas cada pared se corrige por separado (dos empujes ortogonales).
 * crouch_height: altura actual de la caja para origen correcto del rayo.
 * Retorna true si se corrigió algo. Sin malloc, O(vértices del mapa). */
bool collision_bloquear_pared(const Map *map, Vec3 *pos, Vec3 *vel, f32 dt, f32 crouch_height);

/* Step-up (5a-3): intenta subir un escalón de hasta SV_STEPSIZE (18 u).
 * Llamar tras collision_bloquear_pared y ANTES de collision_detectar_suelo.
 * Si el jugador estaba en suelo y se mueve en XZ, lanza un rayo hacia abajo
 * desde (pos + wish_dir * 20 + (0, crouch_height/2 + SV_STEPSIZE, 0)).
 * Si hay suelo a <= SV_STEPSIZE por encima de pos.y → eleva pos.y a ese suelo.
 * wish_dir es la dirección de movimiento normalizada en XZ (puede venir de
 * input.wish_dir o derivarse de vel). crouch_height para origen correcto.
 * Retorna true si subió un escalón. */
bool collision_intentar_step_up(const Map *map, Vec3 *pos, const Vec3 *wish_dir,
                                 bool en_suelo_antes, f32 crouch_height);

#endif /* OPENSTRIKE_COLLISION_H */