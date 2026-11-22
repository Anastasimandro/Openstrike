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

/* Altura máxima de escalón para step-up (5a-3).
 * 18 u — misma que usa el snap de rampas en pmove.c.
 * Definido aquí para evitar dependencia circular con pmove.h. */
#define SV_STEPSIZE 18.0f

/* Radio del jugador (cápsula aproximada) para el bloqueo de paredes */
#define RADIO_JUGADOR 16.0f

/* Una cara es "pared" si su normal es casi horizontal (|normal.y| < esto) */
#define PARED_NORMAL_Y_MAX 0.3f

/* ────────────────────────────────────────────────────────────────────────── *
 * Möller–Trumbore: intersección rayo–triángulo (pública para raycast de balas,
 * comprobación de techo, etc.).                                            *
 * ────────────────────────────────────────────────────────────────────────── */
/* Acepta triángulos de CUALQUIER orientación. El filtrado por orientación
 * lo hace cada llamada según su caso (suelo: normal.y > 0; pared: |normal.y| < 0.3;
 * techo: normal.y < -0.7).
 * Escribe la distancia t en *t_out y la normal normalizada en *normal_out
 * (si no es NULL). t > 0 siempre al retornar true. */
bool collision_ray_triangulo(Vec3 orig, Vec3 dir,
                              Vec3 v0, Vec3 v1, Vec3 v2,
                              f32 *t_out, Vec3 *normal_out);

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
 * (|normal.y| < 0.3) con rayos a varias alturas de la caja del jugador
 * (pies, mitad y cabeza) en dirección opuesta a la normal de la cara.
 * Usar varias alturas evita atravesar bloques altos: un solo rayo a la
 * altura de la cintura pasaba por encima de cajas de 32 u.
 * Si estamos a menos de RADIO_JUGADOR (16 u) y nos movemos hacia la cara
 * (dot(vel, normal) < 0):
 *   1. empuja pos perpendicularmente hasta dejar 16 u,
 *   2. proyecta vel sobre el plano de la pared (vel -= n * dot(vel, n))
 *      → el jugador se resbala a lo largo de ella.
 * En esquinas cada pared se corrige por separado (dos empujes ortogonales).
 * crouch_height: altura actual de la caja para las alturas de muestreo.
 * Retorna true si se corrigió algo. Sin malloc, O(vértices del mapa). */
bool collision_bloquear_pared(const Map *map, Vec3 *pos, Vec3 *vel, f32 dt, f32 crouch_height);

/* Step-up (5a-3): intenta subir un escalón de hasta SV_STEPSIZE (18 u).
 * Llamar tras collision_bloquear_pared (solo si devolvió true) y ANTES de
 * collision_detectar_suelo.
 * Si el jugador estaba en suelo y se mueve en XZ, lanza un rayo hacia abajo
 * desde (pos + wish_dir * 20 + (0, SV_STEPSIZE + 2, 0)).
 * Solo acepta superficies planas (normal.y > 0.95): las rampas se resuelven
 * con el snap al suelo, no con step-up (evita saltos al subir pendientes).
 * Si hay suelo a <= SV_STEPSIZE por encima de pos.y → eleva pos.y a ese suelo
 * y avanza al punto sondeado (pos + wish_dir * 20), que queda sobre la
 * superficie del escalón (si no, el centro seguiría sobre el suelo bajo y el
 * snap lo devolvería).
 * wish_dir es la dirección de movimiento normalizada en XZ.
 * Retorna true si subió un escalón. */
bool collision_intentar_step_up(const Map *map, Vec3 *pos, const Vec3 *wish_dir,
                                 bool en_suelo_antes, f32 crouch_height);

#endif /* OPENSTRIKE_COLLISION_H */
