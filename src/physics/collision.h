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
#include "broadphase.h"

/* Altura máxima de escalón para step-up (5a-3).
 * Debe coincidir con SV_STEPSIZE en pmove.h (18 u).
 * Definido aquí para evitar dependencia circular con pmove.h. */
#define SV_STEPSIZE 18.0f

/* Alturas de agachado (definidas aquí para que collision.c pueda usarlas
 * sin incluir pmove.h y crear dependencia circular). */
#define CROUCH_HEIGHT_STAND  72.0f
#define CROUCH_HEIGHT_DUCK   36.0f

/* Radio del jugador (cápsula aproximada) para el bloqueo de paredes */
#define RADIO_JUGADOR 16.0f

/* Altura de la cápsula (sin hemisferios) = crouch_height - 2*RADIO_JUGADOR.
 * Para pie: 72 - 32 = 40 u. Para agachado: 36 - 32 = 4 u (degenerado, usa esfera). */
#define CAPSULE_MIN_HEIGHT 4.0f

/* Una cara es "pared" si su normal es casi horizontal (|normal.y| < esto) */
#define PARED_NORMAL_Y_MAX 0.3f

/* Umbral para considerar suelo (normal.y > esto) */
#define SUELO_NORMAL_Y_MIN 0.7f

/* Umbral para considerar techo (normal.y < -esto) */
#define TECHO_NORMAL_Y_MAX -0.7f

/* Distancia extra para check de techo (margen de seguridad) */
#define CEILING_CHECK_DIST 4.0f

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

/* ── Swept Capsule Collision ──────────────────────────────────────────────── *
 * Nueva API: barrido de cápsula (inicio→fin) contra triángulos del mapa.
 * Elimina tunneling, clipping en esquinas, y hace step-up robusto.
 * Usa broadphase para O(1) por tick en mapas grandes.
 * ────────────────────────────────────────────────────────────────────────── */

/* Resultado de barrido de cápsula */
typedef struct {
    bool hit;           /* true si hubo colisión */
    f32  t;             /* parámetro de impacto [0, 1] (0 = inicio, 1 = fin) */
    Vec3 normal;        /* normal de contacto (apunta fuera de la geometría)  */
    Vec3 pos;           /* posición del centro de la cápsula en el impacto     */
} SweepHit;

/* Barrido de cápsula desde 'start' hasta 'end' contra el mapa.
 * radius: radio de la cápsula (RADIO_JUGADOR = 16)
 * half_height: semi-altura del cilindro central (crouch_height/2 - radius).
 *   Para pie: 72/2 - 16 = 20. Para agachado: max(36/2 - 16, 2) = 2.
 * broadphase: puntero al broadphase construido (puede ser NULL → fallback O(N)).
 * ignore_floor_ceiling: si true, ignora triángulos con normal.y > 0.7 o < -0.7
 *   (útil para bloqueo de paredes: evita choques falsos con suelo/techo).
 * Retorna el primer hit (menor t). Si no hay hit, hit=false.
 * Sin malloc, usa broadphase para limitar triángulos testeados. */
SweepHit collision_sweep_capsule(const Map *map, const Broadphase *bp,
                                  Vec3 start, Vec3 end,
                                  f32 radius, f32 half_height,
                                  bool ignore_floor_ceiling);

/* ── API legacy (mantenida para compatibilidad, redirige a swept) ─────────── */

/* Lanza un rayo hacia abajo desde 'pos' y busca la superficie de suelo más
 * cercana dentro del rango permitido según crouch_height.
 * Ahora usa broadphase + swept capsule (radio 0.1, half_height 0) para robustez.
 * crouch_height: altura actual de la caja del jugador (36 agachado, 72 de pie,
 * o valor interpolado durante transición).
 * Seguro llamarlo cada tick: sin malloc, O(celdas broadphase). */
FloorHit collision_detectar_suelo(const Map *map, const Broadphase *bp, Vec3 pos, f32 crouch_height);

/* Bloqueo de paredes con slide usando swept capsule.
 * Barrido desde start_pos hasta start_pos + vel*dt con cápsula del jugador.
 * Desliza proyectando velocidad restante en el plano de contacto (iterativo, máx 2 pasadas).
 * broadphase: para O(1) en mapas grandes.
 * floor_normal: normal del suelo actual (para evitar corrección de rampa en la superficie
 *   sobre la que ya está el jugador). Pasar {0,1,0} si no se conoce.
 * Escribe la posición final en out_pos y actualiza vel (velocidad real tras slide).
 * Retorna true si se corrigió algo. */
bool collision_bloquear_pared(const Map *map, const Broadphase *bp, Vec3 start_pos, Vec3 *vel, f32 dt, f32 crouch_height, Vec3 floor_normal, Vec3 *out_pos);

/* Step-up robusto: barrido de cápsula hacia adelante y arriba.
 * Intenta mover la cápsula en dirección 'move_dir' (XZ normalizada) por STEP_UP_AHEAD_DIST,
 * y simultáneamente hacia arriba por SV_STEPSIZE. Si choca con suelo en el rango [0.5, 18],
 * sube la posición.
 * Usa broadphase. Retorna true si subió. */
bool collision_intentar_step_up(const Map *map, const Broadphase *bp, Vec3 *pos, const Vec3 *move_dir,
                                 bool en_suelo_antes, f32 crouch_height);

/* Comprobación de techo para agachado: barrido de cápsula hacia arriba.
 * Retorna true si hay techo a distancia < max_dist. */
bool collision_check_ceiling(const Map *map, const Broadphase *bp, Vec3 pos, f32 crouch_height, f32 max_dist);

/* Predictive ramp detection using swept capsule (more reliable than raycast
 * for detecting ramps at lookahead positions).
 * Sweeps capsule DOWN from HIGH above (20u) at lookahead position.
 * Returns true if ramp detected, fills out_normal with ramp normal,
 * and out_hit_y with floor y at lookahead position. */
bool collision_detectar_rampa_predictiva(const Map *map, const Broadphase *bp,
                                          Vec3 pos, Vec3 wish_dir, f32 lookahead,
                                          f32 crouch_height, Vec3 *out_normal, f32 *out_hit_y);

#endif /* OPENSTRIKE_COLLISION_H */