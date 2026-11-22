// Copyright (c) 2026 OpenStrike Project
// collision.c is part of OpenStrike.
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

#include "collision.h"
#include <math.h>
#include <string.h>

/* ── Constantes internas ─────────────────────────────────────────────────── */

/* Distancia adelante del step-up (un poco más que el radio para que el
 * rayo salga del borde del escalón). Valor fijo, no configurable. */
#define STEP_UP_AHEAD_DIST 20.0f

/* Margen vertical del rayo de step-up sobre la cabeza del escalón. */
#define STEP_UP_RAY_MARGIN 2.0f

/* Un escalón es "plano" si su normal es casi vertical. Las rampas
 * (normal.y menor) no se suben con step-up: se resuelven con el snap. */
#define STEP_UP_NORMAL_Y_MIN 0.95f

/* ── Möller–Trumbore: intersección rayo–triángulo ────────────────────────── *
 * Generalizada: acepta triángulos de CUALQUIER orientación — suelo, pared o
 * techo. El filtrado por orientación lo hace cada llamada según su caso
 * (suelo: normal.y > 0; pared: |normal.y| < 0.3; techo: normal.y < -0.7).
 * Escribe la distancia t en *t_out y la normal normalizada en *normal_out
 * (si no es NULL). t > 0 siempre al retornar true.                          */
bool collision_ray_triangulo(Vec3 orig, Vec3 dir,
                              Vec3 v0, Vec3 v1, Vec3 v2,
                              f32 *t_out, Vec3 *normal_out)
{
    Vec3 e1 = vec3_sub(v1, v0);
    Vec3 e2 = vec3_sub(v2, v0);

    /* Test de paralelismo */
    Vec3 h = vec3_cross(dir, e2);
    f32  a = vec3_dot(e1, h);
    if (a > -1e-6f && a < 1e-6f) { return false; }

    f32  f = 1.0f / a;
    Vec3 s = vec3_sub(orig, v0);
    f32  u = f * vec3_dot(s, h);
    if (u < 0.0f || u > 1.0f) { return false; }

    Vec3 q = vec3_cross(s, e1);
    f32  v = f * vec3_dot(dir, q);
    if (v < 0.0f || u + v > 1.0f) { return false; }

    f32 t = f * vec3_dot(e2, q);
    if (t < 1e-6f) { return false; }   /* intersección detrás del origen */

    *t_out = t;
    if (normal_out) {
        *normal_out = vec3_norm(vec3_cross(e1, e2));
    }
    return true;
}

/* ── Detección de suelo ──────────────────────────────────────────────────── *
 * Un único rayo vertical bajo el centro del jugador. Es suficiente porque el
 * snap se hace cada tick: no hacen falta rayos laterales (que eran los que
 * hacían trepar al jugador por las paredes de los bloques).                 */
FloorHit collision_detectar_suelo(const Map *map, Vec3 pos, f32 crouch_height)
{
    FloorHit result;
    memset(&result, 0, sizeof(result));
    result.hit = false;

    if (!map || !map->cargado || map->face_count == 0) {
        return result;
    }

    /* Origen a media altura de la caja: queda dentro del volumen del jugador,
     * así que no puede enganchar una superficie que esté por encima de él. */
    f32 ray_orig_offset = crouch_height * 0.5f;
    /* Alcance: desde el origen hasta los pies + step + margen. */
    f32 suelo_max_dist = ray_orig_offset + SV_STEPSIZE + 2.0f;

    Vec3 orig = { pos.x, pos.y + ray_orig_offset, pos.z };
    Vec3 dir  = { 0.0f, -1.0f, 0.0f };

    f32 best_t = suelo_max_dist;

    for (i32 fi = 0; fi < map->face_count; fi++) {
        const MapFace *face  = &map->faces[fi];
        u32            base  = face->vertex_start;
        u32            count = face->vertex_count;

        /* Cada 3 vértices = 1 triángulo (el formato garantiza múltiplo de 3) */
        for (u32 ti = 0; ti + 2 < count; ti += 3) {
            const MapVertex *va = &map->vertices[base + ti + 0];
            const MapVertex *vb = &map->vertices[base + ti + 1];
            const MapVertex *vc = &map->vertices[base + ti + 2];

            Vec3 v0 = { va->pos[0], va->pos[1], va->pos[2] };
            Vec3 v1 = { vb->pos[0], vb->pos[1], vb->pos[2] };
            Vec3 v2 = { vc->pos[0], vc->pos[1], vc->pos[2] };

            f32 t;
            Vec3 normal;
            if (collision_ray_triangulo(orig, dir, v0, v1, v2, &t, &normal)) {
                /* Solo caras de suelo: la normal debe apuntar hacia arriba. */
                if (normal.y > 0.0f && t < best_t) {
                    best_t      = t;
                    result.hit  = true;
                    /* Y del suelo = Y del origen - distancia recorrida */
                    result.y      = orig.y - t;
                    result.normal = normal;
                }
            }
        }
    }

    return result;
}

/* ── Bloqueo de paredes con slide ────────────────────────────────────────── *
 * Rayos perpendiculares a cada cara vertical, a tres alturas de la caja.
 * Empuja al jugador fuera de la pared y elimina la componente de velocidad
 * que entra en ella.                                                        */
bool collision_bloquear_pared(const Map *map, Vec3 *pos, Vec3 *vel, f32 dt, f32 crouch_height)
{
    (void)dt;  /* la posición ya viene integrada; dt se mantiene por API */

    if (!map || !map->cargado || map->face_count == 0) { return false; }

    f32 xz = sqrtf(vel->x * vel->x + vel->z * vel->z);
    if (xz < 0.001f) { return false; }   /* parado: nada que bloquear */

    bool corregido = false;

    for (i32 fi = 0; fi < map->face_count; fi++) {
        const MapFace *face  = &map->faces[fi];
        u32            base  = face->vertex_start;
        u32            count = face->vertex_count;

        for (u32 ti = 0; ti + 2 < count; ti += 3) {
            const MapVertex *va = &map->vertices[base + ti + 0];
            const MapVertex *vb = &map->vertices[base + ti + 1];
            const MapVertex *vc = &map->vertices[base + ti + 2];

            Vec3 v0 = { va->pos[0], va->pos[1], va->pos[2] };
            Vec3 v1 = { vb->pos[0], vb->pos[1], vb->pos[2] };
            Vec3 v2 = { vc->pos[0], vc->pos[1], vc->pos[2] };

            Vec3 e1 = vec3_sub(v1, v0);
            Vec3 e2 = vec3_sub(v2, v0);
            Vec3 n  = vec3_norm(vec3_cross(e1, e2));

            /* Solo caras casi verticales: suelo, techo y rampas no son pared. */
            if (fabsf(n.y) >= PARED_NORMAL_Y_MAX) { continue; }

            /* ¿Nos acercamos a esta pared? (dot < 0 = movimiento hacia ella) */
            f32 into = vec3_dot(*vel, n);
            if (into >= 0.0f) { continue; }

            /* Rayo desde el jugador HACIA la pared (opuesto a la normal):
             * mide la distancia perpendicular REAL a la pared, sea cual sea
             * el ángulo de la velocidad. Se prueban varias alturas para que
             * una caja alta no pase por debajo del rayo de la cintura. */
            const f32 alturas[3] = {
                pos->y + 2.0f,
                pos->y + crouch_height * 0.5f,
                pos->y + crouch_height - 2.0f
            };
            Vec3 dir_ray = { -n.x, -n.y, -n.z };

            for (int a = 0; a < 3; a++) {
                Vec3 orig = { pos->x, alturas[a], pos->z };
                f32  t;

                if (!collision_ray_triangulo(orig, dir_ray, v0, v1, v2, &t, NULL)) {
                    continue;
                }
                if (t >= RADIO_JUGADOR) { continue; }

                /* Empujar fuera, PERPENDICULAR a la pared, hasta el radio.
                 * En esquinas cada pared se corrige por separado. */
                f32 deficit = RADIO_JUGADOR - t;
                *pos = vec3_add(*pos, vec3_scale(n, deficit));

                /* Slide: quitar la componente de velocidad hacia la pared,
                 * conservando la tangencial (se resbala a lo largo de ella). */
                *vel = vec3_sub(*vel, vec3_scale(n, into));

                corregido = true;
                break;  /* una corrección por cara basta */
            }
        }
    }

    return corregido;
}

/* ── Step-up (5a-3) ──────────────────────────────────────────────────────── *
 * Sube escalones planos de hasta SV_STEPSIZE (18 u) cuando el jugador está
 * en suelo y se mueve en horizontal. Las rampas NO se suben aquí: su
 * normal.y < 0.95 y se resuelven con el snap al suelo de phys_tick.         */
bool collision_intentar_step_up(const Map *map, Vec3 *pos, const Vec3 *wish_dir,
                                 bool en_suelo_antes, f32 crouch_height)
{
    (void)crouch_height;  /* el rayo se mide desde los pies + SV_STEPSIZE */

    if (!map || !map->cargado || map->face_count == 0) { return false; }
    if (!en_suelo_antes) { return false; }

    if (!wish_dir || (wish_dir->x == 0.0f && wish_dir->z == 0.0f)) {
        return false;
    }

    Vec3 dir_xz = { wish_dir->x, 0.0f, wish_dir->z };
    f32  len    = sqrtf(dir_xz.x * dir_xz.x + dir_xz.z * dir_xz.z);
    if (len < 0.001f) { return false; }
    dir_xz.x /= len;
    dir_xz.z /= len;

    /* Origen: un poco adelante del jugador y por encima del escalón máximo. */
    Vec3 orig = {
        pos->x + dir_xz.x * STEP_UP_AHEAD_DIST,
        pos->y + SV_STEPSIZE + STEP_UP_RAY_MARGIN,
        pos->z + dir_xz.z * STEP_UP_AHEAD_DIST
    };
    Vec3 dir  = { 0.0f, -1.0f, 0.0f };

    f32 best_t = SV_STEPSIZE + 2.0f * STEP_UP_RAY_MARGIN;
    Vec3 best_n = { 0.0f, 1.0f, 0.0f };
    bool encontrado = false;

    for (i32 fi = 0; fi < map->face_count; fi++) {
        const MapFace *face  = &map->faces[fi];
        u32            base  = face->vertex_start;
        u32            count = face->vertex_count;

        for (u32 ti = 0; ti + 2 < count; ti += 3) {
            const MapVertex *va = &map->vertices[base + ti + 0];
            const MapVertex *vb = &map->vertices[base + ti + 1];
            const MapVertex *vc = &map->vertices[base + ti + 2];

            Vec3 v0 = { va->pos[0], va->pos[1], va->pos[2] };
            Vec3 v1 = { vb->pos[0], vb->pos[1], vb->pos[2] };
            Vec3 v2 = { vc->pos[0], vc->pos[1], vc->pos[2] };

            f32 t;
            Vec3 normal;
            if (collision_ray_triangulo(orig, dir, v0, v1, v2, &t, &normal)) {
                if (normal.y > 0.0f && t < best_t) {
                    best_t = t;
                    best_n = normal;
                    encontrado = true;
                }
            }
        }
    }

    if (!encontrado) { return false; }

    /* Solo escalones planos. Una rampa se detecta como suelo en phys_tick. */
    if (best_n.y < STEP_UP_NORMAL_Y_MIN) { return false; }

    f32 ground_y = orig.y - best_t;
    f32 delta_y  = ground_y - pos->y;

    /* Solo subir si el escalón está entre 0.5 y 18 u sobre los pies. */
    if (delta_y > 0.5f && delta_y <= SV_STEPSIZE) {
        /* Subir y avanzar hasta el punto sondeado, que está sobre la
         * superficie del escalón. Sin el avance el centro seguiría sobre el
         * suelo bajo y el snap devolvería al jugador abajo, en bucle contra
         * la cara. Avanzar al punto del rayo funciona también en diagonal:
         * es exactamente donde se encontró el suelo del escalón. */
        pos->x = orig.x;
        pos->y = ground_y;
        pos->z = orig.z;
        return true;
    }

    return false;
}
