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
// MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "collision.h"
#include <math.h>
#include <string.h>

/* ── constantes internas ─────────────────────────────────────────────────── */

/* Distancia máxima de búsqueda de suelo = crouch_height/2 + SV_STEPSIZE + 2 
 * (se calcula dinámicamente en cada función) */
#define STEP_UP_AHEAD_DIST  20.0f

/* ── Möller–Trumbore: intersección rayo–triángulo (pública) ──────────────── *
 * Generalizada (5a-2): acepta triángulos de CUALQUIER orientación — suelo,
 * pared o techo. El filtrado por orientación lo hace cada llamada según su
 * caso (suelo: normal.y > 0; pared: |normal.y| < 0.3; techo: normal.y < -0.7).
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

/* ── API pública ─────────────────────────────────────────────────────────── */

FloorHit collision_detectar_suelo(const Map *map, Vec3 pos, f32 crouch_height)
{
    FloorHit result;
    memset(&result, 0, sizeof(result));
    result.hit = false;

    if (!map || !map->cargado || map->face_count == 0) {
        return result;
    }

    /* Rayo hacia abajo: origen a la mitad de la altura de la caja */
    f32 ray_orig_offset = crouch_height * 0.5f;
    /* Distancia máxima: desde el origen hasta los pies + step size + margen */
    f32 suelo_max_dist = ray_orig_offset + SV_STEPSIZE + 2.0f;

    Vec3 orig = { pos.x, pos.y + ray_orig_offset, pos.z };
    Vec3 dir  = { 0.0f, -1.0f, 0.0f };

    /* Buscamos el hit más cercano dentro del rango permitido */
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
                /* Filtrar solo caras de suelo (la base ya no filtra) */
                if (normal.y > 0.0f && t < best_t) {
                    best_t      = t;
                    result.hit  = true;
                    /* Y del suelo = Y del origen - distancia recorrida hacia abajo */
                    result.y      = orig.y - t;
                    result.normal = normal;
                }
            }
        }
    }

    return result;
}

/* ── API pública: bloqueo de paredes con slide ───────────────────────────── */

bool collision_bloquear_pared(const Map *map, Vec3 *pos, Vec3 *vel, f32 dt, f32 crouch_height)
{
    (void)dt;  // no se usa directamente, pero se mantiene por compatibilidad

    if (!map || !map->cargado || map->face_count == 0) { return false; }

    f32 xz = sqrtf(vel->x * vel->x + vel->z * vel->z);
    if (xz < 0.001f) { return false; }   /* parado: nada que bloquear */

    /* Origen del rayo a la mitad de la altura de la caja (equivalente a rodillas) */
    f32 ray_orig_offset = crouch_height * 0.5f;
    Vec3 orig = { pos->x, pos->y + ray_orig_offset, pos->z };

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

            /* Solo caras casi verticales */
            if (fabsf(n.y) >= PARED_NORMAL_Y_MAX) { continue; }

            /* Rayo desde el jugador HACIA la pared (opuesto a la normal):
             * mide la distancia perpendicular REAL a la pared, sea cual sea
             * el ángulo de la velocidad. */
            Vec3 dir_ray = { -n.x, -n.y, -n.z };
            f32  t;
            if (!ray_triangulo(orig, dir_ray, v0, v1, v2, &t, NULL)) {
                continue;
            }

            /* ¿Nos acercamos a esta pared? (dot < 0 = movimiento hacia ella) */
            f32 into = vec3_dot(*vel, n);
            if (into >= 0.0f) { continue; }

            /* ¿Ya estamos dentro del radio de esta pared? */
            if (t >= RADIO_JUGADOR) { continue; }

            /* Empujar fuera, PERPENDICULAR a la pared, hasta el radio.
             * El empuje es perpendicular → la distancia de bloqueo es
             * siempre la misma (16 u) en cualquier ángulo de llegada.
             * En esquinas cada pared se corrige por separado. */
            f32 deficit = RADIO_JUGADOR - t;
            *pos = vec3_add(*pos, vec3_scale(n, deficit));

            /* Slide: quitar la componente de la velocidad hacia la pared,
             * conservando la tangencial (se resbala a lo largo de ella). */
            *vel = vec3_sub(*vel, vec3_scale(n, into));

            corregido = true;
        }
    }

    return corregido;
}

/* ── Step-up (5a-3) ──────────────────────────────────────────────────────── *
 * Intenta subir un escalón de hasta SV_STEPSIZE (18 u) cuando el jugador
 * choca contra un borde bajo mientras camina.
 * Llamar tras collision_bloquear_pared y ANTES de collision_detectar_suelo.
 *
 * Algoritmo:
 *  1. Solo si estaba en suelo antes del tick Y hay movimiento en XZ.
 *  2. Desde (pos + wish_dir * STEP_UP_AHEAD_DIST + (0, crouch_height/2 + SV_STEPSIZE, 0)),
 *     lanza rayo hacia abajo buscando suelo.
 *  3. Si hay suelo y su Y está entre pos.y + 0.5 y pos.y + SV_STEPSIZE:
 *       → pos.y = suelo_y (teleport suave).
 *       → vel.y = 0.
 *  4. Retorna true si subió, false si no. */
bool collision_intentar_step_up(const Map *map, Vec3 *pos, const Vec3 *wish_dir,
                                bool en_suelo_antes, f32 crouch_height)
{
    if (!map || !map->cargado || map->face_count == 0) { return false; }
    if (!en_suelo_antes) { return false; }

    /* Calcular dirección de movimiento en XZ desde wish_dir (dirección
     * deseada normalizada que viene del input). Solo step-up si hay
     * movimiento horizontal. */
    if (!wish_dir || (wish_dir->x == 0.0f && wish_dir->z == 0.0f)) {
        return false;
    }

    Vec3 dir_xz = { wish_dir->x, 0.0f, wish_dir->z };
    f32  len    = sqrtf(dir_xz.x * dir_xz.x + dir_xz.z * dir_xz.z);
    if (len < 0.001f) { return false; }
    dir_xz.x /= len;
    dir_xz.z /= len;

    /* Origen del rayo: un poco adelante del jugador y (crouch_height/2 + 18) u arriba */
    f32 ray_orig_y = pos->y + crouch_height * 0.5f + SV_STEPSIZE;
    Vec3 orig = {
        pos->x + dir_xz.x * STEP_UP_AHEAD_DIST,
        ray_orig_y,
        pos->z + dir_xz.z * STEP_UP_AHEAD_DIST
    };
    Vec3 dir  = { 0.0f, -1.0f, 0.0f };

    f32 best_t = SV_STEPSIZE + 2.0f;  /* un poco más que 18 para tolerancia */

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
                }
            }
        }
    }

    if (best_t < SV_STEPSIZE + 2.0f) {
        f32 ground_y = orig.y - best_t;
        f32 delta_y  = ground_y - pos->y;

        /* Solo subir si el escalón está entre 0.5 y 18 u sobre los pies.
         * 0.5 es el mismo umbral que collision_detectar_suelo usa para
         * aterrizar (pos.y <= fh.y + 0.5). */
        if (delta_y > 0.5f && delta_y <= SV_STEPSIZE) {
            pos->y = ground_y;
            return true;
        }
    }

    return false;
}