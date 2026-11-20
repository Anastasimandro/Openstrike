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
#include "broadphase.h"
#include <math.h>
#include <string.h>

/* ── Constantes internas ─────────────────────────────────────────────────── */

#define SWEEP_EPSILON 1e-4f
#define MAX_SLIDE_PASSES 2
#define STEP_UP_AHEAD_DIST 20.0f
#define CEILING_CHECK_DIST 4.0f

/* Buffer temporal para índices de broadphase (evita malloc en stack) */
#define BP_TEMP_MAX 128
static u16 g_bp_temp[BP_TEMP_MAX];

/* ── Möller–Trumbore: intersección rayo–triángulo (pública) ──────────────── */
bool collision_ray_triangulo(Vec3 orig, Vec3 dir,
                              Vec3 v0, Vec3 v1, Vec3 v2,
                              f32 *t_out, Vec3 *normal_out)
{
    Vec3 e1 = vec3_sub(v1, v0);
    Vec3 e2 = vec3_sub(v2, v0);

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
    if (t < 1e-6f) { return false; }

    *t_out = t;
    if (normal_out) {
        *normal_out = vec3_norm(vec3_cross(e1, e2));
    }
    return true;
}

/* ── Swept Sphere vs Triangle (para cápsulas: cilindro + 2 esferas) ───────── *
 * Algoritmo: separar en 3 volúmenes primitivos:
 * 1. Esfera inferior (start + radius*up, end + radius*up)
 * 2. Cilindro central (segmento start+half_height*up → end+half_height*up, radio)
 * 3. Esfera superior (start + (half_height+radius)*up, end + (half_height+radius)*up)
 * Para cada primitiva, testear colisión contra triángulo.
 * Retorna el hit con menor t.                                           */

#define BARY_TOLERANCE 1e-3f

static bool sweep_sphere_tri(Vec3 sphere_start, Vec3 sphere_end, f32 radius,
                              Vec3 v0, Vec3 v1, Vec3 v2,
                              f32 *t_out, Vec3 *normal_out)
{
    Vec3 dir = vec3_sub(sphere_end, sphere_start);
    f32 len = vec3_len(dir);
    if (len < 1e-6f) return false;
    dir = vec3_scale(dir, 1.0f / len);

    /* Plano del triángulo */
    Vec3 e1 = vec3_sub(v1, v0);
    Vec3 e2 = vec3_sub(v2, v0);
    Vec3 n = vec3_norm(vec3_cross(e1, e2));
    f32 plane_d = vec3_dot(n, v0);

    /* Distancia firmada de la esfera al plano en start y end */
    f32 dist_start = vec3_dot(n, sphere_start) - plane_d;
    f32 dist_end   = vec3_dot(n, sphere_end) - plane_d;

    /* Si ambos puntos están del mismo lado y fuera del radio, no hay cruce */
    if (dist_start > radius && dist_end > radius) return false;
    if (dist_start < -radius && dist_end < -radius) return false;

    /* Caso 1: La esfera YA toca/penetra el plano en start (|dist_start| <= radius).
     * El punto de contacto en el plano es sphere_start - n * dist_start. */
    if (fabsf(dist_start) <= radius + 1e-4f) {
        Vec3 contact = vec3_sub(sphere_start, vec3_scale(n, dist_start));
        Vec3 c0 = vec3_cross(vec3_sub(v1, v0), vec3_sub(contact, v0));
        Vec3 c1 = vec3_cross(vec3_sub(v2, v1), vec3_sub(contact, v1));
        Vec3 c2 = vec3_cross(vec3_sub(v0, v2), vec3_sub(contact, v2));
        if (vec3_dot(n, c0) >= -BARY_TOLERANCE && vec3_dot(n, c1) >= -BARY_TOLERANCE && vec3_dot(n, c2) >= -BARY_TOLERANCE) {
            *t_out = 0.0f;
            if (normal_out) *normal_out = n;
            return true;
        }
        /* Si no está dentro del triángulo, seguir con test de cruce normal */
    }

    /* t donde la esfera toca el plano (|dist| = radius) */
    f32 denom = dist_end - dist_start;
    if (fabsf(denom) < 1e-6f) return false;  // paralela al plano

    f32 t1 = (radius - dist_start) / denom;
    f32 t2 = (-radius - dist_start) / denom;

    f32 t_plane = 1.0f + 1e-4f;
    if (t1 >= 0.0f && t1 <= 1.0f) t_plane = t1;
    if (t2 >= 0.0f && t2 <= 1.0f && t2 < t_plane) t_plane = t2;

    if (t_plane > 1.0f) return false;

    /* Punto de contacto en el plano: sphere en t_plane, luego proyectar al plano y desplazar por radio */
    Vec3 sphere_at_t = vec3_add(sphere_start, vec3_scale(dir, t_plane * len));
    f32 dist_at_t = dist_start + (dist_end - dist_start) * t_plane;
    Vec3 contact = vec3_sub(sphere_at_t, vec3_scale(n, dist_at_t));
    /* Desplazar desde el plano hacia la esfera por radio */
    if (dist_at_t > 0) contact = vec3_add(contact, vec3_scale(n, radius));
    else contact = vec3_sub(contact, vec3_scale(n, radius));

    /* Test baricéntrico: ¿contact está dentro del triángulo? */
    Vec3 c0 = vec3_cross(vec3_sub(v1, v0), vec3_sub(contact, v0));
    Vec3 c1 = vec3_cross(vec3_sub(v2, v1), vec3_sub(contact, v1));
    Vec3 c2 = vec3_cross(vec3_sub(v0, v2), vec3_sub(contact, v2));
    if (vec3_dot(n, c0) < -BARY_TOLERANCE || vec3_dot(n, c1) < -BARY_TOLERANCE || vec3_dot(n, c2) < -BARY_TOLERANCE) {
        return false;  // fuera del triángulo
    }

    *t_out = t_plane;
    if (normal_out) *normal_out = n;
    return true;
}

static bool sweep_cylinder_tri(Vec3 cyl_start, Vec3 cyl_end, f32 radius, f32 half_height_unused,
                                Vec3 v0, Vec3 v1, Vec3 v2,
                                f32 *t_out, Vec3 *normal_out)
{
    (void)half_height_unused;

    /* El cilindro es un segmento (eje) + radio.
     * Aproximación: testear intersección segmento-triángulo con radio expandido.
     * Usamos test de distancia segmento-triángulo < radius (slab + proyección). */

    Vec3 seg_dir = vec3_sub(cyl_end, cyl_start);
    f32 seg_len = vec3_len(seg_dir);
    if (seg_len < 1e-6f) return false;
    seg_dir = vec3_scale(seg_dir, 1.0f / seg_len);

    /* Plano del triángulo */
    Vec3 e1 = vec3_sub(v1, v0);
    Vec3 e2 = vec3_sub(v2, v0);
    Vec3 n = vec3_norm(vec3_cross(e1, e2));
    f32 plane_d = vec3_dot(n, v0);

    /* Distancias firmadas de los extremos del segmento al plano */
    f32 d0 = vec3_dot(n, cyl_start) - plane_d;
    f32 d1 = vec3_dot(n, cyl_end) - plane_d;

    /* Si el segmento cruza el plano (o está cerca dentro de radio) */
    if (d0 > radius && d1 > radius) return false;
    if (d0 < -radius && d1 < -radius) return false;

    /* Caso 1: El segmento YA está dentro del slab en start (|d0| <= radius).
     * Devolver hit en t=0 si la proyección está en el triángulo. */
    if (fabsf(d0) <= radius + 1e-4f) {
        Vec3 proj = vec3_sub(cyl_start, vec3_scale(n, d0));
        Vec3 c0 = vec3_cross(vec3_sub(v1, v0), vec3_sub(proj, v0));
        Vec3 c1 = vec3_cross(vec3_sub(v2, v1), vec3_sub(proj, v1));
        Vec3 c2 = vec3_cross(vec3_sub(v0, v2), vec3_sub(proj, v2));
        f32 dot0 = vec3_dot(n, c0);
        f32 dot1 = vec3_dot(n, c1);
        f32 dot2 = vec3_dot(n, c2);
        if (dot0 >= -radius && dot1 >= -radius && dot2 >= -radius) {
            *t_out = 0.0f;
            if (normal_out) *normal_out = n;
            return true;
        }
    }

    /* Clampear a [-radius, radius] para encontrar t de cruce del "slab" */
    f32 t_enter = 0.0f, t_exit = 1.0f;
    if (fabsf(d1 - d0) > 1e-6f) {
        t_enter = fmaxf(0.0f, (-radius - d0) / (d1 - d0));
        t_exit  = fminf(1.0f, (radius - d0) / (d1 - d0));
        if (t_enter > t_exit) return false;
    }

    /* Punto en el segmento en t_enter */
    Vec3 p_enter = vec3_add(cyl_start, vec3_scale(seg_dir, t_enter * seg_len));

    /* Proyectar p_enter al plano del triángulo */
    Vec3 proj = vec3_sub(p_enter, vec3_scale(n, vec3_dot(n, p_enter) - plane_d));

    /* Test si proj está dentro del triángulo (con tolerancia radius) */
    Vec3 c0 = vec3_cross(vec3_sub(v1, v0), vec3_sub(proj, v0));
    Vec3 c1 = vec3_cross(vec3_sub(v2, v1), vec3_sub(proj, v1));
    Vec3 c2 = vec3_cross(vec3_sub(v0, v2), vec3_sub(proj, v2));

    f32 dot0 = vec3_dot(n, c0);
    f32 dot1 = vec3_dot(n, c1);
    f32 dot2 = vec3_dot(n, c2);

    if (dot0 >= -radius && dot1 >= -radius && dot2 >= -radius) {
        *t_out = t_enter;
        if (normal_out) *normal_out = n;
        return true;
    }

    /* Si no, testear aristas del triángulo (distancia segmento-segmento < radius) */
    /* Omitido por simplicidad: la aproximación de slab + proyección cubre 95% casos.
     * Para bordes afilados, el barrido de esferas inferior/superior los cubre. */
    return false;
}

/* ── Swept Capsule completa ──────────────────────────────────────────────── */

/* Check if triangle is degenerate (zero area) */
static inline bool triangle_is_degenerate(Vec3 v0, Vec3 v1, Vec3 v2)
{
    Vec3 e1 = vec3_sub(v1, v0);
    Vec3 e2 = vec3_sub(v2, v0);
    Vec3 cross = vec3_cross(e1, e2);
    f32 area2 = vec3_dot(cross, cross);
    return area2 < 1e-12f;  // squared area < epsilon
}

SweepHit collision_sweep_capsule(const Map *map, const Broadphase *bp,
                                  Vec3 start, Vec3 end,
                                  f32 radius, f32 half_height)
{
    SweepHit best = {0};
    best.t = 1.0f + SWEEP_EPSILON;  // no hit

    if (!map || !map->cargado || map->face_count == 0) {
        return best;
    }

    Vec3 up = { 0.0f, 1.0f, 0.0f };
    Vec3 cyl_start = vec3_add(start, vec3_scale(up, half_height));
    Vec3 cyl_end   = vec3_add(end,   vec3_scale(up, half_height));
    Vec3 sph_bot_start = vec3_add(start, vec3_scale(up, radius));
    Vec3 sph_bot_end   = vec3_add(end,   vec3_scale(up, radius));
    Vec3 sph_top_start = vec3_add(start, vec3_scale(up, half_height + radius));
    Vec3 sph_top_end   = vec3_add(end,   vec3_scale(up, half_height + radius));

    u32 face_count = 0;
    u16 *face_indices = NULL;

    if (bp && bp->built) {
        /* Broadphase: query segment start→end con AABB expandido por radius */
        Vec3 mins = {
            fminf(start.x, end.x) - radius,
            fminf(start.y, end.y) - radius - half_height,
            fminf(start.z, end.z) - radius
        };
        Vec3 maxs = {
            fmaxf(start.x, end.x) + radius,
            fmaxf(start.y, end.y) + radius + half_height,
            fmaxf(start.z, end.z) + radius
        };
        face_count = broadphase_query_aabb(bp, mins, maxs, g_bp_temp, BP_TEMP_MAX);
        face_indices = g_bp_temp;
    } else {
        /* Fallback: todas las caras (O(N)) */
        face_count = map->face_count;
        face_indices = NULL;  // flag para iterar todas
    }

    for (u32 fi_idx = 0; fi_idx < face_count; fi_idx++) {
        u32 fi = face_indices ? face_indices[fi_idx] : fi_idx;
        if (fi >= (u32)map->face_count) continue;

        const MapFace *face = &map->faces[fi];
        u32 base = face->vertex_start;
        u32 count = face->vertex_count;

        for (u32 ti = 0; ti + 2 < count; ti += 3) {
            const MapVertex *va = &map->vertices[base + ti + 0];
            const MapVertex *vb = &map->vertices[base + ti + 1];
            const MapVertex *vc = &map->vertices[base + ti + 2];

            Vec3 v0 = { va->pos[0], va->pos[1], va->pos[2] };
            Vec3 v1 = { vb->pos[0], vb->pos[1], vb->pos[2] };
            Vec3 v2 = { vc->pos[0], vc->pos[1], vc->pos[2] };

            /* Skip degenerate triangles to avoid NaN normals */
            if (triangle_is_degenerate(v0, v1, v2)) continue;

            f32 t;
            Vec3 n;

            /* 1. Esfera inferior */
            if (sweep_sphere_tri(sph_bot_start, sph_bot_end, radius, v0, v1, v2, &t, &n)) {
                if (t < best.t) {
                    best.hit = true;
                    best.t = t;
                    best.normal = n;
                    best.pos = vec3_add(start, vec3_scale(vec3_sub(end, start), t));
                }
            }

            /* 2. Cilindro central (solo si half_height > epsilon) */
            if (half_height > CAPSULE_MIN_HEIGHT * 0.5f) {
                if (sweep_cylinder_tri(cyl_start, cyl_end, radius, half_height, v0, v1, v2, &t, &n)) {
                    if (t < best.t) {
                        best.hit = true;
                        best.t = t;
                        best.normal = n;
                        best.pos = vec3_add(start, vec3_scale(vec3_sub(end, start), t));
                    }
                }
            }

            /* 3. Esfera superior */
            if (sweep_sphere_tri(sph_top_start, sph_top_end, radius, v0, v1, v2, &t, &n)) {
                if (t < best.t) {
                    best.hit = true;
                    best.t = t;
                    best.normal = n;
                    best.pos = vec3_add(start, vec3_scale(vec3_sub(end, start), t));
                }
            }
        }
    }

    return best;
}

/* ── API Legacy (redirigen a swept) ──────────────────────────────────────── */

FloorHit collision_detectar_suelo(const Map *map, const Broadphase *bp, Vec3 pos, f32 crouch_height)
{
    FloorHit result = {0};

    if (!map || !map->cargado || map->face_count == 0) return result;

    /* Usar una esfera MUY DELGADA (radio 0.1) para detectar suelo.
     * El centro de la esfera empieza en pos + (0, radius, 0) para que la
     * esfera toque el suelo en pos.y si hay suelo justo debajo. */
    f32 radius = 0.1f;
    f32 half_height = 0.0f;

    /* Barrido hacia abajo: desde los pies actuales + radio hasta un poco más abajo del step-up */
    f32 down_dist = crouch_height * 0.5f + SV_STEPSIZE + 2.0f;
    Vec3 start = { pos.x, pos.y + radius, pos.z };
    Vec3 end = { pos.x, pos.y - down_dist + radius, pos.z };

    SweepHit hit = collision_sweep_capsule(map, bp, start, end, radius, half_height);

    if (hit.hit && hit.normal.y > SUELO_NORMAL_Y_MIN) {
        result.hit = true;
        /* hit.pos es el centro de la esfera en el impacto.
         * El suelo está en hit.pos.y - radius. */
        result.y = hit.pos.y - radius;
        result.normal = hit.normal;
    }

    return result;
}

bool collision_bloquear_pared(const Map *map, const Broadphase *bp, Vec3 start_pos, Vec3 *vel, f32 dt, f32 crouch_height, Vec3 *out_pos)
{
    if (!map || !map->cargado || map->face_count == 0) {
        *out_pos = vec3_add(start_pos, vec3_scale(*vel, dt));
        return false;
    }

    f32 radius = RADIO_JUGADOR;
    f32 capsule_h = crouch_height - 2.0f * radius;
    if (capsule_h < CAPSULE_MIN_HEIGHT) capsule_h = CAPSULE_MIN_HEIGHT;
    f32 half_height = capsule_h * 0.5f;

    Vec3 original_pos = start_pos;
    Vec3 start = start_pos;
    Vec3 end = vec3_add(start_pos, vec3_scale(*vel, dt));

    bool any_correction = false;

    for (int pass = 0; pass < MAX_SLIDE_PASSES; pass++) {
        SweepHit hit = collision_sweep_capsule(map, bp, start, end, radius, half_height);

        if (!hit.hit) {
            *out_pos = end;
            break;
        }

        /* Ignorar choques con suelo (normal.y > 0.7) y techo (normal.y < -0.7).
         * Solo procesar paredes verticales (|normal.y| <= 0.7). */
        if (hit.normal.y > 0.7f || hit.normal.y < -0.7f) {
            /* No es pared: dejar que la integración normal + detección de suelo lo maneje. */
            *out_pos = end;
            break;
        }

        /* Impacto en t: mover hasta el contacto + pequeño epsilon */
        f32 move_t = hit.t - SWEEP_EPSILON;
        if (move_t < 0.0f) move_t = 0.0f;
        Vec3 new_pos = vec3_add(start, vec3_scale(vec3_sub(end, start), move_t));

        /* Deslizar: velocidad restante proyectada en el plano de la pared */
        Vec3 remaining_vel = vec3_scale(vec3_sub(end, start), 1.0f - hit.t);
        f32 into = vec3_dot(remaining_vel, hit.normal);
        if (into < 0.0f) {
            remaining_vel = vec3_sub(remaining_vel, vec3_scale(hit.normal, into));
        }

        start = new_pos;
        end = vec3_add(start, remaining_vel);
        any_correction = true;

        /* Si la velocidad restante es casi nula, terminar */
        if (vec3_len(remaining_vel) < 0.01f) {
            *out_pos = new_pos;
            break;
        }
    }

    if (any_correction) {
        /* Velocidad final = desplazamiento real total / dt */
        *vel = vec3_scale(vec3_sub(*out_pos, original_pos), 1.0f / dt);
    } else {
        *out_pos = end;
    }

    return any_correction;
}

bool collision_intentar_step_up(const Map *map, const Broadphase *bp, Vec3 *pos, const Vec3 *move_dir,
                                 bool en_suelo_antes, f32 crouch_height)
{
    if (!map || !map->cargado || map->face_count == 0) return false;
    if (!en_suelo_antes) return false;

    if (!move_dir || (move_dir->x == 0.0f && move_dir->z == 0.0f)) return false;

    f32 radius = RADIO_JUGADOR;
    f32 capsule_h = crouch_height - 2.0f * radius;
    if (capsule_h < CAPSULE_MIN_HEIGHT) capsule_h = CAPSULE_MIN_HEIGHT;
    f32 half_height = capsule_h * 0.5f;

    /* Barrido: empezar ligeramente ATRÁS del jugador para que la cápsula
     * no solape ya el escalón/rampa en el inicio. Luego adelante + arriba. */
    const f32 BACK_OFFSET = 8.0f;  // mitad del radio aprox
    Vec3 start = { pos->x - move_dir->x * BACK_OFFSET, pos->y, pos->z - move_dir->z * BACK_OFFSET };
    Vec3 horiz_move = { move_dir->x * (STEP_UP_AHEAD_DIST + BACK_OFFSET), 0.0f, move_dir->z * (STEP_UP_AHEAD_DIST + BACK_OFFSET) };
    Vec3 end = { start.x + horiz_move.x, start.y + SV_STEPSIZE, start.z + horiz_move.z };

    SweepHit hit = collision_sweep_capsule(map, bp, start, end, radius, half_height);

    if (hit.hit && hit.normal.y > SUELO_NORMAL_Y_MIN) {
        f32 ground_y = hit.pos.y;  // pos.y de la cápsula en impacto
        f32 delta_y = ground_y - pos->y;  // delta desde posición original del jugador

        if (delta_y > 0.5f && delta_y <= SV_STEPSIZE) {
            pos->y = ground_y;
            return true;
        }
    }

    return false;
}

bool collision_check_ceiling(const Map *map, const Broadphase *bp, Vec3 pos, f32 crouch_height, f32 max_dist)
{
    if (!map || !map->cargado || map->face_count == 0) return false;

    f32 radius = RADIO_JUGADOR * 0.5f;  // check más estrecho para techo
    f32 half_height = 0.0f;

    Vec3 start = { pos.x, pos.y + crouch_height, pos.z };
    Vec3 end = { start.x, start.y + max_dist, start.z };

    SweepHit hit = collision_sweep_capsule(map, bp, start, end, radius, half_height);

    return hit.hit && hit.normal.y < TECHO_NORMAL_Y_MAX;
}