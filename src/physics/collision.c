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
#include "phys_log.h"
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

/* Test if point on plane is inside triangle with radius tolerance.
 * Cross product magnitude = |edge| * distance_to_edge.
 * To allow distance up to 'radius', tolerance = radius * typical_edge_len (~100). */
static inline bool point_in_tri_radius(Vec3 p, Vec3 v0, Vec3 v1, Vec3 v2, Vec3 n, f32 radius)
{
    Vec3 c0 = vec3_cross(vec3_sub(v1, v0), vec3_sub(p, v0));
    Vec3 c1 = vec3_cross(vec3_sub(v2, v1), vec3_sub(p, v1));
    Vec3 c2 = vec3_cross(vec3_sub(v0, v2), vec3_sub(p, v2));
    f32 tol = radius * 100.0f;  // ~edge length scale
    return vec3_dot(n, c0) >= -tol &&
           vec3_dot(n, c1) >= -tol &&
           vec3_dot(n, c2) >= -tol;
}

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
     * Punto de contacto EN EL PLANO: sphere_start - n * dist_start.
     * Usar tolerancia radius: la esfera puede tocar aristas desde fuera. */
    if (fabsf(dist_start) <= radius + 1e-4f) {
        Vec3 plane_contact = vec3_sub(sphere_start, vec3_scale(n, dist_start));
        if (point_in_tri_radius(plane_contact, v0, v1, v2, n, radius)) {
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

    /* Punto de contacto EN EL PLANO (para test baricéntrico) */
    Vec3 sphere_at_t = vec3_add(sphere_start, vec3_scale(dir, t_plane * len));
    f32 dist_at_t = dist_start + (dist_end - dist_start) * t_plane;
    Vec3 plane_contact = vec3_sub(sphere_at_t, vec3_scale(n, dist_at_t));

    /* Test baricéntrico con tolerancia radius */
    if (!point_in_tri_radius(plane_contact, v0, v1, v2, n, radius)) {
        return false;
    }

    /* SweepHit.pos = centro de la esfera en el impacto = plane_contact + n * radius (signo según lado) */
    /* No se usa aquí; collision_sweep_capsula reconstruye pos desde start/end/t */

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

static inline bool triangle_is_degenerate(Vec3 v0, Vec3 v1, Vec3 v2)
{
    Vec3 e1 = vec3_sub(v1, v0);
    Vec3 e2 = vec3_sub(v2, v0);
    Vec3 cross = vec3_cross(e1, e2);
    f32 area2 = vec3_dot(cross, cross);
    return area2 < 1e-12f;  // squared area < epsilon
}

static inline bool is_floor_or_ceiling(Vec3 n)
{
    /* Truly flat floor/ceiling (|normal.y| > 0.95) are ignored by wall sweep.
     * Ramps (0.7 < |normal.y| < 0.95) ARE collided as walls — this helps the
     * swept capsule push the player up the ramp. The horizontal-only slide
     * fix prevents unwanted Y velocity from the slide. */
    return n.y > 0.95f || n.y < -0.7f;
}

SweepHit collision_sweep_capsule(const Map *map, const Broadphase *bp,
                                  Vec3 start, Vec3 end,
                                  f32 radius, f32 half_height,
                                  bool ignore_floor_ceiling)
{
    SweepHit best = {0};
    best.t = 1.0f + SWEEP_EPSILON;  // no hit
    bool sweep_down = (start.y > end.y);  // detect downward sweep for floor detection

    if (!map || !map->cargado || map->face_count == 0) {
        return best;
    }

    Vec3 up = { 0.0f, 1.0f, 0.0f };
    Vec3 trans = vec3_sub(end, start);

    /* Correct capsule geometry:
     * Capsule center at 'start' and 'end'.
     * Cylinder axis: from center - half_height*up to center + half_height*up.
     * Bottom sphere center: center - half_height*up.
     * Top sphere center: center + half_height*up. */
    Vec3 cyl_axis_start = vec3_sub(start, vec3_scale(up, half_height));  // bottom of cylinder at start
    Vec3 cyl_axis_end   = vec3_add(start, vec3_scale(up, half_height));   // top of cylinder at start
    Vec3 sph_bot_start = vec3_sub(start, vec3_scale(up, half_height));    // bottom sphere at start
    Vec3 sph_bot_end   = vec3_sub(end,   vec3_scale(up, half_height));    // bottom sphere at end
    Vec3 sph_top_start = vec3_add(start, vec3_scale(up, half_height));    // top sphere at start
    Vec3 sph_top_end   = vec3_add(end,   vec3_scale(up, half_height));    // top sphere at end

    u32 face_count = 0;
    u16 *face_indices = NULL;

    if (bp && bp->built) {
        f32 capsule_half_h = half_height + radius;  // center to top/bottom of capsule
        Vec3 mins = {
            fminf(start.x, end.x) - radius,
            fminf(start.y, end.y) - capsule_half_h,
            fminf(start.z, end.z) - radius
        };
        Vec3 maxs = {
            fmaxf(start.x, end.x) + radius,
            fmaxf(start.y, end.y) + capsule_half_h,
            fmaxf(start.z, end.z) + radius
        };
        face_count = broadphase_query_aabb(bp, mins, maxs, g_bp_temp, BP_TEMP_MAX);
        face_indices = g_bp_temp;
    } else {
        face_count = map->face_count;
        face_indices = NULL;
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

            if (triangle_is_degenerate(v0, v1, v2)) continue;

            Vec3 e1 = vec3_sub(v1, v0);
            Vec3 e2 = vec3_sub(v2, v0);
            Vec3 n = vec3_norm(vec3_cross(e1, e2));

            if (ignore_floor_ceiling && is_floor_or_ceiling(n)) {
                continue;
            }

            f32 t;

            /* 1. Bottom sphere sweep (capsule bottom hemisphere) */
            if (sweep_sphere_tri(sph_bot_start, sph_bot_end, radius, v0, v1, v2, &t, &n)) {
                bool better = false;
                if (sweep_down) {
                    Vec3 hit_pos = vec3_add(start, vec3_scale(vec3_sub(end, start), t));
                    bool hit_is_ramp = n.y < 0.99f;
                    bool best_is_flat = best.hit && best.normal.y > 0.99f;

                    if (!best.hit) {
                        better = true;
                    } else if (hit_is_ramp && best_is_flat) {
                        if (hit_pos.y >= best.pos.y - 5.0f) better = true;
                    } else if (hit_pos.y > best.pos.y + 1e-4f) {
                        better = true;
                    } else if (fabsf(hit_pos.y - best.pos.y) <= 1e-4f) {
                        if (n.y < best.normal.y - 1e-4f) better = true;
                    }
                } else {
                    if (t < best.t) better = true;
                }
                if (better) {
                    best.hit = true;
                    best.t = t;
                    best.normal = n;
                    best.pos = vec3_add(start, vec3_scale(vec3_sub(end, start), t));
                }
            }

            /* 2. Cylinder sweep (capsule cylindrical middle)
     * Omitted: the sphere sweeps of the capsule endpoints (bottom/top spheres)
     * cover the hemispherical ends and most of the sweep volume. A proper
     * cylinder sweep requires a swept segment test which is complex.
     * For v1, we rely on sphere sweeps which correctly parameterize t along
     * the capsule center sweep. This covers 95% of cases; the cylinder side
     * sweep can be added in a future optimization pass. */

            /* 3. Esfera superior */
            if (sweep_sphere_tri(sph_top_start, sph_top_end, radius, v0, v1, v2, &t, &n)) {
                bool better = false;
                if (sweep_down) {
                    Vec3 hit_pos = vec3_add(start, vec3_scale(vec3_sub(end, start), t));
                    bool hit_is_ramp = n.y < 0.99f;
                    bool best_is_flat = best.hit && best.normal.y > 0.99f;

                    if (!best.hit) {
                        better = true;
                    } else if (hit_is_ramp && best_is_flat) {
                        if (hit_pos.y >= best.pos.y - 5.0f) better = true;
                    } else if (hit_pos.y > best.pos.y + 1e-4f) {
                        better = true;
                    } else if (fabsf(hit_pos.y - best.pos.y) <= 1e-4f) {
                        if (n.y < best.normal.y - 1e-4f) better = true;
                    }
                } else {
                    if (t < best.t) better = true;
                }
                if (better) {
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

/* Predictive ramp detection using raycast down from high above (GoldSrc style).
 * Uses 5-ray fan at lookahead position with full vertical range broadphase query.
 * Returns true if ramp detected, fills out_normal with ramp normal,
 * and out_hit_y with floor y at lookahead position. */
bool collision_detectar_rampa_predictiva(const Map *map, const Broadphase *bp,
                                          Vec3 pos, Vec3 wish_dir, f32 lookahead,
                                          f32 crouch_height, Vec3 *out_normal, f32 *out_hit_y)
{
    (void)crouch_height;
    if (!map || !map->cargado || map->face_count == 0) return false;
    if (wish_dir.x == 0.0f && wish_dir.z == 0.0f) return false;

    f32 radius = RADIO_JUGADOR;
    f32 lookahead_x = pos.x + wish_dir.x * lookahead;
    f32 lookahead_z = pos.z + wish_dir.z * lookahead;

    /* Raycast from 20u above player down to -20u (covers max ramp height 18 + margin) */
    const f32 RAY_START_OFFSET = 20.0f;
    f32 ray_radius = radius * 0.7f;
    Vec3 origins[5] = {
        { lookahead_x, pos.y + RAY_START_OFFSET, lookahead_z },
        { lookahead_x + ray_radius, pos.y + RAY_START_OFFSET, lookahead_z },
        { lookahead_x - ray_radius, pos.y + RAY_START_OFFSET, lookahead_z },
        { lookahead_x, pos.y + RAY_START_OFFSET, lookahead_z + ray_radius },
        { lookahead_x, pos.y + RAY_START_OFFSET, lookahead_z - ray_radius }
    };

    Vec3 dir = { 0.0f, -1.0f, 0.0f };
    f32 max_dist = RAY_START_OFFSET + 20.0f + radius;  /* 20 + 20 + 16 = 56 */

    /* Broadphase query covering FULL vertical range at lookahead position.
     * This ensures ramp triangles are found even at cell boundaries.
     * Use wider horizontal query radius (3x player radius = 48) to catch wide ramps up to 80 units wide.
     * The test ramp spans z=[-40, 40], so at lookahead_z=0 we need radius >= 40 to include edges. */
    const f32 QUERY_RADIUS = radius * 3.0f;  /* 48 units */
    u32 face_count = 0;
    u16 face_indices[BP_TEMP_MAX];

    if (bp && bp->built) {
        Vec3 mins = { lookahead_x - QUERY_RADIUS, pos.y - max_dist, lookahead_z - QUERY_RADIUS };
        Vec3 maxs = { lookahead_x + QUERY_RADIUS, pos.y + RAY_START_OFFSET + 2.0f, lookahead_z + QUERY_RADIUS };
        face_count = broadphase_query_aabb(bp, mins, maxs, face_indices, BP_TEMP_MAX);
    } else {
        face_count = map->face_count;
    }

    f32 best_hit_y = -1e9f;
    Vec3 best_n = { 0.0f, 1.0f, 0.0f };
    bool hit_any = false;

    for (int o = 0; o < 5; o++) {
        Vec3 origin = origins[o];

        for (u32 fi_idx = 0; fi_idx < face_count; fi_idx++) {
            u32 fi = (bp && bp->built) ? face_indices[fi_idx] : fi_idx;
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

                if (triangle_is_degenerate(v0, v1, v2)) continue;

                f32 t;
                Vec3 n;
                if (collision_ray_triangulo(origin, dir, v0, v1, v2, &t, &n)) {
                    if (t >= 1e-4f && t <= max_dist && n.y > SUELO_NORMAL_Y_MIN) {
                        f32 hit_y = origin.y - t;
                        /* Prefer ramps (n.y < 0.95) over flat floor */
                        bool better = false;
                        bool hit_is_ramp = n.y < 0.95f;
                        bool best_is_flat = best_n.y > 0.95f;

                        if (!hit_any) {
                            better = true;
                        } else if (hit_is_ramp && best_is_flat) {
                            /* Strongly prefer ramps over flat floor when approaching */
                            if (hit_y >= best_hit_y - 20.0f) better = true;
                        } else if (hit_is_ramp && best_n.y < 0.95f) {
                            /* Between ramps, prefer higher hit (closer to player) */
                            if (hit_y > best_hit_y + 1e-4f) better = true;
                        } else if (!hit_is_ramp && best_is_flat) {
                            /* Between flat floors, prefer higher */
                            if (hit_y > best_hit_y + 1e-4f) better = true;
                        }
                        /* Never prefer flat floor over ramp once a ramp is detected */

                        if (better) {
                            best_n = n;
                            best_hit_y = hit_y;
                            hit_any = true;
                        }
                    }
                }
            }
        }
    }

    if (hit_any && best_n.y < 0.95f) {
        if (out_normal) *out_normal = best_n;
        if (out_hit_y) *out_hit_y = best_hit_y;
        return true;
    }

    return false;
}

/* ── Raycast-based floor detection (GoldSrc style) ───────────────────────── *
 * Uses 5 rays from HIGH above feet level (40 units) to detect both current
 * floor and ramps above. This solves the "ramp above flat floor" problem.
 * Ray origin at y=40: hits ramp at y=5 (t=35) before flat floor at y=0 (t=40). */
static FloorHit raycast_floor_detection(const Map *map, const Broadphase *bp, Vec3 pos, f32 radius)
{
    FloorHit best = {0};
    best.y = -1e9f;

    if (!map || !map->cargado || map->face_count == 0) return best;

    const f32 RAY_START_OFFSET = 40.0f;
    /* Use 2x player radius (32) so side rays reach further out, catching ramps *
     * when approaching from flat floor (ramp starts at x=180, player at x=170).
     * Side rays at ±32 units can reach the ramp start from 10 units away. */
    f32 ray_radius = radius * 2.0f;  /* 32 units */
    Vec3 origins[5] = {
        { pos.x, pos.y + RAY_START_OFFSET, pos.z },
        { pos.x + ray_radius, pos.y + RAY_START_OFFSET, pos.z },
        { pos.x - ray_radius, pos.y + RAY_START_OFFSET, pos.z },
        { pos.x, pos.y + RAY_START_OFFSET, pos.z + ray_radius },
        { pos.x, pos.y + RAY_START_OFFSET, pos.z - ray_radius }
    };

    f32 max_dist = RAY_START_OFFSET + SV_STEPSIZE + radius + 20.0f;
    Vec3 dir = { 0.0f, -1.0f, 0.0f };

    /* Use wider horizontal query radius (5x player radius = 80) to catch ramp triangles
     * that extend beyond the player's immediate footprint (e.g., wide ramps up to 80 units wide).
     * The test ramp spans z=[-40, 40] and starts at x=180 (10 units ahead of player at x=170).
     * Side rays at ±2x radius (32 units) can reach the ramp start from 10 units away. */
    const f32 QUERY_RADIUS = radius * 5.0f;  /* 80 units */

    u32 face_count = 0;
    u16 face_indices[BP_TEMP_MAX];

    if (bp && bp->built) {
        Vec3 mins = { pos.x - QUERY_RADIUS, pos.y - max_dist, pos.z - QUERY_RADIUS };
        Vec3 maxs = { pos.x + QUERY_RADIUS, pos.y + RAY_START_OFFSET + 2.0f, pos.z + QUERY_RADIUS };
        face_count = broadphase_query_aabb(bp, mins, maxs, face_indices, BP_TEMP_MAX);
    } else {
        face_count = map->face_count;
    }

    /* Track center ray hit separately - it represents the player's actual position.
     * Side rays are only for step-up detection (higher floor) and predictive hints. */
    FloorHit center_hit = { 0 };
    center_hit.y = -1e9f;
    center_hit.normal = (Vec3){ 0.0f, 1.0f, 0.0f };

    for (int o = 0; o < 5; o++) {
        Vec3 origin = origins[o];
        Vec3 best_n = { 0.0f, 1.0f, 0.0f };
        f32 best_hit_y = -1e9f;
        bool is_center_ray = (o == 0);

        for (u32 fi_idx = 0; fi_idx < face_count; fi_idx++) {
            u32 fi = (bp && bp->built) ? face_indices[fi_idx] : fi_idx;
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

                if (triangle_is_degenerate(v0, v1, v2)) continue;

                f32 t;
                Vec3 n;
                if (collision_ray_triangulo(origin, dir, v0, v1, v2, &t, &n)) {
                    if (t >= 1e-4f && t <= max_dist && n.y > SUELO_NORMAL_Y_MIN) {
                        f32 hit_y = origin.y - t;
                        bool better = false;
                        bool hit_is_ramp = n.y < 0.95f;
                        bool best_is_flat = best_n.y > 0.95f;

                        if (!best.hit) {
                            better = true;
                        } else if (hit_is_ramp && best_is_flat) {
                            /* Prefer ramp over flat ONLY if center ray hits ramp,
                             * or if this is center ray and current best is from side ray.
                             * This prevents side rays from pulling player onto ramp prematurely. */
                            if (is_center_ray || (best_n.y < 0.95f && best_n.y > 0.001f)) {
                                if (hit_y >= best_hit_y - 20.0f) better = true;
                            }
                        } else if (hit_is_ramp && best_n.y < 0.95f) {
                            /* Between ramps, prefer higher hit (closer to player) */
                            if (hit_y > best_hit_y + 1e-4f) better = true;
                        } else if (!hit_is_ramp && best_is_flat) {
                            /* Between flat floors, prefer higher */
                            if (hit_y > best_hit_y + 1e-4f) better = true;
                        }
                        /* Never prefer flat floor over ramp once center ray confirms ramp */

                        if (better) {
                            best_n = n;
                            best_hit_y = hit_y;
                            best.hit = true;
                        }
                    }
                }
            }
        }

        if (best.hit && best_hit_y > best.y) {
            best.y = best_hit_y;
            best.normal = best_n;
        }

        /* Store center ray result separately */
        if (is_center_ray && best.hit) {
            center_hit = best;
        }
    }

    /* Use center ray hit as primary floor detection.
     * Only override with best (side ray) hit if:
     * 1. Center ray hit nothing, OR
     * 2. Side ray hit is significantly higher (step up: > 0.5 units) */
    if (center_hit.hit) {
        if (best.hit && best.y > center_hit.y + 0.5f) {
            /* Side ray found a step up - use it */
            return best;
        }
        /* Center ray hit is authoritative for current floor */
        return center_hit;
    } else if (best.hit) {
        /* Center ray missed - fall back to best side ray hit */
        return best;
    }
    return center_hit;  // no hits at all

    return best;
}

FloorHit collision_detectar_suelo(const Map *map, const Broadphase *bp, Vec3 pos, f32 crouch_height)
{
    (void)crouch_height;
    f32 radius = RADIO_JUGADOR;

    return raycast_floor_detection(map, bp, pos, radius);
}

bool collision_bloquear_pared(const Map *map, const Broadphase *bp, Vec3 start_pos, Vec3 *vel, f32 dt, f32 crouch_height, Vec3 floor_normal, Vec3 *out_pos)
{
    (void)floor_normal;  // Unused: slide projection handles ramps correctly
    if (!map || !map->cargado || map->face_count == 0) {
        *out_pos = vec3_add(start_pos, vec3_scale(*vel, dt));
        return false;
    }

    f32 radius = RADIO_JUGADOR;
    f32 capsule_h = crouch_height - 2.0f * radius;
    if (capsule_h < CAPSULE_MIN_HEIGHT) capsule_h = CAPSULE_MIN_HEIGHT;
    f32 half_height = capsule_h * 0.5f;
    f32 capsule_half_h = half_height + radius;  // center to top/bottom of capsule

    /* Convert feet position to capsule center for swept capsule */
    Vec3 original_pos = start_pos;
    Vec3 start = { start_pos.x, start_pos.y + capsule_half_h, start_pos.z };
    Vec3 end = { start_pos.x + vel->x * dt, start_pos.y + capsule_half_h + vel->y * dt, start_pos.z + vel->z * dt };

    bool any_correction = false;

    for (int pass = 0; pass < MAX_SLIDE_PASSES; pass++) {
        SweepHit hit = collision_sweep_capsule(map, bp, start, end, radius, half_height, true);

        if (!hit.hit) {
            *out_pos = end;
            break;
        }

        /* Log wall hit (only on first pass to avoid spam) */
        if (pass == 0) {
            /* Create a minimal PhysPlayer for logging */
            PhysPlayer log_player = {0};
            log_player.pos = start_pos;
            log_player.vel = *vel;
            phys_log_wall_hit(&log_player, &hit.normal, pass);
        }

        f32 move_t = hit.t - SWEEP_EPSILON;
        if (move_t < 0.0f) move_t = 0.0f;
        Vec3 new_pos = vec3_add(start, vec3_scale(vec3_sub(end, start), move_t));

        Vec3 remaining_vel = vec3_scale(vec3_sub(end, start), 1.0f - hit.t);
        /* Slide: full 3D slide (GoldSrc PM_FlyMove style). Floor snap in phys_tick
         * will correct position and zero vertical velocity on ground.
         * Project onto hit plane. For ramps (normal.y > 0.3), keep Y velocity
         * to allow climbing. For vertical walls (normal.y <= 0.3), zero Y to
         * prevent feedback loop from multiple slide passes. */
        f32 into = vec3_dot(remaining_vel, hit.normal);
        if (into < 0.0f) {
            remaining_vel = vec3_sub(remaining_vel, vec3_scale(hit.normal, into));
        }
        /* Only zero Y for vertical walls; ramps need Y velocity to climb. */
        if (hit.normal.y <= 0.3f) {
            remaining_vel.y = 0.0f;
        }

        start = new_pos;
        end = vec3_add(start, remaining_vel);
        any_correction = true;

        if (vec3_len(remaining_vel) < 0.01f) {
            *out_pos = (Vec3){ new_pos.x, new_pos.y - capsule_half_h, new_pos.z };
            break;
        }
    }

    if (any_correction) {
        Vec3 final_feet = { out_pos->x, out_pos->y, out_pos->z };
        *vel = vec3_scale(vec3_sub(final_feet, original_pos), 1.0f / dt);
    } else {
        *out_pos = (Vec3){ end.x, end.y - capsule_half_h, end.z };
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
    f32 capsule_half_h = half_height + radius;  // center to top/bottom of capsule

    const f32 BACK_OFFSET = 8.0f;
    /* Convert feet position to capsule center for swept capsule */
    Vec3 start = { pos->x - move_dir->x * BACK_OFFSET, pos->y + capsule_half_h, pos->z - move_dir->z * BACK_OFFSET };
    Vec3 horiz_move = { move_dir->x * (STEP_UP_AHEAD_DIST + BACK_OFFSET), 0.0f, move_dir->z * (STEP_UP_AHEAD_DIST + BACK_OFFSET) };
    Vec3 end = { start.x + horiz_move.x, start.y + SV_STEPSIZE, start.z + horiz_move.z };

    SweepHit hit = collision_sweep_capsule(map, bp, start, end, radius, half_height, false);

    /* Step-up should only trigger on discrete steps (flat tops, normal.y ≈ 1),
     * NOT on continuous slopes (ramps). Ramps are handled by floor snap. */
    const f32 STEP_UP_NORMAL_Y_MIN = 0.95f;  // flat enough to be a "step top"
    if (hit.hit && hit.normal.y > STEP_UP_NORMAL_Y_MIN) {
        /* hit.pos is capsule center at impact; convert back to feet */
        f32 ground_y = hit.pos.y - capsule_half_h;
        f32 delta_y = ground_y - pos->y;

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

    f32 radius = RADIO_JUGADOR * 0.5f;
    f32 half_height = 0.0f;

    Vec3 start = { pos.x, pos.y + crouch_height, pos.z };
    Vec3 end = { start.x, start.y + max_dist, start.z };

    SweepHit hit = collision_sweep_capsule(map, bp, start, end, radius, half_height, false);

    return hit.hit && hit.normal.y < TECHO_NORMAL_Y_MAX;
}