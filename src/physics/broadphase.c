// Copyright (c) 2026 OpenStrike Project
// broadphase.c is part of OpenStrike.
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

#include "broadphase.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ── Helpers internos ─────────────────────────────────────────────────────── */

static inline int bp_clamp_i(int v, int min, int max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

static inline void bp_world_to_cell(const Broadphase *bp, f32 x, f32 y, f32 z,
                                     int *cx, int *cy, int *cz)
{
    *cx = bp_clamp_i((int)((x - bp->world_min.x) / BP_CELL_SIZE_X), 0, BP_GRID_X - 1);
    *cy = bp_clamp_i((int)((y - bp->world_min.y) / BP_CELL_SIZE_Y), 0, BP_GRID_Y - 1);
    *cz = bp_clamp_i((int)((z - bp->world_min.z) / BP_CELL_SIZE_Z), 0, BP_GRID_Z - 1);
}

static inline u32 bp_cell_index(int cx, int cy, int cz)
{
    return (u32)((cx * BP_GRID_Y + cy) * BP_GRID_Z + cz);
}

static void bp_add_face_to_cell(Broadphase *bp, int cx, int cy, int cz, u16 face_idx)
{
    u32 idx = bp_cell_index(cx, cy, cz);
    BroadphaseCell *cell = &bp->cells[idx];
    if (cell->count < BP_MAX_FACES_PER_CELL) {
        /* Evitar duplicados en la misma celda */
        for (u16 i = 0; i < cell->count; i++) {
            if (cell->face_indices[i] == face_idx) return;
        }
        cell->face_indices[cell->count++] = face_idx;
    }
    /* Si se llena, silenciosamente descartamos el exceso.
     * En la práctica no ocurre con BP_MAX_FACES_PER_CELL=64. */
}

/* AABB de un triángulo */
static void bp_triangle_aabb(const MapVertex *va, const MapVertex *vb, const MapVertex *vc,
                              Vec3 *mins, Vec3 *maxs)
{
    mins->x = fminf(fminf(va->pos[0], vb->pos[0]), vc->pos[0]);
    mins->y = fminf(fminf(va->pos[1], vb->pos[1]), vc->pos[1]);
    mins->z = fminf(fminf(va->pos[2], vb->pos[2]), vc->pos[2]);
    maxs->x = fmaxf(fmaxf(va->pos[0], vb->pos[0]), vc->pos[0]);
    maxs->y = fmaxf(fmaxf(va->pos[1], vb->pos[1]), vc->pos[1]);
    maxs->z = fmaxf(fmaxf(va->pos[2], vb->pos[2]), vc->pos[2]);
}

/* ── API pública ──────────────────────────────────────────────────────────── */

void broadphase_build(Broadphase *bp, const Map *map)
{
    memset(bp, 0, sizeof(*bp));
    bp->world_min = (Vec3){ BP_WORLD_MIN_X, BP_WORLD_MIN_Y, BP_WORLD_MIN_Z };
    bp->world_max = (Vec3){ BP_WORLD_MAX_X, BP_WORLD_MAX_Y, BP_WORLD_MAX_Z };

    if (!map || !map->cargado || map->face_count == 0) {
        bp->built = true;
        return;
    }

    for (i32 fi = 0; fi < map->face_count; fi++) {
        const MapFace *face = &map->faces[fi];
        u32 base = face->vertex_start;
        u32 count = face->vertex_count;

        for (u32 ti = 0; ti + 2 < count; ti += 3) {
            const MapVertex *va = &map->vertices[base + ti + 0];
            const MapVertex *vb = &map->vertices[base + ti + 1];
            const MapVertex *vc = &map->vertices[base + ti + 2];

            Vec3 mins, maxs;
            bp_triangle_aabb(va, vb, vc, &mins, &maxs);

            int min_cx, min_cy, min_cz;
            int max_cx, max_cy, max_cz;
            bp_world_to_cell(bp, mins.x, mins.y, mins.z, &min_cx, &min_cy, &min_cz);
            bp_world_to_cell(bp, maxs.x, maxs.y, maxs.z, &max_cx, &max_cy, &max_cz);

            for (int cx = min_cx; cx <= max_cx; cx++) {
                for (int cy = min_cy; cy <= max_cy; cy++) {
                    for (int cz = min_cz; cz <= max_cz; cz++) {
                        bp_add_face_to_cell(bp, cx, cy, cz, (u16)fi);
                    }
                }
            }
        }
    }

    bp->built = true;
}

u32 broadphase_query_aabb(const Broadphase *bp, Vec3 mins, Vec3 maxs,
                           u16 *out_indices, u32 max_out)
{
    if (!bp->built || max_out == 0) return 0;

    int min_cx, min_cy, min_cz;
    int max_cx, max_cy, max_cz;
    bp_world_to_cell((Broadphase*)bp, mins.x, mins.y, mins.z, &min_cx, &min_cy, &min_cz);
    bp_world_to_cell((Broadphase*)bp, maxs.x, maxs.y, maxs.z, &max_cx, &max_cy, &max_cz);

    u32 written = 0;
    u16 seen[BP_MAX_FACES_PER_CELL * 8];  /* buffer temporal para dedup */
    u32 seen_count = 0;

    for (int cx = min_cx; cx <= max_cx; cx++) {
        for (int cy = min_cy; cy <= max_cy; cy++) {
            for (int cz = min_cz; cz <= max_cz; cz++) {
                u32 idx = bp_cell_index(cx, cy, cz);
                const BroadphaseCell *cell = &bp->cells[idx];

                for (u16 i = 0; i < cell->count; i++) {
                    u16 fi = cell->face_indices[i];
                    bool dup = false;
                    for (u32 j = 0; j < seen_count; j++) {
                        if (seen[j] == fi) { dup = true; break; }
                    }
                    if (!dup && written < max_out) {
                        seen[seen_count++] = fi;
                        out_indices[written++] = fi;
                    }
                }
            }
        }
    }

    return written;
}

/* DDA 3D (Amanatides-Woo) para recorrido de grid por segmento */
u32 broadphase_query_segment(const Broadphase *bp, Vec3 a, Vec3 b,
                              u16 *out_indices, u32 max_out)
{
    if (!bp->built || max_out == 0) return 0;

    Vec3 dir = { b.x - a.x, b.y - a.y, b.z - a.z };
    f32 len = sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    if (len < 1e-6f) {
        return broadphase_query_point(bp, a, out_indices, max_out);
    }
    dir.x /= len; dir.y /= len; dir.z /= len;

    int cx, cy, cz;
    bp_world_to_cell((Broadphase*)bp, a.x, a.y, a.z, &cx, &cy, &cz);
    int end_cx, end_cy, end_cz;
    bp_world_to_cell((Broadphase*)bp, b.x, b.y, b.z, &end_cx, &end_cy, &end_cz);

    int step_x = (dir.x > 0) ? 1 : (dir.x < 0) ? -1 : 0;
    int step_y = (dir.y > 0) ? 1 : (dir.y < 0) ? -1 : 0;
    int step_z = (dir.z > 0) ? 1 : (dir.z < 0) ? -1 : 0;

    f32 t_max_x = (step_x != 0) ? 
        ((step_x > 0) ? 
            ((cx + 1) * BP_CELL_SIZE_X + bp->world_min.x - a.x) / dir.x :
            (a.x - cx * BP_CELL_SIZE_X - bp->world_min.x) / -dir.x)
        : 1e30f;
    f32 t_max_y = (step_y != 0) ?
        ((step_y > 0) ?
            ((cy + 1) * BP_CELL_SIZE_Y + bp->world_min.y - a.y) / dir.y :
            (a.y - cy * BP_CELL_SIZE_Y - bp->world_min.y) / -dir.y)
        : 1e30f;
    f32 t_max_z = (step_z != 0) ?
        ((step_z > 0) ?
            ((cz + 1) * BP_CELL_SIZE_Z + bp->world_min.z - a.z) / dir.z :
            (a.z - cz * BP_CELL_SIZE_Z - bp->world_min.z) / -dir.z)
        : 1e30f;

    f32 t_delta_x = (step_x != 0) ? BP_CELL_SIZE_X / fabsf(dir.x) : 1e30f;
    f32 t_delta_y = (step_y != 0) ? BP_CELL_SIZE_Y / fabsf(dir.y) : 1e30f;
    f32 t_delta_z = (step_z != 0) ? BP_CELL_SIZE_Z / fabsf(dir.z) : 1e30f;

    u32 written = 0;
    u16 seen[BP_MAX_FACES_PER_CELL * 16];
    u32 seen_count = 0;

    while (true) {
        if (cx >= 0 && cx < BP_GRID_X && cy >= 0 && cy < BP_GRID_Y && cz >= 0 && cz < BP_GRID_Z) {
            u32 idx = bp_cell_index(cx, cy, cz);
            const BroadphaseCell *cell = &bp->cells[idx];
            for (u16 i = 0; i < cell->count; i++) {
                u16 fi = cell->face_indices[i];
                bool dup = false;
                for (u32 j = 0; j < seen_count; j++) {
                    if (seen[j] == fi) { dup = true; break; }
                }
                if (!dup && written < max_out) {
                    seen[seen_count++] = fi;
                    out_indices[written++] = fi;
                }
            }
        }

        if (cx == end_cx && cy == end_cy && cz == end_cz) break;

        if (t_max_x < t_max_y) {
            if (t_max_x < t_max_z) {
                cx += step_x;
                t_max_x += t_delta_x;
            } else {
                cz += step_z;
                t_max_z += t_delta_z;
            }
        } else {
            if (t_max_y < t_max_z) {
                cy += step_y;
                t_max_y += t_delta_y;
            } else {
                cz += step_z;
                t_max_z += t_delta_z;
            }
        }

        if (cx < 0 || cx >= BP_GRID_X || cy < 0 || cy >= BP_GRID_Y || cz < 0 || cz >= BP_GRID_Z) break;
    }

    return written;
}

u32 broadphase_query_point(const Broadphase *bp, Vec3 pos,
                            u16 *out_indices, u32 max_out)
{
    if (!bp->built || max_out == 0) return 0;

    int cx, cy, cz;
    bp_world_to_cell((Broadphase*)bp, pos.x, pos.y, pos.z, &cx, &cy, &cz);

    if (cx < 0 || cx >= BP_GRID_X || cy < 0 || cy >= BP_GRID_Y || cz < 0 || cz >= BP_GRID_Z) {
        return 0;
    }

    u32 idx = bp_cell_index(cx, cy, cz);
    const BroadphaseCell *cell = &bp->cells[idx];
    u32 count = cell->count < max_out ? cell->count : max_out;
    for (u32 i = 0; i < count; i++) {
        out_indices[i] = cell->face_indices[i];
    }
    return count;
}

void broadphase_debug_stats(const Broadphase *bp)
{
    if (!bp->built) {
        printf("[broadphase] no construido\n");
        return;
    }

    u32 total_faces = 0;
    u32 max_faces = 0;
    u32 empty_cells = 0;
    u32 used_cells = 0;

    for (u32 i = 0; i < BP_TOTAL_CELLS; i++) {
        u32 c = bp->cells[i].count;
        if (c == 0) empty_cells++;
        else {
            used_cells++;
            total_faces += c;
            if (c > max_faces) max_faces = c;
        }
    }

    printf("[broadphase] celdas: %u totales, %u usadas, %u vacías\n",
           BP_TOTAL_CELLS, used_cells, empty_cells);
    printf("[broadphase] caras registradas (con duplicados entre celdas): %u\n", total_faces);
    printf("[broadphase] máx caras/celda: %u (límite %u)\n", max_faces, BP_MAX_FACES_PER_CELL);
    if (used_cells) {
        printf("[broadphase] promedio caras/celda usada: %.2f\n",
               (f32)total_faces / (f32)used_cells);
    }
}