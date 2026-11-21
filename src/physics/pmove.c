// Copyright (c) 2026 OpenStrike Project
// pmove.c is part of OpenStrike.
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

#include "pmove.h"
#include <math.h>
#include <string.h>

/* ── Funciones internas ───────────────────────────────────────────────────── */

/* Aplica fricción sobre el plano XZ cuando el jugador está en suelo. */
static void aplicar_friccion(PhysPlayer *p, f32 dt)
{
    f32 speed = sqrtf(p->vel.x * p->vel.x + p->vel.z * p->vel.z);
    if (speed < 0.1f) { p->vel.x = 0.0f; p->vel.z = 0.0f; return; }

    f32 control   = (speed < SV_STOPSPEED) ? SV_STOPSPEED : speed;
    f32 drop      = control * SV_FRICTION * dt;
    f32 new_speed = speed - drop;
    if (new_speed < 0.0f) { new_speed = 0.0f; }

    f32 factor = new_speed / speed;
    p->vel.x *= factor;
    p->vel.z *= factor;
}

/* Acelera en wish_dir hasta wish_speed con el coeficiente accel dado.
 * Usada tanto para suelo como para aire. */
static void acelerar(PhysPlayer *p, Vec3 wish_dir,
                     f32 wish_speed, f32 accel, f32 dt)
{
    f32 current   = vec3_dot(p->vel, wish_dir);
    f32 add_speed = wish_speed - current;
    if (add_speed <= 0.0f) { return; }

    f32 accel_speed = accel * wish_speed * dt;
    if (accel_speed > add_speed) { accel_speed = add_speed; }

    p->vel = vec3_add(p->vel, vec3_scale(wish_dir, accel_speed));
}

/* Cap XZ a BHOP_MAX_SPEED — el bhop mantiene velocidad pero no se acumula.
 * Se aplica en CADA tick de aire (no solo en el salto): el strafe aéreo
 * no puede superar el cap. */
static void cap_velocidad_bhop(PhysPlayer *p)
{
    f32 xz = sqrtf(p->vel.x * p->vel.x + p->vel.z * p->vel.z);
    if (xz > BHOP_MAX_SPEED) {
        f32 scale = BHOP_MAX_SPEED / xz;
        p->vel.x *= scale;
        p->vel.z *= scale;
    }
}

/* Proyecta un vector 3D sobre un plano definido por su normal.
 * Retorna la componente tangencial (vector - normal * dot(vector, normal)). */
static inline Vec3 vec3_project_on_plane(Vec3 v, Vec3 n)
{
    return vec3_sub(v, vec3_scale(n, vec3_dot(v, n)));
}

/* Calcula la componente tangencial de la gravedad en una pendiente (vector tangent al plano). *
 * gravity_vec = (0, -SV_GRAVITY, 0)                                                          *
 * tangent_gravity = gravity_vec - normal * dot(gravity_vec, normal)                          *
 * Esta es la fórmula correcta (GoldSrc), no la aproximación anterior.                        */
static inline Vec3 gravity_tangent(Vec3 normal)
{
    /* gravity = (0, -SV_GRAVITY, 0) */
    /* dot(gravity, normal) = -SV_GRAVITY * normal.y */
    /* tangent = gravity - normal * dot = (0, -SV_GRAVITY, 0) - normal * (-SV_GRAVITY * normal.y) */
    /*        = (0, -SV_GRAVITY, 0) + normal * (SV_GRAVITY * normal.y) */
    f32 scale = SV_GRAVITY * normal.y;
    Vec3 t = {
        normal.x * scale,
        normal.y * scale - SV_GRAVITY,
        normal.z * scale
    };
    return t;
}

/* ── Agachado (5c): transición suave ─────────────────────────────────────── */

static void phys_crouch_init(PhysPlayer *p)
{
    p->crouch_height    = CROUCH_HEIGHT_STAND;
    p->crouch_target    = CROUCH_HEIGHT_STAND;
    p->crouch_transition = 0;
}

static bool phys_check_ceiling(const Map *map, const Broadphase *bp, const PhysPlayer *p)
{
    if (!map || !map->cargado || map->face_count == 0) { return false; }

    f32 max_dist = (CROUCH_HEIGHT_STAND - p->crouch_height) + CEILING_CHECK_DIST;
    return collision_check_ceiling(map, bp, p->pos, p->crouch_height, max_dist);
}

static void phys_crouch_update(PhysPlayer *p, const PhysInput *input, const Map *map, const Broadphase *bp)
{
    bool quiere_agachado = input->agacharse;

    f32 nuevo_target = quiere_agachado ? CROUCH_HEIGHT_DUCK : CROUCH_HEIGHT_STAND;

    if (nuevo_target != p->crouch_target) {
        if (nuevo_target == CROUCH_HEIGHT_STAND) {
            if (phys_check_ceiling(map, bp, p)) {
                nuevo_target = CROUCH_HEIGHT_DUCK;
                quiere_agachado = true;
            }
        }

        if (nuevo_target != p->crouch_target) {
            p->crouch_target = nuevo_target;
            p->crouch_transition = CROUCH_TRANSITION_TICKS;
        }
    }

    if (p->crouch_transition > 0) {
        f32 t = 1.0f - (f32)p->crouch_transition / (f32)CROUCH_TRANSITION_TICKS;
        p->crouch_height = p->crouch_height + (p->crouch_target - p->crouch_height) * t;
        p->crouch_transition--;
        if (p->crouch_transition == 0) {
            p->crouch_height = p->crouch_target;
        }
    } else {
        p->crouch_height = p->crouch_target;
    }

    p->agachado = quiere_agachado;
}

/* ── API pública ──────────────────────────────────────────────────────────── */

void phys_player_init(PhysPlayer *p, Vec3 spawn_pos)
{
    memset(p, 0, sizeof(*p));
    p->pos = spawn_pos;
    phys_crouch_init(p);
}

void phys_tick(PhysPlayer *p, PhysInput input, const Map *map, f32 dt, const Broadphase *bp)
{
    p->caminando = input.caminar;

    /* ── Actualizar agachado con transición suave y comprobación de techo ── */
    phys_crouch_update(p, &input, map, bp);

    /* ── Jump buffer: si se pulsa salto en aire, guardar buffer ─────────── */
    if (input.saltar && !p->en_suelo) {
        p->jump_buffer_ticks = JUMP_BUFFER_TICKS;
    } else if (p->jump_buffer_ticks > 0) {
        p->jump_buffer_ticks--;
    }

    /* ── Variable jump height (jump cut): si suelta SPACE subiendo, cortar  *
     * la velocidad vertical a JUMP_CUT_VEL.                                */
    if (!input.saltar_held && p->vel.y > JUMP_CUT_VEL) {
        p->vel.y = JUMP_CUT_VEL;
    }

    bool puede_saltar = p->en_suelo || p->coyote_ticks > 0;
    bool debe_saltar  = input.saltar || p->jump_buffer_ticks > 0;

    /* ── Detectar suelo en posición ACTUAL (antes de moverse) ───────────── */
    FloorHit fh_curr = {0};
    if (p->en_suelo) {
        fh_curr = collision_detectar_suelo(map, bp, p->pos, p->crouch_height);
        if (!fh_curr.hit) {
            fh_curr.hit = true;
            fh_curr.y = p->pos.y;
            fh_curr.normal = (Vec3){ 0.0f, 1.0f, 0.0f };
        }
    }

    bool on_ramp_curr = p->en_suelo && fh_curr.hit && fh_curr.normal.y < 0.95f && fh_curr.normal.y > 0.001f;
    Vec3 floor_normal_curr = on_ramp_curr ? fh_curr.normal : (Vec3){ 0.0f, 1.0f, 0.0f };

    /* ── Determinar wish_dir original (horizontal) y wish_speed ─────────── */
    Vec3 wish_dir_input = input.wish_dir;  // horizontal only
    bool tiene_wish = vec3_len(wish_dir_input) > 0.001f;

    f32 wish_speed = 0.0f;
    if (p->en_suelo) {
        wish_speed = p->agachado  ? SPEED_DUCK :
                     p->caminando ? SPEED_WALK :
                                    SPEED_RUN;
    } else {
        wish_speed = AIR_WISH_SPEED;
    }

    /* ── Predictive: detectar rampa inminente (flat→rampa) ──────────────── *
     * Usar wish_dir_input (horizontal) para lookahead.
     * USA swept capsule (más fiable que raycast para detectar rampas).     */
    bool on_ramp_predictive = false;
    Vec3 ramp_normal_predictive = floor_normal_curr;
    Vec3 next_pos = p->pos;

    if (p->en_suelo && !on_ramp_curr && tiene_wish) {
        f32 lookahead = wish_speed * dt * 4.0f;  // ~4 ticks ahead
        next_pos = (Vec3){ p->pos.x + wish_dir_input.x * lookahead, p->pos.y, p->pos.z + wish_dir_input.z * lookahead };
        if (collision_detectar_rampa_predictiva(map, bp, p->pos, wish_dir_input, lookahead, p->crouch_height, &ramp_normal_predictive)) {
            on_ramp_predictive = true;
        }
    }

    /* ── Wish_dir final: proyectar a plano de rampa (actual o predictiva) ─ */
    Vec3 ramp_normal_for_wish = on_ramp_curr ? floor_normal_curr : (on_ramp_predictive ? ramp_normal_predictive : (Vec3){0,1,0});
    bool use_ramp_wish = on_ramp_curr || on_ramp_predictive;
    Vec3 wish_dir = wish_dir_input;
    if (use_ramp_wish && tiene_wish) {
        wish_dir = vec3_project_on_plane(wish_dir_input, ramp_normal_for_wish);
        /* Mantener componente Y para subidas */
        f32 len = vec3_len(wish_dir);
        if (len > 0.001f) {
            wish_dir = vec3_scale(wish_dir, 1.0f / len);
        } else {
            tiene_wish = false;
        }
    }

    /* ── Manejo de salto ────────────────────────────────────────────────── */
    if (p->en_suelo) {
        if (p->ticks_en_suelo > BHOP_GRACE_TICKS) {
            aplicar_friccion(p, dt);
        }

        if (debe_saltar && puede_saltar) {
            p->vel.y             = JUMP_IMPULSE;
            p->en_suelo          = false;
            p->jump_buffer_ticks = 0;

            if (tiene_wish) {
                acelerar(p, wish_dir, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
            }
            cap_velocidad_bhop(p);
        } else {
            if (tiene_wish) {
                acelerar(p, wish_dir, wish_speed, SV_ACCELERATE, dt);
            }
        }
    } else {
        /* En aire: gravedad + air strafing */
        p->vel.y -= SV_GRAVITY * dt;
        if (p->vel.y < -SV_MAXVELOCITY) { p->vel.y = -SV_MAXVELOCITY; }

        if (tiene_wish) {
            acelerar(p, wish_dir, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
        }
        cap_velocidad_bhop(p);
    }

    /* ── Gravedad tangencial en rampa ───────────────────────────────────── *
     * Aplicar si en rampa actual O predictiva (para transición flat→rampa). */
    bool on_ramp_for_gravity = on_ramp_curr || on_ramp_predictive;
    Vec3 ramp_normal_for_gravity = on_ramp_curr ? floor_normal_curr : ramp_normal_predictive;
    if (p->en_suelo && on_ramp_for_gravity) {
        /* Solo si NO se camina HACIA ARRIBA (comparar componente horizontal) */
        Vec3 gt = gravity_tangent(ramp_normal_for_gravity);
        bool walking_up = false;
        if (tiene_wish) {
            Vec3 down_slope = { gt.x, 0.0f, gt.z };
            f32 ds_len = sqrtf(down_slope.x * down_slope.x + down_slope.z * down_slope.z);
            if (ds_len > 0.001f) {
                down_slope.x /= ds_len; down_slope.z /= ds_len;
                /* Comparar solo componente horizontal de wish_dir */
                Vec3 wish_horiz = { wish_dir.x, 0.0f, wish_dir.z };
                f32 wh_len = vec3_len(wish_horiz);
                if (wh_len > 0.001f) {
                    wish_horiz = vec3_scale(wish_horiz, 1.0f / wh_len);
                    if (vec3_dot(wish_horiz, down_slope) < 0.0f) {
                        walking_up = true;
                    }
                }
            }
        }
        if (!walking_up) {
            p->vel = vec3_add(p->vel, vec3_scale(gt, dt));
        }
    }

    /* ── Colisión de paredes con swept capsule (vel horizontal + vertical) ── */
    Vec3 pos_after_walls;
    collision_bloquear_pared(map, bp, p->pos, &p->vel, dt, p->crouch_height, &pos_after_walls);

    /* ── Integrar posición ──────────────────────────────────────────────── */
    p->pos = pos_after_walls;

    /* ── Step-up robusto: usa dirección de VELOCIDAD horizontal ─────────── */
    if (p->en_suelo) {
        f32 vel_xz = sqrtf(p->vel.x * p->vel.x + p->vel.z * p->vel.z);
        if (vel_xz > 0.01f) {
            Vec3 move_dir = { p->vel.x / vel_xz, 0.0f, p->vel.z / vel_xz };
            collision_intentar_step_up(map, bp, &p->pos, &move_dir, true, p->crouch_height);
        }
    }

    /* ── Detectar suelo real en NUEVA posición ──────────────────────────── */
    FloorHit fh = collision_detectar_suelo(map, bp, p->pos, p->crouch_height);

    /* ── Predictive override para snap ──────────────────────────────────── *
     * Si detectamos rampa adelante, calcular su altura en la posición       *
     * actual del jugador y usarla si está por encima del suelo detectado.   */
    f32 floor_y = fh.y;
    Vec3 floor_normal = fh.normal;
    bool floor_hit = fh.hit;

    if (on_ramp_predictive && fh.hit) {
        Vec3 n = ramp_normal_predictive;
        if (fabsf(n.y) > 1e-4f) {
            /* Calcular altura de la rampa en posición actual del jugador.
             * Usamos next_pos como punto de referencia de la rampa. */
            f32 ramp_y_at_player = next_pos.y - (n.x * (p->pos.x - next_pos.x) + n.z * (p->pos.z - next_pos.z)) / n.y;
            if (ramp_y_at_player > fh.y) {  // rampa por encima del suelo plano
                floor_y = ramp_y_at_player;
                floor_normal = n;
                floor_hit = true;
            }
        }
    }

    /* ── Snap al suelo ──────────────────────────────────────────────────── */
    bool moving_into_floor = (p->vel.y <= 0.0f);
    if (floor_hit && floor_normal.y < 0.95f) {
        f32 vn = vec3_dot(p->vel, floor_normal);
        moving_into_floor = (vn <= 0.05f);
    }

    if (floor_hit && p->pos.y <= floor_y + SV_STEPSIZE && moving_into_floor) {
        p->pos.y = floor_y;
        p->en_suelo = true;

        /* En rampa: proyectar velocity COMPLETA (3D) al plano para próximo tick */
        if (floor_normal.y < 0.95f) {
            Vec3 vel_proj = vec3_project_on_plane(p->vel, floor_normal);
            p->vel = vel_proj;
            p->pos.y = floor_y;
        }

        if (p->jump_buffer_ticks > 0) {
            p->vel.y             = JUMP_IMPULSE;
            p->en_suelo          = false;
            p->jump_buffer_ticks = 0;
            if (tiene_wish) {
                Vec3 wish_ramp = wish_dir;
                if (floor_normal.y < 0.95f) {
                    wish_ramp = vec3_project_on_plane(wish_dir, floor_normal);
                    wish_ramp.y = 0.0f;
                    f32 len = sqrtf(wish_ramp.x * wish_ramp.x + wish_ramp.z * wish_ramp.z);
                    if (len > 0.001f) { wish_ramp.x /= len; wish_ramp.z /= len; }
                }
                acelerar(p, wish_ramp, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
            }
            cap_velocidad_bhop(p);
        }
    } else {
        p->en_suelo = false;
    }

    /* ── Coyote time: actualizar basado en transición de suelo ──────────── */
    static bool en_suelo_prev = true;
    if (en_suelo_prev && !p->en_suelo) {
        p->coyote_ticks = COYOTE_TICKS;
    } else if (p->coyote_ticks > 0) {
        p->coyote_ticks--;
    }
    en_suelo_prev = p->en_suelo;

    /* ── Contador de suelo para la ventana de gracia del bhop ───────────── */
    if (p->en_suelo) {
        if (p->ticks_en_suelo < 255) { p->ticks_en_suelo++; }
    } else {
        p->ticks_en_suelo = 0;
    }
}

f32 phys_view_height(const PhysPlayer *p)
{
    f32 ratio = (p->crouch_height - CROUCH_HEIGHT_DUCK) / (CROUCH_HEIGHT_STAND - CROUCH_HEIGHT_DUCK);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return VIEW_HEIGHT_DUCK + (VIEW_HEIGHT_STAND - VIEW_HEIGHT_DUCK) * ratio;
}