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

/* Calcula la componente tangencial de la gravedad en una pendiente.
 * gravity_vec = (0, -SV_GRAVITY, 0)
 * tangent_gravity = gravity_vec - normal * dot(gravity_vec, normal)
 * Magnitud = SV_GRAVITY * sin(theta) donde theta = ángulo pendiente. */
static inline Vec3 gravity_tangent(Vec3 normal)
{
    /* gravity_vec = (0, -SV_GRAVITY, 0) */
    /* dot(gravity, normal) = -SV_GRAVITY * normal.y */
    /* tangent = (0, -SV_GRAVITY, 0) - normal * (-SV_GRAVITY * normal.y)
     *        = (0, -SV_GRAVITY, 0) + normal * (SV_GRAVITY * normal.y)
     *        = SV_GRAVITY * (normal * normal.y - (0, 1, 0)) */
    f32 gy = -SV_GRAVITY * normal.y;
    Vec3 t = {
        normal.x * gy,
        normal.y * gy + SV_GRAVITY,  // -SV_GRAVITY + normal.y * (-SV_GRAVITY * normal.y) = -SV_GRAVITY*(1 - normal.y^2) = -SV_GRAVITY*sin^2
        normal.z * gy
    };
    /* Corrección: tangente real = g * (n * n.y - up) */
    t.x = SV_GRAVITY * normal.x * normal.y;
    t.y = SV_GRAVITY * (normal.y * normal.y - 1.0f);
    t.z = SV_GRAVITY * normal.z * normal.y;
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

    /* Guardar estado de suelo ANTES de procesar el tick para el step-up:
     * el step-up solo funciona si el jugador YA estaba en suelo al
     * empezar el tick (no si aterriza en este mismo tick). */
    bool en_suelo_antes = p->en_suelo;
    Vec3 normal_suelo_antes = { 0.0f, 1.0f, 0.0f };
    if (en_suelo_antes) {
        /* Obtener normal del suelo anterior para gravedad tangencial */
        FloorHit fh_prev = collision_detectar_suelo(map, bp, p->pos, p->crouch_height);
        if (fh_prev.hit) normal_suelo_antes = fh_prev.normal;
    }

    p->caminando = input.caminar;

    /* ── Actualizar agachado con transición suave y comprobación de techo ── */
    phys_crouch_update(p, &input, map, bp);

    /* ── Coyote time: incrementar contador al dejar el suelo ───────────── */
    if (en_suelo_antes && !p->en_suelo) {
        p->coyote_ticks = COYOTE_TICKS;
    } else if (p->coyote_ticks > 0) {
        p->coyote_ticks--;
    }

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

    /* ── Determinar wish_dir y wish_speed ANTES de integrar ─────────────── *
     * Necesario para proyectar velocidad en rampa ANTES de integrar.       */
    Vec3 wish_dir = input.wish_dir;
    f32 wish_speed = 0.0f;
    bool tiene_wish = vec3_len(wish_dir) > 0.001f;

    if (p->en_suelo) {
        if (p->ticks_en_suelo > BHOP_GRACE_TICKS) {
            aplicar_friccion(p, dt);
        }

        if (debe_saltar && puede_saltar) {
            /* Impulso de salto */
            p->vel.y             = JUMP_IMPULSE;
            p->en_suelo          = false;
            p->jump_buffer_ticks = 0;

            if (tiene_wish) {
                acelerar(p, wish_dir, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
            }
            cap_velocidad_bhop(p);
        } else {
            wish_speed = p->agachado  ? SPEED_DUCK :
                         p->caminando ? SPEED_WALK :
                                        SPEED_RUN;
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

    /* ── MANEJO DE RAMPAS: proyectar velocidad ANTES de integrar ────────── *
     * Usamos normal del suelo actual (fh de tick anterior) Y hacemos un  *
     * chequeo predictivo en la posición siguiente para detectar rampas   *
     * inminentes (transición flat→rampa).                               */
    Vec3 ramp_normal = normal_suelo_antes;
    bool on_ramp = en_suelo_antes && ramp_normal.y < 0.95f && ramp_normal.y > 0.001f;

    /* Predictive floor hit (para override en transición flat→rampa) */
    FloorHit fh_predictive = {0};

    /* Predictive: si vamos a movernos en XZ, chequear suelo en pos futura.
     * Usar wish_speed (no vel actual) para mirar más lejos durante aceleración. */
    if (en_suelo_antes && tiene_wish) {
        f32 lookahead = wish_speed * dt * 4.0f;  // ~4 ticks ahead
        Vec3 next_pos = { p->pos.x + wish_dir.x * lookahead, p->pos.y, p->pos.z + wish_dir.z * lookahead };
        fh_predictive = collision_detectar_suelo(map, bp, next_pos, p->crouch_height);
        if (fh_predictive.hit && fh_predictive.normal.y < 0.95f && fh_predictive.normal.y > 0.001f) {
            /* Próximo tick estaremos en rampa: usar esa normal YA */
            ramp_normal = fh_predictive.normal;
            on_ramp = true;
        }
    }

    if (on_ramp) {
        /* Proyectar velocidad al plano de la rampa */
        Vec3 vel_tangent = vec3_project_on_plane(p->vel, ramp_normal);
        p->vel = vel_tangent;

        /* Si no hay input de movimiento, aplicar gravedad tangencial *
         * para que el jugador se deslice cuesta abajo naturalmente.  */
        if (!tiene_wish) {
            Vec3 gt = gravity_tangent(ramp_normal);
            p->vel = vec3_add(p->vel, vec3_scale(gt, dt));
        }

        /* Proyectar wish_dir al plano de la rampa para aceleración posterior */
        if (tiene_wish) {
            wish_dir = vec3_project_on_plane(wish_dir, ramp_normal);
            wish_dir.y = 0.0f;
            f32 len = sqrtf(wish_dir.x * wish_dir.x + wish_dir.z * wish_dir.z);
            if (len > 0.001f) {
                wish_dir.x /= len;
                wish_dir.z /= len;
            } else {
                tiene_wish = false;
            }
        }
    }

    /* ── Colisión de paredes con swept capsule ANTES de integrar ────────── *
     * Barrido desde posición actual hasta actual + vel*dt.                 *
     * Esto evita tunneling y permite slide correcto.                       */
    Vec3 pos_after_walls;
    collision_bloquear_pared(map, bp, p->pos, &p->vel, dt, p->crouch_height, &pos_after_walls);

    /* ── Integrar posición ──────────────────────────────────────────────── */
    p->pos = pos_after_walls;

    /* ── Step-up robusto: usa dirección de VELOCIDAD (no wish_dir) ──────── *
     * Solo si estaba en suelo ANTES y hay movimiento horizontal real.      */
    if (en_suelo_antes) {
        f32 vel_xz = sqrtf(p->vel.x * p->vel.x + p->vel.z * p->vel.z);
        if (vel_xz > 0.01f) {
            Vec3 move_dir = { p->vel.x / vel_xz, 0.0f, p->vel.z / vel_xz };
            collision_intentar_step_up(map, bp, &p->pos, &move_dir, en_suelo_antes, p->crouch_height);
        }
    }

    /* ── Detectar suelo real contra geometría del mapa ──────────────────── */
    FloorHit fh = collision_detectar_suelo(map, bp, p->pos, p->crouch_height);

    /* Override: si el check predictivo detectó rampa pero el floor detection
     * devuelve suelo plano (transición flat→rampa), confiar en el predictivo. */
    if (on_ramp && fh.hit && fh.normal.y > 0.99f && ramp_normal.y < 0.99f && fh_predictive.hit) {
        fh.normal = ramp_normal;
        fh.y = fh_predictive.y;  // usar la altura real de la rampa en la posición predictiva
    }

    if (fh.hit && p->pos.y <= fh.y + 0.5f && p->vel.y <= 0.0f) {
        p->pos.y    = fh.y;
        p->vel.y    = 0.0f;
        p->en_suelo = true;

        /* ── Suelo pendiente: manejar fricción/quieto en rampa ─────────── */
        if (fh.normal.y < 0.95f) {
            f32 vel_xz = sqrtf(p->vel.x * p->vel.x + p->vel.z * p->vel.z);

            /* Re-proyectar wish_dir al plano de la rampa actual (puede haber cambiado) */
            Vec3 wish_ramp = wish_dir;
            if (tiene_wish) {
                wish_ramp = vec3_project_on_plane(wish_dir, fh.normal);
                wish_ramp.y = 0.0f;
                f32 len = sqrtf(wish_ramp.x * wish_ramp.x + wish_ramp.z * wish_ramp.z);
                if (len > 0.001f) {
                    wish_ramp.x /= len;
                    wish_ramp.z /= len;
                } else {
                    tiene_wish = false;
                }
            }

            if (tiene_wish) {
                /* Moverse en la rampa: acelerar como en suelo plano */
                f32 ws = p->agachado ? SPEED_DUCK :
                         p->caminando ? SPEED_WALK : SPEED_RUN;
                acelerar(p, wish_ramp, ws, SV_ACCELERATE, dt);
            } else if (vel_xz > 0.1f) {
                /* Inercia sobre la rampa: el suelo nos sostiene, no frena *
                 * PERO aplicamos gravedad tangencial para deslizar natural */
                Vec3 gt = gravity_tangent(fh.normal);
                p->vel = vec3_add(p->vel, vec3_scale(gt, dt));
            } else {
                /* Quieto en rampa: fricción para no deslizarse solo *
                 * (la gravedad tangencial ya se aplicó arriba si !tiene_wish) */
                aplicar_friccion(p, dt);
            }

            /* Asegurar que velocidad no tiene componente normal al suelo */
            if (fh.normal.y > 0.001f) {
                f32 vn = vec3_dot(p->vel, fh.normal);
                if (vn < 0.0f) {
                    p->vel = vec3_sub(p->vel, vec3_scale(fh.normal, vn));
                }
            }

            p->pos.y = fh.y;

            /* ── Salto automático al aterrizar (jump buffer en rampa) ── */
            if (p->jump_buffer_ticks > 0) {
                p->vel.y             = JUMP_IMPULSE;
                p->en_suelo          = false;
                p->jump_buffer_ticks = 0;
                if (tiene_wish) {
                    acelerar(p, wish_ramp, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
                }
                cap_velocidad_bhop(p);
            }
        }
    } else {
        p->en_suelo = false;
    }

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