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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "pmove.h"
#include "phys_log.h"
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

/* ── Agachado (5c): transición suave ─────────────────────────────────────── */

/* Inicializa estado de agachado al crear el jugador. */
static void phys_crouch_init(PhysPlayer *p)
{
    p->crouch_height    = CROUCH_HEIGHT_STAND;
    p->crouch_target    = CROUCH_HEIGHT_STAND;
    p->crouch_transition = 0;
}

/* Comprueba si hay techo a la altura de pie (72 u) sobre los pies.
 * Lanza rayo hacia arriba desde (pos.x, pos.y + crouch_height, pos.z).
 * Retorna true si hay obstáculo a menos de (CROUCH_HEIGHT_STAND - crouch_height + 4) u.
 * Usa collision_ray_triangulo filtrando normal.y < -0.7 (cara hacia abajo). */
static bool phys_check_ceiling(const Map *map, const PhysPlayer *p)
{
    if (!map || !map->cargado || map->face_count == 0) { return false; }

    /* Origen del rayo: arriba de la caja actual */
    Vec3 orig = { p->pos.x, p->pos.y + p->crouch_height, p->pos.z };
    Vec3 dir  = { 0.0f, 1.0f, 0.0f };

    /* Distancia máxima a comprobar: diferencia hasta altura de pie + margen */
    f32 max_dist = (CROUCH_HEIGHT_STAND - p->crouch_height) + 4.0f;

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
                /* Solo techos: normal apunta hacia abajo (y < -0.7) */
                if (normal.y < -0.7f && t < max_dist) {
                    return true;  /* Techo bloquea */
                }
            }
        }
    }
    return false;
}

/* Actualiza transición de agachado: interpola crouch_height hacia crouch_target.
 * Si se quiere levantar (target = 72) y hay techo, mantiene agachado (target = 36). */
static void phys_crouch_update(PhysPlayer *p, const PhysInput *input, const Map *map)
{
    bool quiere_agachado = input->agacharse;

    f32 nuevo_target = quiere_agachado ? CROUCH_HEIGHT_DUCK : CROUCH_HEIGHT_STAND;

    if (nuevo_target != p->crouch_target) {
        /* Si quiere levantarse, comprobar techo ANTES de permitir la transición */
        if (nuevo_target == CROUCH_HEIGHT_STAND) {
            if (phys_check_ceiling(map, p)) {
                nuevo_target = CROUCH_HEIGHT_DUCK;
                quiere_agachado = true;
            }
        }

        if (nuevo_target != p->crouch_target) {
            bool entering_duck = (nuevo_target == CROUCH_HEIGHT_DUCK);
            phys_log_crouch(p, entering_duck);
            p->crouch_target = nuevo_target;
            p->crouch_transition = CROUCH_TRANSITION_TICKS;
        }
    }

    /* Interpolar altura actual hacia objetivo */
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

void phys_tick(PhysPlayer *p, PhysInput input, const Map *map, f32 dt)
{
    /* Estado de suelo ANTES del tick: el step-up y el coyote time dependen
     * de si el jugador ya estaba apoyado al empezar. */
    bool en_suelo_antes = p->en_suelo;

    p->caminando = input.caminar;

    /* ── Actualizar agachado con transición suave y comprobación de techo ── */
    phys_crouch_update(p, &input, map);

    /* ── Jump buffer: si se pulsa salto en aire, guardar buffer ─────────── */
    if (input.saltar && !p->en_suelo) {
        p->jump_buffer_ticks = JUMP_BUFFER_TICKS;
        phys_log_jump_buffer(p, p->jump_buffer_ticks);
    } else if (p->jump_buffer_ticks > 0) {
        p->jump_buffer_ticks--;
    }

    /* ── Variable jump height (jump cut): si suelta SPACE subiendo, cortar *
     * la velocidad vertical a JUMP_CUT_VEL.                                */
    if (!input.saltar_held && p->vel.y > JUMP_CUT_VEL) {
        p->vel.y = JUMP_CUT_VEL;
    }

    bool puede_saltar = p->en_suelo || p->coyote_ticks > 0;
    bool debe_saltar  = input.saltar || p->jump_buffer_ticks > 0;
    bool tiene_wish   = vec3_len(input.wish_dir) > 0.001f;
    bool salto_consumido = false;

    /* El salto se resuelve antes de separar suelo/aire para que el coyote
     * time funcione: tras dejar el borde aún se puede saltar 2 ticks. */
    if (debe_saltar && puede_saltar) {
        bool from_coyote = !p->en_suelo && p->coyote_ticks > 0;
        bool from_buffer = p->jump_buffer_ticks > 0;
        phys_log_jump(p, from_coyote, from_buffer);

        p->vel.y             = JUMP_IMPULSE;
        p->en_suelo          = false;
        p->jump_buffer_ticks = 0;
        salto_consumido      = true;

        if (tiene_wish) {
            acelerar(p, input.wish_dir, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
        }
        cap_velocidad_bhop(p);
    } else if (p->en_suelo) {
        /* ── En suelo: fricción + movimiento ────────────────────────────── *
         * Ventana de gracia bhop: los primeros BHOP_GRACE_TICKS tras      *
         * aterrizar NO aplican fricción. Saltar dentro de esa ventana     *
         * (~94 ms) conserva la velocidad; llegar tarde ya la perdió       *
         * (castigo por timing impreciso). Andar normal no se beneficia:   *
         * la aceleración en suelo iguala a SPEED_RUN de todos modos.      */
        if (p->ticks_en_suelo > BHOP_GRACE_TICKS) {
            aplicar_friccion(p, dt);
        }

        f32 wish_speed = p->agachado  ? SPEED_DUCK :
                         p->caminando ? SPEED_WALK :
                                        SPEED_RUN;
        if (tiene_wish) {
            acelerar(p, input.wish_dir, wish_speed, SV_ACCELERATE, dt);
        }

    } else {
        /* ── En aire: gravedad + air strafing ───────────────────────────── */
        p->vel.y -= SV_GRAVITY * dt;
        if (p->vel.y < -SV_MAXVELOCITY) { p->vel.y = -SV_MAXVELOCITY; }

        if (tiene_wish) {
            acelerar(p, input.wish_dir, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
        }

        cap_velocidad_bhop(p);
    }

    /* ── Integrar posición ──────────────────────────────────────────────── */
    p->pos = vec3_add(p->pos, vec3_scale(p->vel, dt));

    /* ── Bloquear paredes: slide en caras verticales (5a-2) ─────────────── *
     * Debe correr ANTES de detectar suelo: recorta pos si este tick cruzó  *
     * una pared y proyecta vel para resbalar a lo largo de ella.           */
    bool bloqueado = collision_bloquear_pared(map, &p->pos, &p->vel, dt,
                                              p->crouch_height);

    /* ── Step-up (5a-3): subir escalones planos de hasta 18 u ───────────── *
     * Solo tras chocar con una pared: así no se dispara al caminar cerca  *
     * de un borde. El step-up sube y avanza sobre el escalón.             */
    if (bloqueado) {
        f32 y_antes_step = p->pos.y;
        if (collision_intentar_step_up(map, &p->pos, &input.wish_dir,
                                       en_suelo_antes, p->crouch_height)) {
            p->vel.y = 0.0f;
            f32 delta_y = p->pos.y - y_antes_step;
            f32 vel_xz = sqrtf(p->vel.x * p->vel.x + p->vel.z * p->vel.z);
            if (vel_xz > 0.01f) {
                Vec3 move_dir = { p->vel.x / vel_xz, 0.0f, p->vel.z / vel_xz };
                phys_log_step_up(p, delta_y, &move_dir);
            }
        }
    }

    /* ── Detectar suelo real en la nueva posición ───────────────────────── *
     * Snap normal: 0.5 u (suelo plano). En rampas se permite hasta        *
     * SV_STEPSIZE para quedar pegado a la pendiente al subir y bajar.     */
    FloorHit fh = collision_detectar_suelo(map, p->pos, p->crouch_height);
    bool on_ramp = fh.hit && fh.normal.y < 0.95f && fh.normal.y > 0.001f;
    f32 snap_dist = on_ramp ? SV_STEPSIZE : 0.5f;

    if (fh.hit && p->pos.y <= fh.y + snap_dist && p->vel.y <= 0.0f) {
        f32 impact_vel_y = p->vel.y;

        p->pos.y    = fh.y;
        p->vel.y    = 0.0f;
        p->en_suelo = true;

        if (!en_suelo_antes) {
            phys_log_land(p, impact_vel_y);
        }

        /* Jump buffer al aterrizar: saltar en este mismo tick. */
        if (p->jump_buffer_ticks > 0) {
            phys_log_jump(p, false, true);
            p->vel.y             = JUMP_IMPULSE;
            p->en_suelo          = false;
            p->jump_buffer_ticks = 0;
            salto_consumido      = true;
            if (tiene_wish) {
                acelerar(p, input.wish_dir, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
            }
            cap_velocidad_bhop(p);
        }
    } else {
        p->en_suelo = false;
    }

    /* ── Coyote time: solo al abandonar el suelo sin haber saltado ──────── */
    if (p->en_suelo) {
        p->coyote_ticks = 0;
    } else if (en_suelo_antes && !salto_consumido) {
        p->coyote_ticks = COYOTE_TICKS;
        phys_log_coyote(p, p->coyote_ticks);
    } else if (p->coyote_ticks > 0) {
        p->coyote_ticks--;
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
    /* Interpolar altura de vista entre 28 (agachado) y 64 (de pie)
     * según la altura actual de la caja (36..72). */
    f32 ratio = (p->crouch_height - CROUCH_HEIGHT_DUCK) / (CROUCH_HEIGHT_STAND - CROUCH_HEIGHT_DUCK);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return VIEW_HEIGHT_DUCK + (VIEW_HEIGHT_STAND - VIEW_HEIGHT_DUCK) * ratio;
}
