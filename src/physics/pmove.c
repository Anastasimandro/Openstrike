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
 * Usa ray_triangulo generalizada filtrando normal.y < -0.7 (cara hacia abajo). */
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

    /* Determinar objetivo según input */
    f32 nuevo_target = quiere_agachado ? CROUCH_HEIGHT_DUCK : CROUCH_HEIGHT_STAND;

    /* Si el objetivo cambia, iniciar transición */
    if (nuevo_target != p->crouch_target) {
        /* Si quiere levantarse, comprobar techo ANTES de permitir la transición */
        if (nuevo_target == CROUCH_HEIGHT_STAND) {
            if (phys_check_ceiling(map, p)) {
                /* Techo bajo: forzar a seguir agachado */
                nuevo_target = CROUCH_HEIGHT_DUCK;
                quiere_agachado = true;
            }
        }

        if (nuevo_target != p->crouch_target) {
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
        /* Sin transición activa: asegurar que height == target */
        p->crouch_height = p->crouch_target;
    }

    /* Actualizar flag agachado para compatibilidad (velocidad, etc.) */
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
    /* Guardar estado de suelo ANTES de procesar el tick para el step-up:
     * el step-up solo funciona si el jugador YA estaba en suelo al
     * empezar el tick (no si aterriza en este mismo tick). */
    bool en_suelo_antes = p->en_suelo;

    p->caminando = input.caminar;

    /* ── Actualizar agachado con transición suave y comprobación de techo ── */
    phys_crouch_update(p, &input, map);

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

    bool debe_saltar  = input.saltar || p->jump_buffer_ticks > 0;

    if (p->en_suelo) {
        /* ── En suelo: fricción + movimiento ──────────────────────────── *
         * Ventana de gracia bhop: los primeros BHOP_GRACE_TICKS tras      *
         * aterrizar NO aplican fricción. Saltar dentro de esa ventana     *
         * (~94 ms) conserva la velocidad; llegar tarde ya la perdió       *
         * (castigo por timing impreciso). Andar normal no se beneficia:   *
         * la aceleración en suelo iguala a SPEED_RUN de todos modos.      */
        if (p->ticks_en_suelo > BHOP_GRACE_TICKS) {
            aplicar_friccion(p, dt);
        }

        if (debe_saltar) {
            /* Impulso de salto */
            p->vel.y             = JUMP_IMPULSE;
            p->en_suelo          = false;
            p->jump_buffer_ticks = 0;  /* consumir buffer */

            /* Air strafing inmediato en el frame del salto — clampeado a
             * AIR_WISH_SPEED (30): sin skill no gana velocidad */
            if (vec3_len(input.wish_dir) > 0.001f) {
                acelerar(p, input.wish_dir, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
            }

            cap_velocidad_bhop(p);
        } else {
            /* Aceleración en suelo según estado */
            f32 wish_speed = p->agachado  ? SPEED_DUCK :
                             p->caminando ? SPEED_WALK :
                                            SPEED_RUN;
            if (vec3_len(input.wish_dir) > 0.001f) {
                acelerar(p, input.wish_dir, wish_speed, SV_ACCELERATE, dt);
            }
        }

    } else {
        /* ── En aire: gravedad + air strafing ─────────────────────────── */
        p->vel.y -= SV_GRAVITY * dt;
        if (p->vel.y < -SV_MAXVELOCITY) { p->vel.y = -SV_MAXVELOCITY; }

        if (vec3_len(input.wish_dir) > 0.001f) {
            acelerar(p, input.wish_dir, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
        }

        /* Cap XZ continuo en aire (ver cap_velocidad_bhop) */
        cap_velocidad_bhop(p);
    }

    /* ── Integrar posición ────────────────────────────────────────────── */
    p->pos = vec3_add(p->pos, vec3_scale(p->vel, dt));

    /* ── Bloquear paredes: slide en caras verticales (5a-2) ───────────── *
     * Debe correr ANTES de detectar suelo: recorta pos si este tick cruzó  *
     * una pared y proyecta vel para resbalar a lo largo de ella.           *
     * Pasa altura actual de la caja para colisión correcta agachado/de pie. */
    collision_bloquear_pared(map, &p->pos, &p->vel, dt, p->crouch_height);

    /* ── Step-up (5a-3): subir escalones de hasta 18 u sin saltar ───────── *
     * Solo si estaba en suelo ANTES del tick y hay movimiento en XZ.
     * Lanza rayo desde (pos + wish_dir*20 + (0,18,0)) hacia abajo.
     * Si hay suelo entre pos.y+0.5 y pos.y+18 → pos.y = suelo_y, vel.y = 0.
     * Pasa altura actual para origen correcto del rayo. */
    collision_intentar_step_up(map, &p->pos, &input.wish_dir, en_suelo_antes, p->crouch_height);

    /* ── Detectar suelo real contra geometría del mapa ───────────────── *
     * Reemplaza el suelo temporal "if (pos.y <= 0)".                    *
     * Condiciones para aterrizar:                                        *
     *   - Hay geometría de suelo en rango                                *
     *   - Los pies están dentro de 0.5 u por encima del suelo            *
     *   - La velocidad vertical es 0 o negativa (no estamos subiendo)    *
     * Pasa altura actual para origen y distancia correctos.              */
    FloorHit fh = collision_detectar_suelo(map, p->pos, p->crouch_height);
    if (fh.hit && p->pos.y <= fh.y + 0.5f && p->vel.y <= 0.0f) {
        p->pos.y    = fh.y;
        p->vel.y    = 0.0f;
        p->en_suelo = true;

        /* ── Suelo pendiente (5b): caminar por rampas como suelo ── *
         * El suelo soporta al jugador: no hay gravedad tangente    *
         * que frene al subir ni acelere al bajar. Se proyecta      *
         * wish_dir sobre el plano de la rampa para que el          *
         * movimiento siga la pendiente. Si está quieto, la         *
         * fricción lo mantiene quieto sin deslizarse.              */
        if (fh.normal.y < 0.95f) {
            f32 vel_xz = sqrtf(p->vel.x * p->vel.x + p->vel.z * p->vel.z);

            /* Proyectar wish_dir en el plano de la rampa (quitar componente normal) */
            Vec3 wish_ramp = vec3_sub(input.wish_dir,
                vec3_scale(fh.normal, vec3_dot(input.wish_dir, fh.normal)));
            wish_ramp.y = 0.0f;
            f32 wish_len = sqrtf(wish_ramp.x * wish_ramp.x + wish_ramp.z * wish_ramp.z);
            if (wish_len > 0.001f) {
                wish_ramp.x /= wish_len;
                wish_ramp.z /= wish_len;
            }

            if (wish_len > 0.001f) {
                /* Moverse en la rampa: acelerar como en suelo plano */
                f32 wish_speed = p->agachado ? SPEED_DUCK :
                                 p->caminando ? SPEED_WALK : SPEED_RUN;
                acelerar(p, wish_ramp, wish_speed, SV_ACCELERATE, dt);
            } else if (vel_xz > 0.1f) {
                /* Inercia sobre la rampa: el suelo nos sostiene, no frena */
            } else {
                /* Quieto en rampa: fricción para no deslizarse solo */
                aplicar_friccion(p, dt);
            }

            /* Proyectar velocidad al plano de la rampa para que la integración
             * siga la pendiente sin "saltar" ni penetrar el suelo.
             * Plano: dot(vel, normal) = 0  =>  vel.y = -(vel.x*n.x + vel.z*n.z)/n.y */
            if (fh.normal.y > 0.001f) {
                p->vel.y = -(p->vel.x * fh.normal.x + p->vel.z * fh.normal.z) / fh.normal.y;
            }

            p->pos.y  = fh.y;

            /* ── Salto automático al aterrizar (jump buffer en rampa) ── *
             * Si se pulió SPACE justo antes de tocar suelo (buffer con   *
             * ticks restantes), salta en este mismo tick.                */
            if (p->jump_buffer_ticks > 0) {
                p->vel.y             = JUMP_IMPULSE;
                p->en_suelo          = false;
                p->jump_buffer_ticks = 0;
                if (wish_len > 0.001f) {
                    acelerar(p, wish_ramp, AIR_WISH_SPEED, SV_AIRACCELERATE, dt);
                }
                cap_velocidad_bhop(p);
            }
        }
    } else {
        p->en_suelo = false;
    }

    /* ── Contador de suelo para la ventana de gracia del bhop ─────────── */
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