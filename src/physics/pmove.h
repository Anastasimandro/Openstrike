// Copyright (c) 2026 OpenStrike Project
// pmove.h is part of OpenStrike.
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

#pragma once

#include "types.h"
#include "renderer/math.h"
#include "collision.h"
#include "broadphase.h"

/* ── Constantes de movimiento — exactas CS:GO ─────────────────────────────── */
#define SV_GRAVITY        800.0f
#define SV_MAXVELOCITY   3500.0f
#define SV_FRICTION         5.5f
#define SV_STOPSPEED       80.0f
#define SV_ACCELERATE       5.5f
#define SV_AIRACCELERATE   12.0f   /* aceleración aérea (valor CS:GO)         */
                                   /* velocidad deseada se clampea a 30 u/s:   */
                                   /* solo el strafe bien medido gana velocidad*/
#define AIR_WISH_SPEED     30.0f
#define BHOP_MAX_SPEED    320.0f   /* cap XZ continuo en aire — 28% sobre RUN  */
#define BHOP_GRACE_TICKS      6    /* ticks tras aterrizar sin fricción:       */
                                   /* saltar dentro de la ventana (~94 ms)     */
                                   /* conserva la velocidad — timing exigente  */
                                   /* pero aprendible (sin autobhop)           */
#define SV_STEPSIZE        18.0f
#define SPEED_RUN         250.0f
#define SPEED_WALK        130.0f
#define SPEED_DUCK         85.0f
#define JUMP_IMPULSE  301.993012f
#define VIEW_HEIGHT_STAND  64.0f   /* altura ojo de pie (unidades Hammer)     */
#define VIEW_HEIGHT_DUCK   28.0f   /* altura ojo agachado                     */

/* ── Mejoras de salto (quality of life) ───────────────────────────────────── */
#define JUMP_CUT_VEL      120.0f   /* vel.y mínima al soltar SPACE en subida  */
#define COYOTE_TICKS        2      /* ticks tras dejar suelo donde aún se puede
                                    * saltar (~31 ms) — evita fallos de timing */
#define JUMP_BUFFER_TICKS   3      /* ticks de buffer si se pulsa SPACE antes
                                    * de aterrizar — salta al tocar suelo      */

/* ── Agachado (5c): transición suave ──────────────────────────────────────── */
#define CROUCH_TRANSITION_TICKS  8   /* ~125 ms a 64 Hz */

#define CROUCH_HEIGHT_STAND  72.0f
#define CROUCH_HEIGHT_DUCK   36.0f

/* ── Structs ──────────────────────────────────────────────────────────────── */

typedef struct {
    Vec3 wish_dir;   /* dirección deseada normalizada en plano XZ */
    bool saltar;       /* flanco de subida (pulso) */
    bool saltar_held;  /* tecla mantenida (para variable jump height) */
    bool agacharse;
    bool caminar;
} PhysInput;

typedef struct {
    Vec3 pos;            /* posición en unidades Hammer  */
    Vec3 vel;            /* velocidad en unidades/segundo */
    bool en_suelo;
    bool agachado;
    bool caminando;
    u8   ticks_en_suelo;     /* ticks consecutivos en suelo — ventana bhop */
    u8   coyote_ticks;       /* ticks desde que dejó el suelo (coyote time) */
    u8   jump_buffer_ticks;  /* buffer de salto si se pulsa antes de aterrizar */

    /* ── Agachado (5c): transición suave y comprobación de techo ───────── */
    f32  crouch_height;      /* altura actual de la caja (36..72, interpolada) */
    f32  crouch_target;      /* altura objetivo (36 agachado, 72 de pie)       */
    u8   crouch_transition;  /* ticks restantes de transición (0 = sin trans.) */
} PhysPlayer;

/* ── API pública ──────────────────────────────────────────────────────────── */

/* memset a cero + asigna pos. Llamar una vez al inicio. */
void phys_player_init(PhysPlayer *p, Vec3 spawn_pos);

/* Tick completo de física. Llamar exactamente a 64 Hz (dt = 1/64 = 0.015625).
 * map se usa para detectar el suelo real — pasar &g_map siempre.             */
void phys_tick(PhysPlayer *p, PhysInput input, const Map *map, f32 dt);

/* Retorna altura de vista interpolada según estado de agachado. */
f32  phys_view_height(const PhysPlayer *p);