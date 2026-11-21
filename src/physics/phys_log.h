// Copyright (c) 2026 OpenStrike Project
// phys_log.h is part of OpenStrike.
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

#include "pmove.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Niveles de log ────────────────────────────────────────────────────────── */
typedef enum {
    PHYS_LOG_NONE     = 0,  // Sin logs
    PHYS_LOG_EVENTS   = 1,  // Solo eventos discretos (saltos, landings, step-ups, transiciones rampa)
    PHYS_LOG_VERBOSE  = 2,  // Cada tick: pos, vel, estado suelo, normales, wish_dir
    PHYS_LOG_DEBUG    = 3   // Todo + detalles internos (fricción, aceleración, gravedad tangencial, etc.)
} PhysLogLevel;

/* ── Configuración global ──────────────────────────────────────────────────── */
extern PhysLogLevel g_phys_log_level;

/* Inicializa el sistema de logs (abre archivo si se especifica) */
void phys_log_init(PhysLogLevel level, const char *filepath);

/* Cierra el sistema de logs */
void phys_log_shutdown(void);

/* Cambia el nivel de log en runtime */
void phys_log_set_level(PhysLogLevel level);

/* Alterna entre NONE y VERBOSE (útil para bind a tecla) */
void phys_log_toggle_verbose(void);

/* Retorna el nombre del nivel de log actual (para debug UI) */
const char *phys_log_level_name(PhysLogLevel level);

/* ── Logging interno (llamar desde pmove.c) ────────────────────────────────── */

/* Log de un tick completo */
void phys_log_tick(const PhysPlayer *p, const PhysInput *input,
                   const FloorHit *fh_curr, bool on_ramp_curr,
                   const Vec3 *floor_normal_curr,
                   bool on_ramp_predictive, const Vec3 *ramp_normal_predictive,
                   const Vec3 *wish_dir, f32 wish_speed,
                   bool en_suelo_prev, bool debe_saltar, bool puede_saltar,
                   f32 dt);

/* Log de evento: salto */
void phys_log_jump(const PhysPlayer *p, bool from_coyote, bool from_buffer);

/* Log de evento: aterrizaje */
void phys_log_land(const PhysPlayer *p, f32 impact_vel_y);

/* Log de evento: step-up */
void phys_log_step_up(const PhysPlayer *p, f32 delta_y, const Vec3 *move_dir);

/* Log de evento: transición rampa (flat↔rampa) */
void phys_log_ramp_transition(const PhysPlayer *p, bool entering_ramp, const Vec3 *new_normal);

/* Log de evento: colisión pared */
void phys_log_wall_hit(const PhysPlayer *p, const Vec3 *hit_normal, int slide_pass);

/* Log de evento: coyote time activado */
void phys_log_coyote(const PhysPlayer *p, u8 ticks_remaining);

/* Log de evento: jump buffer activado */
void phys_log_jump_buffer(const PhysPlayer *p, u8 ticks_remaining);

/* Log de evento: agachado/desagachado */
void phys_log_crouch(const PhysPlayer *p, bool entering_duck);

#ifdef __cplusplus
}
#endif