// Copyright (c) 2026 OpenStrike Project
// phys_log.c is part of OpenStrike.
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

#include "phys_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── Estado global ─────────────────────────────────────────────────────────── */
PhysLogLevel g_phys_log_level = PHYS_LOG_NONE;
static PhysLogLevel s_log_level = PHYS_LOG_NONE;
static FILE *s_log_file = NULL;
static bool s_owns_file = false;
static u64 s_tick_counter = 0;

/* ── Helpers ───────────────────────────────────────────────────────────────── */
static const char *level_name(PhysLogLevel lvl)
{
    switch (lvl) {
        case PHYS_LOG_NONE:     return "NONE";
        case PHYS_LOG_EVENTS:   return "EVENTS";
        case PHYS_LOG_VERBOSE:  return "VERBOSE";
        case PHYS_LOG_DEBUG:    return "DEBUG";
        default:                return "UNKNOWN";
    }
}

static void log_prefix(const char *category)
{
    if (!s_log_file) return;
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char time_buf[32];
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", tm_info);
    fprintf(s_log_file, "[%s] Tick %llu [%s] ", time_buf, (unsigned long long)s_tick_counter, category);
}

static void log_flush(void)
{
    if (s_log_file) fflush(s_log_file);
}

/* ── API pública ───────────────────────────────────────────────────────────── */

void phys_log_init(PhysLogLevel level, const char *filepath)
{
    s_log_level = level;
    g_phys_log_level = level;
    if (filepath && filepath[0]) {
        s_log_file = fopen(filepath, "w");
        s_owns_file = true;
        if (!s_log_file) {
            fprintf(stderr, "[phys_log] No se pudo abrir %s, usando stdout\n", filepath);
            s_log_file = stdout;
            s_owns_file = false;
        }
    } else {
        s_log_file = stdout;
        s_owns_file = false;
    }

    if (s_log_file) {
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tm_info);
        fprintf(s_log_file, "=== OpenStrike Physics Log Started: %s ===\n", time_buf);
        fprintf(s_log_file, "Log level: %s\n", level_name(level));
        fprintf(s_log_file, "Format: [HH:MM:SS] Tick N [CATEGORY] message\n");
        fprintf(s_log_file, "===================================================\n");
        fflush(s_log_file);
    }
}

void phys_log_shutdown(void)
{
    if (s_log_file) {
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        char time_buf[64];
        strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tm_info);
        fprintf(s_log_file, "\n=== Physics Log Ended: %s ===\n", time_buf);
        fprintf(s_log_file, "Total ticks logged: %llu\n", (unsigned long long)s_tick_counter);
        fflush(s_log_file);
        if (s_owns_file && s_log_file != stdout) {
            fclose(s_log_file);
        }
        s_log_file = NULL;
        s_owns_file = false;
    }
}

void phys_log_set_level(PhysLogLevel level)
{
    s_log_level = level;
    g_phys_log_level = level;
    if (s_log_file) {
        fprintf(s_log_file, "[CONFIG] Log level changed to %s\n", level_name(level));
        fflush(s_log_file);
    }
}

void phys_log_toggle_verbose(void)
{
    if (s_log_level >= PHYS_LOG_VERBOSE) {
        phys_log_set_level(PHYS_LOG_NONE);
    } else {
        phys_log_set_level(PHYS_LOG_VERBOSE);
    }
}

const char *phys_log_level_name(PhysLogLevel level)
{
    return level_name(level);
}

/* ── Logging interno ───────────────────────────────────────────────────────── */

void phys_log_tick(const PhysPlayer *p, const PhysInput *input,
                   const FloorHit *fh_curr, bool on_ramp_curr,
                   const Vec3 *floor_normal_curr,
                   bool on_ramp_predictive, const Vec3 *ramp_normal_predictive,
                   const Vec3 *wish_dir, f32 wish_speed,
                   bool en_suelo_prev, bool debe_saltar, bool puede_saltar,
                   f32 dt)
{
    if (s_log_level < PHYS_LOG_VERBOSE || !s_log_file) return;

    s_tick_counter++;

    log_prefix("TICK");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) "
        "en_suelo=%d agachado=%d(crouch_h=%.1f→%.1f ticks=%u) "
        "coyote=%u jump_buf=%u ticks_suelo=%u\n",
        p->pos.x, p->pos.y, p->pos.z,
        p->vel.x, p->vel.y, p->vel.z,
        p->en_suelo, p->agachado, p->crouch_height, p->crouch_target, p->crouch_transition,
        p->coyote_ticks, p->jump_buffer_ticks, p->ticks_en_suelo);

    log_prefix("INPUT");
    fprintf(s_log_file,
        "wish=(%.2f,%.2f,%.2f) saltar=%d saltar_held=%d agacharse=%d caminar=%d "
        "debe_saltar=%d puede_saltar=%d en_suelo_prev=%d\n",
        input->wish_dir.x, input->wish_dir.y, input->wish_dir.z,
        input->saltar, input->saltar_held, input->agacharse, input->caminar,
        debe_saltar, puede_saltar, en_suelo_prev);

    log_prefix("FLOOR");
    fprintf(s_log_file,
        "hit=%d y=%.2f normal=(%.2f,%.2f,%.2f) on_ramp=%d "
        "ramp_pred=%d pred_normal=(%.2f,%.2f,%.2f)\n",
        fh_curr->hit, fh_curr->y,
        fh_curr->normal.x, fh_curr->normal.y, fh_curr->normal.z,
        on_ramp_curr,
        on_ramp_predictive,
        ramp_normal_predictive->x, ramp_normal_predictive->y, ramp_normal_predictive->z);

    log_prefix("WISH");
    fprintf(s_log_file,
        "wish_dir=(%.2f,%.2f,%.2f) wish_speed=%.1f floor_norm=(%.2f,%.2f,%.2f)\n",
        wish_dir->x, wish_dir->y, wish_dir->z,
        wish_speed,
        floor_normal_curr->x, floor_normal_curr->y, floor_normal_curr->z);

    if (s_log_level >= PHYS_LOG_DEBUG) {
        log_prefix("DT");
        fprintf(s_log_file, "dt=%.6f\n", dt);
    }

    log_flush();
}

void phys_log_jump(const PhysPlayer *p, bool from_coyote, bool from_buffer)
{
    if (s_log_level < PHYS_LOG_EVENTS || !s_log_file) return;

    log_prefix("JUMP");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) vel_before=(%.2f,%.2f,%.2f) "
        "coyote=%d buffer=%d impulse=%.2f\n",
        p->pos.x, p->pos.y, p->pos.z,
        p->vel.x, p->vel.y - JUMP_IMPULSE, p->vel.z,  // vel antes del impulso
        from_coyote, from_buffer, JUMP_IMPULSE);
    log_flush();
}

void phys_log_land(const PhysPlayer *p, f32 impact_vel_y)
{
    if (s_log_level < PHYS_LOG_EVENTS || !s_log_file) return;

    log_prefix("LAND");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) impact_vel_y=%.2f vel_after=(%.2f,%.2f,%.2f)\n",
        p->pos.x, p->pos.y, p->pos.z,
        impact_vel_y,
        p->vel.x, p->vel.y, p->vel.z);
    log_flush();
}

void phys_log_step_up(const PhysPlayer *p, f32 delta_y, const Vec3 *move_dir)
{
    if (s_log_level < PHYS_LOG_EVENTS || !s_log_file) return;

    log_prefix("STEP_UP");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) delta_y=%.2f move_dir=(%.2f,%.2f,%.2f)\n",
        p->pos.x, p->pos.y, p->pos.z,
        delta_y, move_dir->x, move_dir->y, move_dir->z);
    log_flush();
}

void phys_log_ramp_transition(const PhysPlayer *p, bool entering_ramp, const Vec3 *new_normal)
{
    if (s_log_level < PHYS_LOG_EVENTS || !s_log_file) return;

    log_prefix("RAMP_TRANSITION");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) %s new_normal=(%.2f,%.2f,%.2f)\n",
        p->pos.x, p->pos.y, p->pos.z,
        entering_ramp ? "ENTERING_RAMP" : "LEAVING_RAMP",
        new_normal->x, new_normal->y, new_normal->z);
    log_flush();
}

void phys_log_wall_hit(const PhysPlayer *p, const Vec3 *hit_normal, int slide_pass)
{
    if (s_log_level < PHYS_LOG_EVENTS || !s_log_file) return;

    log_prefix("WALL_HIT");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) hit_normal=(%.2f,%.2f,%.2f) slide_pass=%d "
        "vel_before=(%.2f,%.2f,%.2f)\n",
        p->pos.x, p->pos.y, p->pos.z,
        hit_normal->x, hit_normal->y, hit_normal->z,
        slide_pass,
        p->vel.x, p->vel.y, p->vel.z);
    log_flush();
}

void phys_log_coyote(const PhysPlayer *p, u8 ticks_remaining)
{
    if (s_log_level < PHYS_LOG_EVENTS || !s_log_file) return;

    log_prefix("COYOTE");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) ticks_remaining=%u\n",
        p->pos.x, p->pos.y, p->pos.z, ticks_remaining);
    log_flush();
}

void phys_log_jump_buffer(const PhysPlayer *p, u8 ticks_remaining)
{
    if (s_log_level < PHYS_LOG_EVENTS || !s_log_file) return;

    log_prefix("JUMP_BUFFER");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) ticks_remaining=%u\n",
        p->pos.x, p->pos.y, p->pos.z, ticks_remaining);
    log_flush();
}

void phys_log_crouch(const PhysPlayer *p, bool entering_duck)
{
    if (s_log_level < PHYS_LOG_EVENTS || !s_log_file) return;

    log_prefix("CROUCH");
    fprintf(s_log_file,
        "pos=(%.2f,%.2f,%.2f) %s crouch_h=%.1f→%.1f transition=%u\n",
        p->pos.x, p->pos.y, p->pos.z,
        entering_duck ? "DUCK" : "STAND",
        p->crouch_height, p->crouch_target, p->crouch_transition);
    log_flush();
}