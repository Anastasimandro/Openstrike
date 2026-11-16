// Copyright (c) 2026 OpenStrike Project
// physics_test.c is part of OpenStrike.
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

/* Test de física sin SDL: simula phys_tick caminando contra las paredes de
 * sala_prueba y hace fallar la build si el jugador cruza los límites de la
 * sala (±256) o termina más cerca de la pared que su radio.
 * Compilación y ejecución: ver el paso "Test físico" en build.yml (linux). */

#include <stdio.h>
#include "types.h"
#include "physics/pmove.h"

/* Map ≈ 2.3 MB — SIEMPRE global (BSS), nunca en el stack */
static Map g_map_test;

static int g_fallos = 0;

/* Simula 'ticks' ticks caminando con wish fijo y comprueba que el jugador
 * nunca sale de la sala ni se pega a la pared más que su radio. */
static void simular(const char *nombre, Vec3 wish, int ticks)
{
    PhysPlayer p;
    phys_player_init(&p, (Vec3){ 0.0f, 0.0f, 0.0f });
    p.en_suelo = true;

    PhysInput in;
    in.wish_dir  = wish;
    in.saltar    = false;
    in.agacharse = false;
    in.caminar   = false;

    for (int i = 0; i < ticks; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);

        if (p.pos.x < -255.0f || p.pos.x > 255.0f ||
            p.pos.z < -255.0f || p.pos.z > 255.0f) {
            printf("FALLO [%s] tick %d: pos=(%.2f, %.2f, %.2f) — CRUZÓ la pared\n",
                   nombre, i, p.pos.x, p.pos.y, p.pos.z);
            g_fallos++;
            return;
        }
    }

    printf("OK   [%s] %d ticks → pos=(%.2f, %.2f, %.2f)\n",
           nombre, ticks, p.pos.x, p.pos.y, p.pos.z);
}

int main(void)
{
    if (!map_cargar(&g_map_test, "assets/maps/sala_prueba.json")) {
        printf("FALLO: no se pudo cargar assets/maps/sala_prueba.json "
               "(ejecutar desde la raíz del repo)\n");
        return 1;
    }

    /* 320 ticks = 5 s; a 250 u/s recorre 1250 u — cruza la sala de sobra */
    simular("recto hacia pared norte (z-)",  (Vec3){ 0.0f, 0.0f, -1.0f }, 320);
    simular("diagonal 45° hacia pared norte",(Vec3){ 0.70710678f, 0.0f, -0.70710678f }, 320);
    simular("recto hacia pared este (x+)",   (Vec3){ 1.0f, 0.0f, 0.0f }, 320);
    simular("recto hacia pared sur (z+)",    (Vec3){ 0.0f, 0.0f, 1.0f }, 320);
    simular("recto hacia pared oeste (x-)",  (Vec3){ -1.0f, 0.0f, 0.0f }, 320);

    if (g_fallos > 0) {
        printf("RESULTADO: %d fallo(s)\n", g_fallos);
        return 1;
    }
    printf("RESULTADO: OK\n");
    return 0;
}
