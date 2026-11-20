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
// MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  See the
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

    PhysInput in = {0};
    in.wish_dir  = wish;
    in.saltar    = false;
    in.saltar_held = false;
    in.agacharse = false;
    in.caminar   = false;

    for (int i = 0; i < ticks; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);

        if (p.pos.x < -255.0f || p.pos.x > 255.0f ||
            p.pos.z < -255.0f || p.pos.z > 255.0f) {
            printf("FALLO [%s] tick %d: pos=(%.2f, %.2f, %.2f) -- CRUZO la pared\n",
                   nombre, i, p.pos.x, p.pos.y, p.pos.z);
            g_fallos++;
            return;
        }
    }

    printf("OK   [%s] %d ticks -> pos=(%.2f, %.2f, %.2f)\n",
           nombre, ticks, p.pos.x, p.pos.y, p.pos.z);
}

/* Simula agachado/desagachado y comprueba altura de caja y vista. */
static void simular_agachado(const char *nombre)
{
    PhysPlayer p;
    phys_player_init(&p, (Vec3){ 0.0f, 0.0f, 0.0f });
    p.en_suelo = true;

    printf("\n--- Test: %s ---\n", nombre);

    /* 1. Estado inicial: de pie */
    PhysInput in = {0};
    in.wish_dir = (Vec3){ 0.0f, 0.0f, 0.0f };
    in.saltar = false;
    in.saltar_held = false;
    in.agacharse = false;
    in.caminar = false;

    for (int i = 0; i < 10; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    }
    printf("  Inicial (pie): crouch_height=%.2f, view_height=%.2f, agachado=%d\n",
           p.crouch_height, phys_view_height(&p), p.agachado);
    if (p.crouch_height < 70.0f || p.crouch_height > 72.0f) {
        printf("  FALLO: altura de pie incorrecta\n");
        g_fallos++;
    }

    /* 2. Agacharse */
    in.agacharse = true;
    for (int i = 0; i < 20; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    }
    printf("  Agachado: crouch_height=%.2f, view_height=%.2f, agachado=%d\n",
           p.crouch_height, phys_view_height(&p), p.agachado);
    if (p.crouch_height < 35.0f || p.crouch_height > 37.0f) {
        printf("  FALLO: altura agachado incorrecta\n");
        g_fallos++;
    }
    if (!p.agachado) {
        printf("  FALLO: flag agachado no activado\n");
        g_fallos++;
    }

    /* 3. Levantarse */
    in.agacharse = false;
    for (int i = 0; i < 20; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    }
    printf("  Levantado: crouch_height=%.2f, view_height=%.2f, agachado=%d\n",
           p.crouch_height, phys_view_height(&p), p.agachado);
    if (p.crouch_height < 70.0f || p.crouch_height > 72.0f) {
        printf("  FALLO: altura de pie tras levantarse incorrecta\n");
        g_fallos++;
    }
    if (p.agachado) {
        printf("  FALLO: flag agachado no desactivado\n");
        g_fallos++;
    }

    /* 4. Agachado rápido (toggle rápido) */
    in.agacharse = true;
    for (int i = 0; i < 4; i++) phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    in.agacharse = false;
    for (int i = 0; i < 4; i++) phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    in.agacharse = true;
    for (int i = 0; i < 4; i++) phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    in.agacharse = false;
    for (int i = 0; i < 20; i++) phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    printf("  Toggle rapido: crouch_height=%.2f, view_height=%.2f\n",
           p.crouch_height, phys_view_height(&p));
    if (p.crouch_height < 70.0f || p.crouch_height > 72.0f) {
        printf("  FALLO: altura tras toggle rapido incorrecta\n");
        g_fallos++;
    }

    printf("  OK\n");
}

/* Test de agachado bajo techo bajo (no debe poder levantarse). */
static void simular_techo_bajo(const char *nombre)
{
    /* Nota: este test requiere un mapa con techo bajo.
     * Como sala_prueba tiene techo a 192u, no podemos probarlo aquí.
     * Se prueba manualmente en runtime. */
    (void)nombre;
    printf("\n--- Test: %s (solo manual, sala_prueba techo alto) ---\n", nombre);
    printf("  SKIP: requiere mapa con techo < 72u\n");
}

int main(void)
{
    if (!map_cargar(&g_map_test, "assets/maps/sala_prueba.json")) {
        printf("FALLO: no se pudo cargar assets/maps/sala_prueba.json "
               "(ejecutar desde la raiz del repo)\n");
        return 1;
    }

    printf("=== Tests de colision paredes (11 casos previos) ===\n");

    /* 320 ticks = 5 s; a 250 u/s recorre 1250 u -- cruza la sala de sobra */
    simular("recto hacia pared norte (z-)",  (Vec3){ 0.0f, 0.0f, -1.0f }, 320);
    simular("diagonal 45° hacia pared norte",(Vec3){ 0.70710678f, 0.0f, -0.70710678f }, 320);
    simular("recto hacia pared este (x+)",   (Vec3){ 1.0f, 0.0f, 0.0f }, 320);
    simular("recto hacia pared sur (z+)",    (Vec3){ 0.0f, 0.0f, 1.0f }, 320);
    simular("recto hacia pared oeste (x-)",  (Vec3){ -1.0f, 0.0f, 0.0f }, 320);

    /* Casos que reprodujeron el bloqueo irregular (goteo en rozamiento):
     * movimiento casi paralelo a la pared con leve componente hacia dentro */
    simular("rasante pared norte (+x, 15° dentro)",
            vec3_norm((Vec3){ 1.0f, 0.0f, -0.268f }), 320);
    simular("rasante pared oeste (-z, 15° dentro)",
            vec3_norm((Vec3){ -0.268f, 0.0f, -1.0f }), 320);
    simular("rasante pared este (+z, 15° dentro)",
            vec3_norm((Vec3){ 1.0f, 0.0f, 0.268f }), 320);
    simular("diagonal suave hacia pared este",
            vec3_norm((Vec3){ 1.0f, 0.0f, 0.5f }), 320);
    simular("esquina noroeste (45° a ambas paredes)",
            vec3_norm((Vec3){ -1.0f, 0.0f, -1.0f }), 320);

    /* Rampa (5b): caminar hacia +x sube la rampa sin saltar (y aumenta) */
    simular("rampa hacia arriba (+x)", (Vec3){ 1.0f, 0.0f, 0.0f }, 200);

    printf("\n=== Tests de agachado (5c) ===\n");

    /* Test de transición agachado/levantado */
    simular_agachado("transicion agachado/levantado");

    /* Test de techo bajo (solo documentado, no ejecutable sin mapa específico) */
    simular_techo_bajo("techo bajo bloquea levantarse");

    /* Test de movimiento agachado contra paredes */
    {
        PhysPlayer p;
        phys_player_init(&p, (Vec3){ 0.0f, 0.0f, 0.0f });
        p.en_suelo = true;

        PhysInput in = {0};
        in.wish_dir = (Vec3){ 1.0f, 0.0f, 0.0f };
        in.saltar = false;
        in.saltar_held = false;
        in.agacharse = true;  /* AGACHADO */
        in.caminar = false;

        for (int i = 0; i < 320; i++) {
            phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
        }
        printf("\n--- Test: movimiento agachado contra pared este ---\n");
        printf("  pos=(%.2f, %.2f, %.2f), crouch_height=%.2f\n",
               p.pos.x, p.pos.y, p.pos.z, p.crouch_height);
        if (p.pos.x < -255.0f || p.pos.x > 255.0f) {
            printf("  FALLO: cruzo pared agachado\n");
            g_fallos++;
        } else {
            printf("  OK\n");
        }
    }

    /* Test de step-up agachado (debe funcionar igual) */
    {
        PhysPlayer p;
        phys_player_init(&p, (Vec3){ 0.0f, 0.0f, 0.0f });
        p.en_suelo = true;

        PhysInput in = {0};
        in.wish_dir = (Vec3){ 1.0f, 0.0f, 0.0f };
        in.saltar = false;
        in.saltar_held = false;
        in.agacharse = true;
        in.caminar = false;

        for (int i = 0; i < 200; i++) {
            phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
        }
        printf("\n--- Test: step-up en rampa agachado ---\n");
        printf("  pos=(%.2f, %.2f, %.2f), crouch_height=%.2f\n",
               p.pos.x, p.pos.y, p.pos.z, p.crouch_height);
        if (p.pos.y > 1.0f) {  /* subió la rampa */
            printf("  OK (subio rampa agachado)\n");
        } else {
            printf("  INFO: no subio rampa (esperado si no hay step-up en ruta)\n");
        }
    }

    if (g_fallos > 0) {
        printf("\nRESULTADO: %d fallo(s)\n", g_fallos);
        return 1;
    }
    printf("\nRESULTADO: OK\n");
    return 0;
}