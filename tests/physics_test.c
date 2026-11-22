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

/* Test de física sin SDL: simula phys_tick contra sala_prueba.
 * Comprueba tres cosas que la regresión de rampas rompió:
 *   1. El jugador nunca sale de la sala ni atraviesa paredes.
 *   2. Sube bloques de hasta SV_STEPSIZE (32 u): la caja de 32 u
 *      en x=[-116,-84] se sube, no bloquea al jugador.
 *   3. Las rampas se suben/bajan pegadas al suelo, sin volar ni deslizarse
 *      solas cuando el jugador está quieto.
 * Compilación y ejecución: ver el paso "Test físico" en build.yml (linux). */

#include <stdio.h>
#include <math.h>
#include "types.h"
#include "physics/pmove.h"

/* Map ≈ 2.3 MB — SIEMPRE global (BSS), nunca en el stack */
static Map g_map_test;

static int g_fallos = 0;

typedef struct {
    f32  min_y;
    f32  max_y;
    Vec3 final_pos;
    bool ok;
} SimResult;

static void comprobar(bool condicion, const char *mensaje)
{
    if (!condicion) {
        printf("  FALLO: %s\n", mensaje);
        g_fallos++;
    }
}

/* Simula 'ticks' ticks caminando con wish fijo y comprueba que el jugador
 * nunca sale de la sala ni supera y_max_permitido en vertical. */
static SimResult simular(const char *nombre, Vec3 wish, int ticks,
                          Vec3 start_pos, f32 y_max_permitido)
{
    PhysPlayer p;
    phys_player_init(&p, start_pos);
    p.en_suelo = true;

    PhysInput in = {0};
    in.wish_dir    = wish;
    in.saltar      = false;
    in.saltar_held = false;
    in.agacharse   = false;
    in.caminar     = false;

    SimResult r = { start_pos.y, start_pos.y,
                    { start_pos.x, start_pos.y, start_pos.z }, true };

    for (int i = 0; i < ticks; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);

        if (p.pos.x < -255.0f || p.pos.x > 255.0f ||
            p.pos.z < -255.0f || p.pos.z > 255.0f) {
            printf("FALLO [%s] tick %d: pos=(%.2f, %.2f, %.2f) -- CRUZO la pared\n",
                   nombre, i, p.pos.x, p.pos.y, p.pos.z);
            g_fallos++;
            r.ok = false;
            break;
        }
        if (p.pos.y > y_max_permitido + 0.01f) {
            printf("FALLO [%s] tick %d: pos=(%.2f, %.2f, %.2f) -- Y FUERA DE RANGO\n",
                   nombre, i, p.pos.x, p.pos.y, p.pos.z);
            g_fallos++;
            r.ok = false;
            break;
        }
        if (p.pos.y < -1.0f) {
            printf("FALLO [%s] tick %d: pos=(%.2f, %.2f, %.2f) -- ATRAVESO EL SUELO\n",
                   nombre, i, p.pos.x, p.pos.y, p.pos.z);
            g_fallos++;
            r.ok = false;
            break;
        }

        if (p.pos.y < r.min_y) r.min_y = p.pos.y;
        if (p.pos.y > r.max_y) r.max_y = p.pos.y;
    }

    r.final_pos = p.pos;
    printf("%s [%s] %d ticks -> pos=(%.2f, %.2f, %.2f) y=[%.2f..%.2f]\n",
           r.ok ? "OK  " : "FALLO", nombre, ticks,
           p.pos.x, p.pos.y, p.pos.z, r.min_y, r.max_y);
    return r;
}

/* Simula agachado/desagachado y comprueba altura de caja y vista. */
static void simular_agachado(const char *nombre)
{
    PhysPlayer p;
    phys_player_init(&p, (Vec3){ 0.0f, 0.0f, 0.0f });
    p.en_suelo = true;

    printf("\n--- Test: %s ---\n", nombre);

    PhysInput in = {0};
    in.wish_dir    = (Vec3){ 0.0f, 0.0f, 0.0f };
    in.saltar      = false;
    in.saltar_held = false;
    in.agacharse   = false;
    in.caminar     = false;

    /* 1. Estado inicial: de pie */
    for (int i = 0; i < 10; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    }
    printf("  Inicial (pie): crouch_height=%.2f, view_height=%.2f, agachado=%d\n",
           p.crouch_height, phys_view_height(&p), p.agachado);
    comprobar(p.crouch_height >= 70.0f && p.crouch_height <= 72.0f,
              "altura de pie incorrecta");

    /* 2. Agacharse */
    in.agacharse = true;
    for (int i = 0; i < 20; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    }
    printf("  Agachado: crouch_height=%.2f, view_height=%.2f, agachado=%d\n",
           p.crouch_height, phys_view_height(&p), p.agachado);
    comprobar(p.crouch_height >= 35.0f && p.crouch_height <= 37.0f,
              "altura agachado incorrecta");
    comprobar(p.agachado, "flag agachado no activado");

    /* 3. Levantarse */
    in.agacharse = false;
    for (int i = 0; i < 20; i++) {
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
    }
    printf("  Levantado: crouch_height=%.2f, view_height=%.2f, agachado=%d\n",
           p.crouch_height, phys_view_height(&p), p.agachado);
    comprobar(p.crouch_height >= 70.0f && p.crouch_height <= 72.0f,
              "altura de pie tras levantarse incorrecta");
    comprobar(!p.agachado, "flag agachado no desactivado");

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
    comprobar(p.crouch_height >= 70.0f && p.crouch_height <= 72.0f,
              "altura tras toggle rapido incorrecta");
}

/* Test de agachado bajo techo bajo (no debe poder levantarse). */
static void simular_techo_bajo(const char *nombre)
{
    /* sala_prueba tiene techo a 192 u: no se puede probar aquí. */
    (void)nombre;
    printf("\n--- Test: %s (solo manual, sala_prueba techo alto) ---\n", nombre);
    printf("  SKIP: requiere mapa con techo < 72u\n");
}

/* Test de salto: altura de pico correcta y sin doble salto. */
static void simular_salto(const char *nombre)
{
    PhysPlayer p;
    phys_player_init(&p, (Vec3){ 0.0f, 0.0f, 0.0f });
    p.en_suelo = true;

    PhysInput in = {0};
    in.wish_dir    = (Vec3){ 0.0f, 0.0f, 0.0f };
    in.saltar      = true;
    in.saltar_held = true;
    in.agacharse   = false;
    in.caminar     = false;

    f32 max_y = p.pos.y;

    for (int i = 0; i < 120; i++) {
        if (i > 0) { in.saltar = false; }  /* solo un pulso */
        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
        if (p.pos.y > max_y) max_y = p.pos.y;
    }

    printf("\n--- Test: %s ---\n", nombre);
    printf("  pico y=%.2f, final pos=(%.2f, %.2f, %.2f), en_suelo=%d\n",
           max_y, p.pos.x, p.pos.y, p.pos.z, p.en_suelo);

    /* v^2/(2g) = 302^2/1600 ≈ 57 u. Sin doble salto no debe pasar de ~80. */
    comprobar(max_y > 40.0f && max_y < 80.0f, "altura de salto incorrecta");
    comprobar(p.pos.y <= 0.5f, "no aterrizo tras el salto");
    comprobar(p.en_suelo, "no quedo en suelo tras aterrizar");
}

int main(void)
{
    if (!map_cargar(&g_map_test, "assets/maps/sala_prueba.json")) {
        printf("FALLO: no se pudo cargar assets/maps/sala_prueba.json "
               "(ejecutar desde la raiz del repo)\n");
        return 1;
    }

    printf("=== Tests de colision paredes ===\n");

    /* Paredes de la sala: el jugador debe quedar a 16 u (radio) de la pared.
     * 320 ticks = 5 s; a 250 u/s recorre 1250 u -- cruza la sala de sobra. */
    SimResult r;

    r = simular("recto hacia pared norte (z-)", (Vec3){ 0.0f, 0.0f, -1.0f },
                320, (Vec3){ 0.0f, 0.0f, 0.0f }, 0.5f);
    comprobar(r.final_pos.z <= -238.0f && r.final_pos.z >= -242.0f,
              "no se detuvo en la pared norte");

    r = simular("diagonal 45 hacia pared norte",
                vec3_norm((Vec3){ 1.0f, 0.0f, -1.0f }),
                320, (Vec3){ 0.0f, 0.0f, 0.0f }, 0.5f);
    comprobar(r.final_pos.z <= -238.0f, "no se detuvo en la pared norte (diagonal)");

    r = simular("recto hacia pared este (x+)", (Vec3){ 1.0f, 0.0f, 0.0f },
                320, (Vec3){ 0.0f, 0.0f, 0.0f }, 20.0f);
    comprobar(r.final_pos.x >= 235.0f, "no llego a la pared este");
    comprobar(r.max_y >= 15.0f, "no subio el bloque/rampa de +x");

    r = simular("recto hacia pared sur (z+)", (Vec3){ 0.0f, 0.0f, 1.0f },
                320, (Vec3){ 0.0f, 0.0f, 0.0f }, 0.5f);
    comprobar(r.final_pos.z >= 238.0f, "no se detuvo en la pared sur");

    /* Caja de 32 u en x=[-116,-84]: con SV_STEPSIZE=32 se sube al top. */
    r = simular("recto hacia bloque oeste (x-)", (Vec3){ -1.0f, 0.0f, 0.0f },
                320, (Vec3){ 0.0f, 0.0f, 0.0f }, 0.5f);
    comprobar(r.max_y >= 30.0f,
              "subio al bloque de 32 u (SV_STEPSIZE ahora es 32)");

    /* Movimiento casi paralelo a la pared con leve componente hacia dentro.
     * El jugador arranca pegado a la pared que quiere rozar: si arranca en
     * el centro, la componente dominante choca contra la pared perpendicular
     * y la fricción frena el deslizamiento (rozamiento normal de CS, no un
     * fallo). Así se comprueba de verdad que se desliza por la pared. */
    r = simular("rasante pared norte (+x, 15 dentro)",
                vec3_norm((Vec3){ 1.0f, 0.0f, -0.268f }),
                320, (Vec3){ 0.0f, 0.0f, -240.0f }, 0.5f);
    comprobar(r.final_pos.x >= 238.0f && r.final_pos.z <= -238.0f,
              "no se mantuvo pegado a la pared norte");

    r = simular("rasante pared oeste (-z, 15 dentro)",
                vec3_norm((Vec3){ -0.268f, 0.0f, -1.0f }),
                320, (Vec3){ -240.0f, 0.0f, 0.0f }, 0.5f);
    comprobar(r.final_pos.z <= -238.0f && r.final_pos.x <= -238.0f,
              "no se mantuvo pegado a la pared oeste");

    r = simular("rasante pared este (+z, 15 dentro)",
                vec3_norm((Vec3){ 0.268f, 0.0f, 1.0f }),
                320, (Vec3){ 240.0f, 0.0f, 0.0f }, 0.5f);
    comprobar(r.final_pos.z >= 238.0f && r.final_pos.x >= 238.0f,
              "no se mantuvo pegado a la pared este");

    r = simular("diagonal suave hacia pared este",
                vec3_norm((Vec3){ 1.0f, 0.0f, 0.5f }),
                320, (Vec3){ 0.0f, 0.0f, 0.0f }, 0.5f);
    comprobar(r.final_pos.x >= 238.0f, "no llego a la pared este (diagonal suave)");

    r = simular("esquina noroeste (45 a ambas paredes)",
                vec3_norm((Vec3){ -1.0f, 0.0f, -1.0f }),
                320, (Vec3){ 0.0f, 0.0f, 0.0f }, 0.5f);
    comprobar(r.final_pos.x <= -238.0f && r.final_pos.z <= -238.0f,
              "no se detuvo en la esquina noroeste");

    printf("\n=== Tests de rampas (5b) ===\n");

    /* Rampa: x=[180,220], z=[-40,40], y=0->18. Subir y bajar pegada al suelo. */
    r = simular("rampa hacia arriba (+x)", (Vec3){ 1.0f, 0.0f, 0.0f },
                200, (Vec3){ 170.0f, 0.0f, 0.0f }, 20.0f);
    comprobar(r.max_y >= 15.0f && r.max_y <= 20.0f, "no subio la rampa");
    comprobar(r.final_pos.x >= 235.0f, "no llego a la pared tras la rampa");

    r = simular("rampa hacia abajo (-x)", (Vec3){ -1.0f, 0.0f, 0.0f },
                200, (Vec3){ 220.0f, 18.0f, 0.0f }, 20.0f);
    comprobar(r.min_y <= 1.0f, "no bajo la rampa");
    comprobar(r.max_y <= 18.5f, "se elevo por encima del tope de la rampa");
    comprobar(r.final_pos.y <= 1.0f, "no quedo a nivel del suelo");

    /* Quieto en la rampa: el suelo lo sostiene, no se desliza solo. */
    {
        PhysPlayer p;
        phys_player_init(&p, (Vec3){ 200.0f, 9.0f, 0.0f });
        p.en_suelo = true;

        PhysInput in = {0};
        in.wish_dir = (Vec3){ 0.0f, 0.0f, 0.0f };
        in.agacharse = false;
        in.caminar = false;

        for (int i = 0; i < 300; i++) {
            phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
        }
        printf("\n--- Test: quieto en rampa ---\n");
        printf("  pos=(%.2f, %.2f, %.2f), en_suelo=%d\n",
               p.pos.x, p.pos.y, p.pos.z, p.en_suelo);
        comprobar(fabsf(p.pos.y - 9.0f) <= 1.0f, "se deslizo solo en la rampa");
        comprobar(fabsf(p.pos.x - 200.0f) <= 1.0f, "se movio solo en la rampa");
        comprobar(p.en_suelo, "dejo de estar en suelo estando quieto en la rampa");
    }

    printf("\n=== Tests de agachado (5c) ===\n");

    simular_agachado("transicion agachado/levantado");
    simular_techo_bajo("techo bajo bloquea levantarse");

    /* Movimiento agachado por la rampa. */
    {
        PhysPlayer p;
        phys_player_init(&p, (Vec3){ 170.0f, 0.0f, 0.0f });
        p.en_suelo = true;

        PhysInput in = {0};
        in.wish_dir = (Vec3){ 1.0f, 0.0f, 0.0f };
        in.agacharse = true;

        f32 max_y = p.pos.y;
        for (int i = 0; i < 320; i++) {
            phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
            if (p.pos.y > max_y) max_y = p.pos.y;
        }
        printf("\n--- Test: movimiento agachado por rampa ---\n");
        printf("  pos=(%.2f, %.2f, %.2f), crouch_height=%.2f, pico=%.2f\n",
               p.pos.x, p.pos.y, p.pos.z, p.crouch_height, max_y);
        comprobar(p.pos.x >= 235.0f, "agachado no llego a la pared este");
        comprobar(max_y >= 15.0f && max_y <= 20.0f, "agachado no subio la rampa");
        comprobar(p.crouch_height >= 35.0f && p.crouch_height <= 37.0f,
                  "altura agachado incorrecta en movimiento");
    }

    /* Step-up agachado: debe subir el bloque de 16 u igual que de pie. */
    {
        PhysPlayer p;
        phys_player_init(&p, (Vec3){ 0.0f, 0.0f, 0.0f });
        p.en_suelo = true;

        PhysInput in = {0};
        in.wish_dir = (Vec3){ 1.0f, 0.0f, 0.0f };
        in.agacharse = true;

        f32 max_y = p.pos.y;
        for (int i = 0; i < 320; i++) {
            phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
            if (p.pos.y > max_y) max_y = p.pos.y;
        }
        printf("\n--- Test: step-up agachado (bloque 16 u) ---\n");
        printf("  pos=(%.2f, %.2f, %.2f), pico=%.2f\n",
               p.pos.x, p.pos.y, p.pos.z, max_y);
        comprobar(p.pos.x >= 235.0f, "agachado no paso el bloque de 16 u");
        comprobar(max_y >= 15.0f && max_y <= 20.0f, "agachado no subio el bloque");
    }

    printf("\n=== Tests de salto ===\n");

    /* Saltar pegado al bloque de 16 u no debe disparar el step-up: en el
     * tick del salto la Y solo sube por el impulso (~5 u). Antes el step-up
     * teletransportaba al jugador a x=88, y=16 en ese mismo tick (imán). */
    {
        PhysPlayer p;
        phys_player_init(&p, (Vec3){ 68.0f, 0.0f, 0.0f });  /* 16 u del bloque */
        p.en_suelo = true;

        PhysInput in = {0};
        in.wish_dir    = (Vec3){ 1.0f, 0.0f, 0.0f };
        in.saltar      = true;
        in.saltar_held = true;

        phys_tick(&p, in, &g_map_test, 1.0f / 64.0f);
        printf("\n--- Test: salto junto al bloque (sin step-up) ---\n");
        printf("  tick 1: pos=(%.2f, %.2f, %.2f)\n",
               p.pos.x, p.pos.y, p.pos.z);
        comprobar(p.pos.y < 10.0f,
                  "el step-up teletransporto al jugador al saltar junto al bloque");
        comprobar(p.pos.x < 75.0f,
                  "el step-up avanzo al jugador al saltar junto al bloque");
    }

    simular_salto("salto simple sin doble salto");

    if (g_fallos > 0) {
        printf("\nRESULTADO: %d fallo(s)\n", g_fallos);
        return 1;
    }
    printf("\nRESULTADO: OK\n");
    return 0;
}
