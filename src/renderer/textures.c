// Copyright (c) 2026 OpenStrike Project
// textures.c is part of OpenStrike.
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

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "types.h"
#include "renderer/gl.h"
#include "renderer/textures.h"
#include "map/map.h"   /* Para MapFace */

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

/* ── Estado estático del gestor de texturas ─────────────────────────── */

typedef struct {
    char  nombre[MAP_MAX_TEX_NAME];
    GLuint gl_id;
    int    ancho;
    int    alto;
    bool   valido;
} MaterialTex;

static MaterialTex g_materiales[TEXTURAS_MAX_MATERIALES];
static int g_material_count = 0;   /* 0 = fallback (checkerboard), 1..N = reales */
static bool g_texturas_inicializadas = false;

/* ── Utilidades internas ────────────────────────────────────────────── */

static GLuint crear_textura_checkerboard(void)
{
    unsigned char pixeles[64 * 64 * 4];
    int x, y;
    for (y = 0; y < 64; y++) {
        for (x = 0; x < 64; x++) {
            int idx   = (y * 64 + x) * 4;
            int celda = ((x / 8) + (y / 8)) % 2;
            if (celda == 0) {
                pixeles[idx + 0] = 210; pixeles[idx + 1] = 200;
                pixeles[idx + 2] = 179; pixeles[idx + 3] = 255;
            } else {
                pixeles[idx + 0] = 80;  pixeles[idx + 1] = 76;
                pixeles[idx + 2] = 68;  pixeles[idx + 3] = 255;
            }
        }
    }
    GLuint tex_id;
    glGenTextures(1, &tex_id);
    glBindTexture(GL_TEXTURE_2D, tex_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 64, 64, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, pixeles);
    return tex_id;
}

static GLuint cargar_textura_desde_archivo(const char *ruta, int *out_w, int *out_h)
{
    int w, h, canales;
    unsigned char *pix = stbi_load(ruta, &w, &h, &canales, 4);
    if (!pix) {
        fprintf(stderr, "texturas: stbi_load fallo '%s': %s\n", ruta, stbi_failure_reason());
        return 0;
    }

    if (w > 512 || h > 512) {
        fprintf(stderr, "texturas: ADVERTENCIA '%s' %dx%d supera 512x512, usando fallback\n", ruta, w, h);
        stbi_image_free(pix);
        return 0;
    }

    GLuint tex_id;
    glGenTextures(1, &tex_id);
    glBindTexture(GL_TEXTURE_2D, tex_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, pix);

    stbi_image_free(pix);

    if (out_w) *out_w = w;
    if (out_h) *out_h = h;
    return tex_id;
}

static int buscar_o_crear_material(const char *nombre)
{
    if (!nombre || nombre[0] == '\0') {
        return TEXTURA_FALLBACK_IDX;
    }

    /* Buscar si ya existe */
    for (int i = 1; i < g_material_count; i++) {
        if (strcmp(g_materiales[i].nombre, nombre) == 0) {
            return i;
        }
    }

    /* Crear nuevo si hay espacio */
    if (g_material_count >= TEXTURAS_MAX_MATERIALES) {
        fprintf(stderr, "texturas: máximo %d materiales alcanzado, usando fallback para '%s'\n",
                TEXTURAS_MAX_MATERIALES, nombre);
        return TEXTURA_FALLBACK_IDX;
    }

    int idx = g_material_count;
    strncpy(g_materiales[idx].nombre, nombre, MAP_MAX_TEX_NAME - 1);
    g_materiales[idx].nombre[MAP_MAX_TEX_NAME - 1] = '\0';
    g_materiales[idx].gl_id = 0;
    g_materiales[idx].ancho = 0;
    g_materiales[idx].alto = 0;
    g_materiales[idx].valido = false;
    g_material_count++;
    return idx;
}

/* ── API pública ────────────────────────────────────────────────────── */

int texturas_init(void)
{
    if (g_texturas_inicializadas) {
        return 1;
    }

    /* Activar flip vertical ANTES de cualquier carga */
    stbi_set_flip_vertically_on_load(1);

    /* Material 0 = fallback checkerboard */
    g_materiales[0].gl_id = crear_textura_checkerboard();
    if (g_materiales[0].gl_id == 0) {
        fprintf(stderr, "texturas_init: fallo crear fallback\n");
        return 0;
    }
    strcpy(g_materiales[0].nombre, "fallback_checkerboard");
    g_materiales[0].ancho = 64;
    g_materiales[0].alto = 64;
    g_materiales[0].valido = true;
    g_material_count = 1;

    g_texturas_inicializadas = true;
    return 1;
}

int texturas_cargar_mapa(const void *map, const char *ruta_base)
{
    if (!g_texturas_inicializadas) {
        fprintf(stderr, "texturas_cargar_mapa: texturas_init no llamado\n");
        return 0;
    }
    if (!map || !ruta_base) {
        return 0;
    }

    const Map *m = (const Map *)map;

    /* Primera pasada: registrar todos los nombres de textura únicos */
    for (int i = 0; i < m->face_count; i++) {
        const MapFace *face = &m->faces[i];
        if (face->textura[0] != '\0') {
            buscar_o_crear_material(face->textura);
        }
    }

    /* Segunda pasada: cargar cada material en GPU */
    char ruta_completa[256];
    for (int i = 1; i < g_material_count; i++) {
        MaterialTex *mat = &g_materiales[i];

        /* Construir ruta: assets/textures/<nombre>.png (y variantes) */
        const char *exts[] = { ".png", ".tga", ".bmp", ".jpg", ".jpeg" };
        bool cargado = false;

        for (size_t e = 0; e < sizeof(exts)/sizeof(exts[0]); e++) {
            snprintf(ruta_completa, sizeof(ruta_completa), "%s%s%s",
                     ruta_base, mat->nombre, exts[e]);

            mat->gl_id = cargar_textura_desde_archivo(ruta_completa, &mat->ancho, &mat->alto);
            if (mat->gl_id != 0) {
                mat->valido = true;
                printf("texturas: cargado '%s' (%dx%d) desde %s\n",
                       mat->nombre, mat->ancho, mat->alto, ruta_completa);
                cargado = true;
                break;
            }
        }

        if (!cargado) {
            fprintf(stderr, "texturas: no se encontró '%s' en %s (usando fallback)\n",
                    mat->nombre, ruta_base);
            mat->gl_id = g_materiales[0].gl_id;  /* fallback */
            mat->ancho = 64;
            mat->alto = 64;
            mat->valido = false;
        }
    }

    return 1;
}

void texturas_usar(int idx)
{
    if (!g_texturas_inicializadas) {
        return;
    }
    if (idx < 0 || idx >= g_material_count) {
        idx = TEXTURA_FALLBACK_IDX;
    }
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, g_materiales[idx].gl_id);
}

void texturas_shutdown(void)
{
    if (!g_texturas_inicializadas) {
        return;
    }

    for (int i = 0; i < g_material_count; i++) {
        if (g_materiales[i].gl_id && g_materiales[i].gl_id != g_materiales[0].gl_id) {
            glDeleteTextures(1, &g_materiales[i].gl_id);
        }
    }
    if (g_materiales[0].gl_id) {
        glDeleteTextures(1, &g_materiales[0].gl_id);
    }

    g_material_count = 0;
    g_texturas_inicializadas = false;
}

int texturas_indice_por_nombre(const char *nombre)
{
    if (!nombre || nombre[0] == '\0') {
        return TEXTURA_FALLBACK_IDX;
    }
    for (int i = 1; i < g_material_count; i++) {
        if (strcmp(g_materiales[i].nombre, nombre) == 0) {
            return i;
        }
    }
    return TEXTURA_FALLBACK_IDX;
}