// Copyright (c) 2026 OpenStrike Project
// audio.c is part of OpenStrike.
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

#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
#include <stdio.h>
#include "audio.h"

/* Estructura opaca para el chunk de audio (evita exponer Mix_Chunk en el header) */
struct AudioChunk {
    Mix_Chunk *chunk;
};

static bool g_audio_inicializado = false;

int audio_init(void)
{
    if (g_audio_inicializado) {
        fprintf(stderr, "audio_init: ya inicializado\n");
        return 1;
    }

    /* SDL_Init con SDL_INIT_AUDIO debe haberse llamado antes en main.c */
    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) != 0) {
        fprintf(stderr, "Mix_OpenAudio: %s\n", Mix_GetError());
        return 0;
    }

    /* Reservar 32 canales según la regla del proyecto */
    Mix_AllocateChannels(32);

    g_audio_inicializado = true;
    return 1;
}

void *audio_cargar_wav(const char *ruta)
{
    if (!g_audio_inicializado) {
        fprintf(stderr, "audio_cargar_wav: audio no inicializado\n");
        return NULL;
    }

    if (!ruta) {
        fprintf(stderr, "audio_cargar_wav: ruta NULL\n");
        return NULL;
    }

    Mix_Chunk *chunk = Mix_LoadWAV(ruta);
    if (!chunk) {
        fprintf(stderr, "Mix_LoadWAV('%s'): %s\n", ruta, Mix_GetError());
        return NULL;
    }

    struct AudioChunk *wrapped = (struct AudioChunk *)malloc(sizeof(struct AudioChunk));
    if (!wrapped) {
        fprintf(stderr, "audio_cargar_wav: malloc falló\n");
        Mix_FreeChunk(chunk);
        return NULL;
    }

    wrapped->chunk = chunk;
    return (void *)wrapped;
}

int audio_reproducir(void *chunk)
{
    if (!g_audio_inicializado) {
        fprintf(stderr, "audio_reproducir: audio no inicializado\n");
        return -1;
    }

    struct AudioChunk *wrapped = (struct AudioChunk *)chunk;
    if (!wrapped || !wrapped->chunk) {
        fprintf(stderr, "audio_reproducir: chunk NULL\n");
        return -1;
    }

    int canal = Mix_PlayChannel(-1, wrapped->chunk, 0);
    if (canal < 0) {
        fprintf(stderr, "Mix_PlayChannel: %s\n", Mix_GetError());
    }

    return canal;
}

void audio_liberar_chunk(void *chunk)
{
    if (!chunk) {
        return;
    }

    struct AudioChunk *wrapped = (struct AudioChunk *)chunk;
    if (wrapped->chunk) {
        Mix_FreeChunk(wrapped->chunk);
    }
    free(wrapped);
}

void audio_shutdown(void)
{
    if (!g_audio_inicializado) {
        return;
    }

    Mix_CloseAudio();
    g_audio_inicializado = false;
}