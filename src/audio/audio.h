// Copyright (c) 2026 OpenStrike Project
// audio.h is part of OpenStrike.
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

#ifndef OPENSTRIKE_AUDIO_H
#define OPENSTRIKE_AUDIO_H

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Inicializa SDL2_mixer y reserva 32 canales.
   Retorna 1 en éxito, 0 en fallo. */
int audio_init(void);

/* Carga un archivo WAV desde la ruta dada.
   Retorna puntero al chunk cargado o NULL si falla.
   El llamador es responsable de liberar con audio_liberar_chunk. */
void *audio_cargar_wav(const char *ruta);

/* Reproduce un chunk en el primer canal disponible.
   Retorna el índice del canal usado o -1 si falla. */
int audio_reproducir(void *chunk);

/* Libera un chunk cargado. */
void audio_liberar_chunk(void *chunk);

/* Cierra el sistema de audio y libera recursos. */
void audio_shutdown(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENSTRIKE_AUDIO_H */