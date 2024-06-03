/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/* This file is based on VFGS, available on
 * https://github.com/InterDigitalInc/VersatileFilmGrain
 *
 * VFGS implements film grain synthesis as a hardware model: it simulates the
 * output of a cost-effective hardware implementation in a video display
 * pipeline. Also, the C code is split into "fw" (firmware) and "hw" (hardware)
 * parts, and as self-explanatory as possible. See VFGS github repository for
 * more details.
 *
 * The VFGS github repository also contains other tools to experiment with film
 * grain synthesis (e.g. a graphical display and tuning tool for FGC SEI
 * message).
 */

#pragma once

#include <cstdint>

#define VFGS_MAX_PATTERNS 8

namespace vvdec
{

class FilmGrainImpl
{
  // Note: declarations optimized for code readability; e.g. pattern storage in
  //       actual hardware implementation would differ significantly
  int8_t  pattern[2][VFGS_MAX_PATTERNS + 1][64][64] = { 0, };   // +1 to simplify interpolation code
  uint8_t sLUT[3][256]                              = { 0, };
  uint8_t pLUT[3][256]                              = { 0, };

  uint32_t rnd         = 0xdeadbeef;
  uint32_t rnd_up      = 0xdeadbeef;
  uint32_t line_rnd    = 0xdeadbeef;
  uint32_t line_rnd_up = 0xdeadbeef;
  uint8_t  scale_shift = 5 + 6;
  uint8_t  bs          = 0;   // bitshift = bitdepth - 8
  int      csubx       = 2;
  int      csuby       = 2;

  constexpr static uint8_t Y_min = 0;
  constexpr static uint8_t Y_max = 255;
  constexpr static uint8_t C_min = 0;
  constexpr static uint8_t C_max = 255;

  // Processing pipeline (needs only 2 registers for each color actually, for horizontal deblocking)
  int16_t grain[3][32];   // 9 bit needed because of overlap (has norm > 1)
  uint8_t scale[3][32];

  void get_offset_u( uint32_t val, int* s, uint8_t* x, uint8_t* y );
  void get_offset_v( uint32_t val, int* s, uint8_t* x, uint8_t* y );
  void add_grain_block( void* I, int c, int x, int y, int width );

protected:
  void set_luma_pattern( int index, int8_t* P );
  void set_chroma_pattern( int index, int8_t* P );
  void set_scale_lut( int c, uint8_t lut[] );
  void set_pattern_lut( int c, uint8_t lut[] );

  void set_seed( uint32_t seed );
  void set_scale_shift( int shift );

public:
  void set_depth( int depth );
  void set_chroma_subsampling( int subx, int suby );

  void add_grain_line( void* Y, void* U, void* V, int y, int width );
};

}   // namespace vvdec
