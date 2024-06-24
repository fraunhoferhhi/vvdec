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
#define PATTERN_INTERPOLATION 0

namespace vvdec
{

/** Pseudo-random number generator (32-bit)
 * Note: loops on the 31 MSBs, so seed should be MSB-aligned in the register
 * (the register LSB has basically no effect since it is never fed back)
 */
static inline uint32_t prng( uint32_t x )
{
#if 1   // same as HW (bit-reversed RDD-5)
  uint32_t s = ( ( x << 30 ) ^ ( x << 2 ) ) & 0x80000000;
  x          = s | ( x >> 1 );
#else   // RDD-5
  uint32_t s = ( ( x >> 30 ) ^ ( x >> 2 ) ) & 1;
  x          = ( x << 1 ) | s;
#endif
  return x;
}

template<class T>
constexpr inline auto round( T a, uint8_t s )
{
  return ( a + ( 1 << ( s - 1 ) ) ) >> s;
}

class FilmGrainImpl
{
protected:
  // Note: declarations optimized for code readability; e.g. pattern storage in
  //       actual hardware implementation would differ significantly
  int8_t  pattern[2][VFGS_MAX_PATTERNS + 1][64][64];   // +1 to simplify interpolation code
  uint8_t sLUT[3][256];
  uint8_t pLUT[3][256];

  uint8_t scale_shift = 5 + 6;
  uint8_t bs          = 0;   // bitshift = bitdepth - 8
  int     csubx       = 2;
  int     csuby       = 2;
  bool    allZero[3]  = { 0, 0, 0 };

  constexpr static uint8_t Y_min = 0;
  constexpr static uint8_t Y_max = 255;
  constexpr static uint8_t C_min = 0;
  constexpr static uint8_t C_max = 255;

  static void get_offset_y( uint32_t val, int* s, uint8_t* x, uint8_t* y );
         void get_offset_u( uint32_t val, int* s, uint8_t* x, uint8_t* y ) const;
         void get_offset_v( uint32_t val, int* s, uint8_t* x, uint8_t* y ) const;

public:
  FilmGrainImpl();
  virtual ~FilmGrainImpl() = default;

  void add_grain_block( void* I, int c, int x, int y, int width, uint32_t rnd, uint32_t rnd_up, int16_t grain[3][32], uint8_t scale[3][32] ) const;
  void set_luma_pattern( int index, int8_t* P );
  void set_chroma_pattern( int index, int8_t* P );
  void set_scale_lut( int c, uint8_t lut[] );
  void set_pattern_lut( int c, uint8_t lut[], bool all0 );
  void set_scale_shift( int shift );

  void set_depth( int depth );
  void set_chroma_subsampling( int subx, int suby );

private:
  virtual void make_grain_pattern( const void* I,
                                   int         c,
                                   int         x,
                                   int         subx,
                                   uint8_t     oc1,
                                   uint8_t     oc2,
                                   uint8_t     ox,
                                   uint8_t     ox_up,
                                   uint8_t     oy,
                                   uint8_t     oy_up,
                                   int         s,
                                   int         s_up,
                                   int16_t     grain[3][32],
                                   uint8_t     scale[3][32] ) const;
  virtual void scale_and_output( void*   I,   //
                                 int     c,
                                 int     x,
                                 int     subx,
                                 int     width,
                                 int16_t grain[3][32],
                                 uint8_t scale[3][32] ) const;
};

}   // namespace vvdec
