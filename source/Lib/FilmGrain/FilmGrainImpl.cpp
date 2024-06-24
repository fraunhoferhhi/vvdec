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

#include "FilmGrainImpl.h"

#include <cstring>   // memcpy
#include <algorithm>

#include <CommonDef.h>

namespace vvdec
{

/** Derive Y x/y offsets from (random) number
 *
 * Bit fields are designed to minimize overlaps across color channels, to
 * decorrelate them as much as possible.
 *
 * 10-bit for 12 or 13 bins makes a reasonably uniform distribution (1.2%
 * probability error).
 *
 * If 8-bit is requested to further simplify the multiplier, at the cost of less
 * uniform probability, the following bitfields can be considered:
 *
 * Y: sign = rnd[31], x = (rnd[7:0]*13 >> 8)*4,   y = (rnd[21:14]*12 >> 8)*4
 * U: sign = rnd[0],  x = (rnd[17:10]*13 >> 8)*2, y = (rnd[31:24]*12 >> 8)*2
 * V: sign = rnd[13], x = (rnd[27:20]*13 >> 8)*2, y = (rnd[11:4]*12 >> 8)*2
 *
 * Note: to fully support cross-component correlation within patterns, we would
 * need to align luma/chroma offsets.
 */
void FilmGrainImpl::get_offset_y( uint32_t val, int* s, uint8_t* x, uint8_t* y )
{
  uint32_t bf;   // bit field

  *s = ( ( val >> 31 ) & 1 ) ? -1 : 1;

  bf = ( val >> 0 ) & 0x3ff;
  *x = ( ( bf * 13 ) >> 10 ) * 4;   // 13 = 8 + 4 + 1 (two adders)

  bf = ( val >> 14 ) & 0x3ff;
  *y = ( ( bf * 12 ) >> 10 ) * 4;   // 12 = 8 + 4 (one adder)
                                    // Note: could shift 9 and * 2, to make a multiple of 2 and make use of all
                                    // pattern samples (when using overlap).
}

void FilmGrainImpl::get_offset_u( uint32_t val, int* s, uint8_t* x, uint8_t* y ) const
{
  uint32_t bf;   // bit field

  *s = ( ( val >> 2 ) & 1 ) ? -1 : 1;

  bf = ( val >> 10 ) & 0x3ff;
  *x = ( ( bf * 13 ) >> 10 ) * ( 4 / csubx );

  bf = ( ( val >> 24 ) & 0x0ff ) | ( ( val << 8 ) & 0x300 );
  *y = ( ( bf * 12 ) >> 10 ) * ( 4 / csuby );
}

void FilmGrainImpl::get_offset_v( uint32_t val, int* s, uint8_t* x, uint8_t* y ) const
{
  uint32_t bf;   // bit field

  *s = ( ( val >> 15 ) & 1 ) ? -1 : 1;

  bf = ( val >> 20 ) & 0x3ff;
  *x = ( ( bf * 13 ) >> 10 ) * ( 4 / csubx );

  bf = ( val >> 4 ) & 0x3ff;
  *y = ( ( bf * 12 ) >> 10 ) * ( 4 / csuby );
}

void FilmGrainImpl::add_grain_block( void* I, int c, int x, int y, int width, uint32_t rnd, uint32_t rnd_up, int16_t grain[3][32], uint8_t scale[3][32] ) const
{
  const int subx = c ? csubx : 1;
  const int suby = c ? csuby : 1;

  if( ( y & 1 ) && suby > 1 )
  {
    return;
  }

  CHECK( x & 15, "x not a multiple of 16" );
  CHECK( width <= 128, "wrong width" );
  CHECK( bs != 0 && bs != 2, "wrong bs" );
  CHECK( scale_shift + bs < 8 || scale_shift + bs > 13, "wrong scale_shift" );

  // TODO: assert subx, suby, Y/C min/max, max pLUT values, etc

  const int j = y & 0xf;

  uint8_t oc1, oc2;                 // overlapping coefficients
  if( y > 15 && j == 0 )            // first line of overlap
  {
    oc1 = ( suby > 1 ) ? 20 : 12;   // current
    oc2 = ( suby > 1 ) ? 20 : 24;   // upper
  }
  else if( y > 15 && j == 1 )       // second line of overlap
  {
    oc1 = 24;
    oc2 = 12;
  }
  else
  {
    oc1 = oc2 = 0;
  }

  // Derive block offsets + sign
  int     s;        // random sign flip (current)
  uint8_t ox, oy;   // random offset (current)
  if( c == 0 )
  {
    get_offset_y( rnd, &s, &ox, &oy );
  }
  else if( c == 1 )
  {
    get_offset_u( rnd, &s, &ox, &oy );
  }
  else
  {
    get_offset_v( rnd, &s, &ox, &oy );
  }
  oy += j / suby;

  // Same for upper block (overlap)
  int     s_up;           // random sign flip (upper row)
  uint8_t ox_up, oy_up;   // random offset (upper row)
  if( c == 0 )
  {
    get_offset_y( rnd_up, &s_up, &ox_up, &oy_up );
  }
  else if( c == 1 )
  {
    get_offset_u( rnd_up, &s_up, &ox_up, &oy_up );
  }
  else
  {
    get_offset_v( rnd_up, &s_up, &ox_up, &oy_up );
  }
  oy_up += ( 16 + j ) / suby;

  // Make grain pattern
  make_grain_pattern( I, c, x, subx, oc1, oc2, ox, ox_up, oy, oy_up, s, s_up, grain, scale );

  // Scale & output
  scale_and_output( I, c, x, subx, width, grain, scale );
}

void FilmGrainImpl::make_grain_pattern( const void* I,
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
                                        uint8_t     scale[3][32] ) const
{
  const uint8_t*  I8  = (const uint8_t*) I;
  const uint16_t* I16 = (const uint16_t*) I;
  {
    for( int i = 0; i < 16 / subx; i++ )
    {
      uint8_t intensity = bs ? I16[x / subx + i] >> bs : I8[x / subx + i];
      uint8_t pi        = pLUT[c][intensity] >> 4;                  // pattern index (integer part)
      int     P         = pattern[c ? 1 : 0][pi][oy][ox + i] * s;   // Pattern sample (from current pattern index)
                                                                    // We could consider just XORing the sign bit
#if PATTERN_INTERPOLATION
      uint8_t pf = pLUT[c][intensity] & 15;           // pattern index fractional part (interpolate with next) -- could restrict to less bits (e.g. 2)
      int     Pn =
        pattern[c ? 1 : 0][pi + 1][oy][ox + i] * s;   // Next-pattern sample (from pattern index+1)
                                                      // But there are equivalent hw tricks, e.g. storing values as sign + amplitude instead of two's complement
#endif

      if( oc1 )   // overlap
      {
        P = round( P * oc1 + pattern[c ? 1 : 0][pi][oy_up][ox_up + i] * oc2 * s_up, 5 );
#if PATTERN_INTERPOLATION
        Pn = round( Pn * oc1 + pattern[c ? 1 : 0][pi + 1][oy_up][ox_up + i] * oc2 * s_up, 5 );
#endif
      }
#if PATTERN_INTERPOLATION
      // Pattern interpolation: P is current, Pn is next, pf is interpolation coefficient
      grain[c][16 / subx + i] = round( P * ( 16 - pf ) + Pn * pf, 4 );
#else
      grain[c][16 / subx + i] = P;
#endif
      // Scale sign already integrated above because of overlap
      scale[c][16 / subx + i] = sLUT[c][intensity];
    }
  }
}

void FilmGrainImpl::scale_and_output( void* I, int c, int x, int subx, int width, int16_t grain[3][32], uint8_t scale[3][32] ) const
{
  uint8_t*  I8  = (uint8_t*) I;
  uint16_t* I16 = (uint16_t*) I;

  const uint8_t I_min = c ? C_min : Y_min;
  const uint8_t I_max = c ? C_max : Y_max;

  int flush = 0;
  do
  {
    if( x > 0 )
    {
      if( !flush )
      {
        // Horizontal deblock (across previous block)
        int16_t l1, l0, r0, r1;

        l1 = grain[c][16 / subx - 2];
        l0 = grain[c][16 / subx - 1];
        r0 = grain[c][16 / subx + 0];
        r1 = grain[c][16 / subx + 1];

        grain[c][16 / subx - 1] = round( l1 + 3 * l0 + r0, 2 );
        grain[c][16 / subx + 0] = round( l0 + 3 * r0 + r1, 2 );
      }
      {
        for( int i = 0; i < 16 / subx; i++ )
        {
          // Output previous block (or flush current)
          int32_t g = round( scale[c][i] * (int16_t) grain[c][i], scale_shift );
          if( bs )
          {
            I16[( x - 16 ) / subx + i] = std::max<int32_t>( I_min << bs, std::min<int32_t>( I_max << bs, I16[( x - 16 ) / subx + i] + g ) );
          }
          else
          {
            I8[( x - 16 ) / subx + i] = std::max<int32_t>( I_min, std::min<int32_t>( I_max, I8[( x - 16 ) / subx + i] + g ) );
          }
        }
      }
    }

    // Shift pipeline
    if( !flush )
    {
      if( c == 0 )
      {
        for( int i = 0; i < 16; i++ )
        {
          grain[0][i] = grain[0][i + 16];
          scale[0][i] = scale[0][i + 16];
        }
      }
      else
      {
        for( int i = 0; i < 8; i++ )
        {
          grain[c][i] = grain[c][i + 8];
          scale[c][i] = scale[c][i + 8];
        }
      }
    }

    if( x + 16 >= width )
    {
      flush++;
      x += 16;
    }
  } while( flush == 1 );
}

/* Public interface ***********************************************************/

void FilmGrainImpl::set_luma_pattern( int index, int8_t* P )
{
  CHECK( index < 0 || index >= 8, "luma pattern index out of bounds" );
  memcpy( pattern[0][index], P, 64 * 64 );
}

void FilmGrainImpl::set_chroma_pattern( int index, int8_t* P )
{
  CHECK( index < 0 || index >= 8, "chroma pattern index out of bounds" );
  for( int i = 0; i < 64 / csuby; i++ )
  {
    memcpy( pattern[1][index][i], P + ( 64 / csuby ) * i, 64 / csubx );
  }
}

void FilmGrainImpl::set_scale_lut( int c, uint8_t lut[] )
{
  CHECK( c < 0 || c >= 3, "scale lut idx out of bounds" );
  memcpy( sLUT[c], lut, 256 );
}

void FilmGrainImpl::set_pattern_lut( int c, uint8_t lut[], bool all0 )
{
  CHECK( c < 0 || c >= 3, "pattern lut idx out of bounds" );
  allZero[c] = all0;
  memcpy( pLUT[c], lut, 256 );
}

void FilmGrainImpl::set_scale_shift( int shift )
{
  CHECK( shift < 2 || shift >= 8, "scale shift out of range" );
  scale_shift = shift + 6 - bs;
}

void FilmGrainImpl::set_depth( int depth )
{
  CHECK( depth != 8 && depth != 10, "only bit depth 8 and 10 supported." )

  if( bs == 0 && depth > 8 )
  {
    scale_shift -= 2;
  }
  if( bs == 2 && depth == 8 )
  {
    scale_shift += 2;
  }

  bs = depth - 8;
}

void FilmGrainImpl::set_chroma_subsampling( int subx, int suby )
{
  CHECK( subx != 1 && subx != 2, "chroma subsampling should be 1 or 2" );
  CHECK( suby != 1 && suby != 2, "chroma subsampling should be 1 or 2" );
  csubx = subx;
  csuby = suby;
}

FilmGrainImpl::FilmGrainImpl()
{
  memset( pattern, 0, sizeof( pattern ) );
  memset( sLUT,    0, sizeof( sLUT ) );
  memset( pLUT,    0, sizeof( pLUT ) );
}

}   // namespace vvdec
