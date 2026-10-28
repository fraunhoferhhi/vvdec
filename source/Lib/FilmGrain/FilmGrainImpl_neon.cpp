/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/** \file     FilmGrainImpl_neon.cpp
    \brief    Neon implementation of the film grain synthesis of FilmGrainImpl.
*/

#include "FilmGrainImplARM.h"

#include <cstring>

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT

namespace vvdec
{

static_assert( PATTERN_INTERPOLATION == 0, "pattern interpolation is not implemented in the Neon code" );

// A block holds 16 samples, or 8 in a horizontally subsampled chroma plane; the code works on 8 samples at a time.

// a + b, with b zero-extended to 32 bits. The reinterpretation is exact: a 32-bit addition gives the same bits for
// signed and unsigned operands; only the extension of b depends on its sign.
static inline int32x4_t vvdec_vaddw_s32_u16( int32x4_t a, uint16x4_t b )
{
  return vreinterpretq_s32_u32( vaddw_u16( vreinterpretq_u32_s32( a ), b ) );
}

// The 8-bit LUT indexes of the n samples of a picture row from sample x0 on: the samples, shifted down to 8 bits
static inline void load_intensities( const void* I, int x0, int n, int bs, uint8_t intensity[16] )
{
  if( bs )
  {
    const uint16_t* I16   = (const uint16_t*) I + x0;
    const int16x8_t shift = vdupq_n_s16( -bs );
    for( int i = 0; i < n; i += 8 )
    {
      vst1_u8( intensity + i, vmovn_u16( vshlq_u16( vld1q_u16( I16 + i ), shift ) ) );
    }
  }
  else
  {
    const uint8_t* I8 = (const uint8_t*) I + x0;
    for( int i = 0; i < n; i += 8 )
    {
      vst1_u8( intensity + i, vld1_u8( I8 + i ) );
    }
  }
}

template<>
void FilmGrainImplARM<NEON>::make_grain_pattern( const void* I,
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
  const int n = 16 / subx;   // samples per block
  const int p = c ? 1 : 0;   // luma or chroma patterns

  uint8_t intensity[16];
  load_intensities( I, x / subx, n, bs, intensity );

  // The pattern samples of the block, and of the block above for the two rows that blend into it (oc1 != 0). With a
  // single pattern for the component, they are a row of pattern 0; else each comes from the pattern of its intensity.
  const int8_t* current = &pattern[p][0][oy][ox];
  const int8_t* above   = oc1 ? &pattern[p][0][oy_up][ox_up] : nullptr;
  int8_t        currentPerIntensity[16];
  int8_t        abovePerIntensity[16];
  if( !allZero[c] )
  {
    for( int i = 0; i < n; i++ )
    {
      const int pi           = pLUT[c][intensity[i]] >> 4;
      currentPerIntensity[i] = pattern[p][pi][oy][ox + i];
      abovePerIntensity[i]   = oc1 ? pattern[p][pi][oy_up][ox_up + i] : 0;
    }
    current = currentPerIntensity;
    above   = abovePerIntensity;
  }

  // With a negative sign, the pattern samples are widened by subtracting them from zero, which also negates them.
  const int8x8_t zero = vdup_n_s8( 0 );
  for( int i = 0; i < n; i += 8 )
  {
    const int8x8_t g8 = vld1_s8( current + i );
    int16x8_t      g  = s < 0 ? vsubl_s8( zero, g8 ) : vmovl_s8( g8 );
    if( oc1 )
    {
      const int8x8_t  gAbove8 = vld1_s8( above + i );
      const int16x8_t gAbove  = s_up < 0 ? vsubl_s8( zero, gAbove8 ) : vmovl_s8( gAbove8 );
      // round( g * oc1 + gAbove * oc2, 5 ); at most 2 * 127 * 24 in magnitude
      g = vrshrq_n_s16( vmlaq_n_s16( vmulq_n_s16( g, oc1 ), gAbove, oc2 ), 5 );
    }
    vst1q_s16( &grain[c][n + i], g );
  }

  for( int i = 0; i < n; i++ )
  {
    scale[c][n + i] = sLUT[c][intensity[i]];
  }
}

template<>
void FilmGrainImplARM<NEON>::scale_and_output( void* I, int c, int x, int subx, int width, int16_t grain[3][32], uint8_t scale[3][32] ) const
{
  const int       n        = 16 / subx;   // samples per block
  const int32x4_t minValue = vdupq_n_s32( ( c ? C_min : Y_min ) << bs );
  const int32x4_t maxValue = vdupq_n_s32( ( c ? C_max : Y_max ) << bs );
  const int32x4_t shift    = vdupq_n_s32( -scale_shift );   // a rounding right shift with vrshlq

  int flush = 0;
  do
  {
    if( x > 0 )
    {
      if( !flush )
      {
        // Horizontal deblock (across previous block)
        const int16_t l1 = grain[c][n - 2];
        const int16_t l0 = grain[c][n - 1];
        const int16_t r0 = grain[c][n + 0];
        const int16_t r1 = grain[c][n + 1];

        grain[c][n - 1] = round( l1 + 3 * l0 + r0, 2 );
        grain[c][n + 0] = round( l0 + 3 * r0 + r1, 2 );
      }

      // Output previous block (or flush current): add round( scale * grain, scale_shift ) to the samples and clip.
      for( int i = 0; i < n; i += 8 )
      {
        const int16x8_t g  = vld1q_s16( &grain[c][i] );
        const int16x8_t sc = vreinterpretq_s16_u16( vmovl_u8( vld1_u8( &scale[c][i] ) ) );
        const int32x4_t lo = vrshlq_s32( vmull_s16( vget_low_s16( sc ), vget_low_s16( g ) ), shift );
        const int32x4_t hi = vrshlq_s32( vmull_s16( vget_high_s16( sc ), vget_high_s16( g ) ), shift );

        if( bs )
        {
          uint16_t*        dst  = (uint16_t*) I + ( x - 16 ) / subx + i;
          const uint16x8_t smp  = vld1q_u16( dst );
          int32x4_t        sLo  = vvdec_vaddw_s32_u16( lo, vget_low_u16( smp ) );
          int32x4_t        sHi  = vvdec_vaddw_s32_u16( hi, vget_high_u16( smp ) );
          sLo                   = vminq_s32( vmaxq_s32( sLo, minValue ), maxValue );
          sHi                   = vminq_s32( vmaxq_s32( sHi, minValue ), maxValue );
          vst1q_u16( dst, vcombine_u16( vmovn_u32( vreinterpretq_u32_s32( sLo ) ), vmovn_u32( vreinterpretq_u32_s32( sHi ) ) ) );
        }
        else
        {
          uint8_t*         dst  = (uint8_t*) I + ( x - 16 ) / subx + i;
          const uint16x8_t smp  = vmovl_u8( vld1_u8( dst ) );
          int32x4_t        sLo  = vvdec_vaddw_s32_u16( lo, vget_low_u16( smp ) );
          int32x4_t        sHi  = vvdec_vaddw_s32_u16( hi, vget_high_u16( smp ) );
          sLo                   = vminq_s32( vmaxq_s32( sLo, minValue ), maxValue );
          sHi                   = vminq_s32( vmaxq_s32( sHi, minValue ), maxValue );
          const uint16x8_t clip = vcombine_u16( vmovn_u32( vreinterpretq_u32_s32( sLo ) ), vmovn_u32( vreinterpretq_u32_s32( sHi ) ) );
          vst1_u8( dst, vmovn_u16( clip ) );
        }
      }
    }

    // Shift pipeline
    if( !flush )
    {
      for( int i = 0; i < n; i += 8 )
      {
        vst1q_s16( &grain[c][i], vld1q_s16( &grain[c][n + i] ) );
        vst1_u8( &scale[c][i], vld1_u8( &scale[c][n + i] ) );
      }
    }

    if( x + 16 >= width )
    {
      flush++;
      x += 16;
    }
  } while( flush == 1 );
}

}   // namespace vvdec

#endif   // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT
