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
/**
 * \file IntraPred_neon.cpp
 * \brief Neon implementation of angular intra prediction.
 */

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/IntraPrediction.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_INTRAPRED

#include <arm_neon.h>

namespace vvdec
{

template<int W>
static void IntraPredAngleChroma_neon( int16_t* pDst, const ptrdiff_t dstStride,
                                       int16_t* pBorder, int width, int height,
                                       int deltaPos, int intraPredAngle )
{
  static_assert( W == 4 || W == 8, "unsupported chroma width tag" );

  for( int y = 0; y < height; y++ )
  {
    const int deltaInt   = deltaPos >> 5;
    const int deltaFract = deltaPos & ( 32 - 1 );

    const int16x8_t vw0 = vdupq_n_s16( (int16_t)( 32 - deltaFract ) );
    const int16x8_t vw1 = vdupq_n_s16( (int16_t)( deltaFract ) );

    const int16_t* base = pBorder + deltaInt + 1;

    if( W == 4 )
    {
      int16x4_t vlast = vld1_s16( base );
      int16x4_t vcur  = vld1_s16( base + 1 );
      int32x4_t acc   = vmull_s16( vlast, vget_low_s16( vw0 ) );
                acc   = vmlal_s16( acc, vcur, vget_low_s16( vw1 ) );
      vst1_s16( pDst, vrshrn_n_s32( acc, 5 ) );
    }
    else // W == 8
    {
      CHECKD( ( width & 7 ) != 0, "chroma width must be a multiple of 8 here" );
      for( int l = 0; l < width; l += 8 )
      {
        int16x8_t vlast = vld1q_s16( base + l );
        int16x8_t vcur  = vld1q_s16( base + l + 1 );

        int32x4_t acc_lo = vmull_s16( vget_low_s16( vlast ),  vget_low_s16( vw0 ) );
                  acc_lo = vmlal_s16( acc_lo, vget_low_s16( vcur ),  vget_low_s16( vw1 ) );
        int32x4_t acc_hi = vmull_s16( vget_high_s16( vlast ), vget_high_s16( vw0 ) );
                  acc_hi = vmlal_s16( acc_hi, vget_high_s16( vcur ), vget_high_s16( vw1 ) );

        int16x8_t res = vcombine_s16( vrshrn_n_s32( acc_lo, 5 ),
                                      vrshrn_n_s32( acc_hi, 5 ) );
        vst1q_s16( pDst + l, res );
      }
    }

    deltaPos += intraPredAngle;
    pDst     += dstStride;
  }
}

template<int W>
static void IntraPredAngleCore_neon( int16_t* pDstBuf, const ptrdiff_t dstStride, int16_t* refMain,
                                      int width, int height, int deltaPos, int intraPredAngle,
                                      const int16_t* ff, const bool useCubicFilter, const ClpRng& clpRng )
{
  static_assert( W == 4 || W == 8, "unsupported luma angular width tag" );

  uint16x8_t vbdmax;

  if( useCubicFilter )
  {
    vbdmax = vdupq_n_u16( (uint16_t)clpRng.max() );
  }

  for( int y = 0; y < height; y++ )
  {
    const int deltaInt      = deltaPos >> 5;
    const int deltaFract    = deltaPos & ( 32 - 1 );
    const int refMainIndex  = deltaInt + 1;

    const int16x4_t coeff = vld1_s16( &ff[deltaFract << 2] );
    int16_t*        pDst  = &pDstBuf[y * dstStride];

    if( W == 4 )
    {
      const int16x4_t p0 = vld1_s16( &refMain[refMainIndex - 1] );
      const int16x4_t p1 = vld1_s16( &refMain[refMainIndex    ] );
      const int16x4_t p2 = vld1_s16( &refMain[refMainIndex + 1] );
      const int16x4_t p3 = vld1_s16( &refMain[refMainIndex + 2] );

      int32x4_t acc = vmull_lane_s16( p0, coeff, 0 );
                acc = vmlal_lane_s16( acc, p1, coeff, 1 );
                acc = vmlal_lane_s16( acc, p2, coeff, 2 );
                acc = vmlal_lane_s16( acc, p3, coeff, 3 );

      int16x4_t res;

      if( useCubicFilter )
      {
        uint16x4_t ures = vqrshrun_n_s32( acc, 6 );
                   ures = vmin_u16( ures, vget_low_u16( vbdmax ) );
        res = vreinterpret_s16_u16( ures );
      }
      else
      {
        res = vrshrn_n_s32( acc, 6 );
      }
      vst1_s16( pDst, res );
    }
    else // W == 8
    {
      CHECKD( ( width & 7 ) != 0, "luma angular width must be a multiple of 8 here" );
      for( int x = 0; x < width; x += 8 )
      {
        const int16x8_t p0 = vld1q_s16( &refMain[refMainIndex + x - 1] );
        const int16x8_t p1 = vld1q_s16( &refMain[refMainIndex + x    ] );
        const int16x8_t p2 = vld1q_s16( &refMain[refMainIndex + x + 1] );
        const int16x8_t p3 = vld1q_s16( &refMain[refMainIndex + x + 2] );

        int32x4_t acc_lo = vmull_lane_s16( vget_low_s16( p0 ), coeff, 0 );
                  acc_lo = vmlal_lane_s16( acc_lo, vget_low_s16( p1 ), coeff, 1 );
                  acc_lo = vmlal_lane_s16( acc_lo, vget_low_s16( p2 ), coeff, 2 );
                  acc_lo = vmlal_lane_s16( acc_lo, vget_low_s16( p3 ), coeff, 3 );

        int32x4_t acc_hi = vmull_lane_s16( vget_high_s16( p0 ), coeff, 0 );
                  acc_hi = vmlal_lane_s16( acc_hi, vget_high_s16( p1 ), coeff, 1 );
                  acc_hi = vmlal_lane_s16( acc_hi, vget_high_s16( p2 ), coeff, 2 );
                  acc_hi = vmlal_lane_s16( acc_hi, vget_high_s16( p3 ), coeff, 3 );

        int16x8_t res;

        if( useCubicFilter )
        {
          uint16x8_t ures = vcombine_u16( vqrshrun_n_s32( acc_lo, 6 ), vqrshrun_n_s32( acc_hi, 6 ) );
                     ures = vminq_u16( ures, vbdmax );
          res = vreinterpretq_s16_u16( ures );
        }
        else
        {
          res = vcombine_s16( vrshrn_n_s32( acc_lo, 6 ), vrshrn_n_s32( acc_hi, 6 ) );
        }
        vst1q_s16( pDst + x, res );
      }
    }

    deltaPos += intraPredAngle;
  }
}

// Pure vertical prediction with PDPC, see IntraPredHorVerPdpcCore().
static void IntraPredHorVerPdpc_neon( Pel* pDst, const ptrdiff_t dstStride, const Pel* refMain, const Pel* refSide,
                                       const int width, const int height, const int scale, const Pel topLeft, const ClpRng& clpRng )
{
  const int numWeighted = std::min( 3 << scale, width );
  CHECKD( numWeighted > 32, "PDPC weights do not fit" );

  int16_t weights[32] = { 0 };
  for( int x = 0; x < numWeighted; x++ )
  {
    weights[x] = ( int16_t )( ( 32 >> std::min( 31, ( x << 1 ) >> scale ) ) << 9 );
  }

  const Pel* ref = refMain + 1;

  if( width == 4 )
  {
    const int16x4_t vref = vld1_s16( ref );
    const int16x4_t vw   = vld1_s16( weights );
    const int16x4_t vmin = vdup_n_s16( clpRng.min() );
    const int16x4_t vmax = vdup_n_s16( clpRng.max() );

    for( int y = 0; y < height; y++ )
    {
      const int16x4_t diff = vdup_n_s16( ( int16_t )( refSide[y + 1] - topLeft ) );
      int16x4_t       val  = vadd_s16( vref, vqrdmulh_s16( diff, vw ) );
                      val  = vmin_s16( vmax_s16( val, vmin ), vmax );
      vst1_s16( pDst + y * dstStride, val );
    }
    return;
  }

  CHECKD( width & 7, "unsupported block width" );

  const int weightedEnd = ( numWeighted + 7 ) & ~7;

  const int16x8_t vmin = vdupq_n_s16( clpRng.min() );
  const int16x8_t vmax = vdupq_n_s16( clpRng.max() );

  for( int y = 0; y < height; y++ )
  {
    const int16x8_t diff = vdupq_n_s16( ( int16_t )( refSide[y + 1] - topLeft ) );
    Pel*            line = pDst + y * dstStride;

    for( int x = 0; x < weightedEnd; x += 8 )
    {
      int16x8_t val = vld1q_s16( &ref[x] );
                val = vaddq_s16( val, vqrdmulhq_s16( diff, vld1q_s16( &weights[x] ) ) );
                val = vminq_s16( vmaxq_s16( val, vmin ), vmax );
      vst1q_s16( &line[x], val );
    }

    if( weightedEnd < width )
    {
      memcpy( &line[weightedEnd], &ref[weightedEnd], ( width - weightedEnd ) * sizeof( Pel ) );
    }
  }
}

template<>
void IntraPrediction::_initIntraPredictionARM<NEON>()
{
  IntraPredAngleChroma4 = IntraPredAngleChroma_neon<4>;
  IntraPredAngleChroma8 = IntraPredAngleChroma_neon<8>;
  IntraPredAngleCore4   = IntraPredAngleCore_neon<4>;
  IntraPredAngleCore8   = IntraPredAngleCore_neon<8>;
  IntraPredHorVerPdpc   = IntraPredHorVerPdpc_neon;
}

} // namespace vvdec

#endif // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_INTRAPRED
