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
 * \brief Neon implementation of angular chroma intra prediction.
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

template<>
void IntraPrediction::_initIntraPredictionARM<NEON>()
{
  IntraPredAngleChroma4 = IntraPredAngleChroma_neon<4>;
  IntraPredAngleChroma8 = IntraPredAngleChroma_neon<8>;
}

} // namespace vvdec

#endif // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_INTRAPRED
