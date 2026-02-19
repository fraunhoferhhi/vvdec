/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/** \file     InterPred_neon.cpp
    \brief    SIMD for InterPrediction
*/

//! \ingroup CommonLib
//! \{

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/InterPrediction.h"

namespace vvdec
{

#if ENABLE_SIMD_OPT_INTER && defined( TARGET_SIMD_ARM )

template<bool PAD>
void gradFilter_neon( Pel* src, ptrdiff_t _srcStride, int width, int height, ptrdiff_t _gradStride, Pel* gradX,
                      Pel* gradY, const int bitDepth )
{
  const int widthInside = PAD ? width - 2 * BIO_EXTEND_SIZE : 4;
  const int heightInside = PAD ? height - 2 * BIO_EXTEND_SIZE : 4;
  const ptrdiff_t gradStride = PAD ? _gradStride : 4;
  const ptrdiff_t srcStride = PAD ? _srcStride : 6;

  int16_t* srcTmp = PAD ? src + srcStride + 1 : src;
  int16_t* gradXTmp = PAD ? gradX + gradStride + 1 : gradX;
  int16_t* gradYTmp = PAD ? gradY + gradStride + 1 : gradY;

  constexpr int shift = 6;

  CHECKD( widthInside < 4, "(Width - 2) must be greater than or equal to 4!" );
  CHECKD( heightInside % 2 != 0, "(Height - 2) must be multiple of 2!" );

  if( widthInside % 8 == 0 )
  {
    int y = heightInside;
    do
    {
      int x = 0;
      do
      {
        int16x8_t srcRight = vld1q_s16( srcTmp + x + 1 );
        int16x8_t srcLeft = vld1q_s16( srcTmp + x - 1 );

        int16x8_t srcBottom0 = vld1q_s16( srcTmp + srcStride + x );
        int16x8_t srcTop0 = vld1q_s16( srcTmp - srcStride + x );

        srcRight = vshrq_n_s16( srcRight, shift );
        srcLeft = vshrq_n_s16( srcLeft, shift );
        srcBottom0 = vshrq_n_s16( srcBottom0, shift );
        srcTop0 = vshrq_n_s16( srcTop0, shift );

        int16x8_t grad_x = vsubq_s16( srcRight, srcLeft );
        int16x8_t grad_y = vsubq_s16( srcBottom0, srcTop0 );

        vst1q_s16( gradXTmp, grad_x );
        vst1q_s16( gradYTmp, grad_y );

        gradXTmp += 8;
        gradYTmp += 8;
        x += 8;
      } while( x != widthInside );

      gradXTmp += gradStride - widthInside;
      gradYTmp += gradStride - widthInside;
      srcTmp += srcStride;
    } while( --y != 0 );
  }
  else
  {
    CHECKD( widthInside != 4, "(Width - 2) must be equal to 4!" );
    int y = heightInside >> 1;

    int16x8_t srcTop = vcombine_s16( vld1_s16( srcTmp - srcStride ), vld1_s16( srcTmp ) );
    srcTop = vshrq_n_s16( srcTop, shift );

    do
    {
      int16x8_t srcRight = vcombine_s16( vld1_s16( srcTmp + 1 ), vld1_s16( srcTmp + srcStride + 1 ) );
      int16x8_t srcLeft = vcombine_s16( vld1_s16( srcTmp - 1 ), vld1_s16( srcTmp + srcStride - 1 ) );
      int16x8_t srcBottom = vcombine_s16( vld1_s16( srcTmp + srcStride ), vld1_s16( srcTmp + ( srcStride << 1 ) ) );

      srcRight = vshrq_n_s16( srcRight, shift );
      srcLeft = vshrq_n_s16( srcLeft, shift );
      srcBottom = vshrq_n_s16( srcBottom, shift );

      const int16x8_t grad_x = vsubq_s16( srcRight, srcLeft );
      const int16x8_t grad_y = vsubq_s16( srcBottom, srcTop );

      vst1_s16( gradXTmp, vget_low_s16( grad_x ) );
      vst1_s16( gradXTmp + gradStride, vget_high_s16( grad_x ) );
      vst1_s16( gradYTmp, vget_low_s16( grad_y ) );
      vst1_s16( gradYTmp + gradStride, vget_high_s16( grad_y ) );

      gradXTmp += gradStride << 1;
      gradYTmp += gradStride << 1;
      srcTmp += srcStride << 1;
      srcTop = srcBottom; // For next iteration.
    } while( --y != 0 );
  }

  if( PAD )
  {
    gradXTmp = gradX + gradStride + 1;
    gradYTmp = gradY + gradStride + 1;
    srcTmp = src + srcStride + 1;
    int y = heightInside;
    do
    {
      gradXTmp[-1] = gradXTmp[0];
      gradXTmp[widthInside] = gradXTmp[widthInside - 1];
      gradXTmp += gradStride;

      gradYTmp[-1] = gradYTmp[0];
      gradYTmp[widthInside] = gradYTmp[widthInside - 1];
      gradYTmp += gradStride;

      srcTmp[-1] = srcTmp[0];
      srcTmp[widthInside] = srcTmp[widthInside - 1];
      srcTmp += srcStride;
    } while( --y != 0 );

    gradXTmp = gradX + gradStride;
    gradYTmp = gradY + gradStride;
    srcTmp = src + srcStride;

    memcpy( gradXTmp - gradStride, gradXTmp, sizeof( Pel ) * width );
    memcpy( gradXTmp + heightInside * gradStride, gradXTmp + ( heightInside - 1 ) * gradStride, sizeof( Pel ) * width );
    memcpy( gradYTmp - gradStride, gradYTmp, sizeof( Pel ) * width );
    memcpy( gradYTmp + heightInside * gradStride, gradYTmp + ( heightInside - 1 ) * gradStride, sizeof( Pel ) * width );
    memcpy( srcTmp - srcStride, srcTmp, sizeof( Pel ) * ( width ) );
    memcpy( srcTmp + heightInside * srcStride, srcTmp + ( heightInside - 1 ) * srcStride, sizeof( Pel ) * width );
  }
}

template<>
void InterPrediction::_initInterPredictionARM<NEON>()
{
  BioGradFilter = gradFilter_neon<true>;
  profGradFilter = gradFilter_neon<false>;
}

#endif // ENABLE_SIMD_OPT_INTER && defined(TARGET_SIMD_ARM)

} // namespace vvdec
