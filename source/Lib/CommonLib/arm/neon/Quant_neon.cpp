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
 * \file Quant_neon.cpp
 * \brief Neon implementation of dequantization kernels.
 */

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Quant.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_QUANT

namespace vvdec
{

template<class T, bool UseScalingList>
static void DeQuantImplNeon( const SizeType width,
                             const int      maxX,
                             const int      maxY,
                             const int      scaleQP,
                             const int*     piDequantCoef,
                             const T* const piQCoef,
                             const size_t   piQCfStride,
                             TCoeff* const  piCoef,
                             const int      rightShift,
                             const int      inputMaximum,
                             const TCoeff   transformMaximum )
{
  constexpr bool QCoef_16bit = sizeof( piQCoef[0] ) == sizeof( int16_t );

  const int    inputMinimum     = -( inputMaximum + 1 );
  const TCoeff transformMinimum = -( transformMaximum + 1 );

  const int32x4_t vInputMin     = vdupq_n_s32( inputMinimum );
  const int32x4_t vInputMax     = vdupq_n_s32( inputMaximum );
  const int32x4_t vTransformMin = vdupq_n_s32( transformMinimum );
  const int32x4_t vTransformMax = vdupq_n_s32( transformMaximum );
  const int32x4_t vShift        = vdupq_n_s32( -rightShift );
        int32x4_t vScale        = vdupq_n_s32( scaleQP );

  const int endX       = maxX + 1;
  const int maskCoeffs = endX & 3;

  static const int32_t maskTbl[4][4] = {
    {  0,  0,  0,  0 },
    { -1,  0,  0,  0 },
    { -1, -1,  0,  0 },
    { -1, -1, -1,  0 },
  };
  const int32x4_t vMask = vld1q_s32( maskTbl[maskCoeffs] );

  for( int y = 0; y <= maxY; y++ )
  {
    int x = 0;
    int n = y * width;

    for( ; x + 3 < endX; x += 4, n += 4 )
    {
      if( UseScalingList )
      {
        vScale = vmulq_s32( vdupq_n_s32( scaleQP ), vld1q_s32( &piDequantCoef[n] ) );
      }

      int32x4_t vLevel = QCoef_16bit
        ? vmovl_s16( vld1_s16( reinterpret_cast<const int16_t*>( &piQCoef[x + y * piQCfStride] ) ) )
        : vld1q_s32( reinterpret_cast<const int32_t*>( &piQCoef[x + y * piQCfStride] ) );

      vLevel = vmaxq_s32( vLevel, vInputMin );
      vLevel = vminq_s32( vLevel, vInputMax );
      vLevel = vmulq_s32( vLevel, vScale );
      vLevel = vrshlq_s32( vLevel, vShift );

      vLevel = vmaxq_s32( vLevel, vTransformMin );
      vLevel = vminq_s32( vLevel, vTransformMax );

      vst1q_s32( &piCoef[n], vLevel );
    }

    if( x < endX )
    {
      CHECKD( endX - x >= 4 || endX - x != maskCoeffs, "wrong mask for remaining coeffs" << ( endX - x ) << " " << maskCoeffs );

      if( UseScalingList )
      {
        vScale = vmulq_s32( vdupq_n_s32( scaleQP ), vld1q_s32( &piDequantCoef[n] ) );
      }

      int32x4_t vLevel = QCoef_16bit
        ? vmovl_s16( vld1_s16( reinterpret_cast<const int16_t*>( &piQCoef[x + y * piQCfStride] ) ) )
        : vld1q_s32( reinterpret_cast<const int32_t*>( &piQCoef[x + y * piQCfStride] ) );

      vLevel = vandq_s32( vLevel, vMask );

      vLevel = vmaxq_s32( vLevel, vInputMin );
      vLevel = vminq_s32( vLevel, vInputMax );
      vLevel = vmulq_s32( vLevel, vScale );
      vLevel = vrshlq_s32( vLevel, vShift );

      vLevel = vmaxq_s32( vLevel, vTransformMin );
      vLevel = vminq_s32( vLevel, vTransformMax );

      if( maskCoeffs <= 2 )
        vst1_s32( &piCoef[n], vget_low_s32( vLevel ) );
      else
        vst1q_s32( &piCoef[n], vLevel );
    }
  }
}

template<class T>
static void DeQuantCoreNeon( const SizeType width,
                             const int      maxX,
                             const int      maxY,
                             const int      scale,
                             const T* const piQCoef,
                             const size_t   piQCfStride,
                             TCoeff* const  piCoef,
                             const int      rightShift,
                             const int      inputMaximum,
                             const TCoeff   transformMaximum )
{
  if( maxX < 2 )
  {
    Quant::DeQuantCore<T>( width, maxX, maxY, scale, piQCoef, piQCfStride, piCoef, rightShift, inputMaximum, transformMaximum );
  }
  else
  {
    DeQuantImplNeon<T, false>( width, maxX, maxY, scale, nullptr, piQCoef, piQCfStride, piCoef, rightShift, inputMaximum, transformMaximum );
  }
}

template<class T>
static void DeQuantScalingCoreNeon( const SizeType width,
                                    const int      maxX,
                                    const int      maxY,
                                    const int      scaleQP,
                                    const int*     piDequantCoef,
                                    const T* const piQCoef,
                                    const size_t   piQCfStride,
                                    TCoeff* const  piCoef,
                                    const int      rightShift,
                                    const int      inputMaximum,
                                    const TCoeff   transformMaximum )
{
  if( maxX < 2 )
  {
    Quant::DeQuantScalingCore<T>( width, maxX, maxY, scaleQP, piDequantCoef, piQCoef, piQCfStride, piCoef, rightShift, inputMaximum, transformMaximum );
  }
  else
  {
    DeQuantImplNeon<T, true>( width, maxX, maxY, scaleQP, piDequantCoef, piQCoef, piQCfStride, piCoef, rightShift, inputMaximum, transformMaximum );
  }
}

template<>
void Quant::_initQuantARM<NEON>()
{
  DeQuant           = DeQuantCoreNeon<TCoeffSig>;
  DeQuantPCM        = DeQuantCoreNeon<TCoeff>;
  DeQuantScaling    = DeQuantScalingCoreNeon<TCoeffSig>;
  DeQuantScalingPCM = DeQuantScalingCoreNeon<TCoeff>;
}

} // namespace vvdec

#endif
