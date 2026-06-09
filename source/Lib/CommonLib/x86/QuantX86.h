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

/** \file     QuantX86.h
    \brief    SIMD for Quant/Dequant
*/

#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "CommonLib/Quant.h"

namespace vvdec
{

#if ENABLE_SIMD_OPT_QUANT
#ifdef TARGET_SIMD_X86

template<X86_VEXT vext, class T, bool UseScalingList, bool RightShiftPositive>
static inline void DeQuantImplSIMD( const SizeType width,
                                    const int      maxX,
                                    const int      maxY,
                                    const int      scaleQP,
                                    const int*     piDequantCoef,   // unused if UseScalingList == false
                                    const T* const piQCoef,
                                    const size_t   piQCfStride,
                                    TCoeff* const  piCoef,
                                    const int      rightShift,
                                    const int      inputMaximum,
                                    const TCoeff   transformMaximum )
{
  static_assert( sizeof( piQCoef[0] ) == sizeof( int16_t ) || sizeof( piQCoef[0] ) == sizeof( int32_t ), "wrong coeff type" );

  constexpr static bool QCoef_16bit = sizeof( piQCoef[0] ) == sizeof( int16_t );

  const int    inputMinimum     = -( inputMaximum + 1 );
  const TCoeff transformMinimum = -( transformMaximum + 1 );

  const int iAdd  = RightShiftPositive ? 1 << ( rightShift - 1 ) : 0;
  const int shift = RightShiftPositive ? rightShift : -rightShift;

  const __m128i vInputMin     = _mm_set1_epi32( inputMinimum );
  const __m128i vInputMax     = _mm_set1_epi32( inputMaximum );
  const __m128i vTransformMin = _mm_set1_epi32( transformMinimum );
  const __m128i vTransformMax = _mm_set1_epi32( transformMaximum );

  const __m128i vAdd   = _mm_set1_epi32( iAdd );
  const __m128i vShift = _mm_set_epi64x( 0, shift );
        __m128i vScale = _mm_set1_epi32( scaleQP );

#if USE_AVX2
  const __m256i xvInputMin     = _mm256_set1_epi32( inputMinimum );
  const __m256i xvInputMax     = _mm256_set1_epi32( inputMaximum );
  const __m256i xvTransformMin = _mm256_set1_epi32( transformMinimum );
  const __m256i xvTransformMax = _mm256_set1_epi32( transformMaximum );

  const __m256i xvAdd   = _mm256_set1_epi32( iAdd );
        __m256i xvScale = _mm256_set1_epi32( scaleQP );
#endif   // USE_AVX2

  const int endX       = maxX + 1;
  const int maskCoeffs = endX & 3;   // number of coefficients in the last vector read
  // clang-format off
  const __m128i vMask = maskCoeffs == 3 ? _mm_set_epi32( 0, -1, -1, -1 ) :
                      ( maskCoeffs == 2 ? _mm_set_epi32( 0, 0, -1, -1 )
                                        : _mm_set_epi32( 0, 0, 0, -1 ) );
  // clang-format on

  for( int y = 0; y <= maxY; y++ )
  {
    int x = 0;
    int n = y * width;

#if USE_AVX2
    for( ; x + 7 < endX; x += 8, n += 8 )
    {
      if( UseScalingList )
      {
        xvScale = _mm256_set1_epi32( scaleQP );
        xvScale = _mm256_mullo_epi32( xvScale, _mm256_loadu_si256( (__m256i*) &piDequantCoef[n] ) );
      }

      __m256i xvLevel = QCoef_16bit ? _mm256_cvtepi16_epi32( _mm_loadu_si128( (__m128i*) &piQCoef[x + y * piQCfStride] ) )   // 16 bit coeffs
                                    : _mm256_loadu_si256( (__m256i*) &piQCoef[x + y * piQCfStride] );                        // 32 bit coeffs

      xvLevel = _mm256_max_epi32( xvLevel, xvInputMin );
      xvLevel = _mm256_min_epi32( xvLevel, xvInputMax );

      xvLevel = _mm256_mullo_epi32( xvLevel, xvScale );
      if( RightShiftPositive )
      {
        xvLevel = _mm256_add_epi32( xvLevel, xvAdd );
        xvLevel = _mm256_sra_epi32( xvLevel, vShift );
      }
      else
      {
        xvLevel = _mm256_sll_epi32( xvLevel, vShift );
      }

      xvLevel = _mm256_max_epi32( xvLevel, xvTransformMin );
      xvLevel = _mm256_min_epi32( xvLevel, xvTransformMax );

      _mm256_storeu_si256( (__m256i*) &piCoef[n], xvLevel );
    }
#endif   // USE_AVX2

    for( ; x + 3 < endX; x += 4, n += 4 )
    {
      if( UseScalingList )
      {
        vScale = _mm_set1_epi32( scaleQP );
        vScale = _mm_mullo_epi32( vScale, _mm_loadu_si128( (__m128i*) &piDequantCoef[n] ) );
      }

      __m128i vLevel = QCoef_16bit ? _mm_cvtepi16_epi32( _mm_loadu_si64( &piQCoef[x + y * piQCfStride] ) )   // 16 bit coeffs
                                   : _mm_loadu_si128( (__m128i*) &piQCoef[x + y * piQCfStride] );            // 32 bit coeffs

      vLevel = _mm_max_epi32( vLevel, vInputMin );
      vLevel = _mm_min_epi32( vLevel, vInputMax );

      vLevel = _mm_mullo_epi32( vLevel, vScale );
      if( RightShiftPositive )
      {
        vLevel = _mm_add_epi32( vLevel, vAdd );
        vLevel = _mm_sra_epi32( vLevel, vShift );
      }
      else
      {
        vLevel = _mm_sll_epi32( vLevel, vShift );
      }

      vLevel = _mm_max_epi32( vLevel, vTransformMin );
      vLevel = _mm_min_epi32( vLevel, vTransformMax );

      _mm_storeu_si128( (__m128i*) &piCoef[n], vLevel );
    }

#if 0   // dequant remaining coefficients using scalar code
    (void)vMask;
    for( ; x < endX; x++, n++ )
    {
      const TCoeff level = piQCoef[x + y * piQCfStride];
      if( !level )
      {
        continue;
      }

      const int        scale     = UseScalingList ? piDequantCoef[n] * scaleQP   //
                                                  : scaleQP;
      const TCoeff     clipQCoef = TCoeff( Clip3<Intermediate_Int>( inputMinimum, inputMaximum, level ) );
      Intermediate_Int iCoeffQ   = RightShiftPositive ? ( Intermediate_Int( clipQCoef ) * scale + iAdd ) >> rightShift   //
                                                      : ( Intermediate_Int( clipQCoef ) * scale ) * ( 1 << shift );

      piCoef[n] = TCoeff( Clip3<Intermediate_Int>( transformMinimum, transformMaximum, iCoeffQ ) );
    }

#else   // dequant remaining coefficients using SSE

    if( x < endX )
    {
      CHECKD( endX - x >= 4 || endX - x != maskCoeffs, "wrong mask for remaining coeffs" << ( endX - x ) << " " << maskCoeffs );

      if( UseScalingList )
      {
        vScale = _mm_set1_epi32( scaleQP );
        vScale = _mm_mullo_epi32( vScale, _mm_loadu_si128( (__m128i*) &piDequantCoef[n] ) );
        vScale = _mm_and_si128( vScale, vMask );
      }

      __m128i vLevel = QCoef_16bit ? _mm_cvtepi16_epi32( _mm_loadu_si64( &piQCoef[x + y * piQCfStride] ) )   // 16 bit coeffs
                                   : _mm_loadu_si128( (__m128i*) &piQCoef[x + y * piQCfStride] );            // 32 bit coeffs

      vLevel = _mm_and_si128( vLevel, vMask );

      vLevel = _mm_max_epi32( vLevel, vInputMin );
      vLevel = _mm_min_epi32( vLevel, vInputMax );

      vLevel = _mm_mullo_epi32( vLevel, vScale );
      if( RightShiftPositive )
      {
        vLevel = _mm_add_epi32( vLevel, vAdd );
        vLevel = _mm_sra_epi32( vLevel, vShift );
      }
      else
      {
        vLevel = _mm_sll_epi32( vLevel, vShift );
      }

      vLevel = _mm_max_epi32( vLevel, vTransformMin );
      vLevel = _mm_min_epi32( vLevel, vTransformMax );

      if( maskCoeffs <= 2 )
      {
        _mm_storeu_si64( (__m128i*) &piCoef[n], vLevel );
      }
      else
      {
        _mm_storeu_si128( (__m128i*) &piCoef[n], vLevel );
      }
    }
#endif
  }
}

template<X86_VEXT vext, class T>
static void DeQuantCoreSIMD( const SizeType width,
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
    Quant::DeQuantCore<T>(width,
                   maxX,
                   maxY,
                   scale,
                   piQCoef,
                   piQCfStride,
                   piCoef,
                   rightShift,
                   inputMaximum,
                   transformMaximum );
  }
  else if( rightShift > 0 )
  {
    DeQuantImplSIMD<vext, T, false, true>( width, maxX, maxY, scale, nullptr, piQCoef, piQCfStride, piCoef, rightShift, inputMaximum, transformMaximum );
  }
  else
  {
    DeQuantImplSIMD<vext, T, false, false>( width, maxX, maxY, scale, nullptr, piQCoef, piQCfStride, piCoef, rightShift, inputMaximum, transformMaximum );
  }
}

template<X86_VEXT vext, class T>
static void DeQuantScalingCoreSIMD( const SizeType width,
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
    Quant::DeQuantScalingCore<T>(width,
                          maxX,
                          maxY,
                          scaleQP,
                          piDequantCoef,
                          piQCoef,
                          piQCfStride,
                          piCoef,
                          rightShift,
                          inputMaximum,
                          transformMaximum );
  }
  else if( rightShift > 0 )
  {
    DeQuantImplSIMD<vext, T, true, true>( width, maxX, maxY, scaleQP, piDequantCoef, piQCoef, piQCfStride, piCoef, rightShift, inputMaximum, transformMaximum );
  }
  else
  {
    DeQuantImplSIMD<vext, T, true, false>( width, maxX, maxY, scaleQP, piDequantCoef, piQCoef, piQCfStride, piCoef, rightShift, inputMaximum, transformMaximum );
  }
}

template<X86_VEXT vext>
void Quant::_initQuantX86()
{
  DeQuant           = DeQuantCoreSIMD<vext, TCoeffSig>;
  DeQuantPCM        = DeQuantCoreSIMD<vext, TCoeff>;
  DeQuantScaling    = DeQuantScalingCoreSIMD<vext, TCoeffSig>;
  DeQuantScalingPCM = DeQuantScalingCoreSIMD<vext, TCoeff>;
}
template void Quant::_initQuantX86<SIMDX86>();

#endif  // TARGET_SIMD_X86
#endif  // ENABLE_SIMD_OPT_QUANT

}   // namespace vvdec
