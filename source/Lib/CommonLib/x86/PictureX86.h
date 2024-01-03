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

/** \file     InterPredX86.h
    \brief    SIMD for InterPrediction
*/

//! \ingroup CommonLib
//! \{


#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "CommonLib/Picture.h"

#if ENABLE_SIMD_OPT_PICTURE
#ifdef TARGET_SIMD_X86

namespace vvdec
{

template<X86_VEXT vext>
void paddPicBorderLeftRightSIMD(Pel *pi, ptrdiff_t stride,int width,int xmargin,int height)
{
  __m128i xleft;
  __m128i xright;

  for (int i=1;i<height-1;i++)
  {
    xleft  = _mm_set1_epi16( pi[0] );
    xright = _mm_set1_epi16( pi[width - 1] );

    int temp=xmargin;
    int x=0;
    while ((temp >> 3) > 0)
    {
      _mm_storeu_si128((__m128i*)&pi[-xmargin + x], xleft);
      _mm_storeu_si128((__m128i*)&pi[width  + x], xright);
      x+=8;
      temp-=8;
    }
    while ((temp >> 2) > 0)
    {
      _mm_storeu_si64((__m128i*)&pi[-xmargin + x], xleft);
      _mm_storeu_si64((__m128i*)&pi[width  + x], xright);
      x+=4;
      temp-=4;
    }
    while ((temp >> 1) > 0)
    {
      _mm_storeu_si32(( __m128i * )&pi[-xmargin + x], xleft);
      _mm_storeu_si32(( __m128i * )&pi[width  + x], xright);
      x+=2;
      temp-=2;
    }
    pi += stride;
  }
}

template<X86_VEXT vext>
void paddPicBorderBotSIMD( Pel *pi, ptrdiff_t stride, int width, int xmargin, int ymargin )
{
  paddPicBorderLeftRightSIMD<vext>( pi, stride, width, xmargin, 3 );

  pi -= xmargin;

  __m128i x8;
#ifdef USE_AVX2
  __m256i v16;
#endif
  int j, temp;
  for( int i = 1; i <= ymargin; i++ )
  {
    j = 0;
    temp = width + ( xmargin << 1 );
#ifdef USE_AVX2
    while( ( temp >> 4 ) > 0 )
    {
      v16 = _mm256_loadu_si256( ( __m256i* )( pi + j ) );
      _mm256_storeu_si256( ( __m256i* )( pi + j + i * stride ), v16 );
      j = j + 16;
      temp = temp - 16;
    }
#endif
    while( ( temp >> 3 ) > 0 )
    {
      x8 = _mm_loadu_si128( ( __m128i* )( pi + j ) );
      _mm_storeu_si128( ( __m128i* )( pi + j + i * stride ), x8 );
      j = j + 8;
      temp = temp - 8;
    }
    while( ( temp >> 2 ) > 0 )
    {
      x8 = _mm_loadu_si64( ( __m128i * )( pi + j ) );
      _mm_storeu_si64( ( __m128i* )( pi + j + i * stride ), x8 );
      j = j + 4;
      temp = temp - 4;
    }
    while( ( temp >> 1 ) > 0 )
    {
      x8 = _mm_loadu_si32( ( __m128i * )( pi + j ) );
      _mm_storeu_si32( ( __m128i * )( pi + j + i * stride ), x8 );
      j += 2;
      temp -= 2;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void paddPicBorderTopSIMD( Pel *pi, ptrdiff_t stride, int width, int xmargin, int ymargin )
{
  paddPicBorderLeftRightSIMD<vext>( pi, stride, width, xmargin, 3 );

  pi -= xmargin;

  __m128i x8;
#ifdef USE_AVX2
  __m256i v16;
#endif
  int j, temp;
  for( int i = 1; i <= ymargin; i++ )
  {
    j = 0;
    temp = width + ( xmargin << 1 );
#ifdef USE_AVX2
    while( ( temp >> 4 ) > 0 )
    {
      v16 = _mm256_loadu_si256( ( __m256i* )( pi + j ) );
      _mm256_storeu_si256( ( __m256i* )( pi + j - i * stride ), v16 );
      j = j + 16;
      temp = temp - 16;
    }
#endif
    while( ( temp >> 3 ) > 0 )
    {
      x8 = _mm_loadu_si128( ( __m128i* )( pi + j ) );
      _mm_storeu_si128( ( __m128i* )( pi + j - i * stride ), x8 );
      j = j + 8;
      temp = temp - 8;
    }
    while( ( temp >> 2 ) > 0 )
    {
      x8 = _mm_loadu_si64( ( __m128i * )( pi + j ) );
      _mm_storeu_si64( ( __m128i* )( pi + j - i * stride ), x8 );
      j = j + 4;
      temp = temp - 4;
    }
    while( ( temp >> 1 ) > 0 )
    {
      x8 = _mm_loadu_si32( ( __m128i * )( pi + j ) );
      _mm_storeu_si32( ( __m128i * )( pi + j - i * stride ), x8 );
      j += 2;
      temp -= 2;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void Picture::_initPictureX86()
{
  paddPicBorderBot       = paddPicBorderBotSIMD<vext>;
  paddPicBorderTop       = paddPicBorderTopSIMD<vext>;
  paddPicBorderLeftRight = paddPicBorderLeftRightSIMD<vext>;
}
template void Picture::_initPictureX86<SIMDX86>();

}

#endif // TARGET_SIMD_X86
#endif
//! \}
