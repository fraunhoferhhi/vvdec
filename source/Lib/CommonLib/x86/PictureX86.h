/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVdec

(c) Copyright (2018-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVdec (“Fraunhofer Versatile Video Decoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Decoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Decoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Decoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Decoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Decoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Decoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Decoding Library. If You use the Fraunhofer Versatile Video Decoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Decoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Decoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Decoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Decoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
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



#define _mm_storeu_si32(p, a) (void)(*(int*)(p) = _mm_cvtsi128_si32((a)))
#ifndef _mm_loadu_si32
#define _mm_loadu_si32(p) _mm_cvtsi32_si128(*(unsigned int const *)(p))
#endif


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
      _mm_storel_epi64((__m128i*)&pi[-xmargin + x], xleft);
      _mm_storel_epi64((__m128i*)&pi[width  + x], xright);
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
      x8 = _mm_loadl_epi64( ( __m128i * )( pi + j ) );
      _mm_storel_epi64( ( __m128i* )( pi + j + i * stride ), x8 );
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
      x8 = _mm_loadl_epi64( ( __m128i * )( pi + j ) );
      _mm_storel_epi64( ( __m128i* )( pi + j - i * stride ), x8 );
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
  paddPicBorderBot = paddPicBorderBotSIMD<vext>;
  paddPicBorderTop = paddPicBorderTopSIMD<vext>;
  paddPicBorderLeftRight = paddPicBorderLeftRightSIMD<vext>;
}
template void Picture::_initPictureX86<SIMDX86>();

#endif // TARGET_SIMD_X86
#endif
//! \}
