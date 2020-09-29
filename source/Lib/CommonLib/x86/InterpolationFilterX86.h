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

/**
 * \file
 * \brief Implementation of InterpolationFilter class
 */
//#define USE_AVX2
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "CommonDefX86.h"
#include "../Rom.h"
#include "../InterpolationFilter.h"

#include "../Unit.h"

//#include "../ChromaFormat.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86

#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

#if USE_AVX2 && !defined( _mm256_set_m128i )
#define VVCLIB_OWN_mm256_set_m128i
#define _mm256_set_m128i( v0, v1 ) _mm256_inserti128_si256( _mm256_castsi128_si256( v1 ), ( v0 ), 1 )

#endif

// ===========================
// Full-pel copy 8-bit/16-bit
// ===========================
template<typename Tsrc, int N, bool isFirst, bool isLast>
static void fullPelCopySSE( const ClpRng& clpRng, const void*_src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height )
{
  Tsrc* src = (Tsrc*)_src;

  int headroom = IF_INTERNAL_PREC - clpRng.bd;
  int headroom_offset = 1 << ( headroom - 1 );
  int offset   = IF_INTERNAL_OFFS;
  __m128i voffset  = _mm_set1_epi16( offset );
  __m128i voffset_headroom  = _mm_set1_epi16( headroom_offset );
  __m128i vmin = _mm_set1_epi16( clpRng.min() );
  __m128i vmax = _mm_set1_epi16( clpRng.max() );

  __m128i vsrc, vsum;

  for( int row = 0; row < height; row++ )
  {
    for( int col = 0; col < width; col+=N )
    {
      _mm_prefetch( (const char*)src+2*srcStride, _MM_HINT_T0 );
      _mm_prefetch( (const char*)src+( width>>1 ) + 2*srcStride, _MM_HINT_T0 );
      _mm_prefetch( (const char*)src+width-1 + 2*srcStride, _MM_HINT_T0 );
      for( int i=0; i<N; i+=8 )
      {
        if( sizeof( Tsrc )==1 )
        {
          vsrc = _mm_cvtepu8_epi16( _mm_loadu_si128( ( __m128i const * )&src[col+i] ) );
        }
        else
        {
          vsrc = _mm_loadu_si128( ( __m128i const * )&src[col+i] );
        }

        if( isFirst == isLast )
        {
          vsum = vsrc;
        }
        else if( isFirst )
        {
          vsrc = _mm_slli_epi16( vsrc, headroom );
          vsum = _mm_sub_epi16( vsrc, voffset );
        }
        else
        {
          vsrc = _mm_add_epi16( vsrc, voffset );
          vsrc = _mm_add_epi16( vsrc, voffset_headroom );
          vsrc = _mm_srai_epi16( vsrc, headroom );
          vsum = vsrc;
        }

        if( isLast )
        {
          vsum = _mm_max_epi16( _mm_min_epi16( vmax, vsum ), vmin );
        }

        _mm_storeu_si128( ( __m128i * )&dst[col+i], vsum );
      }
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<typename Tsrc, bool isFirst, bool isLast>
static void fullPelCopySSE_M4( const ClpRng& clpRng, const void*_src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height )
{
  Tsrc* src = (Tsrc*)_src;

  int headroom = IF_INTERNAL_PREC - clpRng.bd;
  int headroom_offset = 1 << ( headroom - 1 );
  int offset   = IF_INTERNAL_OFFS;
  __m128i voffset  = _mm_set1_epi16( offset );
  __m128i voffset_headroom  = _mm_set1_epi16( headroom_offset );
  __m128i vmin = _mm_set1_epi16( clpRng.min() );
  __m128i vmax = _mm_set1_epi16( clpRng.max() );

  __m128i vsrc, vsum;

  for( int row = 0; row < height; row++ )
  {
    for( int col = 0; col < width; col+=4 )
    {
      _mm_prefetch( (const char*)src+2*srcStride, _MM_HINT_T0 );
      _mm_prefetch( (const char*)src+( width>>1 ) + 2*srcStride, _MM_HINT_T0 );
      _mm_prefetch( (const char*)src+width-1 + 2*srcStride, _MM_HINT_T0 );

      if( sizeof( Tsrc )==1 )
      {
        vsrc = _mm_cvtepu8_epi16( _mm_loadl_epi64( ( __m128i const * )&src[col] ) );
      }
      else
      {
        vsrc = _mm_loadl_epi64( ( __m128i const * )&src[col] );
      }

      if( isFirst == isLast )
      {
        vsum = vsrc;
      }
      else if( isFirst )
      {
        vsrc = _mm_slli_epi16( vsrc, headroom );
        vsum = _mm_sub_epi16( vsrc, voffset );
      }
      else
      {
        vsrc = _mm_add_epi16( vsrc, voffset );
        vsrc = _mm_add_epi16( vsrc, voffset_headroom );
        vsrc = _mm_srai_epi16( vsrc, headroom );
        vsum = vsrc;
      }

      if( isLast )
      {
        vsum = _mm_max_epi16( _mm_min_epi16( vmax, vsum ), vmin );
      }

      _mm_storel_epi64( ( __m128i * )&dst[col], vsum );
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<typename Tsrc, int N, bool isFirst, bool isLast>
static void fullPelCopyAVX2( const ClpRng& clpRng, const void*_src, ptrdiff_t srcStride, short *dst, ptrdiff_t dstStride, int width, int height )
{
#ifdef USE_AVX2
  Tsrc* src = (Tsrc*)_src;

  int headroom = IF_INTERNAL_PREC - clpRng.bd;
  int offset   = 1 << ( headroom - 1 );
  int internal_offset = IF_INTERNAL_OFFS;

  __m256i vinternal_offset = _mm256_set1_epi16( internal_offset );
  __m256i vheadroom_offset = _mm256_set1_epi16( offset );
  __m256i vmin = _mm256_set1_epi16( clpRng.min() );
  __m256i vmax = _mm256_set1_epi16( clpRng.max() );

  __m256i vsrc, vsum;


  for( int row = 0; row < height; row++ )
  {
    for( int col = 0; col < width; col+=N )
    {
      _mm_prefetch( (const char*)( src+3*srcStride ), _MM_HINT_T0 );
      _mm_prefetch( (const char*)( src+( width>>1 ) + 3*srcStride ), _MM_HINT_T0 );
      _mm_prefetch( (const char*)( src+width-1 + 3*srcStride ), _MM_HINT_T0 );
      for( int i=0; i<N; i+=16 )
      {
        if( sizeof( Tsrc )==1 )
        {
          vsrc = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )&src[col+i] ) );
        }
        else
        {
          vsrc = _mm256_loadu_si256( ( const __m256i * )&src[col+i] );
        }

        if( isFirst == isLast )
        {
          vsum = vsrc;
        }
        else if( isFirst )
        {
          vsrc = _mm256_slli_epi16( vsrc, headroom );
          vsum = _mm256_sub_epi16( vsrc, vinternal_offset );
        }
        else
        {
          vsrc = _mm256_add_epi16( vsrc, vinternal_offset );
          vsrc = _mm256_add_epi16( vsrc, vheadroom_offset );
          vsrc = _mm256_srai_epi16( vsrc, headroom );
          vsum = vsrc;
        }

        if( isLast )
        {
          vsum = _mm256_max_epi16( _mm256_min_epi16( vmax, vsum ), vmin );
        }

        _mm256_storeu_si256( ( __m256i * )&dst[col+i], vsum );
      }
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void fullPelCopyDMVR_SSE( const int16_t* src, ptrdiff_t srcStride, int16_t* dst, ptrdiff_t dstStride, int width, int height, const ClpRng& clpRng )
{
  const int shift = IF_INTERNAL_PREC_BILINEAR - clpRng.bd;

  CHECKD( shift < 0, "Only bit-depths of up to 10 supported!" );

#if USE_AVX2
  if( vext >= AVX2 && ( ( width - 4 ) & 15 ) == 0 )
  {
    for( int row = 0; row < height; row++ )
    {
      int x = 0;
      for( ; x < width - 4; x += 16 )
      {
        __m256i vmm;
        vmm = _mm256_loadu_si256( ( const __m256i * ) &src[x] );
        vmm = _mm256_slli_epi16 ( vmm, shift );
        _mm256_storeu_si256     ( ( __m256i * )&dst[x], vmm );
      }
      
      __m128i xmm;
      xmm = _mm_loadl_epi64   ( ( const __m128i * ) &src[x] );
      xmm = _mm_slli_epi16    ( xmm, shift );
      _mm_storel_epi64        ( ( __m128i * )&dst[x], xmm );

      src += srcStride;
      dst += dstStride;
    }
  }
  else
#endif
  {
    CHECKD( ( ( width - 4 ) & 7 ), "Unsupported size " << width );

    __m128i xmm;

    for( int row = 0; row < height; row++ )
    {
      int x = 0;
      for( ; x < width - 4; x += 8 )
      {
        xmm = _mm_loadu_si128( ( const __m128i * ) &src[x] );
        xmm = _mm_slli_epi16 ( xmm, shift );
        _mm_storeu_si128     ( ( __m128i * )&dst[x], xmm );
      }

      xmm = _mm_loadl_epi64( ( const __m128i * ) &src[x] );
      xmm = _mm_slli_epi16 ( xmm, shift );
      _mm_storel_epi64     ( ( __m128i * )&dst[x], xmm );

      src += srcStride;
      dst += dstStride;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}


template<X86_VEXT vext, bool isFirst, bool isLast>
static void simdFilterCopy( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, bool biMCForDMVR )
{
  if( biMCForDMVR )
  {
    CHECKD( !isFirst || isLast, "Should never happen!" );
    if( IF_INTERNAL_PREC_BILINEAR < clpRng.bd )
    {
      InterpolationFilter::filterCopy<isFirst, isLast>( clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR );
    }
    else
    {
      fullPelCopyDMVR_SSE<vext>( src, srcStride, dst, dstStride, width, height, clpRng );
    }
  }
  else
  if( vext >= AVX2 && ( width % 16 ) == 0 )
  {
    fullPelCopyAVX2<Pel, 16, isFirst, isLast >(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if( ( width % 16 ) == 0 )
  {
    fullPelCopySSE<Pel, 16, isFirst, isLast >(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if( ( width % 8 ) == 0 )
  {
    fullPelCopySSE<Pel, 8, isFirst, isLast>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if( ( width % 4 ) == 0 )
  {
    fullPelCopySSE_M4<Pel, isFirst, isLast>( clpRng, src, srcStride, dst, dstStride, width, height );
  }
  else
  { //Scalar
    InterpolationFilter::filterCopy<isFirst, isLast>( clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR );
  }
}


// SIMD interpolation horizontal, block width modulo 4
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM4( const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  _mm_prefetch( (const char*)src + srcStride, _MM_HINT_T0 );
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
  __m128i vcoeffh = _mm_loadu_si128( ( __m128i const * )coeff );

  __m128i vzero, vshufc0, vshufc1;
  __m128i vsum;

  if( N != 8 ){
    vcoeffh = _mm_shuffle_epi32( vcoeffh, 0x44 );
    vzero = _mm_setzero_si128();
    vshufc0 = _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
    vshufc1 = _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
  }

  vzero = _mm_setzero_si128();

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)src + 2 * srcStride, _MM_HINT_T0 );
    for( int col = 0; col < width; col += 4 )
    {
      if( N == 8 )
      {
        __m128i vtmp[2];
        for( int i = 0; i < 4; i += 2 )
        {
          __m128i vsrc0 = _mm_loadu_si128( ( __m128i const * )&src[col + i] );
          __m128i vsrc1 = _mm_loadu_si128( ( __m128i const * )&src[col + i + 1] );
          vsrc0 = _mm_madd_epi16( vsrc0, vcoeffh );
          vsrc1 = _mm_madd_epi16( vsrc1, vcoeffh );
          vtmp[i / 2] = _mm_hadd_epi32( vsrc0, vsrc1 );
        }
        vsum = _mm_hadd_epi32( vtmp[0], vtmp[1] );
      }
      else
      {
        __m128i vtmp0, vtmp1;
        __m128i vsrc = _mm_loadu_si128( ( __m128i const * )&src[col] );
        vtmp0 = _mm_shuffle_epi8( vsrc, vshufc0 );
        vtmp1 = _mm_shuffle_epi8( vsrc, vshufc1 );

        vtmp0 = _mm_madd_epi16( vtmp0, vcoeffh );
        vtmp1 = _mm_madd_epi16( vtmp1, vcoeffh );
        vsum = _mm_hadd_epi32( vtmp0, vtmp1 );
      }

      vsum = _mm_add_epi32( vsum, voffset );
      vsum = _mm_srai_epi32( vsum, shift );
      vsum = _mm_packs_epi32( vsum, vzero );

      if( shiftBack )
      { //clip
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }
      _mm_storel_epi64( ( __m128i * )&dst[col], vsum );
    }
    src += srcStride;
    dst += dstStride;
  }
}


// SIMD interpolation horizontal, block width modulo 8
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM8( const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  const int filterSpan = ( N - 1 );
  _mm_prefetch( (const char*)src + srcStride, _MM_HINT_T0 );
  _mm_prefetch( (const char*)src + ( width >> 1 ) + srcStride, _MM_HINT_T0 );
  _mm_prefetch( (const char*)src + width + filterSpan + srcStride, _MM_HINT_T0 );

  __m128i voffset  = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
  __m128i vcoeffh  = _mm_loadu_si128( ( __m128i const * )coeff );

  __m128i vshufc0, vshufc1;
  __m128i vsum, vsuma, vsumb;

  if( N != 8 ){
    vcoeffh = _mm_shuffle_epi32( vcoeffh, 0x44 );
    vshufc0 = _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
    vshufc1 = _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
  }

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)src + 2 * srcStride, _MM_HINT_T0 );
    _mm_prefetch( (const char*)src + ( width >> 1 ) + 2 * srcStride, _MM_HINT_T0 );
    _mm_prefetch( (const char*)src + width + filterSpan + 2 * srcStride, _MM_HINT_T0 );
    for( int col = 0; col < width; col += 8 )
    {
      if( N == 8 )
      {
        __m128i vtmp[4];
        for( int i = 0; i < 8; i += 2 ){
          __m128i vsrc0 = _mm_loadu_si128( ( __m128i const * )&src[col + i] );
          __m128i vsrc1 = _mm_loadu_si128( ( __m128i const * )&src[col + i + 1] );
          vsrc0 = _mm_madd_epi16( vsrc0, vcoeffh );
          vsrc1 = _mm_madd_epi16( vsrc1, vcoeffh );
          vtmp[i / 2] = _mm_hadd_epi32( vsrc0, vsrc1 );
        }
        vsuma = _mm_hadd_epi32( vtmp[0], vtmp[1] );
        vsumb = _mm_hadd_epi32( vtmp[2], vtmp[3] );
      }
      else
      {
        __m128i vtmp00, vtmp01, vtmp10, vtmp11;
        __m128i vsrc0 = _mm_loadu_si128( ( __m128i const * )&src[col] );
        __m128i vsrc1 = _mm_loadu_si128( ( __m128i const * )&src[col + 4] );
        vtmp00 = _mm_shuffle_epi8( vsrc0, vshufc0 );
        vtmp01 = _mm_shuffle_epi8( vsrc0, vshufc1 );
        vtmp10 = _mm_shuffle_epi8( vsrc1, vshufc0 );
        vtmp11 = _mm_shuffle_epi8( vsrc1, vshufc1 );

        vtmp00 = _mm_madd_epi16( vtmp00, vcoeffh );
        vtmp01 = _mm_madd_epi16( vtmp01, vcoeffh );
        vtmp10 = _mm_madd_epi16( vtmp10, vcoeffh );
        vtmp11 = _mm_madd_epi16( vtmp11, vcoeffh );

        vsuma = _mm_hadd_epi32( vtmp00, vtmp01 );
        vsumb = _mm_hadd_epi32( vtmp10, vtmp11 );
      }

      vsuma = _mm_add_epi32( vsuma, voffset );
      vsumb = _mm_add_epi32( vsumb, voffset );

      vsuma = _mm_srai_epi32( vsuma, shift );
      vsumb = _mm_srai_epi32( vsumb, shift );

      vsum = _mm_packs_epi32( vsuma, vsumb );

      if( shiftBack ){ //clip
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }

      _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
    }
    src += srcStride;
    dst += dstStride;
  }
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM8_AVX2( const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  const int filterSpan =/* bChromaIntl ? 2* ( N-1 ) : */( N-1 );
  _mm_prefetch( (const char*)( src+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+srcStride ), _MM_HINT_T0 );

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min() );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max() );

  __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                    0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
  __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                    0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );

#if __INTEL_COMPILER
  __m256i vcoeff[4];
#else
  __m256i vcoeff[N/2];
#endif
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  __m256i vsum;

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+width+filterSpan + 2*srcStride ), _MM_HINT_T0 );
    for( int col = 0; col < width; col+=8 )
    {
      if( N==8 )
      {
        __m128i xsrc0 = _mm_loadu_si128( (const __m128i *) &src[col] );
        __m128i xsrc1 = _mm_loadu_si128( (const __m128i *) &src[col + 4] );

        __m256i vsrc0, vsrca0, vsrca1;

        vsrc0  = _mm256_castsi128_si256( xsrc0 );
        vsrc0  = _mm256_inserti128_si256( vsrc0, xsrc1, 1 );
        xsrc0  = _mm_loadu_si128( (const __m128i *) &src[col + 8] );
        vsrca0 = _mm256_shuffle_epi8( vsrc0, vshuf0 );
        vsrca1 = _mm256_shuffle_epi8( vsrc0, vshuf1 );
        vsum   = _mm256_add_epi32( _mm256_madd_epi16( vsrca0, vcoeff[0] ), _mm256_madd_epi16( vsrca1, vcoeff[1] ) );

        vsrc0  = _mm256_castsi128_si256( xsrc1 );
        vsrc0  = _mm256_inserti128_si256( vsrc0, xsrc0, 1 );
        vsrca0 = _mm256_shuffle_epi8( vsrc0, vshuf0 );
        vsrca1 = _mm256_shuffle_epi8( vsrc0, vshuf1 );
        vsum   = _mm256_add_epi32( vsum, _mm256_add_epi32( _mm256_madd_epi16( vsrca0, vcoeff[2] ), _mm256_madd_epi16( vsrca1, vcoeff[3] ) ) );
      }
      else
      {
        __m256i vtmp02, vtmp13;

        __m256i vsrc = _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* )&src[col] ) );
        vsrc = _mm256_inserti128_si256( vsrc, _mm_loadu_si128( ( const __m128i* )&src[col+4] ), 1 );

        vtmp02 = _mm256_shuffle_epi8( vsrc, vshuf0 );
        vtmp13 = _mm256_shuffle_epi8( vsrc, vshuf1 );

        vtmp02 = _mm256_madd_epi16( vtmp02, vcoeff[0] );
        vtmp13 = _mm256_madd_epi16( vtmp13, vcoeff[1] );
        vsum   = _mm256_add_epi32( vtmp02, vtmp13 );
      }

      vsum = _mm256_add_epi32( vsum, voffset );
      vsum = _mm256_srai_epi32( vsum, shift );

      __m128i vsump = _mm256_cvtepi32_epi16x( vsum );
      if( shiftBack )
      { //clip
        vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
      }

      _mm_storeu_si128( ( __m128i * )&dst[col], vsump );
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
#if USE_AVX2

  _mm256_zeroupper();
#endif
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM16_AVX2( const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  const int filterSpan = ( N-1 );
  _mm_prefetch( (const char*)( src+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+3*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+3*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+3*srcStride ), _MM_HINT_T0 );

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m256i vibdimin   = _mm256_set1_epi16( clpRng.min() );
  __m256i vibdimax   = _mm256_set1_epi16( clpRng.max() );
  __m256i vsum, vsuma, vsumb;

  __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                    0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
  __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                    0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );
#if __INTEL_COMPILER
  __m256i vcoeff[4];
#else
  __m256i vcoeff[N/2];
#endif
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+width+filterSpan + 2*srcStride ), _MM_HINT_T0 );

    for( int col = 0; col < width; col+=16 )
    {
      if( N==8 )
      {
        __m256i vsrca0, vsrca1, vsrcb0, vsrcb1;
        __m256i vsrc0 = _mm256_loadu_si256( ( const __m256i * )&src[col] );
        __m256i vsrc1 = _mm256_loadu_si256( ( const __m256i * )&src[col + 4] );

        vsrca0 = _mm256_shuffle_epi8( vsrc0, vshuf0 );
        vsrca1 = _mm256_shuffle_epi8( vsrc0, vshuf1 );
        vsrc0  = _mm256_loadu_si256( ( const __m256i * )&src[col+8] );
        vsuma  = _mm256_add_epi32( _mm256_madd_epi16( vsrca0, vcoeff[0] ), _mm256_madd_epi16( vsrca1, vcoeff[1] ) );
        vsrcb0 = _mm256_shuffle_epi8( vsrc1, vshuf0 );
        vsrcb1 = _mm256_shuffle_epi8( vsrc1, vshuf1 );
        vsumb  = _mm256_add_epi32( _mm256_madd_epi16( vsrcb0, vcoeff[0] ), _mm256_madd_epi16( vsrcb1, vcoeff[1] ) );
        vsrc1  = _mm256_add_epi32( _mm256_madd_epi16( vsrcb0, vcoeff[2] ), _mm256_madd_epi16( vsrcb1, vcoeff[3] ) );
        vsrca0 = _mm256_shuffle_epi8( vsrc0, vshuf0 );
        vsrca1 = _mm256_shuffle_epi8( vsrc0, vshuf1 );
        vsrc0  = _mm256_add_epi32( _mm256_madd_epi16( vsrca0, vcoeff[2] ), _mm256_madd_epi16( vsrca1, vcoeff[3] ) );
        vsuma  = _mm256_add_epi32( vsuma, vsrc1 );
        vsumb  = _mm256_add_epi32( vsumb, vsrc0 );
      }
      else
      {
        __m256i vsrc0 = _mm256_loadu_si256( ( const __m256i * )&src[col] );
        __m256i vsrc1 = _mm256_loadu_si256( ( const __m256i * )&src[col+4] );

        __m256i vtmp00, vtmp01, vtmp10, vtmp11;

        vtmp00 = _mm256_shuffle_epi8( vsrc0, vshuf0 );
        vtmp01 = _mm256_shuffle_epi8( vsrc0, vshuf1 );
        vtmp10 = _mm256_shuffle_epi8( vsrc1, vshuf0 );
        vtmp11 = _mm256_shuffle_epi8( vsrc1, vshuf1 );

        vtmp00 = _mm256_madd_epi16( vtmp00, vcoeff[0] );
        vtmp01 = _mm256_madd_epi16( vtmp01, vcoeff[1] );
        vtmp10 = _mm256_madd_epi16( vtmp10, vcoeff[0] );
        vtmp11 = _mm256_madd_epi16( vtmp11, vcoeff[1] );

        vsuma = _mm256_add_epi32( vtmp00, vtmp01 );
        vsumb = _mm256_add_epi32( vtmp10, vtmp11 );
      }

      vsuma = _mm256_add_epi32( vsuma, voffset );
      vsumb = _mm256_add_epi32( vsumb, voffset );
      vsuma = _mm256_srai_epi32( vsuma, shift );
      vsumb = _mm256_srai_epi32( vsumb, shift );
      vsum  = _mm256_packs_epi32( vsuma, vsumb );

      if( shiftBack )
      { //clip
        vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
      }

      _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
#if USE_AVX2

  _mm256_zeroupper();
#endif
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM4( const int16_t *src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  _mm_prefetch( (const char *) &src[0 * srcStride], _MM_HINT_T0 );
  _mm_prefetch( (const char *) &src[1 * srcStride], _MM_HINT_T0 );
  if( N >= 2 )
  {
    _mm_prefetch( (const char *) &src[2 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[3 * srcStride], _MM_HINT_T0 );
  }
  if( N >= 6 )
  {
    _mm_prefetch( (const char *) &src[4 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[5 * srcStride], _MM_HINT_T0 );
  }
  if( N >= 8 )
  {
    _mm_prefetch( (const char *) &src[6 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[7 * srcStride], _MM_HINT_T0 );
  }

  const int16_t *srcOrig = src;
  int16_t *dstOrig = dst;

  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max() );

  __m128i vsum;

  for( int i = 0; i < N; i += 2 )
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16( _mm_set1_epi16( coeff[i] ), _mm_set1_epi16( coeff[i + 1] ) );
  }

  for( int col = 0; col < width; col += 4 )
  {
    for( int i = 0; i < N - 1; i++ )
    {
      vsrc[i] = _mm_loadl_epi64( ( __m128i const * )&src[col + i * srcStride] );
    }
    for( int row = 0; row < height; row++ )
    {
      _mm_prefetch( (const char *) &src[col + ( N + 0 ) * srcStride], _MM_HINT_T0 );
      _mm_prefetch( (const char *) &src[col + ( N + 1 ) * srcStride], _MM_HINT_T0 );

      vsrc[N - 1] = _mm_loadl_epi64( ( __m128i const * )&src[col + ( N - 1 ) * srcStride] );

      vsum = vzero;
      for( int i = 0; i < N; i += 2 )
      {
        __m128i vsrc0 = _mm_unpacklo_epi16( vsrc[i], vsrc[i + 1] );
        vsum = _mm_add_epi32( vsum, _mm_madd_epi16( vsrc0, vcoeff[i / 2] ) );
      }

      for( int i = 0; i < N - 1; i++ )
      {
        vsrc[i] = vsrc[i + 1];
      }

      vsum = _mm_add_epi32( vsum, voffset );
      vsum = _mm_srai_epi32( vsum, shift );
      vsum = _mm_packs_epi32( vsum, vzero );

      if( shiftBack ) //clip
      {
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }

      _mm_storel_epi64( ( __m128i* )&dst[col], vsum );

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM8( const int16_t *src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  _mm_prefetch( (const char *) &src[0 * srcStride], _MM_HINT_T0 );
  _mm_prefetch( (const char *) &src[1 * srcStride], _MM_HINT_T0 );
  if( N >= 2 )
  {
    _mm_prefetch( (const char *) &src[2 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[3 * srcStride], _MM_HINT_T0 );
  }
  if( N >= 6 )
  {
    _mm_prefetch( (const char *) &src[4 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[5 * srcStride], _MM_HINT_T0 );
  }
  if( N >= 8 )
  {
    _mm_prefetch( (const char *) &src[6 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[7 * srcStride], _MM_HINT_T0 );
  }

  const Pel *srcOrig = src;
  int16_t *dstOrig = dst;

  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max() );

  __m128i vsum, vsuma, vsumb;

  for( int i = 0; i < N; i += 2 )
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16( _mm_set1_epi16( coeff[i] ), _mm_set1_epi16( coeff[i + 1] ) );
  }

  for( int col = 0; col < width; col += 8 )
  {
    for( int i = 0; i < N - 1; i++ )
    {
      vsrc[i] = _mm_loadu_si128( ( __m128i const * )&src[col + i * srcStride] );
    }

    for( int row = 0; row < height; row++ )
    {
      _mm_prefetch( (const char *) &src[col + ( N + 0 ) * srcStride], _MM_HINT_T0 );
      _mm_prefetch( (const char *) &src[col + ( N + 1 ) * srcStride], _MM_HINT_T0 );

      vsrc[N - 1] = _mm_loadu_si128( ( __m128i const * )&src[col + ( N - 1 ) * srcStride] );
      vsuma = vsumb = vzero;
      for( int i = 0; i < N; i += 2 )
      {
        __m128i vsrca = _mm_unpacklo_epi16( vsrc[i], vsrc[i + 1] );
        __m128i vsrcb = _mm_unpackhi_epi16( vsrc[i], vsrc[i + 1] );
        vsuma = _mm_add_epi32( vsuma, _mm_madd_epi16( vsrca, vcoeff[i / 2] ) );
        vsumb = _mm_add_epi32( vsumb, _mm_madd_epi16( vsrcb, vcoeff[i / 2] ) );
      }
      for( int i = 0; i < N - 1; i++ )
      {
        vsrc[i] = vsrc[i + 1];
      }

      vsuma = _mm_add_epi32( vsuma, voffset );
      vsumb = _mm_add_epi32( vsumb, voffset );

      vsuma = _mm_srai_epi32( vsuma, shift );
      vsumb = _mm_srai_epi32( vsumb, shift );

      vsum = _mm_packs_epi32( vsuma, vsumb );

      if( shiftBack ) //clip
      {
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }

      _mm_storeu_si128( ( __m128i * )&dst[col], vsum );

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}

// template<typename Tdst, int N, bool shiftBack, bool biPred, bool bWeight, bool bChromaIntl>
// static void qpelV16AVX2M8( const short* src, int srcStride, Tdst *dst, int dstStride, int width, int height, int shift, int bitdepth, short const *coeff, wpPredParam *pwp1=NULL, wpPredParam *pwp2=NULL )
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM8_AVX2( const int16_t *src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  _mm_prefetch( (const char *) &src[0 * srcStride], _MM_HINT_T0 );
  _mm_prefetch( (const char *) &src[1 * srcStride], _MM_HINT_T0 );
  if( N >= 2 )
  {
    _mm_prefetch( (const char *) &src[2 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[3 * srcStride], _MM_HINT_T0 );
  }
  if( N >= 6 )
  {
    _mm_prefetch( (const char *) &src[4 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[5 * srcStride], _MM_HINT_T0 );
  }
  if( N >= 8 )
  {
    _mm_prefetch( (const char *) &src[6 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[7 * srcStride], _MM_HINT_T0 );
  }

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min() );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max() );

  __m256i vsum;
  __m128i vsrc[N];
  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  const short *srcOrig = src;
  int16_t *dstOrig = dst;

  for( int col = 0; col < width; col+=8 )
  {
    for( int i=0; i<N-1; i++ )
    {
      vsrc[i]= _mm_loadu_si128( ( const __m128i * )&src[col + i * srcStride] );
    }
    for( int row = 0; row < height; row++ )
    {
      _mm_prefetch( (const char *) &src[col + ( N + 0 ) * srcStride], _MM_HINT_T0 );
      _mm_prefetch( (const char *) &src[col + ( N + 1 ) * srcStride], _MM_HINT_T0 );

      vsrc[N-1]= _mm_loadu_si128( ( const __m128i * )&src[col + ( N-1 ) * srcStride] );

      vsum = _mm256_setzero_si256();
      for( int i=0; i<N; i+=2 )
      {
        __m128i vsrc0 = _mm_unpacklo_epi16( vsrc[i], vsrc[i+1] );
        __m128i vsrc1 = _mm_unpackhi_epi16( vsrc[i], vsrc[i+1] );
        __m256i vsrc  = _mm256_inserti128_si256( _mm256_castsi128_si256( vsrc0 ), vsrc1, 1 );
        vsum  = _mm256_add_epi32( vsum, _mm256_madd_epi16( vsrc, vcoeff[i/2] ) );

      }
      for( int i=0; i<N-1; i++ )
      {
        vsrc[i] = vsrc[i+1];
      }

      vsum = _mm256_add_epi32( vsum, voffset );
      vsum = _mm256_srai_epi32( vsum, shift );

      __m128i vsump = _mm256_cvtepi32_epi16x( vsum );
      if( shiftBack )
      { //clip
        vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
      }
      _mm_storeu_si128( ( __m128i * )&dst[col], vsump );

      src += srcStride;
      dst += dstStride;
    }
    src= srcOrig;
    dst= dstOrig;
  }
#endif
}

// template<typename Tdst, int N, bool shiftBack, bool biPred, bool bWeight, bool bChromaIntl>
// static void qpelV16AVX2M16( const short *src, int srcStride, Tdst *dst, int dstStride, int width, int height, int shift, int bitdepth, short const *coeff, wpPredParam *pwp1=NULL, wpPredParam *pwp2=NULL )
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM16_AVX2( const int16_t *src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  _mm_prefetch( (const char *) &src[0 * srcStride], _MM_HINT_T0 );
  _mm_prefetch( (const char *) &src[1 * srcStride], _MM_HINT_T0 );
  if( N >= 2 )
  {
    _mm_prefetch( (const char *) &src[2 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[3 * srcStride], _MM_HINT_T0 );
  }
  if( N >= 6 )
  {
    _mm_prefetch( (const char *) &src[4 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[5 * srcStride], _MM_HINT_T0 );
  }
  if( N >= 8 )
  {
    _mm_prefetch( (const char *) &src[6 * srcStride], _MM_HINT_T0 );
    _mm_prefetch( (const char *) &src[7 * srcStride], _MM_HINT_T0 );
  }

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m256i vibdimin   = _mm256_set1_epi16( clpRng.min() );
  __m256i vibdimax   = _mm256_set1_epi16( clpRng.max() );
  __m256i vzero      = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;

  __m256i vsrc[N];
  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  const short *srcOrig = src;
  int16_t *dstOrig = dst;

  for( int col = 0; col < width; col+=16 )
  {
    for( int i=0; i<N-1; i++ )
    {
      vsrc[i] = _mm256_loadu_si256( ( const __m256i * )&src[col + i * srcStride] );
    }
    for( int row = 0; row < height; row++ )
    {
      _mm_prefetch( (const char *) &src[col + ( N + 0 ) * srcStride], _MM_HINT_T0 );
      _mm_prefetch( (const char *) &src[col + ( N + 1 ) * srcStride], _MM_HINT_T0 );

      vsrc[N-1]= _mm256_loadu_si256( ( const __m256i * )&src[col + ( N-1 ) * srcStride] );
      vsuma = vsumb = vzero;
      for( int i=0; i<N; i+=2 )
      {
        __m256i vsrca = _mm256_unpacklo_epi16( vsrc[i], vsrc[i+1] );
        __m256i vsrcb = _mm256_unpackhi_epi16( vsrc[i], vsrc[i+1] );
        vsuma  = _mm256_add_epi32( vsuma, _mm256_madd_epi16( vsrca, vcoeff[i/2] ) );
        vsumb  = _mm256_add_epi32( vsumb, _mm256_madd_epi16( vsrcb, vcoeff[i/2] ) );
      }
      for( int i=0; i<N-1; i++ )
      {
        vsrc[i] = vsrc[i+1];
      }

      vsuma = _mm256_add_epi32  ( vsuma, voffset );
      vsumb = _mm256_add_epi32  ( vsumb, voffset );
      vsuma = _mm256_srai_epi32 ( vsuma, shift );
      vsumb = _mm256_srai_epi32 ( vsumb, shift );

      vsum  = _mm256_packs_epi32( vsuma, vsumb );

      if( shiftBack )
      { //clip
        vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
      }

      _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );

      src += srcStride;
      dst += dstStride;
    }
    src= srcOrig;
    dst= dstOrig;
  }
#endif
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

static inline __m128i simdInterpolateLuma2P8( int16_t const *src, ptrdiff_t srcStride, __m128i *mmCoeff, const __m128i & mmOffset, int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( int n = 0; n < 2; n++ )
  {
    __m128i mmPix = _mm_loadu_si128( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix, mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix, mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi, _mm_unpackhi_epi16( lo, hi ) );
    sumLo = _mm_add_epi32( sumLo, _mm_unpacklo_epi16( lo, hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi, mmOffset ), shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo, mmOffset ), shift );
  return( _mm_packs_epi32( sumLo, sumHi ) );
}

static inline __m128i simdInterpolateLuma2P4( int16_t const *src, ptrdiff_t srcStride, __m128i *mmCoeff, const __m128i & mmOffset, int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( int n = 0; n < 2; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix, mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix, mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi, _mm_unpackhi_epi16( lo, hi ) );
    sumLo = _mm_add_epi32( sumLo, _mm_unpacklo_epi16( lo, hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi, mmOffset ), shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo, mmOffset ), shift );
  return( _mm_packs_epi32( sumLo, sumHi ) );
}

static inline __m128i simdClip3( __m128i mmMin, __m128i mmMax, __m128i mmPix )
{
  __m128i mmMask = _mm_cmpgt_epi16( mmPix, mmMin );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask, mmPix ), _mm_andnot_si128( mmMask, mmMin ) );
  mmMask = _mm_cmplt_epi16( mmPix, mmMax );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask, mmPix ), _mm_andnot_si128( mmMask, mmMax ) );
  return( mmPix );
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_M8( const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, ptrdiff_t cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c )
{
  int row, col;
  __m128i mmOffset = _mm_set1_epi32( offset );
  __m128i mmCoeff[2];
  __m128i mmMin = _mm_set1_epi16( clpRng.min() );
  __m128i mmMax = _mm_set1_epi16( clpRng.max() );
  for( int n = 0; n < 2; n++ )
    mmCoeff[n] = _mm_set1_epi16( c[n] );
  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col += 8 )
    {
      __m128i mmFiltered = simdInterpolateLuma2P8( src + col, cStride, mmCoeff, mmOffset, shift );
      if( isLast )
      {
        mmFiltered = simdClip3( mmMin, mmMax, mmFiltered );
      }
      _mm_storeu_si128( ( __m128i * )( dst + col ), mmFiltered );
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_M4( const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, ptrdiff_t cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c )
{
  int row, col;
  __m128i mmOffset = _mm_set1_epi32( offset );
  __m128i mmCoeff[8];
  __m128i mmMin = _mm_set1_epi16( clpRng.min() );
  __m128i mmMax = _mm_set1_epi16( clpRng.max() );
  for( int n = 0; n < 2; n++ )
    mmCoeff[n] = _mm_set1_epi16( c[n] );
  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col += 4 )
    {
      __m128i mmFiltered = simdInterpolateLuma2P4( src + col, cStride, mmCoeff, mmOffset, shift );
      if( isLast )
      {
        mmFiltered = simdClip3( mmMin, mmMax, mmFiltered );
      }
      _mm_storel_epi64( ( __m128i * )( dst + col ), mmFiltered );
    }
    src += srcStride;
    dst += dstStride;
  }
}

#ifdef USE_AVX2
static inline __m256i simdInterpolateLumaHighBit2P16(int16_t const *src1, ptrdiff_t srcStride, __m256i *mmCoeff, const __m256i & mmOffset, __m128i &mmShift)
{
  __m256i mm_mul_lo = _mm256_setzero_si256();
  __m256i mm_mul_hi = _mm256_setzero_si256();

  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m256i mmPix = _mm256_lddqu_si256((__m256i*)(src1 + coefIdx * srcStride));
    __m256i mm_hi = _mm256_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m256i mm_lo = _mm256_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    mm_mul_lo = _mm256_add_epi32(mm_mul_lo, _mm256_unpacklo_epi16(mm_lo, mm_hi));
    mm_mul_hi = _mm256_add_epi32(mm_mul_hi, _mm256_unpackhi_epi16(mm_lo, mm_hi));
  }
  mm_mul_lo = _mm256_sra_epi32(_mm256_add_epi32(mm_mul_lo, mmOffset), mmShift);
  mm_mul_hi = _mm256_sra_epi32(_mm256_add_epi32(mm_mul_hi, mmOffset), mmShift);
  __m256i mm_sum = _mm256_packs_epi32(mm_mul_lo, mm_mul_hi);
  return (mm_sum);
}
#endif

static inline __m128i simdInterpolateLumaHighBit2P8(int16_t const *src1, ptrdiff_t srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i mm_mul_lo = _mm_setzero_si128();
  __m128i mm_mul_hi = _mm_setzero_si128();

  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m128i mmPix = _mm_loadu_si128((__m128i*)(src1 + coefIdx * srcStride));
    __m128i mm_hi = _mm_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_lo = _mm_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    mm_mul_lo = _mm_add_epi32(mm_mul_lo, _mm_unpacklo_epi16(mm_lo, mm_hi));
    mm_mul_hi = _mm_add_epi32(mm_mul_hi, _mm_unpackhi_epi16(mm_lo, mm_hi));
  }
  mm_mul_lo = _mm_sra_epi32(_mm_add_epi32(mm_mul_lo, mmOffset), mmShift);
  mm_mul_hi = _mm_sra_epi32(_mm_add_epi32(mm_mul_hi, mmOffset), mmShift);
  __m128i mm_sum = _mm_packs_epi32(mm_mul_lo, mm_mul_hi);
  return(mm_sum);
}

static inline __m128i simdInterpolateLumaHighBit2P4(int16_t const *src1, ptrdiff_t srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i mm_sum = _mm_setzero_si128();
  __m128i mm_zero = _mm_setzero_si128();
  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m128i mmPix = _mm_loadl_epi64((__m128i*)(src1 + coefIdx * srcStride));
    __m128i mm_hi = _mm_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_lo = _mm_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_mul = _mm_unpacklo_epi16(mm_lo, mm_hi);
    mm_sum = _mm_add_epi32(mm_sum, mm_mul);
  }
  mm_sum = _mm_sra_epi32(_mm_add_epi32(mm_sum, mmOffset), mmShift);
  mm_sum = _mm_packs_epi32(mm_sum, mm_zero);
  return(mm_sum);
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_HIGHBIT_M4(const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, ptrdiff_t cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c)
{
#if USE_AVX2
  __m256i mm256Offset = _mm256_set1_epi32(offset);
  __m256i mm256Coeff[2];
  for (int n = 0; n < 2; n++)
  {
    mm256Coeff[n] = _mm256_set1_epi16(c[n]);
  }
#endif
  __m128i mmOffset = _mm_set1_epi32(offset);
  __m128i mmCoeff[2];
  for (int n = 0; n < 2; n++)
    mmCoeff[n] = _mm_set1_epi16(c[n]);

  __m128i mmShift = _mm_cvtsi64_si128(shift);

  CHECK(isLast, "Not Supported");
  CHECK(width % 4 != 0, "Not Supported");

  for (int row = 0; row < height; row++)
  {
    int col = 0;
#if USE_AVX2
    for (; col < ((width >> 4) << 4); col += 16)
    {
      __m256i mmFiltered = simdInterpolateLumaHighBit2P16(src + col, cStride, mm256Coeff, mm256Offset, mmShift);
      _mm256_storeu_si256((__m256i *)(dst + col), mmFiltered);
    }
#endif
    for (; col < ((width >> 3) << 3); col += 8)
    {
      __m128i mmFiltered = simdInterpolateLumaHighBit2P8(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storeu_si128((__m128i *)(dst + col), mmFiltered);
    }

    for (; col < ((width >> 2) << 2); col += 4)
    {
      __m128i mmFiltered = simdInterpolateLumaHighBit2P4(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storel_epi64((__m128i *)(dst + col), mmFiltered);
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext>
static void simdInterpolateN2_2D( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *ch, TFilterCoeff const *cv )
{
  const int shift1st  = IF_FILTER_PREC_BILINEAR - ( IF_INTERNAL_PREC_BILINEAR - clpRng.bd );
  const int offset1st = 1 << ( shift1st - 1 );

  const int shift2nd  = 4;
  const int offset2nd = 1 << ( shift2nd - 1 );
  
  _mm_prefetch( ( const char * ) src, _MM_HINT_T0 );

#if USE_AVX2
  if( ( ( width - 4 ) & 15 ) == 0 )
  {
    __m256i mm256Offset1   = _mm256_set1_epi16( offset1st );
    __m256i mm256Offset2   = _mm256_set1_epi16( offset2nd );
    __m256i mm256CoeffH[2] = { _mm256_set1_epi16( ch[0] ), _mm256_set1_epi16( ch[1] ) };
    __m256i mm256CoeffV[2] = { _mm256_set1_epi16( cv[0] ), _mm256_set1_epi16( cv[1] ) };
    __m256i mm256LastH [8];
    __m128i mmLast4H;

    for( int row = -1; row < height; row++ )
    {
      _mm_prefetch( ( const char * ) &src[srcStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char * ) &src[4],         _MM_HINT_T0 );

      {
        __m128i mmFiltered = _mm256_castsi256_si128( mm256Offset1 );
        __m128i mmPix  = _mm_loadl_epi64( ( __m128i* )( src ) );
        __m128i mmPix1 = _mm_loadl_epi64( ( __m128i* )( src + 1 ) );
        mmFiltered = _mm_add_epi16 ( mmFiltered, _mm_mullo_epi16( mmPix,  _mm256_castsi256_si128( mm256CoeffH[0] ) ) );
        mmFiltered = _mm_add_epi16 ( mmFiltered, _mm_mullo_epi16( mmPix1, _mm256_castsi256_si128( mm256CoeffH[1] ) ) );
        mmFiltered = _mm_srai_epi16( mmFiltered, shift1st );

        if( row >= 0 )
        {
          __m128i
          mmFiltered2 = _mm256_castsi256_si128( mm256Offset2 );
          mmFiltered2 = _mm_add_epi16 ( mmFiltered2, _mm_mullo_epi16( mmLast4H,    _mm256_castsi256_si128( mm256CoeffV[0] ) ) );
          mmFiltered2 = _mm_add_epi16 ( mmFiltered2, _mm_mullo_epi16( mmFiltered,  _mm256_castsi256_si128( mm256CoeffV[1] ) ) );
          mmFiltered2 = _mm_srai_epi16( mmFiltered2, shift2nd );

          _mm_storel_epi64( ( __m128i* ) dst, mmFiltered2 );
        }

        mmLast4H = mmFiltered;
      }

      for( int x = 4; x < width; x += 16 )
      {
        _mm_prefetch( ( const char * ) &src[x + 16], _MM_HINT_T0 );

        __m256i mmFiltered = mm256Offset1;
        __m256i mmPix   = _mm256_loadu_si256( ( __m256i* )( src + x ) );
        __m256i mmPix1  = _mm256_loadu_si256( ( __m256i* )( src + x + 1 ) );
        mmFiltered      = _mm256_add_epi16  ( mmFiltered, _mm256_mullo_epi16( mmPix,  mm256CoeffH[0] ) );
        mmFiltered      = _mm256_add_epi16  ( mmFiltered, _mm256_mullo_epi16( mmPix1, mm256CoeffH[1] ) );
        mmFiltered      = _mm256_srai_epi16 ( mmFiltered, shift1st );

        int idx = x >> 4;

        if( row >= 0 )
        {
          __m256i
          mmFiltered2 = mm256Offset2;
          mmFiltered2 = _mm256_add_epi16  ( mmFiltered2, _mm256_mullo_epi16( mm256LastH[idx], mm256CoeffV[0] ) );
          mmFiltered2 = _mm256_add_epi16  ( mmFiltered2, _mm256_mullo_epi16( mmFiltered,      mm256CoeffV[1] ) );
          mmFiltered2 = _mm256_srai_epi16 ( mmFiltered2, shift2nd );

          _mm256_storeu_si256( ( __m256i* ) ( dst + x ), mmFiltered2 );
        }

        mm256LastH[idx] = mmFiltered;
      }

      if( row >= 0 ) dst += dstStride;

      src += srcStride;
    }
  }
  else
#endif
  {
    __m128i mmOffset1   = _mm_set1_epi16( offset1st );
    __m128i mmOffset2   = _mm_set1_epi16( offset2nd );
    __m128i mmCoeffH[2] = { _mm_set1_epi16( ch[0] ), _mm_set1_epi16( ch[1] ) };
    __m128i mmCoeffV[2] = { _mm_set1_epi16( cv[0] ), _mm_set1_epi16( cv[1] ) };
#if USE_AVX2
    __m128i mmLastH [1];
#else
    __m128i mmLastH [16];
#endif
    __m128i mmLast4H;

    for( int row = -1; row < height; row++ )
    {
      _mm_prefetch( ( const char * ) &src[srcStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char * ) &src[4],         _MM_HINT_T0 );

      {
        __m128i mmFiltered = mmOffset1;
        __m128i mmPix  = _mm_loadl_epi64( ( __m128i* )( src ) );
        __m128i mmPix1 = _mm_loadl_epi64( ( __m128i* )( src + 1 ) );
        mmFiltered = _mm_add_epi16 ( mmFiltered, _mm_mullo_epi16( mmPix,  mmCoeffH[0] ) );
        mmFiltered = _mm_add_epi16 ( mmFiltered, _mm_mullo_epi16( mmPix1, mmCoeffH[1] ) );
        mmFiltered = _mm_srai_epi16( mmFiltered, shift1st );

        if( row >= 0 )
        {
          __m128i
          mmFiltered2 = mmOffset2;
          mmFiltered2 = _mm_add_epi16 ( mmFiltered2, _mm_mullo_epi16( mmLast4H,   mmCoeffV[0] ) );
          mmFiltered2 = _mm_add_epi16 ( mmFiltered2, _mm_mullo_epi16( mmFiltered, mmCoeffV[1] ) );
          mmFiltered2 = _mm_srai_epi16( mmFiltered2, shift2nd );

          _mm_storel_epi64( ( __m128i* ) dst, mmFiltered2 );
        }

        mmLast4H = mmFiltered;
      }

      for( int x = 4; x < width; x += 8 )
      {
#if !USE_AVX2
        _mm_prefetch( ( const char * ) &src[x + 8], _MM_HINT_T0 );

#endif
        __m128i mmFiltered = mmOffset1;
        __m128i mmPix   = _mm_loadu_si128( ( __m128i* )( src + x ) );
        __m128i mmPix1  = _mm_loadu_si128( ( __m128i* )( src + x + 1 ) );
        mmFiltered      = _mm_add_epi16  ( mmFiltered, _mm_mullo_epi16( mmPix,  mmCoeffH[0] ) );
        mmFiltered      = _mm_add_epi16  ( mmFiltered, _mm_mullo_epi16( mmPix1, mmCoeffH[1] ) );
        mmFiltered      = _mm_srai_epi16 ( mmFiltered, shift1st );

        int idx = x >> 3;

        if( row >= 0 )
        {
          __m128i
          mmFiltered2 = mmOffset2;
          mmFiltered2 = _mm_add_epi16  ( mmFiltered2, _mm_mullo_epi16( mmLastH[idx], mmCoeffV[0] ) );
          mmFiltered2 = _mm_add_epi16  ( mmFiltered2, _mm_mullo_epi16( mmFiltered,   mmCoeffV[1] ) );
          mmFiltered2 = _mm_srai_epi16 ( mmFiltered2, shift2nd );

          _mm_storeu_si128( ( __m128i* ) ( dst + x ), mmFiltered2 );
        }

        mmLastH[idx] = mmFiltered;
      }

      if( row >= 0 ) dst += dstStride;

      src += srcStride;
    }
  }
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_10BIT_M4(const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, ptrdiff_t cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c)
{
  CHECK( isLast, "Not Supported" );

  _mm_prefetch( ( const char * ) &src[0 * srcStride + 0 * cStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char * ) &src[0 * srcStride + 1 * cStride], _MM_HINT_T0 );

  int row;
  width -= 4;

#if USE_AVX2
  if( ( width & 15 ) == 0 && vext >= AVX2 )
  {
    __m256i mm256Offset = _mm256_set1_epi16( offset );
    __m256i mm256Coeff[2] = { _mm256_set1_epi16( c[0] ), _mm256_set1_epi16( c[1] ) };

    for( row = 0; row < height; row++ )
    {
      _mm_prefetch( ( const char * ) &src[1 * srcStride + 0 * cStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char * ) &src[1 * srcStride + 1 * cStride], _MM_HINT_T0 );

      // multiple of 16
      for( int x = 0; x < width; x += 16 )
      {
        __m256i mmFiltered = mm256Offset;
        __m256i mmPix   = _mm256_loadu_si256( ( __m256i* )( src + x ) );
        __m256i mmPix1  = _mm256_loadu_si256( ( __m256i* )( src + x + cStride ) );
        mmFiltered      = _mm256_add_epi16  ( mmFiltered, _mm256_mullo_epi16( mmPix,  mm256Coeff[0] ) );
        mmFiltered      = _mm256_add_epi16  ( mmFiltered, _mm256_mullo_epi16( mmPix1, mm256Coeff[1] ) );
        mmFiltered      = _mm256_srai_epi16 ( mmFiltered, shift );
        _mm256_storeu_si256( ( __m256i * )  ( dst + x ), mmFiltered );
      }

      // last 4 samples
      {
        __m128i mmFiltered = _mm256_castsi256_si128( mm256Offset );
        __m128i mmPix   = _mm_loadl_epi64( ( __m128i* )( src + width ) );
        __m128i mmPix1  = _mm_loadl_epi64( ( __m128i* )( src + width + cStride ) );
        mmFiltered      = _mm_add_epi16  ( mmFiltered, _mm_mullo_epi16( mmPix,  _mm256_castsi256_si128( mm256Coeff[0] ) ) );
        mmFiltered      = _mm_add_epi16  ( mmFiltered, _mm_mullo_epi16( mmPix1, _mm256_castsi256_si128( mm256Coeff[1] ) ) );
        mmFiltered      = _mm_srai_epi16 ( mmFiltered, shift );
        _mm_storel_epi64( ( __m128i * )  ( dst + width ), mmFiltered );
      }
      src += srcStride;
      dst += dstStride;
    }
  }
  else
#endif
  {
    CHECKD( ( width & 7 ), "Unsupported size " << ( width + 4 ) );

    __m128i mmOffset   =   _mm_set1_epi16( offset );
    __m128i mmCoeff[2] = { _mm_set1_epi16( c[0] ), _mm_set1_epi16( c[1] ) };
    __m128i mmFiltered, mmPix, mmPix1, lo0, lo1;

    for( row = 0; row < height; row++ )
    {
      _mm_prefetch( ( const char * ) &src[1 * srcStride + 0 * cStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char * ) &src[1 * srcStride + 1 * cStride], _MM_HINT_T0 );

      for( int x = 0; x < width; x += 8 )
      {
        mmPix           = _mm_loadu_si128( ( __m128i* )( src + x ) );
        mmPix1          = _mm_loadu_si128( ( __m128i* )( src + x + cStride ) );
        lo0             = _mm_mullo_epi16( mmPix,  mmCoeff[0] );
        lo1             = _mm_mullo_epi16( mmPix1, mmCoeff[1] );
        mmFiltered      = _mm_add_epi16  ( lo0, lo1 );
        mmFiltered      = _mm_srai_epi16 ( _mm_add_epi16( mmFiltered, mmOffset ), shift );
        _mm_storeu_si128( ( __m128i * )( dst + x ), mmFiltered );
      }
        
      mmPix           = _mm_loadl_epi64( ( __m128i* )( src + width ) );
      mmPix1          = _mm_loadl_epi64( ( __m128i* )( src + width + cStride ) );
      lo0             = _mm_mullo_epi16( mmPix,  mmCoeff[0] );
      lo1             = _mm_mullo_epi16( mmPix1, mmCoeff[1] );
      mmFiltered      = _mm_add_epi16  ( lo0, lo1 );
      mmFiltered      = _mm_srai_epi16 ( _mm_add_epi16( mmFiltered, mmOffset ), shift );
      _mm_storel_epi64( ( __m128i * )( dst + width ), mmFiltered );

      src += srcStride;
      dst += dstStride;
    }
  }
}

template<X86_VEXT vext, int N, bool isVertical, bool isFirst, bool isLast>
static void simdFilter( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR )
{
  int row, col;

  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }

  ptrdiff_t cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  int offset;
  int headRoom = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift    = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  CHECK( shift < 0, "Negative shift" );

#define USE_M16_AVX2_IF 1

  if( biMCForDMVR )
  {
    if( isFirst )
    {
      shift = IF_FILTER_PREC_BILINEAR - (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
      offset = 1 << (shift - 1);
    }
    else
    {
      shift = 4;
      offset = 1 << (shift - 1);
    }
  }
  else
  {
    if( isLast )
    {
      shift += ( isFirst ) ? 0 : headRoom;
      offset = 1 << ( shift - 1 );
      offset += ( isFirst ) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
    }
    else
    {
      shift -= ( isFirst ) ? headRoom : 0;
      offset = ( isFirst ) ? -IF_INTERNAL_OFFS << shift : 0;
    }
  }
  {
    if( N == 8 && !( width & 0x07 ) )
    {
      if( !isVertical )
      {
        if( vext >= AVX2 )
#if USE_M16_AVX2_IF
          if( !( width & 15 ) )
            simdInterpolateHorM16_AVX2<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          else
#endif
            simdInterpolateHorM8_AVX2 <vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        else
          simdInterpolateHorM8<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
      {
        if( vext>= AVX2 )
#if USE_M16_AVX2_IF
          if( !( width & 15 ) )
            simdInterpolateVerM16_AVX2<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          else
#endif
            simdInterpolateVerM8_AVX2 <vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        else
          simdInterpolateVerM8<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      return;
    }
    else if( N == 8 && !( width & 0x03 ) )
    {
      if( !isVertical )
      {
        simdInterpolateHorM4<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
        simdInterpolateVerM4<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      return;
    }
    else if( N == 4 && !( width & 0x03 ) )
    {
      if( !isVertical )
      {
        if( ( width % 8 ) == 0 )
        {
          if( vext>= AVX2 )
#if USE_M16_AVX2_IF
            if( !( width & 15 ) )
              simdInterpolateHorM16_AVX2<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
            else
#endif
              simdInterpolateHorM8_AVX2 <vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          else
            simdInterpolateHorM8<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else
          simdInterpolateHorM4<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
      {
        if( ( width % 8 ) == 0 )
        {
          if( vext >= AVX2 )
#if USE_M16_AVX2_IF
            if( !( width & 15 ) )
              simdInterpolateVerM16_AVX2<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
            else
#endif
              simdInterpolateVerM8_AVX2 <vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          else
            simdInterpolateVerM8<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else
          simdInterpolateVerM4<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      return;
    }
    else if( biMCForDMVR )
    {
      if( N == 2 && !( width & 0x03 ) )
      {
        if (clpRng.bd <= 10)
        {
          simdInterpolateN2_10BIT_M4<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c );
        }
        else
        {
          simdInterpolateN2_HIGHBIT_M4<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c);
        }
        return;
      }
    }
    else if( N == 2 && !( width & 0x07 ) )
    {
      simdInterpolateN2_M8<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c );
      return;
    }
    else if( N == 2 && !( width & 0x03 ) )
    {
      simdInterpolateN2_M4<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c );
      return;
    }
  }

  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col++ )
    {
      int sum;

      sum  = src[col + 0 * cStride] * c[0];
      sum += src[col + 1 * cStride] * c[1];
      if( N >= 4 )
      {
        sum += src[col + 2 * cStride] * c[2];
        sum += src[col + 3 * cStride] * c[3];
      }
      if( N >= 6 )
      {
        sum += src[col + 4 * cStride] * c[4];
        sum += src[col + 5 * cStride] * c[5];
      }
      if( N == 8 )
      {
        sum += src[col + 6 * cStride] * c[6];
        sum += src[col + 7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[col] = val;
    }

    INCY( src, srcStride );
    INCY( dst, dstStride );
  }
}

#define _mm256_seti_m128i(/* __m128i */ hi, /* __m128i */ lo) \
   _mm256_inserti128_si256(_mm256_castsi128_si256(lo), (hi), 0x1)

template<X86_VEXT vext, bool isLast>
void simdFilter4x4_N6( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *_coeffV )
{
  int row;

  OFFSET( src, srcStride, -2, -2 );

  _mm_prefetch( ( const char* ) ( src                 ), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) ( src + 1 * srcStride ), _MM_HINT_T0 );

  int offset1st, offset2nd;
  int headRoom   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st   = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if( isLast )
  {
    shift1st  -= headRoom;
    shift2nd  += headRoom;
    offset1st  = -IF_INTERNAL_OFFS << shift1st;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS << shift1st;
    offset2nd = 0;
  }

  __m128i _dst0x;
  __m128i _dst1x;
  __m128i _dst2x;
  __m128i _dst3x;
  __m128i zerox = _mm_setzero_si128();

#if USE_AVX2
  if( vext >= AVX2 )
  {
    __m256i off   = _mm256_set1_epi32     ( offset1st );
    __m128i _src1x, _src2x, cVp1, cVp2;
    __m256i _dst0, _dst2, _src1, _src2, _src3;
    __m256i cV, cH;
    _src1x        = _mm_loadu_si128       ( ( const __m128i* ) ( coeffH ) );
    _src1x        = _mm_bsrli_si128       ( _src1x, 2 );
    cH            = _mm256_set_m128i      ( _src1x, _src1x );
    cVp1          = _mm_setzero_si128     ();
    cVp2          = _mm_loadu_si128       ( ( const __m128i* ) ( _coeffV ) );
    cVp2          = _mm_shuffle_epi8      ( cVp2, _mm_setr_epi8( -1, -1, -1, -1, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3 ) );

    _dst0         = _mm256_set1_epi32     ( offset2nd );
    _dst2         = _mm256_set1_epi32     ( offset2nd );

    for( row = 1; row < 9; row += 2 )
    {
      _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + 3 * srcStride ), _MM_HINT_T0 );

      _src1x = _mm_alignr_epi8      ( cVp1, cVp2, 12 );
      _src2x = _mm_alignr_epi8      ( cVp2, cVp1, 12 );
      cVp1   = _src1x;
      cVp2   = _src2x;
      _src2x = _mm_bsrli_si128      ( _src1x, 2 );
      _src2x = _mm_unpacklo_epi16   ( _src2x, _src1x );
      cV     = _mm256_seti_m128i    ( _src2x, _src2x );

      // hor filter of row 0
      _src1x = _mm_loadu_si128      ( ( const __m128i* ) &src[0] );
      _src2x = _mm_loadu_si128      ( ( const __m128i* ) &src[1] );
      _src1  = _mm256_seti_m128i    ( _src2x, _src1x );
      _src1  = _mm256_madd_epi16    ( _src1, cH );

      _src1x = _mm_loadu_si128      ( ( const __m128i* ) &src[2] );
      _src2x = _mm_loadu_si128      ( ( const __m128i* ) &src[3] );
      _src2  = _mm256_seti_m128i    ( _src2x, _src1x );
      _src2  = _mm256_madd_epi16    ( _src2, cH );

      _src3  = _mm256_hadd_epi32    ( _src1, _src2 );

      INCY( src, srcStride );

      // hor filter of row 1
      _src1x = _mm_loadu_si128      ( ( const __m128i* ) &src[0] );
      _src2x = _mm_loadu_si128      ( ( const __m128i* ) &src[1] );
      _src1  = _mm256_seti_m128i    ( _src2x, _src1x );
      _src1x = _mm_loadu_si128      ( ( const __m128i* ) &src[2] );
      _src2x = _mm_loadu_si128      ( ( const __m128i* ) &src[3] );
      _src2  = _mm256_seti_m128i    ( _src2x, _src1x );

      _src1  = _mm256_madd_epi16    ( _src1, cH );
      _src2  = _mm256_madd_epi16    ( _src2, cH );
      _src1  = _mm256_hadd_epi32    ( _src1, _src2 );
      
      _src1  = _mm256_hadd_epi32    ( _src1, _src3 );
      _src1  = _mm256_add_epi32     ( _src1, off );
      _src3  = _mm256_srai_epi32    ( _src1, shift1st );

      INCY( src, srcStride );

      // vertical filter
      _src2  = _mm256_unpacklo_epi16( _mm256_srli_si256( _src3, 8 ), _src3 );
      _src1  = _mm256_shuffle_epi32 ( _src2, ( 0 << 0 ) + ( 0 << 2 ) + ( 0 << 4 ) + ( 0 << 6 ) );
      _src3  = _mm256_shuffle_epi32 ( _src2, ( 2 << 0 ) + ( 2 << 2 ) + ( 2 << 4 ) + ( 2 << 6 ) );

      _src1  = _mm256_madd_epi16    ( _src1, cV );
      _src3  = _mm256_madd_epi16    ( _src3, cV );

      _dst0  = _mm256_add_epi32     ( _src1, _dst0 );
      _dst2  = _mm256_add_epi32     ( _src3, _dst2 );
    }

    // process last row (9)
    _src1x = _mm_alignr_epi8      ( cVp1, cVp2, 14 );
    _src1x = _mm_cvtepu16_epi32   ( _src1x );
    cV     = _mm256_seti_m128i    ( _src1x, _src1x );

    _src1x = _mm_loadu_si128      ( ( const __m128i* ) &src[0] );
    _src2x = _mm_loadu_si128      ( ( const __m128i* ) &src[1] );
    _src1  = _mm256_seti_m128i    ( _src2x, _src1x );
    _src1x = _mm_loadu_si128      ( ( const __m128i* ) &src[2] );
    _src2x = _mm_loadu_si128      ( ( const __m128i* ) &src[3] );
    _src2  = _mm256_seti_m128i    ( _src2x, _src1x );
    _src1  = _mm256_madd_epi16    ( _src1, cH );
    _src2  = _mm256_madd_epi16    ( _src2, cH );
    _src1  = _mm256_hadd_epi32    ( _src1, _src2 );
    _src1  = _mm256_hadd_epi32    ( _src1, _mm256_setzero_si256() );
    _src1  = _mm256_add_epi32     ( _src1, off );
    _src3  = _mm256_srai_epi32    ( _src1, shift1st );
    _src2  = _mm256_shuffle_epi32 ( _src3, ( 1 << 0 ) + ( 1 << 2 ) + ( 1 << 4 ) + ( 1 << 6 ) );
    _src1  = _mm256_shuffle_epi32 ( _src3, ( 0 << 0 ) + ( 0 << 2 ) + ( 0 << 4 ) + ( 0 << 6 ) );

    _src2  = _mm256_madd_epi16    ( _src2, cV );
    _src1  = _mm256_madd_epi16    ( _src1, cV );

    _dst0  = _mm256_add_epi32     ( _src1, _dst0 );
    _dst2  = _mm256_add_epi32     ( _src2, _dst2 );

    _dst0 = _mm256_srai_epi32     ( _dst0, shift2nd );
    _dst2 = _mm256_srai_epi32     ( _dst2, shift2nd );

    if( isLast )
    {
      __m256i vmin = _mm256_set1_epi32( clpRng.min() );
      __m256i vmax = _mm256_set1_epi32( clpRng.max() );

      _dst0 = _mm256_max_epi32( _mm256_min_epi32( _dst0, vmax ), vmin );
      _dst2 = _mm256_max_epi32( _mm256_min_epi32( _dst2, vmax ), vmin );
    }

    _dst0x = _mm256_castsi256_si128  ( _dst0 );
    _dst1x = _mm256_extracti128_si256( _dst0, 1 );
    _dst2x = _mm256_castsi256_si128  ( _dst2 );
    _dst3x = _mm256_extracti128_si256( _dst2, 1 );
  }
  else
#endif
  {
    ALIGN_DATA( 64, TFilterCoeff coeffV[17] ) = { 0, 0, 0, _coeffV[6], _coeffV[5], _coeffV[4], _coeffV[3], _coeffV[2], _coeffV[1], 0, 0, 0, 0, 0, 0 };

    __m128i cH    = _mm_loadu_si128  ( ( const __m128i* ) ( coeffH ) );
            cH    = _mm_bsrli_si128  ( cH, 2 );
    __m128i off   = _mm_set1_epi32   ( offset1st );
    __m128i _src1, _src2, _srcx, cV;

    _dst0x = _mm_set1_epi32( offset2nd );
    _dst1x = _mm_set1_epi32( offset2nd );
    _dst2x = _mm_set1_epi32( offset2nd );
    _dst3x = _mm_set1_epi32( offset2nd );

    for( row = 0; row < 9; row++ )
    {
      _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + 3 * srcStride ), _MM_HINT_T0 );

      cV = _mm_loadl_epi64    ( ( const __m128i* ) ( coeffV + 9 - row - 1 ) );
      cV = _mm_cvtepu16_epi32 ( cV );

      _src1 = _mm_loadu_si128      ( ( const __m128i* ) &src[0] );
      _src1 = _mm_madd_epi16       ( _src1, cH );

      _src2 = _mm_loadu_si128      ( ( const __m128i* ) &src[1] );
      _src2 = _mm_madd_epi16       ( _src2, cH );
      _srcx = _mm_hadd_epi32       ( _src1, _src2 );

      _src1 = _mm_loadu_si128      ( ( const __m128i* ) &src[2] );
      _src1 = _mm_madd_epi16       ( _src1, cH );

      _src2 = _mm_loadu_si128      ( ( const __m128i* ) &src[3] );
      _src2 = _mm_madd_epi16       ( _src2, cH );
      _src2 = _mm_hadd_epi32       ( _src1, _src2 );

      _src2 = _mm_hadd_epi32       ( _srcx, _src2 );
      _src2 = _mm_add_epi32        ( _src2, off );
      _src2 = _mm_srai_epi32       ( _src2, shift1st );

      _src1 = _mm_shuffle_epi32    ( _src2, 0 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst0x = _mm_add_epi32       ( _src1, _dst0x );

      _src1 = _mm_shuffle_epi32    ( _src2, 1 + 4 + 16 + 64 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst1x = _mm_add_epi32       ( _src1, _dst1x );

      _src1 = _mm_shuffle_epi32    ( _src2, 2 + 8 + 32 + 128 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst2x = _mm_add_epi32       ( _src1, _dst2x );

      _src1 = _mm_shuffle_epi32    ( _src2, 3 + 12 + 48 + 192 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst3x = _mm_add_epi32       ( _src1, _dst3x );

      INCY( src, srcStride );
    }

    _dst0x = _mm_srai_epi32( _dst0x, shift2nd );
    _dst1x = _mm_srai_epi32( _dst1x, shift2nd );
    _dst2x = _mm_srai_epi32( _dst2x, shift2nd );
    _dst3x = _mm_srai_epi32( _dst3x, shift2nd );

    if( isLast )
    {
      __m128i vmin = _mm_set1_epi32( clpRng.min() );
      __m128i vmax = _mm_set1_epi32( clpRng.max() );

      _dst0x = _mm_max_epi32( _mm_min_epi32( _dst0x, vmax ), vmin );
      _dst1x = _mm_max_epi32( _mm_min_epi32( _dst1x, vmax ), vmin );
      _dst2x = _mm_max_epi32( _mm_min_epi32( _dst2x, vmax ), vmin );
      _dst3x = _mm_max_epi32( _mm_min_epi32( _dst3x, vmax ), vmin );
    }
  }

  __m128i a01b01 = _mm_unpacklo_epi32( _dst0x, _dst1x );
  __m128i a23b23 = _mm_unpackhi_epi32( _dst0x, _dst1x );
  __m128i c01d01 = _mm_unpacklo_epi32( _dst2x, _dst3x );
  __m128i c23d23 = _mm_unpackhi_epi32( _dst2x, _dst3x );

  _dst0x = _mm_unpacklo_epi64( a01b01, c01d01 );
  _dst1x = _mm_unpackhi_epi64( a01b01, c01d01 );
  _dst2x = _mm_unpacklo_epi64( a23b23, c23d23 );
  _dst3x = _mm_unpackhi_epi64( a23b23, c23d23 );

  _dst0x = _mm_packs_epi32( _dst0x, zerox );
  _dst1x = _mm_packs_epi32( _dst1x, zerox );
  _dst2x = _mm_packs_epi32( _dst2x, zerox );
  _dst3x = _mm_packs_epi32( _dst3x, zerox );

  _mm_storel_epi64( ( __m128i* ) ( dst                  ), _dst0x );
  _mm_storel_epi64( ( __m128i* ) ( dst +     dstStride ), _dst1x );
  _mm_storel_epi64( ( __m128i* ) ( dst + 2 * dstStride ), _dst2x );
  _mm_storel_epi64( ( __m128i* ) ( dst + 3 * dstStride ), _dst3x );
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext, bool isLast>
void simdFilter4x4_N4( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *_coeffV )
{
  int row;

  OFFSET( src, srcStride, -1, -1 );
  
  _mm_prefetch( ( const char* ) ( src                 ), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) ( src + 1 * srcStride ), _MM_HINT_T0 );

  int offset1st, offset2nd;
  int headRoom   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st   = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if( isLast )
  {
    shift1st  -= headRoom;
    shift2nd  += headRoom;
    offset1st  = -IF_INTERNAL_OFFS << shift1st;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS << shift1st;
    offset2nd = 0;
  }

  __m128i _dst0x;
  __m128i _dst1x;
  __m128i _dst2x;
  __m128i _dst3x;
  __m128i zerox = _mm_setzero_si128();

#if USE_AVX2
  if( vext >= AVX2 )
  {
    ALIGN_DATA( 64, const TFilterCoeff coeffV[4] ) = { _coeffV[3], _coeffV[2], _coeffV[1], _coeffV[0] };

    __m256i off   = _mm256_set1_epi32     ( offset1st );
    __m256i zero  = _mm256_setzero_si256  ();
    __m128i _src1x, _src2x, cVp1, cVp2;
    __m256i _dst0, _dst2, _src1, _src2, _src3;
    __m256i cV, cH;
    cH      = _mm256_set1_epi64x    ( *( ( const long long int* ) coeffH ) );
    cVp1    = _mm_setzero_si128     ();
    cVp2    = _mm_loadl_epi64       ( ( const __m128i* ) coeffV );

    _dst0   = _mm256_set1_epi32     ( offset2nd );
    _dst2   = _mm256_set1_epi32     ( offset2nd );

    _src1x  = _mm_alignr_epi8       ( cVp1, cVp2, 8 );
    _src2x  = _mm_alignr_epi8       ( cVp2, cVp1, 8 );
    cVp1    = _src1x;
    cVp2    = _src2x;

    for( row = 0; row < 6; row += 2 )
    {
      _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + 3 * srcStride ), _MM_HINT_T0 );

      _src1x = _mm_alignr_epi8      ( cVp1, cVp2, 12 );
      _src2x = _mm_alignr_epi8      ( cVp2, cVp1, 12 );
      cVp1   = _src1x;
      cVp2   = _src2x;
      _src2x = _mm_bsrli_si128      ( _src1x, 2 );
      _src2x = _mm_unpacklo_epi16   ( _src2x, _src1x );
      cV     = _mm256_seti_m128i    ( _src2x, _src2x );

      // hor filter of row 0
      _src1x = _mm_loadl_epi64      ( ( const __m128i* ) &src[0] );
      _src2x = _mm_loadl_epi64      ( ( const __m128i* ) &src[1] );
      _src1  = _mm256_seti_m128i    ( _src2x, _src1x );

      _src1x = _mm_loadl_epi64      ( ( const __m128i* ) &src[2] );
      _src2x = _mm_loadl_epi64      ( ( const __m128i* ) &src[3] );
      _src2  = _mm256_seti_m128i    ( _src2x, _src1x );

      _src1  = _mm256_unpacklo_epi64( _src1, _src2 );

      _src2  = _mm256_madd_epi16    ( _src1, cH );

      _src1  = _mm256_hadd_epi32    ( _src2, zero );
      _src1  = _mm256_add_epi32     ( _src1, off );
      _src3  = _mm256_srai_epi32    ( _src1, shift1st );

      INCY( src, srcStride );

      // hor filter of row 1
      _src1x = _mm_loadl_epi64      ( ( const __m128i* ) &src[0] );
      _src2x = _mm_loadl_epi64      ( ( const __m128i* ) &src[1] );
      _src1  = _mm256_seti_m128i    ( _src2x, _src1x );

      _src1x = _mm_loadl_epi64      ( ( const __m128i* ) &src[2] );
      _src2x = _mm_loadl_epi64      ( ( const __m128i* ) &src[3] );
      _src2  = _mm256_seti_m128i    ( _src2x, _src1x );

      _src1  = _mm256_unpacklo_epi64( _src1, _src2 );

      _src2  = _mm256_madd_epi16    ( _src1, cH );

      _src1  = _mm256_hadd_epi32    ( _src2, zero );
      _src1  = _mm256_add_epi32     ( _src1, off );
      _src1  = _mm256_srai_epi32    ( _src1, shift1st );

      INCY( src, srcStride );

      // vertical filter
      _src2  = _mm256_unpacklo_epi16( _src3, _src1 );
      _src1  = _mm256_shuffle_epi32 ( _src2, ( 0 << 0 ) + ( 0 << 2 ) + ( 0 << 4 ) + ( 0 << 6 ) );
      _src3  = _mm256_shuffle_epi32 ( _src2, ( 2 << 0 ) + ( 2 << 2 ) + ( 2 << 4 ) + ( 2 << 6 ) );

      _src1  = _mm256_madd_epi16    ( _src1, cV );
      _src3  = _mm256_madd_epi16    ( _src3, cV );

      _dst0  = _mm256_add_epi32     ( _src1, _dst0 );
      _dst2  = _mm256_add_epi32     ( _src3, _dst2 );
      // REF END
    }

    _src1x = _mm_alignr_epi8      ( cVp1, cVp2, 14 );
    _src1x = _mm_cvtepu16_epi32   ( _src1x );
    cV     = _mm256_seti_m128i    ( _src1x, _src1x );

    _src1x = _mm_loadl_epi64      ( ( const __m128i* ) &src[0] );
    _src2x = _mm_loadl_epi64      ( ( const __m128i* ) &src[1] );
    _src1  = _mm256_seti_m128i    ( _src2x, _src1x );

    _src1x = _mm_loadl_epi64      ( ( const __m128i* ) &src[2] );
    _src2x = _mm_loadl_epi64      ( ( const __m128i* ) &src[3] );
    _src2  = _mm256_seti_m128i    ( _src2x, _src1x );

    _src1  = _mm256_unpacklo_epi64( _src1, _src2 );

    _src2  = _mm256_madd_epi16    ( _src1, cH );

    _src1  = _mm256_hadd_epi32    ( _src2, zero );
    _src1  = _mm256_add_epi32     ( _src1, off );
    _src1  = _mm256_srai_epi32    ( _src1, shift1st );

    _src2  = _mm256_unpacklo_epi16( _src1, zero );
    _src1  = _mm256_shuffle_epi32 ( _src2, ( 0 << 0 ) + ( 0 << 2 ) + ( 0 << 4 ) + ( 0 << 6 ) );
    _src3  = _mm256_shuffle_epi32 ( _src2, ( 2 << 0 ) + ( 2 << 2 ) + ( 2 << 4 ) + ( 2 << 6 ) );

    _src1  = _mm256_madd_epi16    ( _src1, cV );
    _src3  = _mm256_madd_epi16    ( _src3, cV );

    _dst0  = _mm256_add_epi32     ( _src1, _dst0 );
    _dst2  = _mm256_add_epi32     ( _src3, _dst2 );

    _dst0 = _mm256_srai_epi32( _dst0, shift2nd );
    _dst2 = _mm256_srai_epi32( _dst2, shift2nd );

    if( isLast )
    {
      __m256i vmin = _mm256_set1_epi32( clpRng.min() );
      __m256i vmax = _mm256_set1_epi32( clpRng.max() );

      _dst0 = _mm256_max_epi32( _mm256_min_epi32( _dst0, vmax ), vmin );
      _dst2 = _mm256_max_epi32( _mm256_min_epi32( _dst2, vmax ), vmin );
    }

    _dst0x = _mm256_castsi256_si128  ( _dst0 );
    _dst1x = _mm256_extracti128_si256( _dst0, 1 );
    _dst2x = _mm256_castsi256_si128  ( _dst2 );
    _dst3x = _mm256_extracti128_si256( _dst2, 1 );
  }
  else
#endif
  {
    ALIGN_DATA( 64, const TFilterCoeff coeffV[10] ) = { 0, 0, 0, _coeffV[3], _coeffV[2], _coeffV[1], _coeffV[0], 0, };

    __m128i cH    = _mm_loadl_epi64  ( ( const __m128i* ) coeffH );
            cH    = _mm_unpacklo_epi64( cH, cH );
    __m128i off   = _mm_set1_epi32   ( offset1st );
    __m128i _src1, _src2, _srcx, cV;

    _dst0x = _mm_set1_epi32( offset2nd );
    _dst1x = _mm_set1_epi32( offset2nd );
    _dst2x = _mm_set1_epi32( offset2nd );
    _dst3x = _mm_set1_epi32( offset2nd );

    for( row = 0; row < 7; row++ )
    {
      _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + 3 * srcStride ), _MM_HINT_T0 );

      cV = _mm_loadl_epi64      ( ( const __m128i* ) ( coeffV + 7 - row - 1 ) );
      cV = _mm_cvtepu16_epi32   ( cV );

      _src1 = _mm_loadl_epi64   ( ( const __m128i* ) &src[0] );
      _src2 = _mm_loadl_epi64   ( ( const __m128i* ) &src[1] );
      _src1 = _mm_unpacklo_epi64( _src1, _src2 );

      _srcx = _mm_madd_epi16    ( _src1, cH );

      _src1 = _mm_loadl_epi64   ( ( const __m128i* ) &src[2] );
      _src2 = _mm_loadl_epi64   ( ( const __m128i* ) &src[3] );
      _src1 = _mm_unpacklo_epi64( _src1, _src2 );

      _src2 = _mm_madd_epi16    ( _src1, cH );

      _src2 = _mm_hadd_epi32    ( _srcx, _src2 );
      _src2 = _mm_add_epi32     ( _src2, off );
      _src2 = _mm_srai_epi32    ( _src2, shift1st );

      _src1 = _mm_shuffle_epi32 ( _src2, 0 );
      _src1 = _mm_madd_epi16    ( _src1, cV );
      _dst0x = _mm_add_epi32    ( _src1, _dst0x );

      _src1 = _mm_shuffle_epi32 ( _src2, 1 + 4 + 16 + 64 );
      _src1 = _mm_madd_epi16    ( _src1, cV );
      _dst1x = _mm_add_epi32    ( _src1, _dst1x );

      _src1 = _mm_shuffle_epi32 ( _src2, 2 + 8 + 32 + 128 );
      _src1 = _mm_madd_epi16    ( _src1, cV );
      _dst2x = _mm_add_epi32    ( _src1, _dst2x );

      _src1 = _mm_shuffle_epi32 ( _src2, 3 + 12 + 48 + 192 );
      _src1 = _mm_madd_epi16    ( _src1, cV );
      _dst3x = _mm_add_epi32    ( _src1, _dst3x );

      INCY( src, srcStride );
    }

    _dst0x = _mm_srai_epi32( _dst0x, shift2nd );
    _dst1x = _mm_srai_epi32( _dst1x, shift2nd );
    _dst2x = _mm_srai_epi32( _dst2x, shift2nd );
    _dst3x = _mm_srai_epi32( _dst3x, shift2nd );

    if( isLast )
    {
      __m128i vmin = _mm_set1_epi32( clpRng.min() );
      __m128i vmax = _mm_set1_epi32( clpRng.max() );

      _dst0x = _mm_max_epi32( _mm_min_epi32( _dst0x, vmax ), vmin );
      _dst1x = _mm_max_epi32( _mm_min_epi32( _dst1x, vmax ), vmin );
      _dst2x = _mm_max_epi32( _mm_min_epi32( _dst2x, vmax ), vmin );
      _dst3x = _mm_max_epi32( _mm_min_epi32( _dst3x, vmax ), vmin );
    }
  }

  __m128i a01b01 = _mm_unpacklo_epi32( _dst0x, _dst1x );
  __m128i a23b23 = _mm_unpackhi_epi32( _dst0x, _dst1x );
  __m128i c01d01 = _mm_unpacklo_epi32( _dst2x, _dst3x );
  __m128i c23d23 = _mm_unpackhi_epi32( _dst2x, _dst3x );

  _dst0x = _mm_unpacklo_epi64( a01b01, c01d01 );
  _dst1x = _mm_unpackhi_epi64( a01b01, c01d01 );
  _dst2x = _mm_unpacklo_epi64( a23b23, c23d23 );
  _dst3x = _mm_unpackhi_epi64( a23b23, c23d23 );

  _dst0x = _mm_packs_epi32( _dst0x, zerox );
  _dst1x = _mm_packs_epi32( _dst1x, zerox );
  _dst2x = _mm_packs_epi32( _dst2x, zerox );
  _dst3x = _mm_packs_epi32( _dst3x, zerox );

  Pel *realDstLines[4] = { dst, dst + dstStride, dst + 2 * dstStride, dst + 3 * dstStride };

  _mm_storel_epi64( ( __m128i* ) realDstLines[0], _dst0x );
  _mm_storel_epi64( ( __m128i* ) realDstLines[1], _dst1x );
  _mm_storel_epi64( ( __m128i* ) realDstLines[2], _dst2x );
  _mm_storel_epi64( ( __m128i* ) realDstLines[3], _dst3x );
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext, bool isLast>
void simdFilter16xX_N8( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  OFFSET( src, srcStride, -3, -3 );

  _mm_prefetch( ( const char* ) ( src ), _MM_HINT_T0 );

  int offset1st, offset2nd;
  int headRoom  = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st  = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  int extHeight = height + 7;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  shift1st -= headRoom;
  offset1st = -IF_INTERNAL_OFFS << shift1st;

  if( isLast )
  {
    shift2nd  += headRoom;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    offset2nd = 0;
  }

#if USE_AVX2
  if( vext >= AVX2 )
  {
    static const int filterSpan = 8;

    __m256i voffset1   = _mm256_set1_epi32( offset1st );
    __m256i vibdimin   = _mm256_set1_epi16( clpRng.min() );
    __m256i vibdimax   = _mm256_set1_epi16( clpRng.max() );
    __m256i vsum, vsuma, vsumb;

    __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                      0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
    __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                      0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );

    int vcoeffh[4];
    int vcoeffv[4];
    
    __m256i vsrcv  [8];

    for( int i = 0; i < 8; i += 2 )
    {
      vcoeffh[i/2] = ( coeffH[i] & 0xffff ) | ( coeffH[i+1] << 16 );
      vcoeffv[i/2] = ( coeffV[i] & 0xffff ) | ( coeffV[i+1] << 16 );
    }

    for( int row = 0; row < extHeight; row++ )
    {
      _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + ( 16 >> 1 ) + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + 16 + filterSpan + 2 * srcStride ), _MM_HINT_T0 );

      __m256i vsrca0, vsrca1, vsrcb0, vsrcb1;
      __m256i vsrc0 = _mm256_loadu_si256( ( const __m256i * ) &src[0] );
      __m256i vsrc1 = _mm256_loadu_si256( ( const __m256i * ) &src[4] );

      vsrca0 = _mm256_shuffle_epi8 ( vsrc0, vshuf0 );
      vsrca1 = _mm256_shuffle_epi8 ( vsrc0, vshuf1 );  
      vsrc0  = _mm256_loadu_si256  ( ( const __m256i * ) &src[8] );
      vsuma  = _mm256_add_epi32    ( _mm256_madd_epi16( vsrca0, _mm256_set1_epi32( vcoeffh[0] ) ), _mm256_madd_epi16( vsrca1, _mm256_set1_epi32( vcoeffh[1] ) ) );
      vsrcb0 = _mm256_shuffle_epi8 ( vsrc1, vshuf0 );
      vsrcb1 = _mm256_shuffle_epi8 ( vsrc1, vshuf1 );
      vsumb  = _mm256_add_epi32    ( _mm256_madd_epi16( vsrcb0, _mm256_set1_epi32( vcoeffh[0] ) ), _mm256_madd_epi16( vsrcb1, _mm256_set1_epi32( vcoeffh[1] ) ) );
      vsrc1  = _mm256_add_epi32    ( _mm256_madd_epi16( vsrcb0, _mm256_set1_epi32( vcoeffh[2] ) ), _mm256_madd_epi16( vsrcb1, _mm256_set1_epi32( vcoeffh[3] ) ) );
      vsrca0 = _mm256_shuffle_epi8 ( vsrc0, vshuf0 );
      vsrca1 = _mm256_shuffle_epi8 ( vsrc0, vshuf1 );
      vsrc0  = _mm256_add_epi32    ( _mm256_madd_epi16( vsrca0, _mm256_set1_epi32( vcoeffh[2] ) ), _mm256_madd_epi16( vsrca1, _mm256_set1_epi32( vcoeffh[3] ) ) );
      vsuma  = _mm256_add_epi32    ( vsuma, vsrc1 );
      vsumb  = _mm256_add_epi32    ( vsumb, vsrc0 );

      vsuma = _mm256_add_epi32  ( vsuma, voffset1 );
      vsumb = _mm256_add_epi32  ( vsumb, voffset1 );
      vsuma = _mm256_srai_epi32 ( vsuma, shift1st );
      vsumb = _mm256_srai_epi32 ( vsumb, shift1st );
      vsum  = _mm256_packs_epi32( vsuma, vsumb );

      if( row < 7 )
      {
        vsrcv[row + 1] = vsum;
      }
      else
      {
        for( int i = 0; i < 7; i++ )
        {
          vsrcv[i] = vsrcv[i + 1];
        }
        vsrcv[7] = vsum;
        
        vsuma = _mm256_set1_epi32( offset2nd );
        vsumb = _mm256_set1_epi32( offset2nd );
        for( int i=0; i<8; i+=2 )
        {
          __m256i vsrca = _mm256_unpacklo_epi16( vsrcv[i], vsrcv[i+1] );
          __m256i vsrcb = _mm256_unpackhi_epi16( vsrcv[i], vsrcv[i+1] );
          vsuma  = _mm256_add_epi32( vsuma, _mm256_madd_epi16( vsrca, _mm256_set1_epi32( vcoeffv[i/2] ) ) );
          vsumb  = _mm256_add_epi32( vsumb, _mm256_madd_epi16( vsrcb, _mm256_set1_epi32( vcoeffv[i/2] ) ) );
        }

        vsuma = _mm256_srai_epi32 ( vsuma, shift2nd );
        vsumb = _mm256_srai_epi32 ( vsumb, shift2nd );

        vsum  = _mm256_packs_epi32( vsuma, vsumb );

        if( isLast )
        { //clip
          vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
        }

        _mm256_storeu_si256( ( __m256i * ) dst, vsum );
        
        INCY( dst, dstStride );
      }

      INCY( src, srcStride );
    }
  }
  else
#endif
  {
    Pel* tmp = ( Pel* ) alloca( 16 * extHeight * sizeof( Pel ) );
    VALGRIND_MEMCLEAR( tmp );

    simdInterpolateHorM8<vext, 8, false >( src, srcStride, tmp, 16, 16, extHeight, shift1st, offset1st, clpRng, coeffH );
    simdInterpolateVerM8<vext, 8, isLast>( tmp, 16, dst, dstStride, 16,    height, shift2nd, offset2nd, clpRng, coeffV );
  }
}

template<X86_VEXT vext, bool isLast>
void simdFilter16xX_N4( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  OFFSET( src, srcStride, -1, -1 );

  _mm_prefetch( ( const char* ) ( src ), _MM_HINT_T0 );

  int offset1st, offset2nd;
  int headRoom = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  shift1st -= headRoom;
  offset1st = -IF_INTERNAL_OFFS << shift1st;

  if( isLast )
  {
    shift2nd  += headRoom;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    offset2nd = 0;
  }

  const int extHeight = height + 3;

  Pel* tmp = ( Pel* ) alloca( 16 * extHeight * sizeof( Pel ) );
  VALGRIND_MEMCLEAR( tmp );

#if USE_AVX2
  if( vext >= AVX2 )
  {
    simdInterpolateHorM16_AVX2<vext, 4, false >( src, srcStride, tmp, 16, 16, extHeight, shift1st, offset1st, clpRng, coeffH );
    simdInterpolateVerM16_AVX2<vext, 4, isLast>( tmp, 16, dst, dstStride, 16,    height, shift2nd, offset2nd, clpRng, coeffV );
  }
  else
#endif
  {
    simdInterpolateHorM8<vext, 4, false >( src, srcStride, tmp, 16, 16, extHeight, shift1st, offset1st, clpRng, coeffH );
    simdInterpolateVerM8<vext, 4, isLast>( tmp, 16, dst, dstStride, 16,    height, shift2nd, offset2nd, clpRng, coeffV );
  }
}

template<X86_VEXT vext, bool isLast>
void simdFilter8xX_N8( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  OFFSET( src, srcStride, -3, -3 );

  _mm_prefetch( ( const char* ) ( src ), _MM_HINT_T0 );

  int offset1st, offset2nd;
  int headRoom = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  shift1st -= headRoom;
  offset1st = -IF_INTERNAL_OFFS << shift1st;

  if( isLast )
  {
    shift2nd  += headRoom;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    offset2nd = 0;
  }

  const int extHeight = height + 7;

#if USE_AVX2
  if( vext >= AVX2 )
  {
    static const int filterSpan = 8;

    __m256i voffset1   = _mm256_set1_epi32( offset1st );
    __m128i xbdimin    = _mm_set1_epi16( clpRng.min() );
    __m128i xbdimax    = _mm_set1_epi16( clpRng.max() );
    __m256i vsum;

    __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                      0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
    __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                      0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );

    int vcoeffh[4];
    int vcoeffv[4];
    
    __m128i xsrcv  [8];

    for( int i = 0; i < 8; i += 2 )
    {
      vcoeffh[i / 2] = ( coeffH[i] & 0xffff ) | ( coeffH[i + 1] << 16 );
      vcoeffv[i / 2] = ( coeffV[i] & 0xffff ) | ( coeffV[i + 1] << 16 );
    }

    for( int row = 0; row < extHeight; row++ )
    {
      _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + ( 16 >> 1 ) + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + 16 + filterSpan + 2 * srcStride ), _MM_HINT_T0 );

      __m128i xsrc0 = _mm_loadu_si128( ( const __m128i * ) &src[0] );
      __m128i xsrc1 = _mm_loadu_si128( ( const __m128i * ) &src[4] );

      __m256i vsrc0, vsrca0, vsrca1;

      vsrc0  = _mm256_castsi128_si256   ( xsrc0 );
      vsrc0  = _mm256_inserti128_si256  ( vsrc0, xsrc1, 1 );
      vsrca0 = _mm256_shuffle_epi8      ( vsrc0, vshuf0 );
      vsrca1 = _mm256_shuffle_epi8      ( vsrc0, vshuf1 );
      vsum   = _mm256_add_epi32         ( _mm256_madd_epi16( vsrca0, _mm256_set1_epi32( vcoeffh[0] ) ), _mm256_madd_epi16( vsrca1, _mm256_set1_epi32( vcoeffh[1] ) ) );
      
      xsrc0  = _mm_loadu_si128          ( (const __m128i *) &src[8] );

      vsrc0  = _mm256_castsi128_si256   ( xsrc1 );
      vsrc0  = _mm256_inserti128_si256  ( vsrc0, xsrc0, 1 );
      vsrca0 = _mm256_shuffle_epi8      ( vsrc0, vshuf0 );
      vsrca1 = _mm256_shuffle_epi8      ( vsrc0, vshuf1 );
      vsum   = _mm256_add_epi32         ( vsum, _mm256_add_epi32( _mm256_madd_epi16( vsrca0, _mm256_set1_epi32( vcoeffh[2] ) ), _mm256_madd_epi16( vsrca1, _mm256_set1_epi32( vcoeffh[3] ) ) ) );

      vsum   = _mm256_add_epi32         ( vsum, voffset1 );
      vsum   = _mm256_srai_epi32        ( vsum, shift1st );

      __m128i
      xsump  = _mm256_cvtepi32_epi16x   ( vsum );

      if( row < 7 )
      {
        xsrcv[row + 1] = xsump;
      }
      else
      {
        for( int i = 0; i < 7; i++ )
        {
          xsrcv[i] = xsrcv[i + 1];
        }
        xsrcv[7] = xsump;
        
        vsum = _mm256_set1_epi32( offset2nd );

        for( int i = 0; i < 8; i += 2 )
        {
          __m128i xsrc0 = _mm_unpacklo_epi16( xsrcv[i], xsrcv[i + 1] );
          __m128i xsrc1 = _mm_unpackhi_epi16( xsrcv[i], xsrcv[i + 1] );
          __m256i vsrc  = _mm256_inserti128_si256( _mm256_castsi128_si256( xsrc0 ), xsrc1, 1 );
          vsum          = _mm256_add_epi32( vsum, _mm256_madd_epi16( vsrc, _mm256_set1_epi32( vcoeffv[i / 2] ) ) );
        }

        vsum = _mm256_srai_epi32 ( vsum, shift2nd );

        __m128i xsum = _mm256_cvtepi32_epi16x( vsum );

        if( isLast )
        { //clip
          xsum = _mm_min_epi16( xbdimax, _mm_max_epi16( xbdimin, xsum ) );
        }

        _mm_storeu_si128( ( __m128i * ) dst, xsum );

        INCY( dst, dstStride );
      }

      INCY( src, srcStride );
    }
  }
  else
#endif
  {
    Pel* tmp = ( Pel* ) alloca( 8 * extHeight * sizeof( Pel ) );
    VALGRIND_MEMCLEAR( tmp );

    simdInterpolateHorM8<vext, 8, false >( src, srcStride, tmp, 8, 8, extHeight, shift1st, offset1st, clpRng, coeffH );
    simdInterpolateVerM8<vext, 8, isLast>( tmp, 8, dst, dstStride, 8,    height, shift2nd, offset2nd, clpRng, coeffV );
  }
}

template<X86_VEXT vext, bool isLast>
void simdFilter8xX_N4( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  OFFSET( src, srcStride, -1, -1 );

  _mm_prefetch( ( const char* ) src, _MM_HINT_T0 );

  int offset1st, offset2nd;
  int headRoom = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  shift1st -= headRoom;
  offset1st = -IF_INTERNAL_OFFS << shift1st;

  if( isLast )
  {
    shift2nd  += headRoom;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    offset2nd = 0;
  }

  const int extHeight = height + 3;

#if USE_AVX2
  if( vext >= AVX2 )
  {
    static const int width      = 8;
    static const int N          = 4;
    static const int filterSpan = N - 1;

    _mm_prefetch( ( const char* ) ( src + srcStride ), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) ( src + ( width >> 1 ) + srcStride ), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) ( src + width + filterSpan + srcStride ), _MM_HINT_T0 );

    __m256i voffset1   = _mm256_set1_epi32( offset1st );
    __m128i vibdimin   = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax   = _mm_set1_epi16( clpRng.max() );

    __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                      0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
    __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                      0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );

    int vcoeffh[2];
    int vcoeffv[2];

    __m128i vsrcv  [4];

    for( int i = 0; i < 4; i += 2 )
    {
      vcoeffh[i / 2] = ( coeffH[i] & 0xffff ) | ( coeffH[i + 1] << 16 );
      vcoeffv[i / 2] = ( coeffV[i] & 0xffff ) | ( coeffV[i + 1] << 16 );
    }

    __m256i vsum;

    for( int row = 0; row < extHeight; row++ )
    {
      _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + ( width >> 1 ) + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + width + filterSpan + 2 * srcStride ), _MM_HINT_T0 );

      __m256i vtmp02, vtmp13;

      __m256i vsrc = _mm256_castsi128_si256(   _mm_loadu_si128( ( const __m128i* )&src[0] ) );
      vsrc    = _mm256_inserti128_si256( vsrc, _mm_loadu_si128( ( const __m128i* )&src[4] ), 1 );

      vtmp02  = _mm256_shuffle_epi8( vsrc, vshuf0 );
      vtmp13  = _mm256_shuffle_epi8( vsrc, vshuf1 );

      vtmp02  = _mm256_madd_epi16  ( vtmp02, _mm256_set1_epi32( vcoeffh[0] ) );
      vtmp13  = _mm256_madd_epi16  ( vtmp13, _mm256_set1_epi32( vcoeffh[1] ) );
      vsum    = _mm256_add_epi32   ( vtmp02, vtmp13 );

      vsum    = _mm256_add_epi32   ( vsum, voffset1 );
      vsum    = _mm256_srai_epi32  ( vsum, shift1st );

      __m128i vsump = _mm256_cvtepi32_epi16x( vsum );

      if( row < 3 )
      {
        vsrcv[row + 1] = vsump;
      }
      else
      {
        for( int i = 0; i < 3; i++ )
        {
          vsrcv[i] = vsrcv[i + 1];
        }
        vsrcv[3] = vsump;

        vsum = _mm256_set1_epi32( offset2nd );

        for( int i=0; i<N; i+=2 )
        {
          __m128i vsrc0 = _mm_unpacklo_epi16( vsrcv[i], vsrcv[i+1] );
          __m128i vsrc1 = _mm_unpackhi_epi16( vsrcv[i], vsrcv[i+1] );
          __m256i vsrc  = _mm256_inserti128_si256( _mm256_castsi128_si256( vsrc0 ), vsrc1, 1 );
          vsum  = _mm256_add_epi32( vsum, _mm256_madd_epi16( vsrc, _mm256_set1_epi32( vcoeffv[i/2] ) ) );
        }

        vsum = _mm256_srai_epi32( vsum, shift2nd );

        vsump = _mm256_cvtepi32_epi16x( vsum );

        if( isLast )
        { //clip
          vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
        }

        _mm_storeu_si128( ( __m128i * ) dst, vsump );

        INCY( dst, dstStride );
      }

      INCY( src, srcStride );
    }
  }
  else
#endif
  {
    Pel* tmp = ( Pel* ) alloca( 8 * extHeight * sizeof( Pel ) );
    VALGRIND_MEMCLEAR( tmp );

    simdInterpolateHorM8<vext, 4, false >( src, srcStride, tmp, 8, 8, extHeight, shift1st, offset1st, clpRng, coeffH );
    simdInterpolateVerM8<vext, 4, isLast>( tmp, 8, dst, dstStride, 8,    height, shift2nd, offset2nd, clpRng, coeffV );
  }
}

#if USE_AVX2 && defined( VVCLIB_OWN_mm256_set_m128i )
#undef VVCLIB_OWN_mm256_set_m128i
#undef _mm256_set_m128i
#endif

template< X86_VEXT vext >
void xWeightedGeoBlk_SSE(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, const ClpRng& clpRng)
{
  Pel* dst = predDst.get(compIdx).buf;
  Pel* src0 = predSrc0.get(compIdx).buf;
  Pel* src1 = predSrc1.get(compIdx).buf;
  ptrdiff_t strideDst = predDst.get(compIdx).stride;
  ptrdiff_t strideSrc0 = predSrc0.get(compIdx).stride;
  ptrdiff_t strideSrc1 = predSrc1.get(compIdx).stride;

  const char    log2WeightBase = 3;
//  const ClpRng  clpRng = pu.slice->clpRngs().comp[compIdx];
  const int32_t shiftWeighted = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd)) + log2WeightBase;
  const int32_t offsetWeighted = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);

  int16_t wIdx = getLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = getLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t angle = g_GeoParams[splitDir][0];
  int16_t stepY = 0;
  int16_t* weight = nullptr;
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }

  const __m128i mmEight = _mm_set1_epi16(8);
  const __m128i mmOffset = _mm_set1_epi32(offsetWeighted);
  const __m128i mmShift = _mm_cvtsi32_si128(shiftWeighted);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min());
  const __m128i mmMax = _mm_set1_epi16(clpRng.max());

  if (compIdx != COMPONENT_Y && pu.chromaFormat == CHROMA_420)
    stepY <<= 1;
  if (width == 4)
  {
    // it will occur to chroma only
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *) (src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *) (src1));
      __m128i w0;
      if (g_angle2mirror[angle] == 1)
      {
        w0 = _mm_loadu_si128((__m128i *) (weight - (8 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
        w0 = _mm_shuffle_epi8(w0, shuffle_mask);
      }
      else
      {
        w0 = _mm_loadu_si128((__m128i *) (weight));
      }
      w0 = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0 = _mm_unpacklo_epi16(s0, s1);
      w0 = _mm_unpacklo_epi16(w0, w1);
      s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0 = _mm_sra_epi32(s0, mmShift);
      s0 = _mm_packs_epi32(s0, s0);
      s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
      _mm_storel_epi64((__m128i *) (dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (width >= 16)
  {
    const __m256i mmEightAVX2 = _mm256_set1_epi16(8);
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(offsetWeighted);
    const __m256i mmMinAVX2 = _mm256_set1_epi16(clpRng.min());
    const __m256i mmMaxAVX2 = _mm256_set1_epi16(clpRng.max());
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *) (src0 + x)); // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *) (src1 + x));

        __m256i w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
        if (compIdx != COMPONENT_Y &&  pu.chromaFormat != CHROMA_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - (16 - 1))); // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - 16 - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0 = _mm256_shuffle_epi8(w0p0, shuffle_mask);
            w0p0 = _mm256_permute4x64_epi64(w0p0, _MM_SHUFFLE(1, 0, 3, 2));
            w0p1 = _mm256_shuffle_epi8(w0p1, shuffle_mask);
            w0p1 = _mm256_permute4x64_epi64(w0p1, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1))); // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1) + 16));
          }
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0 = _mm256_packs_epi16(w0p0, w0p1);
          w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight - x - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0 = _mm256_shuffle_epi8(w0, shuffle_mask);
            w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
          }
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w0, w1);
        s0tmp = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w0, w1);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        s0 = _mm256_min_epi16(mmMaxAVX2, _mm256_max_epi16(s0, mmMinAVX2));
        _mm256_storeu_si256((__m256i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *) (src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *) (src1 + x));
        __m128i w0;
        if (compIdx != COMPONENT_Y && pu.chromaFormat != CHROMA_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - (8 - 1))); // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - 8 - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0 = _mm_shuffle_epi8(w0p0, shuffle_mask);
            w0p1 = _mm_shuffle_epi8(w0p1, shuffle_mask);
          }
          else
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight + (x << 1))); // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight + (x << 1) + 8));
          }
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0 = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight - x - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0 = _mm_shuffle_epi8(w0, shuffle_mask);
          }
          else
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight + x));
          }
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w0, w1);
        s0tmp = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w0, w1);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
        _mm_storeu_si128((__m128i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}

template <X86_VEXT vext>
void InterpolationFilter::_initInterpolationFilterX86()
{
  // [taps][bFirst][bLast]
  m_filterHor[0][0][0] = simdFilter<vext, 8, false, false, false>;
  m_filterHor[0][0][1] = simdFilter<vext, 8, false, false, true>;
  m_filterHor[0][1][0] = simdFilter<vext, 8, false, true, false>;
  m_filterHor[0][1][1] = simdFilter<vext, 8, false, true, true>;

  m_filterHor[1][0][0] = simdFilter<vext, 4, false, false, false>;
  m_filterHor[1][0][1] = simdFilter<vext, 4, false, false, true>;
  m_filterHor[1][1][0] = simdFilter<vext, 4, false, true, false>;
  m_filterHor[1][1][1] = simdFilter<vext, 4, false, true, true>;

  m_filterHor[2][0][0] = simdFilter<vext, 2, false, false, false>;
  m_filterHor[2][0][1] = simdFilter<vext, 2, false, false, true>;
  m_filterHor[2][1][0] = simdFilter<vext, 2, false, true, false>;
  m_filterHor[2][1][1] = simdFilter<vext, 2, false, true, true>;

  m_filterVer[0][0][0] = simdFilter<vext, 8, true, false, false>;
  m_filterVer[0][0][1] = simdFilter<vext, 8, true, false, true>;
  m_filterVer[0][1][0] = simdFilter<vext, 8, true, true, false>;
  m_filterVer[0][1][1] = simdFilter<vext, 8, true, true, true>;

  m_filterVer[1][0][0] = simdFilter<vext, 4, true, false, false>;
  m_filterVer[1][0][1] = simdFilter<vext, 4, true, false, true>;
  m_filterVer[1][1][0] = simdFilter<vext, 4, true, true, false>;
  m_filterVer[1][1][1] = simdFilter<vext, 4, true, true, true>;

  m_filterVer[2][0][0] = simdFilter<vext, 2, true, false, false>;
  m_filterVer[2][0][1] = simdFilter<vext, 2, true, false, true>;
  m_filterVer[2][1][0] = simdFilter<vext, 2, true, true, false>;
  m_filterVer[2][1][1] = simdFilter<vext, 2, true, true, true>;

  m_filterCopy[0][0]   = simdFilterCopy<vext, false, false>;
  m_filterCopy[0][1]   = simdFilterCopy<vext, false, true>;
  m_filterCopy[1][0]   = simdFilterCopy<vext, true, false>;
  m_filterCopy[1][1]   = simdFilterCopy<vext, true, true>;

  m_filter4x4[0][0]    = simdFilter4x4_N6<vext, false>;
  m_filter4x4[0][1]    = simdFilter4x4_N6<vext, true>;

  m_filter4x4[1][0]    = simdFilter4x4_N4<vext, false>;
  m_filter4x4[1][1]    = simdFilter4x4_N4<vext, true>;

  m_filter8x8[0][0]    = simdFilter8xX_N8<vext, false>;
  m_filter8x8[0][1]    = simdFilter8xX_N8<vext, true>;

  m_filter8x8[1][0]    = simdFilter8xX_N4<vext, false>;
  m_filter8x8[1][1]    = simdFilter8xX_N4<vext, true>;

  m_filter16x16[0][0]    = simdFilter16xX_N8<vext, false>;
  m_filter16x16[0][1]    = simdFilter16xX_N8<vext, true>;

  m_filter16x16[1][0]    = simdFilter16xX_N4<vext, false>;
  m_filter16x16[1][1]    = simdFilter16xX_N4<vext, true>;

  m_filterN2_2D = simdInterpolateN2_2D<vext>;

  m_weightedGeoBlk = xWeightedGeoBlk_SSE<vext>;
}

template void InterpolationFilter::_initInterpolationFilterX86<SIMDX86>();

#endif //#ifdef TARGET_SIMD_X86
//! \}
