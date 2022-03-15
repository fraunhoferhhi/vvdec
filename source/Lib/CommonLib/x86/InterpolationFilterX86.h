/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

#ifdef TARGET_SIMD_X86

namespace vvdec
{

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
    const ptrdiff_t halfLine = width >> 1;

    _mm_prefetch( (const char*) &src[0], _MM_HINT_T0 );
    _mm_prefetch( (const char*) &src[0 + halfLine], _MM_HINT_T0 );

    for( int row = 0; row < height; row++ )
    {
      _mm_prefetch( ( const char* ) &src[srcStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) &src[srcStride + halfLine], _MM_HINT_T0 );

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


// SIMD interpolation horizontal, block width modulo 8
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM1(const int16_t* src, int srcStride, int16_t* dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const* coeff)
{
  _mm_prefetch((const char*)src, _MM_HINT_T0);
  _mm_prefetch((const char*)src + srcStride, _MM_HINT_T0);

  const __m128i vcoeffh  = N == 8 ? _mm_loadu_si128((__m128i const*) coeff) : _mm_set1_epi64x( *( int64_t const* ) coeff );
  const __m128i voffset  = _mm_set1_epi32(offset);
  const __m128i vibdimin = _mm_set1_epi16(clpRng.min());
  const __m128i vibdimax = _mm_set1_epi16(clpRng.max());
  const __m128i vzero    = _mm_setzero_si128();

  int row;

  for( row = 0; row < ( height & ~3 ); row += 4 )
  {
    _mm_prefetch((const char*)src + 2 * srcStride, _MM_HINT_T0);

    __m128i vsrc0;
    if( N == 8 )
    { 
      vsrc0 = _mm_loadu_si128( (__m128i const*) src ); src += srcStride;
      vsrc0 = _mm_madd_epi16( vsrc0, vcoeffh );

      __m128i
      vsrc1 = _mm_loadu_si128( (__m128i const*) src ); src += srcStride;
      vsrc1 = _mm_madd_epi16( vsrc1, vcoeffh );

      __m128i
      vsrc2 = _mm_loadu_si128( (__m128i const*) src ); src += srcStride;
      vsrc2 = _mm_madd_epi16( vsrc2, vcoeffh );

      __m128i
      vsrc3 = _mm_loadu_si128( (__m128i const*) src ); src += srcStride;
      vsrc3 = _mm_madd_epi16( vsrc3, vcoeffh );

      vsrc0 = _mm_hadd_epi32( vsrc0, vsrc1 );
      vsrc2 = _mm_hadd_epi32( vsrc2, vsrc3 );
      vsrc0 = _mm_hadd_epi32( vsrc0, vsrc2 );
    }
    else
    {
      vsrc0 = _mm_loadl_epi64( (__m128i const*) src ); src += srcStride;
      __m128i
      vsrc1 = _mm_loadl_epi64( (__m128i const*) src ); src += srcStride;
      
      vsrc0 = _mm_madd_epi16( _mm_unpacklo_epi64( vsrc0, vsrc1 ), vcoeffh );
      
      __m128i
      vsrc2 = _mm_loadl_epi64( (__m128i const*) src ); src += srcStride;
      __m128i
      vsrc3 = _mm_loadl_epi64( (__m128i const*) src ); src += srcStride;
      
      vsrc2 = _mm_madd_epi16( _mm_unpacklo_epi64( vsrc2, vsrc3 ), vcoeffh );

      vsrc0 = _mm_hadd_epi32( vsrc0, vsrc2 );
    }

    vsrc0 = _mm_add_epi32 (vsrc0, voffset);
    vsrc0 = _mm_srai_epi32(vsrc0, shift);

    if (shiftBack) { //clip
      vsrc0 = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc0));
    }
    
    *dst = _mm_cvtsi128_si32(vsrc0);    dst += dstStride;
    *dst = _mm_extract_epi32(vsrc0, 1); dst += dstStride;
    *dst = _mm_extract_epi32(vsrc0, 2); dst += dstStride;
    *dst = _mm_extract_epi32(vsrc0, 3); dst += dstStride;
  }
  

  for( ; row < height; row++ )
  {
    _mm_prefetch((const char*)src + 2 * srcStride, _MM_HINT_T0);

    __m128i
    vsrc0 = N == 8 ? _mm_loadu_si128((__m128i const*) src) : _mm_loadl_epi64( (__m128i const*) src );
    vsrc0 = _mm_madd_epi16 (vsrc0, vcoeffh);

    vsrc0 = _mm_hadd_epi32(vsrc0, vzero);
    if( N == 8 ) vsrc0 = _mm_hadd_epi32(vsrc0, vzero);

    vsrc0 = _mm_add_epi32 (vsrc0, voffset);
    vsrc0 = _mm_srai_epi32(vsrc0, shift);

    if (shiftBack) { //clip
      vsrc0 = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc0));
    }
    
    *dst = _mm_cvtsi128_si32(vsrc0);
    
    dst += dstStride;
    src += srcStride;
  }
}


// SIMD interpolation horizontal, block width modulo 2
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM2( const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  CHECKD( N != 4, "Only allowing w=2 filtering for chroma blocks using 4-tap IF" );

  _mm_prefetch( (const char*) src + srcStride, _MM_HINT_T0 );

  const __m128i voffset  = _mm_set1_epi32( offset );
  const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
  const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
  const __m128i vzero    = _mm_setzero_si128();
  const __m128i vcoeffh  = _mm_set1_epi64x( *( int64_t const* ) coeff );

  __m128i vsum, vsrc, vsrc0, vsrc1;

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)src + 2 * srcStride, _MM_HINT_T0 );

    vsrc0 = _mm_loadl_epi64( ( __m128i const* )&src[0] );
    vsrc1 = _mm_loadl_epi64( ( __m128i const* )&src[1] );
    vsrc  = _mm_unpacklo_epi64( vsrc0, vsrc1 );

    vsum  = _mm_madd_epi16( vsrc, vcoeffh );
    vsum  = _mm_hadd_epi32( vsum, vsum );

    vsum  = _mm_add_epi32  ( vsum, voffset );
    vsum  = _mm_srai_epi32 ( vsum, shift );
    vsum  = _mm_packs_epi32( vsum, vzero );

    if( shiftBack )
    { //clip
      vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
    }
    _mm_storeu_si32( ( __m128i * )&dst[0], vsum );

    src += srcStride;
    dst += dstStride;
  }
}


// SIMD interpolation horizontal, block width modulo 4
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM4( const int16_t* src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  _mm_prefetch( (const char*) src + srcStride, _MM_HINT_T0 );

  const __m128i voffset  = _mm_set1_epi32( offset );
  const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
  const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
  const __m128i vzero    = _mm_setzero_si128();

  __m128i vcoeffh;
  __m128i vshufc0, vshufc1;
  __m128i vsum;
  if( N == 8 )
  {
    vcoeffh = _mm_loadu_si128( (__m128i const*) coeff );
  }
  else
  {
    vcoeffh = _mm_set1_epi64x( *(int64_t const*) coeff );
    vshufc0 = _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
    vshufc1 = _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
  }

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

  const __m128i voffset  = _mm_set1_epi32( offset );
  const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
  const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );

  __m128i vcoeffh;
  __m128i vshufc0, vshufc1;
  __m128i vsum, vsuma, vsumb;

  if( N == 8 )
  {
    vcoeffh = _mm_loadu_si128( (__m128i const*) coeff );
  }
  else
  {
    vcoeffh = _mm_set1_epi64x( *(int64_t const*) coeff );
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
static void simdInterpolateVerM2( const int16_t* src, ptrdiff_t srcStride, int16_t* dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const* coeff )
{
  CHECKD( N != 4, "Only allowing w=2 filtering for chroma blocks using 4-tap IF" );

  _mm_prefetch( ( const char* ) &src[0 * srcStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &src[1 * srcStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &src[2 * srcStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &src[3 * srcStride], _MM_HINT_T0 );

  const __m128i vcoeffv  = _mm_set1_epi64x( *( int64_t const* ) coeff );
  const __m128i vzero    = _mm_setzero_si128();
  const __m128i voffset  = _mm_set1_epi32( offset );
  const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
  const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
  const __m128i vshuff   = _mm_set_epi8( 15, 14, 11, 10, 7, 6, 3, 2, 13, 12, 9, 8, 5, 4, 1, 0 );

  __m128i vsrc, vnl, vsum, vtmp;

  const int nextLine = srcStride * ( N - 1 );

#if 0
  // workaround for GCC-11+, TODO: understand the problem
  vsrc = _mm_unpacklo_epi32( _mm_loadu_si32( ( const __m128i* ) &src[0] ), _mm_loadu_si32( ( const __m128i* ) &src[srcStride] ) );
  vsrc = _mm_unpacklo_epi64( vsrc, _mm_loadu_si32( (const __m128i*) & src[2 * srcStride] ) );
  Pel out[8];
  _mm_storeu_si128( ( __m128i* ) out, vsrc );
  CHECK( out[0] != src[0], "" );
  CHECK( out[1] != src[1], "" );
  CHECK( out[2] != src[0 + srcStride], "" );
  CHECK( out[3] != src[1 + srcStride], "" );
  CHECK( out[4] != src[0 + 2 * srcStride], "" );
  CHECK( out[5] != src[1 + 2 * srcStride], "" );
  CHECK( out[6] != 0, "" );
  CHECK( out[7] != 0, "" );
#else
  vsrc = _mm_setr_epi32( *reinterpret_cast<const int32_t*>( &src[0] ), *reinterpret_cast<const int32_t*>( &src[1 * srcStride] ), *reinterpret_cast<const int32_t*>( &src[2 * srcStride] ), 0 );
#endif

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( ( const char* ) &src[( N + 0 ) * srcStride], _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) &src[( N + 1 ) * srcStride], _MM_HINT_T0 );

#if 0
    // workaround for GCC-11+, TODO: understand the problem
    vnl  = _mm_loadu_si32( (const __m128i*) & src[nextLine] );
#else
    vnl  = _mm_setr_epi32   ( *reinterpret_cast<const int32_t*>( &src[nextLine] ), 0, 0, 0 );
#endif
    vnl  = _mm_slli_si128   ( vnl, 12 );
    vsrc = _mm_or_si128     ( vsrc, vnl );
    vtmp = _mm_shuffle_epi8 ( vsrc, vshuff );
    vsum = _mm_madd_epi16   ( vtmp, vcoeffv );
    vsum = _mm_hadd_epi32   ( vsum, vzero );
    vsrc = _mm_srli_si128   ( vsrc, 4 );

    vsum = _mm_add_epi32    ( vsum, voffset );
    vsum = _mm_srai_epi32   ( vsum, shift );
    vsum = _mm_packs_epi32  ( vsum, vzero );

    if( shiftBack ) //clip
    {
      vsum = _mm_min_epi16  ( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
    }

    _mm_storeu_si32( (__m128i*) &dst[0], vsum );

    src += srcStride;
    dst += dstStride;
  }
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

template<X86_VEXT vext>
static void simdInterpolateN2_2D( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *ch, TFilterCoeff const *cv )
{
  const int shift1st  = IF_FILTER_PREC_BILINEAR - ( IF_INTERNAL_PREC_BILINEAR - clpRng.bd );
  const int offset1st = 1 << ( shift1st - 1 );

  const int shift2nd  = 4;
  const int offset2nd = 1 << ( shift2nd - 1 );
  
  _mm_prefetch( ( const char * ) src, _MM_HINT_T0 );

#if USE_AVX2
  if( ( ( width - 4 ) & 15 ) == 0 )  {
    __m256i mm256Offset1   = _mm256_set1_epi16( offset1st );
    __m256i mm256Offset2   = _mm256_set1_epi16( offset2nd );
    __m256i mm256CoeffH    = _mm256_set1_epi16( ch[1] );
    __m256i mm256CoeffV    = _mm256_set1_epi16( cv[1] );
    __m256i mm256LastH [8];
    __m128i mmLast4H;

    for( int row = -1; row < height; row++ )
    {
      _mm_prefetch( ( const char * ) &src[srcStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char * ) &src[4],         _MM_HINT_T0 );

      {
        __m128i mmPix  = _mm_loadl_epi64( ( const __m128i* )( src ) );
        __m128i mmPix1 = _mm_loadl_epi64( ( const __m128i* )( src + 1 ) );
        __m128i mmFiltered = _mm256_castsi256_si128( mm256Offset1 );
        mmFiltered = _mm_add_epi16 ( mmFiltered, _mm_slli_epi16( mmPix, 4 ) );
        mmFiltered = _mm_add_epi16 ( mmFiltered, _mm_mullo_epi16( _mm_sub_epi16( mmPix1, mmPix ),  _mm256_castsi256_si128( mm256CoeffH ) ) );
        mmFiltered = _mm_srai_epi16( mmFiltered, shift1st );

        if( row >= 0 )
        {
          __m128i
          mmFiltered2 = _mm256_castsi256_si128( mm256Offset2 );
          mmFiltered2 = _mm_add_epi16 ( mmFiltered2, _mm_slli_epi16 ( mmLast4H, 4 ) );
          mmFiltered2 = _mm_add_epi16 ( mmFiltered2, _mm_mullo_epi16( _mm_sub_epi16( mmFiltered, mmLast4H ), _mm256_castsi256_si128( mm256CoeffV ) ) );
          mmFiltered2 = _mm_srai_epi16( mmFiltered2, shift2nd );

          _mm_storel_epi64( ( __m128i* ) dst, mmFiltered2 );
        }

        mmLast4H = mmFiltered;
      }

      for( int x = 4; x < width; x += 16 )
      {
        _mm_prefetch( ( const char * ) &src[x + 16], _MM_HINT_T0 );
        _mm_prefetch( ( const char * ) &src[x + 32], _MM_HINT_T0 );

        __m256i mmPix   = _mm256_loadu_si256( ( const __m256i* )( src + x ) );
        __m256i mmPix1  = _mm256_loadu_si256( ( const __m256i* )( src + x + 1 ) );
        __m256i mmFiltered
                        = _mm256_add_epi16  ( mm256Offset1, _mm256_slli_epi16( mmPix, 4 ) );
        mmFiltered      = _mm256_add_epi16  ( mmFiltered, _mm256_mullo_epi16( _mm256_sub_epi16( mmPix1, mmPix ),  mm256CoeffH ) );
        mmFiltered      = _mm256_srai_epi16 ( mmFiltered, shift1st );

        int idx = x >> 4;

        __m256i m256Last = mm256LastH[idx];
        mm256LastH[idx] = mmFiltered;

        if( row >= 0 )
        {
          __m256i
          mmFiltered2 = _mm256_add_epi16  ( mm256Offset2, _mm256_slli_epi16( m256Last, 4 ) );
          mmFiltered2 = _mm256_add_epi16  ( mmFiltered2,  _mm256_mullo_epi16( _mm256_sub_epi16( mmFiltered, m256Last ), mm256CoeffV ) );
          mmFiltered2 = _mm256_srai_epi16 ( mmFiltered2,  shift2nd );

          _mm256_storeu_si256( ( __m256i* ) ( dst + x ), mmFiltered2 );
        }
      }

      if( row >= 0 ) dst += dstStride;

      src += srcStride;
    }
  }
  else
#endif
  {
    __m128i mmOffset1 = _mm_set1_epi16( offset1st );
    __m128i mmOffset2 = _mm_set1_epi16( offset2nd );
    __m128i mmCoeffH  = _mm_set1_epi16( ch[1] );
    __m128i mmCoeffV  = _mm_set1_epi16( cv[1] );
#if USE_AVX2
    __m128i mmLastH [1];
#else
    __m128i mmLastH[16];
#endif
    __m128i mmLast4H;

#ifndef REAL_TARGET_X86
    // on some platforms gcc thinks this is used uninitialized with simd-everywhere
    mmLastH[0] = _mm_setzero_si128();
#endif

    for( int row = -1; row < height; row++ )
    {
      _mm_prefetch( ( const char * ) &src[srcStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char * ) &src[4],         _MM_HINT_T0 );

      {
        __m128i mmPix  = _mm_loadl_epi64( ( const __m128i* )( src ) );
        __m128i mmPix1 = _mm_loadl_epi64( ( const __m128i* )( src + 1 ) );
        __m128i mmFiltered
                   = _mm_add_epi16 ( mmOffset1,  _mm_slli_epi16( mmPix, 4 ) );
        mmFiltered = _mm_add_epi16 ( mmFiltered, _mm_mullo_epi16( _mm_sub_epi16( mmPix1, mmPix ), mmCoeffH ) );
        mmFiltered = _mm_srai_epi16( mmFiltered, shift1st );

        if( row >= 0 )
        {
          __m128i
          mmFiltered2 = _mm_add_epi16 ( mmOffset2,   _mm_slli_epi16( mmLast4H, 4 ) );
          mmFiltered2 = _mm_add_epi16 ( mmFiltered2, _mm_mullo_epi16( _mm_sub_epi16( mmFiltered, mmLast4H ), mmCoeffV ) );
          mmFiltered2 = _mm_srai_epi16( mmFiltered2, shift2nd );

          _mm_storel_epi64( ( __m128i* ) dst, mmFiltered2 );
        }

        mmLast4H = mmFiltered;
      }

      for( int x = 4; x < width; x += 8 )
      {
#if !USE_AVX2
        _mm_prefetch( ( const char * ) &src[x +  8], _MM_HINT_T0 );
        _mm_prefetch( ( const char * ) &src[x + 16], _MM_HINT_T0 );

#endif
        __m128i mmPix   = _mm_loadu_si128( ( const __m128i* )( src + x ) );
        __m128i mmPix1  = _mm_loadu_si128( ( const __m128i* )( src + x + 1 ) );
        __m128i mmFiltered 
                        = _mm_add_epi16  ( mmOffset1,  _mm_slli_epi16( mmPix, 4 ) );
        mmFiltered      = _mm_add_epi16  ( mmFiltered, _mm_mullo_epi16( _mm_sub_epi16( mmPix1, mmPix ), mmCoeffH ) );
        mmFiltered      = _mm_srai_epi16 ( mmFiltered, shift1st );

        int idx = x >> 3;
        __m128i mLast = mmLastH[idx];
        mmLastH[idx] = mmFiltered;

        if( row >= 0 )
        {
          __m128i
          mmFiltered2 = _mm_add_epi16  ( mmOffset2,   _mm_slli_epi16( mLast, 4 ) );
          mmFiltered2 = _mm_add_epi16  ( mmFiltered2, _mm_mullo_epi16( _mm_sub_epi16( mmFiltered, mLast ),  mmCoeffV ) );
          mmFiltered2 = _mm_srai_epi16 ( mmFiltered2, shift2nd );

          _mm_storeu_si128( ( __m128i* ) ( dst + x ), mmFiltered2 );
        }
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

  const ptrdiff_t halfLine = width >> 1;

  _mm_prefetch( ( const char * ) &src[0 * srcStride + 0 * cStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char * ) &src[0 * srcStride + 1 * cStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char * ) &src[0 * srcStride + 0 * cStride + halfLine], _MM_HINT_T0 );
  _mm_prefetch( ( const char * ) &src[0 * srcStride + 1 * cStride + halfLine], _MM_HINT_T0 );

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
      _mm_prefetch( ( const char * ) &src[1 * srcStride + 0 * cStride + halfLine], _MM_HINT_T0 );
      _mm_prefetch( ( const char * ) &src[1 * srcStride + 1 * cStride + halfLine], _MM_HINT_T0 );

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
      _mm_prefetch( ( const char * ) &src[1 * srcStride + 0 * cStride + halfLine], _MM_HINT_T0 );
      _mm_prefetch( ( const char * ) &src[1 * srcStride + 1 * cStride + halfLine], _MM_HINT_T0 );

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
    else if( N == 8 && width == 1 )
    {
      simdInterpolateHorM1<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      return;
    }
    else if( N == 4 && ( width % 2 ) == 0 )
    {
      CHECKD( ( width & 1 ), "Blocks of width 1 are not allowed!" );

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
        else if( ( width % 4 ) == 0 )
          simdInterpolateHorM4<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        else
          simdInterpolateHorM2<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );

        return;
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
        else if( ( width % 4 ) == 0 )
          simdInterpolateVerM4<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        else
          simdInterpolateVerM2<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );

        return;
      }
    }
    else if( N == 4 && width == 1 )
    {
      simdInterpolateHorM1<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      return;
    }
    else if( biMCForDMVR )
    {
      if( N == 2 && !( width & 0x03 ) )
      {
        CHECKD( clpRng.bd > 10, "Bit depths over 10 not supported!" );
        simdInterpolateN2_10BIT_M4<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c );
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

    for( row = 1; row <= 9; row += 2 )
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

      if( row == 9 ) goto skip_second_line_4x4_simd_N6;

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

skip_second_line_4x4_simd_N6:
      
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
    ALIGN_DATA( 64, TFilterCoeff coeffV[18] ) = { 0, 0, 0, 0, _coeffV[6], _coeffV[5], _coeffV[4], _coeffV[3], _coeffV[2], _coeffV[1], 0, 0, 0, 0, 0, 0 };

    const __m128i cH12 = _mm_unpacklo_epi16( _mm_set1_epi16( coeffH[1] ), _mm_set1_epi16( coeffH[2] ) );
    const __m128i cH34 = _mm_unpacklo_epi16( _mm_set1_epi16( coeffH[3] ), _mm_set1_epi16( coeffH[4] ) );
    const __m128i cH56 = _mm_unpacklo_epi16( _mm_set1_epi16( coeffH[5] ), _mm_set1_epi16( coeffH[6] ) );
    const __m128i off  = _mm_set1_epi32   ( offset1st );
    __m128i _src1, _src2, _srcx, _srcy, cV;

    _dst0x = _mm_set1_epi32( offset2nd );
    _dst1x = _mm_set1_epi32( offset2nd );
    _dst2x = _mm_set1_epi32( offset2nd );
    _dst3x = _mm_set1_epi32( offset2nd );

    for( row = 0; row < 9; row += 2 )
    {
      _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + 3 * srcStride ), _MM_HINT_T0 );

      cV = _mm_unpacklo_epi16( _mm_loadl_epi64( ( const __m128i* ) ( coeffV + 10 -   row       - 1 ) ),
                               _mm_loadl_epi64( ( const __m128i* ) ( coeffV + 10 - ( row + 1 ) - 1 ) ) );
      
      // line 1
      _srcx = off;

      _src1 = _mm_loadu_si128      ( ( const __m128i* ) &src[0] );
      _src2 = _mm_loadu_si128      ( ( const __m128i* ) &src[1] );
      _srcx = _mm_add_epi32        ( _srcx, _mm_madd_epi16( _mm_unpacklo_epi16( _src1, _src2 ), cH12 ) );
      _srcx = _mm_add_epi32        ( _srcx, _mm_madd_epi16( _mm_unpackhi_epi16( _src1, _src2 ), cH56 ) );
      
      _src1 = _mm_loadl_epi64      ( ( const __m128i* ) &src[2] );
      _src2 = _mm_loadl_epi64      ( ( const __m128i* ) &src[3] );
      _srcx = _mm_add_epi32        ( _srcx, _mm_madd_epi16( _mm_unpacklo_epi16( _src1, _src2 ), cH34 ) );
      _srcx = _mm_srai_epi32       ( _srcx, shift1st );

      if( row < 8 )
      {
        // line 2
        INCY( src, srcStride );

        _srcy = off;
        
        _src1 = _mm_loadu_si128    ( ( const __m128i* ) &src[0] );
        _src2 = _mm_loadu_si128    ( ( const __m128i* ) &src[1] );
        _srcy = _mm_add_epi32      ( _srcy, _mm_madd_epi16( _mm_unpacklo_epi16( _src1, _src2 ), cH12 ) );
        _srcy = _mm_add_epi32      ( _srcy, _mm_madd_epi16( _mm_unpackhi_epi16( _src1, _src2 ), cH56 ) );
      
        _src1 = _mm_loadl_epi64    ( ( const __m128i* ) &src[2] );
        _src2 = _mm_loadl_epi64    ( ( const __m128i* ) &src[3] );
        _srcy = _mm_add_epi32      ( _srcy, _mm_madd_epi16( _mm_unpacklo_epi16( _src1, _src2 ), cH34 ) );
        _srcy = _mm_srai_epi32     ( _srcy, shift1st );
      }
      else
      {
        _srcy = _mm_setzero_si128();
      }

      _srcx = _mm_packs_epi32      ( _srcx, _srcy );
      _srcy = _mm_unpackhi_epi64   ( _srcx, _srcx );
      _srcx = _mm_unpacklo_epi16   ( _srcx, _srcy );
      
      _src1 = _mm_shuffle_epi32    ( _srcx, 0 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst0x = _mm_add_epi32       ( _src1, _dst0x );

      _src1 = _mm_shuffle_epi32    ( _srcx, 1 + 4 + 16 + 64 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst1x = _mm_add_epi32       ( _src1, _dst1x );

      _src1 = _mm_shuffle_epi32    ( _srcx, 2 + 8 + 32 + 128 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst2x = _mm_add_epi32       ( _src1, _dst2x );

      _src1 = _mm_shuffle_epi32    ( _srcx, 3 + 12 + 48 + 192 );
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

  _mm_storel_epi64( ( __m128i* ) ( dst                 ), _dst0x );
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

    for( row = 0; row <= 6; row += 2 )
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

      _src3  = _mm256_madd_epi16    ( _src1, cH );

      if( row == 6 ) goto skip_last_line_4x4_simd_N4;

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

      INCY( src, srcStride );

skip_last_line_4x4_simd_N4:

      _src1 = _mm256_hadd_epi32     ( _src3, _src2 );

      _src1 = _mm256_add_epi32      ( _src1, off );
      _src1 = _mm256_srai_epi32     ( _src1, shift1st );

      // vertical filter
      _src2  = _mm256_unpacklo_epi16( _src1, _mm256_unpackhi_epi64( _src1, _src1 ) );
      _src1  = _mm256_shuffle_epi32 ( _src2, ( 0 << 0 ) + ( 0 << 2 ) + ( 0 << 4 ) + ( 0 << 6 ) );
      _src3  = _mm256_shuffle_epi32 ( _src2, ( 2 << 0 ) + ( 2 << 2 ) + ( 2 << 4 ) + ( 2 << 6 ) );

      _src1  = _mm256_madd_epi16    ( _src1, cV );
      _src3  = _mm256_madd_epi16    ( _src3, cV );

      _dst0  = _mm256_add_epi32     ( _src1, _dst0 );
      _dst2  = _mm256_add_epi32     ( _src3, _dst2 );
    }

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
  
  _mm_prefetch( ( const char* ) ( src +      0 * srcStride ), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) ( src + 24 + 0 * srcStride ), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) ( src +      1 * srcStride ), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) ( src + 24 + 1 * srcStride ), _MM_HINT_T0 );

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
      _mm_prefetch( ( const char* ) ( src +      2 * srcStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src + 24 + 2 * srcStride ), _MM_HINT_T0 );

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
        
        vsuma = vsumb = _mm256_set1_epi32( offset2nd );

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
#if 1
    const int filterSpan = 7;

    _mm_prefetch( ( const char* ) src + srcStride, _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) src + (width >> 1) + srcStride, _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) src + width + filterSpan + srcStride, _MM_HINT_T0 );

    const __m128i voffset1 = _mm_set1_epi32( offset1st );
    const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
    const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
    const __m128i vshuf0   = _mm_set_epi8  ( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
    const __m128i vshuf1   = _mm_set_epi8  ( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );

    int32_t vcoeffh[4];
    int32_t vcoeffv[4];

    for( int i = 0; i < 8; i += 2 )
    {
      vcoeffh[i/2] = ( coeffH[i] & 0xffff ) | ( coeffH[i+1] << 16 );
      vcoeffv[i/2] = ( coeffV[i] & 0xffff ) | ( coeffV[i+1] << 16 );
    }

    __m128i vsum, vsuma, vsumb;

    __m128i vsrcv[2][8];

    for( int row = 0; row < extHeight; row++ )
    {
      _mm_prefetch( ( const char* ) src + 2 * srcStride, _MM_HINT_T0 );

      for( int j = 0; j < 2; j++ )
      {
        __m128i vsrca0, vsrca1, vsrcb0, vsrcb1;
        __m128i vsrc0 = _mm_loadu_si128( ( const __m128i* ) &src[(j << 3) + 0] );
        __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* ) &src[(j << 3) + 4] );

        vsrca0 = _mm_shuffle_epi8 ( vsrc0, vshuf0 );
        vsrca1 = _mm_shuffle_epi8 ( vsrc0, vshuf1 );  
        vsrc0  = _mm_loadu_si128  ( ( const __m128i* ) &src[(j << 3) + 8] );
        vsuma  = _mm_add_epi32    ( _mm_madd_epi16( vsrca0, _mm_set1_epi32( vcoeffh[0] ) ), _mm_madd_epi16( vsrca1, _mm_set1_epi32( vcoeffh[1] ) ) );
        vsrcb0 = _mm_shuffle_epi8 ( vsrc1, vshuf0 );
        vsrcb1 = _mm_shuffle_epi8 ( vsrc1, vshuf1 );
        vsumb  = _mm_add_epi32    ( _mm_madd_epi16( vsrcb0, _mm_set1_epi32( vcoeffh[0] ) ), _mm_madd_epi16( vsrcb1, _mm_set1_epi32( vcoeffh[1] ) ) );
        vsrc1  = _mm_add_epi32    ( _mm_madd_epi16( vsrcb0, _mm_set1_epi32( vcoeffh[2] ) ), _mm_madd_epi16( vsrcb1, _mm_set1_epi32( vcoeffh[3] ) ) );
        vsrca0 = _mm_shuffle_epi8 ( vsrc0, vshuf0 );
        vsrca1 = _mm_shuffle_epi8 ( vsrc0, vshuf1 );
        vsrc0  = _mm_add_epi32    ( _mm_madd_epi16( vsrca0, _mm_set1_epi32( vcoeffh[2] ) ), _mm_madd_epi16( vsrca1, _mm_set1_epi32( vcoeffh[3] ) ) );
        vsuma  = _mm_add_epi32    ( vsuma, vsrc1 );
        vsumb  = _mm_add_epi32    ( vsumb, vsrc0 );

        vsuma  = _mm_add_epi32    ( vsuma, voffset1 );
        vsumb  = _mm_add_epi32    ( vsumb, voffset1 );

        vsuma  = _mm_srai_epi32   ( vsuma, shift1st );
        vsumb  = _mm_srai_epi32   ( vsumb, shift1st );

        vsum   = _mm_packs_epi32  ( vsuma, vsumb );

        if( row < 7 )
        {
          vsrcv[j][row + 1] = vsum;
        }
        else
        {
          for( int i = 0; i < 7; i++ )
          {
            vsrcv[j][i] = vsrcv[j][i + 1];
          }
          vsrcv[j][7] = vsum;

          vsuma = vsumb = _mm_set1_epi32( offset2nd );

          for( int i = 0; i < 8; i += 2 )
          {
            vsrca0 = _mm_unpacklo_epi16( vsrcv[j][i], vsrcv[j][i+1] );
            vsrcb0 = _mm_unpackhi_epi16( vsrcv[j][i], vsrcv[j][i+1] );

            vsuma = _mm_add_epi32( vsuma, _mm_madd_epi16( vsrca0, _mm_set1_epi32( vcoeffv[i / 2] ) ) );
            vsumb = _mm_add_epi32( vsumb, _mm_madd_epi16( vsrcb0, _mm_set1_epi32( vcoeffv[i / 2] ) ) );
          }

          vsuma = _mm_srai_epi32( vsuma, shift2nd );
          vsumb = _mm_srai_epi32( vsumb, shift2nd );

          vsum = _mm_packs_epi32( vsuma, vsumb );

          if( isLast ) //clip
          {
            vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
          }

          _mm_storeu_si128( ( __m128i* ) &dst[j << 3], vsum );

          INCY( dst, j * dstStride );
        }
      }

      INCY( src, srcStride );
    }
#else
    Pel* tmp = ( Pel* ) alloca( 16 * extHeight * sizeof( Pel ) );
    VALGRIND_MEMCLEAR( tmp );
    
    simdInterpolateHorM8<vext, 8, false >( src, srcStride, tmp, 16, 16, extHeight, shift1st, offset1st, clpRng, coeffH );
    simdInterpolateVerM8<vext, 8, isLast>( tmp, 16, dst, dstStride, 16,    height, shift2nd, offset2nd, clpRng, coeffV );
#endif
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
  VALGRIND_MEMZERO( tmp, 16 * extHeight * sizeof( Pel ) );

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
#if 1
    const int filterSpan = 7;

    _mm_prefetch( ( const char* ) src + srcStride, _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) src + (width >> 1) + srcStride, _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) src + width + filterSpan + srcStride, _MM_HINT_T0 );

    const __m128i voffset1 = _mm_set1_epi32( offset1st );
    const __m128i voffset2 = _mm_set1_epi32( offset2nd );
    const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
    const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );

    const __m128i vshuf0   = _mm_set_epi8  ( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
    const __m128i vshuf1   = _mm_set_epi8  ( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );
    
    int vcoeffv[4];
    int vcoeffh[4];

    for( int i = 0; i < 8; i += 2 )
    {
      vcoeffh[i/2] = ( coeffH[i] & 0xffff ) | ( coeffH[i+1] << 16 );
      vcoeffv[i/2] = ( coeffV[i] & 0xffff ) | ( coeffV[i+1] << 16 );
    }

    __m128i vsum, vsuma, vsumb;

    __m128i vsrcv[8];

    for( int row = 0; row < extHeight; row++ )
    {
      _mm_prefetch( ( const char* ) src + 2 * srcStride, _MM_HINT_T0 );

      __m128i vsrca0, vsrca1, vsrcb0, vsrcb1;
      __m128i vsrc0 = _mm_loadu_si128( ( const __m128i* ) &src[0] );
      __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* ) &src[4] );

      vsrca0 = _mm_shuffle_epi8 ( vsrc0, vshuf0 );
      vsrca1 = _mm_shuffle_epi8 ( vsrc0, vshuf1 );  
      vsrc0  = _mm_loadu_si128  ( ( const __m128i* ) &src[8] );
      vsuma  = _mm_add_epi32    ( _mm_madd_epi16( vsrca0, _mm_set1_epi32( vcoeffh[0] ) ), _mm_madd_epi16( vsrca1, _mm_set1_epi32( vcoeffh[1] ) ) );
      vsrcb0 = _mm_shuffle_epi8 ( vsrc1, vshuf0 );
      vsrcb1 = _mm_shuffle_epi8 ( vsrc1, vshuf1 );
      vsumb  = _mm_add_epi32    ( _mm_madd_epi16( vsrcb0, _mm_set1_epi32( vcoeffh[0] ) ), _mm_madd_epi16( vsrcb1, _mm_set1_epi32( vcoeffh[1] ) ) );
      vsrc1  = _mm_add_epi32    ( _mm_madd_epi16( vsrcb0, _mm_set1_epi32( vcoeffh[2] ) ), _mm_madd_epi16( vsrcb1, _mm_set1_epi32( vcoeffh[3] ) ) );
      vsrca0 = _mm_shuffle_epi8 ( vsrc0, vshuf0 );
      vsrca1 = _mm_shuffle_epi8 ( vsrc0, vshuf1 );
      vsrc0  = _mm_add_epi32    ( _mm_madd_epi16( vsrca0, _mm_set1_epi32( vcoeffh[2] ) ), _mm_madd_epi16( vsrca1, _mm_set1_epi32( vcoeffh[3] ) ) );
      vsuma  = _mm_add_epi32    ( vsuma, vsrc1 );
      vsumb  = _mm_add_epi32    ( vsumb, vsrc0 );

      vsuma  = _mm_add_epi32    ( vsuma, voffset1 );
      vsumb  = _mm_add_epi32    ( vsumb, voffset1 );

      vsuma  = _mm_srai_epi32   ( vsuma, shift1st );
      vsumb  = _mm_srai_epi32   ( vsumb, shift1st );

      vsum   = _mm_packs_epi32  ( vsuma, vsumb );

      if( row < filterSpan )
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

        vsuma = vsumb = voffset2;

        for( int i = 0; i < 8; i += 2 )
        {
          const __m128i vsrca = _mm_unpacklo_epi16( vsrcv[i], vsrcv[i+1] );
          const __m128i vsrcb = _mm_unpackhi_epi16( vsrcv[i], vsrcv[i+1] );

          vsuma = _mm_add_epi32( vsuma, _mm_madd_epi16( vsrca, _mm_set1_epi32( vcoeffv[i / 2] ) ) );
          vsumb = _mm_add_epi32( vsumb, _mm_madd_epi16( vsrcb, _mm_set1_epi32( vcoeffv[i / 2] ) ) );
        }

        vsuma = _mm_srai_epi32( vsuma, shift2nd );
        vsumb = _mm_srai_epi32( vsumb, shift2nd );

        vsum = _mm_packs_epi32( vsuma, vsumb );

        if( isLast ) //clip
        {
          vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
        }

        _mm_storeu_si128( ( __m128i * )&dst[0], vsum );

        INCY( dst, dstStride );
      }

      INCY( src, srcStride );
    }
#else
    Pel* tmp = ( Pel* ) alloca( 8 * extHeight * sizeof( Pel ) );
    VALGRIND_MEMCLEAR( tmp );
    
    simdInterpolateHorM8<vext, 8, false >( src, srcStride, tmp, 8, 8, extHeight, shift1st, offset1st, clpRng, coeffH );
    simdInterpolateVerM8<vext, 8, isLast>( tmp, 8, dst, dstStride, 8,    height, shift2nd, offset2nd, clpRng, coeffV );
#endif
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

    __m256i vsrcv[4];

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
      vsum    = _mm256_packs_epi32 ( vsum, vsum );

      if( row < 3 )
      {
        vsrcv[row + 1] = vsum;
      }
      else
      {
        for( int i = 0; i < 3; i++ )
        {
          vsrcv[i] = vsrcv[i + 1];
        }
        vsrcv[3] = vsum;

        vsum = _mm256_set1_epi32( offset2nd );

        vsrc = _mm256_unpacklo_epi16( vsrcv[0], vsrcv[1] );
        vsum = _mm256_add_epi32( vsum, _mm256_madd_epi16( vsrc, _mm256_set1_epi32( vcoeffv[0] ) ) );
        vsrc = _mm256_unpacklo_epi16( vsrcv[2], vsrcv[3] );
        vsum = _mm256_add_epi32( vsum, _mm256_madd_epi16( vsrc, _mm256_set1_epi32( vcoeffv[1] ) ) );

        vsum = _mm256_srai_epi32( vsum, shift2nd );

        __m128i
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
#if 1
    const int filterSpan = 3;

    _mm_prefetch( ( const char* ) src + srcStride, _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) src + (width >> 1) + srcStride, _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) src + width + filterSpan + srcStride, _MM_HINT_T0 );

    const __m128i voffset1 = _mm_set1_epi32( offset1st );
    const __m128i voffset2 = _mm_set1_epi32( offset2nd );
    const __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
    const __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
    const __m128i vshuf0   = _mm_set_epi8  ( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
    const __m128i vshuf1   = _mm_set_epi8  ( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );
    
    int32_t vcoeffv[2], vcoeffh[2];

    for( int i = 0; i < 4; i += 2 )
    {
      vcoeffh[i/2] = ( coeffH[i] & 0xffff ) | ( coeffH[i+1] << 16 );
      vcoeffv[i/2] = ( coeffV[i] & 0xffff ) | ( coeffV[i+1] << 16 );
    }

    __m128i vsum, vsuma, vsumb;

    __m128i vsrcv[4];

    for( int row = 0; row < extHeight; row++ )
    {
      _mm_prefetch( ( const char* ) src + 2 * srcStride, _MM_HINT_T0 );

      __m128i vtmp02, vtmp13;

      __m128i vsrc0 = _mm_loadu_si128( ( const __m128i* )&src[0] );
      __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* )&src[4] );

      vtmp02  = _mm_shuffle_epi8( vsrc0, vshuf0 );
      vtmp13  = _mm_shuffle_epi8( vsrc0, vshuf1 );

      vtmp02  = _mm_madd_epi16  ( vtmp02, _mm_set1_epi32( vcoeffh[0] ) );
      vtmp13  = _mm_madd_epi16  ( vtmp13, _mm_set1_epi32( vcoeffh[1] ) );
      vsuma   = _mm_add_epi32   ( vtmp02, vtmp13 );
      
      vtmp02  = _mm_shuffle_epi8( vsrc1, vshuf0 );
      vtmp13  = _mm_shuffle_epi8( vsrc1, vshuf1 );

      vtmp02  = _mm_madd_epi16  ( vtmp02, _mm_set1_epi32( vcoeffh[0] ) );
      vtmp13  = _mm_madd_epi16  ( vtmp13, _mm_set1_epi32( vcoeffh[1] ) );
      vsumb   = _mm_add_epi32   ( vtmp02, vtmp13 );

      vsuma   = _mm_add_epi32   ( vsuma, voffset1 );
      vsumb   = _mm_add_epi32   ( vsumb, voffset1 );

      vsuma   = _mm_srai_epi32  ( vsuma, shift1st );
      vsumb   = _mm_srai_epi32  ( vsumb, shift1st );

      vsum    = _mm_packs_epi32 ( vsuma, vsumb );

      if( row < 3 )
      {
        vsrcv[row + 1] = vsum;
      }
      else
      {
        for( int i = 0; i < 3; i++ )
        {
          vsrcv[i] = vsrcv[i + 1];
        }
        vsrcv[3] = vsum;

        vsuma = vsumb = voffset2;

        for( int i = 0; i < 4; i += 2 )
        {
          const __m128i vsrca = _mm_unpacklo_epi16( vsrcv[i], vsrcv[i+1] );
          const __m128i vsrcb = _mm_unpackhi_epi16( vsrcv[i], vsrcv[i+1] );

          vsuma = _mm_add_epi32( vsuma, _mm_madd_epi16( vsrca, _mm_set1_epi32( vcoeffv[i / 2] ) ) );
          vsumb = _mm_add_epi32( vsumb, _mm_madd_epi16( vsrcb, _mm_set1_epi32( vcoeffv[i / 2] ) ) );
        }

        vsuma = _mm_srai_epi32( vsuma, shift2nd );
        vsumb = _mm_srai_epi32( vsumb, shift2nd );

        vsum = _mm_packs_epi32( vsuma, vsumb );

        if( isLast ) //clip
        {
          vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
        }

        _mm_storeu_si128( ( __m128i * )&dst[0], vsum );

        INCY( dst, dstStride );
      }

      INCY( src, srcStride );
    }
#else
    Pel* tmp = ( Pel* ) alloca( 8 * extHeight * sizeof( Pel ) );
    VALGRIND_MEMCLEAR( tmp );
    
    simdInterpolateHorM8<vext, 4, false >( src, srcStride, tmp, 8, 8, extHeight, shift1st, offset1st, clpRng, coeffH );
    simdInterpolateVerM8<vext, 4, isLast>( tmp, 8, dst, dstStride, 8,    height, shift2nd, offset2nd, clpRng, coeffV );
#endif
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

}

#endif //#ifdef TARGET_SIMD_X86
