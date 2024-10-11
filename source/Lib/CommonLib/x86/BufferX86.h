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

/** \file     BufferX86.h
    \brief    SIMD averaging.
*/

//! \ingroup CommonLib
//! \{

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP 1

#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/InterpolationFilter.h"

#if ENABLE_SIMD_OPT_BUFFER
#ifdef TARGET_SIMD_X86

namespace vvdec
{

template< X86_VEXT vext, int W >
void addAvg_SSE( const int16_t* src0, ptrdiff_t src0Stride, const int16_t* src1, ptrdiff_t src1Stride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng )
{
#if USE_AVX2
  if( W == 16 )
  {
    __m256i vone      = _mm256_set1_epi16( 1 );
    __m256i voffset   = _mm256_set1_epi32( offset );
    __m256i vibdimin  = _mm256_set1_epi16( clpRng.min() );
    __m256i vibdimax  = _mm256_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 16 )
      {
        __m256i vsrc0 = _mm256_loadu_si256( ( const __m256i* )&src0[col] );
        __m256i vsrc1 = _mm256_loadu_si256( ( const __m256i* )&src1[col] );

        __m256i vsumlo = _mm256_madd_epi16( _mm256_unpacklo_epi16( vsrc0, vsrc1 ), vone );
        __m256i vsumhi = _mm256_madd_epi16( _mm256_unpackhi_epi16( vsrc0, vsrc1 ), vone );

        vsumlo = _mm256_add_epi32        ( vsumlo, voffset );
        vsumhi = _mm256_add_epi32        ( vsumhi, voffset );
        vsumlo = _mm256_srai_epi32       ( vsumlo, shift );
        vsumhi = _mm256_srai_epi32       ( vsumhi, shift );

        __m256i vsum = _mm256_packs_epi32( vsumlo, vsumhi );
        vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );

        _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
#endif
  if( W >= 8 )
  {
    __m128i vone      = _mm_set1_epi16( 1 );
    __m128i voffset   = _mm_set1_epi32( offset );
    __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m128i vsrc0 = _mm_loadu_si128( ( const __m128i* )&src0[col] );
        __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* )&src1[col] );

        __m128i vsumlo = _mm_madd_epi16( _mm_unpacklo_epi16( vsrc0, vsrc1 ), vone );
        __m128i vsumhi = _mm_madd_epi16( _mm_unpackhi_epi16( vsrc0, vsrc1 ), vone );

        vsumlo = _mm_add_epi32        ( vsumlo, voffset );
        vsumhi = _mm_add_epi32        ( vsumhi, voffset );
        vsumlo = _mm_srai_epi32       ( vsumlo, shift );
        vsumhi = _mm_srai_epi32       ( vsumhi, shift );

        __m128i vsum = _mm_packs_epi32( vsumlo, vsumhi );
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );

        _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else if( W == 4 )
  {
    __m128i vone      = _mm_set1_epi16( 1 );
    __m128i voffset   = _mm_set1_epi32( offset );
    __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );
    __m128i vsumhi    = _mm_setzero_si128();

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsrc0 = _mm_loadu_si64( ( const __m128i* )&src0[col] );
        __m128i vsrc1 = _mm_loadu_si64( ( const __m128i* )&src1[col] );

        __m128i vsumlo = _mm_madd_epi16( _mm_unpacklo_epi16( vsrc0, vsrc1 ), vone );

        vsumlo = _mm_add_epi32        ( vsumlo, voffset );
        vsumlo = _mm_srai_epi32       ( vsumlo, shift );

        __m128i vsum = _mm_packs_epi32( vsumlo, vsumhi );
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );

        _mm_storeu_si64( ( __m128i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW_FATAL( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext, int W >
void reco_SSE( const int16_t* src0, ptrdiff_t src0Stride, const int16_t* src1, ptrdiff_t src1Stride, int16_t *dst, ptrdiff_t dstStride, int width, int height, const ClpRng& clpRng )
{
  // src0 needs to be aligned for AVX2

  if( W == 8 )
  {
#if USE_AVX2
    if( vext >= AVX2 && (width & 15) == 0 )
    {
      __m256i vbdmin = _mm256_set1_epi16( clpRng.min() );
      __m256i vbdmax = _mm256_set1_epi16( clpRng.max() );

      _mm_prefetch( ( const char* ) src0, _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) src1, _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src0 + src0Stride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( src1 + src1Stride ), _MM_HINT_T0 );

      for( int row = 0; row < height; row++ )
      {
        _mm_prefetch( ( const char* ) ( src0 + 2 * src0Stride ), _MM_HINT_T0 );
        _mm_prefetch( ( const char* ) ( src1 + 2 * src1Stride ), _MM_HINT_T0 );

        for( int col = 0; col < width; col += 16 )
        {
          __m256i vdest = _mm256_loadu_si256( (const __m256i*) & src0[col] );
          __m256i vsrc1 = _mm256_loadu_si256( (const __m256i*) & src1[col] );

          vdest = _mm256_adds_epi16( vdest, vsrc1 );
          vdest = _mm256_min_epi16 ( vbdmax, _mm256_max_epi16( vbdmin, vdest ) );

          _mm256_storeu_si256( (__m256i*) & dst[col], vdest );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst += dstStride;
      }
    }
    else
#endif
    {
      __m128i vbdmin = _mm_set1_epi16( clpRng.min() );
      __m128i vbdmax = _mm_set1_epi16( clpRng.max() );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 8 )
        {
          __m128i vdest = _mm_loadu_si128( ( const __m128i * )&src0[col] );
          __m128i vsrc1 = _mm_loadu_si128( ( const __m128i * )&src1[col] );

          vdest = _mm_adds_epi16( vdest, vsrc1 );
          vdest = _mm_min_epi16 ( vbdmax, _mm_max_epi16( vbdmin, vdest ) );

          _mm_storeu_si128( ( __m128i * )&dst[col], vdest );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  += dstStride;
      }
    }
  }
  else if( W == 4 )
  {
    __m128i vbdmin = _mm_set1_epi16( clpRng.min() );
    __m128i vbdmax = _mm_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsrc = _mm_loadu_si64( ( const __m128i * )&src0[col] );
        __m128i vdst = _mm_loadu_si64( ( const __m128i * )&src1[col] );

        vdst = _mm_adds_epi16( vdst, vsrc );
        vdst = _mm_min_epi16 ( vbdmax, _mm_max_epi16( vbdmin, vdst ) );

        _mm_storeu_si64( ( __m128i * )&dst[col], vdst );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW_FATAL( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template< X86_VEXT vext, int W >
void addWghtAvg_SSE( const int16_t* src0, ptrdiff_t src0Stride, const int16_t* src1, ptrdiff_t src1Stride, int16_t *dst, ptrdiff_t dstStride, int width, int height, int shift, int offset, int w0, int w1, const ClpRng& clpRng )
{
  if( W == 8 )
  {
#if USE_AVX2
    if( ( width & 15 ) == 0 && vext >= AVX2 )
    {
      __m256i voffset  = _mm256_set1_epi32( offset );
      __m256i vibdimin = _mm256_set1_epi16( clpRng.min() );
      __m256i vibdimax = _mm256_set1_epi16( clpRng.max() );
      __m256i vw       = _mm256_unpacklo_epi16( _mm256_set1_epi16( w0 ), _mm256_set1_epi16( w1 ) );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 16 )
        {
          __m256i vsrc0 = _mm256_loadu_si256( ( const __m256i * )&src0[col] );
          __m256i vsrc1 = _mm256_loadu_si256( ( const __m256i * )&src1[col] );

          __m256i vtmp, vsum;
          vsum = _mm256_madd_epi16       ( vw, _mm256_unpacklo_epi16( vsrc0, vsrc1 ) );
          vsum = _mm256_add_epi32        ( vsum, voffset );
          vtmp = _mm256_srai_epi32       ( vsum, shift );

          vsum = _mm256_madd_epi16       ( vw, _mm256_unpackhi_epi16( vsrc0, vsrc1 ) );
          vsum = _mm256_add_epi32        ( vsum, voffset );
          vsum = _mm256_srai_epi32       ( vsum, shift );
          vsum = _mm256_packs_epi32      ( vtmp, vsum );

          vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
          _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  +=  dstStride;
      }
    }
    else
#endif
    {
      __m128i voffset  = _mm_set1_epi32( offset );
      __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
      __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
      __m128i vw       = _mm_unpacklo_epi16( _mm_set1_epi16( w0 ), _mm_set1_epi16( w1 ) );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 8 )
        {
          __m128i vsrc0 = _mm_loadu_si128( ( const __m128i * )&src0[col] );
          __m128i vsrc1 = _mm_loadu_si128( ( const __m128i * )&src1[col] );

          __m128i vtmp, vsum;
          vsum = _mm_madd_epi16       ( vw, _mm_unpacklo_epi16( vsrc0, vsrc1 ) );
          vsum = _mm_add_epi32        ( vsum, voffset );
          vtmp = _mm_srai_epi32       ( vsum, shift );

          vsum = _mm_madd_epi16       ( vw, _mm_unpackhi_epi16( vsrc0, vsrc1 ) );
          vsum = _mm_add_epi32        ( vsum, voffset );
          vsum = _mm_srai_epi32       ( vsum, shift );
          vsum = _mm_packs_epi32      ( vtmp, vsum );

          vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
          _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  +=  dstStride;
      }
    }
  }
  else if( W == 4 )
  {
    __m128i vzero     = _mm_setzero_si128();
    __m128i voffset   = _mm_set1_epi32( offset );
    __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );
    __m128i vw        = _mm_unpacklo_epi16( _mm_set1_epi16( w0 ), _mm_set1_epi16( w1 ) );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsum = _mm_loadu_si64  ( ( const __m128i * )&src0[col] );
        __m128i vdst = _mm_loadu_si64  ( ( const __m128i * )&src1[col] );
        vsum = _mm_madd_epi16          ( vw, _mm_unpacklo_epi16( vsum, vdst ) );
        vsum = _mm_add_epi32           ( vsum, voffset );
        vsum = _mm_srai_epi32          ( vsum, shift );
        vsum = _mm_packs_epi32         ( vsum, vzero );

        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
        _mm_storeu_si64( ( __m128i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW_FATAL( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<bool doShift, bool shiftR, typename T> static inline void do_shift( T &vreg, int num );
#if USE_AVX2
template<> inline void do_shift<true,  true , __m256i>( __m256i &vreg, int num ) { vreg = _mm256_sra_epi32( vreg, _mm_cvtsi32_si128( num ) ); }
template<> inline void do_shift<true,  false, __m256i>( __m256i &vreg, int num ) { vreg = _mm256_sll_epi32( vreg, _mm_cvtsi32_si128( num ) ); }
template<> inline void do_shift<false, true , __m256i>( __m256i &vreg, int num ) { }
template<> inline void do_shift<false, false, __m256i>( __m256i &vreg, int num ) { }
#endif
template<> inline void do_shift<true,  true , __m128i>( __m128i &vreg, int num ) { vreg = _mm_sra_epi32( vreg, _mm_cvtsi32_si128( num ) ); }
template<> inline void do_shift<true,  false, __m128i>( __m128i &vreg, int num ) { vreg = _mm_sll_epi32( vreg, _mm_cvtsi32_si128( num ) ); }
template<> inline void do_shift<false, true , __m128i>( __m128i &vreg, int num ) { }
template<> inline void do_shift<false, false, __m128i>( __m128i &vreg, int num ) { }

template<bool mult, typename T> static inline void do_mult( T& vreg, T& vmult );
template<> inline void do_mult<false, __m128i>( __m128i&, __m128i& ) { }
#if USE_AVX2
template<> inline void do_mult<false, __m256i>( __m256i&, __m256i& ) { }
#endif
template<> inline void do_mult<true,   __m128i>( __m128i& vreg, __m128i& vmult ) { vreg = _mm_mullo_epi32   ( vreg, vmult ); }
#if USE_AVX2
template<> inline void do_mult<true,   __m256i>( __m256i& vreg, __m256i& vmult ) { vreg = _mm256_mullo_epi32( vreg, vmult ); }
#endif

template<bool add, typename T> static inline void do_add( T& vreg, T& vadd );
template<> inline void do_add<false, __m128i>( __m128i&, __m128i& ) { }
#if USE_AVX2
template<> inline void do_add<false, __m256i>( __m256i&, __m256i& ) { }
#endif
template<> inline void do_add<true,  __m128i>( __m128i& vreg, __m128i& vadd ) { vreg = _mm_add_epi32( vreg, vadd ); }
#if USE_AVX2
template<> inline void do_add<true,  __m256i>( __m256i& vreg, __m256i& vadd ) { vreg = _mm256_add_epi32( vreg, vadd ); }
#endif

template<bool clip, typename T> static inline void do_clip( T& vreg, T& vbdmin, T& vbdmax );
template<> inline void do_clip<false, __m128i>( __m128i&, __m128i&, __m128i& ) { }
template<> inline void do_clip<true,  __m128i>( __m128i& vreg, __m128i& vbdmin, __m128i& vbdmax ) { vreg = _mm_min_epi16   ( vbdmax, _mm_max_epi16   ( vbdmin, vreg ) ); }


template<X86_VEXT vext, int W, bool doAdd, bool doMult, bool doShift, bool shiftR, bool clip>
void linTf_SSE( const int16_t* src, ptrdiff_t srcStride, int16_t* dst, ptrdiff_t dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng )
{
  if( vext >= AVX2 && ( width & 7 ) == 0 && W == 8 )
  {
#if USE_AVX2
    __m128i xbdmin   = _mm_set1_epi16( clpRng.min() );
    __m128i xbdmax   = _mm_set1_epi16( clpRng.max() );
    __m256i voffset  = _mm256_set1_epi32( offset );
    __m256i vscale   = _mm256_set1_epi32( scale );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i val;
        val = _mm256_cvtepi16_epi32       (  _mm_lddqu_si128( ( const __m128i * )&src[col] ) );
        do_mult<doMult, __m256i>          ( val, vscale );
        do_shift<doShift, shiftR, __m256i>( val, shift );
        do_add<doAdd, __m256i>            ( val, voffset );
        __m128i
        xal = _mm256_cvtepi32_epi16x      ( val );
        do_clip<clip, __m128i>            ( xal, xbdmin, xbdmax );

        _mm_storeu_si128                  ( ( __m128i * )&dst[col], xal );
      }

      src += srcStride;
      dst += dstStride;
    }
#endif
  }
  else
  {
    __m128i vzero   = _mm_setzero_si128();
    __m128i vbdmin  = _mm_set1_epi16   ( clpRng.min() );
    __m128i vbdmax  = _mm_set1_epi16   ( clpRng.max() );
    __m128i voffset = _mm_set1_epi32   ( offset );
    __m128i vscale  = _mm_set1_epi32   ( scale );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i val;
        val = _mm_loadu_si64             ( ( const __m128i * )&src[col] );
        val = _mm_cvtepi16_epi32          ( val );
        do_mult<doMult, __m128i>          ( val, vscale );
        do_shift<doShift, shiftR, __m128i>( val, shift );
        do_add<doAdd, __m128i>            ( val, voffset );
        val = _mm_packs_epi32             ( val, vzero );
        do_clip<clip, __m128i>            ( val, vbdmin, vbdmax );

        _mm_storeu_si64                  ( ( __m128i * )&dst[col], val );
      }

      src += srcStride;
      dst += dstStride;
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext, int W>
void linTf_SSE_entry( const int16_t* src, ptrdiff_t srcStride, int16_t* dst, ptrdiff_t dstStride, int width, int height, int scale, int shift, int offset, const ClpRng& clpRng, bool clip )
{
  linTf_SSE<vext, W, true,  true,  true,  true,  true >( src, srcStride, dst, dstStride, width, height, scale,  shift, offset, clpRng );
}

template<X86_VEXT vext, int W>
void transposePel_SSE( const Pel* src, ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride )
{
  if( W == 4 )
  {
    __m128i va, vb, vc, vd;

    va = _mm_loadu_si64( ( const __m128i* ) src ); src += srcStride;
    vb = _mm_loadu_si64( ( const __m128i* ) src ); src += srcStride;
    vc = _mm_loadu_si64( ( const __m128i* ) src ); src += srcStride;
    vd = _mm_loadu_si64( ( const __m128i* ) src );

    __m128i va01b01 = _mm_unpacklo_epi16( va,      vb );
    __m128i va23b23 = _mm_unpackhi_epi64( va01b01, vb );
    __m128i vc01d01 = _mm_unpacklo_epi16( vc,      vd );
    __m128i vc23d23 = _mm_unpackhi_epi64( vc01d01, vd );

    va = _mm_unpacklo_epi32( va01b01, vc01d01 );
    vb = _mm_unpackhi_epi64( va,      va );
    vc = _mm_unpacklo_epi32( va23b23, vc23d23 );
    vd = _mm_unpackhi_epi64( vc,      vc );

    _mm_storeu_si64( ( __m128i* ) dst, va ); dst += dstStride;
    _mm_storeu_si64( ( __m128i* ) dst, vb ); dst += dstStride;
    _mm_storeu_si64( ( __m128i* ) dst, vc ); dst += dstStride;
    _mm_storeu_si64( ( __m128i* ) dst, vd );
  }
  else if( W == 8 )
  {
    __m128i va, vb, vc, vd, ve, vf, vg, vh;

    va = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vb = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vc = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vd = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    ve = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vf = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vg = _mm_loadu_si128( ( const __m128i* ) src ); src += srcStride;
    vh = _mm_loadu_si128( ( const __m128i* ) src );

    __m128i va01b01 = _mm_unpacklo_epi16( va, vb );
    __m128i va23b23 = _mm_unpackhi_epi16( va, vb );
    __m128i vc01d01 = _mm_unpacklo_epi16( vc, vd );
    __m128i vc23d23 = _mm_unpackhi_epi16( vc, vd );
    __m128i ve01f01 = _mm_unpacklo_epi16( ve, vf );
    __m128i ve23f23 = _mm_unpackhi_epi16( ve, vf );
    __m128i vg01h01 = _mm_unpacklo_epi16( vg, vh );
    __m128i vg23h23 = _mm_unpackhi_epi16( vg, vh );

    va = _mm_unpacklo_epi32( va01b01, vc01d01 );
    vb = _mm_unpackhi_epi32( va01b01, vc01d01 );
    vc = _mm_unpacklo_epi32( va23b23, vc23d23 );
    vd = _mm_unpackhi_epi32( va23b23, vc23d23 );
    ve = _mm_unpacklo_epi32( ve01f01, vg01h01 );
    vf = _mm_unpackhi_epi32( ve01f01, vg01h01 );
    vg = _mm_unpacklo_epi32( ve23f23, vg23h23 );
    vh = _mm_unpackhi_epi32( ve23f23, vg23h23 );

    va01b01 = _mm_unpacklo_epi64( va, ve );
    va23b23 = _mm_unpackhi_epi64( va, ve );
    vc01d01 = _mm_unpacklo_epi64( vb, vf );
    vc23d23 = _mm_unpackhi_epi64( vb, vf );
    ve01f01 = _mm_unpacklo_epi64( vc, vg );
    ve23f23 = _mm_unpackhi_epi64( vc, vg );
    vg01h01 = _mm_unpacklo_epi64( vd, vh );
    vg23h23 = _mm_unpackhi_epi64( vd, vh );

    _mm_storeu_si128( ( __m128i* ) dst, va01b01 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, va23b23 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, vc01d01 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, vc23d23 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, ve01f01 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, ve23f23 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, vg01h01 ); dst += dstStride;
    _mm_storeu_si128( ( __m128i* ) dst, vg23h23 );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void copyBuffer_SSE( const char *src, ptrdiff_t srcStride, char *dst, ptrdiff_t dstStride, int width, int height )
{
  _mm_prefetch( (const char *) ( src             ), _MM_HINT_T0 );
  _mm_prefetch( (const char *) ( src + srcStride ), _MM_HINT_T0 );

  if( width == srcStride && width == dstStride )
  {
    memcpy( dst, src, width * height );
    return;
  }

  while( height-- )
  {
    const char* nextSrcLine = src + srcStride;
          char* nextDstLine = dst + dstStride;

    _mm_prefetch( nextSrcLine, _MM_HINT_T0 );

    memcpy( dst, src, width );

    src = nextSrcLine;
    dst = nextDstLine;
  }
}

template<X86_VEXT vext>
void applyLut_SIMD( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const Pel* lut )
{
  _mm_prefetch( ( const char* ) &ptr[0 * ptrStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &ptr[1 * ptrStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &ptr[0 * ptrStride + (width >> 1)], _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &ptr[1 * ptrStride + (width >> 1)], _MM_HINT_T0 );

#if USE_AVX2
  if( ( width & 15 ) == 0 && ( height & 1 ) == 0 )
  {
    const __m256i vLutShuf = _mm256_setr_epi8( 0, 1, 4, 5, 8, 9, 12, 13, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 4, 5, 8, 9, 12, 13, -1, -1, -1, -1, -1, -1, -1, -1 );

    for( int y = 0; y < height; y += 2 )
    {
      _mm_prefetch( ( const char* ) &ptr[2 * ptrStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) &ptr[3 * ptrStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) &ptr[2 * ptrStride + ( width >> 1 )], _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) &ptr[3 * ptrStride + ( width >> 1 )], _MM_HINT_T0 );

      for( int x = 0; x < width; x += 16 )
      {
        __m256i vin16    = _mm256_load_si256        ( ( const __m256i * ) &ptr[x] );
        __m256i vin16x   = _mm256_load_si256        ( ( const __m256i * ) &ptr[x + ptrStride] );

        __m256i vin32_1  = _mm256_unpacklo_epi16    ( vin16,  _mm256_setzero_si256() );
        __m256i vin32_2  = _mm256_unpackhi_epi16    ( vin16,  _mm256_setzero_si256() );
        __m256i vin32_1x = _mm256_unpacklo_epi16    ( vin16x, _mm256_setzero_si256() );
        __m256i vin32_2x = _mm256_unpackhi_epi16    ( vin16x, _mm256_setzero_si256() );

        __m256i vout32_1 = _mm256_i32gather_epi32   ( ( const int * ) lut, vin32_1,  2 );
        __m256i vout32_2 = _mm256_i32gather_epi32   ( ( const int * ) lut, vin32_2,  2 );
        __m256i vout32_1x= _mm256_i32gather_epi32   ( ( const int * ) lut, vin32_1x, 2 );
        __m256i vout32_2x= _mm256_i32gather_epi32   ( ( const int * ) lut, vin32_2x, 2 );

        vout32_1         = _mm256_shuffle_epi8      ( vout32_1,  vLutShuf );
        vout32_2         = _mm256_shuffle_epi8      ( vout32_2,  vLutShuf );
        vout32_1x        = _mm256_shuffle_epi8      ( vout32_1x, vLutShuf );
        vout32_2x        = _mm256_shuffle_epi8      ( vout32_2x, vLutShuf );

        __m256i vout16   = _mm256_unpacklo_epi64    ( vout32_1,  vout32_2 );
        __m256i vout16x  = _mm256_unpacklo_epi64    ( vout32_1x, vout32_2x );

        _mm256_store_si256( ( __m256i * ) &ptr[x],             vout16 );
        _mm256_store_si256( ( __m256i * ) &ptr[x + ptrStride], vout16x );
      }

      ptr += ( ptrStride << 1 );
    }

    _mm256_zeroupper();
  }
  else
#endif
  {
#define RSP_SGNL_OP( ADDR ) ptr[ADDR] = lut[ptr[ADDR]]
#define RSP_SGNL_INC        ptr      += ptrStride;

    SIZE_AWARE_PER_EL_OP( RSP_SGNL_OP, RSP_SGNL_INC )

#undef RSP_SGNL_OP
#undef RSP_SGNL_INC
  }
}

template<X86_VEXT vext>
void rspBcwCore_SIMD( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const int bd, const int minBin, const int maxBin, const Pel* LmcsPivot, const Pel* InvScCoeff, const Pel* InputPivot )
{
  const int effMaxBin = maxBin < PIC_CODE_CW_BINS - 1 ? maxBin + 1 : maxBin;

  _mm_prefetch( ( const char* ) ( ptr + 0 * ptrStride ), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) ( ptr + 1 * ptrStride ), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) ( ptr + 2 * ptrStride ), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) ( ptr + 3 * ptrStride ), _MM_HINT_T0 );

#if USE_AVX2
  if( ( width & 15 ) == 0 && vext >= AVX2 )
  {
    __m128i xtmp1, xtmp2, xtmp3, xtmp4;
    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &InputPivot[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &InputPivot[8] );
    xtmp3 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp4 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp1 = _mm_unpacklo_epi64( xtmp3, xtmp4 );
    xtmp2 = _mm_unpackhi_epi64( xtmp3, xtmp4 );

    const __m256i mInputPivotLo = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp1 ), xtmp1, 1 );
    const __m256i mInputPivotHi = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp2 ), xtmp2, 1 );

    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &InvScCoeff[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &InvScCoeff[8] );
    xtmp3 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp4 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp1 = _mm_unpacklo_epi64( xtmp3, xtmp4 );
    xtmp2 = _mm_unpackhi_epi64( xtmp3, xtmp4 );

    const __m256i mScaleCoeffLo = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp1 ), xtmp1, 1 );
    const __m256i mScaleCoeffHi = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp2 ), xtmp2, 1 );

    const __m256i mMin = _mm256_setzero_si256();
    const __m256i mMax = _mm256_set1_epi16( ( 1 << bd ) - 1 );

    for( int y = 0; y < height; y += 4 )
    {
      _mm_prefetch( ( const char* ) ( ptr + 4 * ptrStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( ptr + 5 * ptrStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( ptr + 6 * ptrStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( ptr + 7 * ptrStride ), _MM_HINT_T0 );

      for( int x = 0; x < width; x += 16 )
      {
        const __m256i xsrc0 = _mm256_load_si256( ( const __m256i* ) ( ptr + x + 0 * ptrStride ) );
        const __m256i xsrc1 = _mm256_load_si256( ( const __m256i* ) ( ptr + x + 1 * ptrStride ) );
        const __m256i xsrc2 = _mm256_load_si256( ( const __m256i* ) ( ptr + x + 2 * ptrStride ) );
        const __m256i xsrc3 = _mm256_load_si256( ( const __m256i* ) ( ptr + x + 3 * ptrStride ) );

        __m256i diff0 = _mm256_min_epi16( xsrc0, xsrc1 );
        __m256i diff2 = _mm256_min_epi16( xsrc2, xsrc3 );
        __m256i diff1, diff3;

        diff0   = _mm256_min_epi16( diff0, diff2 );
        __m128i
        diffx   = _mm_minpos_epu16( _mm_min_epi16( _mm256_castsi256_si128  ( diff0 ),
                                                   _mm256_extracti128_si256( diff0, 1 ) ) );
        int min = _mm_extract_epi16( diffx, 0 );

        int i = minBin;
        switch( effMaxBin - minBin )
        {
        default:
        case 15: if( min < LmcsPivot[++i] ) { break; };
        case 14: if( min < LmcsPivot[++i] ) { break; };
        case 13: if( min < LmcsPivot[++i] ) { break; };
        case 12: if( min < LmcsPivot[++i] ) { break; };
        case 11: if( min < LmcsPivot[++i] ) { break; };
        case 10: if( min < LmcsPivot[++i] ) { break; };
        case  9: if( min < LmcsPivot[++i] ) { break; };
        case  8: if( min < LmcsPivot[++i] ) { break; };
        case  7: if( min < LmcsPivot[++i] ) { break; };
        case  6: if( min < LmcsPivot[++i] ) { break; };
        case  5: if( min < LmcsPivot[++i] ) { break; };
        case  4: if( min < LmcsPivot[++i] ) { break; };
        case  3: if( min < LmcsPivot[++i] ) { break; };
        case  2: if( min < LmcsPivot[++i] ) { break; };
        case  1: if( min < LmcsPivot[++i] ) { break; };
        case  0: if( min < LmcsPivot[++i] ) { break; };
        }

        --i;

        __m256i xidx0 = _mm256_set1_epi16( i );
        __m256i xidx1 = xidx0;
        __m256i xidx2 = xidx0;
        __m256i xidx3 = xidx0;

        __m256i xlmcs = _mm256_set1_epi16( LmcsPivot[i] );

        diff0 = _mm256_sub_epi16( xsrc0, xlmcs );
        diff1 = _mm256_sub_epi16( xsrc1, xlmcs );
        diff2 = _mm256_sub_epi16( xsrc2, xlmcs );
        diff3 = _mm256_sub_epi16( xsrc3, xlmcs );

        for( ++i; i <= effMaxBin; ++i )
        {
          __m256i
          xlmcs         = _mm256_set1_epi16( LmcsPivot[i] );

          __m256i currd = _mm256_sub_epi16( xsrc0, xlmcs );
          diff0         = _mm256_min_epu16( diff0, currd );
          __m256i chnd0 = _mm256_cmpeq_epi16( currd, diff0 );

          currd         = _mm256_sub_epi16( xsrc1, xlmcs );
          diff1         = _mm256_min_epu16( diff1, currd );
          __m256i chnd1 = _mm256_cmpeq_epi16( currd, diff1 );

          currd         = _mm256_sub_epi16( xsrc2, xlmcs );
          diff2         = _mm256_min_epu16( diff2, currd );
          __m256i chnd2 = _mm256_cmpeq_epi16( currd, diff2 );

          currd         = _mm256_sub_epi16( xsrc3, xlmcs );
          diff3         = _mm256_min_epu16( diff3, currd );
          __m256i chnd3 = _mm256_cmpeq_epi16( currd, diff3 );

          xidx0         = _mm256_sub_epi16( xidx0, chnd0 );
          xidx1         = _mm256_sub_epi16( xidx1, chnd1 );
          xidx2         = _mm256_sub_epi16( xidx2, chnd2 );
          xidx3         = _mm256_sub_epi16( xidx3, chnd3 );

          chnd0         = _mm256_or_si256( chnd0, chnd1 );
          chnd2         = _mm256_or_si256( chnd2, chnd3 );
          chnd0         = _mm256_or_si256( chnd0, chnd2 );

          if( _mm256_movemask_epi8( chnd0 ) == 0 ) break;
        }

        xidx0 = _mm256_packs_epi16( xidx0, _mm256_set1_epi8( -1 ) );
        xidx1 = _mm256_packs_epi16( xidx1, _mm256_set1_epi8( -1 ) );
        xidx2 = _mm256_packs_epi16( xidx2, _mm256_set1_epi8( -1 ) );
        xidx3 = _mm256_packs_epi16( xidx3, _mm256_set1_epi8( -1 ) );

        __m256i xinp = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mInputPivotLo, xidx0 ), _mm256_shuffle_epi8( mInputPivotHi, xidx0 ) );
        __m256i xscl = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mScaleCoeffLo, xidx0 ), _mm256_shuffle_epi8( mScaleCoeffHi, xidx0 ) );

        __m256i
        xtmp1 = _mm256_slli_epi16( diff0, 4 );
        xtmp1 = _mm256_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm256_add_epi16( xinp, xtmp1 );

        xtmp1 = _mm256_min_epi16( xtmp1, mMax );
        xtmp1 = _mm256_max_epi16( xtmp1, mMin );

        _mm256_store_si256( ( __m256i * ) &ptr[x], xtmp1 );

        xinp = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mInputPivotLo, xidx1 ), _mm256_shuffle_epi8( mInputPivotHi, xidx1 ) );
        xscl = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mScaleCoeffLo, xidx1 ), _mm256_shuffle_epi8( mScaleCoeffHi, xidx1 ) );

        xtmp1 = _mm256_slli_epi16( diff1, 4 );
        xtmp1 = _mm256_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm256_add_epi16( xinp, xtmp1 );

        xtmp1 = _mm256_min_epi16( xtmp1, mMax );
        xtmp1 = _mm256_max_epi16( xtmp1, mMin );

        _mm256_store_si256( (__m256i*) & ptr[x+ptrStride], xtmp1 );

        xinp = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mInputPivotLo, xidx2 ), _mm256_shuffle_epi8( mInputPivotHi, xidx2 ) );
        xscl = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mScaleCoeffLo, xidx2 ), _mm256_shuffle_epi8( mScaleCoeffHi, xidx2 ) );

        xtmp1 = _mm256_slli_epi16( diff2, 4 );
        xtmp1 = _mm256_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm256_add_epi16( xinp, xtmp1 );

        xtmp1 = _mm256_min_epi16( xtmp1, mMax );
        xtmp1 = _mm256_max_epi16( xtmp1, mMin );

        _mm256_store_si256( (__m256i*) & ptr[x+2*ptrStride], xtmp1 );

        xinp = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mInputPivotLo, xidx3 ), _mm256_shuffle_epi8( mInputPivotHi, xidx3 ) );
        xscl = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mScaleCoeffLo, xidx3 ), _mm256_shuffle_epi8( mScaleCoeffHi, xidx3 ) );

        xtmp1 = _mm256_slli_epi16( diff3, 4 );
        xtmp1 = _mm256_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm256_add_epi16( xinp, xtmp1 );

        xtmp1 = _mm256_min_epi16( xtmp1, mMax );
        xtmp1 = _mm256_max_epi16( xtmp1, mMin );

        _mm256_store_si256( (__m256i*) & ptr[x+3*ptrStride], xtmp1 );
      }

      ptr += ( ptrStride << 2 );
    }

    _mm256_zeroupper();
  }
  else
#endif
  if( ( width & 7 ) == 0 )
  {
    __m128i xtmp1, xtmp2;

    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &InputPivot[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &InputPivot[8] );
    xtmp1 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp2 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );

    const __m128i mInputPivotLo = _mm_unpacklo_epi64( xtmp1, xtmp2 );
    const __m128i mInputPivotHi = _mm_unpackhi_epi64( xtmp1, xtmp2 );

    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &InvScCoeff[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &InvScCoeff[8] );
    xtmp1 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp2 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );

    const __m128i mScaleCoeffLo = _mm_unpacklo_epi64( xtmp1, xtmp2 );
    const __m128i mScaleCoeffHi = _mm_unpackhi_epi64( xtmp1, xtmp2 );

    const __m128i mMin    = _mm_setzero_si128();
    const __m128i mMax    = _mm_set1_epi16( ( 1 << bd ) - 1 );

    for( int y = 0; y < height; y += 4 )
    {
      _mm_prefetch( ( const char* ) ( ptr + 4 * ptrStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( ptr + 5 * ptrStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( ptr + 6 * ptrStride ), _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) ( ptr + 7 * ptrStride ), _MM_HINT_T0 );

      for( int x = 0; x < width; x += 8 )
      {
        const __m128i xsrc0 = _mm_load_si128( ( const __m128i* ) ( ptr + x + 0 * ptrStride ) );
        const __m128i xsrc1 = _mm_load_si128( ( const __m128i* ) ( ptr + x + 1 * ptrStride ) );
        const __m128i xsrc2 = _mm_load_si128( ( const __m128i* ) ( ptr + x + 2 * ptrStride ) );
        const __m128i xsrc3 = _mm_load_si128( ( const __m128i* ) ( ptr + x + 3 * ptrStride ) );

        __m128i diff0, diff1, diff2, diff3;
        diff0   = _mm_minpos_epu16( _mm_min_epi16( _mm_min_epi16( xsrc0, xsrc1 ), _mm_min_epi16( xsrc2, xsrc3 ) ) );
        int min = _mm_extract_epi16( diff0, 0 );

        int i = minBin;
        switch( effMaxBin - minBin )
        {
        default:
        case 15: if( min < LmcsPivot[++i] ) { break; };
        case 14: if( min < LmcsPivot[++i] ) { break; };
        case 13: if( min < LmcsPivot[++i] ) { break; };
        case 12: if( min < LmcsPivot[++i] ) { break; };
        case 11: if( min < LmcsPivot[++i] ) { break; };
        case 10: if( min < LmcsPivot[++i] ) { break; };
        case  9: if( min < LmcsPivot[++i] ) { break; };
        case  8: if( min < LmcsPivot[++i] ) { break; };
        case  7: if( min < LmcsPivot[++i] ) { break; };
        case  6: if( min < LmcsPivot[++i] ) { break; };
        case  5: if( min < LmcsPivot[++i] ) { break; };
        case  4: if( min < LmcsPivot[++i] ) { break; };
        case  3: if( min < LmcsPivot[++i] ) { break; };
        case  2: if( min < LmcsPivot[++i] ) { break; };
        case  1: if( min < LmcsPivot[++i] ) { break; };
        case  0: if( min < LmcsPivot[++i] ) { break; };
        }

        --i;

        __m128i xidx0 = _mm_set1_epi16( i );
        __m128i xidx1 = xidx0;
        __m128i xidx2 = xidx0;
        __m128i xidx3 = xidx0;

        __m128i xlmcs = _mm_set1_epi16( LmcsPivot[i] );

        diff0 = _mm_sub_epi16( xsrc0, xlmcs );
        diff1 = _mm_sub_epi16( xsrc1, xlmcs );
        diff2 = _mm_sub_epi16( xsrc2, xlmcs );
        diff3 = _mm_sub_epi16( xsrc3, xlmcs );

        for( ++i; i <= effMaxBin; ++i )
        {
          xlmcs         = _mm_set1_epi16( LmcsPivot[i] );

          __m128i currd = _mm_sub_epi16( xsrc0, xlmcs );
          diff0         = _mm_min_epu16( diff0, currd );
          __m128i chnd0 = _mm_cmpeq_epi16( currd, diff0 );

          currd         = _mm_sub_epi16( xsrc1, xlmcs );
          diff1         = _mm_min_epu16( diff1, currd );
          __m128i chnd1 = _mm_cmpeq_epi16( currd, diff1 );

          currd         = _mm_sub_epi16( xsrc2, xlmcs );
          diff2         = _mm_min_epu16( diff2, currd );
          __m128i chnd2 = _mm_cmpeq_epi16( currd, diff2 );

          currd         = _mm_sub_epi16( xsrc3, xlmcs );
          diff3         = _mm_min_epu16( diff3, currd );
          __m128i chnd3 = _mm_cmpeq_epi16( currd, diff3 );

          xidx0         = _mm_sub_epi16( xidx0, chnd0 );
          xidx1         = _mm_sub_epi16( xidx1, chnd1 );
          xidx2         = _mm_sub_epi16( xidx2, chnd2 );
          xidx3         = _mm_sub_epi16( xidx3, chnd3 );

          chnd0         = _mm_or_si128( chnd0, chnd1 );
          chnd2         = _mm_or_si128( chnd2, chnd3 );
          chnd0         = _mm_or_si128( chnd0, chnd2 );

          if( _mm_movemask_epi8( chnd0 ) == 0 ) break;
        }

        xidx0 = _mm_packs_epi16( xidx0, _mm_set1_epi8( -1 ) );
        xidx1 = _mm_packs_epi16( xidx1, _mm_set1_epi8( -1 ) );
        xidx2 = _mm_packs_epi16( xidx2, _mm_set1_epi8( -1 ) );
        xidx3 = _mm_packs_epi16( xidx3, _mm_set1_epi8( -1 ) );

        __m128i xinp = _mm_unpacklo_epi8( _mm_shuffle_epi8( mInputPivotLo, xidx0 ), _mm_shuffle_epi8( mInputPivotHi, xidx0 ) );
        __m128i xscl = _mm_unpacklo_epi8( _mm_shuffle_epi8( mScaleCoeffLo, xidx0 ), _mm_shuffle_epi8( mScaleCoeffHi, xidx0 ) );

        xtmp1 = _mm_slli_epi16( diff0, 4 );
        xtmp1 = _mm_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm_add_epi16( xinp, xtmp1 );

        xtmp1 = _mm_min_epi16( xtmp1, mMax );
        xtmp1 = _mm_max_epi16( xtmp1, mMin );

        _mm_store_si128( ( __m128i * ) &ptr[x], xtmp1 );

        xinp = _mm_unpacklo_epi8( _mm_shuffle_epi8( mInputPivotLo, xidx1 ), _mm_shuffle_epi8( mInputPivotHi, xidx1 ) );
        xscl = _mm_unpacklo_epi8( _mm_shuffle_epi8( mScaleCoeffLo, xidx1 ), _mm_shuffle_epi8( mScaleCoeffHi, xidx1 ) );

        xtmp1 = _mm_slli_epi16( diff1, 4 );
        xtmp1 = _mm_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm_add_epi16( xinp, xtmp1 );

        xtmp1 = _mm_min_epi16( xtmp1, mMax );
        xtmp1 = _mm_max_epi16( xtmp1, mMin );

        _mm_store_si128( (__m128i*) & ptr[x+ptrStride], xtmp1 );

        xinp = _mm_unpacklo_epi8( _mm_shuffle_epi8( mInputPivotLo, xidx2 ), _mm_shuffle_epi8( mInputPivotHi, xidx2 ) );
        xscl = _mm_unpacklo_epi8( _mm_shuffle_epi8( mScaleCoeffLo, xidx2 ), _mm_shuffle_epi8( mScaleCoeffHi, xidx2 ) );

        xtmp1 = _mm_slli_epi16( diff2, 4 );
        xtmp1 = _mm_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm_add_epi16( xinp, xtmp1 );

        xtmp1 = _mm_min_epi16( xtmp1, mMax );
        xtmp1 = _mm_max_epi16( xtmp1, mMin );

        _mm_store_si128( (__m128i*) & ptr[x + 2 * ptrStride], xtmp1 );

        xinp = _mm_unpacklo_epi8( _mm_shuffle_epi8( mInputPivotLo, xidx3 ), _mm_shuffle_epi8( mInputPivotHi, xidx3 ) );
        xscl = _mm_unpacklo_epi8( _mm_shuffle_epi8( mScaleCoeffLo, xidx3 ), _mm_shuffle_epi8( mScaleCoeffHi, xidx3 ) );

        xtmp1 = _mm_slli_epi16( diff3, 4 );
        xtmp1 = _mm_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm_add_epi16( xinp, xtmp1 );

        xtmp1 = _mm_min_epi16( xtmp1, mMax );
        xtmp1 = _mm_max_epi16( xtmp1, mMin );

        _mm_store_si128( (__m128i*) & ptr[x + 3 * ptrStride], xtmp1 );
      }

      ptr += ptrStride << 2;
    }
  }
  else
  {
    THROW_FATAL( "Unsupported size!" );
  }
}

template<X86_VEXT vext>
void rspFwdCore_SIMD( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const int bd, const Pel OrgCW, const Pel* LmcsPivot, const Pel* ScaleCoeff, const Pel* InputPivot )
{
  _mm_prefetch( ( const char* ) (ptr + 0 * ptrStride), _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) (ptr + 1 * ptrStride), _MM_HINT_T0 );

  int shift = getLog2( OrgCW );

#if USE_AVX2
  if( ( width & 15 ) == 0 )
  {
    __m128i xtmp1, xtmp2, xtmp3, xtmp4;
    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &LmcsPivot[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &LmcsPivot[8] );
    xtmp3 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp4 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp1 = _mm_unpacklo_epi64( xtmp3, xtmp4 );
    xtmp2 = _mm_unpackhi_epi64( xtmp3, xtmp4 );

    const __m256i mLmcsPivotLo = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp1 ), xtmp1, 1 );
    const __m256i mLmcsPivotHi = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp2 ), xtmp2, 1 );

    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &InputPivot[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &InputPivot[8] );
    xtmp3 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp4 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp1 = _mm_unpacklo_epi64( xtmp3, xtmp4 );
    xtmp2 = _mm_unpackhi_epi64( xtmp3, xtmp4 );

    const __m256i mInputPivotLo = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp1 ), xtmp1, 1 );
    const __m256i mInputPivotHi = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp2 ), xtmp2, 1 );

    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &ScaleCoeff[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &ScaleCoeff[8] );
    xtmp3 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp4 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp1 = _mm_unpacklo_epi64( xtmp3, xtmp4 );
    xtmp2 = _mm_unpackhi_epi64( xtmp3, xtmp4 );

    const __m256i mScaleCoeffLo = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp1 ), xtmp1, 1 );
    const __m256i mScaleCoeffHi = _mm256_inserti128_si256( _mm256_castsi128_si256( xtmp2 ), xtmp2, 1 );

    const __m256i mMin    = _mm256_setzero_si256();
    const __m256i mMax    = _mm256_set1_epi16( ( 1 << bd ) - 1 );

    //#define RSP_FWD_OP( ADDR ) { idxY = ( ptr[ADDR] >> shift ); ptr[ADDR] = static_cast<Pel>( ClipBD<int>( LmcsPivot[idxY] + ( ( ScaleCoeff[idxY] * ( ptr[ADDR] - InputPivot[idxY] ) + ( 1 << 10 ) ) >> 11 ), bd ) ); }

    while( height-- )
    {
      _mm_prefetch( ( const char* ) ( ptr + ptrStride ), _MM_HINT_T0 );

      for( int x = 0; x < width; x += 16 )
      {
        const __m256i xsrc = _mm256_loadu_si256( ( const __m256i* ) &ptr[x] );
        const __m256i xidx = _mm256_packs_epi16( _mm256_srai_epi16 ( xsrc, shift ), _mm256_set1_epi8( -1 ) );

        const __m256i xinp = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mInputPivotLo, xidx ), _mm256_shuffle_epi8( mInputPivotHi, xidx ) );
        const __m256i xscl = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mScaleCoeffLo, xidx ), _mm256_shuffle_epi8( mScaleCoeffHi, xidx ) );
        const __m256i xlmc = _mm256_unpacklo_epi8( _mm256_shuffle_epi8( mLmcsPivotLo,  xidx ), _mm256_shuffle_epi8( mLmcsPivotHi,  xidx ) );

        __m256i
        vtmp1 = _mm256_slli_epi16( _mm256_subs_epi16( xsrc, xinp ), 4 );
        vtmp1 = _mm256_mulhrs_epi16( vtmp1, xscl );

        vtmp1 = _mm256_add_epi16( xlmc, vtmp1 );

        vtmp1 = _mm256_min_epi16( vtmp1, mMax );
        vtmp1 = _mm256_max_epi16( vtmp1, mMin );

        _mm256_storeu_si256( ( __m256i * ) &ptr[x], vtmp1 );
      }

      ptr += ptrStride;
    }

    _mm256_zeroupper();
  }
  else
#endif
  if( ( width & 7 ) == 0 )
  {
    __m128i xtmp1, xtmp2, xtmp3, xtmp4;
    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &LmcsPivot[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &LmcsPivot[8] );
    xtmp3 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp4 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );

    const __m128i mLmcsPivotLo = _mm_unpacklo_epi64( xtmp3, xtmp4 );
    const __m128i mLmcsPivotHi = _mm_unpackhi_epi64( xtmp3, xtmp4 );

    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &InputPivot[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &InputPivot[8] );
    xtmp3 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp4 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );

    const __m128i mInputPivotLo = _mm_unpacklo_epi64( xtmp3, xtmp4 );
    const __m128i mInputPivotHi = _mm_unpackhi_epi64( xtmp3, xtmp4 );


    xtmp1 = _mm_loadu_si128( ( const __m128i* ) &ScaleCoeff[0] );
    xtmp2 = _mm_loadu_si128( ( const __m128i* ) &ScaleCoeff[8] );
    xtmp3 = _mm_shuffle_epi8( xtmp1, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );
    xtmp4 = _mm_shuffle_epi8( xtmp2, _mm_setr_epi8( 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 ) );

    const __m128i mScaleCoeffLo = _mm_unpacklo_epi64( xtmp3, xtmp4 );
    const __m128i mScaleCoeffHi = _mm_unpackhi_epi64( xtmp3, xtmp4 );

    const __m128i mMin    = _mm_setzero_si128();
    const __m128i mMax    = _mm_set1_epi16( ( 1 << bd ) - 1 );

    while( height-- )
    {
      _mm_prefetch( ( const char* ) ( ptr + ptrStride ), _MM_HINT_T0 );

      for( int x = 0; x < width; x += 8 )
      {
        const __m128i xsrc = _mm_loadu_si128( ( const __m128i* ) &ptr[x] );
        const __m128i xidx = _mm_packs_epi16( _mm_srai_epi16 ( xsrc, shift ), _mm_set1_epi8( -1 ) );

        const __m128i xlmc = _mm_unpacklo_epi8( _mm_shuffle_epi8( mLmcsPivotLo,  xidx ), _mm_shuffle_epi8( mLmcsPivotHi,  xidx ) );
        const __m128i xinp = _mm_unpacklo_epi8( _mm_shuffle_epi8( mInputPivotLo, xidx ), _mm_shuffle_epi8( mInputPivotHi, xidx ) );
        const __m128i xscl = _mm_unpacklo_epi8( _mm_shuffle_epi8( mScaleCoeffLo, xidx ), _mm_shuffle_epi8( mScaleCoeffHi, xidx ) );

        xtmp1 = _mm_slli_epi16( _mm_subs_epi16( xsrc, xinp ), 4 );
        xtmp1 = _mm_mulhrs_epi16( xtmp1, xscl );

        xtmp1 = _mm_add_epi16( xlmc, xtmp1 );

        xtmp1 = _mm_min_epi16( xtmp1, mMax );
        xtmp1 = _mm_max_epi16( xtmp1, mMin );

        _mm_storeu_si128( ( __m128i * ) &ptr[x], xtmp1 );
      }

      ptr += ptrStride;
    }
  }
  else
  {
    int idxY;

    //    const auto rsp_sgnl_op  = [=, &dst]( int ADDR ){ idxY = ( dst[ADDR] >> shift ); dst[ADDR] = static_cast<Pel>( ClipBD<int>( LmcsPivot[idxY] + ( ( ScaleCoeff[idxY] * ( dst[ADDR] - InputPivot[idxY] ) + ( 1 << 10 ) ) >> 11 ), bd ) ); };
    //    const auto rsp_sgnl_inc = [=, &dst]            { dst += stride; };

    //    size_aware_pel_op( rsp_sgnl_op, rsp_sgnl_inc, width, height );

#define RSP_FWD_OP( ADDR ) { idxY = ( ptr[ADDR] >> shift ); ptr[ADDR] = static_cast<Pel>( ClipBD<int>( LmcsPivot[idxY] + ( ( ScaleCoeff[idxY] * ( ptr[ADDR] - InputPivot[idxY] ) + ( 1 << 10 ) ) >> 11 ), bd ) ); }
#define RSP_FWD_INC        ptr      += ptrStride;

    SIZE_AWARE_PER_EL_OP( RSP_FWD_OP, RSP_FWD_INC )

#undef RSP_FWD_OP
#undef RSP_FWD_INC
  }
}

#if INTPTR_MAX == INT64_MAX
template<X86_VEXT vext>
void fillN_CU_SIMD( CodingUnit** ptr, ptrdiff_t ptrStride, int width, int height, CodingUnit* cuPtr )
{
  static_assert( sizeof( cuPtr ) == 8, "Only supported for 64bit systems!" );
  if( ( width & 3 ) == 0 )
  {
#if USE_AVX2
    __m256i vval = _mm256_set1_epi64x( ( int64_t ) cuPtr );

    while( height-- )
    {
      for( int x = 0; x < width; x += 4 ) _mm256_storeu_si256( ( __m256i* ) &ptr[x], vval );

      ptr += ptrStride;
    }
#else
    __m128i vval = _mm_set1_epi64x( ( int64_t ) cuPtr );

    while( height-- )
    {
      for( int x = 0; x < width; x += 4 )
      {
        _mm_storeu_si128( ( __m128i* ) &ptr[x + 0], vval );
        _mm_storeu_si128( ( __m128i* ) &ptr[x + 2], vval );
      }

      ptr += ptrStride;
    }
#endif
  }
  else if( ( width & 1 ) == 0 )
  {
    __m128i vval = _mm_set1_epi64x( ( int64_t ) cuPtr );

    while( height-- )
    {
      for( int x = 0; x < width; x += 2 ) _mm_storeu_si128( ( __m128i* ) &ptr[x], vval );

      ptr += ptrStride;
    }
  }
  else
  {
    while( height-- )
    {
      *ptr = cuPtr; ptr += ptrStride;
    }
  }
}
#elif INTPTR_MAX == INT32_MAX
template<X86_VEXT vext>
void fillN_CU_SIMD( CodingUnit** ptr, ptrdiff_t ptrStride, int width, int height, CodingUnit* cuPtr )
{
  static_assert( sizeof( cuPtr ) == 4, "Only supported for 32bit systems!" );
  if( ( width & 7 ) == 0 )
  {
#if USE_AVX2
    __m256i vval = _mm256_set1_epi32( ( int32_t ) cuPtr );

    while( height-- )
    {
      for( int x = 0; x < width; x += 8 )
      {
        _mm256_storeu_si256( (__m256i*) &ptr[x], vval );
      }

      ptr += ptrStride;
    }
#else
    __m128i vval = _mm_set1_epi32( ( int32_t ) cuPtr );

    while( height-- )
    {
      for( int x = 0; x < width; x += 8 )
      {
        _mm_storeu_si128( ( __m128i* ) &ptr[x + 0], vval );
        _mm_storeu_si128( ( __m128i* ) &ptr[x + 4], vval );
      }

      ptr += ptrStride;
    }
#endif
  }
  else if( ( width & 3 ) == 0 )
  {
    __m128i vval = _mm_set1_epi32( ( int32_t ) cuPtr );

    while( height-- )
    {
      for( int x = 0; x < width; x += 4 )
      {
        _mm_storeu_si128( (__m128i*) &ptr[x], vval );
      }

      ptr += ptrStride;
    }
  }
  else if( ( width & 1 ) == 0 )
  {
    while( height-- )
    {
      ptr[0] = cuPtr;
      ptr[1] = cuPtr;

      ptr += ptrStride;
    }
  }
  else
  {
    while( height-- )
    {
      for( int x = 0; x < width; ++x )
      {
        ptr[x] = cuPtr;
      }
      ptr += ptrStride;
    }
  }
}
#endif  // INTPTR_MAX == INT32_MAX

template<X86_VEXT vext>
void sampleRateConvSIMD_8tap( const std::pair<int, int> scalingRatio,
                              const std::pair<int, int> compScale,
                              const Pel*                orgSrc,
                              const ptrdiff_t           orgStride,
                              const int                 orgWidth,
                              const int                 orgHeight,
                              const int                 beforeScaleLeftOffset,
                              const int                 beforeScaleTopOffset,
                              Pel*                      scaledSrc,
                              const ptrdiff_t           scaledStride,
                              const int                 scaledWidth,
                              const int                 scaledHeight,
                              const int                 afterScaleLeftOffset,
                              const int                 afterScaleTopOffset,
                              const int                 bitDepth )
{
  static constexpr bool useLumaFilter = true;
  static constexpr int horCollocatedPositionFlag = 1;
  static constexpr int verCollocatedPositionFlag = 1;

  const TFilterCoeff* filterHor = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const TFilterCoeff* filterVer = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];

  const int numFracPositions  = useLumaFilter ? 15 : 31;
  const int numFracShift      = useLumaFilter ? 4 : 5;
  const int posShiftX = SCALE_RATIO_BITS - numFracShift + compScale.first;
  const int posShiftY = SCALE_RATIO_BITS - numFracShift + compScale.second;
  int addX = (1 << (posShiftX - 1)) + (beforeScaleLeftOffset << SCALE_RATIO_BITS) + ((int( 1 - horCollocatedPositionFlag ) * 8 * (scalingRatio.first - SCALE_1X.first) + (1 << (2 + compScale.first))) >> (3 + compScale.first));
  int addY = (1 << (posShiftY - 1)) + (beforeScaleTopOffset << SCALE_RATIO_BITS) + ((int( 1 - verCollocatedPositionFlag ) * 8 * (scalingRatio.second - SCALE_1X.second) + (1 << (2 + compScale.second))) >> (3 + compScale.second));

  const int filterLength = useLumaFilter ? NTAPS_LUMA : NTAPS_CHROMA;
  const int log2Norm = 12;

  CHECK( bitDepth > 17, "Overflow may happen!" );
  const int maxVal = (1 << bitDepth) - 1;

  const int tmpStride = ( ( scaledWidth + 3 ) / 4 ) * 4;
  const int tmpHeight = ( ( orgHeight + 3 ) / 4 ) * 4;
  int*      tmpBuf    = new int[tmpStride * tmpHeight];

  for( int j = 0; j < orgHeight; j += 4 )
  {
    const Pel* org0 = orgSrc +                                j * orgStride;
    const Pel* org1 = orgSrc + std::min( j + 1, orgHeight - 1 ) * orgStride;
    const Pel* org2 = orgSrc + std::min( j + 2, orgHeight - 1 ) * orgStride;
    const Pel* org3 = orgSrc + std::min( j + 3, orgHeight - 1 ) * orgStride;

    _mm_prefetch( ( const char* ) (org0 + (orgStride << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (org1 + (orgStride << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (org2 + (orgStride << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (org3 + (orgStride << 2)), _MM_HINT_T0 );

    for( int i = 0; i < scaledWidth; i++ )
    {
      int refPos  = ( ( ( i << compScale.first ) - afterScaleLeftOffset ) * scalingRatio.first + addX ) >> posShiftX;
      int integer = refPos >> numFracShift;
      int frac    = refPos  & numFracPositions;

      const TFilterCoeff* f = filterHor + frac * filterLength;

      __m128i vsrc0, vsrc1, vsrc2, vsrc3;

      if( integer + 0 - ( filterLength / 2 ) + 1 >= 0 && integer + ( NTAPS_LUMA - 1 ) - ( filterLength / 2 ) + 1 < orgWidth )
      {
        int xInt = integer + 0 - ( filterLength / 2 ) + 1;

        vsrc0 = _mm_loadu_si128( (const __m128i*) &org0[xInt] );
        vsrc1 = _mm_loadu_si128( (const __m128i*) &org1[xInt] );
        vsrc2 = _mm_loadu_si128( (const __m128i*) &org2[xInt] );
        vsrc3 = _mm_loadu_si128( (const __m128i*) &org3[xInt] );
      }
      else
      {
        Pel src[4][NTAPS_LUMA];

        for( int k = 0; k < filterLength; k++ )
        {
          int xInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgWidth - 1 );

          src[0][k] = org0[xInt];
          src[1][k] = org1[xInt];
          src[2][k] = org2[xInt];
          src[3][k] = org3[xInt];
        }

        vsrc0 = _mm_loadu_si128( (const __m128i*) &src[0][0] );
        vsrc1 = _mm_loadu_si128( (const __m128i*) &src[1][0] );
        vsrc2 = _mm_loadu_si128( (const __m128i*) &src[2][0] );
        vsrc3 = _mm_loadu_si128( (const __m128i*) &src[3][0] );
      }

      __m128i vflt  = _mm_loadu_si128( (const __m128i*)  f );

      __m128i vres0 = _mm_madd_epi16( vsrc0, vflt );
      __m128i vres1 = _mm_madd_epi16( vsrc1, vflt );
      __m128i vres2 = _mm_madd_epi16( vsrc2, vflt );
      __m128i vres3 = _mm_madd_epi16( vsrc3, vflt );

      vres0 = _mm_hadd_epi32( vres0, vres1 );
      vres2 = _mm_hadd_epi32( vres2, vres3 );

      vres0 = _mm_hadd_epi32( vres0, vres2 );

      int* tmp = tmpBuf + i * tmpHeight + j;

      _mm_storeu_si128( (__m128i*) tmp, vres0 );
    }
  }

  __m128i vzero = _mm_setzero_si128();
  __m128i vnorm = _mm_set1_epi32( ( 1 << ( log2Norm - 1 ) ) );

  for( int i = 0; i < scaledWidth; i += 4 )
  {
    Pel* dst = scaledSrc;

    int* tmp0 =                       tmpBuf + i       * tmpHeight;
    int* tmp1 = i + 1 < scaledWidth ? tmpBuf + (i + 1) * tmpHeight : tmp0;
    int* tmp2 = i + 2 < scaledWidth ? tmpBuf + (i + 2) * tmpHeight : tmp0;
    int* tmp3 = i + 3 < scaledWidth ? tmpBuf + (i + 3) * tmpHeight : tmp0;

    _mm_prefetch( ( const char* ) (tmp0 + (tmpHeight << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (tmp1 + (tmpHeight << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (tmp2 + (tmpHeight << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (tmp3 + (tmpHeight << 2)), _MM_HINT_T0 );

    for( int j = 0; j < scaledHeight; j++ )
    {
      const int refPos      = ( ( ( j << compScale.second ) - afterScaleTopOffset ) * scalingRatio.second + addY ) >> posShiftY;
      const int integer     = refPos >> numFracShift;
      const int frac        = refPos & numFracPositions;
      const TFilterCoeff* f = filterVer + frac * filterLength;
      __m128i vres0, vres1, vres2, vres3;

#if USE_AVX2
      if( vext >= AVX2 )
      {
        __m256i vflt = _mm256_cvtepi16_epi32( _mm_loadu_si128( (const __m128i*)  f ) );

        __m256i vsrc0, vsrc1, vsrc2, vsrc3;

        if( integer + 0 - (filterLength / 2) + 1 >= 0 && integer + (NTAPS_LUMA - 1) - (filterLength / 2) + 1 < orgHeight )
        {
          int yInt = integer + 0 - (filterLength / 2) + 1;

          vsrc0 = _mm256_loadu_si256( (const __m256i*) &tmp0[yInt] );
          vsrc1 = _mm256_loadu_si256( (const __m256i*) &tmp1[yInt] );
          vsrc2 = _mm256_loadu_si256( (const __m256i*) &tmp2[yInt] );
          vsrc3 = _mm256_loadu_si256( (const __m256i*) &tmp3[yInt] );
        }
        else
        {
          int src[4][NTAPS_LUMA];

          for( int k = 0; k < filterLength; k++ )
          {
            int yInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgHeight - 1 );
            src[0][k] = tmp0[yInt];
            src[1][k] = tmp1[yInt];
            src[2][k] = tmp2[yInt];
            src[3][k] = tmp3[yInt];
          }

          vsrc0 = _mm256_loadu_si256( (const __m256i*) &src[0][0] );
          vsrc1 = _mm256_loadu_si256( (const __m256i*) &src[1][0] );
          vsrc2 = _mm256_loadu_si256( (const __m256i*) &src[2][0] );
          vsrc3 = _mm256_loadu_si256( (const __m256i*) &src[3][0] );
        }

        __m256i xres0 = _mm256_mullo_epi32( vsrc0, vflt );
        __m256i xres1 = _mm256_mullo_epi32( vsrc1, vflt );
        __m256i xres2 = _mm256_mullo_epi32( vsrc2, vflt );
        __m256i xres3 = _mm256_mullo_epi32( vsrc3, vflt );

        xres0 = _mm256_hadd_epi32( xres0, xres1 );
        xres2 = _mm256_hadd_epi32( xres2, xres3 );

        xres0 = _mm256_hadd_epi32( xres0, xres2 );

        vres0 = _mm_add_epi32( _mm256_castsi256_si128( xres0 ), _mm256_extracti128_si256( xres0, 1 ) );
      }
      else
#endif
      {
        __m128i vflt[2];
        vflt[1] = _mm_loadu_si128( (const __m128i*)  f );
        vflt[0] = _mm_cvtepi16_epi32( vflt[1] );
        vflt[1] = _mm_cvtepi16_epi32( _mm_unpackhi_epi64( vflt[1], _mm_setzero_si128() ) );

        __m128i vsrc0[2], vsrc1[2], vsrc2[2], vsrc3[2];

        if( integer + 0 - ( filterLength / 2 ) + 1 >= 0 && integer + ( NTAPS_LUMA - 1 ) - ( filterLength / 2 ) + 1 < orgHeight )
        {
          int yInt = integer + 0 - ( filterLength / 2 ) + 1;

          vsrc0[0] = _mm_loadu_si128( (const __m128i*) &tmp0[yInt]);
          vsrc0[1] = _mm_loadu_si128( (const __m128i*) &tmp0[yInt + 4]);

          vsrc1[0] = _mm_loadu_si128( (const __m128i*) &tmp1[yInt]);
          vsrc1[1] = _mm_loadu_si128( (const __m128i*) &tmp1[yInt + 4]);

          vsrc2[0] = _mm_loadu_si128( (const __m128i*) &tmp2[yInt]);
          vsrc2[1] = _mm_loadu_si128( (const __m128i*) &tmp2[yInt + 4]);

          vsrc3[0] = _mm_loadu_si128( (const __m128i*) &tmp3[yInt]);
          vsrc3[1] = _mm_loadu_si128( (const __m128i*) &tmp3[yInt + 4]);
        }
        else
        {
          int src[4][NTAPS_LUMA];

          for( int k = 0; k < filterLength; k++ )
          {
            int yInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgHeight - 1 );
            src[0][k] = tmp0[yInt];
            src[1][k] = tmp1[yInt];
            src[2][k] = tmp2[yInt];
            src[3][k] = tmp3[yInt];
          }

          vsrc0[0] = _mm_loadu_si128( (const __m128i*) &src[0][0] );
          vsrc0[1] = _mm_loadu_si128( (const __m128i*) &src[0][4] );

          vsrc1[0] = _mm_loadu_si128( (const __m128i*) &src[1][0] );
          vsrc1[1] = _mm_loadu_si128( (const __m128i*) &src[1][4] );

          vsrc2[0] = _mm_loadu_si128( (const __m128i*) &src[2][0] );
          vsrc2[1] = _mm_loadu_si128( (const __m128i*) &src[2][4] );

          vsrc3[0] = _mm_loadu_si128( (const __m128i*) &src[3][0] );
          vsrc3[1] = _mm_loadu_si128( (const __m128i*) &src[3][4] );
        }

        vres0 = _mm_add_epi32( _mm_mullo_epi32( vsrc0[0], vflt[0] ), _mm_mullo_epi32( vsrc0[1], vflt[1] ) );
        vres1 = _mm_add_epi32( _mm_mullo_epi32( vsrc1[0], vflt[0] ), _mm_mullo_epi32( vsrc1[1], vflt[1] ) );
        vres2 = _mm_add_epi32( _mm_mullo_epi32( vsrc2[0], vflt[0] ), _mm_mullo_epi32( vsrc2[1], vflt[1] ) );
        vres3 = _mm_add_epi32( _mm_mullo_epi32( vsrc3[0], vflt[0] ), _mm_mullo_epi32( vsrc3[1], vflt[1] ) );

        vres0 = _mm_hadd_epi32( vres0, vres1 );
        vres2 = _mm_hadd_epi32( vres2, vres3 );

        vres0 = _mm_hadd_epi32( vres0, vres2 );
      }

      vres0 = _mm_add_epi32( vres0, vnorm );
      vres0 = _mm_srai_epi32( vres0, log2Norm );
      vres0 = _mm_max_epi32( _mm_min_epi32( _mm_set1_epi32( maxVal ), vres0 ), vzero );

      vres0 = _mm_packs_epi32( vres0, _mm_setzero_si128() );

      if( i + 3 < scaledWidth )
      {
        _mm_storeu_si64( (__m128i*) &dst[i], vres0 );
      }
      else if( i + 2 < scaledWidth )
      {
        _mm_storeu_si32( (__m128i*) &dst[i], vres0 );
        dst[i + 2] = _mm_extract_epi16( vres0, 2 );
      }
      else if( i + 1 < scaledWidth )
      {
        _mm_storeu_si32( (__m128i*) &dst[i], vres0 );
      }
      else
      {
        dst[i] = _mm_extract_epi16( vres0, 0 );
      }

      dst += scaledStride;
    }
  }

  delete[] tmpBuf;
}

template<X86_VEXT vext>
void sampleRateConvSIMD_4tap( const std::pair<int, int> scalingRatio,
                              const std::pair<int, int> compScale,
                              const Pel*                orgSrc,
                              const ptrdiff_t           orgStride,
                              const int                 orgWidth,
                              const int                 orgHeight,
                              const int                 beforeScaleLeftOffset,
                              const int                 beforeScaleTopOffset,
                              Pel*                      scaledSrc,
                              const ptrdiff_t           scaledStride,
                              const int                 scaledWidth,
                              const int                 scaledHeight,
                              const int                 afterScaleLeftOffset,
                              const int                 afterScaleTopOffset,
                              const int                 bitDepth,
                              const bool                horCollocatedPositionFlag,
                              const bool                verCollocatedPositionFlag )
{
  static constexpr bool useLumaFilter = false;

  const TFilterCoeff* filterHor = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const TFilterCoeff* filterVer = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];

  const int numFracPositions  = useLumaFilter ? 15 : 31;
  const int numFracShift      = useLumaFilter ? 4 : 5;
  const int posShiftX = SCALE_RATIO_BITS - numFracShift + compScale.first;
  const int posShiftY = SCALE_RATIO_BITS - numFracShift + compScale.second;
  int addX = (1 << (posShiftX - 1)) + (beforeScaleLeftOffset << SCALE_RATIO_BITS) + ((int( 1 - horCollocatedPositionFlag ) * 8 * (scalingRatio.first - SCALE_1X.first) + (1 << (2 + compScale.first))) >> (3 + compScale.first));
  int addY = (1 << (posShiftY - 1)) + (beforeScaleTopOffset << SCALE_RATIO_BITS) + ((int( 1 - verCollocatedPositionFlag ) * 8 * (scalingRatio.second - SCALE_1X.second) + (1 << (2 + compScale.second))) >> (3 + compScale.second));

  const int filterLength = useLumaFilter ? NTAPS_LUMA : NTAPS_CHROMA;
  const int log2Norm = 12;

  CHECK( bitDepth > 17, "Overflow may happen!" );
  const int maxVal = (1 << bitDepth) - 1;

  const int tmpStride = ( ( scaledWidth + 3 ) / 4 ) * 4;
  const int tmpHeight = ( ( orgHeight + 3 ) / 4 ) * 4;
  int*      tmpBuf    = new int[tmpStride * tmpHeight];

  for( int j = 0; j < orgHeight; j += 4 )
  {
    const Pel* org0 = orgSrc +                                j * orgStride;
    const Pel* org1 = orgSrc + std::min( j + 1, orgHeight - 1 ) * orgStride;
    const Pel* org2 = orgSrc + std::min( j + 2, orgHeight - 1 ) * orgStride;
    const Pel* org3 = orgSrc + std::min( j + 3, orgHeight - 1 ) * orgStride;

    _mm_prefetch( ( const char* ) (org0 + (orgStride << 1)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (org1 + (orgStride << 1)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (org2 + (orgStride << 1)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (org3 + (orgStride << 1)), _MM_HINT_T0 );

    for( int i = 0; i < scaledWidth; i++ )
    {
      int refPos = (((i << compScale.first) - afterScaleLeftOffset) * scalingRatio.first + addX) >> posShiftX;
      int integer = refPos >> numFracShift;
      int frac = refPos & numFracPositions;

      const TFilterCoeff* f = filterHor + frac * filterLength;

      __m128i vsrc0, vsrc1, vsrc2, vsrc3;

      if( integer + 0 - (filterLength / 2) + 1 >= 0 && integer + (NTAPS_CHROMA - 1) - (filterLength / 2) + 1 < orgWidth )
      {
        int xInt = integer + 0 - (filterLength / 2) + 1;

        vsrc0 = _mm_loadu_si64( (const __m128i*) & org0[xInt] );
        vsrc1 = _mm_loadu_si64( (const __m128i*) & org1[xInt] );
        vsrc2 = _mm_loadu_si64( (const __m128i*) & org2[xInt] );
        vsrc3 = _mm_loadu_si64( (const __m128i*) & org3[xInt] );
      }
      else
      {
        Pel src[4][NTAPS_CHROMA];

        for( int k = 0; k < filterLength; k++ )
        {
          int xInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgWidth - 1 );

          src[0][k] = org0[xInt];
          src[1][k] = org1[xInt];
          src[2][k] = org2[xInt];
          src[3][k] = org3[xInt];
        }

        vsrc0 = _mm_loadu_si64( (const __m128i*) & src[0][0] );
        vsrc1 = _mm_loadu_si64( (const __m128i*) & src[1][0] );
        vsrc2 = _mm_loadu_si64( (const __m128i*) & src[2][0] );
        vsrc3 = _mm_loadu_si64( (const __m128i*) & src[3][0] );
      }

      __m128i vflt = _mm_loadu_si128( (const __m128i*) f );
      vflt = _mm_unpacklo_epi64( vflt, vflt );

      __m128i vres0 = _mm_madd_epi16( _mm_unpacklo_epi64( vsrc0, vsrc1 ), vflt );
      __m128i vres2 = _mm_madd_epi16( _mm_unpacklo_epi64( vsrc2, vsrc3 ), vflt );

      vres0 = _mm_hadd_epi32( vres0, vres2 );

      int* tmp = tmpBuf + i * tmpHeight + j;

      _mm_storeu_si128( (__m128i*) tmp, vres0 );
    }
  }

  __m128i vzero = _mm_setzero_si128();
  __m128i vnorm = _mm_set1_epi32( (1 << (log2Norm - 1)) );

  for( int i = 0; i < scaledWidth; i += 4 )
  {
    Pel* dst = scaledSrc;

    int* tmp0 =                       tmpBuf + i       * tmpHeight;
    int* tmp1 = i + 1 < scaledWidth ? tmpBuf + (i + 1) * tmpHeight : tmp0;
    int* tmp2 = i + 2 < scaledWidth ? tmpBuf + (i + 2) * tmpHeight : tmp0;
    int* tmp3 = i + 3 < scaledWidth ? tmpBuf + (i + 3) * tmpHeight : tmp0;

    _mm_prefetch( ( const char* ) (tmp0 + (tmpHeight << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (tmp1 + (tmpHeight << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (tmp2 + (tmpHeight << 2)), _MM_HINT_T0 );
    _mm_prefetch( ( const char* ) (tmp3 + (tmpHeight << 2)), _MM_HINT_T0 );

    for( int j = 0; j < scaledHeight; j++ )
    {
      const int refPos = (((j << compScale.second) - afterScaleTopOffset) * scalingRatio.second + addY) >> posShiftY;
      const int integer = refPos >> numFracShift;
      const int frac = refPos & numFracPositions;
      const TFilterCoeff* f = filterVer + frac * filterLength;

      __m128i vflt;
      vflt = _mm_cvtepi16_epi32( _mm_loadu_si128( (const __m128i*)  f ) );

      __m128i vsrc0, vsrc1, vsrc2, vsrc3;

      if( integer + 0 - (filterLength / 2) + 1 >= 0 && integer + (NTAPS_CHROMA - 1) - (filterLength / 2) + 1 < orgHeight )
      {
        int yInt = integer + 0 - (filterLength / 2) + 1;

        vsrc0 = _mm_loadu_si128( (const __m128i*) &tmp0[yInt] );
        vsrc1 = _mm_loadu_si128( (const __m128i*) &tmp1[yInt] );
        vsrc2 = _mm_loadu_si128( (const __m128i*) &tmp2[yInt] );
        vsrc3 = _mm_loadu_si128( (const __m128i*) &tmp3[yInt] );
      }
      else
      {
        int src[4][NTAPS_CHROMA];

        for( int k = 0; k < filterLength; k++ )
        {
          int yInt = std::min<int>( std::max( 0, integer + k - filterLength / 2 + 1 ), orgHeight - 1 );
          src[0][k] = tmp0[yInt];
          src[1][k] = tmp1[yInt];
          src[2][k] = tmp2[yInt];
          src[3][k] = tmp3[yInt];
        }

        vsrc0 = _mm_loadu_si128( (const __m128i*) & src[0][0] );
        vsrc1 = _mm_loadu_si128( (const __m128i*) & src[1][0] );
        vsrc2 = _mm_loadu_si128( (const __m128i*) & src[2][0] );
        vsrc3 = _mm_loadu_si128( (const __m128i*) & src[3][0] );
      }

      __m128i vres0 = _mm_mullo_epi32( vsrc0, vflt );
      __m128i vres1 = _mm_mullo_epi32( vsrc1, vflt );
      __m128i vres2 = _mm_mullo_epi32( vsrc2, vflt );
      __m128i vres3 = _mm_mullo_epi32( vsrc3, vflt );

      vres0 = _mm_hadd_epi32( vres0, vres1 );
      vres2 = _mm_hadd_epi32( vres2, vres3 );

      vres0 = _mm_hadd_epi32( vres0, vres2 );

      vres0 = _mm_add_epi32( vres0, vnorm );
      vres0 = _mm_srai_epi32( vres0, log2Norm );
      vres0 = _mm_max_epi32( _mm_min_epi32( _mm_set1_epi32( maxVal ), vres0 ), vzero );

      vres0 = _mm_packs_epi32( vres0, _mm_setzero_si128() );

      if( i + 3 < scaledWidth )
      {
        _mm_storeu_si64( (__m128i*) & dst[i], vres0 );
      }
      else if( i + 2 < scaledWidth )
      {
        _mm_storeu_si32( (__m128i*) &dst[i], vres0 );
        dst[i + 2] = _mm_extract_epi16( vres0, 2 );
      }
      else if( i + 1 < scaledWidth )
      {
        _mm_storeu_si32( (__m128i*) &dst[i], vres0 );
      }
      else
      {
        dst[i] = _mm_extract_epi16( vres0, 0 );
      }

      dst += scaledStride;
    }
  }

  delete[] tmpBuf;
}

template<X86_VEXT vext>
void sampleRateConvSIMD( const std::pair<int, int> scalingRatio,
                         const std::pair<int, int> compScale,
                         const Pel*                orgSrc,
                         const ptrdiff_t           orgStride,
                         const int                 orgWidth,
                         const int                 orgHeight,
                         const int                 beforeScaleLeftOffset,
                         const int                 beforeScaleTopOffset,
                         Pel*                      scaledSrc,
                         const ptrdiff_t           scaledStride,
                         const int                 scaledWidth,
                         const int                 scaledHeight,
                         const int                 afterScaleLeftOffset,
                         const int                 afterScaleTopOffset,
                         const int                 bitDepth,
                         const bool                useLumaFilter,
                         const bool                horCollocatedPositionFlag,
                         const bool                verCollocatedPositionFlag )
{
  if( orgWidth == scaledWidth && orgHeight == scaledHeight && scalingRatio == SCALE_1X && !beforeScaleLeftOffset && !beforeScaleTopOffset && !afterScaleLeftOffset && !afterScaleTopOffset )
  {
    g_pelBufOP.copyBuffer( ( const char* ) orgSrc, orgStride * sizeof( Pel ), ( char* ) scaledSrc, scaledStride * sizeof( Pel ), orgWidth * sizeof( Pel ), orgHeight );

    return;
  }
  else if( useLumaFilter )
  {
    sampleRateConvSIMD_8tap<vext>( scalingRatio, compScale, orgSrc, orgStride, orgWidth, orgHeight, beforeScaleLeftOffset, beforeScaleTopOffset, scaledSrc, scaledStride, scaledWidth, scaledHeight, afterScaleLeftOffset, afterScaleTopOffset, bitDepth );
  }
  else
  {
    sampleRateConvSIMD_4tap<vext>( scalingRatio, compScale, orgSrc, orgStride, orgWidth, orgHeight, beforeScaleLeftOffset, beforeScaleTopOffset, scaledSrc, scaledStride, scaledWidth, scaledHeight, afterScaleLeftOffset, afterScaleTopOffset, bitDepth, horCollocatedPositionFlag, verCollocatedPositionFlag );
  }
}

template<X86_VEXT vext>
void PelBufferOps::_initPelBufOpsX86()
{
  addAvg16 = addAvg_SSE<vext, 16>;
  addAvg8  = addAvg_SSE<vext,  8>;
  addAvg4  = addAvg_SSE<vext,  4>;

  reco8 = reco_SSE<vext, 8>;
  reco4 = reco_SSE<vext, 4>;

  linTf8 = linTf_SSE_entry<vext, 8>;
  linTf4 = linTf_SSE_entry<vext, 4>;
#if ENABLE_SIMD_OPT_GBI

  wghtAvg4 = addWghtAvg_SSE<vext, 4>;
  wghtAvg8 = addWghtAvg_SSE<vext, 8>;
#endif

  copyBuffer = copyBuffer_SSE<vext>;

  transpose4x4 = transposePel_SSE<vext, 4>;
  transpose8x8 = transposePel_SSE<vext, 8>;

#if defined( REAL_TARGET_X86 ) // looks like those function only really work for x86 SIMD
  if( vext >= AVX2 )
    applyLut = applyLut_SIMD<vext>;
  else
    rspBcw = rspBcwCore_SIMD<vext>;

#endif
  rspFwd = rspFwdCore_SIMD<vext>;

#if INTPTR_MAX == INT64_MAX || INTPTR_MAX == INT32_MAX
  fillN_CU = fillN_CU_SIMD<vext>;
#endif

  sampleRateConv = sampleRateConvSIMD<vext>;
}

template void PelBufferOps::_initPelBufOpsX86<SIMDX86>();

}

#endif // TARGET_SIMD_X86
#endif // ENABLE_SIMD_OPT_BUFFER
//! \}
