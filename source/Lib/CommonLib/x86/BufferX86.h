/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software, 
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2018-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


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


#if ENABLE_SIMD_OPT_BUFFER
#ifdef TARGET_SIMD_X86

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
        __m128i vsrc0 = _mm_loadl_epi64( ( const __m128i* )&src0[col] );
        __m128i vsrc1 = _mm_loadl_epi64( ( const __m128i* )&src1[col] );

        __m128i vsumlo = _mm_madd_epi16( _mm_unpacklo_epi16( vsrc0, vsrc1 ), vone );

        vsumlo = _mm_add_epi32        ( vsumlo, voffset );
        vsumlo = _mm_srai_epi32       ( vsumlo, shift );
        
        __m128i vsum = _mm_packs_epi32( vsumlo, vsumhi );
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );

        _mm_storel_epi64( ( __m128i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<X86_VEXT vext>
void paddingSimd2( Pel *dst, ptrdiff_t stride, int width, int height )
{
  __m128i x;
#ifdef USE_AVX2
  __m256i x16;
#endif
  int temp = width, j = 0;
#ifdef USE_AVX2
    while ((temp >> 4) > 0)
    {

      x16 = _mm256_loadu_si256((const __m256i*)(&(dst[j])));
    _mm256_storeu_si256( ( __m256i* )( dst + j - 1 * stride ), x16 );                     // top
    _mm256_storeu_si256( ( __m256i* )( dst + j - 2 * stride ), x16 );                     // top

      x16 = _mm256_loadu_si256((const __m256i*)(dst + j + (height - 1)*stride));
    _mm256_storeu_si256( ( __m256i* )( dst + j + ( height - 1 + 1 )*stride ), x16 );      // bottom
    _mm256_storeu_si256( ( __m256i* )( dst + j + ( height - 1 + 2 )*stride ), x16 );      // bottom


      j = j + 16;
      temp = temp - 16;
    }
#endif
    while ((temp >> 3) > 0)
    {
      x = _mm_loadu_si128((const __m128i*)(&(dst[j])));
    _mm_storeu_si128( ( __m128i* )( dst + j - 1 * stride ), x );
    _mm_storeu_si128( ( __m128i* )( dst + j - 2 * stride ), x );

      x = _mm_loadu_si128((const __m128i*)(dst + j + (height - 1)*stride));
    _mm_storeu_si128( ( __m128i* )( dst + j + ( height - 1 + 1 )*stride ), x );
    _mm_storeu_si128( ( __m128i* )( dst + j + ( height - 1 + 2 )*stride ), x );

      j = j + 8;
      temp = temp - 8;
    }
    while ((temp >> 2) > 0)
    {
      x = _mm_loadl_epi64((const __m128i*)(&dst[j]));
    _mm_storel_epi64( ( __m128i* )( dst + j - 1 * stride ), x );
    _mm_storel_epi64( ( __m128i* )( dst + j - 2 * stride ), x );

      x = _mm_loadl_epi64((const __m128i*)(dst + j + (height - 1)*stride));
    _mm_storel_epi64( ( __m128i* )( dst + j + ( height - 1 + 1 )*stride ), x );
    _mm_storel_epi64( ( __m128i* )( dst + j + ( height - 1 + 2 )*stride ), x );

      j = j + 4;
      temp = temp - 4;
    }
    while (temp > 0)
    {
    dst[j - 2 * stride] = dst[j];
    dst[j - 1 * stride] = dst[j];
    dst[j + ( height - 1 + 1 )*stride] = dst[j + ( height - 1 )*stride];
    dst[j + ( height - 1 + 2 )*stride] = dst[j + ( height - 1 )*stride];
      j++;
      temp--;
    }

  static constexpr int padSize = 2;

  //Left and Right Padding
  Pel* ptr1 = dst - padSize*stride;
  Pel* ptr2 = dst - padSize*stride + width - 1;
  ptrdiff_t offset = 0;
  for (int i = 0; i < height + 2 * padSize; i++)
  {
    offset = stride * i;

    *( ptr1 - 2 + offset ) = *( ptr1 + offset );      // left
    *( ptr1 - 1 + offset ) = *( ptr1 + offset );      // left
    *( ptr2 + 1 + offset ) = *( ptr2 + offset );      // right
    *( ptr2 + 2 + offset ) = *( ptr2 + offset );      // right
  }
}

template<X86_VEXT vext>
void paddingSimd1( Pel *dst, ptrdiff_t stride, int width, int height )
    {
  __m128i x;
#ifdef USE_AVX2
  __m256i x16;
#endif
  int temp = width, j = 0;
#ifdef USE_AVX2
  while( ( temp >> 4 ) > 0 )
  {

    x16 = _mm256_loadu_si256( ( const __m256i* )( &( dst[j] ) ) );
    _mm256_storeu_si256( ( __m256i* )( dst + j - 1 * stride ), x16 );                     // top

    x16 = _mm256_loadu_si256( ( const __m256i* )( dst + j + ( height - 1 )*stride ) );
    _mm256_storeu_si256( ( __m256i* )( dst + j + ( height - 1 + 1 )*stride ), x16 );      // bottom


    j = j + 16;
    temp = temp - 16;
  }
#endif
  while( ( temp >> 3 ) > 0 )
  {
    x = _mm_loadu_si128( ( const __m128i* )( &( dst[j] ) ) );
    _mm_storeu_si128( ( __m128i* )( dst + j - 1 * stride ), x );

    x = _mm_loadu_si128( ( const __m128i* )( dst + j + ( height - 1 )*stride ) );
    _mm_storeu_si128( ( __m128i* )( dst + j + ( height - 1 + 1 )*stride ), x );

    j = j + 8;
    temp = temp - 8;
  }
  while( ( temp >> 2 ) > 0 )
  {
    x = _mm_loadl_epi64( ( const __m128i* )( &dst[j] ) );
    _mm_storel_epi64( ( __m128i* )( dst + j - 1 * stride ), x );

    x = _mm_loadl_epi64( ( const __m128i* )( dst + j + ( height - 1 )*stride ) );
    _mm_storel_epi64( ( __m128i* )( dst + j + ( height - 1 + 1 )*stride ), x );

    j = j + 4;
    temp = temp - 4;
  }
  while( temp > 0 )
    {
    dst[j - 1 * stride] = dst[j];
    dst[j + ( height - 1 + 1 )*stride] = dst[j + ( height - 1 )*stride];
    j++;
    temp--;
    }

  static constexpr int padSize = 1;

  //Left and Right Padding
  Pel* ptr1 = dst - stride;
  Pel* ptr2 = dst - stride + width - 1;
  ptrdiff_t offset = 0;
  for( int i = 0; i < height + 2 * padSize; i++ )
  {
    offset = stride * i;
    *( ptr1 - 1 + offset ) = *( ptr1 + offset );      // left
    *( ptr2 + 1 + offset ) = *( ptr2 + offset );      // right
  }
}

template< X86_VEXT vext, int W >
void reco_SSE( const int16_t* src0, ptrdiff_t src0Stride, const int16_t* src1, ptrdiff_t src1Stride, int16_t *dst, ptrdiff_t dstStride, int width, int height, const ClpRng& clpRng )
{
  // src0 needs to be aligned for AVX2

  if( W == 8 )
  {
#if USE_AVX2
    if( vext >= AVX2 && ( width & 15 ) == 0 )
    {
      __m256i vbdmin = _mm256_set1_epi16( clpRng.min() );
      __m256i vbdmax = _mm256_set1_epi16( clpRng.max() );

      for( int row = 0; row < height; row++ )
      {
        for( int col = 0; col < width; col += 16 )
        {
          __m256i vdest = _mm256_loadu_si256( ( const __m256i * )&src0[col] );
          __m256i vsrc1 = _mm256_loadu_si256( ( const __m256i * )&src1[col] );

          vdest = _mm256_add_epi16( vdest, vsrc1 );
          vdest = _mm256_min_epi16( vbdmax, _mm256_max_epi16( vbdmin, vdest ) );

          _mm256_storeu_si256( ( __m256i * )&dst[col], vdest );
        }

        src0 += src0Stride;
        src1 += src1Stride;
        dst  += dstStride;
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

          vdest = _mm_add_epi16( vdest, vsrc1 );
          vdest = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, vdest ) );

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
        __m128i vsrc = _mm_loadl_epi64( ( const __m128i * )&src0[col] );
        __m128i vdst = _mm_loadl_epi64( ( const __m128i * )&src1[col] );

        vdst = _mm_add_epi16( vdst, vsrc );
        vdst = _mm_min_epi16( vbdmax, _mm_max_epi16( vbdmin, vdst ) );

        _mm_storel_epi64( ( __m128i * )&dst[col], vdst );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
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
        __m128i vsum = _mm_loadl_epi64  ( ( const __m128i * )&src0[col] );
        __m128i vdst = _mm_loadl_epi64  ( ( const __m128i * )&src1[col] );
        vsum = _mm_madd_epi16           ( vw, _mm_unpacklo_epi16( vsum, vdst ) );
        vsum = _mm_add_epi32            ( vsum, voffset );
        vsum = _mm_srai_epi32           ( vsum, shift );
        vsum = _mm_packs_epi32          ( vsum, vzero );

        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
        _mm_storel_epi64( ( __m128i * )&dst[col], vsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

template<bool doShift, bool shiftR, typename T> static inline void do_shift( T &vreg, int num );
#if USE_AVX2
template<> inline void do_shift<true,  true , __m256i>( __m256i &vreg, int num ) { vreg = _mm256_srai_epi32( vreg, num ); }
template<> inline void do_shift<true,  false, __m256i>( __m256i &vreg, int num ) { vreg = _mm256_slli_epi32( vreg, num ); }
template<> inline void do_shift<false, true , __m256i>( __m256i &vreg, int num ) { }
template<> inline void do_shift<false, false, __m256i>( __m256i &vreg, int num ) { }
#endif
template<> inline void do_shift<true,  true , __m128i>( __m128i &vreg, int num ) { vreg = _mm_srai_epi32( vreg, num ); }
template<> inline void do_shift<true,  false, __m128i>( __m128i &vreg, int num ) { vreg = _mm_slli_epi32( vreg, num ); }
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
        val = _mm_loadl_epi64             ( ( const __m128i * )&src[col] );
        val = _mm_cvtepi16_epi32          ( val );
        do_mult<doMult, __m128i>          ( val, vscale );
        do_shift<doShift, shiftR, __m128i>( val, shift );
        do_add<doAdd, __m128i>            ( val, voffset );
        val = _mm_packs_epi32             ( val, vzero );
        do_clip<clip, __m128i>            ( val, vbdmin, vbdmax );

        _mm_storel_epi64                  ( ( __m128i * )&dst[col], val );
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

    va = _mm_loadl_epi64( ( const __m128i* ) src ); src += srcStride;
    vb = _mm_loadl_epi64( ( const __m128i* ) src ); src += srcStride;
    vc = _mm_loadl_epi64( ( const __m128i* ) src ); src += srcStride;
    vd = _mm_loadl_epi64( ( const __m128i* ) src );

    __m128i va01b01 = _mm_unpacklo_epi16( va,      vb );
    __m128i va23b23 = _mm_unpackhi_epi64( va01b01, vb );
    __m128i vc01d01 = _mm_unpacklo_epi16( vc,      vd );
    __m128i vc23d23 = _mm_unpackhi_epi64( vc01d01, vd );

    va = _mm_unpacklo_epi32( va01b01, vc01d01 );
    vb = _mm_unpackhi_epi64( va,      va );
    vc = _mm_unpacklo_epi32( va23b23, vc23d23 );
    vd = _mm_unpackhi_epi64( vc,      vc );

    _mm_storel_epi64( ( __m128i* ) dst, va ); dst += dstStride;
    _mm_storel_epi64( ( __m128i* ) dst, vb ); dst += dstStride;
    _mm_storel_epi64( ( __m128i* ) dst, vc ); dst += dstStride;
    _mm_storel_epi64( ( __m128i* ) dst, vd );
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
        __m256i vin16    = _mm256_loadu_si256       ( ( const __m256i * ) &ptr[x] );
        __m256i vin16x   = _mm256_loadu_si256       ( ( const __m256i * ) &ptr[x + ptrStride] );
                                                    
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

        _mm256_storeu_si256( ( __m256i * ) &ptr[x],             vout16 );
        _mm256_storeu_si256( ( __m256i * ) &ptr[x + ptrStride], vout16x );
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

template<X86_VEXT vext>
void PelBufferOps::_initPelBufOpsX86()
{
  addAvg16 = addAvg_SSE<vext, 16>;
  addAvg8  = addAvg_SSE<vext,  8>;
  addAvg4  = addAvg_SSE<vext,  4>;

  padding1 = paddingSimd1<vext>;
  padding2 = paddingSimd2<vext>;

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

  if( vext >= AVX2 )
    applyLut = applyLut_SIMD<vext>;

  fillN_CU = fillN_CU_SIMD<vext>;
}

template void PelBufferOps::_initPelBufOpsX86<SIMDX86>();

#endif // TARGET_SIMD_X86
#endif
//! \}
