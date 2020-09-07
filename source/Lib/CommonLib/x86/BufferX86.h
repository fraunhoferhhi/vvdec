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
    __m256i voffset   = _mm256_set1_epi32( offset );
    __m256i vibdimin  = _mm256_set1_epi16( clpRng.min() );
    __m256i vibdimax  = _mm256_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 16 )
      {
        __m256i vsrc0 = _mm256_load_si256( ( const __m256i* )&src0[col] );
        __m256i vsrc1 = _mm256_load_si256( ( const __m256i* )&src1[col] );
        __m256i vsgn0 = _mm256_cmpgt_epi16( _mm256_setzero_si256(), vsrc0 );
        __m256i vsgn1 = _mm256_cmpgt_epi16( _mm256_setzero_si256(), vsrc1 );

        __m256i vsumlo = _mm256_add_epi32( _mm256_unpacklo_epi16( vsrc0, vsgn0 ),
                                           _mm256_unpacklo_epi16( vsrc1, vsgn1 ) );
        __m256i vsumhi = _mm256_add_epi32( _mm256_unpackhi_epi16( vsrc0, vsgn0 ),
                                           _mm256_unpackhi_epi16( vsrc1, vsgn1 ) );
 
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
  else if( W >= 8 )
  {
    __m256i voffset  = _mm256_set1_epi32( offset );
    __m128i vibdimin = _mm_set1_epi16   ( clpRng.min() );
    __m128i vibdimax = _mm_set1_epi16   ( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i vsrc0 = _mm256_cvtepi16_epi32( _mm_load_si128( ( const __m128i* )&src0[col] ) );
        __m256i vsrc1 = _mm256_cvtepi16_epi32( _mm_load_si128( ( const __m128i* )&src1[col] ) );

        __m256i
        vsum = _mm256_add_epi32        ( vsrc0, vsrc1 );
        vsum = _mm256_add_epi32        ( vsum, voffset );
        vsum = _mm256_srai_epi32       ( vsum, shift );

        __m128i
        xsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, _mm256_cvtepi32_epi16x( vsum ) ) );
        _mm_storeu_si128( ( __m128i * )&dst[col], xsum );
      }

      src0 += src0Stride;
      src1 += src1Stride;
      dst  +=  dstStride;
    }
  }
#else
  if( W >= 8 )
  {
    __m128i vzero    = _mm_setzero_si128();
    __m128i voffset  = _mm_set1_epi32( offset );
    __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax = _mm_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m128i vsrc0 = _mm_load_si128( ( const __m128i* )&src0[col] );
        __m128i vsrc1 = _mm_load_si128( ( const __m128i* )&src1[col] );

        __m128i vtmp, vsum, vdst;
        vsum = _mm_cvtepi16_epi32   ( vsrc0 );
        vdst = _mm_cvtepi16_epi32   ( vsrc1 );
        vsum = _mm_add_epi32        ( vsum, vdst );
        vsum = _mm_add_epi32        ( vsum, voffset );
        vtmp = _mm_srai_epi32       ( vsum, shift );

        vsrc0 = _mm_unpackhi_epi64  ( vsrc0, vzero );
        vsrc1 = _mm_unpackhi_epi64  ( vsrc1, vzero );
        vsum = _mm_cvtepi16_epi32   ( vsrc0 );
        vdst = _mm_cvtepi16_epi32   ( vsrc1 );
        vsum = _mm_add_epi32        ( vsum, vdst );
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
#endif
  else if( W == 4 )
  {
    __m128i vzero     = _mm_setzero_si128();
    __m128i voffset   = _mm_set1_epi32( offset );
    __m128i vibdimin  = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax  = _mm_set1_epi16( clpRng.max() );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i vsum = _mm_loadl_epi64  ( ( const __m128i * )&src0[col] );
        __m128i vdst = _mm_loadl_epi64  ( ( const __m128i * )&src1[col] );
        vsum = _mm_cvtepi16_epi32       ( vsum );
        vdst = _mm_cvtepi16_epi32       ( vdst );
        vsum = _mm_add_epi32            ( vsum, vdst );
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
    __m128i voffset  = _mm_set1_epi32( offset );
    __m128i vibdimin = _mm_set1_epi16( clpRng.min() );
    __m128i vibdimax = _mm_set1_epi16( clpRng.max() );
    __m128i vw       = _mm_unpacklo_epi16( _mm_set1_epi16( w0 ), _mm_set1_epi16( w1 ) );

    for( int row = 0; row < height; row++ )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m128i vsrc0 = _mm_load_si128( ( const __m128i * )&src0[col] );
        __m128i vsrc1 = _mm_load_si128( ( const __m128i * )&src1[col] );

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
  _mm_prefetch( ( const char* ) &lut[ptr[0]], _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &ptr[0 * ptrStride], _MM_HINT_T0 );
  _mm_prefetch( ( const char* ) &ptr[1 * ptrStride], _MM_HINT_T0 );

#if USE_AVX2
  if( ( width & 15 ) == 0 && ( height & 1 ) == 0 )
  {
    const __m256i vLutShuf = _mm256_setr_epi8( 0, 1, 4, 5, 8, 9, 12, 13, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 4, 5, 8, 9, 12, 13, -1, -1, -1, -1, -1, -1, -1, -1 );

    for( int y = 0; y < height; y += 2 )
    {
      _mm_prefetch( ( const char* ) &ptr[2 * ptrStride], _MM_HINT_T0 );
      _mm_prefetch( ( const char* ) &ptr[3 * ptrStride], _MM_HINT_T0 );

      for( int x = 0; x < width; x += 16 )
      {
        __m256i vin16    = _mm256_loadu_si256       ( ( const __m256i * ) &ptr[x] );
                                                    
        __m256i vin32_1  = _mm256_unpacklo_epi16    ( vin16, _mm256_setzero_si256() );
        __m256i vin32_2  = _mm256_unpackhi_epi16    ( vin16, _mm256_setzero_si256() );

        __m256i vout32_1 = _mm256_i32gather_epi32   ( ( const int * ) lut, vin32_1, 2 );
        __m256i vout32_2 = _mm256_i32gather_epi32   ( ( const int * ) lut, vin32_2, 2 );

        vout32_1         = _mm256_shuffle_epi8      ( vout32_1, vLutShuf );
        vout32_2         = _mm256_shuffle_epi8      ( vout32_2, vLutShuf );

        __m256i vout16   = _mm256_unpacklo_epi64    ( vout32_1, vout32_2 );

        _mm256_storeu_si256( ( __m256i * ) &ptr[x], vout16 );
        
        vin16            = _mm256_loadu_si256       ( ( const __m256i * ) &ptr[x + ptrStride] );
                                                    
        vin32_1          = _mm256_unpacklo_epi16    ( vin16, _mm256_setzero_si256() );
        vin32_2          = _mm256_unpackhi_epi16    ( vin16, _mm256_setzero_si256() );
                         
        vout32_1         = _mm256_i32gather_epi32   ( ( const int * ) lut, vin32_1, 2 );
        vout32_2         = _mm256_i32gather_epi32   ( ( const int * ) lut, vin32_2, 2 );

        vout32_1         = _mm256_shuffle_epi8      ( vout32_1, vLutShuf );
        vout32_2         = _mm256_shuffle_epi8      ( vout32_2, vLutShuf );

        vout16           = _mm256_unpacklo_epi64    ( vout32_1, vout32_2 );

        _mm256_storeu_si256( ( __m256i * ) &ptr[x + ptrStride], vout16 );
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
