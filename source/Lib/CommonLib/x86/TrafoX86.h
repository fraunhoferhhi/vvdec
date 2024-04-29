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

/** \file     TrafoX86.h
    \brief    SIMD trafo
*/

//! \ingroup CommonLib
//! \{


#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"

#include "CommonDefX86.h"

#include "TrQuant.h"
#include "TrQuant_EMT.h"

namespace vvdec
{

#if ENABLE_SIMD_TCOEFF_OPS
#ifdef TARGET_SIMD_X86

template< X86_VEXT vext, int trSize >
void fastInv_SSE( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned lines, unsigned reducedLines, unsigned rows )
{
  unsigned maxLoopL = std::min<int>( reducedLines, 4 );

#if USE_AVX2
  if( trSize >= 8 && vext >= AVX2 )
  {
    if( ( trSize & 15 ) == 0 )
    {
      static constexpr unsigned trLoops = trSize >= 16 ? trSize >> 4 : 1;

      for( int k = 0; k < rows; k += 2 )
      {
              TCoeff* dstPtr =  dst;

        const TCoeff* srcPtr0 = &src[ k      * lines];
        const TCoeff* srcPtr1 = &src[(k + 1) * lines];

        __m256i vsrc1v[trLoops][2];
        
        const TMatrixCoeff*  itPtr0 = &it[ k      * trSize];
        const TMatrixCoeff*  itPtr1 = &it[(k + 1) * trSize];

        for( int col = 0; col < trLoops; col++, itPtr0 += 16, itPtr1 += 16 )
        {
#if defined( _MSC_VER ) && _MSC_VER > 1900
          __m256i vit16_0 = _mm256_permute4x64_epi64( _mm256_stream_load_si256( ( const __m256i * ) itPtr0 ), ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
          __m256i vit16_1 = _mm256_permute4x64_epi64( _mm256_stream_load_si256( ( const __m256i * ) itPtr1 ), ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
#else
          __m256i vit16_0 = _mm256_permute4x64_epi64( _mm256_stream_load_si256( (       __m256i * ) itPtr0 ), ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
          __m256i vit16_1 = _mm256_permute4x64_epi64( _mm256_stream_load_si256( (       __m256i * ) itPtr1 ), ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
#endif

          vsrc1v[col][0] = _mm256_unpacklo_epi16( vit16_0, vit16_1 );
          vsrc1v[col][1] = _mm256_unpackhi_epi16( vit16_0, vit16_1 );
        }

        for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
        {
          __m128i xscale = maxLoopL == 4
                         ? _mm_packs_epi32( _mm_load_si128( ( const __m128i* )srcPtr0 ), _mm_load_si128( ( const __m128i* )srcPtr1 ) )
                         : _mm_packs_epi32( _mm_loadu_si64( ( const __m128i* )srcPtr0 ), _mm_loadu_si64( ( const __m128i* )srcPtr1 ) );
          xscale = _mm_shuffle_epi8( xscale, _mm_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 ) );

          if( _mm_test_all_zeros( xscale, xscale ) ) { dstPtr += ( trSize * maxLoopL ); continue; }

          for( int l = 0; l < maxLoopL; l++ )
          {
            __m256i
            vscale = _mm256_broadcastd_epi32( xscale );
            xscale = _mm_bsrli_si128( xscale, 4 );

            for( int col = 0; col < trLoops; col++, dstPtr += 16 )
            {
              __m256i vsrc0 = _mm256_load_si256       ( ( const __m256i * ) dstPtr );

              __m256i
              vsrc1 = vsrc1v[col][0];
              vsrc1 = _mm256_madd_epi16    ( vsrc1, vscale );
              vsrc0 = _mm256_add_epi32     ( vsrc0, vsrc1 );

              _mm256_store_si256           ( ( __m256i * ) dstPtr, vsrc0 );
            
              vsrc0 = _mm256_load_si256    ( ( const __m256i * ) &dstPtr[8] );

              vsrc1 = vsrc1v[col][1];
              vsrc1 = _mm256_madd_epi16    ( vsrc1, vscale );
              vsrc0 = _mm256_add_epi32     ( vsrc0, vsrc1 );

              _mm256_store_si256           ( ( __m256i * ) &dstPtr[8], vsrc0 );
            }
          }
        }
      }
    }
    else
    {
      for( int k = 0; k < rows; k += 2 )
      {
              TCoeff* dstPtr  =  dst;

        const TCoeff* srcPtr0 = &src[ k      * lines];
        const TCoeff* srcPtr1 = &src[(k + 1) * lines];

        const TMatrixCoeff*  itPtr0 = &it[  k      * trSize];
        const TMatrixCoeff*  itPtr1 = &it[( k + 1 ) * trSize];

        __m256i vit;

        {
#if defined( _MSC_VER ) && _MSC_VER > 1900
          __m256i vsrc1 = _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_stream_load_si128( ( const __m128i * ) itPtr0 ) ), ( 0 << 0 ) + ( 1 << 4 ) );
#else
          __m256i vsrc1 = _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_stream_load_si128( ( __m128i * ) itPtr0 ) ), ( 0 << 0 ) + ( 1 << 4 ) );
#endif
#if defined( _MSC_VER ) && _MSC_VER > 1900
          __m256i vsrc2 = _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_stream_load_si128( ( const __m128i * ) itPtr1 ) ), ( 0 << 0 ) + ( 1 << 4 ) );
#else
          __m256i vsrc2 = _mm256_permute4x64_epi64( _mm256_castsi128_si256( _mm_stream_load_si128( ( __m128i * ) itPtr1 ) ), ( 0 << 0 ) + ( 1 << 4 ) );
#endif

          vit = _mm256_unpacklo_epi16( vsrc1, vsrc2 );
        }
        
        for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
        {
          __m128i xscale = maxLoopL == 4
                         ? _mm_packs_epi32( _mm_load_si128( ( const __m128i* )srcPtr0 ), _mm_load_si128( ( const __m128i* )srcPtr1 ) )
                         : _mm_packs_epi32( _mm_loadu_si64( ( const __m128i* )srcPtr0 ), _mm_loadu_si64( ( const __m128i* )srcPtr1 ) );
          xscale = _mm_shuffle_epi8( xscale, _mm_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 ) );

          if( _mm_test_all_zeros( xscale, xscale ) ) { dstPtr += ( trSize * maxLoopL ); continue; }

          for( int l = 0; l < maxLoopL; l++ )
          {
            __m256i
            vscale = _mm256_broadcastd_epi32( xscale );
            xscale = _mm_bsrli_si128( xscale, 4 );

            for( int col = 0; col < trSize; col += 8, dstPtr += 8, itPtr0 += 8, itPtr1 += 8 )
            {
              __m256i
              vsrc0 = _mm256_load_si256    ( ( const __m256i * ) dstPtr );
              __m256i
              vsrc1 = _mm256_madd_epi16    ( vit, vscale );
              vsrc0 = _mm256_add_epi32     ( vsrc0, vsrc1 );

              _mm256_store_si256           ( ( __m256i * ) dstPtr, vsrc0 );
            }
          }
        }
      }
    }
  }
#else
  if( trSize >= 8 )
  {
    for( int k = 0; k < rows; k += 2 )
    {
            TCoeff* dstPtr  =  dst;

      const TCoeff* srcPtr0 = &src[ k      * lines];
      const TCoeff* srcPtr1 = &src[(k + 1) * lines];
        
      for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
      {
        __m128i xscale = maxLoopL == 4
                        ? _mm_packs_epi32( _mm_load_si128( ( const __m128i* )srcPtr0 ), _mm_load_si128( ( const __m128i* )srcPtr1 ) )
                        : _mm_packs_epi32( _mm_loadu_si64( ( const __m128i* )srcPtr0 ), _mm_loadu_si64( ( const __m128i* )srcPtr1 ) );
        xscale = _mm_shuffle_epi8( xscale, _mm_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 ) );

        if( _mm_test_all_zeros( xscale, xscale ) ) { dstPtr += ( trSize * maxLoopL ); continue; }

        for( int l = 0; l < maxLoopL; l++ )
        {
          const TMatrixCoeff*  itPtr0 = &it[k      * trSize];
          const TMatrixCoeff*  itPtr1 = &it[( k + 1 ) * trSize];

          __m128i
          vscale = _mm_set1_epi32( _mm_cvtsi128_si32( xscale ) );
          xscale = _mm_bsrli_si128( xscale, 4 );

          for( int col = 0; col < trSize; col += 8, dstPtr += 8, itPtr0 += 8, itPtr1 += 8 )
          {
            __m128i vsrc0   = _mm_load_si128       ( ( const __m128i * ) dstPtr );
#if defined( _MSC_VER ) && _MSC_VER > 1900
            __m128i vit16_0 = _mm_stream_load_si128( ( const __m128i * ) itPtr0 );
            __m128i vit16_1 = _mm_stream_load_si128( ( const __m128i * ) itPtr1 );
#else
            __m128i vit16_0 = _mm_stream_load_si128( (       __m128i * ) itPtr0 );
            __m128i vit16_1 = _mm_stream_load_si128( (       __m128i * ) itPtr1 );
#endif

            __m128i vsrc1 = _mm_unpacklo_epi16( vit16_0, vit16_1 );

            vsrc1 = _mm_madd_epi16 ( vsrc1, vscale );
            vsrc0 = _mm_add_epi32  ( vsrc0, vsrc1 );

            _mm_store_si128        ( ( __m128i * ) dstPtr, vsrc0 );
          
            vsrc0 = _mm_load_si128 ( ( const __m128i * ) &dstPtr[4] );
          
            vsrc1 = _mm_unpackhi_epi16( vit16_0, vit16_1 );

            vsrc1 = _mm_madd_epi16 ( vsrc1, vscale );
            vsrc0 = _mm_add_epi32  ( vsrc0, vsrc1 );
          
            _mm_store_si128        ( ( __m128i * ) &dstPtr[4], vsrc0 );
          }
        }
      }
    }
  }
#endif
  else if( trSize >= 4 )
  {
    CHECKD( trSize != 4, "trSize needs to be '4'!" );

    for( int k = 0; k < rows; k += 2 )
    {
            TCoeff* dstPtr  =  dst;

      const TCoeff* srcPtr0 = &src[ k      * lines];
      const TCoeff* srcPtr1 = &src[(k + 1) * lines];

      const TMatrixCoeff*  itPtr0 = &it[  k       * trSize];
      const TMatrixCoeff*  itPtr1 = &it[( k + 1 ) * trSize];

      __m128i vit = _mm_unpacklo_epi16( _mm_loadu_si64( ( const __m128i * ) itPtr0 ), _mm_loadu_si64( ( const __m128i * ) itPtr1 ) );
 
      for( int i = 0; i < reducedLines; i += 4, srcPtr0 += maxLoopL, srcPtr1 += maxLoopL )
      {
        __m128i xscale = maxLoopL == 4
                        ? _mm_packs_epi32( _mm_load_si128( ( const __m128i* )srcPtr0 ), _mm_load_si128( ( const __m128i* )srcPtr1 ) )
                        : _mm_packs_epi32( _mm_loadu_si64( ( const __m128i* )srcPtr0 ), _mm_loadu_si64( ( const __m128i* )srcPtr1 ) );
        xscale = _mm_shuffle_epi8( xscale, _mm_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 ) );

        if( _mm_test_all_zeros( xscale, xscale ) ) { dstPtr += ( trSize * maxLoopL ); continue; }

        for( int l = 0; l < maxLoopL; l++ )
        {
          __m128i
          vscale = _mm_set1_epi32( _mm_cvtsi128_si32( xscale ) );
          xscale = _mm_bsrli_si128( xscale, 4 );

          for( int col = 0; col < trSize; col += 4, dstPtr += 4 )
          {
            __m128i
            vsrc0 = _mm_load_si128 ( ( const __m128i * ) dstPtr );
            __m128i 
            vsrc1 = _mm_madd_epi16 ( vit, vscale );
            vsrc0 = _mm_add_epi32  ( vsrc0, vsrc1 );

            _mm_store_si128        ( ( __m128i * ) dstPtr, vsrc0 );
          }
        }
      }
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
void roundClip_SSE( TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift )
{
#if USE_AVX2
  if( W >= 8 && vext >= AVX2 )
  {
    __m256i vmin = _mm256_set1_epi32( outputMin );
    __m256i vmax = _mm256_set1_epi32( outputMax );
    __m256i vrnd = _mm256_set1_epi32( round );

    while( height-- )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m256i
        vdst = _mm256_load_si256( ( __m256i * ) &dst[col] );
        vdst = _mm256_add_epi32 ( vdst, vrnd );
        vdst = _mm256_srai_epi32( vdst, shift );
        vdst = _mm256_max_epi32 ( vdst, vmin );
        vdst = _mm256_min_epi32 ( vdst, vmax );
        _mm256_store_si256      ( ( __m256i * ) &dst[col], vdst );
      }

      dst += stride;
    }
  }
  else
#endif
  if( W >= 4 )
  {
    __m128i vmin = _mm_set1_epi32( outputMin );
    __m128i vmax = _mm_set1_epi32( outputMax );
    __m128i vrnd = _mm_set1_epi32( round );

    while( height-- )
    {
      for( int col = 0; col < width; col += 4 )
      {
        __m128i
        vdst = _mm_load_si128 ( ( __m128i * ) &dst[col] );
        vdst = _mm_add_epi32  ( vdst, vrnd );
        vdst = _mm_srai_epi32 ( vdst, shift );
        vdst = _mm_max_epi32  ( vdst, vmin );
        vdst = _mm_min_epi32  ( vdst, vmax );
        _mm_store_si128       ( ( __m128i * ) &dst[col], vdst );
      }

      dst += stride;
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
void cpyResiClip_SSE( const TCoeff* src, Pel* dst, ptrdiff_t stride, unsigned width, unsigned height, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift )
{
#if USE_AVX2
  if( W >= 16 )
  {
    __m256i vmin = _mm256_set1_epi32( outputMin );
    __m256i vmax = _mm256_set1_epi32( outputMax );
    __m256i vrnd = _mm256_set1_epi32( round );

    while( height-- )
    {
      for( int col = 0; col < width; col += 16 )
      {
        __m256i
        vsrc1 = _mm256_load_si256 ( ( const __m256i * ) &src[col] );
        vsrc1 = _mm256_add_epi32  ( vsrc1, vrnd );
        vsrc1 = _mm256_srai_epi32 ( vsrc1, shift );
        vsrc1 = _mm256_max_epi32  ( _mm256_min_epi32( vsrc1, vmax ), vmin );

        __m256i                
        vsrc2 = _mm256_load_si256 ( ( const __m256i * ) &src[col+8] );
        vsrc2 = _mm256_add_epi32  ( vsrc2, vrnd );
        vsrc2 = _mm256_srai_epi32 ( vsrc2, shift );
        vsrc2 = _mm256_max_epi32  ( _mm256_min_epi32( vsrc2, vmax ), vmin );

        __m256i
        vdst  = _mm256_packs_epi32( vsrc1, vsrc2 );
        vdst  = _mm256_permute4x64_epi64( vdst, ( 0 << 0 ) + ( 1 << 4 ) + ( 2 << 2 ) + ( 3 << 6 ) );
        _mm256_storeu_si256       ( ( __m256i * ) &dst[col], vdst );
      }

      src += width;
      dst += stride;
    }
  }
  else
#endif
  if( W >= 8 )
  {
    __m128i vmin = _mm_set1_epi32( outputMin );
    __m128i vmax = _mm_set1_epi32( outputMax );
    __m128i vrnd = _mm_set1_epi32( round );

    while( height-- )
    {
      for( int col = 0; col < width; col += 8 )
      {
        __m128i
        vsrc1 = _mm_load_si128 ( ( const __m128i * ) &src[col] );
        vsrc1 = _mm_add_epi32  ( vsrc1, vrnd );
        vsrc1 = _mm_srai_epi32 ( vsrc1, shift );
        vsrc1 = _mm_max_epi32  ( _mm_min_epi32( vsrc1, vmax ), vmin );
        __m128i                
        vsrc2 = _mm_load_si128 ( ( const __m128i * ) &src[col+4] );
        vsrc2 = _mm_add_epi32  ( vsrc2, vrnd );
        vsrc2 = _mm_srai_epi32 ( vsrc2, shift );
        vsrc2 = _mm_max_epi32  ( _mm_min_epi32( vsrc2, vmax ), vmin );
        __m128i
        vdst  = _mm_packs_epi32( vsrc1, vsrc2 );
        _mm_storeu_si128       ( ( __m128i * ) &dst[col], vdst );
      }

      src += width;
      dst += stride;
    }
  }
  else if( W >= 4 )
  {
    __m128i vmin = _mm_set1_epi32( outputMin );
    __m128i vmax = _mm_set1_epi32( outputMax );
    __m128i vrnd = _mm_set1_epi32( round );

    __m128i vzero = _mm_setzero_si128();
    __m128i vdst;

    while( height-- )
    {
      for( int col = 0; col < width; col += 4 )
      {
        vdst = _mm_load_si128 ( ( const __m128i * ) &src[col] );
        vdst = _mm_add_epi32  ( vdst, vrnd );
        vdst = _mm_srai_epi32 ( vdst, shift );
        vdst = _mm_max_epi32  ( _mm_min_epi32( vdst, vmax ), vmin );
        vdst = _mm_packs_epi32( vdst, vzero );
        _mm_storeu_si64       ( ( __m128i * ) &dst[col], vdst );
      }

      src += width;
      dst += stride;
    }
  }
  else
  {
    THROW_FATAL( "Unsupported size" );
  }
}

template<X86_VEXT vext>
static void simdInvLfnstNxNCore( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  CHECK( index > 2 || ( zeroOutSize != 8 && zeroOutSize != 16 ), "Wrong parameters" );

  static constexpr int maxLog2TrDynamicRange = 15;
  const TCoeff    outputMinimum = -( 1 << maxLog2TrDynamicRange );
  const TCoeff    outputMaximum =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const int8_t*   trMat         = ( size > 4 ) ? g_lfnst8x8[mode][index][0] : g_lfnst4x4[mode][index][0];
  const int       trSize        = ( size > 4 ) ? 48 : 16;
  int*            out           = dst;

  const __m128i vzero = _mm_setzero_si128();
  const __m128i vmin  = _mm_set1_epi32( outputMinimum );
  const __m128i vmax  = _mm_set1_epi32( outputMaximum );

  for( int j = 0; j < trSize; j += 4, out += 4 )
  {
    __m128i       vsum[4];

    for( int k = 0; k < 4; k++, trMat += 16 )
    {
      const int8_t* trMatTmp = trMat;
      int* srcPtr = src;

      __m128i vsrc;
      __m128i vtr;
      __m128i vtmp;
      __m128i vcur = vzero;

      for( int i = 0; i < zeroOutSize; i += 8, srcPtr += 8, trMatTmp += 8 )
      {
        vsrc = _mm_loadu_si128( ( const __m128i* ) srcPtr );
        vtr  = _mm_loadu_si64( ( const __m128i* ) trMatTmp );
        vtr  = _mm_cvtepi8_epi16( vtr );
        vtmp = _mm_cvtepi16_epi32( vtr );

        vtmp = _mm_mullo_epi32( vsrc, vtmp );
        vcur = _mm_add_epi32( vtmp, vcur );

        vsrc = _mm_loadu_si128( ( const __m128i* ) &srcPtr[4] );
        vtmp = _mm_cvtepi16_epi32( _mm_unpackhi_epi64( vtr, vzero ) );
      
        vtmp = _mm_mullo_epi32( vsrc, vtmp );
        vcur = _mm_add_epi32( vtmp, vcur );
      }

      vsum[k] = vcur;
    }

    __m128i vout = _mm_hadd_epi32( _mm_hadd_epi32( vsum[0], vsum[1] ), _mm_hadd_epi32( vsum[2], vsum[3] ) );
    vout = _mm_add_epi32( vout, _mm_set1_epi32( 64 ) );
    vout = _mm_srai_epi32( vout, 7 );
    vout = _mm_min_epi32( _mm_max_epi32( vmin, vout ), vmax );

    _mm_storeu_si128( ( __m128i* ) out, vout );
  }
}

template<X86_VEXT vext>
void TCoeffOps::_initTCoeffOpsX86()
{
  cpyResiClip[2] = cpyResiClip_SSE<vext,  4>;
  cpyResiClip[3] = cpyResiClip_SSE<vext,  8>;
  cpyResiClip[4] = cpyResiClip_SSE<vext, 16>;
  cpyResiClip[5] = cpyResiClip_SSE<vext, 32>;
  cpyResiClip[6] = cpyResiClip_SSE<vext, 64>;
  roundClip4     = roundClip_SSE<vext,  4>;
  roundClip8     = roundClip_SSE<vext,  8>;
  fastInvCore[0] = fastInv_SSE  <vext,  4>;
  fastInvCore[1] = fastInv_SSE  <vext,  8>;
  fastInvCore[2] = fastInv_SSE  <vext, 16>;
  fastInvCore[3] = fastInv_SSE  <vext, 32>;
  fastInvCore[4] = fastInv_SSE  <vext, 64>;
}

template<X86_VEXT vext>
void TrQuant::_initTrQuantX86()
{
  m_invLfnstNxN = simdInvLfnstNxNCore<vext>;
}

template void TCoeffOps::_initTCoeffOpsX86<SIMDX86>();
template void TrQuant::_initTrQuantX86<SIMDX86>();

#endif // TARGET_SIMD_X86
#endif

}
