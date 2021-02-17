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

Copyright (c) 2018-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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

/** \file     RdCostX86.cpp
    \brief    RD cost computation class, SIMD version
*/

#include <math.h>
#include <limits>

#include "CommonDefX86.h"
#include "../RdCost.h"

#ifdef TARGET_SIMD_X86


template<X86_VEXT vext>
Distortion RdCost::xGetSAD_16xN_SIMD( const DistParam &rcDtParam )
{
  if( rcDtParam.bitDepth > 10 )
    return RdCost::xGetSAD( rcDtParam );

  //  assert( rcDtParam.iCols == iWidth);
  const short* pSrc1          = (const short*)rcDtParam.org.buf;
  const short* pSrc2          = (const short*)rcDtParam.cur.buf;
  const int  iRows            = rcDtParam.org.height;
  const int  iSubShift        = rcDtParam.subShift;
  const ptrdiff_t iStrideSrc1 = rcDtParam.org.stride << iSubShift;
  const ptrdiff_t iStrideSrc2 = rcDtParam.cur.stride << iSubShift;

  uint32_t uiSum = 0;

  if( vext >= AVX2 )
  {
#ifdef USE_AVX2
    __m256i vone   = _mm256_set1_epi16( 1 );
    __m256i vsum32 = _mm256_setzero_si256();

    if( iRows == 16 && iSubShift == 1 )
    {
      if( rcDtParam.bitDepth <= 12 )
      {
        for( int i = 0; i < 2; i++ )
        {
          __m256i vsum16 = _mm256_setzero_si256();

          //0
          __m256i vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) );
          __m256i vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );

          pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

          // 2x 12bit abs-diff -> 12 bit 
          vsum16 = _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) );

          // 1
          vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );
          pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
          // 2x 12bit sum -> 13 bit
          vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );

          // 2
          vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );
          pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
          // 13 bit and 12bit sum, or 3x 12 bit sum -> 13 bit
          vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );

          // 3
          vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );
          pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
          // 4x 12 bit sum -> 14 bit
          vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );

          vsum32 = _mm256_add_epi32( vsum32, _mm256_madd_epi16( vsum16, vone ) );
        }
      }
      else
      {
        for( int i = 0; i < 2; i++ )
        {
          //0
          __m256i vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) );
          __m256i vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );

          pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

          vsum32 = _mm256_add_epi32( vsum32, _mm256_madd_epi16( _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ), vone ) );

          // 1
          vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) ); pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
          vsum32 = _mm256_add_epi32( vsum32, _mm256_madd_epi16( _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ), vone ) );

          // 2
          vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) ); pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
          vsum32 = _mm256_add_epi32( vsum32, _mm256_madd_epi16( _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ), vone ) );

          // 3
          vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) ); pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
          vsum32 = _mm256_add_epi32( vsum32, _mm256_madd_epi16( _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ), vone ) );
        }
      }
    }
    else
    {
      // Do for width that multiple of 16
      for( int iY = 0; iY < iRows; iY += ( 1 << iSubShift ) )
      {
        __m256i vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) );
        __m256i vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );

        vsum32 = _mm256_add_epi32( vsum32, _mm256_madd_epi16( _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ), vone ) );

        pSrc1 += iStrideSrc1;
        pSrc2 += iStrideSrc2;
      }
    }
    vsum32 = _mm256_hadd_epi32( vsum32, vone );
    vsum32 = _mm256_hadd_epi32( vsum32, vone );
    uiSum =  _mm_cvtsi128_si32( _mm256_castsi256_si128( vsum32 ) ) + _mm_cvtsi128_si32( _mm256_extracti128_si256( vsum32, 1 ) );
#endif
  }
  else
  {
    // For width that multiple of 8
    __m128i vone = _mm_set1_epi16( 1 );
    __m128i vsum32 = _mm_setzero_si128();
    for( int iY = 0; iY < iRows; iY+= ( 1 << iSubShift ) )
    {
      __m128i vsum16 = _mm_setzero_si128();
      for( int iX = 0; iX < 16; iX+=8 )
      {
        __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* )( &pSrc1[iX] ) );
        __m128i vsrc2 = _mm_lddqu_si128( ( const __m128i* )( &pSrc2[iX] ) );
        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
      }
      __m128i vsumtemp = _mm_madd_epi16( vsum16, vone );
      vsum32 = _mm_add_epi32( vsum32, vsumtemp );
      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;
    }
    vsum32 = _mm_hadd_epi32( vsum32, vone );
    vsum32 = _mm_hadd_epi32( vsum32, vone );
    uiSum =  _mm_cvtsi128_si32( vsum32 );
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template< int iWidth, X86_VEXT vext >
Distortion RdCost::xGetSAD_NxN_SIMD( const DistParam &rcDtParam )
{
  if( rcDtParam.bitDepth > 10 )
    return RdCost::xGetSAD( rcDtParam );

  //  assert( rcDtParam.iCols == iWidth);
  const short* pSrc1   = (const short*)rcDtParam.org.buf;
  const short* pSrc2   = (const short*)rcDtParam.cur.buf;
  int  iRows           = rcDtParam.org.height;
  int  iSubShift       = rcDtParam.subShift;
  int  iSubStep        = ( 1 << iSubShift );
  const ptrdiff_t iStrideSrc1 = rcDtParam.org.stride * iSubStep;
  const ptrdiff_t iStrideSrc2 = rcDtParam.cur.stride * iSubStep;

  uint32_t uiSum = 0;

  if( iWidth == 4 )
  {
    if( iRows == 4 && iSubShift == 0 )
    {
      __m128i vzero = _mm_setzero_si128();  
      __m128i vsrc1 = _mm_or_si128( _mm_loadl_epi64( ( const __m128i* )pSrc1 ), _mm_slli_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc1[iStrideSrc1] ) ), 8 ) );
      __m128i vsrc2 = _mm_or_si128( _mm_loadl_epi64( ( const __m128i* )pSrc2 ), _mm_slli_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc2[iStrideSrc2] ) ), 8 ) );
      __m128i vsum  = _mm_cvtepu16_epi32( _mm_hadd_epi16( _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ), vzero ) );

      vsrc1 = _mm_or_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc1[2 * iStrideSrc1] ) ), _mm_slli_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc1[3 * iStrideSrc1] ) ), 8 ) );
      vsrc2 = _mm_or_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc2[2 * iStrideSrc2] ) ), _mm_slli_si128( _mm_loadl_epi64( ( const __m128i* )( &pSrc2[3 * iStrideSrc2] ) ), 8 ) );
      vsum  = _mm_add_epi32( vsum, _mm_cvtepu16_epi32( _mm_hadd_epi16( _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ), vzero ) ) );
      vsum  = _mm_hadd_epi32( vsum, vzero );
      vsum  = _mm_hadd_epi32( vsum, vzero );

      uiSum = _mm_cvtsi128_si32( vsum );
    }
    else
    {
      __m128i vone = _mm_set1_epi16( 1 );
      __m128i vsum32 = _mm_setzero_si128();
      for( int iY = 0; iY < iRows; iY += iSubStep )
      {
        __m128i vsumtemp = _mm_setzero_si128();
        {
          __m128i vsrc1 = _mm_cvtepu16_epi32( _mm_loadl_epi64( ( const __m128i* )pSrc1 ) );
          __m128i vsrc2 = _mm_cvtepu16_epi32( _mm_loadl_epi64( ( const __m128i* )pSrc2 ) );
          vsumtemp = _mm_add_epi32( vsumtemp, _mm_abs_epi32( _mm_sub_epi32( vsrc1, vsrc2 ) ) );
        }
        vsum32 = _mm_add_epi32( vsum32, vsumtemp );
        pSrc1 += iStrideSrc1;
        pSrc2 += iStrideSrc2;
      }
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      uiSum = _mm_cvtsi128_si32( vsum32 );
    }
  }
  else
  {
    if( vext >= AVX2 && iWidth >= 16 )
    {
#ifdef USE_AVX2
      // Do for width that multiple of 16
      __m256i vone   = _mm256_set1_epi16( 1 );
      __m256i vsum32 = _mm256_setzero_si256();
      for( int iY = 0; iY < iRows; iY+=iSubStep )
      {
        __m256i vsum16 = _mm256_setzero_si256();
        for( int iX = 0; iX < iWidth; iX+=16 )
        {
          __m256i vsrc1 = _mm256_loadu_si256( ( __m256i* )( &pSrc1[iX] ) );
          __m256i vsrc2 = _mm256_loadu_si256( ( __m256i* )( &pSrc2[iX] ) );
          vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );
        }
        __m256i vsumtemp = _mm256_madd_epi16( vsum16, vone );
        vsum32 = _mm256_add_epi32( vsum32, vsumtemp );
        pSrc1   += iStrideSrc1;
        pSrc2   += iStrideSrc2;
      }
      vsum32 = _mm256_hadd_epi32( vsum32, vone );
      vsum32 = _mm256_hadd_epi32( vsum32, vone );
      uiSum =  _mm_cvtsi128_si32( _mm256_castsi256_si128( vsum32 ) ) + _mm_cvtsi128_si32( _mm256_extracti128_si256( vsum32, 1 ) );
#endif
    }
    else
    {
      // For width that multiple of 8
      __m128i vone = _mm_set1_epi16( 1 );
      __m128i vsum32 = _mm_setzero_si128();
      for( int iY = 0; iY < iRows; iY+=iSubStep )
      {
        __m128i vsum16 = _mm_setzero_si128();
        for( int iX = 0; iX < iWidth; iX+=8 )
        {
          __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* )( &pSrc1[iX] ) );
          __m128i vsrc2 = _mm_lddqu_si128( ( const __m128i* )( &pSrc2[iX] ) );
          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
        }
        __m128i vsumtemp = _mm_madd_epi16( vsum16, vone );
        vsum32 = _mm_add_epi32( vsum32, vsumtemp );
        pSrc1   += iStrideSrc1;
        pSrc2   += iStrideSrc2;
      }
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      vsum32 = _mm_hadd_epi32( vsum32, vone );
      uiSum =  _mm_cvtsi128_si32( vsum32 );
    }
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template <X86_VEXT vext>
void RdCost::_initRdCostX86()
{
  m_afpDistortFunc[DF_SAD4   ] = xGetSAD_NxN_SIMD<4,  vext>;
  m_afpDistortFunc[DF_SAD8   ] = xGetSAD_NxN_SIMD<8,  vext>;
  m_afpDistortFunc[DF_SAD16  ] = xGetSAD_16xN_SIMD   <vext>;
  m_afpDistortFunc[DF_SAD32  ] = xGetSAD_NxN_SIMD<32, vext>;
  m_afpDistortFunc[DF_SAD64  ] = xGetSAD_NxN_SIMD<64, vext>;
  m_afpDistortFunc[DF_SAD16N ] = xGetSAD_NxN_SIMD<16, vext>;
}

template void RdCost::_initRdCostX86<SIMDX86>();

#endif //#if TARGET_SIMD_X86
//! \}
