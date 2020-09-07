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
