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

/** \file     RdCostX86.cpp
    \brief    RD cost computation class, SIMD version
*/

#include <math.h>
#include <limits>

#include "CommonDefX86.h"
#include "../RdCost.h"

namespace vvdec
{
using namespace x86_simd;

#ifdef TARGET_SIMD_X86

template<X86_VEXT vext, bool isWdt16>
Distortion xGetSAD_MxN_SIMD( const DistParam &rcDtParam )
{
  if( rcDtParam.bitDepth > 10 )
    return isWdt16 ? RdCost::xGetSAD16( rcDtParam ) : RdCost::xGetSAD8( rcDtParam );

  //  assert( rcDtParam.iCols == iWidth);
  const short* pSrc1          = (const short*)rcDtParam.org.buf;
  const short* pSrc2          = (const short*)rcDtParam.cur.buf;
  const int  iRows            = rcDtParam.org.height;
  const int  iSubShift        = rcDtParam.subShift;
  const ptrdiff_t iStrideSrc1 = rcDtParam.org.stride << iSubShift;
  const ptrdiff_t iStrideSrc2 = rcDtParam.cur.stride << iSubShift;

  uint32_t uiSum = 0;

  if( vext >= AVX2 && isWdt16 )
  {
#ifdef USE_AVX2
    __m256i vone   = _mm256_set1_epi16( 1 );
    __m256i vsum32 = _mm256_setzero_si256();
    __m256i vsum16 = _mm256_setzero_si256();

    // sum of 8 unsigned 10-bit ints (0-1023) can maximally be 3 + 10 bits, i.e. fits into 16 bit

    for( int i = 0; i < ( iRows >> 3 ); i++ )
    {
      //0
      __m256i vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) );
      __m256i vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );

      pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
      vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );

      // 1
      vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );
      pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
      vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );

      // 2
      vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );
      pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
      vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );

      // 3
      vsrc1 = _mm256_loadu_si256( ( __m256i* )( pSrc1 ) ); vsrc2 = _mm256_loadu_si256( ( __m256i* )( pSrc2 ) );
      pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
      vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );
    }


    vsum32 = _mm256_madd_epi16( vsum16, vone );
    vsum32 = _mm256_hadd_epi32( vsum32, vone );
    vsum32 = _mm256_hadd_epi32( vsum32, vone );
    uiSum =  _mm_cvtsi128_si32( _mm256_castsi256_si128( vsum32 ) ) + _mm_cvtsi128_si32( _mm256_extracti128_si256( vsum32, 1 ) );
#endif
  }
  else
  {
    __m128i vone   = _mm_set1_epi16( 1 );
    __m128i vsum32 = _mm_setzero_si128();
    __m128i vsum16 = _mm_setzero_si128();

    // sum of 16 unsigned 10-bit ints (0-1023) can maximally be 4 + 10 bits, i.e. fits into 16 bit

    for( int i = 0; i < ( iRows >> 3 ); i++ )
    {
      //0
      __m128i vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1) );
      __m128i vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2) );

      vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );

      if( isWdt16 )
      {
        vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1 + 8) );
        vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2 + 8) );

        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
      }

      pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

      // 1
      vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1) );
      vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2) );

      vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );

      if( isWdt16 )
      {
        vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1 + 8) );
        vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2 + 8) );

        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
      }

      pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

      // 2
      vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1) );
      vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2) );

      vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );

      if( isWdt16 )
      {
        vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1 + 8) );
        vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2 + 8) );

        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
      }

      pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;

      // 3
      vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1) );
      vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2) );

      vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );

      if( isWdt16 )
      {
        vsrc1 = _mm_loadu_si128( (const __m128i*)(pSrc1 + 8) );
        vsrc2 = _mm_loadu_si128( (const __m128i*)(pSrc2 + 8) );

        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
      }

      pSrc1 += iStrideSrc1; pSrc2 += iStrideSrc2;
    }


    vsum32 = _mm_madd_epi16( vsum16, vone );
    vsum32 = _mm_hadd_epi32( vsum32, vone );
    vsum32 = _mm_hadd_epi32( vsum32, vone );
    uiSum = _mm_cvtsi128_si32( vsum32 );
  }

  uiSum <<= iSubShift;
  return uiSum;
}

template <X86_VEXT vext, bool isCalCentrePos>
void xGetSADX5_8xN_SIMDImp(const DistParam& rcDtParam, Distortion* cost) {
  int i;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf - 4;
  int height = rcDtParam.org.height;
  int iSubShift = rcDtParam.subShift;
  int iSubStep = (1 << iSubShift);
  ptrdiff_t iStrideCur = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t iStrideOrg = rcDtParam.org.stride * iSubStep;

  __m128i sum0 = _mm_setzero_si128();
  __m128i sum1 = _mm_setzero_si128();
  __m128i sum2 = _mm_setzero_si128();
  __m128i sum3 = _mm_setzero_si128();
  __m128i sum4 = _mm_setzero_si128();

  __m128i vone = _mm_set1_epi16(1);
  for (i = 0; i < height; i += iSubStep) {
    __m128i s0 = _mm_loadu_si128((__m128i*)piOrg);
    __m128i s1 = _mm_loadu_si128((__m128i*)piCur);
    __m128i s2 = _mm_loadu_si64 ((__m128i*)(piOrg + 8));
    __m128i s3 = _mm_loadu_si64 ((__m128i*)(piCur + 8));

    __m128i org0, org1, org2, org3, org4;
    org0 = s0;
    org1 = _mm_alignr_epi8(s2, s0, 2);
    if (isCalCentrePos) org2 = _mm_alignr_epi8(s2, s0, 4);
    org3 = _mm_alignr_epi8(s2, s0, 6);
    org4 = _mm_alignr_epi8(s2, s0, 8);

    __m128i cur0, cur1, cur2, cur3, cur4;
    cur4 = s1;
    cur0 = _mm_alignr_epi8(s3, s1, 8);
    cur1 = _mm_alignr_epi8(s3, s1, 6);
    if (isCalCentrePos) cur2 = _mm_alignr_epi8(s3, s1, 4);
    cur3 = _mm_alignr_epi8(s3, s1, 2);

    __m128i diff0, diff1, diff2, diff3, diff4;
    diff0 = _mm_sub_epi16(org0, cur0);
    diff1 = _mm_sub_epi16(org1, cur1);
    if (isCalCentrePos) diff2 = _mm_sub_epi16(org2, cur2);
    diff3 = _mm_sub_epi16(org3, cur3);
    diff4 = _mm_sub_epi16(org4, cur4);

    diff0 = _mm_abs_epi16(diff0);
    diff1 = _mm_abs_epi16(diff1);
    if (isCalCentrePos) diff2 = _mm_abs_epi16(diff2);
    diff3 = _mm_abs_epi16(diff3);
    diff4 = _mm_abs_epi16(diff4);

    sum0 = _mm_add_epi16(sum0, diff0);
    sum1 = _mm_add_epi16(sum1, diff1);
    if (isCalCentrePos) sum2 = _mm_add_epi32(sum2, diff2);
    sum3 = _mm_add_epi16(sum3, diff3);
    sum4 = _mm_add_epi16(sum4, diff4);

    INCY(piOrg, iStrideOrg);
    INCY(piCur, iStrideCur);
  }

  sum0 = _mm_madd_epi16( sum0, vone );
  sum1 = _mm_madd_epi16( sum1, vone );
  if( isCalCentrePos ) sum2 = _mm_madd_epi16( sum2, vone );
  sum3 = _mm_madd_epi16( sum3, vone );
  sum4 = _mm_madd_epi16( sum4, vone );

  sum0 = _mm_hadd_epi32(sum0, sum1);
  sum3 = _mm_hadd_epi32(sum3, sum4);
  if (isCalCentrePos) sum2 = _mm_hadd_epi32(sum2, sum2);

  sum0 = _mm_hadd_epi32(sum0, sum3);
  if (isCalCentrePos) sum2 = _mm_hadd_epi32(sum2, sum2);

  sum0 = _mm_sll_epi32(sum0, _mm_cvtsi32_si128(iSubShift));
  if (isCalCentrePos) sum2 = _mm_sll_epi32(sum2, _mm_cvtsi32_si128(iSubShift));

  sum0 = _mm_srli_epi32(sum0, 1);
  if (isCalCentrePos) sum2 = _mm_srli_epi32(sum2, 1);

  _mm_storeu_si64( ( __m128i* ) &cost[0], sum0 );
  if (isCalCentrePos) cost[2] = (_mm_cvtsi128_si32(sum2));
  _mm_storeu_si64( ( __m128i* ) &cost[3], _mm_unpackhi_epi64( sum0, sum0 ) );
}

template <X86_VEXT vext>
void xGetSADX5_8xN_SIMD(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos) {
  if( rcDtParam.bitDepth > 10 ){
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }

  if (isCalCentrePos)
    xGetSADX5_8xN_SIMDImp<vext, true>(rcDtParam, cost);
  else
    xGetSADX5_8xN_SIMDImp<vext, false>(rcDtParam, cost);
}

template <X86_VEXT vext, bool isCalCentrePos>
void xGetSADX5_16xN_SIMDImp(const DistParam& rcDtParam, Distortion* cost) {
  int i, j;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf - 4;
  int height = rcDtParam.org.height;
  int iSubShift = rcDtParam.subShift;
  int iSubStep = (1 << iSubShift);
  ptrdiff_t iStrideCur = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t iStrideOrg = rcDtParam.org.stride * iSubStep;

#  ifdef USE_AVX2
  if (vext >= AVX2) {
    // sum of 8 unsigned 10-bit ints (0-1023) can maximally be 3 + 10 bits, i.e. fits into 16 bit

    __m256i sum0 = _mm256_setzero_si256();
    __m256i sum1 = _mm256_setzero_si256();
    __m256i sum2 = _mm256_setzero_si256();
    __m256i sum3 = _mm256_setzero_si256();
    __m256i sum4 = _mm256_setzero_si256();

    __m256i vone = _mm256_set1_epi16(1);

    for (int i = 0; i < ( height >> 3 ); i++) {
      __m256i s0 = _mm256_loadu_si256((__m256i*)piOrg);
      __m256i s1 = _mm256_loadu_si256((__m256i*)piCur);
      __m256i s2 = _mm256_castsi128_si256(_mm_loadu_si64((__m128i*)(piOrg + 16)));
      __m256i s3 = _mm256_castsi128_si256(_mm_loadu_si64((__m128i*)(piCur + 16)));
      s2 = _mm256_permute2x128_si256(s0, s2, 0x21);
      s3 = _mm256_permute2x128_si256(s1, s3, 0x21);

      INCY(piOrg, iStrideOrg);
      INCY(piCur, iStrideCur);

      __m256i org0, org1, org2, org3, org4;
      org0 = s0;
      org1 = _mm256_alignr_epi8(s2, s0, 2);
      if (isCalCentrePos) org2 = _mm256_alignr_epi8(s2, s0, 4);
      org3 = _mm256_alignr_epi8(s2, s0, 6);
      org4 = _mm256_alignr_epi8(s2, s0, 8);

      __m256i cur0, cur1, cur2, cur3, cur4;
      cur4 = s1;
      cur0 = _mm256_alignr_epi8(s3, s1, 8);
      cur1 = _mm256_alignr_epi8(s3, s1, 6);
      if (isCalCentrePos) cur2 = _mm256_alignr_epi8(s3, s1, 4);
      cur3 = _mm256_alignr_epi8(s3, s1, 2);

      __m256i diff0, diff1, diff2, diff3, diff4;
      diff0 = _mm256_sub_epi16(org0, cur0);
      diff1 = _mm256_sub_epi16(org1, cur1);
      if (isCalCentrePos) diff2 = _mm256_sub_epi16(org2, cur2);
      diff3 = _mm256_sub_epi16(org3, cur3);
      diff4 = _mm256_sub_epi16(org4, cur4);

      diff0 = _mm256_abs_epi16( diff0 );
      diff1 = _mm256_abs_epi16( diff1 );
      if( isCalCentrePos ) diff2 = _mm256_abs_epi16( diff2 );
      diff3 = _mm256_abs_epi16( diff3 );
      diff4 = _mm256_abs_epi16( diff4 );

      sum0 = _mm256_add_epi16( diff0, sum0 );
      sum1 = _mm256_add_epi16( diff1, sum1 );
      if( isCalCentrePos ) sum2 = _mm256_add_epi16( diff2, sum2 );
      sum3 = _mm256_add_epi16( diff3, sum3 );
      sum4 = _mm256_add_epi16( diff4, sum4 );

      s0 = _mm256_loadu_si256((__m256i*)piOrg);
      s1 = _mm256_loadu_si256((__m256i*)piCur);
      s2 = _mm256_castsi128_si256(_mm_loadu_si64((__m128i*)(piOrg + 16)));
      s3 = _mm256_castsi128_si256(_mm_loadu_si64((__m128i*)(piCur + 16)));
      s2 = _mm256_permute2x128_si256(s0, s2, 0x21);
      s3 = _mm256_permute2x128_si256(s1, s3, 0x21);

      INCY(piOrg, iStrideOrg);
      INCY(piCur, iStrideCur);

      org0 = s0;
      org1 = _mm256_alignr_epi8(s2, s0, 2);
      if (isCalCentrePos) org2 = _mm256_alignr_epi8(s2, s0, 4);
      org3 = _mm256_alignr_epi8(s2, s0, 6);
      org4 = _mm256_alignr_epi8(s2, s0, 8);

      cur4 = s1;
      cur0 = _mm256_alignr_epi8(s3, s1, 8);
      cur1 = _mm256_alignr_epi8(s3, s1, 6);
      if (isCalCentrePos) cur2 = _mm256_alignr_epi8(s3, s1, 4);
      cur3 = _mm256_alignr_epi8(s3, s1, 2);

      diff0 = _mm256_sub_epi16(org0, cur0);
      diff1 = _mm256_sub_epi16(org1, cur1);
      if (isCalCentrePos) diff2 = _mm256_sub_epi16(org2, cur2);
      diff3 = _mm256_sub_epi16(org3, cur3);
      diff4 = _mm256_sub_epi16(org4, cur4);

      diff0 = _mm256_abs_epi16(diff0);
      diff1 = _mm256_abs_epi16(diff1);
      if (isCalCentrePos) diff2 = _mm256_abs_epi16(diff2);
      diff3 = _mm256_abs_epi16(diff3);
      diff4 = _mm256_abs_epi16(diff4);

      sum0 = _mm256_add_epi16(diff0, sum0);
      sum1 = _mm256_add_epi16(diff1, sum1);
      if (isCalCentrePos) sum2 = _mm256_add_epi16(diff2, sum2);
      sum3 = _mm256_add_epi16(diff3, sum3);
      sum4 = _mm256_add_epi16(diff4, sum4);

      s0 = _mm256_loadu_si256((__m256i*)piOrg);
      s1 = _mm256_loadu_si256((__m256i*)piCur);
      s2 = _mm256_castsi128_si256(_mm_loadu_si64((__m128i*)(piOrg + 16)));
      s3 = _mm256_castsi128_si256(_mm_loadu_si64((__m128i*)(piCur + 16)));
      s2 = _mm256_permute2x128_si256(s0, s2, 0x21);
      s3 = _mm256_permute2x128_si256(s1, s3, 0x21);

      INCY(piOrg, iStrideOrg);
      INCY(piCur, iStrideCur);

      org0 = s0;
      org1 = _mm256_alignr_epi8(s2, s0, 2);
      if (isCalCentrePos) org2 = _mm256_alignr_epi8(s2, s0, 4);
      org3 = _mm256_alignr_epi8(s2, s0, 6);
      org4 = _mm256_alignr_epi8(s2, s0, 8);

      cur4 = s1;
      cur0 = _mm256_alignr_epi8(s3, s1, 8);
      cur1 = _mm256_alignr_epi8(s3, s1, 6);
      if (isCalCentrePos) cur2 = _mm256_alignr_epi8(s3, s1, 4);
      cur3 = _mm256_alignr_epi8(s3, s1, 2);

      diff0 = _mm256_sub_epi16(org0, cur0);
      diff1 = _mm256_sub_epi16(org1, cur1);
      if (isCalCentrePos) diff2 = _mm256_sub_epi16(org2, cur2);
      diff3 = _mm256_sub_epi16(org3, cur3);
      diff4 = _mm256_sub_epi16(org4, cur4);

      diff0 = _mm256_abs_epi16(diff0);
      diff1 = _mm256_abs_epi16(diff1);
      if (isCalCentrePos) diff2 = _mm256_abs_epi16(diff2);
      diff3 = _mm256_abs_epi16(diff3);
      diff4 = _mm256_abs_epi16(diff4);

      sum0 = _mm256_add_epi16( diff0, sum0 );
      sum1 = _mm256_add_epi16( diff1, sum1 );
      if( isCalCentrePos ) sum2 = _mm256_add_epi16( diff2, sum2 );
      sum3 = _mm256_add_epi16( diff3, sum3 );
      sum4 = _mm256_add_epi16( diff4, sum4 );

      s0 = _mm256_loadu_si256((__m256i*)piOrg);
      s1 = _mm256_loadu_si256((__m256i*)piCur);
      s2 = _mm256_castsi128_si256(_mm_loadu_si64((__m128i*)(piOrg + 16)));
      s3 = _mm256_castsi128_si256(_mm_loadu_si64((__m128i*)(piCur + 16)));
      s2 = _mm256_permute2x128_si256(s0, s2, 0x21);
      s3 = _mm256_permute2x128_si256(s1, s3, 0x21);

      INCY(piOrg, iStrideOrg);
      INCY(piCur, iStrideCur);

      org0 = s0;
      org1 = _mm256_alignr_epi8(s2, s0, 2);
      if (isCalCentrePos) org2 = _mm256_alignr_epi8(s2, s0, 4);
      org3 = _mm256_alignr_epi8(s2, s0, 6);
      org4 = _mm256_alignr_epi8(s2, s0, 8);

      cur4 = s1;
      cur0 = _mm256_alignr_epi8(s3, s1, 8);
      cur1 = _mm256_alignr_epi8(s3, s1, 6);
      if (isCalCentrePos) cur2 = _mm256_alignr_epi8(s3, s1, 4);
      cur3 = _mm256_alignr_epi8(s3, s1, 2);

      diff0 = _mm256_sub_epi16(org0, cur0);
      diff1 = _mm256_sub_epi16(org1, cur1);
      if (isCalCentrePos) diff2 = _mm256_sub_epi16(org2, cur2);
      diff3 = _mm256_sub_epi16(org3, cur3);
      diff4 = _mm256_sub_epi16(org4, cur4);

      diff0 = _mm256_abs_epi16(diff0);
      diff1 = _mm256_abs_epi16(diff1);
      if (isCalCentrePos) diff2 = _mm256_abs_epi16(diff2);
      diff3 = _mm256_abs_epi16(diff3);
      diff4 = _mm256_abs_epi16(diff4);

      sum0 = _mm256_add_epi16(diff0, sum0);
      sum1 = _mm256_add_epi16(diff1, sum1);
      if (isCalCentrePos) sum2 = _mm256_add_epi16(diff2, sum2);
      sum3 = _mm256_add_epi16(diff3, sum3);
      sum4 = _mm256_add_epi16(diff4, sum4);
    }

    sum0 = _mm256_madd_epi16( sum0, vone );
    sum1 = _mm256_madd_epi16( sum1, vone );
    if( isCalCentrePos ) sum2 = _mm256_madd_epi16( sum2, vone );
    sum3 = _mm256_madd_epi16( sum3, vone );
    sum4 = _mm256_madd_epi16( sum4, vone );

    sum0 = _mm256_hadd_epi32(sum0, sum1);
    sum3 = _mm256_hadd_epi32(sum3, sum4);
    if (isCalCentrePos) sum2 = _mm256_hadd_epi32(sum2, sum2);

    sum0 = _mm256_hadd_epi32(sum0, sum3);
    if (isCalCentrePos) sum2 = _mm256_hadd_epi32(sum2, sum2);

    __m128i sum0134 = _mm_add_epi32(_mm256_castsi256_si128(sum0), _mm256_extracti128_si256(sum0, 1));

    sum0134 = _mm_sll_epi32(sum0134, _mm_cvtsi32_si128(iSubShift));

    sum0134 = _mm_srli_epi32(sum0134, 1);

    _mm_storeu_si64( ( __m128i* ) &cost[0], sum0134 );
    if (isCalCentrePos) {
      int tmp = _mm_cvtsi128_si32(_mm256_castsi256_si128(sum2)) + _mm256_extract_epi32(sum2, 4);
      tmp <<= iSubShift;
      tmp >>= 1;
      cost[2] = tmp;
    }
    _mm_storeu_si64( ( __m128i* ) &cost[3], _mm_unpackhi_epi64( sum0134, sum0134 ) );
  }
  else
#  endif
  {
    // sum of 16 unsigned 10-bit ints (0-1023) can maximally be 4 + 10 bits, i.e. fits into 16 bit

    __m128i sum0 = _mm_setzero_si128();
    __m128i sum1 = _mm_setzero_si128();
    __m128i sum2 = _mm_setzero_si128();
    __m128i sum3 = _mm_setzero_si128();
    __m128i sum4 = _mm_setzero_si128();

    __m128i vone = _mm_set1_epi16(1);
    for (i = 0; i < height; i += iSubStep) {
      for (j = 0; j < 16; j += 8) {
        __m128i s0 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(piOrg + j + 0));
        __m128i s1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(piCur + j + 0));
        __m128i s2 = _mm_loadu_si64 (reinterpret_cast<const __m128i*>(piOrg + j + 8));
        __m128i s3 = _mm_loadu_si64 (reinterpret_cast<const __m128i*>(piCur + j + 8));

        __m128i org0, org1, org2, org3, org4;
        org0 = s0;
        org1 = _mm_alignr_epi8(s2, s0, 2);
        if (isCalCentrePos) org2 = _mm_alignr_epi8(s2, s0, 4);
        org3 = _mm_alignr_epi8(s2, s0, 6);
        org4 = _mm_alignr_epi8(s2, s0, 8);

        __m128i cur0, cur1, cur2, cur3, cur4;
        cur4 = s1;
        cur0 = _mm_alignr_epi8(s3, s1, 8);
        cur1 = _mm_alignr_epi8(s3, s1, 6);
        if (isCalCentrePos) cur2 = _mm_alignr_epi8(s3, s1, 4);
        cur3 = _mm_alignr_epi8(s3, s1, 2);

        __m128i diff0, diff1, diff2, diff3, diff4;
        diff0 = _mm_sub_epi16(org0, cur0);
        diff1 = _mm_sub_epi16(org1, cur1);
        if (isCalCentrePos) diff2 = _mm_sub_epi16(org2, cur2);
        diff3 = _mm_sub_epi16(org3, cur3);
        diff4 = _mm_sub_epi16(org4, cur4);

        diff0 = _mm_abs_epi16(diff0);
        diff1 = _mm_abs_epi16(diff1);
        if (isCalCentrePos) diff2 = _mm_abs_epi16(diff2);
        diff3 = _mm_abs_epi16(diff3);
        diff4 = _mm_abs_epi16(diff4);

        sum0 = _mm_add_epi16(sum0, diff0);
        sum1 = _mm_add_epi16(sum1, diff1);
        if (isCalCentrePos) sum2 = _mm_add_epi16(sum2, diff2);
        sum3 = _mm_add_epi16(sum3, diff3);
        sum4 = _mm_add_epi16(sum4, diff4);
      }

      INCY(piOrg, iStrideOrg);
      INCY(piCur, iStrideCur);
    }

    sum0 = _mm_madd_epi16( sum0, vone );
    sum1 = _mm_madd_epi16( sum1, vone );
    if( isCalCentrePos ) sum2 = _mm_madd_epi16( sum2, vone );
    sum3 = _mm_madd_epi16( sum3, vone );
    sum4 = _mm_madd_epi16( sum4, vone );

    sum0 = _mm_hadd_epi32(sum0, sum1);
    sum3 = _mm_hadd_epi32(sum3, sum4);
    if (isCalCentrePos) sum2 = _mm_hadd_epi32(sum2, sum2);

    sum0 = _mm_hadd_epi32(sum0, sum3);
    if (isCalCentrePos) sum2 = _mm_hadd_epi32(sum2, sum2);

    sum0 = _mm_sll_epi32(sum0, _mm_cvtsi32_si128(iSubShift));
    if (isCalCentrePos) sum2 = _mm_sll_epi32(sum2, _mm_cvtsi32_si128(iSubShift));

    sum0 = _mm_srli_epi32(sum0, 1);
    if (isCalCentrePos) sum2 = _mm_srli_epi32(sum2, 1);

    _mm_storeu_si64( ( __m128i* ) &cost[0], sum0 );
    if (isCalCentrePos) cost[2] = (_mm_cvtsi128_si32(sum2));
    _mm_storeu_si64( ( __m128i* ) &cost[3], _mm_unpackhi_epi64( sum0, sum0 ) );
  }
}

template <X86_VEXT vext>
void xGetSADX5_16xN_SIMD(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos) {
  if( rcDtParam.bitDepth > 10 ){
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }

  if (isCalCentrePos)
    xGetSADX5_16xN_SIMDImp<vext, true>(rcDtParam, cost);
  else
    xGetSADX5_16xN_SIMDImp<vext, false>(rcDtParam, cost);
}

template <X86_VEXT vext>
void RdCost::_initRdCostX86()
{
  m_afpDistortFunc[DF_SAD8   ] = xGetSAD_MxN_SIMD<vext, false>;
  m_afpDistortFunc[DF_SAD16  ] = xGetSAD_MxN_SIMD<vext, true>;

  m_afpDistortFuncX5[DF_SAD8] = xGetSADX5_8xN_SIMD<vext>;
  m_afpDistortFuncX5[DF_SAD16] = xGetSADX5_16xN_SIMD<vext>;
}

template void RdCost::_initRdCostX86<SIMDX86>();

#endif //#if TARGET_SIMD_X86

}
