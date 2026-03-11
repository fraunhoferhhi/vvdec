/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/** \file     RdCost_neon.cpp
    \brief    RD cost computation class, Neon version
*/

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/RdCost.h"
#include "sum_neon.h"

namespace vvdec
{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_DIST

template<int iCols>
Distortion xGetSAD_MxN_neon( const DistParam& rcDtParam )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int iRows = rcDtParam.org.height;
  constexpr int iSubShift = 1;
  constexpr int iSubStep = 1 << iSubShift;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride * iSubStep;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride * iSubStep;

  CHECKD( rcDtParam.subShift != 1, "Only SubShift = 1 is supported!" );
  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );
  CHECKD( iRows != 8 && iRows != 16, "Only iRows == 8 or iRows == 16 supported!" );

  Distortion uiSum = 0;
  if( iCols == 16 )
  {
    uint16x8_t sum_u16_lo = vdupq_n_u16( 0 );
    uint16x8_t sum_u16_hi = vdupq_n_u16( 0 );

    do
    {
      const int16x8_t org_lo = vld1q_s16( piOrg + 0 );
      const int16x8_t org_hi = vld1q_s16( piOrg + 8 );
      const int16x8_t cur_lo = vld1q_s16( piCur + 0 );
      const int16x8_t cur_hi = vld1q_s16( piCur + 8 );

      sum_u16_lo = vvdec_vabaq_s16( sum_u16_lo, org_lo, cur_lo );
      sum_u16_hi = vvdec_vabaq_s16( sum_u16_hi, org_hi, cur_hi );

      piOrg += iStrideOrg;
      piCur += iStrideCur;
      iRows -= iSubStep;
    } while( iRows != 0 );

    uiSum = horizontal_add_long_u16x8( vaddq_u16( sum_u16_lo, sum_u16_hi ) );
  }
  else // iCols == 8
  {
    uint16x8_t sum_u16[2] = { vdupq_n_u16( 0 ), vdupq_n_u16( 0 ) };
    do
    {
      const int16x8_t org0 = vld1q_s16( piOrg );
      const int16x8_t cur0 = vld1q_s16( piCur );
      const int16x8_t org1 = vld1q_s16( piOrg + iStrideOrg );
      const int16x8_t cur1 = vld1q_s16( piCur + iStrideOrg );

      sum_u16[0] = vvdec_vabaq_s16( sum_u16[0], org0, cur0 );
      sum_u16[1] = vvdec_vabaq_s16( sum_u16[1], org1, cur1 );

      piOrg += 2 * iStrideOrg;
      piCur += 2 * iStrideCur;
      iRows -= 2 * iSubStep;
    } while( iRows != 0 );

    uiSum = horizontal_add_long_u16x8( vaddq_u16( sum_u16[0], sum_u16[1] ) );
  }

  uiSum <<= iSubShift;
  return uiSum;
}

#if __ARM_ARCH >= 8

template<ARM_VEXT vext, bool isCalCentrePos>
void xGetSADX5_16xN_SIMDImp( const DistParam& rcDtParam, Distortion* cost )
{
  int i, j;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf - 4;
  int height = rcDtParam.org.height;
  int iSubShift = rcDtParam.subShift;
  int iSubStep = ( 1 << iSubShift );
  ptrdiff_t iStrideCur = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t iStrideOrg = rcDtParam.org.stride * iSubStep;

  int16x8_t sum0 = vdupq_n_s16( 0 );
  int16x8_t sum1 = vdupq_n_s16( 0 );
  int16x8_t sum2 = vdupq_n_s16( 0 );
  int16x8_t sum3 = vdupq_n_s16( 0 );
  int16x8_t sum4 = vdupq_n_s16( 0 );

  for( i = 0; i < height; i += iSubStep )
  {
    for( j = 0; j < 16; j += 8 )
    {
      int16x8_t s0 = vld1q_s16( piOrg + j + 0 );
      int16x8_t s1 = vld1q_s16( piCur + j + 0 );
      int16x8_t s2 = vcombine_s16( vld1_s16( piOrg + j + 8 ), vdup_n_s16( 0 ) );
      int16x8_t s3 = vcombine_s16( vld1_s16( piCur + j + 8 ), vdup_n_s16( 0 ) );

      int16x8_t org0, org1, org2, org3, org4;
      org0 = s0;
      org1 = vextq_s16( s0, s2, 1 );
      if( isCalCentrePos )
        org2 = vextq_s16( s0, s2, 2 );
      org3 = vextq_s16( s0, s2, 3 );
      org4 = vextq_s16( s0, s2, 4 );

      int16x8_t cur0, cur1, cur2, cur3, cur4;
      cur4 = s1;
      cur0 = vextq_s16( s1, s3, 4 );
      cur1 = vextq_s16( s1, s3, 3 );
      if( isCalCentrePos )
        cur2 = vextq_s16( s1, s3, 2 );
      cur3 = vextq_s16( s1, s3, 1 );

      sum0 = vabaq_s16( sum0, org0, cur0 );
      sum1 = vabaq_s16( sum1, org1, cur1 );
      if( isCalCentrePos )
        sum2 = vabaq_s16( sum2, org2, cur2 );
      sum3 = vabaq_s16( sum3, org3, cur3 );
      sum4 = vabaq_s16( sum4, org4, cur4 );
    }

    INCY( piOrg, iStrideOrg );
    INCY( piCur, iStrideCur );
  }

#if defined( _MSC_VER ) && !defined( __clang__ ) && !defined( __INTEL_COMPILER )
  int32x4_t sum = vdupq_n_s32( 0 );
  sum = vsetq_lane_s32( vaddlvq_s16( sum0 ), sum, 0 );
  sum = vsetq_lane_s32( vaddlvq_s16( sum1 ), sum, 1 );
  sum = vsetq_lane_s32( vaddlvq_s16( sum3 ), sum, 2 );
  sum = vsetq_lane_s32( vaddlvq_s16( sum4 ), sum, 3 );
#else
  int32x4_t sum = { vaddlvq_s16( sum0 ), vaddlvq_s16( sum1 ), vaddlvq_s16( sum3 ), vaddlvq_s16( sum4 ) };
#endif
  int32x4_t sumTwo;
  if( isCalCentrePos )
    sumTwo = vdupq_n_s32( vaddlvq_s16( sum2 ) );

  sum = vshlq_s32( sum, vdupq_n_s32( iSubShift ) );
  if( isCalCentrePos )
    sumTwo = vshlq_s32( sumTwo, vdupq_n_s32( iSubShift ) );

  sum = vshrq_n_s32( sum, 1 );
  if( isCalCentrePos )
    sumTwo = vshrq_n_s32( sumTwo, 1 );

  vst1q_lane_u64( ( uint64_t* )&cost[0], ( uint64x2_t )sum, 0 );
  if( isCalCentrePos )
    cost[2] = vgetq_lane_s32( sumTwo, 0 );
  vst1q_lane_u64( ( uint64_t* )&cost[3], ( uint64x2_t )sum, 1 );
}

template<ARM_VEXT vext>
void xGetSADX5_16xN_SIMD( const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos )
{
  if( rcDtParam.bitDepth > 10 )
  {
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }

  if( isCalCentrePos )
    xGetSADX5_16xN_SIMDImp<vext, true>( rcDtParam, cost );
  else
    xGetSADX5_16xN_SIMDImp<vext, false>( rcDtParam, cost );
}

#endif // !__ARM_ARCH >= 8

template<>
void RdCost::_initRdCostARM<NEON>()
{
  m_afpDistortFunc[DF_SAD8] = xGetSAD_MxN_neon<8>;
  m_afpDistortFunc[DF_SAD16] = xGetSAD_MxN_neon<16>;

#if __ARM_ARCH >= 8
  m_afpDistortFuncX5[DF_SAD16] = xGetSADX5_16xN_SIMD<NEON>;
#endif
}

#endif // TARGET_SIMD_ARM

} // namespace vvdec
