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
#include "mem_neon.h"
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

template<bool isCalCentrePos>
void xGetSADX5_16xN_neon_impl( const DistParam& rcDtParam, Distortion* cost )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf - 4;
  int height = rcDtParam.org.height;
  constexpr int iSubShift = 1;
  constexpr int iSubStep = 1 << iSubShift;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride * iSubStep;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride * iSubStep;

  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );
  CHECKD( height <= 0, "Height cannot be <= 0!" );
  CHECKD( rcDtParam.subShift != 1, "Only SubShift = 1 is supported!" );

  int16x8_t sum0 = vdupq_n_s16( 0 );
  int16x8_t sum1 = vdupq_n_s16( 0 );
  int16x8_t sum2 = vdupq_n_s16( 0 );
  int16x8_t sum3 = vdupq_n_s16( 0 );
  int16x8_t sum4 = vdupq_n_s16( 0 );

  do
  {
    int16x8_t org0 = vld1q_s16( piOrg + 0 );
    int16x8_t org1 = vld1q_s16( piOrg + 1 );
    int16x8_t org3 = vld1q_s16( piOrg + 3 );
    int16x8_t org4 = vld1q_s16( piOrg + 4 );

    int16x8_t org8 = vld1q_s16( piOrg + 8 );
    int16x8_t org9 = vld1q_s16( piOrg + 9 );
    int16x8_t org11 = vld1q_s16( piOrg + 11 );
    int16x8_t org12 = vld1q_s16( piOrg + 12 );

    int16x8_t cur0 = vld1q_s16( piCur + 0 );
    int16x8_t cur1 = vld1q_s16( piCur + 1 );
    int16x8_t cur3 = vld1q_s16( piCur + 3 );
    int16x8_t cur4 = vld1q_s16( piCur + 4 );

    int16x8_t cur8 = vld1q_s16( piCur + 8 );
    int16x8_t cur9 = vld1q_s16( piCur + 9 );
    int16x8_t cur11 = vld1q_s16( piCur + 11 );
    int16x8_t cur12 = vld1q_s16( piCur + 12 );

    sum0 = vabaq_s16( sum0, org0, cur4 );
    sum0 = vabaq_s16( sum0, org8, cur12 );

    sum1 = vabaq_s16( sum1, org1, cur3 );
    sum1 = vabaq_s16( sum1, org9, cur11 );

    if( isCalCentrePos )
    {
      int16x8_t org2 = vld1q_s16( piOrg + 2 );
      int16x8_t cur2 = vld1q_s16( piCur + 2 );

      int16x8_t org10 = vld1q_s16( piOrg + 10 );
      int16x8_t cur10 = vld1q_s16( piCur + 10 );

      sum2 = vabaq_s16( sum2, org2, cur2 );
      sum2 = vabaq_s16( sum2, org10, cur10 );
    }
    sum3 = vabaq_s16( sum3, org3, cur1 );
    sum3 = vabaq_s16( sum3, org11, cur9 );

    sum4 = vabaq_s16( sum4, org4, cur0 );
    sum4 = vabaq_s16( sum4, org12, cur8 );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
    height -= iSubStep;
  } while( height != 0 );

  int32x4_t sum = horizontal_add_long_4d_s16x8( sum0, sum1, sum3, sum4 );

  cost[0] = vgetq_lane_s32( sum, 0 );
  cost[1] = vgetq_lane_s32( sum, 1 );
  if( isCalCentrePos )
  {
    cost[2] = horizontal_add_long_s16x8( sum2 );
  }
  cost[3] = vgetq_lane_s32( sum, 2 );
  cost[4] = vgetq_lane_s32( sum, 3 );
}

void xGetSADX5_16xN_neon( const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos )
{
  if( rcDtParam.bitDepth > 10 )
  {
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }

  if( isCalCentrePos )
  {
    xGetSADX5_16xN_neon_impl<true>( rcDtParam, cost );
  }
  else
  {
    xGetSADX5_16xN_neon_impl<false>( rcDtParam, cost );
  }
}

template<>
void RdCost::_initRdCostARM<NEON>()
{
  m_afpDistortFunc[DF_SAD8] = xGetSAD_MxN_neon<8>;
  m_afpDistortFunc[DF_SAD16] = xGetSAD_MxN_neon<16>;

  m_afpDistortFuncX5[DF_SAD16] = xGetSADX5_16xN_neon;
}

#endif // TARGET_SIMD_ARM

} // namespace vvdec
