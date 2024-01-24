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

/** \file     RdCost.cpp
    \brief    RD cost computation class
*/

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

#include "RdCost.h"

namespace vvdec
{

FpDistFunc RdCost::m_afpDistortFunc[DF_TOTAL_FUNCTIONS] = { nullptr, };
FpDistFuncX5 RdCost::m_afpDistortFuncX5[DF_TOTAL_FUNCTIONS] = { nullptr, };

RdCost::RdCost()
{
  m_afpDistortFunc[DF_SAD8   ] = RdCost::xGetSAD8;
  m_afpDistortFunc[DF_SAD16  ] = RdCost::xGetSAD16;
  
  m_afpDistortFuncX5[DF_SAD8   ] = RdCost::xGetSAD8X5;
  m_afpDistortFuncX5[DF_SAD16  ] = RdCost::xGetSAD16X5;

#if ENABLE_SIMD_OPT_DIST
#  ifdef TARGET_SIMD_X86
  initRdCostX86();
#  endif
#  ifdef TARGET_SIMD_ARM
  initRdCostARM();
#  endif
#endif
}

RdCost::~RdCost()
{
}

void RdCost::setDistParam( DistParam &rcDP, const Pel* pOrg, const Pel* piRefY, ptrdiff_t iOrgStride, ptrdiff_t iRefStride, int bitDepth, int width, int height, int subShiftMode )
{
  rcDP.bitDepth   = bitDepth;

  rcDP.org.buf    = pOrg;
  rcDP.org.stride = iOrgStride;
  rcDP.org.width  = width;
  rcDP.org.height = height;

  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;
  rcDP.cur.width  = width;
  rcDP.cur.height = height;

  rcDP.subShift   = subShiftMode;

  rcDP.distFunc   = m_afpDistortFunc[getLog2( width ) - 3];
  rcDP.distFuncX5 = m_afpDistortFuncX5[getLog2( width ) - 3];
}

// ====================================================================================================================
// Distortion functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xGetSAD8( const DistParam& rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  ptrdiff_t iStrideCur       = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return uiSum;
}

Distortion RdCost::xGetSAD16( const DistParam& rcDtParam )
{
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  ptrdiff_t iStrideCur       = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return uiSum;
}

void RdCost::xGetSAD8X5(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos) {
  DistParam rcDtParamTmp0 = rcDtParam;
  DistParam rcDtParamTmp1 = rcDtParam;
  rcDtParamTmp1.org.buf += 1;
  rcDtParamTmp1.cur.buf -= 1;
  DistParam rcDtParamTmp2 = rcDtParam;
  rcDtParamTmp2.org.buf += 2;
  rcDtParamTmp2.cur.buf -= 2;
  DistParam rcDtParamTmp3 = rcDtParam;
  rcDtParamTmp3.org.buf += 3;
  rcDtParamTmp3.cur.buf -= 3;
  DistParam rcDtParamTmp4 = rcDtParam;
  rcDtParamTmp4.org.buf += 4;
  rcDtParamTmp4.cur.buf -= 4;
  
  cost[0] = (RdCost::xGetSAD8(rcDtParamTmp0)) >> 1;
  cost[1] = (RdCost::xGetSAD8(rcDtParamTmp1)) >> 1;
  if (isCalCentrePos) cost[2] = (RdCost::xGetSAD8(rcDtParamTmp2)) >> 1;
  cost[3] = (RdCost::xGetSAD8(rcDtParamTmp3)) >> 1;
  cost[4] = (RdCost::xGetSAD8(rcDtParamTmp4)) >> 1;
}

void RdCost::xGetSAD16X5(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos) {
  DistParam rcDtParamTmp0 = rcDtParam;
  DistParam rcDtParamTmp1 = rcDtParam;
  rcDtParamTmp1.org.buf += 1;
  rcDtParamTmp1.cur.buf -= 1;
  DistParam rcDtParamTmp2 = rcDtParam;
  rcDtParamTmp2.org.buf += 2;
  rcDtParamTmp2.cur.buf -= 2;
  DistParam rcDtParamTmp3 = rcDtParam;
  rcDtParamTmp3.org.buf += 3;
  rcDtParamTmp3.cur.buf -= 3;
  DistParam rcDtParamTmp4 = rcDtParam;
  rcDtParamTmp4.org.buf += 4;
  rcDtParamTmp4.cur.buf -= 4;
  
  cost[0] = (RdCost::xGetSAD16(rcDtParamTmp0)) >> 1;
  cost[1] = (RdCost::xGetSAD16(rcDtParamTmp1)) >> 1;
  if (isCalCentrePos) cost[2] = (RdCost::xGetSAD16(rcDtParamTmp2)) >> 1;
  cost[3] = (RdCost::xGetSAD16(rcDtParamTmp3)) >> 1;
  cost[4] = (RdCost::xGetSAD16(rcDtParamTmp4)) >> 1;
}

}
