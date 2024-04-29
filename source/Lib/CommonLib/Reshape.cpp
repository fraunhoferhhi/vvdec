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

/** \file     Reshape.cpp
    \brief    common reshaper class
*/
#include "Reshape.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <UnitTools.h>
#include "CommonLib/TimeProfiler.h"

#if ENABLE_SIMD_OPT_BUFFER && defined( TARGET_SIMD_X86 )
#include <CommonDefX86.h>
#include <simde/x86/sse.h>
#endif

namespace vvdec
{

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

Reshape::Reshape()
{
  m_invLUT = nullptr;
  m_chromaScale = 1 << CSCALE_FP_PREC;
  m_vpduX = -1;
  m_vpduY = -1;
}

Reshape::~Reshape()
{
  destroy();
}

void  Reshape::createDec(int bitDepth)
{
  m_lumaBD = bitDepth;
  m_reshapeLUTSize = 1 << m_lumaBD;
  m_initCW = m_reshapeLUTSize / PIC_CODE_CW_BINS;
  if( !m_invLUT )
  {
    m_invLUT = ( Pel* ) xMalloc( Pel, m_reshapeLUTSize + 1 );
    memset( m_invLUT, 0, ( m_reshapeLUTSize + 1 ) * sizeof( Pel ) );
  }
  if (m_binCW.empty())
    m_binCW.resize(PIC_CODE_CW_BINS, 0);
  if (m_inputPivot.empty())
    m_inputPivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (m_fwdScaleCoef.empty())
    m_fwdScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
  if (m_invScaleCoef.empty())
    m_invScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
  if (m_reshapePivot.empty())
    m_reshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (m_chromaAdjHelpLUT.empty())
    m_chromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 1<<CSCALE_FP_PREC);
}

void  Reshape::destroy()
{
  xFree( m_invLUT );
  m_invLUT = nullptr;
}

void  Reshape::initSlice( int nalUnitLayerId, const PicHeader& picHeader, const VPS* vps )
{
  if( picHeader.getLmcsEnabledFlag() )
  {
    if( nalUnitLayerId != picHeader.getLmcsAPS()->getLayerId() )
    {
      for (int i = 0; vps && i < vps->getNumOutputLayerSets(); i++ )
      {
        bool isCurrLayerInOls = false;
        bool isRefLayerInOls = false;
        for( int j = vps->getNumLayersInOls(i) - 1; j >= 0; j-- )
        {
          if( vps->getLayerIdInOls(i, j) == nalUnitLayerId )
          {
            isCurrLayerInOls = true;
          }
          if( vps->getLayerIdInOls(i, j) == picHeader.getLmcsAPS()->getLayerId() )
          {
            isRefLayerInOls = true;
          }
        }
        CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
      }
    }

    const SliceReshapeInfo& sInfo = picHeader.getLmcsAPS()->getReshaperAPSInfo();
    m_sliceReshapeInfo.sliceReshaperEnableFlag       = true;
    m_sliceReshapeInfo.sliceReshaperModelPresentFlag = true;
    m_sliceReshapeInfo.enableChromaAdj               = picHeader.getLmcsChromaResidualScaleFlag();
    m_sliceReshapeInfo.reshaperModelMaxBinIdx        = sInfo.reshaperModelMaxBinIdx;
    m_sliceReshapeInfo.reshaperModelMinBinIdx        = sInfo.reshaperModelMinBinIdx;
    m_sliceReshapeInfo.maxNbitsNeededDeltaCW         = sInfo.maxNbitsNeededDeltaCW;
    m_sliceReshapeInfo.chrResScalingOffset           = sInfo.chrResScalingOffset;
    memcpy( m_sliceReshapeInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof( int ) * ( PIC_CODE_CW_BINS ) );
    constructReshaper();
  }
  else
  {
    m_sliceReshapeInfo.sliceReshaperEnableFlag       = false;
    m_sliceReshapeInfo.enableChromaAdj               = false;
    m_sliceReshapeInfo.sliceReshaperModelPresentFlag = false;
  }
  m_vpduX = -1;
  m_vpduY = -1;
}

bool Reshape::getCTUFlag( const Slice& slice ) const
{
  if( (slice.getSliceType() == I_SLICE) && m_sliceReshapeInfo.sliceReshaperEnableFlag )
  {
    return false;
  }
  else
  {
    return m_sliceReshapeInfo.sliceReshaperEnableFlag;
  }
}



/** compute chroma residuce scale for TU
* \param average luma pred of TU
* \return chroma residue scale
*/
int  Reshape::calculateChromaAdj(Pel avgLuma) const
{
  int iAdj = m_chromaAdjHelpLUT[getPWLIdxInv(avgLuma)];
  return(iAdj);
}

/** compute chroma residuce scale for TU
* \param average luma pred of TU
* \return chroma residue scale
*/
int  Reshape::calculateChromaAdjVpduNei(TransformUnit &tu, const Position pos)
{
  CodingStructure &cs = *tu.cu->cs;
  int xPos = pos.x;
  int yPos = pos.y;
  int ctuSize = cs.sps->getCTUSize();
  int numNeighbor = std::min(64, ctuSize);
  int numNeighborLog = getLog2(numNeighbor);
  if (ctuSize == 128)
  {
    xPos &= ~63;
    yPos &= ~63;
  }
  else
  {
    xPos &= ~( ctuSize - 1 );
    yPos &= ~( ctuSize - 1 );
  }

  if( isVPDUprocessed( xPos, yPos ) )
  {
    return getChromaScale();
  }
  else
  {
    setVPDULoc(xPos, yPos);
    Position topLeft(xPos, yPos);
    CodingUnit *topLeftLuma;
    const CodingUnit *cuAbove, *cuLeft;

    topLeftLuma = cs.getCU( topLeft, CHANNEL_TYPE_LUMA );
    cuAbove     = cs.getCURestricted( topLeftLuma->lumaPos().offset( 0, -1 ), *topLeftLuma, CHANNEL_TYPE_LUMA, topLeftLuma->ly() == yPos ? topLeftLuma : topLeftLuma->above );
    cuLeft      = cs.getCURestricted( topLeftLuma->lumaPos().offset( -1, 0 ), *topLeftLuma, CHANNEL_TYPE_LUMA, topLeftLuma->lx() == xPos ? topLeftLuma : topLeftLuma->left  );

    xPos = topLeftLuma->lumaPos().x;
    yPos = topLeftLuma->lumaPos().y;

    CompArea lumaArea = CompArea(COMPONENT_Y, topLeftLuma->lumaPos(), topLeftLuma->lumaSize());
    PelBuf piRecoY = cs.picture->getRecoBuf(lumaArea);
    ptrdiff_t strideY = piRecoY.stride;
    int chromaScale = (1 << CSCALE_FP_PREC);
    int lumaValue = -1;

    Pel* recSrc0 = piRecoY.bufAt(0, 0);
    const uint32_t picH = tu.cu->cs->picture->lheight();
    const uint32_t picW = tu.cu->cs->picture->lwidth();
    const Pel   valueDC = 1 << (tu.cu->sps->getBitDepth() - 1);
    int32_t recLuma = 0;
    int pelnum = 0;
    if (cuLeft != nullptr)
    {
      for (int i = 0; i < numNeighbor; i++)
      {
        int k = (yPos + i) >= picH ? (picH - yPos - 1) : i;
        recLuma += recSrc0[-1 + k * strideY];
        pelnum++;
      }
    }
    if (cuAbove != nullptr)
    {
      for (int i = 0; i < numNeighbor; i++)
      {
        int k = (xPos + i) >= picW ? (picW - xPos - 1) : i;
        recLuma += recSrc0[-strideY + k];
        pelnum++;
      }
    }
    if (pelnum == numNeighbor)
    {
      lumaValue = (recLuma + (1 << (numNeighborLog - 1))) >> numNeighborLog;
    }
    else if (pelnum == (numNeighbor << 1))
    {
      lumaValue = (recLuma + (1 << numNeighborLog)) >> (numNeighborLog + 1);
    }
    else
    {
      CHECK(pelnum != 0, "");
      lumaValue = valueDC;
    }
    chromaScale = calculateChromaAdj(lumaValue);
    setChromaScale(chromaScale);
    return(chromaScale);
  }
}
/** find inx of PWL for inverse mapping
* \param average luma pred of TU
* \return idx of PWL for inverse mapping
*/
int Reshape::getPWLIdxInv(int lumaVal) const
{
  int idxS = 0;
  for (idxS = m_sliceReshapeInfo.reshaperModelMinBinIdx; (idxS <= m_sliceReshapeInfo.reshaperModelMaxBinIdx); idxS++)
  {
    if (lumaVal < m_reshapePivot[idxS + 1])     break;
  }
  return std::min(idxS, PIC_CODE_CW_BINS-1);
}

/**
-copy Slice reshaper info structure
\param   tInfo describing the target Slice reshaper info structure
\param   sInfo describing the source Slice reshaper info structure
*/
void Reshape::copySliceReshaperInfo(SliceReshapeInfo& tInfo, SliceReshapeInfo& sInfo)
{
  tInfo.sliceReshaperModelPresentFlag = sInfo.sliceReshaperModelPresentFlag;
  if (sInfo.sliceReshaperModelPresentFlag)
  {
    tInfo.reshaperModelMaxBinIdx = sInfo.reshaperModelMaxBinIdx;
    tInfo.reshaperModelMinBinIdx = sInfo.reshaperModelMinBinIdx;
    memcpy(tInfo.reshaperModelBinCWDelta, sInfo.reshaperModelBinCWDelta, sizeof(int)*(PIC_CODE_CW_BINS));
    tInfo.maxNbitsNeededDeltaCW = sInfo.maxNbitsNeededDeltaCW;
    tInfo.chrResScalingOffset = sInfo.chrResScalingOffset;
  }
  tInfo.sliceReshaperEnableFlag = sInfo.sliceReshaperEnableFlag;
  if (sInfo.sliceReshaperEnableFlag)
    tInfo.enableChromaAdj = sInfo.enableChromaAdj;
  else
    tInfo.enableChromaAdj = 0;
}

/** Construct reshaper from syntax
* \param void
* \return void
*/
void Reshape::constructReshaper()
{
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen = m_reshapeLUTSize / PIC_CODE_CW_BINS;

  for (int i = 0; i < m_sliceReshapeInfo.reshaperModelMinBinIdx; i++)
    m_binCW[i] = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1; i < PIC_CODE_CW_BINS; i++)
    m_binCW[i] = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
    m_binCW[i] = (uint16_t)(m_sliceReshapeInfo.reshaperModelBinCWDelta[i] + (int)m_initCW);

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_reshapePivot[i + 1] = m_reshapePivot[i] + m_binCW[i];
    m_inputPivot[i + 1] = m_inputPivot[i] + m_initCW;
    m_fwdScaleCoef[i] = ((int32_t)m_binCW[i] * (1 << FP_PREC) + (1 << (getLog2(pwlFwdBinLen) - 1))) >> getLog2(pwlFwdBinLen);
    if (m_binCW[i] == 0)
    {
      m_invScaleCoef[i] = 0;
      m_chromaAdjHelpLUT[i] = 1 << CSCALE_FP_PREC;
    }
    else
    {
      CHECK( m_initCW * (1 << FP_PREC) / m_binCW[i] > (1 << 15) - 1, "Inverse scale coeff doesn't fit in a short!" );
      m_invScaleCoef[i] = (int32_t)(m_initCW * (1 << FP_PREC) / m_binCW[i]);
      m_chromaAdjHelpLUT[i] = (int32_t)(m_initCW * (1 << FP_PREC) / ( m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset ) );
    }
  }
  for (int lumaSample = 0; lumaSample < m_reshapeLUTSize; lumaSample++)
  {
    int idxYInv = getPWLIdxInv(lumaSample);
    int invSample = m_inputPivot[idxYInv] + ((m_invScaleCoef[idxYInv] * (lumaSample - m_reshapePivot[idxYInv]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_invLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(invSample));
  }
}

void Reshape::rspCtuBcw( CodingStructure& cs, int col, int ln ) const
{
  if( !cs.sps->getUseReshaper() || !m_sliceReshapeInfo.sliceReshaperEnableFlag )
  {
    return;
  }

  const Slice* slice = cs.getCtuData( col, ln ).slice;
  if( !slice->getLmcsEnabledFlag() )
  {
    return;
  }

  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_RESHAPER, cs, CH_L );

  const PreCalcValues& pcv = *cs.pcv;

  int xPos = pcv.maxCUWidth * col;
  int lw = std::min( pcv.lumaWidth - xPos, pcv.maxCUWidth );

  int yPos = ln * pcv.maxCUHeight;
  int lh = std::min( pcv.lumaHeight - yPos, pcv.maxCUHeight );

  PelBuf picYuvRec = cs.getRecoBuf( COMPONENT_Y ).subBuf( Position( xPos, yPos ), Size( lw, lh ) );

#if 1
  if( g_pelBufOP.rspBcw )
    g_pelBufOP.rspBcw( picYuvRec.buf, picYuvRec.stride, picYuvRec.width, picYuvRec.height, m_lumaBD, m_sliceReshapeInfo.reshaperModelMinBinIdx, m_sliceReshapeInfo.reshaperModelMaxBinIdx, m_reshapePivot.data(), m_invScaleCoef.data(), m_inputPivot.data() );
  else
#endif
    g_pelBufOP.applyLut( picYuvRec.buf, picYuvRec.stride, picYuvRec.width, picYuvRec.height, m_invLUT );
}

void Reshape::rspBufFwd( PelBuf& buf ) const
{
  g_pelBufOP.rspFwd( buf.buf, buf.stride, buf.width, buf.height, m_lumaBD, m_initCW, m_reshapePivot.data(), m_fwdScaleCoef.data(), m_inputPivot.data() );
}

}
