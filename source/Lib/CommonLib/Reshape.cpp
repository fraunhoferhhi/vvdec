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

/** \file     Reshape.cpp
    \brief    common reshaper class
*/
#include "Reshape.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <UnitTools.h>
#include "CommonLib/TimeProfiler.h"
 //! \ingroup CommonLib
 //! \{

 // ====================================================================================================================
 // Constructor / destructor / create / destroy
 // ====================================================================================================================

Reshape::Reshape()
{
  m_CTUFlag = false;
  m_fwdLUT = nullptr;
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
  if( !m_fwdLUT )
  {
    m_fwdLUT = ( Pel* ) xMalloc( Pel, m_reshapeLUTSize + 1 );
    memset( m_fwdLUT, 0, ( m_reshapeLUTSize + 1 ) * sizeof( Pel ) );
  }
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
  xFree( m_fwdLUT );
  m_fwdLUT = nullptr;
  xFree( m_invLUT );
  m_invLUT = nullptr;
}

void  Reshape::initSlice( Slice* pcSlice )
{
  if( pcSlice->getPicHeader()->getLmcsEnabledFlag() )
  {
    if( pcSlice->getNalUnitLayerId() != pcSlice->getPicHeader()->getLmcsAPS()->getLayerId() )
    {
      CHECK( pcSlice->getPicHeader()->getLmcsAPS()->getLayerId() > pcSlice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
      CHECK( pcSlice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
      for (int i = 0; i < pcSlice->getVPS()->getNumOutputLayerSets(); i++ )
      {
        bool isCurrLayerInOls = false;
        bool isRefLayerInOls = false;
        for( int j = pcSlice->getVPS()->getNumLayersInOls(i) - 1; j >= 0; j-- )
        {
          if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getNalUnitLayerId() )
          {
            isCurrLayerInOls = true;
          }
          if( pcSlice->getVPS()->getLayerIdInOls(i, j) == pcSlice->getPicHeader()->getLmcsAPS()->getLayerId() )
          {
            isRefLayerInOls = true;
          }
        }
        CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
      }
    }

    SliceReshapeInfo& sInfo = pcSlice->getPicHeader()->getLmcsAPS()->getReshaperAPSInfo();
    m_sliceReshapeInfo.sliceReshaperEnableFlag       = true;
    m_sliceReshapeInfo.sliceReshaperModelPresentFlag = true;
    m_sliceReshapeInfo.enableChromaAdj               = pcSlice->getPicHeader()->getLmcsChromaResidualScaleFlag();
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
  if( ( pcSlice->getSliceType() == I_SLICE ) && m_sliceReshapeInfo.sliceReshaperEnableFlag )
  {
    m_CTUFlag = false;
  }
  else
  {
    if( m_sliceReshapeInfo.sliceReshaperEnableFlag )
    {
      m_CTUFlag = true;
    }
    else
    {
      m_CTUFlag = false;
    }
  }
  m_vpduX = -1;
  m_vpduX = -1;
}

void Reshape::rspLine( CodingStructure &cs, int ln, const int offset ) const
{
  if( !( cs.sps->getUseReshaper() && m_sliceReshapeInfo.sliceReshaperEnableFlag ) )
  {
    return;
  }
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_RESHAPER, cs, CH_L );

  const PreCalcValues &pcv = *cs.pcv;

  const bool firstLine = ln == 0;

//  const int lh = frstLine ? pcv.maxCUHeight + ( offset ) : pcv.maxCUHeight;

  int lw   = pcv.lumaWidth;
  int yPos = firstLine ? 0 : ln * pcv.maxCUHeight + offset;
  int lh   = firstLine ? pcv.maxCUHeight + offset : std::min( pcv.lumaHeight - yPos, pcv.maxCUHeight );
  PelBuf picYuvRec = cs.getRecoBuf( COMPONENT_Y ).subBuf( Position( 0, yPos ), Size( lw, lh ) );
  picYuvRec.rspSignal( m_invLUT );
}

void Reshape::rspCtu( CodingStructure &cs, int col, int ln, const int offset ) const
{
  if( !( cs.sps->getUseReshaper() && m_sliceReshapeInfo.sliceReshaperEnableFlag ) )
  {
    return;
  }
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_RESHAPER, cs, CH_L );

  const PreCalcValues &pcv = *cs.pcv;

  const bool firstLine = ln == 0;

//  const int lh = frstLine ? pcv.maxCUHeight + ( offset ) : pcv.maxCUHeight;

  int xPos = pcv.maxCUWidth * col;
  int lw   = std::min( pcv.lumaWidth - xPos, pcv.maxCUWidth );

  int yPos = firstLine ? 0 : ln * pcv.maxCUHeight + offset;
  int lh   = firstLine ? pcv.maxCUHeight + offset : std::min( pcv.lumaHeight - yPos, pcv.maxCUHeight );

  PelBuf picYuvRec = cs.getRecoBuf( COMPONENT_Y ).subBuf( Position( xPos, yPos ), Size( lw, lh ) );
  picYuvRec.rspSignal( m_invLUT );
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
    const Pel   valueDC = 1 << (tu.cu->cs->sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
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
      m_invScaleCoef[i] = (int32_t)(m_initCW * (1 << FP_PREC) / m_binCW[i]);
      m_chromaAdjHelpLUT[i] = (int32_t)(m_initCW * (1 << FP_PREC) / ( m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset ) );
    }
  }
  for (int lumaSample = 0; lumaSample < m_reshapeLUTSize; lumaSample++)
  {
    int idxY = lumaSample / m_initCW;
    int tempVal = m_reshapePivot[idxY] + ((m_fwdScaleCoef[idxY] * (lumaSample - m_inputPivot[idxY]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_fwdLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(tempVal));

    int idxYInv = getPWLIdxInv(lumaSample);
    int invSample = m_inputPivot[idxYInv] + ((m_invScaleCoef[idxYInv] * (lumaSample - m_reshapePivot[idxYInv]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_invLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(invSample));
  }
}



//
//! \}
