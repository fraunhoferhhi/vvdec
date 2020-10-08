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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"

CoeffCodingContext::CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide )
  : m_chType                    (toChannelType(component))
  , m_width                     (tu.block(component).width)
  , m_height                    (tu.block(component).height)
  , m_log2CGWidth               ( g_log2SbbSize[ getLog2(m_width) ][ getLog2(m_height) ][0] )
  , m_log2CGHeight              ( g_log2SbbSize[ getLog2(m_width) ][ getLog2(m_height) ][1] )
  , m_log2CGSize                (m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups             (std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width ) >> m_log2CGWidth )
  , m_heightInGroups            (std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) >> m_log2CGHeight)
  , m_log2BlockWidth            (getLog2(m_width ))
  , m_log2BlockHeight           (getLog2(m_height))
  , m_log2BlockSize             ((m_log2BlockWidth + m_log2BlockHeight)>>1)
  , m_maxNumCoeff               (m_width * m_height)
  , m_signHiding                (signHide)
  , m_maxLog2TrDynamicRange     (tu.cu->cs->sps->getMaxLog2TrDynamicRange(m_chType))
  , m_scanType                  (SCAN_DIAG)
  , m_scan                      (g_scanOrder     [SCAN_GROUPED_4x4][m_scanType][g_sizeIdxInfo.idxFrom(m_width        )][g_sizeIdxInfo.idxFrom(m_height        )])
  , m_scanCG                    (g_scanOrder     [SCAN_UNGROUPED  ][m_scanType][g_sizeIdxInfo.idxFrom(m_widthInGroups)][g_sizeIdxInfo.idxFrom(m_heightInGroups)])
  , m_CtxSetLastX               (Ctx::LastX[m_chType])
  , m_CtxSetLastY               (Ctx::LastY[m_chType])
  , m_maxLastPosX               (g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width)  - 1])
  , m_maxLastPosY               (g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) - 1])
  , m_lastOffsetX               (0)
  , m_lastOffsetY               (0)
  , m_lastShiftX                (0)
  , m_lastShiftY                (0)
  , m_scanPosLast               (-1)
  , m_subSetId                  (-1)
  , m_subSetPos                 (-1)
  , m_subSetPosX                (-1)
  , m_subSetPosY                (-1)
  , m_minSubPos                 (-1)
  , m_maxSubPos                 (-1)
  , m_sigGroupCtxId             (-1)
  , m_tmplCpSum1                (-1)
  , m_tmplCpDiag                (-1)
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType+4] }
  , m_parFlagCtxSet             ( Ctx::ParFlag[m_chType] )
  , m_gtxFlagCtxSet             { Ctx::GtxFlag[m_chType], Ctx::GtxFlag[m_chType+2] }
  , m_sigGroupCtxIdTS           (-1)
  , m_tsSigFlagCtxSet           ( Ctx::TsSigFlag )
  , m_tsParFlagCtxSet           ( Ctx::TsParFlag )
  , m_tsGtxFlagCtxSet           ( Ctx::TsGtxFlag )
  , m_tsLrg1FlagCtxSet          (Ctx::TsLrg1Flag)
  , m_tsSignFlagCtxSet          (Ctx::TsResidualSign)
  , m_sigCoeffGroupFlag         ()
  , m_bdpcm                     (isLuma(component) ? tu.cu->bdpcmMode() : tu.cu->bdpcmModeChroma())
  , m_regBinLimit               ( ( TU::getTbAreaAfterCoefZeroOut( tu, component ) * ( isLuma( component ) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA ) ) >> 4 )
  , m_ts                        (tu.mtsIdx[component] == MTS_SKIP)
{
  if (m_chType == CHANNEL_TYPE_CHROMA)
  {
    const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( m_width  >> 3) );
    const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( m_height >> 3) );
  }
  else
  {
    static const int prefix_ctx[8]  = { 0, 0, 0, 3, 6, 10, 15, 21 };
    const_cast<int&>(m_lastOffsetX) = prefix_ctx[ m_log2BlockWidth  ];
    const_cast<int&>(m_lastOffsetY) = prefix_ctx[ m_log2BlockHeight ];
    const_cast<int&>(m_lastShiftX)  = (m_log2BlockWidth  + 1) >> 2;
    const_cast<int&>(m_lastShiftY)  = (m_log2BlockHeight + 1) >> 2;
  }
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[ m_subSetId ];
  m_subSetPosY              = m_subSetPos >> getLog2( m_widthInGroups );
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY * m_widthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  const bool lastHorGrp     = m_subSetPosX == m_widthInGroups  - 1;
  const bool lastVerGrp     = m_subSetPosY == m_heightInGroups - 1;
  m_checkTplBnd             = lastHorGrp;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
  unsigned  CGPosY    = m_subSetPosY;
  unsigned  CGPosX    = m_subSetPosX;
  unsigned  sigRight  = unsigned( !lastHorGrp  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
  unsigned  sigLower  = unsigned( !lastVerGrp  ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  m_sigGroupCtxId     = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );

  if( m_ts )
  {
    unsigned sigLeft  = unsigned( CGPosX > 0 ? m_sigCoeffGroupFlag[m_subSetPos - 1              ] : false );
    unsigned sigAbove = unsigned( CGPosY > 0 ? m_sigCoeffGroupFlag[m_subSetPos - m_widthInGroups] : false );
    m_sigGroupCtxIdTS = Ctx::TsSigCoeffGroup( sigLeft  + sigAbove );
  }
}


unsigned DeriveCtx::CtxModeConsFlag( const CodingStructure& cs, Partitioner& partitioner )
{
  CHECKD( partitioner.chType != CHANNEL_TYPE_LUMA, "Channel type has to be luma" );
                             
  const CodingUnit* cuLeft   = partitioner.currPartLevel().cuLeft;
  const CodingUnit* cuAbove  = partitioner.currPartLevel().cuAbove;

  unsigned ctxId = ( ( cuAbove && cuAbove->predMode() == MODE_INTRA ) || ( cuLeft && cuLeft->predMode() == MODE_INTRA ) ) ? 1 : 0;
  return ctxId;
}


void DeriveCtx::CtxSplit( const CodingStructure& cs, Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, bool *canSplit /*= nullptr */ )
{
  // get left depth
  const CodingUnit* cuLeft   = partitioner.currPartLevel().cuLeft;
  // get above depth
  const CodingUnit* cuAbove  = partitioner.currPartLevel().cuAbove;

  ///////////////////////
  // CTX do split (0-8)
  ///////////////////////
  const unsigned widthCurr  = partitioner.currArea().blocks[partitioner.chType].width;
  const unsigned heightCurr = partitioner.currArea().blocks[partitioner.chType].height;

  ctxSpl  = !!( cuLeft  && cuLeft ->blocks[partitioner.chType].height < heightCurr );
  ctxSpl += !!( cuAbove && cuAbove->blocks[partitioner.chType].width  < widthCurr  );

  unsigned
  numSplit  = canSplit[1] ? 2 : 0;
  numSplit += canSplit[2];
  numSplit += canSplit[3];
  numSplit += canSplit[4];
  numSplit += canSplit[5];

  if( numSplit > 0 ) numSplit--;

  ctxSpl += 3 * ( numSplit >> 1 );

  //////////////////////////
  // CTX is qt split (0-5)
  //////////////////////////
  ctxQt =  !!( cuLeft  && cuLeft->qtDepth  > partitioner.currQtDepth );
  ctxQt += !!( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth );
  ctxQt += partitioner.currQtDepth < 2 ? 0 : 3;

  ////////////////////////////
  // CTX is ver split (0-4)
  ////////////////////////////
  ctxHv = 0;

  const unsigned numHor = ( !!canSplit[2] ) + ( !!canSplit[4] );
  const unsigned numVer = ( !!canSplit[3] ) + ( !!canSplit[5] );

  if( numVer == numHor )
  {
    const unsigned wIdxAbove    = cuAbove ? getLog2( cuAbove->blocks[partitioner.chType].width  ) : 0;
    const unsigned hIdxLeft     = cuLeft  ? getLog2( cuLeft ->blocks[partitioner.chType].height ) : 0;

    const unsigned depAbove     = widthCurr  >> wIdxAbove;
    const unsigned depLeft      = heightCurr >> hIdxLeft;

    if( depAbove == depLeft || !cuLeft || !cuAbove ) ctxHv = 0;
    else if( depAbove < depLeft ) ctxHv = 1;
    else ctxHv = 2;
  }
  else if( numVer < numHor )
  {
    ctxHv = 3;
  }
  else
  {
    ctxHv = 4;
  }

  //////////////////////////
  // CTX is h/v bt (0-3)
  //////////////////////////
  ctxHorBt = !!( partitioner.currMtDepth <= 1 );
  ctxVerBt = !!( partitioner.currMtDepth <= 1 ) + 2;
}

unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const bool prevCbCbf, const int ispIdx )
{
  if( ispIdx && isLuma( compID ) )
  {
    return 2 + ( int ) prevCbCbf;
  }
  if( compID == COMPONENT_Cr )
  {
    return ( prevCbCbf ? 1 : 0 );
  }
  return 0;
}

unsigned DeriveCtx::CtxInterDir( const PredictionUnit& pu )
{
  {
    return ( 7 - ( ( getLog2( pu.lumaSize().width ) + getLog2( pu.lumaSize().height ) + 1 ) >> 1 ) );
  }
  return pu.qtDepth;
}

unsigned DeriveCtx::CtxAffineFlag( const CodingUnit& cu )
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cu.left;
  ctxId  = ( cuLeft  && cuLeft ->affineFlag() ) ? 1 : 0;

  const CodingUnit *cuAbove = cu.above;
  ctxId += ( cuAbove && cuAbove->affineFlag() ) ? 1 : 0;

  return ctxId;
}
unsigned DeriveCtx::CtxSkipFlag( const CodingUnit& cu )
{
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft  = cu.left;
  ctxId  = ( cuLeft  && cuLeft->skip() )  ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cu.above;
  ctxId += ( cuAbove && cuAbove->skip() ) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxPredModeFlag( const CodingUnit& cu )
{
  const CodingUnit *cuLeft  = cu.left;
  const CodingUnit *cuAbove = cu.above;

  unsigned ctxId = ( ( cuAbove && cuAbove->predMode() == MODE_INTRA ) || ( cuLeft && cuLeft->predMode() == MODE_INTRA ) ) ? 1 : 0;

  return ctxId;
}

unsigned DeriveCtx::CtxIBCFlag( const CodingUnit& cu )
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cu.left;
  ctxId += ( cuLeft  && CU::isIBC( *cuLeft ) )  ? 1 : 0;

  const CodingUnit *cuAbove = cu.above;
  ctxId += ( cuAbove && CU::isIBC( *cuAbove ) ) ? 1 : 0;

  return ctxId;
}

void MergeCtx::setMergeInfo( PredictionUnit& pu, int candIdx )
{
  CHECK( candIdx >= numValidMergeCand, "Merge candidate does not exist" );

  pu.setMergeFlag            ( true );
  pu.setMmvdFlag             ( false );
  pu.setInterDir             ( interDirNeighbours[candIdx] );
  pu.setImv                  ( ( !pu.geoFlag() && useAltHpelIf[candIdx] ) ? IMV_HPEL : 0 );
  pu.setMergeIdx             ( candIdx );
  pu.setMergeType            ( mrgTypeNeighbours[candIdx] );
  pu.mv  [REF_PIC_LIST_0][0] = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  pu.mv  [REF_PIC_LIST_1][0] = mvFieldNeighbours[(candIdx << 1) + 1].mv;
  pu.refIdx [REF_PIC_LIST_0] = mvFieldNeighbours[( candIdx << 1 ) + 0].refIdx;
  pu.refIdx [REF_PIC_LIST_1] = mvFieldNeighbours[( candIdx << 1 ) + 1].refIdx;
  pu.mvpIdx [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx [REF_PIC_LIST_1] = NOT_VALID;
  pu.setBcwIdx               ( ( interDirNeighbours[candIdx] == 3 ) ? BcwIdx[candIdx] : BCW_DEFAULT );

  PU::restrictBiPredMergeCandsOne( pu );
}

void MergeCtx::setMmvdMergeCandiInfo( PredictionUnit& pu, int candIdx )
{
  const Slice &slice        = *pu.slice;
  const int mvShift         = MV_FRACTIONAL_BITS_DIFF;
  const int refMvdCands[8]  = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };

  int fPosGroup     = 0;
  int fPosBaseIdx   = 0;
  int fPosStep      = 0;
  int tempIdx       = 0;
  int fPosPosition  = 0;

  Mv tempMv[2];

  tempIdx       = candIdx;
  fPosGroup     = tempIdx / (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  tempIdx       = tempIdx - fPosGroup * (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  fPosBaseIdx   = tempIdx / MMVD_MAX_REFINE_NUM;
  tempIdx       = tempIdx - fPosBaseIdx * (MMVD_MAX_REFINE_NUM);
  fPosStep      = tempIdx / 4;
  fPosPosition  = tempIdx - fPosStep * (4);
  int offset    = refMvdCands[fPosStep];

  if( pu.slice->getPicHeader()->getDisFracMMVD() )
  {
    offset <<= 2;
  }
  const int refList0 = mmvdBaseMv[fPosBaseIdx][0].refIdx;
  const int refList1 = mmvdBaseMv[fPosBaseIdx][1].refIdx;

  if( ( refList0 != -1 ) && ( refList1 != -1 ) )
  {
    const int poc0    = slice.getRefPOC(REF_PIC_LIST_0, refList0);
    const int poc1    = slice.getRefPOC(REF_PIC_LIST_1, refList1);
    const int currPoc = slice.getPOC();

    if( fPosPosition == 0 )
    {
      tempMv[0] = Mv( offset, 0 );
    }
    else if( fPosPosition == 1 )
    {
      tempMv[0] = Mv( -offset, 0 );
    }
    else if( fPosPosition == 2 )
    {
      tempMv[0] = Mv( 0, offset );
    }
    else
    {
      tempMv[0] = Mv( 0, -offset );
    }
    if( ( poc0 - currPoc ) == ( poc1 - currPoc ) )
    {
      tempMv[1] = tempMv[0];
    }
    else if( abs( poc1 - currPoc ) > abs( poc0 - currPoc ) )
    {
      tempMv[1] = tempMv[0];

      const int  scale           = PU::getDistScaleFactor( currPoc, poc0, currPoc, poc1 );
      const bool isL0RefLongTerm = pu.slice->getLocalRPL0()->isRefPicLongterm( refList0 );
      const bool isL1RefLongTerm = pu.slice->getLocalRPL1()->isRefPicLongterm( refList1 );

      if( isL0RefLongTerm || isL1RefLongTerm )
      {
        if( ( poc1 - currPoc ) * ( poc0 - currPoc ) > 0 )
        {
          tempMv[0] = tempMv[1];
        }
        else
        {
          tempMv[0].set( -1 * tempMv[1].getHor(), -1 * tempMv[1].getVer() );
        }
      }
      else
        tempMv[0] = tempMv[1].scaleMv( scale );
    }
    else
    {
      const int  scale            = PU::getDistScaleFactor( currPoc, poc1, currPoc, poc0 );
      const bool isL0RefLongTerm  = pu.slice->getLocalRPL0()->isRefPicLongterm( refList0 );
      const bool isL1RefLongTerm  = pu.slice->getLocalRPL1()->isRefPicLongterm( refList1 );

      if( isL0RefLongTerm || isL1RefLongTerm )
      {
        if( ( poc1 - currPoc ) * ( poc0 - currPoc ) > 0 )
        {
          tempMv[1] = tempMv[0];
        }
        else
        {
          tempMv[1].set( -1 * tempMv[0].getHor(), -1 * tempMv[0].getVer() );
        }
      }
      else
        tempMv[1] = tempMv[0].scaleMv( scale );
    }

    pu.setInterDir   ( 3 );
    pu.mv    [L0][0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[L0]    = refList0;
    pu.mv    [L1][0] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[L1]    = refList1;
  }
  else if( refList0 != -1 )
  {
    if( fPosPosition == 0 )
    {
      tempMv[0] = Mv( offset, 0 );
    }
    else if( fPosPosition == 1 )
    {
      tempMv[0] = Mv( -offset, 0 );
    }
    else if( fPosPosition == 2 )
    {
      tempMv[0] = Mv( 0, offset );
    }
    else
    {
      tempMv[0] = Mv( 0, -offset );
    }

    pu.setInterDir   ( 1 );
    pu.mv    [L0][0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[L0]    = refList0;
    pu.mv    [L1][0] = Mv(0, 0);
    pu.refIdx[L1]    = -1;
  }
  else if( refList1 != -1 )
  {
    if( fPosPosition == 0 )
    {
      tempMv[1] = Mv( offset, 0 );
    }
    else if( fPosPosition == 1 )
    {
      tempMv[1] = Mv( -offset, 0 );
    }
    else if( fPosPosition == 2 )
    {
      tempMv[1] = Mv( 0, offset );
    }
    else
    {
      tempMv[1] = Mv( 0, -offset );
    }

    pu.setInterDir               ( 2 );
    pu.mv    [REF_PIC_LIST_0][0] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_0]    = -1;
    pu.mv    [REF_PIC_LIST_1][0] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1]    = refList1;
  }

  pu.setMmvdFlag        ( true );
  pu.mmvdIdx            = candIdx;
  pu.setMergeFlag       ( true );
  //pu.setMergeIdx        ( candIdx );
  pu.setMergeType       ( MRG_TYPE_DEFAULT_N );
  pu.mvpIdx [L0]        = NOT_VALID;
  pu.mvpIdx [L1]        = NOT_VALID;
  pu.setImv             ( mmvdUseAltHpelIf[fPosBaseIdx] ? IMV_HPEL : 0 );
  pu.setBcwIdx          ( ( interDirNeighbours[fPosBaseIdx] == 3 ) ? BcwIdx[fPosBaseIdx] : BCW_DEFAULT );

  for( int refList = 0; refList < 2; refList++ )
  {
    if( pu.refIdx[refList] >= 0 )
    {
      pu.mv[refList][0].clipToStorageBitDepth();
    }
  }

  PU::restrictBiPredMergeCandsOne( pu );
}

unsigned DeriveCtx::CtxMipFlag( const CodingUnit& cu )
{
  unsigned ctxId = 0;

  const CodingUnit *cuLeft  = cu.left;
  ctxId  = ( cuLeft  && cuLeft->mipFlag()  ) ? 1 : 0;

  const CodingUnit *cuAbove = cu.above;
  ctxId += ( cuAbove && cuAbove->mipFlag() ) ? 1 : 0;

  ctxId = ( cu.lwidth() > 2 * cu.lheight() || cu.lheight() > 2 * cu.lwidth() ) ? 3 : ctxId;

  return ctxId;
}
