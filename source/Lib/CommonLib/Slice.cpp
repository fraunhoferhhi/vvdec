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

/** \file     Slice.cpp
    \brief    slice header and SPS class
*/

#include "CommonDef.h"
#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "dtrace_next.h"

#include "UnitTools.h"
#include "vvdec/sei.h"

namespace vvdec
{

void VPS::deriveOutputLayerSets()
{
  if( m_uiMaxLayers == 1 )
  {
    m_totalNumOLSs = 1;
  }
  else if( m_vpsEachLayerIsAnOlsFlag || m_vpsOlsModeIdc < 2 )
  {
    m_totalNumOLSs = m_uiMaxLayers;
  }
  else if( m_vpsOlsModeIdc == 2 )
  {
    m_totalNumOLSs = m_vpsNumOutputLayerSets;
  }

  m_olsDpbParamsIdx.resize( m_totalNumOLSs );
  m_olsDpbPicSize.resize( m_totalNumOLSs, Size(0, 0) );
  m_numOutputLayersInOls.resize( m_totalNumOLSs );
  m_numLayersInOls.resize( m_totalNumOLSs );
  m_outputLayerIdInOls.resize( m_totalNumOLSs, std::vector<int>( m_uiMaxLayers, NOT_VALID ) );
  m_layerIdInOls.resize( m_totalNumOLSs, std::vector<int>( m_uiMaxLayers, NOT_VALID ) );

  std::vector<int> numRefLayers( m_uiMaxLayers );
  std::vector<std::vector<int>> outputLayerIdx( m_totalNumOLSs, std::vector<int>( m_uiMaxLayers, NOT_VALID ) );
  std::vector<std::vector<int>> layerIncludedInOlsFlag( m_totalNumOLSs, std::vector<int>( m_uiMaxLayers, 0 ) );
  std::vector<std::vector<int>> dependencyFlag( m_uiMaxLayers, std::vector<int>( m_uiMaxLayers, NOT_VALID ) );
  std::vector<std::vector<int>> refLayerIdx( m_uiMaxLayers, std::vector<int>( m_uiMaxLayers, NOT_VALID ) );
  std::vector<int> layerUsedAsRefLayerFlag( m_uiMaxLayers, 0 );
  std::vector<int> layerUsedAsOutputLayerFlag( m_uiMaxLayers, NOT_VALID );
  for( int i = 0; i < m_uiMaxLayers; i++ )
  {
    int r = 0;

    for( int j = 0; j < m_uiMaxLayers; j++ )
    {
      dependencyFlag[i][j] = m_vpsDirectRefLayerFlag[i][j];

      for( int k = 0; k < i; k++ )
      {
        if( m_vpsDirectRefLayerFlag[i][k] && dependencyFlag[k][j] )
        {
          dependencyFlag[i][j] = 1;
        }
      }
      if (m_vpsDirectRefLayerFlag[i][j])
      {
        layerUsedAsRefLayerFlag[j] = 1;
      }
      if( dependencyFlag[i][j] )
      {
        refLayerIdx[i][r++] = j;
      }
    }

    numRefLayers[i] = r;
  }

  m_numOutputLayersInOls[0] = 1;
  m_outputLayerIdInOls[0][0] = m_vpsLayerId[0];
  layerUsedAsOutputLayerFlag[0] = 1;
  for (int i = 1; i < m_uiMaxLayers; i++)
  {
    if (m_vpsEachLayerIsAnOlsFlag || m_vpsOlsModeIdc < 2)
    {
      layerUsedAsOutputLayerFlag[i] = 1;
    }
    else
    {
      layerUsedAsOutputLayerFlag[i] = 0;
    }
  }
  for( int i = 1; i < m_totalNumOLSs; i++ )
  {
    if( m_vpsEachLayerIsAnOlsFlag || m_vpsOlsModeIdc == 0 )
    {
      m_numOutputLayersInOls[i] = 1;
      m_outputLayerIdInOls[i][0] = m_vpsLayerId[i];
    }
    else if( m_vpsOlsModeIdc == 1 )
    {
      m_numOutputLayersInOls[i] = i + 1;

      for( int j = 0; j < m_numOutputLayersInOls[i]; j++ )
      {
        m_outputLayerIdInOls[i][j] = m_vpsLayerId[j];
      }
    }
    else if( m_vpsOlsModeIdc == 2 )
    {
      int j = 0;
      for( int k = 0; k < m_uiMaxLayers; k++ )
      {
        if( m_vpsOlsOutputLayerFlag[i][k] )
        {
          layerIncludedInOlsFlag[i][k] = 1;
          layerUsedAsOutputLayerFlag[k] = 1;
          outputLayerIdx[i][j] = k;
          m_outputLayerIdInOls[i][j++] = m_vpsLayerId[k];
        }
      }
      m_numOutputLayersInOls[i] = j;

      for( j = 0; j < m_numOutputLayersInOls[i]; j++ )
      {
        int idx = outputLayerIdx[i][j];
        for( int k = 0; k < numRefLayers[idx]; k++ )
        {
          layerIncludedInOlsFlag[i][refLayerIdx[idx][k]] = 1;
        }
      }
    }
  }
  for (int i = 0; i < m_uiMaxLayers; i++)
  {
    CHECK(layerUsedAsRefLayerFlag[i] == 0 && layerUsedAsOutputLayerFlag[i] == 0, "There shall be no layer that is neither an output layer nor a direct reference layer");
  }

  m_numLayersInOls[0]   = 1;
  m_layerIdInOls[0][0]  = m_vpsLayerId[0];
  m_numMultiLayeredOlss = 0;

  for( int i = 1; i < m_totalNumOLSs; i++ )
  {
    if( m_vpsEachLayerIsAnOlsFlag )
    {
      m_numLayersInOls[i] = 1;
      m_layerIdInOls[i][0] = m_vpsLayerId[i];
    }
    else if( m_vpsOlsModeIdc == 0 || m_vpsOlsModeIdc == 1 )
    {
      m_numLayersInOls[i] = i + 1;
      for( int j = 0; j < m_numLayersInOls[i]; j++ )
      {
        m_layerIdInOls[i][j] = m_vpsLayerId[j];
      }
    }
    else if( m_vpsOlsModeIdc == 2 )
    {
      int j = 0;
      for( int k = 0; k < m_uiMaxLayers; k++ )
      {
        if( layerIncludedInOlsFlag[i][k] )
        {
          m_layerIdInOls[i][j++] = m_vpsLayerId[k];
        }
      }

      m_numLayersInOls[i] = j;
    }
    if( m_numLayersInOls[i] > 1 )
    {
      m_multiLayerOlsIdx[i] = m_numMultiLayeredOlss;
      m_numMultiLayeredOlss++;
    }
  }
  m_multiLayerOlsIdxToOlsIdx.resize(m_numMultiLayeredOlss);

  for (int i=0, j=0; i<m_totalNumOLSs; i++)
  {
    if (m_numLayersInOls[i] > 1)
    {
      m_multiLayerOlsIdxToOlsIdx[j] = i;
    }
  }
}

void VPS::checkVPS()
{
  for (int multiLayerOlsIdx=0; multiLayerOlsIdx < m_numMultiLayeredOlss; multiLayerOlsIdx++)
  {
    const int olsIdx = m_multiLayerOlsIdxToOlsIdx[multiLayerOlsIdx];
    const int olsHrdIdx = getOlsHrdIdx(multiLayerOlsIdx);
    const int olsPtlIdx = getOlsPtlIdx(olsIdx);
    CHECK (getHrdMaxTid(olsHrdIdx) < getPtlMaxTemporalId(olsPtlIdx), "The value of vps_hrd_max_tid[vps_ols_timing_hrd_idx[m]] shall be greater than or equal to "
                                                                     "vps_ptl_max_tid[ vps_ols_ptl_idx[n]] for each m-th multi-layer OLS for m from 0 to "
                                                                     "NumMultiLayerOlss - 1, inclusive, and n being the OLS index of the m-th multi-layer OLS among all OLSs.");
    const int olsDpbParamsIdx = getOlsDpbParamsIdx(multiLayerOlsIdx);
    CHECK (m_dpbMaxTemporalId[olsDpbParamsIdx] < getPtlMaxTemporalId(olsPtlIdx), "The value of vps_dpb_max_tid[vps_ols_dpb_params_idx[m]] shall be greater than or equal to "
                                                                     "vps_ptl_max_tid[ vps_ols_ptl_idx[n]] for each m-th multi-layer OLS for m from 0 to "
                                                                     "NumMultiLayerOlss - 1, inclusive, and n being the OLS index of the m-th multi-layer OLS among all OLSs.");
  }
}

void VPS::deriveTargetOutputLayerSet( int targetOlsIdx )
{
  m_iTargetLayer = targetOlsIdx < 0 ? m_uiMaxLayers - 1 : targetOlsIdx;
  m_targetOutputLayerIdSet.clear();
  m_targetLayerIdSet.clear();

  for( int i = 0; i < m_numOutputLayersInOls[m_iTargetLayer]; i++ )
  {
    m_targetOutputLayerIdSet.push_back( m_outputLayerIdInOls[m_iTargetLayer][i] );
  }

  for( int i = 0; i < m_numLayersInOls[m_iTargetLayer]; i++ )
  {
    m_targetLayerIdSet.push_back( m_layerIdInOls[m_iTargetLayer][i] );
  }
}

Slice::Slice()
{
  for ( int idx = 0; idx < MAX_NUM_REF; idx++ )
  {
    m_list1IdxToList0Idx[idx] = -1;
  }

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    for(int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++)
    {
      m_apcRefPicList [i][iNumCount] = nullptr;
      m_aiRefPOCList  [i][iNumCount] = 0;
    }

    m_apcRefPicList[i][MAX_NUM_REF] = nullptr;
  }

  resetWpScaling();
  initWpAcDcParam();

  memset( m_alfApss, 0, sizeof( m_alfApss ) );
}


void Slice::initSlice()
{
  for(uint32_t i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]      = 0;
  }
  m_colFromL0Flag = true;
  m_colRefIdx = 0;

  m_bCheckLDC = false;

  m_biDirPred = false;
  m_symRefIdx[0] = -1;
  m_symRefIdx[1] = -1;

  for (uint32_t component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = 0;
  }
  m_iSliceChromaQpDelta[JOINT_CbCr] = 0;


  m_substreamSizes.clear();
  m_cabacInitFlag        = false;
  m_cabacWinUpdateMode   = 0;
  resetTileGroupAlfEnabledFlag();
  resetTileGroupCcAlfEnabledFlags();
  m_sliceMap.initSliceMap();
}

void Slice::inheritFromPicHeader( PicHeader *picHeader, const PPS *pps, const SPS *sps )
{
  if( pps->getRplInfoInPhFlag() )
  {
    for( auto l: { REF_PIC_LIST_0, REF_PIC_LIST_1 } )
    {
      m_RPLIdx[l] = picHeader->getRPLIdx( l );
      if( m_RPLIdx[l] == -1 )
      {
        m_RPL[l] = *picHeader->getRPL( l );
      }
      else
      {
        m_RPL[l] = sps->getRPLList1()[m_RPLIdx[l]];
      }
    }
  }

  setDeblockingFilterDisable( picHeader->getDeblockingFilterDisable() );
  setDeblockingFilterBetaOffsetDiv2( picHeader->getDeblockingFilterBetaOffsetDiv2() );
  setDeblockingFilterTcOffsetDiv2( picHeader->getDeblockingFilterTcOffsetDiv2() );
  if( pps->getPPSChromaToolFlag() )
  {
    setDeblockingFilterCbBetaOffsetDiv2( picHeader->getDeblockingFilterCbBetaOffsetDiv2() );
    setDeblockingFilterCbTcOffsetDiv2( picHeader->getDeblockingFilterCbTcOffsetDiv2() );
    setDeblockingFilterCrBetaOffsetDiv2( picHeader->getDeblockingFilterCrBetaOffsetDiv2() );
    setDeblockingFilterCrTcOffsetDiv2( picHeader->getDeblockingFilterCrTcOffsetDiv2() );
  }
  else
  {
    setDeblockingFilterCbBetaOffsetDiv2 ( getDeblockingFilterBetaOffsetDiv2() );
    setDeblockingFilterCbTcOffsetDiv2   ( getDeblockingFilterTcOffsetDiv2()   );
    setDeblockingFilterCrBetaOffsetDiv2 ( getDeblockingFilterBetaOffsetDiv2() );
    setDeblockingFilterCrTcOffsetDiv2   ( getDeblockingFilterTcOffsetDiv2()   );
  }

  setSaoEnabledFlag(CHANNEL_TYPE_LUMA,     picHeader->getSaoEnabledFlag(CHANNEL_TYPE_LUMA));
  setSaoEnabledFlag(CHANNEL_TYPE_CHROMA,   picHeader->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA));

  setTileGroupAlfEnabledFlag(COMPONENT_Y,  picHeader->getAlfEnabledFlag(COMPONENT_Y));
  setTileGroupAlfEnabledFlag(COMPONENT_Cb, picHeader->getAlfEnabledFlag(COMPONENT_Cb));
  setTileGroupAlfEnabledFlag(COMPONENT_Cr, picHeader->getAlfEnabledFlag(COMPONENT_Cr));
  setTileGroupNumAps(picHeader->getNumAlfAps());
  setAlfAPSids( picHeader->getAlfAPSs() );
  setTileGroupApsIdChroma(picHeader->getAlfApsIdChroma());   
  setTileGroupCcAlfCbEnabledFlag(picHeader->getCcAlfEnabledFlag(COMPONENT_Cb));
  setTileGroupCcAlfCrEnabledFlag(picHeader->getCcAlfEnabledFlag(COMPONENT_Cr));
  setTileGroupCcAlfCbApsId(picHeader->getCcAlfCbApsId());
  setTileGroupCcAlfCrApsId(picHeader->getCcAlfCrApsId());
}

void  Slice::setNumEntryPoints( const SPS *sps, const PPS *pps )
{
  uint32_t ctuAddr, ctuX, ctuY;
  m_numEntryPoints = 0;

  if( !sps->getEntryPointsPresentFlag() )
  {
    return;
  }

  // count the number of CTUs that align with either the start of a tile, or with an entropy coding sync point
  // ignore the first CTU since it doesn't count as an entry point
  for( uint32_t i = 1; i < m_sliceMap.getNumCtuInSlice(); i++ )
  {
    ctuAddr = m_sliceMap.getCtuAddrInSlice( i );
    ctuX = ( ctuAddr % pps->getPicWidthInCtu() );
    ctuY = ( ctuAddr / pps->getPicWidthInCtu() );
    uint32_t prevCtuAddr = m_sliceMap.getCtuAddrInSlice(i - 1);
    uint32_t prevCtuX    = (prevCtuAddr % pps->getPicWidthInCtu());
    uint32_t prevCtuY    = (prevCtuAddr / pps->getPicWidthInCtu());

    if( pps->ctuToTileRowBd(ctuY) != pps->ctuToTileRowBd(prevCtuY) || pps->ctuToTileColBd(ctuX) != pps->ctuToTileColBd(prevCtuX) || (ctuY != prevCtuY && sps->getEntropyCodingSyncEnabledFlag()) )

    {
      m_numEntryPoints++;
    }
  }
}

void Slice::setDefaultClpRng( const SPS& sps )
{
  CHECK( sps.getBitDepth( CH_L ) != sps.getBitDepth( CH_C ), "Luma and chroma bit depths are different!" );
  m_clpRngs.bd  = sps.getBitDepth(CHANNEL_TYPE_LUMA);
}


bool Slice::getRapPicFlag() const
{
  return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}

Picture* Slice::xGetRefPic( const PicListRange & rcListPic, int poc, const int layerId )
{
  // return a nullptr, if picture is not found
  for ( auto &currPic : rcListPic )
  {
    if( currPic->getPOC() == poc && currPic->layerId == layerId )
    {
      return currPic;
    }
  }

  return nullptr;
}

Picture* Slice::xGetLongTermRefPic( const PicListRange & rcListPic, int poc, bool pocHasMsb, const int layerId )
{
  for( auto & pcPic: rcListPic )
  {
    if( pcPic && pcPic->getPOC() != this->getPOC() && pcPic->referenced && pcPic->layerId == layerId )
    {
      if( isLTPocEqual( poc,  pcPic->getPOC(), getSPS()->getBitsForPOC(), pocHasMsb ) )
      {
        if( !pcPic->longTerm )
          return nullptr;

        return pcPic;
      }
    }
  }

  return nullptr;
}

void Slice::setRefPOCList()
{
  for (int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
  {
    for (int iNumRefIdx = 0; iNumRefIdx < m_aiNumRefIdx[iDir]; iNumRefIdx++)
    {
      m_aiRefPOCList[iDir][iNumRefIdx] = m_apcRefPicList[iDir][iNumRefIdx]->getPOC();
    }
  }
}

void Slice::setList1IdxToList0Idx()
{
  int idxL0, idxL1;
  for ( idxL1 = 0; idxL1 < getNumRefIdx( REF_PIC_LIST_1 ); idxL1++ )
  {
    m_list1IdxToList0Idx[idxL1] = -1;
    for ( idxL0 = 0; idxL0 < getNumRefIdx( REF_PIC_LIST_0 ); idxL0++ )
    {
      if ( m_apcRefPicList[REF_PIC_LIST_0][idxL0 + 1]->getPOC() == m_apcRefPicList[REF_PIC_LIST_1][idxL1 + 1]->getPOC() )
      {
        m_list1IdxToList0Idx[idxL1] = idxL0;
        break;
      }
    }
  }
}

void Slice::constructRefPicLists( const PicListRange& rcListPic )
{
  ::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));
  if (m_eSliceType == I_SLICE)
  {
    ::memset(m_apcRefPicList, 0, sizeof(m_apcRefPicList));
    ::memset(m_aiNumRefIdx, 0, sizeof(m_aiNumRefIdx));
    return;
  }

  constructSingleRefPicList( rcListPic, REF_PIC_LIST_0 );
  constructSingleRefPicList( rcListPic, REF_PIC_LIST_1 );
}

void Slice::constructSingleRefPicList(const PicListRange& rcListPic, RefPicList listId )
{
  ReferencePictureList& rRPL = m_RPL[listId];

  uint32_t numOfActiveRef = getNumRefIdx( listId );
  for( int ii = 0; ii < numOfActiveRef; ii++ )
  {
    Picture* pcRefPic = nullptr;

    if( !rRPL.isRefPicLongterm( ii ) )
    {
      pcRefPic           = xGetRefPic( rcListPic, getPOC() + rRPL.getRefPicIdentifier( ii ), m_pcPic->layerId );
      pcRefPic->longTerm = false;
    }
    else
    {
      int ltrpPoc = rRPL.calcLTRefPOC( getPOC(), getSPS()->getBitsForPOC(), ii );

      pcRefPic           = xGetLongTermRefPic( rcListPic, ltrpPoc, rRPL.getDeltaPocMSBPresentFlag( ii ), m_pcPic->layerId );
      pcRefPic->longTerm = true;
    }

    m_apcRefPicList    [listId][ii] = pcRefPic;
    m_bIsUsedAsLongTerm[listId][ii] = pcRefPic->longTerm;

    rRPL.setRefPicLongterm( ii,pcRefPic->longTerm );
  }
}

void Slice::checkColRefIdx(uint32_t curSliceSegmentIdx, const Picture* pic)
{
  int i;
  Slice* curSlice = pic->slices[curSliceSegmentIdx];
  int currColRefPOC =  curSlice->getRefPOC( RefPicList(1 - curSlice->getColFromL0Flag()), curSlice->getColRefIdx());

  for(i=curSliceSegmentIdx-1; i>=0; i--)
  {
    const Slice* preSlice = pic->slices[i];
    if(preSlice->getSliceType() != I_SLICE)
    {
      const int preColRefPOC  = preSlice->getRefPOC( RefPicList(1 - preSlice->getColFromL0Flag()), preSlice->getColRefIdx());
      if(currColRefPOC != preColRefPOC)
      {
        THROW("Collocated_ref_idx shall always be the same for all slices of a coded picture!");
      }
      else
      {
        break;
      }
    }
  }
}

void Slice::checkCRA( int& pocCRA, NalUnitType& associatedIRAPType, const PicListRange& rcListPic )
{
  if( pocCRA < MAX_UINT && getPOC() > pocCRA )
  {
    for( int l = 0; l < NUM_REF_PIC_LIST_01; ++l )
    {
      const uint32_t numRefPic = m_RPL[l].getNumberOfShorttermPictures() + m_RPL[l].getNumberOfLongtermPictures();
      for( int i = 0; i < numRefPic; i++ )
      {
        if( !m_RPL[l].isRefPicLongterm( i ) )
        {
          CHECK( getPOC() + m_RPL[l].getRefPicIdentifier( i ) < pocCRA, "Invalid state" );
        }
        else
        {
          CHECK( xGetLongTermRefPic( rcListPic, m_RPL[l].getRefPicIdentifier( i ), m_RPL[l].getDeltaPocMSBPresentFlag( i ), m_pcPic->layerId )->getPOC() < pocCRA, "Invalid state" );
        }
      }
    }
  }
  if (getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP) // IDR picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
  else if (getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA) // CRA picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
}

void Slice::checkSTSA( const PicListRange& rcListPic )
{
  int ii;
  Picture* pcRefPic = NULL;
  int numOfActiveRef = getNumRefIdx(REF_PIC_LIST_0);

  for (ii = 0; ii < numOfActiveRef; ii++)
  {
    pcRefPic = m_apcRefPicList[REF_PIC_LIST_0][ii];

    if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA && pcRefPic->layerId == m_pcPic->layerId )
    {
      CHECK( pcRefPic->layer == m_uiTLayer, "When the current picture is an STSA picture and nuh_layer_id equal to that of the current picture, there shall be no active entry in the RPL that has TemporalId equal to that of the current picture" );
    }
    
    // Checking this: "When the current picture is a picture that follows, in decoding order, an STSA picture that has TemporalId equal to that of the current picture, there shall be no
    // picture that has TemporalId equal to that of the current picture included as an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes the STSA picture in decoding order."
    CHECK(pcRefPic->subLayerNonReferencePictureDueToSTSA, "The RPL of the current picture contains a picture that is not allowed in this temporal layer due to an earlier STSA picture");
  }

  numOfActiveRef = getNumRefIdx(REF_PIC_LIST_1);
  for (ii = 0; ii < numOfActiveRef; ii++)
  {
    pcRefPic = m_apcRefPicList[REF_PIC_LIST_1][ii];

    if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA && pcRefPic->layerId == m_pcPic->layerId )
    {
      CHECK( pcRefPic->layer == m_uiTLayer, "When the current picture is an STSA picture and nuh_layer_id equal to that of the current picture, there shall be no active entry in the RPL that has TemporalId equal to that of the current picture" );
    }
    
    // Checking this: "When the current picture is a picture that follows, in decoding order, an STSA picture that has TemporalId equal to that of the current picture, there shall be no
    // picture that has TemporalId equal to that of the current picture included as an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes the STSA picture in decoding order."
    CHECK(pcRefPic->subLayerNonReferencePictureDueToSTSA, "The active RPL part of the current picture contains a picture that is not allowed in this temporal layer due to an earlier STSA picture");
  }

  // If the current picture is an STSA picture, make all reference pictures in the DPB with temporal
  // id equal to the temproal id of the current picture sub-layer non-reference pictures. The flag
  // subLayerNonReferencePictureDueToSTSA equal to true means that the picture may not be used for
  // reference by a picture that follows the current STSA picture in decoding order
  if (getNalUnitType() == NAL_UNIT_CODED_SLICE_STSA)
  {
    for( auto & pcPic: rcListPic )
    {
      if( !pcPic->referenced || pcPic->getPOC() == m_iPOC )
      {
        continue;
      }

      if( pcPic->layer == m_uiTLayer )
      {
        pcPic->subLayerNonReferencePictureDueToSTSA = true;
      }
    }
  }
}

void Slice::checkRPL(const ReferencePictureList* pRPL0, const ReferencePictureList* pRPL1, const int associatedIRAPDecodingOrderNumber, const PicListRange& rcListPic)
{
  Picture* pcRefPic;
  int refPicPOC;
  int refPicDecodingOrderNumber;

  int irapPOC = getAssociatedIRAPPOC();
  
  const int                   numEntries[]       = { pRPL0->getNumberOfShorttermPictures() + pRPL0->getNumberOfLongtermPictures() + pRPL0->getNumberOfInterLayerPictures(),
                                                     pRPL1->getNumberOfShorttermPictures() + pRPL1->getNumberOfLongtermPictures() + pRPL1->getNumberOfInterLayerPictures() };
  const int numActiveEntries[] = { getNumRefIdx( REF_PIC_LIST_0 ), getNumRefIdx( REF_PIC_LIST_1 ) };
  const ReferencePictureList* rpl[] = { pRPL0, pRPL1 };
  const bool fieldSeqFlag = getSPS()->getFieldSeqFlag();
  const int layerIdx = m_pcPic->cs->vps == nullptr ? 0 : m_pcPic->cs->vps->getGeneralLayerIdx( m_pcPic->layerId );

  for( int refPicList = 0; refPicList < 2; refPicList++ )
  {
    for( int i = 0; i < numEntries[refPicList]; i++ )
    {
      if( rpl[refPicList]->isInterLayerRefPic( i ) )
      {
        int refLayerId = m_pcPic->cs->vps->getLayerId( m_pcPic->cs->vps->getDirectRefLayerIdx( layerIdx, rpl[refPicList]->getInterLayerRefPicIdx( i ) ) );
        pcRefPic = xGetRefPic( rcListPic, getPOC(), refLayerId );
        refPicPOC = pcRefPic->getPOC();
      }
      else if( !rpl[refPicList]->isRefPicLongterm( i ) )
      {
        refPicPOC = getPOC() + rpl[refPicList]->getRefPicIdentifier(i);
        pcRefPic = xGetRefPic( rcListPic, refPicPOC, m_pcPic->layerId );
      }
      else
      {
        int ltrpPoc = rpl[refPicList]->calcLTRefPOC( getPOC(), getSPS()->getBitsForPOC(), i );

        pcRefPic = xGetLongTermRefPic( rcListPic, ltrpPoc, rpl[refPicList]->getDeltaPocMSBPresentFlag( i ), m_pcPic->layerId );
        refPicPOC = pcRefPic->getPOC();
      }
      if( !pcRefPic )
      {
        // can't check decoding order for unavailable reference pictures
        continue;
      }
      refPicDecodingOrderNumber = pcRefPic->getDecodingOrderNumber();

      if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP )
      {
        CHECK( refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber, "When the current picture, with nuh_layer_id equal to a particular value layerId, "
          "is an IRAP picture, there shall be no picture referred to by an entry in RefPicList[ 0 ] that precedes, in output order or decoding order, any preceding IRAP picture "
          "with nuh_layer_id equal to layerId in decoding order (when present)." );
      }

      if( irapPOC < getPOC() && !fieldSeqFlag )
      {
        CHECK( refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber, "When the current picture follows an IRAP picture having the same value "
          "of nuh_layer_id and the leading pictures, if any, associated with that IRAP picture, in both decoding order and output order, there shall be no picture referred "
          "to by an entry in RefPicList[ 0 ] or RefPicList[ 1 ] that precedes that IRAP picture in output order or decoding order." );
      }

      // Generated reference picture does not have picture header
      const bool isGeneratedRefPic = pcRefPic->slices[0]->getPicHeader() ? false : true;

      const bool nonReferencePictureFlag = isGeneratedRefPic ? pcRefPic->slices[0]->getPicHeader()->getNonReferencePictureFlag() : pcRefPic->nonReferencePictureFlag;
      CHECK( pcRefPic == m_pcPic || nonReferencePictureFlag, "The picture referred to by each entry in RefPicList[ 0 ] or RefPicList[ 1 ] shall not be the current picture and shall have ph_non_ref_pic_flag equal to 0" );

      if( i < numActiveEntries[refPicList] )
      {
        if( irapPOC < getPOC() )
        {
          CHECK( refPicPOC < irapPOC || refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber, "When the current picture follows an IRAP picture having the same value "
            "of nuh_layer_id in both decoding order and output order, there shall be no picture referred to by an active entry in RefPicList[ 0 ] or RefPicList[ 1 ] that "
            "precedes that IRAP picture in output order or decoding order." );
        }

        // Checking this: "When the current picture is a RADL picture, there shall be no active entry in RefPicList[ 0 ] or
        // RefPicList[ 1 ] that is any of the following: A picture that precedes the associated IRAP picture in decoding order"
        if( m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL )
        {
          CHECK( refPicDecodingOrderNumber < associatedIRAPDecodingOrderNumber, "RADL picture detected that violate the rule that no active entry in RefPicList[] shall precede the associated IRAP picture in decoding order" );
        }

        CHECK( pcRefPic->layer > m_pcPic->layer, "The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] shall be present in the DPB and shall have TemporalId less than or equal to that of the current picture." );
      }
    }
  }
}

void Slice::copySliceInfo(Slice *pSrc, bool cpyAlmostAll)
{
  CHECK(!pSrc, "Source is NULL");

  int i, j;

  m_iPOC                 = pSrc->m_iPOC;
  m_eNalUnitType         = pSrc->m_eNalUnitType;
  m_eSliceType           = pSrc->m_eSliceType;
  m_iSliceQp             = pSrc->m_iSliceQp;
  m_iSliceQpBase         = pSrc->m_iSliceQpBase;
  m_ChromaQpAdjEnabled              = pSrc->m_ChromaQpAdjEnabled;
  m_deblockingFilterDisable         = pSrc->m_deblockingFilterDisable;
  m_deblockingFilterOverrideFlag    = pSrc->m_deblockingFilterOverrideFlag;
  m_deblockingFilterBetaOffsetDiv2  = pSrc->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2    = pSrc->m_deblockingFilterTcOffsetDiv2;
  m_deblockingFilterCbBetaOffsetDiv2  = pSrc->m_deblockingFilterCbBetaOffsetDiv2;
  m_deblockingFilterCbTcOffsetDiv2    = pSrc->m_deblockingFilterCbTcOffsetDiv2;
  m_deblockingFilterCrBetaOffsetDiv2  = pSrc->m_deblockingFilterCrBetaOffsetDiv2;
  m_deblockingFilterCrTcOffsetDiv2    = pSrc->m_deblockingFilterCrTcOffsetDiv2;
  m_depQuantEnabledFlag               = pSrc->m_depQuantEnabledFlag;
  m_signDataHidingEnabledFlag         = pSrc->m_signDataHidingEnabledFlag;
  m_tsResidualCodingDisabledFlag      = pSrc->m_tsResidualCodingDisabledFlag;

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]     = pSrc->m_aiNumRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)
  {
    m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i];
  }

  m_bCheckLDC            = pSrc->m_bCheckLDC;
  m_iSliceQpDelta        = pSrc->m_iSliceQpDelta;

  m_biDirPred = pSrc->m_biDirPred;
  m_symRefIdx[0] = pSrc->m_symRefIdx[0];
  m_symRefIdx[1] = pSrc->m_symRefIdx[1];

  for (uint32_t component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = pSrc->m_iSliceChromaQpDelta[component];
  }
  m_iSliceChromaQpDelta[JOINT_CbCr] = pSrc->m_iSliceChromaQpDelta[JOINT_CbCr];

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      m_apcRefPicList[i][j]     = pSrc->m_apcRefPicList[i][j];
      m_aiRefPOCList[i][j]      = pSrc->m_aiRefPOCList[i][j];
      m_bIsUsedAsLongTerm[i][j] = pSrc->m_bIsUsedAsLongTerm[i][j];
    }

    m_apcRefPicList[i][MAX_NUM_REF]     = pSrc->m_apcRefPicList[i][MAX_NUM_REF];
    m_bIsUsedAsLongTerm[i][MAX_NUM_REF] = pSrc->m_bIsUsedAsLongTerm[i][MAX_NUM_REF];
  }

  // access channel
  if( cpyAlmostAll )
  {
    memcpy( m_RPL, pSrc->m_RPL, sizeof( m_RPL ) );
  }

  m_iLastIDR             = pSrc->m_iLastIDR;

  if( cpyAlmostAll ) m_pcPic  = pSrc->m_pcPic;

  m_pcPicHeader          = pSrc->m_pcPicHeader;

  m_colFromL0Flag        = pSrc->m_colFromL0Flag;
  m_colRefIdx            = pSrc->m_colRefIdx;

  if( cpyAlmostAll ) setLambdas(pSrc->getLambdas());

  m_uiTLayer                      = pSrc->m_uiTLayer;
  m_bTLayerSwitchingFlag          = pSrc->m_bTLayerSwitchingFlag;

//  m_sliceCurStartCtuTsAddr        = pSrc->m_sliceCurStartCtuTsAddr;
//  m_sliceCurEndCtuTsAddr          = pSrc->m_sliceCurEndCtuTsAddr;
  m_sliceMap                      = pSrc->m_sliceMap;
  m_independentSliceIdx           = pSrc->m_independentSliceIdx;
  m_nextSlice                     = pSrc->m_nextSlice;
  m_clpRngs                       = pSrc->m_clpRngs;
  m_lmcsEnabledFlag               = pSrc->m_lmcsEnabledFlag;
  m_pendingRasInit                = pSrc->m_pendingRasInit;

  for ( uint32_t e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( uint32_t n=0 ; n<MAX_NUM_REF ; n++ )
    {
      memcpy(m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof(WPScalingParam)*MAX_NUM_COMPONENT );
    }
  }

  for( uint32_t ch = 0 ; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = pSrc->m_saoEnabledFlag[ch];
  }

  m_cabacInitFlag                 = pSrc->m_cabacInitFlag;
  m_cabacWinUpdateMode            = pSrc->m_cabacWinUpdateMode;
  memcpy( m_alfApss,                 pSrc->m_alfApss,                 sizeof( m_alfApss ) );
  memcpy( m_tileGroupAlfEnabledFlag, pSrc->m_tileGroupAlfEnabledFlag, sizeof(m_tileGroupAlfEnabledFlag));
  m_tileGroupNumAps               = pSrc->m_tileGroupNumAps;
  m_tileGroupLumaApsId            = pSrc->m_tileGroupLumaApsId;
  m_tileGroupChromaApsId          = pSrc->m_tileGroupChromaApsId;

  if( cpyAlmostAll ) m_encCABACTableIdx  = pSrc->m_encCABACTableIdx;

  m_tileGroupCcAlfEnabledFlags[0]          = pSrc->m_tileGroupCcAlfEnabledFlags[0];
  m_tileGroupCcAlfEnabledFlags[1]          = pSrc->m_tileGroupCcAlfEnabledFlags[1];
  m_tileGroupCcAlfCbApsId                   = pSrc->m_tileGroupCcAlfCbApsId;
  m_tileGroupCcAlfCrApsId                   = pSrc->m_tileGroupCcAlfCrApsId;
}

void Slice::checkLeadingPictureRestrictions( const PicListRange & rcListPic ) const
{
  int nalUnitType = this->getNalUnitType();

  // When a picture is a leading picture, it shall be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() > this->getPOC())
  {
    // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
    if (nalUnitType < NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
        nalUnitType > NAL_UNIT_CODED_SLICE_CRA)
    {
      CHECK(nalUnitType != NAL_UNIT_CODED_SLICE_RASL &&
            nalUnitType != NAL_UNIT_CODED_SLICE_RADL, "Invalid NAL unit type");
    }
  }

  // When a picture is a trailing picture, it shall not be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() < this->getPOC())
  {
    CHECK(nalUnitType == NAL_UNIT_CODED_SLICE_RASL ||
          nalUnitType == NAL_UNIT_CODED_SLICE_RADL, "Invalid NAL unit type");
  }


  // No RASL pictures shall be present in the bitstream that are associated with
  // an IDR picture.
  if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL)
  {
    CHECK( this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL, "Invalid NAL unit type");
  }

  // No RADL pictures shall be present in the bitstream that are associated with
  // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
  // with an IDR picture having nal_unit_type equal to IDR_N_LP.
  if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
  {
    CHECK (this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP, "Invalid NAL unit type");
  }

  // loop through all pictures in the reference picture buffer
  for( auto & pcPic: rcListPic )
  {
    if( !pcPic->reconstructed || pcPic->wasLost )
    {
      continue;
    }
    if( pcPic->poc == this->getPOC())
    {
      continue;
    }
    const Slice* pcSlice = pcPic->slices[0];

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede the IRAP picture in output order.
    // (Note that any picture following in output order would be present in the DPB)
//    if(pcSlice->getPicHeader()->getPicOutputFlag() == 1 && !this->getPicHeader()->getNoOutputOfPriorPicsFlag())
    if(pcSlice->getPicHeader()->getPicOutputFlag() == 1 && !this->getNoOutputOfPriorPicsFlag()) 
    {
      if (nalUnitType == NAL_UNIT_CODED_SLICE_CRA ||
          nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
          nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL)
      {
        CHECK(pcPic->poc >= this->getPOC(), "Invalid POC");
      }
    }

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede any RADL picture associated with the IRAP
    // picture in output order.
    if(pcSlice->getPicHeader()->getPicOutputFlag() == 1)
    {
      if (nalUnitType == NAL_UNIT_CODED_SLICE_RADL)
      {
        // rpcPic precedes the IRAP in decoding order
        if(this->getAssociatedIRAPPOC() > pcSlice->getAssociatedIRAPPOC())
        {
          // rpcPic must not be the IRAP picture
          if(this->getAssociatedIRAPPOC() != pcPic->poc)
          {
            CHECK( pcPic->poc >= this->getPOC(), "Invalid POC");
          }
        }
      }
    }

    // When a picture is a leading picture, it shall precede, in decoding order,
    // all trailing pictures that are associated with the same IRAP picture.
    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL ||
        nalUnitType == NAL_UNIT_CODED_SLICE_RADL )
      {
        if(pcSlice->getAssociatedIRAPPOC() == this->getAssociatedIRAPPOC())
        {
          // rpcPic is a picture that preceded the leading in decoding order since it exist in the DPB
          // rpcPic would violate the constraint if it was a trailing picture
          CHECK( pcPic->poc > this->getAssociatedIRAPPOC(), "Invalid POC");
        }
      }

    // Any RASL picture associated with a CRA or BLA picture shall precede any
    // RADL picture associated with the CRA or BLA picture in output order
    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL)
    {
      if ((this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA) &&
          this->getAssociatedIRAPPOC() == pcSlice->getAssociatedIRAPPOC())
      {
        if (pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL)
        {
          CHECK( pcPic->poc <= this->getPOC(), "Invalid POC");
        }
      }
    }

    // Any RASL picture associated with a CRA picture shall follow, in output
    // order, any IRAP picture that precedes the CRA picture in decoding order.
    if (nalUnitType == NAL_UNIT_CODED_SLICE_RASL)
    {
      if(this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)
      {
        if(pcSlice->getPOC() < this->getAssociatedIRAPPOC() &&
          (
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA))
        {
          CHECK(this->getPOC() <= pcSlice->getPOC(), "Invalid POC");
        }
      }
    }
  }
}

bool Slice::checkThatAllRefPicsAreAvailable( const PicListRange&         rcListPic,
                                             const ReferencePictureList* pRPL,
                                             int                         numActiveRefPics,
                                             int*                        missingPOC,
                                             int*                        missingRefPicIndex ) const
{
  if( this->isIDR() )
    return true;   // Assume that all pic in the DPB will be flushed anyway so no need to check.

  *missingPOC         = 0;
  *missingRefPicIndex = 0;

  // Check long term ref pics
  for( int ii = 0; pRPL->getNumberOfLongtermPictures() > 0 && ii < numActiveRefPics; ii++ )
  {
    if( !pRPL->isRefPicLongterm( ii ) )
      continue;

    const int notPresentPoc = pRPL->getRefPicIdentifier( ii );
    bool      isAvailable   = 0;
    for( auto& rpcPic: rcListPic )
    {
      const int bitsForPoc = rpcPic->cs->sps->getBitsForPOC();
      const int poc        = rpcPic->getPOC();
      const int refPoc     = pRPL->calcLTRefPOC( this->getPOC(), bitsForPoc, ii );

      if( rpcPic->longTerm && isLTPocEqual( poc, refPoc, bitsForPoc, pRPL->getDeltaPocMSBPresentFlag( ii ) ) && rpcPic->referenced )
      {
        isAvailable = 1;
        break;
      }
    }
    if( isAvailable )
      continue;

    // if there was no such long-term check the short terms
    for( auto& rpcPic: rcListPic )
    {
      const int bitsForPoc = rpcPic->cs->sps->getBitsForPOC();
      const int poc        = rpcPic->getPOC();
      const int refPoc     = pRPL->calcLTRefPOC( this->getPOC(), bitsForPoc, ii );

      if( !rpcPic->longTerm && isLTPocEqual( poc, refPoc, bitsForPoc, pRPL->getDeltaPocMSBPresentFlag( ii ) ) && rpcPic->referenced )
      {
        isAvailable      = 1;
        rpcPic->longTerm = true;
        break;
      }
    }

    if( !isAvailable )
    {
      msg( ERROR, "Current picture: %d Long-term reference picture with POC = %3d seems to have been removed or not correctly decoded.\n", this->getPOC(), notPresentPoc );

      *missingPOC         = notPresentPoc;
      *missingRefPicIndex = ii;
      return false;
    }
  }
  // report that a picture is lost if it is in the Reference Picture List but not in the DPB

  // Check short term ref pics
  for( int ii = 0; ii < numActiveRefPics; ii++ )
  {
    if( pRPL->isRefPicLongterm( ii ) )
      continue;

    const int notPresentPoc = this->getPOC() + pRPL->getRefPicIdentifier( ii );
    bool      isAvailable   = 0;
    for( auto& rpcPic: rcListPic )
    {
      if( !rpcPic->longTerm && rpcPic->getPOC() == notPresentPoc && rpcPic->referenced )
      {
        isAvailable = 1;
        break;
      }
    }

    // report that a picture is lost if it is in the Reference Picture List but not in the DPB
    if( !isAvailable && pRPL->getNumberOfShorttermPictures() > 0 )
    {
      msg( ERROR, "Current picture: %d Short-term reference picture with POC = %3d seems to have been removed or not correctly decoded.\n", this->getPOC(), notPresentPoc );

      *missingPOC         = notPresentPoc;
      *missingRefPicIndex = ii;
      return false;
    }
  }

  return true;
}

//! get AC and DC values for weighted pred
void  Slice::getWpAcDcParam(const WPACDCParam *&wp) const
{
  wp = m_weightACDCParam;
}

//! init AC and DC values for weighted pred
void  Slice::initWpAcDcParam()
{
  for(int iComp = 0; iComp < MAX_NUM_COMPONENT; iComp++ )
  {
    m_weightACDCParam[iComp].iAC = 0;
    m_weightACDCParam[iComp].iDC = 0;
  }
}

//! get tables for weighted prediction
void  Slice::getWpScaling( RefPicList e, int iRefIdx, WPScalingParam *&wp ) const
{
  CHECK(e>=NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  wp = (WPScalingParam*) m_weightPredTable[e][iRefIdx];
}

//! reset Default WP tables settings : no weight.
void  Slice::resetWpScaling()
{
  for ( int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        pwp->bPresentFlag      = false;
        pwp->uiLog2WeightDenom = 0;
        pwp->uiLog2WeightDenom = 0;
        pwp->iWeight           = 1;
        pwp->iOffset           = 0;
      }
    }
  }
}

//! init WP table
void  Slice::initWpScaling(const SPS *sps)
{
  const bool bUseHighPrecisionPredictionWeighting = false;// sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();
  for ( int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        if ( !pwp->bPresentFlag )
        {
          // Inferring values not present :
          pwp->iWeight = (1 << pwp->uiLog2WeightDenom);
          pwp->iOffset = 0;
        }

        const int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (sps->getBitDepth(toChannelType(ComponentID(yuv)))-8));

        pwp->w      = pwp->iWeight;
        pwp->o      = pwp->iOffset * offsetScalingFactor; //NOTE: This value of the ".o" variable is never used - .o is set immediately before it gets used
        pwp->shift  = pwp->uiLog2WeightDenom;
        pwp->round  = (pwp->uiLog2WeightDenom>=1) ? (1 << (pwp->uiLog2WeightDenom-1)) : (0);
      }
    }
  }
}

unsigned Slice::getMinPictureDistance() const
{
  int minPicDist = MAX_INT;
  if (getSPS()->getIBCFlag())
  {
    minPicDist = 0;
  }
  else
  if( ! isIntra() )
  {
    const int currPOC  = getPOC();
    for (int refIdx = 0; refIdx < getNumRefIdx(REF_PIC_LIST_0); refIdx++)
    {
      minPicDist = std::min( minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_0, refIdx)->getPOC()) );
    }
    if( getSliceType() == B_SLICE )
    {
      for (int refIdx = 0; refIdx < getNumRefIdx(REF_PIC_LIST_1); refIdx++)
      {
        minPicDist = std::min( minPicDist, std::abs(currPOC - getRefPic(REF_PIC_LIST_1, refIdx)->getPOC()) );
      }
    }
  }
  return (unsigned) minPicDist;
}

void PicHeader::getWpScaling(RefPicList e, int iRefIdx, WPScalingParam *&wp) const
{
  CHECK(e >= NUM_REF_PIC_LIST_01, "Invalid picture reference list");
  wp = (WPScalingParam *) m_weightPredTable[e][iRefIdx];
}

// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)
// ------------------------------------------------------------------------------------------------



void  SPS::createRPLList0( int numRPL )
{
  m_RPLList0.resize( numRPL );
  m_numRPL0 = numRPL;
  m_rpl1IdxPresentFlag = ( m_RPLList0.size() != m_RPLList1.size() );
}

void  SPS::createRPLList1( int numRPL )
{
  m_RPLList1.resize( numRPL );
  m_numRPL1 = numRPL;
  m_rpl1IdxPresentFlag = ( m_RPLList0.size() != m_RPLList1.size() );
}

const int SPS::m_winUnitX[] = { 1,2,2,1 };
const int SPS::m_winUnitY[] = { 1,2,1,1 };

void ChromaQpMappingTable::setParams(const ChromaQpMappingTableParams &params, const int qpBdOffset)
{
  m_qpBdOffset = qpBdOffset;
  m_sameCQPTableForAllChromaFlag = params.m_sameCQPTableForAllChromaFlag;
  m_numQpTables = params.m_numQpTables;

  for (int i = 0; i < MAX_NUM_CQP_MAPPING_TABLES; i++)
  {
    m_numPtsInCQPTableMinus1[i] = params.m_numPtsInCQPTableMinus1[i];
    m_deltaQpInValMinus1[i] = params.m_deltaQpInValMinus1[i];
    m_qpTableStartMinus26[i] = params.m_qpTableStartMinus26[i];
    m_deltaQpOutVal[i] = params.m_deltaQpOutVal[i];
    m_chromaQpMappingTables[i].resize( MAX_QP + qpBdOffset + 1 );
  }
}
void ChromaQpMappingTable::derivedChromaQPMappingTables()
{
  for (int i = 0; i < getNumQpTables(); i++)
  {
    const int qpBdOffsetC = m_qpBdOffset;
    const int numPtsInCQPTableMinus1 = getNumPtsInCQPTableMinus1(i);
    std::vector<int> qpInVal(numPtsInCQPTableMinus1 + 2), qpOutVal(numPtsInCQPTableMinus1 + 2);

    qpInVal[0] = getQpTableStartMinus26(i) + 26;
    qpOutVal[0] = qpInVal[0];
    for (int j = 0; j <= getNumPtsInCQPTableMinus1(i); j++)
    {
      qpInVal[j + 1] = qpInVal[j] + getDeltaQpInValMinus1(i, j) + 1;
      qpOutVal[j + 1] = qpOutVal[j] + getDeltaQpOutVal(i, j);
    }

    for (int j = 0; j <= getNumPtsInCQPTableMinus1(i); j++)
    {
      CHECK(qpInVal[j]  < -qpBdOffsetC || qpInVal[j]  > MAX_QP, "qpInVal out of range");
      CHECK(qpOutVal[j] < -qpBdOffsetC || qpOutVal[j] > MAX_QP, "qpOutVal out of range");
    }

    m_chromaQpMappingTables[i][qpInVal[0] + qpBdOffsetC] = qpOutVal[0];
    for (int k = qpInVal[0] - 1; k >= -qpBdOffsetC; k--)
    {
      m_chromaQpMappingTables[i][k + qpBdOffsetC] = Clip3(-qpBdOffsetC, MAX_QP, m_chromaQpMappingTables[i][k + 1 + qpBdOffsetC] - 1);
    }
    for (int j = 0; j <= numPtsInCQPTableMinus1; j++)
    {
      int sh = (getDeltaQpInValMinus1(i, j) + 1) >> 1;
      for (int k = qpInVal[j] + 1, m = 1; k <= qpInVal[j + 1]; k++, m++)
      {
        m_chromaQpMappingTables[i][k + qpBdOffsetC] = m_chromaQpMappingTables[i][qpInVal[j] + qpBdOffsetC]
          + ((qpOutVal[j + 1] - qpOutVal[j]) * m + sh) / (getDeltaQpInValMinus1(i, j) + 1);
      }
    }
    for (int k = qpInVal[numPtsInCQPTableMinus1 + 1] + 1; k <= MAX_QP; k++)
    {
      m_chromaQpMappingTables[i][k + qpBdOffsetC] = Clip3(-qpBdOffsetC, MAX_QP, m_chromaQpMappingTables[i][k - 1 + qpBdOffsetC] + 1);
    }
  }
}


void PPS::resetTileSliceInfo()
{
  m_numExpTileCols = 0;
  m_numExpTileRows = 0;
  m_numTileCols    = 0;
  m_numTileRows    = 0;
  m_numSlicesInPic = 0;
  m_tileColumnWidth.clear();
  m_tileRowHeight.clear();
  m_tileColBd.clear();
  m_tileRowBd.clear();
  m_ctuToTileCol.clear();
  m_ctuToTileRow.clear();
  m_ctuToSubPicIdx.clear();
  m_rectSlices.clear();
  m_sliceMap.clear();
}

/**
 - initialize tile row/column sizes and boundaries
 */
void PPS::initTiles()
{
  int       colIdx, rowIdx;
  int       ctuX, ctuY;
  
  // check explicit tile column sizes
  uint32_t  remainingWidthInCtu  = m_picWidthInCtu;
  for( colIdx = 0; colIdx < m_numExpTileCols; colIdx++ )
  {
    CHECK(m_tileColumnWidth[colIdx] > remainingWidthInCtu,    "Tile column width exceeds picture width");
    remainingWidthInCtu -= m_tileColumnWidth[colIdx];
  }

  // divide remaining picture width into uniform tile columns
  uint32_t  uniformTileColWidth = m_tileColumnWidth[colIdx-1];
  while( remainingWidthInCtu > 0 )
  {
    CHECK(colIdx >= MAX_TILE_COLS, "Number of tile columns exceeds valid range");
    uniformTileColWidth = std::min(remainingWidthInCtu, uniformTileColWidth);
    m_tileColumnWidth.push_back( uniformTileColWidth );
    remainingWidthInCtu -= uniformTileColWidth;
    colIdx++;
  }
  m_numTileCols = colIdx;
    
  // check explicit tile row sizes
  uint32_t  remainingHeightInCtu  = m_picHeightInCtu;
  for( rowIdx = 0; rowIdx < m_numExpTileRows; rowIdx++ )
  {
    CHECK(m_tileRowHeight[rowIdx] > remainingHeightInCtu,     "Tile row height exceeds picture height");
    remainingHeightInCtu -= m_tileRowHeight[rowIdx];
  }
    
  // divide remaining picture height into uniform tile rows
  uint32_t  uniformTileRowHeight = m_tileRowHeight[rowIdx - 1];
  while( remainingHeightInCtu > 0 )
  {
    uniformTileRowHeight = std::min(remainingHeightInCtu, uniformTileRowHeight);
    m_tileRowHeight.push_back( uniformTileRowHeight );
    remainingHeightInCtu -= uniformTileRowHeight;
    rowIdx++;
  }
  m_numTileRows = rowIdx;

  // set left column bounaries
  m_tileColBd.push_back( 0 );
  for( colIdx = 0; colIdx < m_numTileCols; colIdx++ )
  {
    m_tileColBd.push_back( m_tileColBd[ colIdx ] + m_tileColumnWidth[ colIdx ] );
  }
  
  // set top row bounaries
  m_tileRowBd.push_back( 0 );
  for( rowIdx = 0; rowIdx < m_numTileRows; rowIdx++ )
  {
    m_tileRowBd.push_back( m_tileRowBd[ rowIdx ] + m_tileRowHeight[ rowIdx ] );
  }

  // set mapping between horizontal CTU address and tile column index
  colIdx = 0;
  for( ctuX = 0; ctuX <= m_picWidthInCtu; ctuX++ )
  {
    if( ctuX == m_tileColBd[ colIdx + 1 ] )
    {
      colIdx++;
    }
    m_ctuToTileCol.push_back( colIdx );
  }
  
  // set mapping between vertical CTU address and tile row index
  rowIdx = 0;
  for( ctuY = 0; ctuY <= m_picHeightInCtu; ctuY++ )
  {
    if( ctuY == m_tileRowBd[ rowIdx + 1 ] )
    {
      rowIdx++;
    }
    m_ctuToTileRow.push_back( rowIdx );
  }
}

void PPS::initRectSlices()
{
  CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
  m_rectSlices.resize(m_numSlicesInPic);
}

/**
 - initialize mapping between rectangular slices and CTUs
 */
void PPS::initRectSliceMap(const SPS  *sps)
{
  if (sps)
  {
    m_ctuToSubPicIdx.resize(getPicWidthInCtu() * getPicHeightInCtu());
    if (sps->getNumSubPics() > 1)
    {
      for (int i = 0; i <= sps->getNumSubPics() - 1; i++)
      {
        for (int y = sps->getSubPicCtuTopLeftY(i); y < sps->getSubPicCtuTopLeftY(i) + sps->getSubPicHeight(i); y++)
        {
          for (int x = sps->getSubPicCtuTopLeftX(i); x < sps->getSubPicCtuTopLeftX(i) + sps->getSubPicWidth(i); x++)
          {
            m_ctuToSubPicIdx[ x+ y * getPicWidthInCtu()] = i;
          }
        }
      }
    }
    else
    {
      for (int i = 0; i < getPicWidthInCtu() * getPicHeightInCtu(); i++)
      {
        m_ctuToSubPicIdx[i] = 0;
      }
    }
  }

  if( getSingleSlicePerSubPicFlag() )
  {
    CHECK (sps==nullptr, "RectSliceMap can only be initialized for slice_per_sub_pic_flag with a valid SPS");
    m_numSlicesInPic = sps->getNumSubPics();

    // allocate new memory for slice list
    CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
    m_sliceMap.resize( m_numSlicesInPic );

    if (sps->getNumSubPics() > 1)
    {
      // Q2001 v15 equation 29
      std::vector<uint32_t> subpicWidthInTiles;
      std::vector<uint32_t> subpicHeightInTiles;
      std::vector<uint32_t> subpicHeightLessThanOneTileFlag;
      subpicWidthInTiles.resize(sps->getNumSubPics());
      subpicHeightInTiles.resize(sps->getNumSubPics());
      subpicHeightLessThanOneTileFlag.resize(sps->getNumSubPics());
      for (uint32_t i = 0; i <sps->getNumSubPics(); i++)
      {
        uint32_t leftX = sps->getSubPicCtuTopLeftX(i);
        uint32_t rightX = leftX + sps->getSubPicWidth(i) - 1;
        subpicWidthInTiles[i] = m_ctuToTileCol[rightX] + 1 - m_ctuToTileCol[leftX];

        uint32_t topY = sps->getSubPicCtuTopLeftY(i);
        uint32_t bottomY = topY + sps->getSubPicHeight(i) - 1;
        subpicHeightInTiles[i] = m_ctuToTileRow[bottomY] + 1 - m_ctuToTileRow[topY];

        if (subpicHeightInTiles[i] == 1 && sps->getSubPicHeight(i) < m_tileRowHeight[m_ctuToTileRow[topY]] )
        {
          subpicHeightLessThanOneTileFlag[i] = 1;
        }
        else
        {
          subpicHeightLessThanOneTileFlag[i] = 0;
        }
      }

      for( int i = 0; i < m_numSlicesInPic; i++ )
      {
        CHECK(m_numSlicesInPic != sps->getNumSubPics(), "in single slice per subpic mode, number of slice and subpic shall be equal");
        m_sliceMap[ i ].initSliceMap();
        if (subpicHeightLessThanOneTileFlag[i])
        {
          m_sliceMap[i].addCtusToSlice(sps->getSubPicCtuTopLeftX(i), sps->getSubPicCtuTopLeftX(i) + sps->getSubPicWidth(i),
                                       sps->getSubPicCtuTopLeftY(i), sps->getSubPicCtuTopLeftY(i) + sps->getSubPicHeight(i), m_picWidthInCtu);
        }
        else
        {
          uint32_t tileX = m_ctuToTileCol[sps->getSubPicCtuTopLeftX(i)];
          uint32_t tileY = m_ctuToTileRow[sps->getSubPicCtuTopLeftY(i)];
          for (uint32_t j = 0; j< subpicHeightInTiles[i]; j++)
          {
            for (uint32_t k = 0; k < subpicWidthInTiles[i]; k++)
            {
              m_sliceMap[i].addCtusToSlice(getTileColumnBd(tileX + k), getTileColumnBd(tileX + k + 1), getTileRowBd(tileY + j), getTileRowBd(tileY + j + 1), m_picWidthInCtu);
            }
          }
        }
      }
      subpicWidthInTiles.clear();
      subpicHeightInTiles.clear();
      subpicHeightLessThanOneTileFlag.clear();
    }
    else
    {
      m_sliceMap[0].initSliceMap();
      for (int tileY=0; tileY<m_numTileRows; tileY++)
      {
        for (int tileX=0; tileX<m_numTileCols; tileX++)
        {
          m_sliceMap[0].addCtusToSlice(getTileColumnBd(tileX), getTileColumnBd(tileX + 1),
                                       getTileRowBd(tileY), getTileRowBd(tileY + 1), m_picWidthInCtu);
        }
      }
      m_sliceMap[0].setSliceID(0);
    }
  }
  else
  {
    // allocate new memory for slice list
    CHECK(m_numSlicesInPic > MAX_SLICES, "Number of slices in picture exceeds valid range");
    m_sliceMap.resize( m_numSlicesInPic );
    // generate CTU maps for all rectangular slices in picture
    for( uint32_t i = 0; i < m_numSlicesInPic; i++ )
    {
      m_sliceMap[ i ].initSliceMap();

      // get position of first tile in slice
      uint32_t tileX =  m_rectSlices[ i ].getTileIdx() % m_numTileCols;
      uint32_t tileY =  m_rectSlices[ i ].getTileIdx() / m_numTileCols;

      // infer slice size for last slice in picture
      if( i == m_numSlicesInPic-1 )
      {
        m_rectSlices[ i ].setSliceWidthInTiles ( m_numTileCols - tileX );
        m_rectSlices[ i ].setSliceHeightInTiles( m_numTileRows - tileY );
        m_rectSlices[ i ].setNumSlicesInTile( 1 );
      }

      // set slice index
      m_sliceMap[ i ].setSliceID(i);

      // complete tiles within a single slice case
      if( m_rectSlices[ i ].getSliceWidthInTiles( ) > 1 || m_rectSlices[ i ].getSliceHeightInTiles( ) > 1)
      {
        for( uint32_t j = 0; j < m_rectSlices[ i ].getSliceHeightInTiles( ); j++ )
        {
          for( uint32_t k = 0; k < m_rectSlices[ i ].getSliceWidthInTiles( ); k++ )
          {
            m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX + k), getTileColumnBd(tileX + k +1),
                                            getTileRowBd(tileY + j), getTileRowBd(tileY + j +1), m_picWidthInCtu);
          }
        }
      }
      // multiple slices within a single tile case
      else
      {
        uint32_t  numSlicesInTile = m_rectSlices[ i ].getNumSlicesInTile( );

        uint32_t ctuY = getTileRowBd( tileY );
        for( uint32_t j = 0; j < numSlicesInTile-1; j++ )
        {
          m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX), getTileColumnBd(tileX+1),
                                          ctuY, ctuY + m_rectSlices[ i ].getSliceHeightInCtu(), m_picWidthInCtu);
          ctuY += m_rectSlices[ i ].getSliceHeightInCtu();
          i++;
          m_sliceMap[ i ].initSliceMap();
          m_sliceMap[ i ].setSliceID(i);
        }

        // infer slice height for last slice in tile
        CHECK( ctuY >= getTileRowBd( tileY + 1 ), "Invalid rectangular slice signalling");
        m_rectSlices[ i ].setSliceHeightInCtu( getTileRowBd( tileY + 1 ) - ctuY );
        m_sliceMap[ i ].addCtusToSlice( getTileColumnBd(tileX), getTileColumnBd(tileX+1),
                                        ctuY, getTileRowBd( tileY + 1 ), m_picWidthInCtu);
      }
    }
  }
  // check for valid rectangular slice map
  checkSliceMap();
}

/**
- initialize mapping between subpicture and CTUs
*/
void PPS::initSubPic( const SPS &sps )
{
  if( getSubPicIdMappingInPpsFlag() )
  {
    // When signalled, the number of subpictures has to match in PPS and SPS
    CHECK( getNumSubPics() != sps.getNumSubPics(), "pps_num_subpics_minus1 shall be equal to sps_num_subpics_minus1" );
  }
  else
  {
    // When not signalled  set the numer equal for convenient access
    setNumSubPics( sps.getNumSubPics() );
  }

  CHECK( getNumSubPics() > MAX_NUM_SUB_PICS, "Number of sub-pictures in picture exceeds valid range" );
  m_subPics.resize(getNumSubPics());

  // Check that no subpicture is specified outside of the conformance cropping window
  for(int i = 0; i < sps.getNumSubPics(); i++)
  {
    CHECK( (sps.getSubPicCtuTopLeftX(i) * sps.getCTUSize()) >=
          (sps.getMaxPicWidthInLumaSamples() - sps.getConformanceWindow().getWindowRightOffset() * SPS::getWinUnitX(sps.getChromaFormatIdc())),
          "No subpicture can be located completely outside of the conformance cropping window");
    CHECK( ((sps.getSubPicCtuTopLeftX(i) + sps.getSubPicWidth(i)) * sps.getCTUSize()) <= (sps.getConformanceWindow().getWindowLeftOffset() * SPS::getWinUnitX(sps.getChromaFormatIdc())),
          "No subpicture can be located completely outside of the conformance cropping window" );
    CHECK( (sps.getSubPicCtuTopLeftY(i) * sps.getCTUSize()) >=
          (sps.getMaxPicHeightInLumaSamples()  - sps.getConformanceWindow().getWindowBottomOffset() * SPS::getWinUnitY(sps.getChromaFormatIdc())),
          "No subpicture can be located completely outside of the conformance cropping window");
    CHECK( ((sps.getSubPicCtuTopLeftY(i) + sps.getSubPicHeight(i)) * sps.getCTUSize()) <= (sps.getConformanceWindow().getWindowTopOffset() * SPS::getWinUnitY(sps.getChromaFormatIdc())),
          "No subpicture can be located completely outside of the conformance cropping window");
  }

  // m_ctuSize,  m_picWidthInCtu, and m_picHeightInCtu might not be initialized yet.
  if( m_ctuSize == 0 || m_picWidthInCtu == 0 || m_picHeightInCtu == 0 )
  {
    m_ctuSize = sps.getCTUSize();
    m_picWidthInCtu = (m_picWidthInLumaSamples + m_ctuSize - 1) / m_ctuSize;
    m_picHeightInCtu = (m_picHeightInLumaSamples + m_ctuSize - 1) / m_ctuSize;
  }
  for( int i=0; i< getNumSubPics(); i++ )
  {
    m_subPics[i].setSubPicIdx(i);
    if( sps.getSubPicIdMappingExplicitlySignalledFlag() )
    {
      if( m_subPicIdMappingInPpsFlag )
      {
        m_subPics[i].setSubPicID( m_subPicId[i] );
      }
      else
      {
        m_subPics[i].setSubPicID( sps.getSubPicId(i) );
      }
    }
    else
    {
      m_subPics[i].setSubPicID(i);
    }
    m_subPics[i].setSubPicCtuTopLeftX(sps.getSubPicCtuTopLeftX(i));
    m_subPics[i].setSubPicCtuTopLeftY(sps.getSubPicCtuTopLeftY(i));
    m_subPics[i].setSubPicWidthInCTUs(sps.getSubPicWidth(i));
    m_subPics[i].setSubPicHeightInCTUs(sps.getSubPicHeight(i));

    uint32_t firstCTU = sps.getSubPicCtuTopLeftY(i) * m_picWidthInCtu + sps.getSubPicCtuTopLeftX(i);
    m_subPics[i].setFirstCTUInSubPic(firstCTU);
    uint32_t lastCTU = (sps.getSubPicCtuTopLeftY(i) + sps.getSubPicHeight(i) - 1) * m_picWidthInCtu + sps.getSubPicCtuTopLeftX(i) + sps.getSubPicWidth(i) - 1;
    m_subPics[i].setLastCTUInSubPic(lastCTU);

    uint32_t left = sps.getSubPicCtuTopLeftX(i) * m_ctuSize;
    m_subPics[i].setSubPicLeft(left);

    uint32_t right = std::min(m_picWidthInLumaSamples - 1, (sps.getSubPicCtuTopLeftX(i) + sps.getSubPicWidth(i)) * m_ctuSize - 1);
    m_subPics[i].setSubPicRight(right);

    m_subPics[i].setSubPicWidthInLumaSample(right - left + 1);

    uint32_t top = sps.getSubPicCtuTopLeftY(i) * m_ctuSize;
    m_subPics[i].setSubPicTop(top);

    uint32_t bottom = std::min(m_picHeightInLumaSamples - 1, (sps.getSubPicCtuTopLeftY(i) + sps.getSubPicHeight(i)) * m_ctuSize - 1);

    m_subPics[i].setSubPicHeightInLumaSample(bottom - top + 1);

    m_subPics[i].setSubPicBottom(bottom);

    m_subPics[i].clearCTUAddrList();

    if( m_numSlicesInPic == 1 )
    {
      CHECK( getNumSubPics() != 1, "only one slice in picture, but number of subpic is not one" );
      m_subPics[i].addAllCtusInPicToSubPic(0, getPicWidthInCtu(), 0, getPicHeightInCtu(), getPicWidthInCtu());
      m_subPics[i].setNumSlicesInSubPic(1);
    }
    else
    {
      int numSlicesInSubPic = 0;
      int idxLastSliceInSubpic = -1;
      int idxFirstSliceAfterSubpic = m_numSlicesInPic;
      for( int j = 0; j < m_numSlicesInPic; j++ )
      {
        uint32_t ctu = m_sliceMap[j].getCtuAddrInSlice(0);
        uint32_t ctu_x = ctu % m_picWidthInCtu;
        uint32_t ctu_y = ctu / m_picWidthInCtu;
        if (ctu_x >= sps.getSubPicCtuTopLeftX(i) &&
          ctu_x < (sps.getSubPicCtuTopLeftX(i) + sps.getSubPicWidth(i)) &&
          ctu_y >= sps.getSubPicCtuTopLeftY(i) &&
          ctu_y < (sps.getSubPicCtuTopLeftY(i) + sps.getSubPicHeight(i)))
        {
          // add ctus in a slice to the subpicture it belongs to
          m_subPics[i].addCTUsToSubPic(m_sliceMap[j].getCtuAddrList());
          numSlicesInSubPic++;
          idxLastSliceInSubpic = j;
        }
        else if (idxFirstSliceAfterSubpic == m_numSlicesInPic && idxLastSliceInSubpic != -1)
        {
          idxFirstSliceAfterSubpic = j;
        }
      }
      CHECK( idxFirstSliceAfterSubpic < idxLastSliceInSubpic, "The signalling order of slices shall follow the coding order" );
      m_subPics[i].setNumSlicesInSubPic(numSlicesInSubPic);
    }
    m_subPics[i].setTreatedAsPicFlag(sps.getSubPicTreatedAsPicFlag(i));
    m_subPics[i].setloopFilterAcrossSubPicEnabledFlag(sps.getLoopFilterAcrossSubpicEnabledFlag(i));
  }
}

const SubPic& PPS::getSubPicFromPos(const Position& pos)  const
{
  for (int i = 0; i< m_numSubPics; i++)
  {
    if (m_subPics[i].isContainingPos(pos))
    {
      return m_subPics[i];
    }
  }
  return m_subPics[0];
}

const SubPic& PPS::getSubPicFromCU(const CodingUnit& cu) const
{
  const Position lumaPos = cu.Y().valid() ? cu.Y().pos() : recalcPosition(cu.chromaFormat, cu.chType(), CHANNEL_TYPE_LUMA, cu.blocks[cu.chType()].pos());
  return getSubPicFromPos(lumaPos);
}

uint32_t PPS::getSubPicIdxFromSubPicId( uint32_t subPicId ) const
{
  for (int i = 0; i < m_numSubPics; i++)
  {
    if(m_subPics[i].getSubPicID() == subPicId)
    {
      return i;
    }
  }
  return 0;
}

void PPS::checkSliceMap()
{
  uint32_t i;
  std::vector<uint32_t>  ctuList, sliceList;
  uint32_t picSizeInCtu = getPicWidthInCtu() * getPicHeightInCtu();
  for( i = 0; i < m_numSlicesInPic; i++ )
  {
    sliceList = m_sliceMap[ i ].getCtuAddrList();
    ctuList.insert( ctuList.end(), sliceList.begin(), sliceList.end() );
  }
  CHECK( ctuList.size() < picSizeInCtu, "Slice map contains too few CTUs");
  CHECK( ctuList.size() > picSizeInCtu, "Slice map contains too many CTUs");
  std::sort( ctuList.begin(), ctuList.end() );
  for( i = 1; i < ctuList.size(); i++ )
  {
    CHECK( ctuList[i] > ctuList[i-1]+1, "CTU missing in slice map");
    CHECK( ctuList[i] == ctuList[i-1],  "CTU duplicated in slice map");
  }
}

void PPS::finalizePPSPartitioning( const SPS* pcSPS )
{
  if( m_partitioningInitialized ) return;

  // initialize tile/slice info for no partitioning case
  if( getNoPicPartitionFlag() )
  {
    resetTileSliceInfo();
    setLog2CtuSize( ( int ) ceil( log2( pcSPS->getCTUSize() ) ) );
    setNumExpTileColumns( 1 );
    setNumExpTileRows( 1 );
    addTileColumnWidth( getPicWidthInCtu()  );
    addTileRowHeight  ( getPicHeightInCtu() );
    initTiles();
    setRectSliceFlag( 1 );
    setNumSlicesInPic( 1 );
    initRectSlices();
    setTileIdxDeltaPresentFlag( 0 );
    setSliceTileIdx( 0, 0 );
    initRectSliceMap( pcSPS );
    // when no Pic partition, number of sub picture shall be less than 2
    CHECK( getNumSubPics() >= 2, "error, no picture partitions, but have equal to or more than 2 sub pictures" );
  }
  else
  {
    CHECK( getCtuSize() != pcSPS->getCTUSize(), "PPS CTU size does not match CTU size in SPS" );
    if( getRectSliceFlag() )
    {
      initRectSliceMap( pcSPS );
    }
  }

  initSubPic( *pcSPS );

  m_partitioningInitialized = true;
}


SliceMap::SliceMap()
//: m_sliceID              (0)
//, m_numTilesInSlice      (0)
//, m_numCtuInSlice        (0)
{
  m_ctuAddrInSlice.clear();
}

SliceMap::~SliceMap()
{
  m_numCtuInSlice = 0;
  m_ctuAddrInSlice.clear();
}

/** Sorts the deltaPOC and Used by current values in the RPS based on the deltaPOC values.
 *  deltaPOC values are sorted with -ve values before the +ve values.  -ve values are in decreasing order.
 *  +ve values are in increasing order.
 * \returns void
 */

ReferencePictureList::ReferencePictureList()
{
  ::memset( this, 0, sizeof( *this ) );
}

void ReferencePictureList::clear()
{
  ::memset( this, 0, sizeof( *this ) );
}

void ReferencePictureList::setRefPicIdentifier( int idx, int identifier, bool isLongterm, bool isInterLayerRefPic, int interLayerIdx )
{
  CHECK( idx > MAX_NUM_REF_PICS, "RPL setRefPicIdentifier out of range (0-15)" );
  m_refPicIdentifier[idx] = identifier;
  m_isLongtermRefPic[idx] = isLongterm;

  m_deltaPocMSBPresentFlag[idx] = false;
  m_deltaPOCMSBCycleLT[idx] = 0;
  
  m_isInterLayerRefPic[idx] = isInterLayerRefPic;
  m_interLayerRefPicIdx[idx] = interLayerIdx;
}

int ReferencePictureList::getRefPicIdentifier(int idx) const
{
  return m_refPicIdentifier[idx];
}


bool ReferencePictureList::isRefPicLongterm(int idx) const
{
  return m_isLongtermRefPic[idx];
}

void ReferencePictureList::setRefPicLongterm(int idx,bool isLongterm)
{
  CHECK( idx > MAX_NUM_REF_PICS, "RPL setRefPicLongterm out of range (0-15)" );
  m_isLongtermRefPic[idx] = isLongterm;
}

void ReferencePictureList::setNumberOfShorttermPictures(int numberOfStrp)
{
  m_numberOfShorttermPictures = numberOfStrp;
}

int ReferencePictureList::getNumberOfShorttermPictures() const
{
  return m_numberOfShorttermPictures;
}

void ReferencePictureList::setNumberOfLongtermPictures(int numberOfLtrp)
{
  m_numberOfLongtermPictures = numberOfLtrp;
}

int ReferencePictureList::getNumberOfLongtermPictures() const
{
  return m_numberOfLongtermPictures;
}

void ReferencePictureList::setPOC(int idx, int POC)
{
  CHECK( idx > MAX_NUM_REF_PICS, "RPL setPOC out of range (0-15)" );
  m_POC[idx] = POC;
}

int ReferencePictureList::getPOC(int idx) const
{
  return m_POC[idx];
}

void ReferencePictureList::setDeltaPocMSBCycleLT(int idx, int x)
{
  CHECK( idx > MAX_NUM_REF_PICS, "RPL setDeltaPocMSBCycleLT out of range (0-15)" );
  m_deltaPOCMSBCycleLT[idx] = x;
}

void ReferencePictureList::setDeltaPocMSBPresentFlag(int idx, bool x)
{
  CHECK( idx > MAX_NUM_REF_PICS, "RPL setDeltaPocMSBPresentFlag out of range (0-15)" );
  m_deltaPocMSBPresentFlag[idx] = x;
}

void ReferencePictureList::setInterLayerRefPicIdx( int idx, int layerIdc )
{
  CHECK( idx > MAX_NUM_REF_PICS, "RPL setInterLayerRefPicIdx out of range (0-15)" );
  m_interLayerRefPicIdx[idx] = layerIdc;
}

void ReferencePictureList::printRefPicInfo() const
{
  DTRACE(g_trace_ctx, D_RPSINFO, "RefPics = { ");
  int numRefPic = getNumberOfShorttermPictures() + getNumberOfLongtermPictures();
  for (int ii = 0; ii < numRefPic; ii++)
  {
    DTRACE(g_trace_ctx, D_RPSINFO, "%d%s ", m_refPicIdentifier[ii], (m_isLongtermRefPic[ii] == 1) ? "[LT]" : "[ST]");
  }
  DTRACE(g_trace_ctx, D_RPSINFO, "}\n");
}

int ReferencePictureList::calcLTRefPOC( int currPoc, int bitsForPoc, int refPicIdentifier, bool pocMSBPresent, int deltaPocMSBCycle )
{
  const int pocCycle = 1 << bitsForPoc;
  int       ltrpPoc  = refPicIdentifier & ( pocCycle - 1 );
  if( pocMSBPresent )
  {
    ltrpPoc += currPoc - deltaPocMSBCycle * pocCycle - ( currPoc & ( pocCycle - 1 ) );
  }
  return ltrpPoc;
}

int ReferencePictureList::calcLTRefPOC( int currPoc, int bitsForPoc, int refPicIdx ) const
{
  return calcLTRefPOC( currPoc,
                       bitsForPoc,
                       this->getRefPicIdentifier( refPicIdx ),
                       this->getDeltaPocMSBPresentFlag( refPicIdx ),
                       this->getDeltaPocMSBCycleLT( refPicIdx ) );
}

bool isLTPocEqual( int poc1, int poc2, int bitsForPoc, bool msbPresent )
{
  if( msbPresent )
  {
    return poc1 == poc2;
  }

  const int pocCycle = 1 << bitsForPoc;
  return ( poc1 & ( pocCycle - 1 ) ) == ( poc2 & ( pocCycle - 1 ) );
}


ScalingList::ScalingList()
{
  m_chromaScalingListPresentFlag = true;
  memset( m_scalingListDC,                 0, sizeof( m_scalingListDC ) );
  memset( m_refMatrixId,                   0, sizeof( m_refMatrixId ) );

  for (uint32_t scalingListId = 0; scalingListId < 28; scalingListId++)
  {
    int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
    m_scalingListCoef[scalingListId].resize( matrixSize*matrixSize );
  }
}

/** set default quantization matrix to array
*/
void ScalingList::setDefaultScalingList()
{
  for( uint32_t scalingListId = 0; scalingListId < 28; scalingListId++ )
  {
    processDefaultMatrix(scalingListId);
  }
}

/** get scaling matrix from RefMatrixID
 * \param sizeId    size index
 * \param listId    index of input matrix
 * \param refListId index of reference matrix
 */
int ScalingList::lengthUvlc(int uiCode)
{
  CHECK( uiCode < 0, "Error UVLC!" )
  
  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}
int ScalingList::lengthSvlc(int uiCode)
{
  uint32_t uiCode2 = uint32_t(uiCode <= 0 ? (-uiCode) << 1 : (uiCode << 1) - 1);
  int uiLength = 1;
  int uiTemp = ++uiCode2;

  CHECK(!uiTemp, "Integer overflow");

  while (1 != uiTemp)
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  return (uiLength >> 1) + ((uiLength + 1) >> 1);
}
void ScalingList::processRefMatrix(uint32_t scalinListId, uint32_t refListId)
{
  int matrixSize = (scalinListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalinListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  ::memcpy(getScalingListAddress(scalinListId), ((scalinListId == refListId) ? getScalingListDefaultAddress(refListId) : getScalingListAddress(refListId)), sizeof(int)*matrixSize*matrixSize);
}

/** get default address of quantization matrix
 * \param sizeId size index
 * \param listId list index
 * \returns pointer of quantization matrix
 */
const int* ScalingList::getScalingListDefaultAddress(uint32_t scalingListId)
{
  const int *src = 0;
  int sizeId = (scalingListId < SCALING_LIST_1D_START_8x8) ? 2 : 3;
  switch(sizeId)
  {
    case SCALING_LIST_1x1:
    case SCALING_LIST_2x2:
    case SCALING_LIST_4x4:
      src = g_quantTSDefault4x4;
      break;
    case SCALING_LIST_8x8:
    case SCALING_LIST_16x16:
    case SCALING_LIST_32x32:
    case SCALING_LIST_64x64:
      src = g_quantInterDefault8x8;
      break;
    default:
      THROW( "Invalid scaling list" );
      src = NULL;
      break;
  }
  return src;
}

/** process of default matrix
 * \param sizeId size index
 * \param listId index of input matrix
 */
void ScalingList::processDefaultMatrix(uint32_t scalingListId)
{
  int matrixSize = (scalingListId < SCALING_LIST_1D_START_4x4) ? 2 : (scalingListId < SCALING_LIST_1D_START_8x8) ? 4 : 8;
  ::memcpy(getScalingListAddress(scalingListId), getScalingListDefaultAddress(scalingListId), sizeof(int)*matrixSize*matrixSize);
  setScalingListDC(scalingListId, SCALING_LIST_DC);
}

bool ScalingList::isLumaScalingList( int scalingListId) const
{
  return (scalingListId % MAX_NUM_COMPONENT == SCALING_LIST_1D_START_4x4 || scalingListId == SCALING_LIST_1D_START_64x64 + 1);
}

void Slice::scaleRefPicList( PicHeader *picHeader, APS** apss, APS* lmcsAps, APS* scalingListAps )
{
  const SPS* sps = getSPS();
  const PPS* pps = getPPS();

  bool refPicIsSameRes = false;
   
  // this is needed for IBC
  m_pcPic->unscaledPic = m_pcPic;

  if( m_eSliceType == I_SLICE )
  {
    return;
  }
  
  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      // if rescaling is needed, otherwise just reuse the original picture pointer; it is needed for motion field, otherwise motion field requires a copy as well
      // reference resampling for the whole picture is not applied at decoder

      int xScale, yScale;
      CU::getRprScaling( sps, pps, m_apcRefPicList[refList][rIdx]->slices[0]->getPPS(), xScale, yScale );
      m_scalingRatio[refList][rIdx] = std::pair<int, int>( xScale, yScale );

      CHECK( !m_apcRefPicList[refList][rIdx], "scaleRefPicList missing ref pic" );
      if( m_apcRefPicList[refList][rIdx]->isRefScaled( pps ) == false )
      {
        refPicIsSameRes = true;
      }

      m_scaledRefPicList[refList][rIdx] = m_apcRefPicList[refList][rIdx];
    }
  }

  // make the scaled reference picture list as the default reference picture list
  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      m_savedRefPicList[refList][rIdx] = m_apcRefPicList[refList][rIdx];
      m_apcRefPicList[refList][rIdx] = m_scaledRefPicList[refList][rIdx];

      // allow the access of the unscaled version in xPredInterBlk()
      m_apcRefPicList[refList][rIdx]->unscaledPic = m_savedRefPicList[refList][rIdx];
    }
  }
  
  //Make sure that TMVP is disabled when there are no reference pictures with the same resolution
  if(!refPicIsSameRes)
  {
    CHECK(getPicHeader()->getEnableTMVPFlag() != 0, "TMVP cannot be enabled in pictures that have no reference pictures with the same resolution")
  }
}

bool Slice::checkRPR()
{
  const PPS* pps = getPPS();

  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {

    if( refList == 1 && m_eSliceType != B_SLICE )
    {
      continue;
    }

    for( int rIdx = 0; rIdx < m_aiNumRefIdx[refList]; rIdx++ )
    {
      if( m_scaledRefPicList[refList][rIdx]->cs->pcv->lumaWidth != pps->getPicWidthInLumaSamples() || m_scaledRefPicList[refList][rIdx]->cs->pcv->lumaHeight != pps->getPicHeightInLumaSamples() )
      {
        return true;
      }
    }
  }

  return false;
}

bool             operator == (const ConstraintInfo& op1, const ConstraintInfo& op2)
{
  if( op1.m_intraOnlyConstraintFlag                      != op2.m_intraOnlyConstraintFlag                        ) return false;
  if( op1.m_maxBitDepthConstraintIdc                     != op2.m_maxBitDepthConstraintIdc                       ) return false;
  if( op1.m_maxChromaFormatConstraintIdc                 != op2.m_maxChromaFormatConstraintIdc                   ) return false;
  if( op1.m_onePictureOnlyConstraintFlag                 != op2.m_onePictureOnlyConstraintFlag                   ) return false;
  if( op1.m_lowerBitRateConstraintFlag                   != op2.m_lowerBitRateConstraintFlag                     ) return false;
  if (op1.m_allLayersIndependentConstraintFlag           != op2.m_allLayersIndependentConstraintFlag             ) return false;
  if (op1.m_noMrlConstraintFlag                          != op2.m_noMrlConstraintFlag                            ) return false;
  if (op1.m_noIspConstraintFlag                          != op2.m_noIspConstraintFlag                            ) return false;
  if (op1.m_noMipConstraintFlag                          != op2.m_noMipConstraintFlag                            ) return false;
  if (op1.m_noLfnstConstraintFlag                        != op2.m_noLfnstConstraintFlag                          ) return false;
  if (op1.m_noMmvdConstraintFlag                         != op2.m_noMmvdConstraintFlag                           ) return false;
  if (op1.m_noSmvdConstraintFlag                         != op2.m_noSmvdConstraintFlag                           ) return false;
  if (op1.m_noProfConstraintFlag                         != op2.m_noProfConstraintFlag                           ) return false;
  if (op1.m_noPaletteConstraintFlag                      != op2.m_noPaletteConstraintFlag                        ) return false;
  if (op1.m_noActConstraintFlag                          != op2.m_noActConstraintFlag                            ) return false;
  if (op1.m_noLmcsConstraintFlag                         != op2.m_noLmcsConstraintFlag                           ) return false;
  if (op1.m_noExplicitScaleListConstraintFlag            != op2.m_noExplicitScaleListConstraintFlag              ) return false;
  if (op1.m_noVirtualBoundaryConstraintFlag              != op2.m_noVirtualBoundaryConstraintFlag                ) return false;
  if (op1.m_noChromaQpOffsetConstraintFlag               != op2.m_noChromaQpOffsetConstraintFlag                 ) return false;
  if (op1.m_noRprConstraintFlag                          != op2.m_noRprConstraintFlag                            ) return false;
  if (op1.m_noResChangeInClvsConstraintFlag              != op2.m_noResChangeInClvsConstraintFlag                ) return false;
  if (op1.m_noMttConstraintFlag                          != op2.m_noMttConstraintFlag                            ) return false;
  if( op1.m_noQtbttDualTreeIntraConstraintFlag           != op2.m_noQtbttDualTreeIntraConstraintFlag             ) return false;
  if( op1.m_noPartitionConstraintsOverrideConstraintFlag != op2.m_noPartitionConstraintsOverrideConstraintFlag   ) return false;
  if( op1.m_noSaoConstraintFlag                          != op2.m_noSaoConstraintFlag                            ) return false;
  if( op1.m_noAlfConstraintFlag                          != op2.m_noAlfConstraintFlag                            ) return false;
  if( op1.m_noCCAlfConstraintFlag                        != op2.m_noCCAlfConstraintFlag                          ) return false;
  if (op1.m_noWeightedPredictionConstraintFlag           != op2.m_noWeightedPredictionConstraintFlag             ) return false;
  if( op1.m_noRefWraparoundConstraintFlag                != op2.m_noRefWraparoundConstraintFlag                  ) return false;
  if( op1.m_noTemporalMvpConstraintFlag                  != op2.m_noTemporalMvpConstraintFlag                    ) return false;
  if( op1.m_noSbtmvpConstraintFlag                       != op2.m_noSbtmvpConstraintFlag                         ) return false;
  if( op1.m_noAmvrConstraintFlag                         != op2.m_noAmvrConstraintFlag                           ) return false;
  if( op1.m_noBdofConstraintFlag                         != op2.m_noBdofConstraintFlag                           ) return false;
  if( op1.m_noDmvrConstraintFlag                         != op2.m_noDmvrConstraintFlag                           ) return false;
  if( op1.m_noCclmConstraintFlag                         != op2.m_noCclmConstraintFlag                           ) return false;
  if( op1.m_noMtsConstraintFlag                          != op2.m_noMtsConstraintFlag                            ) return false;
  if( op1.m_noSbtConstraintFlag                          != op2.m_noSbtConstraintFlag                            ) return false;
  if( op1.m_noAffineMotionConstraintFlag                 != op2.m_noAffineMotionConstraintFlag                   ) return false;
  if( op1.m_noBcwConstraintFlag                          != op2.m_noBcwConstraintFlag                            ) return false;
  if( op1.m_noIbcConstraintFlag                          != op2.m_noIbcConstraintFlag                            ) return false;
  if( op1.m_noCiipConstraintFlag                         != op2.m_noCiipConstraintFlag                           ) return false;
  if( op1.m_noLadfConstraintFlag                         != op2.m_noLadfConstraintFlag                           ) return false;
  if( op1.m_noTransformSkipConstraintFlag                != op2.m_noTransformSkipConstraintFlag                  ) return false;
  if( op1.m_noBDPCMConstraintFlag                        != op2.m_noBDPCMConstraintFlag                          ) return false;
  if( op1.m_noJointCbCrConstraintFlag                    != op2.m_noJointCbCrConstraintFlag                      ) return false;
  if( op1.m_noQpDeltaConstraintFlag                      != op2.m_noQpDeltaConstraintFlag                        ) return false;
  if( op1.m_noDepQuantConstraintFlag                     != op2.m_noDepQuantConstraintFlag                       ) return false;
  if( op1.m_noSignDataHidingConstraintFlag               != op2.m_noSignDataHidingConstraintFlag                 ) return false;
  if( op1.m_noTrailConstraintFlag                        != op2.m_noTrailConstraintFlag                          ) return false;
  if( op1.m_noStsaConstraintFlag                         != op2.m_noStsaConstraintFlag                           ) return false;
  if( op1.m_noRaslConstraintFlag                         != op2.m_noRaslConstraintFlag                           ) return false;
  if( op1.m_noRadlConstraintFlag                         != op2.m_noRadlConstraintFlag                           ) return false;
  if( op1.m_noIdrConstraintFlag                          != op2.m_noIdrConstraintFlag                            ) return false;
  if( op1.m_noCraConstraintFlag                          != op2.m_noCraConstraintFlag                            ) return false;
  if( op1.m_noGdrConstraintFlag                          != op2.m_noGdrConstraintFlag                            ) return false;
  if( op1.m_noApsConstraintFlag                          != op2.m_noApsConstraintFlag                            ) return false;
  return true;
}
bool             operator != (const ConstraintInfo& op1, const ConstraintInfo& op2)
{
  return !(op1 == op2);
}

bool             operator == (const ProfileTierLevel& op1, const ProfileTierLevel& op2)
{
  if (op1.m_tierFlag        != op2.m_tierFlag) return false;
  if (op1.m_profileIdc      != op2.m_profileIdc) return false;
  if (op1.m_numSubProfile   != op2.m_numSubProfile) return false;
  if (op1.m_levelIdc        != op2.m_levelIdc) return false;
  if (op1.m_frameOnlyConstraintFlag != op2.m_frameOnlyConstraintFlag) return false;
  if (op1.m_multiLayerEnabledFlag   != op2.m_multiLayerEnabledFlag) return false;
  if (op1.m_constraintInfo  != op2.m_constraintInfo) return false;
  if (op1.m_subProfileIdc   != op2.m_subProfileIdc) return false;

  for (int i = 0; i < MAX_TLAYER - 1; i++)
  {
    if (op1.m_subLayerLevelPresentFlag[i] != op2.m_subLayerLevelPresentFlag[i])
    {
      return false;
    }
  }
  for (int i = 0; i < MAX_TLAYER; i++)
  {
    if (op1.m_subLayerLevelIdc[i] != op2.m_subLayerLevelIdc[i])
    {
      return false;
    }
  }
  return true;
}
bool             operator != (const ProfileTierLevel& op1, const ProfileTierLevel& op2)
{
  return !(op1 == op2);
}

uint32_t
LevelTierFeatures::getMaxPicWidthInLumaSamples()  const
{
  return uint32_t(sqrt(maxLumaPs*8.0));
}

uint32_t
LevelTierFeatures::getMaxPicHeightInLumaSamples() const
{
  return uint32_t(sqrt(maxLumaPs*8.0));
}

static const uint64_t MAX_CNFUINT64 = std::numeric_limits<uint64_t>::max();

static const LevelTierFeatures mainLevelTierInfo[] =
{
      //  level,       maxlumaps,      maxcpb[tier],,  maxSlicesPerAu,maxTilesPerAu,cols, maxLumaSr,       maxBr[tier],,    minCr[tier],,
    { vvdecLevel::VVDEC_LEVEL1  ,    36864, {      350,        0 },       16,        1,        1,     552960ULL, {     128,        0 }, { 2, 2} },
    { vvdecLevel::VVDEC_LEVEL2  ,   122880, {     1500,        0 },       16,        1,        1,    3686400ULL, {    1500,        0 }, { 2, 2} },
    { vvdecLevel::VVDEC_LEVEL2_1,   245760, {     3000,        0 },       20,        1,        1,    7372800ULL, {    3000,        0 }, { 2, 2} },
    { vvdecLevel::VVDEC_LEVEL3  ,   552960, {     6000,        0 },       30,        4,        2,   16588800ULL, {    6000,        0 }, { 2, 2} },
    { vvdecLevel::VVDEC_LEVEL3_1,   983040, {    10000,        0 },       40,        9,        3,   33177600ULL, {   10000,        0 }, { 2, 2} },
    { vvdecLevel::VVDEC_LEVEL4  ,  2228224, {    12000,    30000 },       75,       25,        5,   66846720ULL, {   12000,    30000 }, { 4, 4} },
    { vvdecLevel::VVDEC_LEVEL4_1,  2228224, {    20000,    50000 },       75,       25,        5,  133693440ULL, {   20000,    50000 }, { 4, 4} },
    { vvdecLevel::VVDEC_LEVEL5  ,  8912896, {    25000,   100000 },      200,      110,       10,  267386880ULL, {   25000,   100000 }, { 6, 4} },
    { vvdecLevel::VVDEC_LEVEL5_1,  8912896, {    40000,   160000 },      200,      110,       10,  534773760ULL, {   40000,   160000 }, { 8, 4} },
    { vvdecLevel::VVDEC_LEVEL5_2,  8912896, {    60000,   240000 },      200,      110,       10, 1069547520ULL, {   60000,   240000 }, { 8, 4} },
    { vvdecLevel::VVDEC_LEVEL6  , 35651584, {    80000,   240000 },      600,      440,       20, 1069547520ULL, {   60000,   240000 }, { 8, 4} },
    { vvdecLevel::VVDEC_LEVEL6_1, 35651584, {   120000,   480000 },      600,      440,       20, 2139095040ULL, {  120000,   480000 }, { 8, 4} },
    { vvdecLevel::VVDEC_LEVEL6_2, 35651584, {   180000,   800000 },      600,      440,       20, 4278190080ULL, {  240000,   800000 }, { 8, 4} },
    { vvdecLevel::VVDEC_LEVEL15_5, MAX_UINT,{ MAX_UINT, MAX_UINT }, MAX_UINT, MAX_UINT, MAX_UINT, MAX_CNFUINT64, {MAX_UINT, MAX_UINT }, { 0, 0} },
    { vvdecLevel::VVDEC_LEVEL_NONE    }
};

static const ProfileFeatures validProfiles[] = {
// profile, pNameString, maxBitDepth, maxChrFmt, lvl15.5, cpbvcl, cpbnal, fcf*1000, mincr*100, levelInfo
// most constrained profiles must appear first.
  { Profile::MAIN_10_STILL_PICTURE, "Main_10_Still_Picture", 10, CHROMA_420, true, 1000, 1100, 1875, 100,
    mainLevelTierInfo, true },
  { Profile::MULTILAYER_MAIN_10_STILL_PICTURE, "Multilayer_Main_10_Still_Picture", 10, CHROMA_420, true, 1000, 1100,
    1875, 100, mainLevelTierInfo, true },
  { Profile::MAIN_10_444_STILL_PICTURE, "Main_444_10_Still_Picture", 10, CHROMA_444, true, 2500, 2750, 3750, 75,
    mainLevelTierInfo, true },
  { Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE, "Multilayer_Main_444_10_Still_Picture", 10, CHROMA_444, true, 2500,
    2750, 3750, 75, mainLevelTierInfo, true },
  { Profile::MAIN_10, "Main_10", 10, CHROMA_420, false, 1000, 1100, 1875, 100, mainLevelTierInfo, false },
  { Profile::MULTILAYER_MAIN_10, "Multilayer_Main_10", 10, CHROMA_420, false, 1000, 1100, 1875, 100, mainLevelTierInfo,
    false },
  { Profile::MAIN_10_444, "Main_444_10", 10, CHROMA_444, false, 2500, 2750, 3750, 75, mainLevelTierInfo, false },
  { Profile::MULTILAYER_MAIN_10_444, "Multilayer_Main_444_10", 10, CHROMA_444, false, 2500, 2750, 3750, 75,
    mainLevelTierInfo, false },
  { Profile::NONE, 0 },
};

const ProfileFeatures *ProfileFeatures::getProfileFeatures(const Profile::Name p)
{
  int i;
  for (i = 0; validProfiles[i].profile != Profile::NONE; i++)
  {
    if (validProfiles[i].profile == p)
    {
      return &validProfiles[i];
    }
  }

  return &validProfiles[i];
}

void
ProfileLevelTierFeatures::extractPTLInformation(const SPS &sps)
{
  const ProfileTierLevel &spsPtl =*(sps.getProfileTierLevel());

  m_tier = spsPtl.getTierFlag();

  // Identify the profile from the profile Idc, and possibly other constraints.
  for(int32_t i=0; validProfiles[i].profile != Profile::NONE; i++)
  {
    if (spsPtl.getProfileIdc() == validProfiles[i].profile)
    {
      m_pProfile = &(validProfiles[i]);
      break;
    }
  }

  if (m_pProfile != 0)
  {
    // Now identify the level:
    const LevelTierFeatures *pLTF  = m_pProfile->pLevelTiersListInfo;
    const vvdecLevel spsLevelName  = spsPtl.getLevelIdc();
    if (spsLevelName!=vvdecLevel::VVDEC_LEVEL15_5 || m_pProfile->canUseLevel15p5)
    {
      for(int i=0; pLTF[i].level!=vvdecLevel::VVDEC_LEVEL_NONE; i++)
      {
        if (pLTF[i].level == spsLevelName)
        {
          m_pLevelTier = &(pLTF[i]);
        }
      }
    }
  }
}

double ProfileLevelTierFeatures::getMinCr() const
{
  return (m_pLevelTier!=0 && m_pProfile!=0) ? (m_pProfile->minCrScaleFactorx100 * m_pLevelTier->minCrBase[m_tier?1:0])/100.0 : 0.0 ;
}

uint64_t ProfileLevelTierFeatures::getCpbSizeInBits() const
{
  return (m_pLevelTier!=0 && m_pProfile!=0) ? uint64_t(m_pProfile->cpbVclFactor) * m_pLevelTier->maxCpb[m_tier?1:0] : uint64_t(0);
}

uint32_t ProfileLevelTierFeatures::getMaxDpbSize( uint32_t picSizeMaxInSamplesY ) const
{
  const uint32_t maxDpbPicBuf = 8;
  uint32_t       maxDpbSize;

  if (m_pLevelTier->level == vvdecLevel::VVDEC_LEVEL15_5)
  {
    // maxDpbSize is unconstrained in this case
    maxDpbSize = std::numeric_limits<uint32_t>::max();
  }
  else if (2 * picSizeMaxInSamplesY <= m_pLevelTier->maxLumaPs)
  {
    maxDpbSize = 2 * maxDpbPicBuf;
  }
  else if (3 * picSizeMaxInSamplesY <= 2 * m_pLevelTier->maxLumaPs)
  {
    maxDpbSize = 3 * maxDpbPicBuf / 2;
  }
  else
  {
    maxDpbSize = maxDpbPicBuf;
  }

  return maxDpbSize;
}

#if ENABLE_TRACING
void xTraceVPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Video Parameter Set     ===========\n" );
}
void xTraceDCIHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== DCI     ===========\n" );
}
void xTraceSPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Sequence Parameter Set  ===========\n" );
}

void xTracePPSHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Parameter Set  ===========\n" );
}

void xTraceAPSHeader()
{
  DTRACE(g_trace_ctx, D_HEADER, "=========== Adaptation Parameter Set  ===========\n");
}

void xTracePictureHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Header ===========\n" );
}

void xTraceSliceHeader()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Slice ===========\n" );
}

void xTraceAccessUnitDelimiter()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Access Unit Delimiter ===========\n" );
}
#endif

}
