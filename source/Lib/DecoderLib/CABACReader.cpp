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

/** \file     CABACReader.cpp
 *  \brief    Reader for low level syntax
 */

#include "CABACReader.h"

#include "CommonLib/CodingStructure.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/Picture.h"
#include "CommonLib/TimeProfiler.h"

namespace vvdec
{

void CABACReader::initCtxModels( Slice& slice )
{
  SliceType sliceType  = slice.getSliceType();
  int       qp         = slice.getSliceQp();
  if( slice.getPPS()->getCabacInitPresentFlag() && slice.getCabacInitFlag() )
  {
    switch( sliceType )
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      THROW_RECOVERABLE( "Invalid slice type" );
      break;
    }
  }
  m_BinDecoder.reset( qp, (int)sliceType );
}


//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    bool  terminating_bit()
//    void  remaining_bytes( noTrailingBytesExpected )
//================================================================================

bool CABACReader::terminating_bit()
{
  if( m_BinDecoder.decodeBinTrm() )
  {
    m_BinDecoder.finish();
    m_Bitstream->readOutTrailingBits();
    return true;
  }
  return false;
}

void CABACReader::remaining_bytes( bool noTrailingBytesExpected )
{
  if( noTrailingBytesExpected )
  {
//    CHECK( 0 != m_Bitstream->getNumBitsLeft(), "Bits left when not supposed" );
  }
  else
  {
    while( m_Bitstream->getNumBitsLeft() )
    {
      unsigned trailingNullByte = m_Bitstream->readByte();
      CHECK( trailingNullByte != 0, "Trailing byte should be '0', but has a value of " << std::hex << trailingNullByte << std::dec << "\n" );
    }
  }
}





//================================================================================
//  clause 7.3.11.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit    ( cs, slice, area, qps[2], ctuRsAddr )
//    bool  dt_implicit_qt_split( cs, pL, cuCtxL, pC, cuCtxC )
//================================================================================

bool CABACReader::coding_tree_unit( CodingStructure& cs, Slice* slice, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr )
{
  m_slice = slice;

  CUCtx cuCtx( qps[CH_L] );
  Partitioner &partitioner = m_partL;

  partitioner.initCtu( area, CH_L, cs, *m_slice );
  partitioner.treeType = TREE_D;
  partitioner.modeType = MODE_TYPE_ALL;

  sao( cs, ctuRsAddr );

  readAlf(cs, ctuRsAddr, partitioner);

  bool isLast = false;

  if( partitioner.isDualITree && cs.pcv->chrFormat != CHROMA_400 )
  {
    CUCtx cuCtxC( qps[CH_C] );
    Partitioner &partitionerC = m_partC;

    partitionerC.initCtu( area, CH_C, cs, *m_slice );
    partitionerC.treeType = TREE_D;
    partitionerC.modeType = MODE_TYPE_ALL;

    dt_implicit_qt_split( cs, partitioner, cuCtx, partitionerC, cuCtxC );

    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = cuCtxC.qp;
  }
  else
  {
    isLast = coding_tree( cs, partitioner, cuCtx );
    qps[CH_L] = cuCtx.qp;
  }

  DTRACE_COND( ctuRsAddr == 0, g_trace_ctx, D_QP_PER_CTU, "\n%4d %2d", cs.picture->poc, m_slice->getSliceQp() );
  DTRACE     (                 g_trace_ctx, D_QP_PER_CTU, " %3d",           qps[CH_L] - m_slice->getSliceQp() );

  return isLast;
}


bool CABACReader::dt_implicit_qt_split( CodingStructure& cs, Partitioner& partitionerL, CUCtx& cuCtxL, Partitioner& partitionerC, CUCtx& cuCtxC )
{
  if( partitionerL.currArea().lwidth() > 64 )
  {
    const PPS& pps = *cs.pps;
    // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
    if( pps.getUseDQP() && partitionerL.currQgEnable() )
    {
      cuCtxL.qgStart    = true;
      cuCtxL.isDQPCoded = false;
      
      cuCtxC.qgStart    = true;
      cuCtxC.isDQPCoded = false;
    }

    if( m_slice->getUseChromaQpAdj() && partitionerL.currQgChromaEnable() )
    {
      cuCtxL.isChromaQpAdjCoded = false;
      cuCtxC.isChromaQpAdjCoded = false;
      cs    .chromaQpAdj        = 0;
    }

    partitionerL.splitCurrArea( CU_QUAD_SPLIT, cs );
    partitionerC.splitCurrArea( CU_QUAD_SPLIT, cs );

    bool lastSegment = false;

    do
    {
      if( !lastSegment && cs.area.blocks[partitionerL.chType].contains( partitionerL.currArea().blocks[partitionerL.chType].pos() ) )
      {
        lastSegment = dt_implicit_qt_split( cs, partitionerL, cuCtxL, partitionerC, cuCtxC );
      }
    } while( partitionerL.nextPart( cs ) && partitionerC.nextPart( cs ) );

    return lastSegment;
  }

  bool isLast = coding_tree( cs, partitionerL, cuCtxL );
  CHECKD( isLast, "Chroma not parsed but end of slice signalled!" );
  isLast      = coding_tree( cs, partitionerC, cuCtxC );

  return isLast;
}

short CABACReader::readAlfCtuFilterIndex( CodingStructure& cs, unsigned ctuRsAddr )
{
  const unsigned numAps               = m_slice->getNumAlfAps();
  const unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  uint32_t filtIndex = 0;

  const bool usePrevFilt = numAps > 0
                        && m_BinDecoder.decodeBin( Ctx::AlfUseTemporalFilt() );

  if( usePrevFilt )
  {
    if( numAps > 1 )
    {
      xReadTruncBinCode( filtIndex, numAvailableFiltSets - NUM_FIXED_FILTER_SETS );
    }

    filtIndex += ( unsigned ) ( NUM_FIXED_FILTER_SETS );
  }
  else
  {
    xReadTruncBinCode( filtIndex, NUM_FIXED_FILTER_SETS );
  }

  return filtIndex;
}

//================================================================================
//  clause 7.3.11.3
//--------------------------------------------------------------------------------
//    void  sao( cs, ctuRsAddr )
//================================================================================

void CABACReader::sao( CodingStructure& cs, unsigned ctuRsAddr )
{

  SAOBlkParam&      sao_ctu_pars            = cs.getCtuData( ctuRsAddr ).saoParam;
  sao_ctu_pars[ COMPONENT_Y  ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cb ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cr ].modeIdc      = SAO_MODE_OFF;

  const SPS&   sps                          = *cs.sps;
  const Slice& slice                        = *m_slice;
  const bool   slice_sao_luma_flag          = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  const bool   slice_sao_chroma_flag        = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );

  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  // merge
  int             frame_width_in_ctus     = cs.pcv->widthInCtus;
  int             ry                      = ctuRsAddr      / frame_width_in_ctus;
  int             rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  int             sao_merge_type          = -1;
  const Position  pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned  curSliceIdx = m_slice->getIndependentSliceIdx();
  const unsigned  curTileIdx  = cs.pps->getTileIdx( pos );

  if( cs.getCURestricted( pos.offset(-(int)cs.pcv->maxCUWidth, 0), pos, curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_left_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) );
  }

  if( sao_merge_type < 0 && cs.getCURestricted( pos.offset(0, -(int)cs.pcv->maxCUHeight), pos, curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_above_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) ) << 1;
  }
  if( sao_merge_type >= 0 )
  {
    if( slice_sao_luma_flag || slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Y  ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Y  ].typeIdc  = sao_merge_type;
    }
    if( slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Cb ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cr ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cb ].typeIdc  = sao_merge_type;
      sao_ctu_pars[ COMPONENT_Cr ].typeIdc  = sao_merge_type;
    }
    return;
  }

  // explicit parameters
  const ComponentID firstComp = ( slice_sao_luma_flag   ? COMPONENT_Y  : COMPONENT_Cb );
  const ComponentID lastComp  = ( slice_sao_chroma_flag ? COMPONENT_Cr : COMPONENT_Y  );

  for( ComponentID compID = firstComp; compID <= lastComp; compID = ComponentID( compID + 1 ) )
  {
    SAOOffset& sao_pars = sao_ctu_pars[ compID ];

    // sao_type_idx_luma / sao_type_idx_chroma
    if( compID != COMPONENT_Cr )
    {
      if( m_BinDecoder.decodeBin( Ctx::SaoTypeIdx() ) )
      {
        if( m_BinDecoder.decodeBinEP( ) )
        {
          // edge offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_EO;
        }
        else
        {
          // band offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_BO;
        }
      }
    }
    else //Cr, follow Cb SAO type
    {
      sao_pars.modeIdc = sao_ctu_pars[ COMPONENT_Cb ].modeIdc;
      sao_pars.typeIdc = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    if( sao_pars.modeIdc == SAO_MODE_OFF )
    {
      continue;
    }

    // sao_offset_abs
    int       offset[4];
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( sps.getBitDepth() );
    offset    [0]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [1]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [2]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [3]           = (int)unary_max_eqprob( maxOffsetQVal );

    // band offset mode
    if( sao_pars.typeIdc == SAO_TYPE_START_BO )
    {
      // sao_offset_sign
      for( int k = 0; k < 4; k++ )
      {
        if( offset[k] && m_BinDecoder.decodeBinEP( ) )
        {
          offset[k] = -offset[k];
        }
      }
      // sao_band_position
      sao_pars.typeAuxInfo = m_BinDecoder.decodeBinsEP( NUM_SAO_BO_CLASSES_LOG2 );
      for( int k = 0; k < 4; k++ )
      {
        sao_pars.offset[ ( sao_pars.typeAuxInfo + k ) % MAX_NUM_SAO_CLASSES ] = offset[k];
      }
      continue;
    }

    // edge offset mode
    sao_pars.typeAuxInfo = 0;
    if( compID != COMPONENT_Cr )
    {
      // sao_eo_class_luma / sao_eo_class_chroma
      sao_pars.typeIdc += m_BinDecoder.decodeBinsEP( NUM_SAO_EO_TYPES_LOG2 );
    }
    else
    {
      sao_pars.typeIdc  = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    sao_pars.offset[ SAO_CLASS_EO_FULL_VALLEY ] =  offset[0];
    sao_pars.offset[ SAO_CLASS_EO_HALF_VALLEY ] =  offset[1];
    sao_pars.offset[ SAO_CLASS_EO_PLAIN       ] =  0;
    sao_pars.offset[ SAO_CLASS_EO_HALF_PEAK   ] = -offset[2];
    sao_pars.offset[ SAO_CLASS_EO_FULL_PEAK   ] = -offset[3];
  }
}

//================================================================================
//    void  readAlf( cs, ctuRsAddr, partitioner )
//================================================================================
void CABACReader::readAlf( CodingStructure& cs, unsigned int ctuRsAddr, const Partitioner& partitioner )
{
  const PreCalcValues& pcv                = *cs.pcv;
  int                 frame_width_in_ctus = pcv.widthInCtus;
  int                 ry                  = ctuRsAddr / frame_width_in_ctus;
  int                 rx                  = ctuRsAddr - ry * frame_width_in_ctus;
  const Position      pos                 ( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  bool                leftAvail           = cs.getCURestricted( pos.offset( -1, 0 ), pos, partitioner.currSliceIdx, partitioner.currTileIdx, CH_L ) ? true : false;
  bool                aboveAvail          = cs.getCURestricted( pos.offset( 0, -1 ), pos, partitioner.currSliceIdx, partitioner.currTileIdx, CH_L ) ? true : false;
  
  CtuAlfData& currAlfData = cs.getCtuData( ctuRsAddr ).alfParam;
  CtuAlfData  leftAlfData, aboveAlfData;

  if( leftAvail )  leftAlfData  = cs.getCtuData( ctuRsAddr -                   1 ).alfParam;
  if( aboveAvail ) aboveAlfData = cs.getCtuData( ctuRsAddr - frame_width_in_ctus ).alfParam;

  if( m_slice->getAlfEnabledFlag( COMPONENT_Y ) )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      if( m_slice->getAlfEnabledFlag( ( ComponentID ) compIdx ) )
      {
        //uint8_t* ctbAlfFlag = m_slice->getPic()->getAlfCtuEnableFlag( compIdx );
        int ctx = 0;
          ctx += leftAlfData.alfCtuEnableFlag[compIdx];
          ctx += aboveAlfData.alfCtuEnableFlag[compIdx];
      
        currAlfData.alfCtuEnableFlag[compIdx] = m_BinDecoder.decodeBin( Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );
      
        if( isLuma( ( ComponentID ) compIdx ) && currAlfData.alfCtuEnableFlag[compIdx] )
        {
          currAlfData.alfCtbFilterIndex = readAlfCtuFilterIndex( cs, ctuRsAddr );
        }

        if( isChroma( ( ComponentID ) compIdx ) )
        {
          const int apsIdx                  = m_slice->getAlfApsIdChroma();
          CHECK( m_slice->getAlfAPSs()[apsIdx] == nullptr, "APS not initialized" );
          const AlfSliceParam& alfParam     = m_slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();
          const int numAlts                 = alfParam.numAlternativesChroma;
          currAlfData.alfCtuAlternative[compIdx - 1] = 0;

          if( currAlfData.alfCtuEnableFlag[compIdx] )
          {
            uint8_t decoded = 0;
            while( decoded < numAlts - 1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( compIdx - 1 ) ) )
              ++decoded;
            currAlfData.alfCtuAlternative[compIdx - 1] = decoded;
          }
        }
      }
    }
  }
  for( int compIdx = 1; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
  {
    if( m_slice->getCcAlfEnabledFlag( compIdx ) )
    {
      int ctxt = 0;
      ctxt += ( leftAlfData.ccAlfFilterControl[compIdx - 1] ) ? 1 : 0;
      ctxt += ( aboveAlfData.ccAlfFilterControl[compIdx - 1] ) ? 1 : 0;
      ctxt += ( compIdx == COMPONENT_Cr ) ? 3 : 0;

      int idcVal  = m_BinDecoder.decodeBin( Ctx::CcAlfFilterControlFlag( ctxt ) );

      if ( idcVal )
      {
        const int apsIdx        = compIdx == 1 ? m_slice->getCcAlfCbApsId() : m_slice->getCcAlfCrApsId();
        const int filterCount   = m_slice->getAlfAPSs()[apsIdx]->getCcAlfAPSParam().ccAlfFilterCount[compIdx - 1];
        while ( ( idcVal != filterCount ) && m_BinDecoder.decodeBinEP() )
        {
          idcVal++;
        }
      }
      currAlfData.ccAlfFilterControl[compIdx - 1] = idcVal;
    }
  }
}

//================================================================================
//  clause 7.3.11.4
//--------------------------------------------------------------------------------
//    bool      coding_tree     ( cs, partitioner, cuCtx )
//    PartSplit split_cu_mode   ( cs, partitioner )
//    ModeType  mode_constraint ( cs, partitioner, splitMode )
//================================================================================

bool CABACReader::coding_tree( CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx )
{
  const PPS      &pps             = *cs.pps;
        UnitArea  currArea        = partitioner.currArea();
  bool            lastSegment     = false;
  bool            chromaNotSplit  = false;
  const ModeType modeTypeParent   = partitioner.modeType;

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if( pps.getUseDQP() && partitioner.currQgEnable() && !isChroma( partitioner.chType ) )
  {
    cuCtx.qgStart     = true;
    cuCtx.isDQPCoded  = false;
  }
  if( m_slice->getUseChromaQpAdj() && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded = false;
    cs.chromaQpAdj           = 0;
  }

  const PartSplit split = split_cu_mode( cs, partitioner );

  if( split != CU_DONT_SPLIT )
  {
    partitioner.modeType = mode_constraint( cs, partitioner, split ); //change for child nodes

    //decide chroma split or not
    chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && partitioner.modeType == MODE_TYPE_INTRA;

    CHECK( chromaNotSplit && partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma" );

    if( partitioner.treeType == TREE_D )
    {
      partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
    }

    partitioner.splitCurrArea( split, cs );

    do
    {
      if( !lastSegment && cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
      {
        lastSegment = coding_tree( cs, partitioner, cuCtx );
      }
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit( cs );

    if( chromaNotSplit )
    {
      partitioner.chType    = CHANNEL_TYPE_CHROMA;
      partitioner.treeType  = TREE_C;
      partitioner.updateNeighbors( cs );
      
      if( !lastSegment && cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
      {
        lastSegment = coding_tree( cs, partitioner, cuCtx );
      }
      else
      {
        THROW_RECOVERABLE( "Unexpected behavior, not parsing chroma even though luma data is available!" );
      }
      
      //recover treeType
      partitioner.chType    = CHANNEL_TYPE_LUMA;
      partitioner.treeType  = TREE_D;
    }

    partitioner.modeType = modeTypeParent;

    return lastSegment;
  }

  TreeType treeType = partitioner.treeType;

  if( isChroma( partitioner.chType ) )                                 { currArea.Y()                  = CompArea(); treeType = TREE_C; }
  else if( partitioner.isDualITree || partitioner.treeType == TREE_L ) { currArea.Cb() = currArea.Cr() = CompArea(); treeType = TREE_L; }

  CodingUnit& cu = cs.addCU( currArea, partitioner.chType, treeType, partitioner.modeType, partitioner.currPartLevel().cuLeft, partitioner.currPartLevel().cuAbove );

#if ENABLE_TRACING && 0
  if( cu.chType() == CHANNEL_TYPE_CHROMA )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "[chroma] CU x=%d, y=%d, w=%d, h=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height );
  }
  else
    DTRACE( g_trace_ctx, D_SYNTAX, "CU x=%d, y=%d, w=%d, h=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height );

#endif

  partitioner.setCUData( cu );
  cu.slice   = m_slice;
  cu.pps     = cu.slice->getPPS();
  cu.sps     = cu.slice->getSPS();
  cu.tileIdx = partitioner.currTileIdx;
  int lumaQPinLocalDualTree = -1;

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp      = CU::predictQP( cu, cuCtx.qp );
  }

  if( pps.getUseDQP() && partitioner.isSepTree( cs ) && isChroma( cu.chType() ) )
  {
    const Position chromaCentral( cu.chromaPos().offset( cu.chromaSize().width >> 1, cu.chromaSize().height >> 1 ) );
    const Position lumaRefPos( chromaCentral.x << getComponentScaleX( COMPONENT_Cb, cu.chromaFormat ), chromaCentral.y << getComponentScaleY( COMPONENT_Cb, cu.chromaFormat ) );
    //derive chroma qp, but the chroma qp is saved in cuCtx.qp which is used for luma qp
    //therefore, after decoding the chroma CU, the cuCtx.qp shall be recovered to luma qp in order to decode next luma cu qp
//    const CodingUnit* colLumaCu = cs.getLumaCU( lumaRefPos );
    const CodingUnit* colLumaCu = cs.getCU( lumaRefPos, CHANNEL_TYPE_LUMA );
    CHECK( colLumaCu == nullptr, "colLumaCU shall exist" );
    lumaQPinLocalDualTree = cuCtx.qp;

    if( colLumaCu ) cuCtx.qp = colLumaCu->qp;
  }

  cu.qp = cuCtx.qp;                 //NOTE: CU QP can be changed by deltaQP signaling at TU level
  cu.chromaQpAdj = cs.chromaQpAdj;  //NOTE: CU chroma QP adjustment can be changed by adjustment signaling at TU level

  // coding unit
  bool isLastCtu = coding_unit( cu, partitioner, cuCtx );
  //recover cuCtx.qp to luma qp after decoding the chroma CU
  if( pps.getUseDQP() && partitioner.isSepTree( cs ) && isChroma( cu.chType() ) )
  {
    cuCtx.qp = lumaQPinLocalDualTree;
  }

  if( isChromaEnabled( cs.pcv->chrFormat ) )
  for( TransformUnit& tu : TUTraverser( &cu.firstTU, cu.lastTU->next ) )
  {
    if( tu.Cb().valid() )
    {
      QpParam cQP( tu, COMPONENT_Cb, false );
      tu.chromaQp[COMPONENT_Cb - 1] = cQP.Qp( false );
    }

    if( tu.Cr().valid() )
    {
      QpParam cQP( tu, COMPONENT_Cr, false );
      tu.chromaQp[COMPONENT_Cr - 1] = cQP.Qp( false );
    }
  }

#if ENABLE_TRACING
  if( cu.chType() == CHANNEL_TYPE_CHROMA )
  {
    DTRACE( g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
    DTRACE( g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );

#endif
  return isLastCtu;
}

int signalModeCons( const CodingStructure& cs, const Slice* slice, const PartSplit split, const Partitioner &partitioner, const ModeType modeTypeParent )
{
  if( partitioner.isDualITree || modeTypeParent != MODE_TYPE_ALL || partitioner.currArea().chromaFormat == CHROMA_444 || partitioner.currArea().chromaFormat == CHROMA_400 )
    return LDT_MODE_TYPE_INHERIT;

  int minLumaArea = partitioner.currArea().lumaSize().area();
  if( split == CU_QUAD_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT ) // the area is split into 3 or 4 parts
  {
    minLumaArea = minLumaArea >> 2;
  }
  else if( split == CU_VERT_SPLIT || split == CU_HORZ_SPLIT ) // the area is split into 2 parts
  {
    minLumaArea = minLumaArea >> 1;
  }

  const int minChromaBlock  = minLumaArea >> ( getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, partitioner.currArea().chromaFormat ) + getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, partitioner.currArea().chromaFormat ) );
  const bool is2xNChroma    = ( partitioner.currArea().chromaSize().width == 4 && split == CU_VERT_SPLIT ) || ( partitioner.currArea().chromaSize().width == 8 && split == CU_TRIV_SPLIT );
  return    minChromaBlock >= 16 &&
           !is2xNChroma ?
              LDT_MODE_TYPE_INHERIT :
            ( minLumaArea < 32 || slice->isIntra() ) ? LDT_MODE_TYPE_INFER : LDT_MODE_TYPE_SIGNAL;
}

ModeType CABACReader::mode_constraint( CodingStructure& cs, Partitioner &partitioner, PartSplit splitMode )
{
  const int val = signalModeCons( cs, m_slice, splitMode, partitioner, partitioner.modeType );

  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    const int ctxIdx = DeriveCtx::CtxModeConsFlag( cs, partitioner );
    const bool flag  = m_BinDecoder.decodeBin( Ctx::ModeConsFlag( ctxIdx ) );

    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
    return flag ? MODE_TYPE_INTRA : MODE_TYPE_INTER;
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    return MODE_TYPE_INTRA;
  }
  else
  {
    return partitioner.modeType;
  }
}

PartSplit CABACReader::split_cu_mode( CodingStructure& cs, Partitioner &partitioner )
{
  PartSplit mode = CU_DONT_SPLIT;

  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  const unsigned numHor = canBh + canTh;
  const unsigned numVer = canBv + canTv;
  unsigned numSplit = ( canQt << 1 ) + numHor + numVer;

  bool isSplit = !!numSplit;

#if !ENABLE_TRACING
  if( canNo && !isSplit )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=0 split=0\n" );
    return CU_DONT_SPLIT;
  }

#endif
  unsigned ctxSplit, ctxQtSplit, ctxBttHV, ctxBtt12;

  const CodingUnit* cuLeft   = partitioner.currPartLevel().cuLeft;
  const CodingUnit* cuAbove  = partitioner.currPartLevel().cuAbove;
  const unsigned widthCurr   = partitioner.currArea().blocks[partitioner.chType].width;
  const unsigned heightCurr  = partitioner.currArea().blocks[partitioner.chType].height;

#if !ENABLE_TRACING
  if( canNo && isSplit )
#endif
  {
    ///////////////////////
    // CTX do split (0-8)
    ///////////////////////

    ctxSplit  = ( cuLeft  && cuLeft ->blocks[partitioner.chType].height < heightCurr );
    ctxSplit += ( cuAbove && cuAbove->blocks[partitioner.chType].width  < widthCurr  );

                                      // 0, 1, 2, 3, 4, 5, 6 // split
                                      // 0, 0, 1, 2, 3, 4, 5 // split - 1
    static constexpr int ctxOffset[] = { 0, 0, 0, 3, 3, 6, 6 };
    ctxSplit += ctxOffset[numSplit];

#if ENABLE_TRACING
    if( canNo && isSplit )
#endif // endif

    isSplit = m_BinDecoder.decodeBin( Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, isSplit );

  if( !isSplit )
  {
    return CU_DONT_SPLIT;
  }

  const bool canBtt = !!numHor || !!numVer;
  bool       isQt   = canQt;

#if !ENABLE_TRACING
  if( isQt && canBtt )
#endif
  {
    //////////////////////////
    // CTX is qt split (0-5)
    //////////////////////////
    ctxQtSplit  = ( cuLeft  && cuLeft->qtDepth  > partitioner.currQtDepth );
    ctxQtSplit += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth );
    ctxQtSplit += partitioner.currQtDepth < 2 ? 0 : 3;

#if ENABLE_TRACING
    if( isQt && canBtt )
#endif
    isQt = m_BinDecoder.decodeBin( Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return CU_QUAD_SPLIT;
  }

  const bool canHor = !!numHor;
  bool        isVer = !!numVer;

#if !ENABLE_TRACING
  if( isVer && canHor )
#endif
  {
    ////////////////////////////
    // CTX is ver split (0-4)
    ////////////////////////////
    ctxBttHV = 0;

    if( numVer == numHor )
    {
      if( cuLeft && cuAbove )
      {
        const int wIdxAbove = getLog2( cuAbove->blocks[partitioner.chType].width  );
        const int hIdxLeft  = getLog2( cuLeft ->blocks[partitioner.chType].height );

        const int depAbove  = widthCurr  >> wIdxAbove;
        const int depLeft   = heightCurr >> hIdxLeft;

        if( depAbove == depLeft ) ctxBttHV = 0;
        else if( depAbove < depLeft ) ctxBttHV = 1;
        else ctxBttHV = 2;
      }
    }
    else if( numVer < numHor )
    {
      ctxBttHV = 3;
    }
    else
    {
      ctxBttHV = 4;
    }

#if ENABLE_TRACING
    if( isVer && canHor )
#endif
    isVer = m_BinDecoder.decodeBin( Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  bool        is12 = isVer ? canBv : canBh;

#if !ENABLE_TRACING
  if( is12 && can14 )
#endif
  {
    //////////////////////////
    // CTX is h/v bt (0-3)
    //////////////////////////
    ctxBtt12 = !!( partitioner.currMtDepth <= 1 ) + ( isVer << 1 );

#if ENABLE_TRACING
    if( is12 && can14 )
#endif
    is12 = m_BinDecoder.decodeBin( Ctx::Split12Flag( ctxBtt12 ) );
  }

  if     ( isVer && is12 )  mode = CU_VERT_SPLIT;
  else if( isVer && !is12 ) mode = CU_TRIV_SPLIT;
  else if( !isVer && is12 ) mode = CU_HORZ_SPLIT;
  else                      mode = CU_TRIH_SPLIT;

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, ctxBtt12, mode );

  return mode;
}

//================================================================================
//  clause 7.3.11.5
//--------------------------------------------------------------------------------
//    bool  coding_unit             ( cu, pm, cuCtx )
//    void  cu_skip_flag            ( cu )
//    void  pred_mode               ( cu )
//    void  bdpcm_mode              ( cu, compID )
//    void  cu_pred_data            ( cu )
//    void  cu_bcw_flag             ( cu )
//    void  extend_ref_line         ( cu )
//    void  intra_luma_pred_mode    ( cu )
//    bool  intra_chroma_lmc_mode   ( cu )
//    void  intra_chroma_pred_mode  ( cu )
//    void  cu_residual             ( cu, pm, cuCtx )
//    void  rqt_root_cbf            ( cu )
//    void  adaptive_color_transform( cu )
//    void  sbt_mode                ( cu )
//    void  mip_flag                ( cu )
//    void  mip_pred_mode           ( cu )
//    bool  end_of_ctu              ( cu, cuCtx )
//================================================================================

bool CABACReader::coding_unit( CodingUnit &cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  CodingStructure& cs = *cu.cs;

  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType(), cu.modeType() );

  // skip flag
  if( !m_slice->isIntra() || cs.sps->getIBCFlag() )
  {
    if( cu.Y().valid() )
    {
      cu_skip_flag( cu );
    }

    // skip data
    if( cu.skip() )
    {
      cu.setColorTransform( false );
      cs.addEmptyTUs      ( partitioner, cu );
      MergeCtx              mrgCtx;
      prediction_unit     ( cu );
      return end_of_ctu   ( cu, cuCtx );
    }
    else
    {
      // prediction mode and partitioning data
      pred_mode( cu );
    }
  }
  else
  {
    cu.setPredMode( MODE_INTRA );
  }

  // TODO: palette stuff

  if( CU::isIntra( cu ) )
  {
    adaptive_color_transform( cu );
  }

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // check end of cu
  return end_of_ctu( cu, cuCtx );
}


void CABACReader::cu_skip_flag( CodingUnit& cu )
{
  bool ibcFlag = cu.slice->getSPS()->getIBCFlag() && cu.lwidth() <= 64 && cu.lheight() <= 64;

  if( ( cu.slice->isIntra() || CU::isConsIntra( cu ) ) && ibcFlag )
  {
    //cu.setSkip       ( false );
    //cu.setRootCbf    ( false );
    //cu.setPredMode   ( MODE_INTRA );
    //cu.setMmvdFlag   ( false );

    const unsigned ctxId = DeriveCtx::CtxSkipFlag( cu );
    const unsigned skip  = m_BinDecoder.decodeBin( Ctx::SkipFlag( ctxId ) );

    DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );

    if( skip )
    {
      cu.setSkip       ( true );
      //cu.setRootCbf    ( false );
      cu.setPredMode   ( MODE_IBC );
      //cu.setMmvdFlag   ( false );
      cu.cs->hasIbcBlock[cu.ctuData->lineIdx] = 1;
    }

    return;
  }
  else if( !ibcFlag && ( ( cu.lwidth() == 4 && cu.lheight() == 4 ) || CU::isConsIntra( cu ) ) )
  {
    return;
  }

  unsigned ctxId  = DeriveCtx::CtxSkipFlag(cu);
  unsigned skip   = m_BinDecoder.decodeBin( Ctx::SkipFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );

  if( skip && ibcFlag )
  {
    if( !CU::isConsInter( cu ) ) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
    {
      if( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        cu.setSkip       ( true );
        //cu.setRootCbf    ( false );
        cu.setPredMode   ( MODE_IBC );
        //cu.setMmvdFlag   ( false );
        cu.cs->hasIbcBlock[cu.ctuData->lineIdx] = 1;
        return;
      }

      unsigned ctxidx = DeriveCtx::CtxIBCFlag( cu );

      if( m_BinDecoder.decodeBin( Ctx::IBCFlag( ctxidx ) ) )
      {
        cu.setSkip               ( true );
        //cu.setRootCbf            ( false );
        cu.setPredMode           ( MODE_IBC );
        //cu.setMmvdFlag           ( false );
        //cu.setRegularMergeFlag   ( false );
        cu.cs->hasIbcBlock[cu.ctuData->lineIdx] = 1;
      }
      else
      {
        //cu.setPredMode( MODE_INTER );
      }

      DTRACE( g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode() );
    }
    else
    {
      //cu.setPredMode( MODE_INTER );
    }
  }

  if( skip )
  {
    cu.setSkip( true );
    //cu.setRootCbf ( false );
    //cu.setPredMode( MODE_INTER );
  }
}

void CABACReader::amvr_mode( CodingUnit& cu )
{
  const SPS *sps = cu.sps;

  if( !sps->getAMVREnabledFlag() || !CU::hasSubCUNonZeroMVd( cu ) )
  {
    return;
  }
  unsigned value = 0;

  if( CU::isIBC( cu ) )
    value = 1;
  else
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 0 ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 0 );

  if( value )
  {
    cu.setImv( value );

    if( !CU::isIBC( cu ) )
    {
      value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 4 ) );
      cu.setImv( value ? 1 : IMV_HPEL );
      DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 4 );
    }

    if( value )
    {
      value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 1 ) );
      value++;
      cu.setImv( value );
      DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", ( value - 1 ), 1 );
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv() );
}

void CABACReader::affine_amvr_mode( CodingUnit& cu )
{
  const SPS* sps = cu.sps;

  if( !sps->getAffineAmvrEnabledFlag() || !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  unsigned value = 0;
  value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 2 );

  if( value )
  {
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 3 );
    value++;
  }

  cu.setImv( value );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv() );
}

void CABACReader::pred_mode( CodingUnit& cu )
{
  if( CU::isConsInter( cu ) )
  {
    //cu.setPredMode( MODE_INTER );
    return;
  }

  bool ibcAllowed = false;

  if( cu.slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || CU::isConsIntra( cu ) )
  {
    ibcAllowed = true;
    cu.setPredMode( MODE_INTRA );
  }
  else
  {
    if( m_BinDecoder.decodeBin( Ctx::PredMode( DeriveCtx::CtxPredModeFlag( cu ) ) ) )
    {
      cu.setPredMode( MODE_INTRA );
    }
    else
    {
      ibcAllowed = true;
    }
  }

  ibcAllowed &= isLuma( cu.chType() ) && cu.sps->getIBCFlag() && cu.lwidth() <= 64 && cu.lheight() <= 64;

  if( ibcAllowed )
  {
    unsigned ctxidx = DeriveCtx::CtxIBCFlag( cu );

    if( m_BinDecoder.decodeBin( Ctx::IBCFlag( ctxidx ) ) )
    {
      cu.setPredMode( MODE_IBC );
      cu.cs->hasIbcBlock[cu.ctuData->lineIdx] = 1;
    }
  }
}

void CABACReader::bdpcm_mode( CodingUnit& cu, const ComponentID compID )
{

  if( !CU::bdpcmAllowed( cu, compID ) )
  {
    //if( isLuma( compID ) )
    //{
    //  cu.setBdpcmMode( 0 );
    //  if( !CS::isDualITree( *cu.cs ) )
    //    cu.setBdpcmModeChroma( 0 );
    //}
    //else
    //{
    //  cu.setBdpcmModeChroma( 0 );
    //}
    return;
  }

  const unsigned ctxId = isLuma( compID ) ? 0 : 2;
  int bdpcmMode = m_BinDecoder.decodeBin( Ctx::BDPCMMode( ctxId ) );

  if( bdpcmMode )
  {
    bdpcmMode += m_BinDecoder.decodeBin( Ctx::BDPCMMode( ctxId + 1 ) );
  }

  if( isLuma( compID ) )
  {
    cu.setBdpcmMode( bdpcmMode );
  }
  else
  {
    cu.setBdpcmModeChroma( bdpcmMode );
  }

#if ENABLE_TRACING
  if( isLuma( compID ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_LUMA, cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode() );
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_CHROMA, cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.bdpcmModeChroma() );
  }
#endif
}

void CABACReader::cu_pred_data( CodingUnit &cu )
{
  if( CU::isIntra( cu ) )
  {
    if( isLuma( cu.chType() ) )
    {
      bdpcm_mode( cu, COMPONENT_Y );
      intra_luma_pred_mode( cu );
    }
    if( ( isChroma( cu.chType() ) || !CU::isSepTree( cu ) ) && isChromaEnabled( cu.chromaFormat ) )
    {
      bdpcm_mode( cu, ComponentID( CHANNEL_TYPE_CHROMA ) );
      intra_chroma_pred_mode( cu );
    }

    return;
  }

  if( !cu.Y().valid() ) // dual tree chroma CU
  {
    cu.setPredMode( MODE_IBC );
    cu.cs->hasIbcBlock[cu.ctuData->lineIdx] = 1;
    return;
  }

  prediction_unit     ( cu );

  if( !cu.mergeFlag() )
  {
    if( cu.affineFlag() )
      affine_amvr_mode( cu );
    else
      amvr_mode       ( cu );
    cu_bcw_flag       ( cu );
  }
}

void CABACReader::cu_bcw_flag(CodingUnit& cu)
{
  if( !CU::isBcwIdxCoded( cu ) )
  {
    return;
  }

  CHECK(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)), " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");

  uint32_t idx    = 0;
  uint32_t symbol = m_BinDecoder.decodeBin( Ctx::BcwIdx( 0 ) );
  int32_t  numBcw = ( cu.slice->getCheckLDC() ) ? 5 : 3;

  if( symbol == 1 )
  {
    uint32_t prefixNumBits = numBcw - 2;
    uint32_t step = 1;

    idx = 1;

    for( int ui = 0; ui < prefixNumBits && m_BinDecoder.decodeBinEP(); ++ui ) idx += step;
  }

  uint8_t bcwIdx = ( uint8_t ) g_BcwParsingOrder[idx];
  CU::setBcwIdx( cu, g_BcwInternFwd[bcwIdx] );

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_bcw_flag() bcw_idx=%d\n", cu.BcwIdx() ? 1 : 0);
}

void CABACReader::xReadTruncBinCode(uint32_t& symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }

  int val = 1 << thresh;
  int b = maxSymbol - val;
  symbol = m_BinDecoder.decodeBinsEP(thresh);

  if (symbol >= val - b)
  {
    uint32_t altSymbol;
    altSymbol = m_BinDecoder.decodeBinEP();
    symbol <<= 1;
    symbol += altSymbol;
    symbol -= (val - b);
  }
}

void CABACReader::extend_ref_line( CodingUnit& cu )
{
  if( cu.bdpcmMode() || !cu.sps->getUseMRL() )
  {
    //cu.setMultiRefIdx( 0 );
    return;
  }

  const bool isFirstLineOfCtu = ( cu.ly() & cu.cs->pcv->maxCUHeightMask ) == 0;

  if( isFirstLineOfCtu )
  {
    //cu.setMultiRefIdx( 0 );
    return;
  }

  int multiRefIdx = 0;

  multiRefIdx = m_BinDecoder.decodeBin( Ctx::MultiRefLineIdx( 0 ) ) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];

  if( multiRefIdx != MULTI_REF_LINE_IDX[0] )
  {
    multiRefIdx = m_BinDecoder.decodeBin( Ctx::MultiRefLineIdx( 1 ) ) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
  }

  cu.setMultiRefIdx( multiRefIdx );
}

void CABACReader::intra_luma_pred_mode( CodingUnit &cu )
{
  if( cu.bdpcmMode() )
  {
    cu.intraDir[0] = cu.bdpcmMode() == 2 ? VER_IDX : HOR_IDX;
    return;
  }

  mip_flag( cu );

  if( cu.mipFlag() )
  {
    mip_pred_mode( cu );
    return;
  }

  extend_ref_line( cu );

  isp_mode( cu );

  // prev_intra_luma_pred_flag
  int mpmFlag;

  if( cu.multiRefIdx() )
  {
    mpmFlag = true;
  }
  else
    mpmFlag = m_BinDecoder.decodeBin( Ctx::IPredMode[0]() );

  unsigned mpm_pred[NUM_MOST_PROBABLE_MODES];  // mpm_idx / rem_intra_luma_pred_mode

  PU::getIntraMPMs( cu, mpm_pred );

  if( mpmFlag )
  {
    uint32_t ipred_idx = 0;

    unsigned ctx = ( cu.ispMode() == NOT_INTRA_SUBPARTITIONS ? 1 : 0 );

    if( cu.multiRefIdx() == 0 )
      ipred_idx = m_BinDecoder.decodeBin( Ctx::IntraLumaPlanarFlag( ctx ) );
    else
      ipred_idx = 1;

    if( ipred_idx ) while( ipred_idx < 5 && m_BinDecoder.decodeBinEP() ) ipred_idx++;

    cu.intraDir[0] = mpm_pred[ipred_idx];

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.intraDir[0] );
  }
  else
  {
    unsigned ipred_mode = 0;

    xReadTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);

    //postponed sorting of MPMs (only in remaining branch)
    std::sort( mpm_pred, mpm_pred + NUM_MOST_PROBABLE_MODES );

    for( uint32_t i = 0; i < NUM_MOST_PROBABLE_MODES; i++ )
    {
      ipred_mode += ( ipred_mode >= mpm_pred[i] );
    }

    cu.intraDir[0] = ipred_mode;

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.intraDir[0] );
  }
}

void CABACReader::intra_chroma_pred_mode( CodingUnit& cu )
{
  if( cu.bdpcmModeChroma() )
  {
    cu.intraDir[1] = cu.bdpcmModeChroma() == 2 ? VER_IDX : HOR_IDX;
    return;
  }

  if( cu.colorTransform() )
  {
    cu.intraDir[CHANNEL_TYPE_CHROMA] = DM_CHROMA_IDX;
    return;
  }

  // LM chroma mode
  if( cu.sps->getUseLMChroma() && CU::checkCCLMAllowed( cu ) )
  {
    bool isLMCMode = m_BinDecoder.decodeBin( Ctx::CclmModeFlag( 0 ) );
    if( isLMCMode )
    {
      intra_chroma_lmc_mode( cu );
      return;
    }
  }

  if( m_BinDecoder.decodeBin( Ctx::IPredMode[1]( 0 ) ) == 0 )
  {
    cu.intraDir[1] = DM_CHROMA_IDX;
    return;
  }

  unsigned candId = m_BinDecoder.decodeBinsEP( 2 );

  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes( cu, chromaCandModes );

  cu.intraDir[1] = chromaCandModes[candId];

  CHECKD( candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds" );
  CHECKD( PU::isLMCMode( chromaCandModes[candId] ), "The intra dir cannot be LM_CHROMA for this path" );
  CHECKD( chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path" );
}

bool CABACReader::intra_chroma_lmc_mode( CodingUnit& cu )
{
  int lmModeList[10];
  PU::getLMSymbolList( cu, lmModeList );

  int symbol = m_BinDecoder.decodeBin( Ctx::CclmModeIdx( 0 ) );

  if( symbol == 0 )
  {
    cu.intraDir[1] = lmModeList[symbol];
    CHECKD( cu.intraDir[1] != LM_CHROMA_IDX, "should be LM_CHROMA" );
  }
  else
  {
    symbol += m_BinDecoder.decodeBinEP();
    cu.intraDir[1] = lmModeList[symbol];
  }
  return true; //it will only enter this function for LMC modes, so always return true ;
}

void CABACReader::cu_residual( CodingUnit& cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  if( !CU::isIntra( cu ) )
  {
    if( !cu.mergeFlag() )
    {
      rqt_root_cbf( cu );
    }
    else
    {
      cu.setRootCbf( true );
    }
    if( cu.rootCbf() )
    {
      sbt_mode( cu );
    }
    if( !cu.rootCbf() )
    {
      //cu.setColorTransform( false );
      cu.cs->addEmptyTUs( partitioner, cu );
      return;
    }
  }
  else
  {
    cu.setRootCbf( true );
  }

  if( CU::isInter( cu ) || CU::isIBC( cu ) )
  {
    adaptive_color_transform( cu );
  }

  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA]   = false;
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
  cuCtx.lfnstLastScanPos = false;
  cuCtx.violatesMtsCoeffConstraint                    = false;
  cuCtx.mtsLastScanPos                                = false;

  ChromaCbfs chromaCbfs;
  transform_tree( *cu.cs, cu, partitioner, cuCtx );

  residual_lfnst_mode( cu, cuCtx );
  mts_idx            ( cu, cuCtx );

  bool rootCbf = false;

  for( const auto& blk : cu.blocks )
  {
    if( blk.valid() ) rootCbf |= cu.planeCbf( blk.compID() );
  }

  cu.setRootCbf( rootCbf );
}

void CABACReader::rqt_root_cbf( CodingUnit& cu )
{
  cu.setRootCbf( !!m_BinDecoder.decodeBin( Ctx::QtRootCbf() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf() ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACReader::adaptive_color_transform( CodingUnit& cu )
{
  if( !cu.sps->getUseColorTrans() || CU::isSepTree( cu ) )
  {
    return;
  }

  cu.setColorTransform( m_BinDecoder.decodeBin( Ctx::ACTFlag() ) );
}

void CABACReader::sbt_mode( CodingUnit& cu )
{
  const uint8_t sbtAllowed = CU::checkAllowedSbt( cu );

  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth  = cu.lwidth();
  SizeType cuHeight = cu.lheight();

  //bin - flag
  if( !m_BinDecoder.decodeBin( Ctx::SbtFlag( ( cuWidth * cuHeight <= 256 ) ? 1 : 0 ) ) )
  {
    return;
  }

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );

  //bin - type
  bool sbtQuadFlag = false;
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    sbtQuadFlag = m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    sbtQuadFlag = 0;
  }

  //bin - dir
  bool sbtHorFlag = false;
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    sbtHorFlag = m_BinDecoder.decodeBin( Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    sbtHorFlag = ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow );
  }

  CU::setSbtIdx( cu, sbtHorFlag ? ( sbtQuadFlag ? SBT_HOR_QUAD : SBT_HOR_HALF ) : ( sbtQuadFlag ? SBT_VER_QUAD : SBT_VER_HALF ) );

  //bin - pos
  CU::setSbtPos( cu, m_BinDecoder.decodeBin( Ctx::SbtPosFlag() ) ? SBT_POS1 : SBT_POS0 );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int) cu.sbtInfo() );
}


bool CABACReader::end_of_ctu( CodingUnit& cu, CUCtx& cuCtx )
{
  const Position rbPos = recalcPosition( cu.chromaFormat, cu.chType(), CHANNEL_TYPE_LUMA, cu.blocks[cu.chType()].bottomRight().offset( 1, 1 ) );

  if( ( ( rbPos.x & cu.cs->pcv->maxCUWidthMask ) == 0 || rbPos.x == cu.pps->getPicWidthInLumaSamples() )
  && ( ( rbPos.y & cu.cs->pcv->maxCUHeightMask ) == 0 || rbPos.y == cu.pps->getPicHeightInLumaSamples() )
    && ( !CU::isSepTree( cu ) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType() ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.pps->getUseDQP() && !cuCtx.isDQPCoded );

    return false;
  }

  return false;
}


//================================================================================
//  clause 7.3.11.7
//--------------------------------------------------------------------------------
//    void  prediction_unit     ( cu, mrgCtx )
//    void  general_merge_flag  ( cu )
//    void  merge_data          ( cu )
//    void  affine_flag         ( cu )
//    void  subblock_merge_flag ( cu )
//    void  merge_idx           ( cu )
//    void  mmvd_merge_idx      ( cu )
//    void  amvr_mode           ( cu )
//    void  affine_amvr_mode    ( cu )
//    void  inter_pred_idc      ( cu )
//    void  ref_idx             ( cu, eRefList )
//    void  mvp_flag            ( cu, eRefList )
//    void  ciip_flag           ( cu )
//    void  smvd_mode           ( cu )
//================================================================================

void CABACReader::prediction_unit( CodingUnit& cu )
{
  if( cu.skip() )
  {
    cu.setMergeFlag( true );
  }
  else
  {
    general_merge_flag( cu );
  }

  if( cu.mergeFlag() )
  {
    merge_data( cu );
  }
  else if( CU::isIBC( cu ) )
  {
    cu.setInterDir  ( 1 );
    //cu.setAffineFlag( false );
    cu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
    mvd_coding( cu.mv[REF_PIC_LIST_0][0] );

    if( cu.sps->getMaxNumIBCMergeCand() == 1 )
    {
      cu.mvpIdx[REF_PIC_LIST_0] = 0;
    }
    else
    {
      mvp_flag    ( cu, REF_PIC_LIST_0 );
    }
  }
  else
  {
    inter_pred_idc( cu );
    affine_flag   ( cu );
    smvd_mode     ( cu );

    if( cu.interDir() != 2 /* PRED_L1 */ )
    {
      ref_idx       ( cu, REF_PIC_LIST_0 );

      mvd_coding    ( cu.mv[REF_PIC_LIST_0][0] );
      if( cu.affineFlag() )
      {
        mvd_coding  ( cu.mv[REF_PIC_LIST_0][1] );
        if ( cu.affineType() == AFFINEMODEL_6PARAM )
        {
          mvd_coding( cu.mv[REF_PIC_LIST_0][2] );
        }
      }

      mvp_flag      ( cu, REF_PIC_LIST_0 );
    }

    if( cu.interDir() != 1 /* PRED_L0 */ )
    {
      if( cu.smvdMode() != 1 )
      {
        ref_idx( cu, REF_PIC_LIST_1 );

        if( cu.cs->picHeader->getMvdL1ZeroFlag() && cu.interDir() == 3 /* PRED_BI */ )
        {
          //cu.mv[REF_PIC_LIST_1][0] = Mv();
          //cu.mv[REF_PIC_LIST_1][1] = Mv();
          //cu.mv[REF_PIC_LIST_1][2] = Mv();
        }
        else
        {
          mvd_coding    ( cu.mv[REF_PIC_LIST_1][0] );
          if( cu.affineFlag() )
          {
            mvd_coding  ( cu.mv[REF_PIC_LIST_1][1] );
            if( cu.affineType() == AFFINEMODEL_6PARAM )
            {
              mvd_coding( cu.mv[REF_PIC_LIST_1][2] );
            }
          }
        }
      }

      mvp_flag        ( cu, REF_PIC_LIST_1 );
    }
  }

  if( cu.smvdMode() )
  {
    RefPicList eCurRefList = ( RefPicList ) ( cu.smvdMode() - 1 );
    cu.mv    [1 - eCurRefList][0] . set( -cu.mv[eCurRefList][0].hor, -cu.mv[eCurRefList][0].ver );
    cu.refIdx[1 - eCurRefList]    = cu.slice->getSymRefIdx( 1 - eCurRefList );

    CHECKD( !( ( cu.mv[1 - eCurRefList][0].getHor() >= MVD_MIN ) && ( cu.mv[1 - eCurRefList][0].getHor() <= MVD_MAX ) ) || !( ( cu.mv[1 - eCurRefList][0].getVer() >= MVD_MIN ) && ( cu.mv[1 - eCurRefList][0].getVer() <= MVD_MAX ) ), "Illegal MVD value" );
  }
}

void CABACReader::smvd_mode( CodingUnit& cu )
{
  //cu.setSmvdMode( 0 );

  if( cu.interDir() != 3 || cu.affineFlag() || !cu.sps->getUseSMVD() || cu.cs->picHeader->getMvdL1ZeroFlag() )
  {
    return;
  }

  if( cu.slice->getBiDirPred() == false )
  {
    return;
  }

  cu.setSmvdMode( m_BinDecoder.decodeBin( Ctx::SmvdFlag() ) ? 1 : 0 );

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", cu.smvdMode() ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height );
}

void CABACReader::subblock_merge_flag( CodingUnit& cu )
{
  //cu.setAffineFlag( false );

  if( !cu.slice->isIntra() && ( cu.slice->getPicHeader()->getMaxNumAffineMergeCand() > 0 ) && cu.lwidth() >= 8 && cu.lheight() >= 8 )
  {
    unsigned ctxId  = DeriveCtx::CtxAffineFlag( cu );
    cu.setAffineFlag( m_BinDecoder.decodeBin( Ctx::SubblockMergeFlag( ctxId ) ) );

    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affineFlag() ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACReader::affine_flag( CodingUnit& cu )
{
  if( cu.sps->getUseAffine() && cu.lumaSize().width >=16 && cu.lumaSize().height >= 16 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    cu.setAffineFlag( m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) ) );

    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affineFlag() ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if( cu.affineFlag() && cu.sps->getUseAffineType() )
    {
      ctxId = 0;
      cu.setAffineType( ( AffineModel ) m_BinDecoder.decodeBin( Ctx::AffineType( ctxId ) ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType() ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
    else
    {
      //cu.setAffineType( AFFINEMODEL_4PARAM );
    }
  }
}

void CABACReader::general_merge_flag( CodingUnit& cu )
{
  cu.setMergeFlag( m_BinDecoder.decodeBin( Ctx::MergeFlag() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", cu.mergeFlag() ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height );

  //if( cu.mergeFlag() && CU::isIBC( cu ) )
  //{
  //  cu.setMmvdFlag        ( false );
  //  cu.setRegularMergeFlag( false );
  //
  //  return;
  //}
}


void CABACReader::merge_data( CodingUnit& cu )
{
  if( CU::isIBC( cu ) )
  {
    merge_idx( cu );
    return;
  }
  else
  {
    subblock_merge_flag( cu );

    if( cu.affineFlag() )
    {
      merge_idx( cu );
      return;
    }

    bool regularMerge = true;

    const bool ciipAvailable = cu.sps->getUseCiip() && !cu.skip() && cu.lwidth() < 128 && cu.lheight() < 128 && cu.Y().area() >= 64;
    const bool geoAvailable  = cu.sps->getUseGeo()  && cu.slice->isInterB()
                                                    && cu.lwidth() >= GEO_MIN_CU_SIZE && cu.lheight() >= GEO_MIN_CU_SIZE
                                                    && cu.lwidth() <= GEO_MAX_CU_SIZE && cu.lheight() <= GEO_MAX_CU_SIZE
                                                    && cu.lwidth() < 8 * cu.lheight() && cu.lheight() < 8 * cu.lwidth();

    if( geoAvailable || ciipAvailable )
    {
      regularMerge = m_BinDecoder.decodeBin( Ctx::RegularMergeFlag( cu.skip() ? 0 : 1 ) );
    }

    if( regularMerge )
    {
      if( cu.sps->getUseMMVD() )
      {
        cu.setMmvdFlag( m_BinDecoder.decodeBin( Ctx::MmvdFlag( 0 ) ) );
      }
      else
      {
        //cu.setMmvdFlag( false );
      }
    }
    else
    {
      //cu.setMmvdFlag( false );
      if( geoAvailable && ciipAvailable )
      {
        ciip_flag( cu );
      }
      else if( ciipAvailable )
      {
        cu.setCiipFlag( true );
      }
      else
      {
        //cu.setCiipFlag( false );
      }

      if( cu.ciipFlag() )
      {
        cu.intraDir[0] = PLANAR_IDX;
        cu.intraDir[1] = DM_CHROMA_IDX;
      }
      else
      {
        cu.setGeoFlag( true );
      }
    }
  }

  if( cu.mmvdFlag() )
    mmvd_merge_idx( cu );
  else
    merge_idx     ( cu );
}


void CABACReader::merge_idx( CodingUnit& cu )
{
  if( cu.geoFlag() )
  {
    uint32_t splitDir = 0;
    xReadTruncBinCode( splitDir, GEO_NUM_PARTITION_MODE );
    cu.geoSplitDir = splitDir;

    const int maxNumGeoCand = cu.sps->getMaxNumGeoCand();
    const int numCandminus2 = maxNumGeoCand - 2;

    CHECK( maxNumGeoCand < 2, "Incorrect max number of geo candidates" );

    int mergeCand0 = 0;
    int mergeCand1 = 0;

    if( m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
    {
      mergeCand0 += unary_max_eqprob( numCandminus2 ) + 1;
    }

    if( numCandminus2 > 0 )
    {
      if( m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
      {
        mergeCand1 += unary_max_eqprob( numCandminus2 - 1 ) + 1;
      }
    }

    mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;

    cu.setGeoMergeIdx0( mergeCand0 );
    cu.setGeoMergeIdx1( mergeCand1 );

    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d\n", splitDir );
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx0=%d\n",      mergeCand0 );
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx1=%d\n",      mergeCand1 );
  }
  else
  {
    int numCandminus1;
    int ctxIdx = Ctx::MergeIdx();

    if( cu.predMode() == MODE_IBC )
    {
      numCandminus1 = int( cu.sps->getMaxNumIBCMergeCand() ) - 1;
    }
    else if( cu.affineFlag() )
    {
      numCandminus1 = int( cu.cs->picHeader->getMaxNumAffineMergeCand() ) - 1;
      ctxIdx        = Ctx::AffMergeIdx();
    }
    else
    {
      numCandminus1 = int( cu.sps->getMaxNumMergeCand() ) - 1;
    }

    int mergeIdx = 0;
    if( numCandminus1 > 0 && m_BinDecoder.decodeBin( ctxIdx ) )
    {
      for( mergeIdx++; mergeIdx < numCandminus1 && m_BinDecoder.decodeBinEP(); mergeIdx++ );
    }

    cu.setMergeIdx( mergeIdx );
#if ENABLE_TRACING

    if( cu.affineFlag() )
      DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", cu.mergeIdx() );
    else
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", cu.mergeIdx() );
#endif
  }
}

void CABACReader::mmvd_merge_idx( CodingUnit& cu )
{
  int var0 = 0, var1 = 0, var2 = 0;

  const int numCand            = int( cu.sps->getMaxNumMergeCand() );
  const int numCandminus1_base = ( numCand > 1 ) ? MMVD_BASE_MV_NUM - 1 : 0;

  if( numCandminus1_base > 0 && m_BinDecoder.decodeBin( Ctx::MmvdMergeIdx() ) )
  {
    for( var0++; var0 < numCandminus1_base && m_BinDecoder.decodeBinEP(); var0++ );
  }

  const int numCandminus1_step = MMVD_REFINE_STEP - 1;
  if( numCandminus1_step > 0 && m_BinDecoder.decodeBin( Ctx::MmvdStepMvpIdx() ) )
  {
    for( var1++; var1 < numCandminus1_step && m_BinDecoder.decodeBinEP(); var1++ );
  }

  if( m_BinDecoder.decodeBinEP() )
  {
    var2 += 2;
  }
  if( m_BinDecoder.decodeBinEP() )
  {
    var2 += 1;
  }

  cu.mmvdIdx = ( var0 * MMVD_MAX_REFINE_NUM + var1 * 4 + var2 );

  DTRACE( g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0 );
  DTRACE( g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1 );
  DTRACE( g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2 );
  DTRACE( g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", cu.mmvdIdx );
}

void CABACReader::inter_pred_idc( CodingUnit& cu )
{
  if( cu.slice->isInterP() )
  {
    cu.setInterDir( 1 );
    return;
  }
  
  if( !PU::isBipredRestriction( cu ) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir( cu );
    if( m_BinDecoder.decodeBin( Ctx::InterDir( ctxId ) ) )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, 3, cu.lumaPos().x, cu.lumaPos().y );
      cu.setInterDir( 3 );
      return;
    }
  }

  if( m_BinDecoder.decodeBin( Ctx::InterDir( 5 ) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=5 value=%d pos=(%d,%d)\n", 2, cu.lumaPos().x, cu.lumaPos().y );
    cu.setInterDir( 2 );
    return;
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=5 value=%d pos=(%d,%d)\n", 1, cu.lumaPos().x, cu.lumaPos().y );
  cu.setInterDir( 1 );
}


void CABACReader::ref_idx( CodingUnit &cu, RefPicList eRefList )
{
  if( cu.smvdMode() )
  {
    cu.refIdx[eRefList] = cu.slice->getSymRefIdx( eRefList );
    return;
  }

  const int numRef = cu.slice->getNumRefIdx( eRefList );

  if( numRef <= 1 || !m_BinDecoder.decodeBin( Ctx::RefPic( 0 ) ) )
  {
    if( numRef > 1 )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 0, cu.lumaPos().x, cu.lumaPos().y );
    }
    cu.refIdx[eRefList] = 0;
    return;
  }

  if( numRef <= 2 || !m_BinDecoder.decodeBin( Ctx::RefPic( 1 ) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 1, cu.lumaPos().x, cu.lumaPos().y );
    cu.refIdx[eRefList] = 1;
    return;
  }

  for( int idx = 3; ; idx++ )
  {
    if( numRef <= idx || !m_BinDecoder.decodeBinEP() )
    {
      cu.refIdx[eRefList] = ( signed char ) ( idx - 1 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", idx-1, cu.lumaPos().x, cu.lumaPos().y );
      return;
    }
  }
}



void CABACReader::mvp_flag( CodingUnit& cu, RefPicList eRefList )
{
  unsigned mvp_idx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", mvp_idx, cu.lumaPos().x, cu.lumaPos().y );
  cu.mvpIdx [eRefList] = mvp_idx;
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, mvp_idx );
}

void CABACReader::ciip_flag( CodingUnit& cu )
{
  cu.setCiipFlag( m_BinDecoder.decodeBin( Ctx::CiipFlag() ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "ciip_flag() Ciip=%d pos=(%d,%d) size=%dx%d\n", cu.ciipFlag() ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height );
}



//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( depth )
//    bool  cbf_comp            ( area, depth )
//================================================================================

void CABACReader::transform_tree( CodingStructure &cs, CodingUnit &cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  const UnitArea& area    = partitioner.currArea();
                          
  // split_transform_flag
  bool            split   = area.Y().width > partitioner.maxTrSize || area.Y().height > partitioner.maxTrSize;
  const PartSplit ispType = CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType ) );

  split |= ( cu.sbtInfo() || ispType != TU_NO_ISP ) && partitioner.currTrDepth == 0;

  if( split )
  {
    if( ispType == TU_NO_ISP && !cu.sbtInfo() )
    {
#if ENABLE_TRACING
      const CompArea& tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE( g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n",
              partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height );

#endif
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( ispType != TU_NO_ISP )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else if( cu.sbtInfo() )
    {
      partitioner.splitCurrArea( PartSplit( CU::getSbtTuSplit( cu ) ), cs );
    }
    else
      THROW_RECOVERABLE( "Implicit TU split not available!" );

    do
    {
      transform_tree( cs, cu, partitioner, cuCtx );
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit( cs );
  }
  else
  {
    TransformUnit &tu = cs.addTU( getArea( *m_slice, area, partitioner.chType, partitioner.treeType ), partitioner.chType, cu );
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType()].x, tu.blocks[tu.chType()].y, tu.blocks[tu.chType()].width, tu.blocks[tu.chType()].height, cu.depth, partitioner.currTrDepth );

    transform_unit( tu, cuCtx, partitioner );
  }
}

bool CABACReader::cbf_comp( CodingUnit& cu, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
  const CtxSet&   ctxSet  = Ctx::QtCbf[ area.compID()];

  unsigned  cbf = 0;
  if( ( area.compID() == COMPONENT_Y && cu.bdpcmMode() ) || ( area.compID() != COMPONENT_Y && cu.bdpcmModeChroma() ) )
  {
    if( area.compID() == COMPONENT_Cr )
      cbf = m_BinDecoder.decodeBin( ctxSet( 2 ) );
    else
      cbf = m_BinDecoder.decodeBin( ctxSet( 1 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID(), area.x, area.y, area.compID() == COMPONENT_Cr ? 2 : 1, cbf );
  }
  else
  {
    const unsigned  ctxId = DeriveCtx::CtxQtCbf( area.compID(), prevCbf, useISP && isLuma( area.compID() ) );
    cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );

    DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID(), area.x, area.y, ctxId, cbf );
  }

  return cbf;
}





//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( cu, refList )
//================================================================================

void CABACReader::mvd_coding( Mv &rMvd )
{
  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs)
  {
    horAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }
  if (verAbs)
  {
    verAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs)
  {
    if (horAbs > 1)
    {
      horAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
    if (m_BinDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    if (verAbs > 1)
    {
      verAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
    if (m_BinDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);

  CHECKD( !( ( horAbs >= MVD_MIN ) && ( horAbs <= MVD_MAX ) ) || !( ( verAbs >= MVD_MIN ) && ( verAbs <= MVD_MAX ) ), "Illegal MVD value" );
}


//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================

void CABACReader::transform_unit( TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner )
{
  const UnitArea& area    = partitioner.currArea();
  const unsigned  trDepth = partitioner.currTrDepth;

  CodingUnit&      cu     = *tu.cu;
  ChromaCbfs       chromaCbfs;

  const bool chromaCbfISP = isChromaEnabled( area.chromaFormat ) && area.blocks[COMPONENT_Cb].valid() && cu.ispMode();
  const bool tuNoResidual = TU::checkTuNoResidual( tu, partitioner.currPartIdx() );

  // cbf_cb & cbf_cr
  if( area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && ( !CU::isSepTree( cu ) || partitioner.chType == CHANNEL_TYPE_CHROMA ) && ( !cu.ispMode() || chromaCbfISP ) )
  {
    const int cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;

    if( !( cu.sbtInfo() && tuNoResidual ) )
    {
      chromaCbfs.Cb = cbf_comp( cu, area.blocks[COMPONENT_Cb], cbfDepth );
      chromaCbfs.Cr = cbf_comp( cu, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb );
    }
  }

  if( !isChroma( partitioner.chType ) )
  {
    if( !CU::isIntra( cu ) && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) )
    {
      TU::setCbf( tu, COMPONENT_Y, true );
    }
    else if( cu.sbtInfo() && tuNoResidual )
    {
      TU::setCbf( tu, COMPONENT_Y, false );
    }
    else if( cu.sbtInfo() && !chromaCbfs.sigChroma( area.chromaFormat ) )
    {
      CHECKD( tuNoResidual, "wrong" );
      TU::setCbf( tu, COMPONENT_Y, true );
    }
    else
    {
      bool cbfY = true;

      if( cu.ispMode() )
      {
        bool lumaCbfIsInferredACT = ( cu.colorTransform() && cu.predMode() == MODE_INTRA && trDepth == 0 && !chromaCbfs.sigChroma( area.chromaFormat ) );
        bool lastCbfIsInferred    = lumaCbfIsInferredACT; // ISP and ACT are mutually exclusive
        bool rootCbfSoFar         = false;

        int nTus = cu.ispMode() == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> getLog2( tu.lheight() ) : cu.lwidth() >> getLog2( tu.lwidth() );
        int idx  = partitioner.currPartIdx();
        if( idx == nTus - 1 )
        {
          for( const TransformUnit &currTu : cTUTraverser( &cu.firstTU, cu.lastTU ) )
          {
            rootCbfSoFar |= TU::getCbf( currTu, COMPONENT_Y );
          }

          if( !rootCbfSoFar )
          {
            lastCbfIsInferred = true;
          }
        }

        if( !lastCbfIsInferred )
        {
          cbfY = cbf_comp( cu, tu.Y(), trDepth, TU::getPrevTUCbf( tu, COMPONENT_Y ), cu.ispMode() );
        }
        else
        {
          cbfY = true;
        }
      }
      else
      {
        cbfY = cbf_comp( cu, tu.Y(), trDepth, false, NOT_INTRA_SUBPARTITIONS );
      }
      TU::setCbf( tu, COMPONENT_Y, cbfY ? 1 : 0 );
    }
  }

  if( area.chromaFormat != CHROMA_400 && ( !cu.ispMode() || chromaCbfISP ) )
  {
    TU::setCbf( tu, COMPONENT_Cb, chromaCbfs.Cb );
    TU::setCbf( tu, COMPONENT_Cr, chromaCbfs.Cr );
  }

  cu.setPlaneCbf( COMPONENT_Y , cu.planeCbf( COMPONENT_Y  ) || TU::getCbf( tu, COMPONENT_Y )  );
  cu.setPlaneCbf( COMPONENT_Cb, cu.planeCbf( COMPONENT_Cb ) || TU::getCbf( tu, COMPONENT_Cb ) );
  cu.setPlaneCbf( COMPONENT_Cr, cu.planeCbf( COMPONENT_Cr ) || TU::getCbf( tu, COMPONENT_Cr ) );

  const bool lumaOnly   = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  const bool cbfLuma    = TU::getCbf( tu, COMPONENT_Y );
  const bool cbfChroma  = ( lumaOnly ? false : ( chromaCbfs.Cb || chromaCbfs.Cr ) );

  if( cu.lwidth() > 64 || cu.lheight() > 64 || cbfLuma || cbfChroma )
  {
    if( cu.pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      if( !CU::isSepTree( cu ) || isLuma( tu.chType() ) )
      {
        cu_qp_delta(cu, cuCtx.qp, cu.qp);
        cuCtx.qp = cu.qp;
        cuCtx.isDQPCoded = true;
      }
    }

    if( !CU::isSepTree( cu ) || isChroma( tu.chType() ) )   // !DUAL_TREE_LUMA
    {
      const SizeType channelWidth  = !CU::isSepTree( cu ) ? cu.lwidth()  : cu.chromaSize().width;
      const SizeType channelHeight = !CU::isSepTree( cu ) ? cu.lheight() : cu.chromaSize().height;

      if( cu.slice->getUseChromaQpAdj() && ( channelWidth > 64 || channelHeight > 64 || cbfChroma ) && !cuCtx.isChromaQpAdjCoded )
      {
        cu_chroma_qp_offset( cu );
        cuCtx.isChromaQpAdjCoded = true;
      }
    }

    if( !lumaOnly )
    {
      joint_cb_cr( tu, ( TU::getCbf( tu, COMPONENT_Cb ) ? 2 : 0 ) + ( TU::getCbf( tu, COMPONENT_Cr ) ? 1 : 0 ) );

      if( tu.jointCbCr )
      {
        cu.setPlaneCbf( COMPONENT_Cb, true );
        cu.setPlaneCbf( COMPONENT_Cr, true );
      }
    }

    if( cbfLuma )
    {
      residual_coding( tu, COMPONENT_Y, cuCtx );
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( TU::getCbf( tu, compID ) )
        {
          residual_coding( tu, compID, cuCtx );
        }
      }
    }
  }
}

void CABACReader::cu_qp_delta( CodingUnit& cu, int predQP, int8_t& qp )
{
  CHECK( predQP == std::numeric_limits<int>::max(), "Invalid predicted QP" );
  int qpY = predQP;
  int DQp = unary_max_symbol( Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( DQp >= CU_DQP_TU_CMAX )
  {
    DQp += exp_golomb_eqprob( CU_DQP_EG_k  );
  }
  if( DQp > 0 )
  {
    if( m_BinDecoder.decodeBinEP( ) )
    {
      DQp = -DQp;
    }
    int     qpBdOffsetY = cu.sps->getQpBDOffset();
    qpY = ( (predQP + DQp + (MAX_QP + 1) + 2 * qpBdOffsetY) % ((MAX_QP + 1) + qpBdOffsetY)) - qpBdOffsetY;
  }
  qp = (int8_t)qpY;

  DTRACE( g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType()].lumaPos( cu.chromaFormat ).x, cu.blocks[cu.chType()].lumaPos( cu.chromaFormat ).y, cu.qtDepth, predQP, DQp, qp );
}


void CABACReader::cu_chroma_qp_offset( CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  int       length  = cu.pps->getChromaQpOffsetListLen();
  unsigned  qpAdj   = m_BinDecoder.decodeBin( Ctx::ChromaQpAdjFlag() );
  if( qpAdj && length > 1 )
  {
    // cu_chroma_qp_offset_idx
    qpAdj += unary_max_symbol( Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu.chromaQpAdj = cu.cs->chromaQpAdj = qpAdj;
}





//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    bool        transform_skip_flag     ( tu, compID )
//    RDPCMMode   explicit_rdpcm_mode     ( tu, compID )
//    int         last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACReader::joint_cb_cr( TransformUnit& tu, const int cbfMask )
{
  if( !tu.cu->sps->getJointCbCrEnabledFlag() )
  {
    return;
  }

  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
  {
    tu.jointCbCr = ( m_BinDecoder.decodeBin( Ctx::JointCbCrFlag( cbfMask-1 ) ) ? cbfMask : 0 );
  }
}

void CABACReader::residual_coding( TransformUnit& tu, ComponentID compID, CUCtx& cuCtx )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_PARSERESIDUALS, *tu.cu->cs, compID );
  const CodingUnit& cu = *tu.cu;

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID(), tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode() );

  if( compID == COMPONENT_Cr && tu.jointCbCr == 3 )
    return;

  // parse transform skip and explicit rdpcm mode
  ts_flag( tu, compID );

  if( tu.mtsIdx( compID ) == MTS_SKIP && !cu.slice->getTSResidualCodingDisabledFlag() )
  {
    residual_codingTS( tu, compID );
    return;
  }

  // determine sign hiding
  bool signHiding = m_slice->getSignDataHidingEnabledFlag();
  CoeffCodingContext  cctx( tu, compID, signHiding, m_tplBuf );
  // parse last coeff position
  cctx.setScanPosLast( last_sig_coeff( cctx, tu, compID ) );
  if (tu.mtsIdx( compID ) != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4 )
  {
    const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
    cuCtx.violatesLfnstConstrained[ toChannelType(compID) ] |= cctx.scanPosLast() > maxLfnstPos;
  }
  if( tu.mtsIdx( compID ) != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4 )
  {
    const int lfnstLastScanPosTh = isLuma( compID ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx.lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
  if( isLuma( compID ) && tu.mtsIdx( compID ) != MTS_SKIP )
  {
    cuCtx.mtsLastScanPos |= cctx.scanPosLast() >= 1;
  }

  // parse subblocks
  const int stateTransTab = ( m_slice->getDepQuantEnabledFlag() ? 32040 : 0 );
  int       state         = 0;

  TCoeffSig *coeff = m_cffTmp;
  ::memset( coeff, 0, cctx.maxNumCoeff() * sizeof( TCoeffSig ) );

  int *sigPos      = m_blkPos;
  int  sigSubSetId = 0;

  int maxX = 0;
  int maxY = 0;

  const bool skipBlkPreCond = compID == COMPONENT_Y && tu.cu->sps->getUseMTS() && tu.cu->sbtInfo() != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32;
  for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock( subSetId );

    if( skipBlkPreCond )
    {
      if( ( cctx.height() == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) ) || ( cctx.width() == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth() ) ) )
      {
        continue;
      }
    }

    int numSigCoef = residual_coding_subblock( cctx, coeff, stateTransTab, state, m_signVal[sigSubSetId], sigPos, m_sub1[sigSubSetId] );

    if( numSigCoef > 0 )
    {
      m_numSig[sigSubSetId] = numSigCoef;
      sigSubSetId++;

      maxX = std::max( maxX, cctx.cgPosX() );
      maxY = std::max( maxY, cctx.cgPosY() );
    }
    if( isLuma( compID ) && cctx.isSigGroup() && ( cctx.cgPosY() > 3 || cctx.cgPosX() > 3 ) )
    {
      cuCtx.violatesMtsCoeffConstraint = true;
    }
  }

  if( cctx.bdpcm() )
  {
    maxX = cctx.width();
    maxY = cctx.height();
  }
  else
  {
    maxX++;
    maxY++;
    maxX <<= cctx.log2CGWidth();
    maxY <<= cctx.log2CGHeight();
  }

  // if not TU split, otherwise already memset
  PelBuf pb = cu.cs->getRecoBuf( CompArea( compID, tu.blocks[compID].pos(), Size( maxX, maxY ) ) );
  pb.memset( 0 );

  const bool depQuant = tu.cu->slice->getDepQuantEnabledFlag() && ( tu.mtsIdx( compID ) != MTS_SKIP );
  CoeffSigBuf dstCff = pb;

  for( sigPos--, sigSubSetId--; sigSubSetId >= 0; sigSubSetId-- )
  {
    unsigned numNonZero  = m_numSig [sigSubSetId];
    unsigned signPattern = m_signVal[sigSubSetId];
    unsigned sub1Pattern = m_sub1   [sigSubSetId];

    //===== set final coefficents =====
    for( unsigned k = 0; k < numNonZero; k++, sigPos--, signPattern >>= 1, sub1Pattern >>= 1 )
    {
      const int blkPos        = *sigPos;
      const int posX          = cctx.posX( blkPos );
      const int posY          = cctx.posY( blkPos );

      int AbsCoeff            = depQuant ? ( coeff[blkPos] << 1 ) - ( ( ( int ) sub1Pattern ) & 1 ) : coeff[blkPos];
      dstCff.at( posX, posY ) = ( signPattern & 1u ? -AbsCoeff : AbsCoeff );
    }
  }

  if( cctx.scanPosLast() == 0 )
  {
    tu.maxScanPosX[compID] = 0;
    tu.maxScanPosY[compID] = 0;
  }
  else
  {
    tu.maxScanPosX[compID] = maxX - 1;
    tu.maxScanPosY[compID] = maxY - 1;
  }
}

void CABACReader::ts_flag( TransformUnit& tu, ComponentID compID )
{
  int tsFlag = ( ( tu.cu->bdpcmMode() && isLuma( compID ) ) || ( tu.cu->bdpcmModeChroma() && isChroma( compID ) ) ) ? 1 : tu.mtsIdx( compID ) == MTS_SKIP ? 1 : 0;
  int ctxIdx = isLuma(compID) ? 4 : 5;

  if( TU::isTSAllowed ( tu, compID ) )
  {
    tsFlag = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );
  }
  
  tu.setMtsIdx( compID, tsFlag ? MTS_SKIP : MTS_DCT2_DCT2 );
  
  DTRACE(g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), tsFlag);
}

void CABACReader::mts_idx( CodingUnit& cu, CUCtx& cuCtx )
{
  TransformUnit &tu = cu.firstTU;
  int        mtsIdx = tu.mtsIdx( COMPONENT_Y ); // Transform skip flag has already been decoded
  
  if( CU::isMTSAllowed( cu, COMPONENT_Y ) && !cuCtx.violatesMtsCoeffConstraint &&
      cuCtx.mtsLastScanPos && cu.lfnstIdx() == 0 && mtsIdx != MTS_SKIP )
  {
    int ctxIdx = 0;
    int symbol = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );
    
    if( symbol )
    {
      ctxIdx = 1;
      mtsIdx = MTS_DST7_DST7; // mtsIdx = 2 -- 4
      for( int i = 0; i < 3; i++, ctxIdx++ )
      {
        symbol  = m_BinDecoder.decodeBin( Ctx::MTSIndex( ctxIdx ) );
        mtsIdx += symbol;
        
        if( !symbol )
        {
          break;
        }
      }
    }
  }
  
  tu.setMtsIdx( COMPONENT_Y, mtsIdx );
  
  DTRACE(g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
}

void CABACReader::isp_mode( CodingUnit& cu )
{
  if( cu.multiRefIdx() || !cu.sps->getUseISP() || cu.bdpcmMode() || cu.colorTransform() )
  {
    //cu.setIspMode( NOT_INTRA_SUBPARTITIONS );
    return;
  }

  const ISPType allowedSplits = CU::canUseISPSplit( cu, COMPONENT_Y );

  if( allowedSplits == NOT_INTRA_SUBPARTITIONS )
  {
    //cu.setIspMode( NOT_INTRA_SUBPARTITIONS );
    return;
  }

  //cu.setIspMode( NOT_INTRA_SUBPARTITIONS );
  int symbol = m_BinDecoder.decodeBin( Ctx::ISPMode( 0 ) );

  if( symbol )
  {
    if( allowedSplits == HOR_INTRA_SUBPARTITIONS )
    {
      cu.setIspMode( HOR_INTRA_SUBPARTITIONS );
    }
    else if( allowedSplits == VER_INTRA_SUBPARTITIONS )
    {
      cu.setIspMode( VER_INTRA_SUBPARTITIONS );
    }
    else
    {
      cu.setIspMode( 1 + m_BinDecoder.decodeBin( Ctx::ISPMode( 1 ) ) );
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType(), cu.blocks[cu.chType()].x, cu.blocks[cu.chType()].y, (int)cu.ispMode() );
}

void CABACReader::residual_lfnst_mode( CodingUnit& cu,  CUCtx& cuCtx  )
{
  if( !cu.sps->getUseLFNST() || !CU::isIntra( cu ) )
  {
    //cu.setLfnstIdx( 0 );
    return;
  }

  const int chIdx  = CU::isSepTree( cu ) && cu.chType() == CHANNEL_TYPE_CHROMA ? 1 : 0;
  if( ( cu.ispMode() && !CU::canUseLfnstWithISP( cu, cu.chType() ) ) ||
      ( cu.mipFlag() && !    allowLfnstWithMip ( cu.lumaSize() ) ) ||
      ( cu.chType() == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[1].width, cu.blocks[1].height ) < 4 )
    )
  {
    return;
  }

  const Size lSize = cu.blocks[chIdx].lumaSize( cu.chromaFormat );
  if( lSize.width > cu.sps->getMaxTbSize() || lSize.height > cu.sps->getMaxTbSize() ) return;

  {
    const bool lumaFlag   = CU::isSepTree( cu ) ?   isLuma( cu.chType() ) : true;
    const bool chromaFlag = CU::isSepTree( cu ) ? isChroma( cu.chType() ) : true;
    const bool nonZeroCoeffNonTsCorner8x8
                          = ( lumaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] );
    bool isTrSkip = false;
    for( auto &currTU : cTUTraverser( &cu.firstTU, cu.lastTU->next ) )
    {
      const uint32_t numValidComp = getNumberValidComponents( cu.chromaFormat );
      for( uint32_t compID = COMPONENT_Y; compID < numValidComp; compID++ )
      {
        if( currTU.blocks[compID].valid() && TU::getCbf( currTU, ( ComponentID ) compID ) && currTU.mtsIdx( compID ) == MTS_SKIP )
        {
          isTrSkip = true;
          break;
        }
      }
    }
    if( nonZeroCoeffNonTsCorner8x8 || ( !cuCtx.lfnstLastScanPos && !cu.ispMode() ) || isTrSkip )
    {
      //cu.setLfnstIdx( 0 );
      return;
    }
  }


  unsigned cctx = 0;

  if( CU::isSepTree( cu ) ) cctx++;

  unsigned idxLFNST = m_BinDecoder.decodeBin( Ctx::LFNSTIdx( cctx ) );

  if( idxLFNST )
  {
    idxLFNST += m_BinDecoder.decodeBin( Ctx::LFNSTIdx(2) );
  }

  cu.setLfnstIdx( idxLFNST );

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx() );
}

int CABACReader::last_sig_coeff( CoeffCodingContext& cctx, TransformUnit& tu, ComponentID compID )
{
  unsigned PosLastX = 0, PosLastY = 0;
  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

  if( isLuma( compID ) && tu.cu->sps->getUseMTS() && tu.cu->sbtInfo() != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 )
  {
    maxLastPosX = ( tu.blocks[ compID ].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[ compID ].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }

  for( ; PosLastX < maxLastPosX; PosLastX++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastXCtxId( PosLastX ) ) )
    {
      break;
    }
  }
  for( ; PosLastY < maxLastPosY; PosLastY++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastYCtxId( PosLastY ) ) )
    {
      break;
    }
  }
  if( PosLastX > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastX - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastX = g_uiMinInGroup[ PosLastX ] + uiTemp;
  }
  if( PosLastY > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastY - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastY = g_uiMinInGroup[ PosLastY ] + uiTemp;
  }

  int blkPos;
  {
    blkPos = PosLastX + ( PosLastY * cctx.width() );
  }

  for( int scanPos = 0; scanPos < cctx.maxNumCoeff() - 1; scanPos++ )
  {
    if( blkPos == cctx.blockPos( scanPos ) )
    {
      return scanPos;
    }
  }

  return cctx.maxNumCoeff() - 1;
}


int CABACReader::residual_coding_subblock(CoeffCodingContext& cctx, TCoeffSig* coeff, const int stateTransTable, int& state, unsigned& signPattern, int *&sigPos, unsigned &stateVal)
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function

  //===== init =====
  const int   minSubPos = cctx.minSubPos();
  const bool  isLast = cctx.isLast();
  int         firstSigPos = (isLast ? cctx.scanPosLast() : cctx.maxSubPos());
  int         nextSigPos = firstSigPos;

  //===== decode significant_coeffgroup_flag =====
  bool sigGroup = (isLast || !minSubPos);
  if (!sigGroup)
  {
    sigGroup = m_BinDecoder.decodeBin(cctx.sigGroupCtxId());
  }
  if (sigGroup)
  {
    cctx.setSigGroup();
  }
  else
  {
    return 0;
  }

  // make sure only takes up single L1 block
  ALIGN_DATA( 64, int     gt1Pos[16] );
  int *    gt1PosPtr    = gt1Pos;

  //===== decode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? (cctx.isNotFirst() ? minSubPos : -1) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  int       numNonZero    = 0, numGt1 = 0;
  int       remRegBins    = cctx.regBinLimit();
  int       gt1Mode1      = 0;
  unsigned  gt2Mask       = 0;
            stateVal      = 0;

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    int  blkPos  = cctx.blockPos(nextSigPos);
    bool sigFlag = (!numNonZero && nextSigPos == inferSigPos);

    unsigned absVal = 0;

    if (!sigFlag)
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbs( blkPos, state );
      sigFlag = m_BinDecoder.decodeBin( sigCtxId );
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId);
      remRegBins--;
    }

    if (sigFlag)
    {
      uint8_t ctxOff = cctx.ctxOffsetAbs();
      stateVal  = ( ( state >> 1 ) & 1 ) | ( stateVal << 1 );
      *sigPos++ = blkPos;
      numNonZero++;
      firstNZPos = nextSigPos;
      lastNZPos  = std::max<int>( lastNZPos, nextSigPos );
      unsigned gt1Flag = m_BinDecoder.decodeBin(cctx.greater1CtxIdAbs(ctxOff));
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1Flag, cctx.greater1CtxIdAbs(ctxOff));
      remRegBins--;

      unsigned gt2Flag = 0;

      if (gt1Flag)
      {
        unsigned parFlag = m_BinDecoder.decodeBin(cctx.parityCtxIdAbs(ctxOff));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbs(ctxOff));
        numGt1++;
        remRegBins--;
        gt2Flag = m_BinDecoder.decodeBin( cctx.greater2CtxIdAbs( ctxOff ) );
        gt2Mask               |= ( gt2Flag << (numGt1-1) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2Flag, cctx.greater2CtxIdAbs( ctxOff ) );
        remRegBins--;
        *gt1PosPtr++    = blkPos;
        absVal          = 2 + parFlag + (gt2Flag << 1);
        state           = ( stateTransTable >> ( ( state << 2 ) + ( parFlag << 1 ) ) ) & 3;
      }
      else
      {
        absVal = 1;
        state  = ( stateTransTable >> ( ( state << 2 ) + 2 ) ) & 3;
      }

      cctx.absVal1stPass( blkPos, coeff, absVal );
    }
    else
    {
      state = ( stateTransTable >> ( state << 2 ) ) & 3;
    }
  }

  cctx.setRegBinLimit( remRegBins );

  gt1Mode1 = numGt1;

  gt1PosPtr = gt1Pos;

  //===== 3rd PASS: Go-rice codes =====
  for( int k = 0; k < gt1Mode1; k++, gt2Mask >>= 1, gt1PosPtr++ )
  {
    if( gt2Mask & 1 )
    {
      int      sumAll  = cctx.templateAbsSum( *gt1PosPtr, coeff, 4 );
      unsigned ricePar = g_auiGoRiceParsCoeff[sumAll];

      int rem = m_BinDecoder.decodeRemAbsEP( ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
      coeff[ *gt1PosPtr ] += ( rem << 1 );
    }
  }


  //===== coeff bypass ====
  for( ; nextSigPos >= minSubPos; nextSigPos-- )
  {
    int       sub1      = ( state >> 1 ) & 1;
    int       blkPos    = cctx.blockPos( nextSigPos );
    int       sumAll    = cctx.templateAbsSum( blkPos, coeff, 0 );
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0(state, rice);
    int       rem       = m_BinDecoder.decodeRemAbsEP( rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    TCoeffSig tcoeff    = ( rem == pos0 ? 0 : rem < pos0 ? rem+1 : rem );
    state               = ( stateTransTable >> ( ( state << 2 ) + ( ( tcoeff & 1 ) << 1 ) ) ) & 3;
    if( tcoeff )
    {
      coeff[blkPos] = tcoeff;
      stateVal      = sub1 | ( stateVal << 1 );
      *sigPos++     = blkPos;
      numNonZero++;
      firstNZPos = nextSigPos;
      lastNZPos  = std::max<int>( lastNZPos, nextSigPos );
    }
  }

  //===== decode sign's =====
  const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
  signPattern = m_BinDecoder.decodeBinsEP( numSigns );
  if( numNonZero > numSigns )
  {
    sigPos -= numNonZero;
    int sumAbs = 0;
    for( int i = 0; i < numNonZero; i++ )
    {
      const int blockPos = *sigPos;
      sumAbs += coeff[blockPos];
      sigPos++;
    }
    signPattern <<= 1;
    signPattern += (sumAbs & 1);
  }
  return numNonZero;
}

void CABACReader::residual_codingTS( TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID(), tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // if not TU split, otherwise already memset
  PelBuf pb = tu.cu->cs->getRecoBuf( tu.blocks[compID] );
  pb.memset( 0 );

  // init coeff coding context
  CoeffCodingContext  cctx( tu, compID, false, m_tplBuf );
  TCoeffSig *coeff = m_cffTmp;
  ::memset( coeff, 0, cctx.maxNumCoeff() * sizeof( TCoeffSig ) );

  int maxCtxBins = ( cctx.maxNumCoeff() * 7 ) >> 2;
  cctx.setNumCtxBins( maxCtxBins );

  int maxX = 0;
  int maxY = 0;

  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId );
    residual_coding_subblockTS( cctx, coeff, tu.cu->cs->getRecoBuf( tu.block( compID ) ), maxX, maxY );
  }

  if( cctx.bdpcm() )
  {
    tu.maxScanPosX[compID] = cctx.width();
    tu.maxScanPosY[compID] = cctx.height();
  }
  else
  {
    tu.maxScanPosX[compID] = maxX;
    tu.maxScanPosY[compID] = maxY;
  }
}

void CABACReader::residual_coding_subblockTS( CoeffCodingContext& cctx, TCoeffSig* coeff, CoeffSigBuf dstcoeff, int& maxX, int& maxY )
{
  // TODO: awi, profile and optimize similar to residual_coding_subblock(...)

  // NOTE: All coefficients of the subblock must be set to zero before calling this function
  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;
  unsigned    signPattern = 0;

  //===== decode significant_coeffgroup_flag =====
  bool sigGroup = cctx.isLastSubSet() && cctx.noneSigGroup();
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId( true ) );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", sigGroup, cctx.sigGroupCtxId() );
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  //===== decode absolute values =====
  const int inferSigPos   = minSubPos;
  int       numNonZero    =  0;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.numCtxBins() >= 4; nextSigPos++)
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );

    if( !sigFlag )
    {
        const unsigned sigCtxId = cctx.sigCtxIdAbsTS( blkPos, coeff );
        sigFlag = m_BinDecoder.decodeBin( sigCtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
        cctx.decNumCtxBins(1);
    }

    if( sigFlag )
    {
      //===== decode sign's =====
      int sign;
        const unsigned signCtxId = cctx.signCtxIdAbsTS(blkPos, coeff, cctx.bdpcm());
        sign = m_BinDecoder.decodeBin(signCtxId);
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sign() bin=%d ctx=%d  nextSigPos=%d  blkPos=%d\n", sign, signCtxId, nextSigPos, blkPos );
        cctx.decNumCtxBins(1);

      signPattern += ( sign << numNonZero );

      sigBlkPos[numNonZero++] = blkPos;

      unsigned gt1Flag;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(blkPos, coeff, cctx.bdpcm());
        gt1Flag = m_BinDecoder.decodeBin(gt1CtxId);
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1Flag, gt1CtxId );
        cctx.decNumCtxBins(1);

      unsigned parFlag = 0;
      if( gt1Flag )
      {
          parFlag = m_BinDecoder.decodeBin( cctx.parityCtxIdAbsTS() );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbsTS() );
          cctx.decNumCtxBins(1);
      }
      coeff[ blkPos ] = (sign ? -1 : 1 ) * (1 + parFlag + gt1Flag);
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "coeff[ blkPos ]=%d  blkPos=%d\n", coeff[ blkPos ], blkPos  );
    }
    lastScanPosPass1 = nextSigPos;
  }

  int cutoffVal = 2;
  int numGtBins = 4;

  //===== 2nd PASS: gt2 =====
  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.numCtxBins() >= 4; scanPos++)
  {
    TCoeffSig& tcoeff = coeff[cctx.blockPos( scanPos )];
    cutoffVal = 2;
    for( int i = 0; i < numGtBins; i++ )
    {
      if( tcoeff < 0 )
      {
        tcoeff = -tcoeff;
      }

      if( tcoeff >= cutoffVal )
      {
        unsigned gt2Flag;
          gt2Flag = m_BinDecoder.decodeBin( cctx.greaterXCtxIdAbsTS( cutoffVal >> 1 ) );
          tcoeff += ( gt2Flag << 1 );
          DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2Flag, cctx.greaterXCtxIdAbsTS( cutoffVal >> 1 ), scanPos, tcoeff );
            cctx.decNumCtxBins(1);
      }
      cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }
  //===== 3rd PASS: Go-rice codes =====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    TCoeffSig& tcoeff = coeff[ cctx.blockPos( scanPos ) ];

    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    if (tcoeff < 0)
    {
      tcoeff = -tcoeff;
    }

    if( tcoeff >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( cctx.blockPos( scanPos ), coeff );
      int       rem  = m_BinDecoder.decodeRemAbsEP( rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );
      tcoeff += (scanPos <= lastScanPosPass1) ? (rem << 1) : rem;
      if (tcoeff && scanPos > lastScanPosPass1)
      {
        int      blkPos = cctx.blockPos(scanPos);
        int sign = m_BinDecoder.decodeBinEP();
        signPattern += (sign << numNonZero);
        sigBlkPos[numNonZero++] = blkPos;
      }
    }
    if (!cctx.bdpcm() && cutoffVal)
    {
      if (tcoeff > 0)
      {
        int rightPixel, belowPixel;
        cctx.neighTS(rightPixel, belowPixel, cctx.blockPos( scanPos ), coeff);
        tcoeff = cctx.decDeriveModCoeff(rightPixel, belowPixel, tcoeff);
      }
    }
  }

  //===== set final coefficents =====
  for( unsigned k = 0; k < numNonZero; k++ )
  {
    int AbsCoeff              = coeff[ sigBlkPos[ k ] ];
    int blkPos                = sigBlkPos[ k ];
    const int posX            = cctx.posX( blkPos );
    const int posY            = cctx.posY( blkPos );
    maxX                      = std::max<int>( maxX, posX );
    maxY                      = std::max<int>( maxY, posY );

    dstcoeff.at( posX, posY ) = ( signPattern & 1u ? -AbsCoeff : AbsCoeff );
    coeff[ sigBlkPos[ k ] ]   = ( signPattern & 1u ? -AbsCoeff : AbsCoeff );
    signPattern         >>= 1;
  }
}


//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    unsigned  unary_max_symbol ( ctxId0, ctxId1, maxSymbol )
//    unsigned  unary_max_eqprob (                 maxSymbol )
//    unsigned  exp_golomb_eqprob( count )
//================================================================================

unsigned CABACReader::unary_max_symbol( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol  )
{
  unsigned onesRead = 0;
  while( onesRead < maxSymbol && m_BinDecoder.decodeBin( onesRead == 0 ? ctxId0 : ctxIdN ) == 1 )
  {
    ++onesRead;
  }
  return onesRead;
}


unsigned CABACReader::unary_max_eqprob( unsigned maxSymbol )
{
  for( unsigned k = 0; k < maxSymbol; k++ )
  {
    if( !m_BinDecoder.decodeBinEP() )
    {
      return k;
    }
  }
  return maxSymbol;
}


unsigned CABACReader::exp_golomb_eqprob( unsigned count )
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  while( bit )
  {
    bit     = m_BinDecoder.decodeBinEP( );
    symbol += bit << count++;
  }
  if( --count )
  {
    symbol += m_BinDecoder.decodeBinsEP( count );
  }
  return symbol;
}

unsigned CABACReader::code_unary_fixed( unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  unsigned idx;
  bool unary = m_BinDecoder.decodeBin( ctxId );
  if( unary )
  {
    idx = unary_max_eqprob( unary_max );
  }
  else
  {
    idx = unary_max + 1 + m_BinDecoder.decodeBinsEP( fixed );
  }
  return idx;
}

void CABACReader::mip_flag( CodingUnit& cu )
{
  if( !cu.sps->getUseMIP() )
  {
    //cu.setMipFlag( false );
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  cu.setMipFlag( m_BinDecoder.decodeBin( Ctx::MipFlag( ctxId ) ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag() ? 1 : 0 );
}

void CABACReader::mip_pred_mode( CodingUnit &cu )
{
  cu.setMipTransposedFlag( !!m_BinDecoder.decodeBinEP() );

  uint32_t mipMode;
  const int numModes = getNumModesMip( cu.Y() );
  xReadTruncBinCode( mipMode, numModes );
  cu.intraDir[CHANNEL_TYPE_LUMA] = mipMode;
  CHECKD( cu.intraDir[CHANNEL_TYPE_LUMA] < 0 || cu.intraDir[CHANNEL_TYPE_LUMA] >= numModes, "Invalid MIP mode" );

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.intraDir[CHANNEL_TYPE_LUMA], cu.mipTransposedFlag() ? 1 : 0 );
}

}
