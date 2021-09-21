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

/** \file     DecCu.cpp
    \brief    CU decoder class
*/

#include "DecCu.h"

#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/TimeProfiler.h"

#include "CommonLib/dtrace_buffer.h"

namespace vvdec
{

//! \ingroup DecoderLib
//! \{

void DecCu::TaskDeriveCtuMotionInfo( CodingStructure &cs, const UnitArea &ctuArea, MotionHist& hist )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, cs, CH_L );

  const unsigned  ctuRsAddr      = getCtuAddr( Position( ctuArea.lumaPos().x, ctuArea.lumaPos().y ), *cs.pcv );
  const unsigned  ctuXPosInCtus  = ctuRsAddr % cs.pcv->widthInCtus;
  const unsigned  tileColIdx     = cs.pps->ctuToTileCol( ctuXPosInCtus );
  const unsigned  tileXPosInCtus = cs.pps->getTileColumnBd( tileColIdx );

  if( ctuXPosInCtus == tileXPosInCtus )
  {
    hist.motionLut.   resize(0);
    hist.motionLutIbc.resize(0);
  }
  
  for( auto &currCU : cs.traverseCUs( ctuArea ) )
  {
    CHECK( !ctuArea.blocks[currCU.chType()].contains( currCU.blocks[currCU.chType()] ), "Should never happen!" );

#if JVET_O1170_CHECK_BV_AT_DECODER
    if( currCU.Y().valid() && currCU.sps->getIBCFlag() )
    {
      const int vSize = cs.sps->getMaxCUHeight() > 64 ? 64 : cs.sps->getMaxCUHeight();
      if( ( currCU.Y().x % vSize ) == 0 && ( currCU.Y().y % vSize ) == 0 )
      {
        m_pcInterPred->resetVPDUforIBC( cs.pcv->chrFormat, cs.sps->getMaxCUHeight(), vSize, currCU.Y().x  + g_IBCBufferSize / cs.sps->getMaxCUHeight() / 2, currCU.Y().y );
      }
    }
#endif

    if( !CU::isIntra( currCU ) )
    {
      xDeriveCUMV( currCU, hist );
    }
  }
}

void DecCu::TaskTrafoCtu( CodingStructure &cs, const UnitArea &ctuArea )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_ITRANS_REC, cs, CH_L );

  for( auto &currCU : cs.traverseCUs( ctuArea ) )
  {
    CHECK( !ctuArea.blocks[currCU.chType()].contains( currCU.blocks[currCU.chType()] ), "Should never happen!" );

    if( currCU.rootCbf() )
    {
      reconstructResi( currCU );
    }
  }
}

void DecCu::TaskInterCtu( CodingStructure &cs, const UnitArea &ctuArea )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, cs, CH_L );

  for( auto &currCU: cs.traverseCUs( ctuArea ) )
  {
    CHECK( !ctuArea.blocks[currCU.chType()].contains( currCU.blocks[currCU.chType()] ), "Should never happen!" );

    if( !CU::isIntra( currCU ) && !CU::isIBC( currCU ) )
    {
      predAndReco( currCU, false );
    }
  }
}

void DecCu::TaskCriticalIntraKernel( CodingStructure &cs, const UnitArea &ctuArea )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, cs, CH_L );
  for( auto &currCU : cs.traverseCUs( ctuArea ) )
  {
    CHECK( !ctuArea.blocks[currCU.chType()].contains( currCU.blocks[currCU.chType()] ), "Should never happen!" );

    if( CU::isIntra( currCU ) || currCU.ciipFlag() || CU::isIBC( currCU ) )
    {
      predAndReco( currCU, true );
    }
    else if( currCU.rootCbf() )
    {
      finishLMCSAndReco( currCU );
    }

    if( cs.sps->getIBCFlag() )
    {
      cs.fillIBCbuffer( currCU, ctuArea.Y().y / cs.sps->getMaxCUHeight() );
    }
  }
}

void DecCu::TaskDeriveDMVRMotionInfo( CodingStructure& cs, const UnitArea& ctuArea )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, cs, CH_L );
  MotionBuf   mb     = cs.getMotionBuf( ctuArea.Y() );
  MotionInfo* orgPtr = mb.buf;

  for( CodingUnit &cu : cs.traverseCUs( ctuArea ) )
  {
    CHECKD( !ctuArea.blocks[cu.chType()].contains( cu.blocks[cu.chType()] ), "Should never happen!" );

    PredictionUnit &pu = cu;
    
    if( isLuma( pu.chType() ) && pu.dmvrCondition() )
    {
      const int dy = std::min<int>( pu.lumaSize().height, DMVR_SUBCU_HEIGHT );
      const int dx = std::min<int>( pu.lumaSize().width,  DMVR_SUBCU_WIDTH );
      
      static constexpr unsigned scale = 4 * AMVP_DECIMATION_FACTOR;
      static constexpr unsigned mask  = scale - 1;

      const Position puPos = pu.lumaPos();
      const Mv mv0 = pu.mv[0][0];
      const Mv mv1 = pu.mv[1][0];

      for( int y = puPos.y, num = 0; y < ( puPos.y + pu.lumaSize().height ); y = y + dy )
      {
        for( int x = puPos.x; x < ( puPos.x + pu.lumaSize().width ); x = x + dx, num++ )
        {
          const Mv subPuMv0 = mv0 + pu.mvdL0SubPu[num];
          const Mv subPuMv1 = mv1 - pu.mvdL0SubPu[num];

          int y2 = ( ( y - 1 ) & ~mask ) + scale;

          for( ; y2 < y + dy; y2 += scale )
          {
            int x2 = ( ( x - 1 ) & ~mask ) + scale;

            for( ; x2 < x + dx; x2 += scale )
            {
              mb.buf = orgPtr + cs.inCtuPos( Position{ x2, y2 }, CH_L );

              MotionInfo& mi = *mb.buf;

              mi.mv[0] = subPuMv0;
              mi.mv[1] = subPuMv1;
            }
          }
        }
      }
    }
  }
}
// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

DecCu::DecCu()
{
  create();
}

DecCu::~DecCu()
{
  destroy();
}

void DecCu::init( IntraPrediction* pcIntra, InterPrediction* pcInter, Reshape *pcReshape, TrQuant *pcTrQuant )
{
  m_pcIntraPred  = pcIntra;
  m_pcInterPred  = pcInter;
  m_pcTrQuant    = pcTrQuant;
  m_pcReshape    = pcReshape;
}

void DecCu::create()
{
}

void DecCu::destroy()
{
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void DecCu::predAndReco( CodingUnit& cu, bool doCiipIntra )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_ITRANS_REC, *cu.cs, CH_L );
  CodingStructure &cs = *cu.cs;
  
  if( CU::isIntra( cu ) )
  {
    if( cu.colorTransform() )
    {
      xIntraRecACT( cu );
      return;
    }

    for( TransformUnit &tu : TUTraverser( &cu.firstTU, cu.lastTU->next ) )
    {
      for( const CompArea &area : tu.blocks )
      {
        if( !area.valid() )
        {
          continue;
        }

        const ComponentID compID = area.compID();
        const ChannelType chType = toChannelType( compID );

        PelBuf piPred;
        {
          PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_INTRAPRED, cs, compID );

          if( chType == CHANNEL_TYPE_LUMA )
          {
            Position pos = Position( tu.Y().x - cu.lumaPos().x, tu.Y().y - cu.lumaPos().y );
            piPred = cs.getPredBuf( cu ).Y().subBuf( pos, tu.lumaSize() );
          }
          else
          {
            piPred = cs.getPredBuf( cu ).Cb().subBuf( Position( 0, 0 ), tu.chromaSize() );
          }

          const PredictionUnit &pu      = cu;
          const uint32_t uiChFinalMode  = PU::getFinalIntraMode( pu, chType );

          if( CU::isMIP( cu, chType ) )
          {
            m_pcIntraPred->initIntraPatternChType( tu, area );
            m_pcIntraPred->initIntraMip( pu, area );
            m_pcIntraPred->predIntraMip( compID, piPred, pu );
          }
          else if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
          {
            m_pcIntraPred->initIntraPatternChType( tu, area );
            m_pcIntraPred->xGetLumaRecPixels( pu, area );
            m_pcIntraPred->predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
          }
          else
          {
            const bool predRegDiffFromTB = CU::isPredRegDiffFromTB( *tu.cu, compID );
            const bool firstTBInPredReg  = CU::isFirstTBInPredReg( *tu.cu, compID, area );
            CompArea areaPredReg( COMPONENT_Y, area );

            const bool bUseFilteredPredictions = isLuma( compID ) && cu.ispMode() == NOT_INTRA_SUBPARTITIONS && IntraPrediction::useFilteredIntraRefSamples( compID, pu, tu );

            if( tu.cu->ispMode() && isLuma( compID ) )
            {
              PelBuf piReco = cs.getRecoBuf( area );
              if( predRegDiffFromTB )
              {
                if( firstTBInPredReg )
                {
                  CU::adjustPredArea( areaPredReg );
                  m_pcIntraPred->initIntraPatternChTypeISP( *tu.cu, areaPredReg, piReco );
                }
              }
              else
              {
                m_pcIntraPred->initIntraPatternChTypeISP( *tu.cu, area, piReco );
              }
            }
            else
            {
              m_pcIntraPred->initIntraPatternChType( tu, area, bUseFilteredPredictions );
            }

            if( predRegDiffFromTB )
            {
              if( firstTBInPredReg )
              {
                Position pos( areaPredReg.x - cu.lumaPos().x, areaPredReg.y - cu.lumaPos().y );
                piPred      = cs.getPredBuf( cu ).Y().subBuf( pos, areaPredReg.size() );

                m_pcIntraPred->predIntraAng( compID, piPred, pu, bUseFilteredPredictions );

                piPred.width = area.width;
              }
            }
            else
              m_pcIntraPred->predIntraAng( compID, piPred, pu, bUseFilteredPredictions );
          }
        }

        //===== inverse transform =====
        PelBuf piReco = cs.getRecoBuf( area );
        PelBuf piResi = cs.getRecoBuf( area );

        const Slice &slice      = *cu.slice;
        const bool   doChrScale = isChroma( compID )
                                  && slice.getLmcsEnabledFlag()
                                  && slice.getPicHeader()->getLmcsChromaResidualScaleFlag()
                                  && tu.blocks[compID].area() > 4
                                  && ( TU::getCbf( tu, compID ) || tu.jointCbCr )
                                  ;

        if( doChrScale )
        {
          const Area area       = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType(), CHANNEL_TYPE_LUMA, tu.blocks[tu.chType()].pos()), recalcSize(tu.chromaFormat, tu.chType(), CHANNEL_TYPE_LUMA, tu.blocks[tu.chType()].size()));
          int chromaResScaleInv = m_pcReshape->calculateChromaAdjVpduNei( tu, area.pos() );
          piReco.scaleSignal( chromaResScaleInv, slice.clpRng( compID ) );
        }

        if( TU::getCbf( tu, compID ) || ( isChroma( compID ) && tu.jointCbCr ) )
        {
          piReco.reconstruct( piPred, piResi, slice.clpRng( compID ) );
        }
        else
        {
          piReco.copyFrom( piPred );
        }
      }
    }
  }
  else
  {
    const UnitArea& cuArea = cu;

    PelUnitBuf predBuf = cu.rootCbf() ? cs.getPredBuf( cu ) : cs.getRecoBuf( cuArea );
    PelUnitBuf recoBuf = cs.getRecoBuf( cuArea );

    // CBF in at least one channel, but no TU split
    if( cu.rootCbf() )
    {
      for( const auto &blk : cu.blocks )
      {
        if( blk.valid() && !cu.planeCbf( blk.compID() ) )
        {
          predBuf.bufs[blk.compID()] = recoBuf.bufs[blk.compID()];
        }
      }
    }

    if( cu.geoFlag() )
    {
      m_pcInterPred->motionCompensationGeo( cu, predBuf );
    }
    else
    {
      if( doCiipIntra )
        m_pcIntraPred->geneIntrainterPred( cu );

      // inter prediction
      CHECK( CU::isIBC( cu ) && cu.ciipFlag(),       "IBC and CIIP cannot be used together"   );
      CHECK( CU::isIBC( cu ) && cu.affineFlag(),     "IBC and AFFINE cannot be used together" );
      CHECK( CU::isIBC( cu ) && cu.geoFlag(),        "IBC and GEO cannot be used together"    );
      CHECK(CU::isIBC(cu) && cu.mmvdFlag(),          "IBC and MMVD cannot be used together"     );

      if( !CU::isIBC( cu ) && !doCiipIntra )
      {
        m_pcInterPred->motionCompensation( cu, predBuf );
      }
      else if( CU::isIBC( cu ) )
      {
        const bool luma   = cu.Y ().valid();
        const bool chroma = isChromaEnabled(cu.chromaFormat) && cu.Cb().valid();
        m_pcInterPred->motionCompensation( cu, predBuf, luma, chroma );
      }
    }

    if( cu.ciipFlag() && doCiipIntra )
    {
      m_pcIntraPred->geneWeightedPred( COMPONENT_Y, predBuf.Y(), cu, m_pcIntraPred->getPredictorPtr2( COMPONENT_Y ) );

      if( isChromaEnabled( cu.chromaFormat ) && cu.chromaSize().width > 2 )
      {
        m_pcIntraPred->geneWeightedPred( COMPONENT_Cb, predBuf.Cb(), cu, m_pcIntraPred->getPredictorPtr2( COMPONENT_Cb ) );
        m_pcIntraPred->geneWeightedPred( COMPONENT_Cr, predBuf.Cr(), cu, m_pcIntraPred->getPredictorPtr2( COMPONENT_Cr ) );
      }
    }

    DTRACE    ( g_trace_ctx, D_TMP, "pred " );
    DTRACE_CRC( g_trace_ctx, D_TMP, *cu.cs, predBuf, &cu.Y() );

    if( cu.rootCbf() )
    {
      if( cu.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag( *cu.slice ) && !doCiipIntra )
      {
         m_pcReshape->rspBufFwd( predBuf.Y() );
      }

      if( ( cu.ciipFlag() && doCiipIntra ) || CU::isIBC( cu ) )
      {
        finishLMCSAndReco( cu );
      }
    }
    else
    {
      if( cu.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag( *cu.slice ) && !CU::isIBC( cu ) )
      {
        m_pcReshape->rspBufFwd( predBuf.Y() );
      }
    }

    DTRACE    ( g_trace_ctx, D_TMP, "reco " );
    DTRACE_CRC( g_trace_ctx, D_TMP, *cu.cs, cu.cs->getRecoBuf( cuArea ), &cu.Y() );
  }
}

void DecCu::finishLMCSAndReco( CodingUnit &cu )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_ITRANS_REC, *cu.cs, CH_L );
  CodingStructure &cs = *cu.cs;

  const uint32_t uiNumVaildComp = getNumberValidComponents( cu.chromaFormat );
  const bool     doCS           = cs.picHeader->getLmcsEnabledFlag() && cs.picHeader->getLmcsChromaResidualScaleFlag() && cu.slice->getLmcsEnabledFlag();
  const PelUnitBuf predUnitBuf  = cs.getPredBuf( cu );

  for( auto& currTU : TUTraverser( &cu.firstTU, cu.lastTU->next ) )
  {
    int chromaResScaleInv = 0;

    for( uint32_t ch = 0; ch < uiNumVaildComp; ch++ )
    {
      const ComponentID compID = ComponentID( ch );
        
      if( doCS )
      {
        if( isLuma( compID ) )
        {
          chromaResScaleInv = m_pcReshape->calculateChromaAdjVpduNei( currTU, currTU.blocks[COMPONENT_Y] );
        }
        else if( ( TU::getCbf( currTU, compID ) || currTU.jointCbCr ) && currTU.blocks[compID].area() > 4 )
        {
          const CompArea &area = currTU.blocks[compID];
          PelBuf resiBuf       = cs.getRecoBuf( area );
          resiBuf.scaleSignal( chromaResScaleInv, cu.slice->clpRng( compID ) );
        }
      }

      if( currTU.blocks[compID].valid() )
      {
        CPelBuf predBuf = predUnitBuf.bufs[compID];
        OFFSET( predBuf.buf, predBuf.stride, currTU.blocks[compID].x - cu.blocks[compID].x, currTU.blocks[compID].y - cu.blocks[compID].y );
        //CPelBuf resiBuf = cs.getRecoBuf( currTU.blocks[compID] );
         PelBuf recoBuf = cs.getRecoBuf( currTU.blocks[compID] );
         predBuf.width  = recoBuf.width;
         predBuf.height = recoBuf.height;

        if( TU::getCbf( currTU, compID ) || ( isChroma( compID ) && currTU.jointCbCr ) )
        {
          recoBuf.reconstruct( predBuf, recoBuf, cu.slice->clpRngs() );
        }
        else if( cu.planeCbf( compID ) )
        {
          recoBuf.copyFrom( predBuf );
        }
      }
    }
  }
}

void DecCu::reconstructResi( CodingUnit &cu )
{
  CodingStructure &cs = *cu.cs;

  for( TransformUnit &tu : TUTraverser( &cu.firstTU, cu.lastTU->next ) )
  {
    for( const auto &area : tu.blocks )
    {
      if( !area.valid() )
      {
        continue;
      }

      const ComponentID compID = area.compID();

      //===== inverse transform =====
      PelBuf piResi = cs.getRecoBuf( area );

      if( tu.jointCbCr && isChroma( compID ) )
      {
        if( compID == COMPONENT_Cb )
        {
          PelBuf resiCr = cs.getRecoBuf( tu.blocks[ COMPONENT_Cr ] );
          if( tu.jointCbCr >> 1 )
          {
            QpParam qpCb( tu, compID );
            m_pcTrQuant->invTransformNxN( tu, COMPONENT_Cb, piResi, qpCb );
          }
          else
          {
            QpParam qpCr( tu, COMPONENT_Cr );
            m_pcTrQuant->invTransformNxN( tu, COMPONENT_Cr, resiCr, qpCr );
          }
          m_pcTrQuant->invTransformICT( tu, piResi, resiCr );
        }
      }
      else if( TU::getCbf( tu, compID ) )
      {
        QpParam cQP( tu, compID );
        m_pcTrQuant->invTransformNxN( tu, compID, piResi, cQP );
      }
    }
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/** Function for deriving reconstructed luma/chroma samples of a PCM mode CU.
* \param pcCU pointer to current CU
* \param uiPartIdx part index
* \param piPCM pointer to PCM code arrays
* \param piReco pointer to reconstructed sample arrays
* \param uiStride stride of reconstructed sample arrays
* \param uiWidth CU width
* \param uiHeight CU height
* \param compID colour component ID
* \returns void
*/

void DecCu::xIntraRecACT( CodingUnit &cu )
{
  CodingStructure &cs  = *cu.cs;
  const Slice &slice   = *cu.slice;

  for( TransformUnit &tu : TUTraverser( &cu.firstTU, cu.lastTU->next ) )
  {
    cs.getRecoBuf( tu ).colorSpaceConvert( cs.getRecoBuf( tu ), slice.clpRng( COMPONENT_Y ) );

    for( const CompArea &area : tu.blocks )
    {
      const ComponentID compID = area.compID();

      if( compID == COMPONENT_Y )
      {
        continue;
      }
      //===== inverse transform =====
      PelBuf piReco = cs.getRecoBuf( area );

      const bool   doChrScale = isChroma( compID )
                                && slice.getLmcsEnabledFlag() 
                                && slice.getPicHeader()->getLmcsChromaResidualScaleFlag()
                                && tu.blocks[compID].area() > 4
                                && ( TU::getCbf( tu, compID ) || tu.jointCbCr )
                                ;

      if( doChrScale )
      {
        const Area area       = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType(), CHANNEL_TYPE_LUMA, tu.blocks[tu.chType()].pos()), recalcSize(tu.chromaFormat, tu.chType(), CHANNEL_TYPE_LUMA, tu.blocks[tu.chType()].size()));
        int chromaResScaleInv = m_pcReshape->calculateChromaAdjVpduNei( tu, area.pos() );
        piReco.scaleSignal( chromaResScaleInv, slice.clpRng( compID ) );
      }
    }

    for( const CompArea &area : tu.blocks )
    {
      if( !area.valid() )
      {
        continue;
      }

      const ComponentID compID = area.compID();
      const ChannelType chType = toChannelType( compID );

      PelBuf piPred;
      if( chType == CHANNEL_TYPE_LUMA )
      {
        Position pos = Position( tu.Y().x - cu.lumaPos().x, tu.Y().y - cu.lumaPos().y );
        piPred = cs.getPredBuf( cu ).Y().subBuf( pos, tu.lumaSize() );
      }
      else
      {
        piPred = cs.getPredBuf( cu ).Cb().subBuf( Position( 0, 0 ), tu.chromaSize() );
      }

      const PredictionUnit &pu      = cu;
      const uint32_t uiChFinalMode  = PU::getFinalIntraMode( pu, chType );

      bool predRegDiffFromTB = CU::isPredRegDiffFromTB( *tu.cu, compID );
      bool firstTBInPredReg  = CU::isFirstTBInPredReg( *tu.cu, compID, area );
      CompArea areaPredReg   ( COMPONENT_Y, area );

      if( CU::isMIP( cu, chType ) )
      {
        m_pcIntraPred->initIntraPatternChType( tu, area );
        m_pcIntraPred->initIntraMip( pu, area );
        m_pcIntraPred->predIntraMip( compID, piPred, pu );
      }
      else
      {
        const bool bUseFilteredPredictions = isLuma( chType ) && cu.ispMode() == NOT_INTRA_SUBPARTITIONS && IntraPrediction::useFilteredIntraRefSamples( compID, pu, tu );
        
        if( tu.cu->ispMode() && isLuma( compID ) )
        {
          PelBuf piReco = cs.getRecoBuf( area );
          if( predRegDiffFromTB )
          {
            if( firstTBInPredReg )
            {
              CU::adjustPredArea( areaPredReg );
              m_pcIntraPred->initIntraPatternChTypeISP( *tu.cu, areaPredReg, piReco );
            }
          }
          else
          {
            m_pcIntraPred->initIntraPatternChTypeISP( *tu.cu, area, piReco );
          }
        }
        else
        {
          m_pcIntraPred->initIntraPatternChType( tu, area, bUseFilteredPredictions );
        }

        if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
        {
          m_pcIntraPred->xGetLumaRecPixels( pu, area );
          m_pcIntraPred->predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
        }
        else
        {
          if( predRegDiffFromTB )
          {
            if( firstTBInPredReg )
            {
              Position pos( areaPredReg.x - cu.lumaPos().x, areaPredReg.y - cu.lumaPos().y );
              piPred      = cs.getPredBuf( cu ).Y().subBuf( pos, areaPredReg.size() );

              m_pcIntraPred->predIntraAng( compID, piPred, pu, bUseFilteredPredictions );
            }
          }
          else
            m_pcIntraPred->predIntraAng( compID, piPred, pu, bUseFilteredPredictions );
        }
      }

      PelBuf piReco = cs.getRecoBuf( area );
      PelBuf piResi = cs.getRecoBuf( area );

      piReco.reconstruct( piPred, piResi, slice.clpRng( compID ) );
    }
  }
}

void DecCu::xDeriveCUMV( CodingUnit &cu, MotionHist& hist )
{
  PredictionUnit& pu = cu;
  MergeCtx mrgCtx;

  if( pu.mergeFlag() )
  {
    if( pu.mmvdFlag() )
    {
      int fPosBaseIdx = pu.mmvdIdx / MMVD_MAX_REFINE_NUM;

      PU::getInterMergeCandidates     ( pu, mrgCtx, hist, fPosBaseIdx + 1 );
      PU::getInterMMVDMergeCandidates ( pu, mrgCtx,       pu.mmvdIdx );
      mrgCtx.setMmvdMergeCandiInfo    ( pu, pu.mmvdIdx );

      PU::spanMotionInfo              ( pu, mrgCtx );
    }
    else if( pu.geoFlag() )
    {
      PU::getGeoMergeCandidates( pu, m_geoMrgCtx, hist );

      const uint8_t splitDir = pu.geoSplitDir;
      const uint8_t candIdx0 = pu.geoMergeIdx0();
      const uint8_t candIdx1 = pu.geoMergeIdx1();

      PU::spanGeoMotionInfo( pu, m_geoMrgCtx, splitDir, candIdx0, candIdx1 );
    }
    else if( pu.affineFlag() )
    {
      AffineMergeCtx affineMergeCtx;

      if( pu.sps->getSBTMVPEnabledFlag() )
      {
        mrgCtx.subPuMvpMiBuf  = cu.getMotionBuf();
        affineMergeCtx.mrgCtx = &mrgCtx;
      }

      int mergeIdx = pu.mergeIdx();
      PU::getAffineMergeCand( pu, affineMergeCtx, mergeIdx );

      pu.setInterDir        ( affineMergeCtx.interDirNeighbours[mergeIdx] );
      pu.setAffineType      ( affineMergeCtx.affineType        [mergeIdx] );
      pu.setBcwIdx          ( affineMergeCtx.BcwIdx            [mergeIdx] );
      pu.setMergeType       ( affineMergeCtx.mergeType         [mergeIdx] );

      if( pu.mergeType() == MRG_TYPE_SUBPU_ATMVP )
      {
        pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[( pu.mergeIdx() << 1 ) + 0][0].refIdx;
        pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[( pu.mergeIdx() << 1 ) + 1][0].refIdx;
      }
      else
      {
        for( int i = 0; i < 2; ++i )
        {
          if( pu.slice->getNumRefIdx( RefPicList( i ) ) > 0 )
          {
            MvField* mvField = affineMergeCtx.mvFieldNeighbours[( pu.mergeIdx() << 1 ) + i];

            PU::setAllAffineMvField( pu, mvField, RefPicList( i ) );
          }
        }
      }

      PU::spanMotionInfo( pu, mrgCtx );
    }
    else
    {
      if( CU::isIBC( pu ) )
        PU::getIBCMergeCandidates  ( pu, mrgCtx, hist, pu.mergeIdx() );
      else
        PU::getInterMergeCandidates( pu, mrgCtx, hist, pu.mergeIdx() );

      mrgCtx.setMergeInfo         ( pu, pu.mergeIdx() );

      PU::spanMotionInfo          ( pu, mrgCtx );
    }
  }
  else
  {
    if( pu.imv() && !pu.affineFlag() )
    {
      PU::applyImv      ( pu, hist );
      PU::spanMotionInfo( pu );
    }
    else
    {
      if( pu.affineFlag() )
      {
        for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
        {
          RefPicList eRefList = RefPicList( uiRefListIdx );
          if( pu.slice->getNumRefIdx( eRefList ) > 0 && ( pu.interDir() & ( 1 << uiRefListIdx ) ) )
          {
            AffineAMVPInfo affineAMVPInfo;
            PU::fillAffineMvpCand( pu, eRefList, pu.refIdx[eRefList], affineAMVPInfo );

            const unsigned mvp_idx = pu.mvpIdx[eRefList];

            //    Mv mv[3];
            CHECK( pu.refIdx[eRefList] < 0, "Unexpected negative refIdx." );
            const int imvShift = pu.imv() == 2 ? MV_FRACTIONAL_BITS_DIFF : 0;
            Mv mv0 = pu.mv[eRefList][0];
            Mv mv1 = pu.mv[eRefList][1];

            mv0 <<= imvShift;
            mv1 <<= imvShift;

            Mv mvLT = affineAMVPInfo.mvCandLT[mvp_idx] + mv0;
            Mv mvRT = affineAMVPInfo.mvCandRT[mvp_idx] + mv1;
            mvRT   += mv0;
            if( pu.imv() != 1 )
            {
              mvLT.changePrecision( MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL );
              mvRT.changePrecision( MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL );
            }

            Mv mvLB;
            if( pu.affineType() == AFFINEMODEL_6PARAM )
            {
              Mv mv2 = pu.mv[eRefList][2];
              mv2 <<= imvShift;
              mvLB = affineAMVPInfo.mvCandLB[mvp_idx] + mv2;
              mvLB += mv0;
              if ( pu.imv() != 1 )
              {
                mvLB.changePrecision( MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL );
              }
            }
            PU::setAllAffineMv( pu, mvLT, mvRT, mvLB, eRefList, true );
          }
        }
      }
      else if( CU::isIBC( pu ) && pu.interDir() == 1 )
      {
        AMVPInfo amvpInfo;
        PU::fillIBCMvpCand( pu, amvpInfo, hist );
        Mv mvd = pu.mv[REF_PIC_LIST_0][0];
        mvd <<= 4;
        if( pu.slice->getPicHeader()->getMaxNumIBCMergeCand() == 1 )
        {
          CHECK( pu.mvpIdx[REF_PIC_LIST_0], "mvpIdx for IBC mode should be 0" );
        }
        pu.mv[REF_PIC_LIST_0][0] = amvpInfo.mvCand[pu.mvpIdx[REF_PIC_LIST_0]] + mvd;
        pu.mv[REF_PIC_LIST_0][0] . mvCliptoStorageBitDepth();
      }
      else
      {
        for( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
        {
          RefPicList eRefList = RefPicList( uiRefListIdx );
          if( ( pu.slice->getNumRefIdx( eRefList ) > 0 || ( eRefList == REF_PIC_LIST_0 && CU::isIBC( pu ) ) ) && ( pu.interDir() & ( 1 << uiRefListIdx ) ) )
          {
            AMVPInfo amvpInfo;
            PU::fillMvpCand( pu, eRefList, pu.refIdx[eRefList], amvpInfo, hist );

            Mv mvd              = pu.mv[eRefList][0];
            mvd                 . changePrecisionAmvr( pu.imv(), MV_PRECISION_INTERNAL);
            pu.mv [eRefList][0] = amvpInfo.mvCand[pu.mvpIdx[eRefList]] + mvd;
            pu.mv [eRefList][0] . mvCliptoStorageBitDepth();
          }
        }
      }
      PU::spanMotionInfo( pu, mrgCtx );
    }
  }

  bool isIbcSmallBlk = CU::isIBC( cu ) && ( cu.lwidth() * cu.lheight() <= 16 );
  if( !pu.affineFlag() && !pu.geoFlag() && !isIbcSmallBlk )
  {
    const unsigned log2ParallelMergeLevel = (pu.sps->getLog2ParallelMergeLevelMinus2() + 2);
    const unsigned xBr = pu.Y().width  + pu.Y().x;
    const unsigned yBr = pu.Y().height + pu.Y().y;
    bool enableHmvp      = ((xBr >> log2ParallelMergeLevel) > (pu.Y().x >> log2ParallelMergeLevel)) && ((yBr >> log2ParallelMergeLevel) > (pu.Y().y >> log2ParallelMergeLevel));
    bool enableInsertion = CU::isIBC( cu ) || enableHmvp;

    if( enableInsertion )
    {
      HPMVInfo mi( pu.getMotionInfo(), pu.interDir() == 3 ? pu.BcwIdx() : BCW_DEFAULT, pu.imv() == IMV_HPEL );
      MotionHist::addMiToLut( CU::isIBC( cu ) ? hist.motionLutIbc : hist.motionLut, mi );
    }
  }
}

}
