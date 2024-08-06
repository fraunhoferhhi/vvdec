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

/** \file     DecCu.cpp
    \brief    CU decoder class
*/

#include "DecCu.h"

#include "CommonLib/InterPrediction.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/TimeProfiler.h"

#include "CommonLib/dtrace_buffer.h"

namespace vvdec
{

//! \ingroup DecoderLib
//! \{

void DecCu::TaskDeriveCtuMotionInfo( CodingStructure &cs, const int ctuRsAddr, const UnitArea &ctuArea, MotionHist& hist )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, cs, CH_L );

  const unsigned  ctuXPosInCtus  = ctuRsAddr % cs.pcv->widthInCtus;
  const unsigned  tileColIdx     = cs.pps->ctuToTileCol( ctuXPosInCtus );
  const unsigned  tileXPosInCtus = cs.pps->getTileColumnBd( tileColIdx );

  if( ctuXPosInCtus == tileXPosInCtus )
  {
    hist.motionLut.   resize(0);
    hist.motionLutIbc.resize(0);
  }
  
  for( auto &currCU : cs.traverseCUs( ctuRsAddr ) )
  {
    CHECK( !ctuArea.blocks[currCU.chType()].contains( currCU.blocks[currCU.chType()] ),
           "Traversing CU at (" << currCU.blocks[currCU.chType()].x << "," << currCU.blocks[currCU.chType()].y
           << ") outside of the CTU at (" << ctuArea.blocks[currCU.chType()].x << "," << ctuArea.blocks[currCU.chType()].y << ")!"
    );

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
    else
    {
      MotionBuf mb = currCU.getMotionBuf();
      mb.memset( MI_NOT_VALID );
    }
  }
}

void DecCu::TaskTrafoCtu( CodingStructure &cs, const int ctuRsAddr, const UnitArea &ctuArea )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_ITRANS_REC, cs, CH_L );

  for( auto &currCU : cs.traverseCUs( ctuRsAddr ) )
  {
    CHECK( !ctuArea.blocks[currCU.chType()].contains( currCU.blocks[currCU.chType()] ), "Should never happen!" );

    if( currCU.rootCbf() )
    {
      reconstructResi( currCU );
    }
  }
}

void DecCu::TaskInterCtu( CodingStructure &cs, const int ctuRsAddr, const UnitArea &ctuArea )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, cs, CH_L );

  for( auto &currCU: cs.traverseCUs( ctuRsAddr ) )
  {
    CHECK( !ctuArea.blocks[currCU.chType()].contains( currCU.blocks[currCU.chType()] ), "Should never happen!" );

    if( !CU::isIntra( currCU ) && !CU::isIBC( currCU ) )
    {
      predAndReco( currCU, false );
    }
  }
}

void DecCu::TaskCriticalIntraKernel( CodingStructure &cs, const int ctuRsAddr, const UnitArea &ctuArea )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, cs, CH_L );
  const CtuData &ctuData = cs.getCtuData( ctuRsAddr );

  for( auto &currCU : cs.traverseCUs( ctuRsAddr ) )
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

    if( cs.sps->getIBCFlag() && cs.hasIbcBlock[ctuData.lineIdx] )
    {
      cs.fillIBCbuffer( currCU, ctuData.lineIdx );
    }
  }
}

void DecCu::TaskFinishMotionInfo( CodingStructure &cs, const int ctuRsAddr, const int col, const int row )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_CONTROL_PARSE_DERIVE_LL, cs, CH_L );

  // first, finish DMVR motion

  UnitArea    ctuArea = getCtuArea( cs, col, row, true );
  MotionBuf   mb      = cs.getMotionBuf( ctuArea.Y() );
  MotionInfo* orgPtr  = mb.buf;

  for( CodingUnit &cu : cs.traverseCUs( ctuRsAddr ) )
  {
    CHECKD( !ctuArea.blocks[cu.chType()].contains( cu.blocks[cu.chType()] ), "Should never happen!" );
    
    if( isLuma( cu.chType() ) && cu.dmvrCondition() )
    {
      const int dy = std::min<int>( cu.lumaSize().height, DMVR_SUBCU_HEIGHT );
      const int dx = std::min<int>( cu.lumaSize().width,  DMVR_SUBCU_WIDTH );
      
      static constexpr unsigned scale = 4 * AMVP_DECIMATION_FACTOR;
      static constexpr unsigned mask  = scale - 1;

      const Position puPos = cu.lumaPos();
      const Mv mv0 = cu.mv[0][0];
      const Mv mv1 = cu.mv[1][0];

      for( int y = puPos.y, num = 0; y < ( puPos.y + cu.lumaSize().height ); y = y + dy )
      {
        for( int x = puPos.x; x < ( puPos.x + cu.lumaSize().width ); x = x + dx, num++ )
        {
          const Mv subPuMv0 = mv0 + cs.m_dmvrMvCache[cu.mvdL0SubPuOff + num];
          const Mv subPuMv1 = mv1 - cs.m_dmvrMvCache[cu.mvdL0SubPuOff + num];

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

  CtuData &ctuData = cs.getCtuData( ctuRsAddr );

  const int size4x4 = (int)cs.get4x4MapStride();

  const MotionInfo          *src = ctuData.motion;
        ColocatedMotionInfo *dst = ctuData.colMotion;

  // ctuDta.colMotion should already be set to '0'
  //memset( dst, 0, sizeof( ctuData.colMotion ) );

  // skip every second source line
  for( int y = 0; y < size4x4; y += 2, src += size4x4 )
  {
    for( int x = 0; x < size4x4; x += 2, src += 2, dst++ )
    {
      //if( src->isInter() )
      {
        *dst = *src;
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

          if( cu.planeCbf( compID ) )
          {
            if( chType == CHANNEL_TYPE_LUMA )
            {
              Position pos = Position( tu.Y().x - cu.lumaPos().x, tu.Y().y - cu.lumaPos().y );
              piPred = cs.getPredBuf( cu ).Y().subBuf( pos, tu.lumaSize() );
            }
            else
            {
              piPred = cs.getPredBuf( cu ).Cb().subBuf( Position( 0, 0 ), tu.chromaSize() );
            }
          }
          else
          {
            piPred = cs.getRecoBuf( area );
          }

          const uint32_t uiChFinalMode  = PU::getFinalIntraMode( cu, chType );

          if( CU::isMIP( cu, chType ) )
          {
            m_pcIntraPred->initIntraPatternChType( tu, area );
            m_pcIntraPred->initIntraMip( cu, area );
            m_pcIntraPred->predIntraMip( compID, piPred, cu );
          }
          else if( compID != COMPONENT_Y && PU::isLMCMode( uiChFinalMode ) )
          {
            m_pcIntraPred->initIntraPatternChType( tu, area );
            m_pcIntraPred->xGetLumaRecPixels( cu, area );
            m_pcIntraPred->predIntraChromaLM( compID, piPred, cu, area, uiChFinalMode );
          }
          else
          {
            const bool predRegDiffFromTB = CU::isPredRegDiffFromTB( *tu.cu, compID );
            const bool firstTBInPredReg  = CU::isFirstTBInPredReg( *tu.cu, compID, area );
            CompArea areaPredReg( COMPONENT_Y, area );

            const bool bUseFilteredPredictions = isLuma( compID ) && cu.ispMode() == NOT_INTRA_SUBPARTITIONS && IntraPrediction::useFilteredIntraRefSamples( compID, cu, tu );

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

                m_pcIntraPred->predIntraAng( compID, piPred, cu, bUseFilteredPredictions );

                piPred.width = area.width;
              }
            }
            else
              m_pcIntraPred->predIntraAng( compID, piPred, cu, bUseFilteredPredictions );
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
        else if( cu.planeCbf( compID ) )
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
      // inter prediction
      CHECK( CU::isIBC( cu ) && cu.ciipFlag(),       "IBC and CIIP cannot be used together"   );
      CHECK( CU::isIBC( cu ) && cu.affineFlag(),     "IBC and AFFINE cannot be used together" );
      CHECK( CU::isIBC( cu ) && cu.geoFlag(),        "IBC and GEO cannot be used together"    );
      CHECK( CU::isIBC( cu ) && cu.mmvdFlag(),       "IBC and MMVD cannot be used together"     );

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
      m_pcIntraPred->predBlendIntraCiip( predBuf, cu );
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

      const uint32_t uiChFinalMode  = PU::getFinalIntraMode( cu, chType );

      bool predRegDiffFromTB = CU::isPredRegDiffFromTB( *tu.cu, compID );
      bool firstTBInPredReg  = CU::isFirstTBInPredReg( *tu.cu, compID, area );
      CompArea areaPredReg   ( COMPONENT_Y, area );

      if( CU::isMIP( cu, chType ) )
      {
        m_pcIntraPred->initIntraPatternChType( tu, area );
        m_pcIntraPred->initIntraMip( cu, area );
        m_pcIntraPred->predIntraMip( compID, piPred, cu );
      }
      else
      {
        const bool bUseFilteredPredictions = isLuma( chType ) && cu.ispMode() == NOT_INTRA_SUBPARTITIONS && IntraPrediction::useFilteredIntraRefSamples( compID, cu, tu );
        
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
          m_pcIntraPred->xGetLumaRecPixels( cu, area );
          m_pcIntraPred->predIntraChromaLM( compID, piPred, cu, area, uiChFinalMode );
        }
        else
        {
          if( predRegDiffFromTB )
          {
            if( firstTBInPredReg )
            {
              Position pos( areaPredReg.x - cu.lumaPos().x, areaPredReg.y - cu.lumaPos().y );
              piPred      = cs.getPredBuf( cu ).Y().subBuf( pos, areaPredReg.size() );

              m_pcIntraPred->predIntraAng( compID, piPred, cu, bUseFilteredPredictions );
            }
          }
          else
            m_pcIntraPred->predIntraAng( compID, piPred, cu, bUseFilteredPredictions );
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
  if( cu.mergeFlag() )
  {
    MergeCtx mrgCtx;

    if( cu.mmvdFlag() )
    {
      int fPosBaseIdx = cu.mmvdIdx / MMVD_MAX_REFINE_NUM;

      PU::getInterMergeCandidates     ( cu, mrgCtx, hist, fPosBaseIdx + 1 );
      PU::getInterMMVDMergeCandidates ( cu, mrgCtx,       cu.mmvdIdx );
      mrgCtx.setMmvdMergeCandiInfo    ( cu, cu.mmvdIdx );

      PU::spanMotionInfo              ( cu );
    }
    else if( cu.geoFlag() )
    {
      PU::getGeoMergeCandidates( cu, m_geoMrgCtx, hist );

      const uint8_t splitDir = cu.geoSplitDir;
      const uint8_t candIdx0 = cu.geoMergeIdx0();
      const uint8_t candIdx1 = cu.geoMergeIdx1();

      PU::spanGeoMotionInfo( cu, m_geoMrgCtx, splitDir, candIdx0, candIdx1 );
    }
    else if( cu.affineFlag() )
    {
      AffineMergeCtx affineMergeCtx;

      if( cu.sps->getSBTMVPEnabledFlag() )
      {
        affineMergeCtx.subPuMvpMiBuf = cu.getMotionBuf();
      }

      int mergeIdx = cu.mergeIdx();
      PU::getAffineMergeCand( cu, affineMergeCtx, mergeIdx );

      cu.setInterDir        ( affineMergeCtx.interDirNeighbours[mergeIdx] );
      cu.setAffineType      ( affineMergeCtx.affineType        [mergeIdx] );
      cu.setBcwIdx          ( affineMergeCtx.BcwIdx            [mergeIdx] );
      cu.setMergeType       ( affineMergeCtx.mergeType         [mergeIdx] );

      if( cu.mergeType() == MRG_TYPE_SUBPU_ATMVP )
      {
        cu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[( cu.mergeIdx() << 1 ) + 0][0].mfRefIdx;
        cu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[( cu.mergeIdx() << 1 ) + 1][0].mfRefIdx;
      }
      else
      {
        for( int i = 0; i < 2; ++i )
        {
          if( cu.slice->getNumRefIdx( RefPicList( i ) ) > 0 )
          {
            MvField* mvField = affineMergeCtx.mvFieldNeighbours[( cu.mergeIdx() << 1 ) + i];

            PU::setAllAffineMvField( cu, mvField, RefPicList( i ) );
          }
        }
      }

      PU::spanMotionInfo( cu );
    }
    else
    {
      if( CU::isIBC( cu ) )
        PU::getIBCMergeCandidates  ( cu, mrgCtx, hist, cu.mergeIdx() );
      else
        PU::getInterMergeCandidates( cu, mrgCtx, hist, cu.mergeIdx() );

      mrgCtx.setMergeInfo         ( cu, cu.mergeIdx() );

      PU::spanMotionInfo          ( cu );
    }
  }
  else
  {
    if( cu.imv() && !cu.affineFlag() )
    {
      PU::applyImv      ( cu, hist );
      PU::spanMotionInfo( cu );
    }
    else
    {
      if( cu.affineFlag() )
      {
        for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
        {
          RefPicList eRefList = RefPicList( uiRefListIdx );
          if( cu.slice->getNumRefIdx( eRefList ) > 0 && ( cu.interDir() & ( 1 << uiRefListIdx ) ) )
          {
            AffineAMVPInfo affineAMVPInfo;
            PU::fillAffineMvpCand( cu, eRefList, cu.refIdx[eRefList], affineAMVPInfo );

            const unsigned mvp_idx = cu.mvpIdx[eRefList];

            //    Mv mv[3];
            CHECK( cu.refIdx[eRefList] < 0, "Unexpected negative refIdx." );
            const int imvShift = cu.imv() == 2 ? MV_FRACTIONAL_BITS_DIFF : 0;
            Mv mv0 = cu.mv[eRefList][0];
            Mv mv1 = cu.mv[eRefList][1];

            mv0 <<= imvShift;
            mv1 <<= imvShift;

            Mv mvLT = affineAMVPInfo.mvCandLT[mvp_idx] + mv0;
            Mv mvRT = affineAMVPInfo.mvCandRT[mvp_idx] + mv1;
            mvRT   += mv0;
            if( cu.imv() != 1 )
            {
              mvLT.changePrecision( MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL );
              mvRT.changePrecision( MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL );
            }

            Mv mvLB;
            if( cu.affineType() == AFFINEMODEL_6PARAM )
            {
              Mv mv2 = cu.mv[eRefList][2];
              mv2 <<= imvShift;
              mvLB = affineAMVPInfo.mvCandLB[mvp_idx] + mv2;
              mvLB += mv0;
              if ( cu.imv() != 1 )
              {
                mvLB.changePrecision( MV_PRECISION_QUARTER, MV_PRECISION_INTERNAL );
              }
            }
            PU::setAllAffineMv( cu, mvLT, mvRT, mvLB, eRefList, true );
          }
        }
      }
      else if( CU::isIBC( cu ) && cu.interDir() == 1 )
      {
        AMVPInfo amvpInfo;
        PU::fillIBCMvpCand( cu, amvpInfo, hist );
        Mv mvd = cu.mv[REF_PIC_LIST_0][0];
        mvd <<= 4;
        if( cu.slice->getPicHeader()->getMaxNumIBCMergeCand() == 1 )
        {
          CHECK( cu.mvpIdx[REF_PIC_LIST_0], "mvpIdx for IBC mode should be 0" );
        }
        cu.mv[REF_PIC_LIST_0][0] = amvpInfo.mvCand[cu.mvpIdx[REF_PIC_LIST_0]] + mvd;
        cu.mv[REF_PIC_LIST_0][0] . mvCliptoStorageBitDepth();
      }
      else
      {
        for( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
        {
          RefPicList eRefList = RefPicList( uiRefListIdx );
          if( ( cu.slice->getNumRefIdx( eRefList ) > 0 || ( eRefList == REF_PIC_LIST_0 && CU::isIBC( cu ) ) ) && ( cu.interDir() & ( 1 << uiRefListIdx ) ) )
          {
            AMVPInfo amvpInfo;
            PU::fillMvpCand( cu, eRefList, cu.refIdx[eRefList], amvpInfo, hist );

            Mv mvd              = cu.mv[eRefList][0];
            mvd                 . changePrecisionAmvr( cu.imv(), MV_PRECISION_INTERNAL);
            cu.mv [eRefList][0] = amvpInfo.mvCand[cu.mvpIdx[eRefList]] + mvd;
            cu.mv [eRefList][0] . mvCliptoStorageBitDepth();
          }
        }
      }
      PU::spanMotionInfo( cu );
    }
  }

  bool isIbcSmallBlk = CU::isIBC( cu ) && ( cu.lwidth() * cu.lheight() <= 16 );
  if( !cu.affineFlag() && !cu.geoFlag() && !isIbcSmallBlk )
  {
    const unsigned log2ParallelMergeLevel = (cu.sps->getLog2ParallelMergeLevelMinus2() + 2);
    const unsigned xBr = cu.Y().width  + cu.Y().x;
    const unsigned yBr = cu.Y().height + cu.Y().y;
    bool enableHmvp      = ((xBr >> log2ParallelMergeLevel) > (cu.Y().x >> log2ParallelMergeLevel)) && ((yBr >> log2ParallelMergeLevel) > (cu.Y().y >> log2ParallelMergeLevel));
    bool isIbc           = CU::isIBC( cu );
    bool enableInsertion = isIbc || enableHmvp;

    if( enableInsertion )
    {
      HPMVInfo mi( cu.getMotionInfo(), cu.interDir() == 3 ? cu.BcwIdx() : BCW_DEFAULT, cu.imv() == IMV_HPEL );
      if( isIbc ) mi.mhRefIdx[0] = MH_NOT_VALID + 1;
      MotionHist::addMiToLut( isIbc ? hist.motionLutIbc : hist.motionLut, mi );
    }
  }
}

}
