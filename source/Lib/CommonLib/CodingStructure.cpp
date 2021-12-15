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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#include "CodingStructure.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"

namespace vvdec
{

const UnitScale UnitScaleArray[NUM_CHROMA_FORMAT][MAX_NUM_COMPONENT] =
{
  { {2,2}, {0,0}, {0,0} },  // 4:0:0
  { {2,2}, {1,1}, {1,1} },  // 4:2:0
  { {2,2}, {1,2}, {1,2} },  // 4:2:2
  { {2,2}, {2,2}, {2,2} }   // 4:4:4
};

// ---------------------------------------------------------------------------
// coding structure method definitions
// ---------------------------------------------------------------------------

CodingStructure::CodingStructure( CUChunkCache* cuChunkCache, TUChunkCache* tuChunkCache )
  : area      ()
  , picture   ( nullptr )
  , m_ctuData ( nullptr )
  , m_ctuDataSize( 0 )
  , m_dmvrMvCache ( nullptr )
  , m_cuCache ( cuChunkCache )
  , m_tuCache ( tuChunkCache )
  , m_IBCBufferWidth( 0 )
{
  m_dmvrMvCacheOffset = 0;
}

void CodingStructure::destroy()
{
  picture   = nullptr;

  m_reco.destroy();
  m_rec_wrap.destroy();

  m_cuCache.releaseAll();
  m_tuCache.releaseAll();
  
  m_virtualIBCbuf.clear();

  deallocTempInternals();

  if( m_ctuData )
  {
    free( m_ctuData );
    m_ctuData = nullptr;
    m_ctuDataSize = 0;
  }
}

CodingUnit& CodingStructure::addCU( const UnitArea &unit, const ChannelType chType, const TreeType treeType, const ModeType modeType, const CodingUnit *cuLeft, const CodingUnit *cuAbove )
{
  CodingUnit *cu = m_cuCache.get();

  GCC_WARNING_DISABLE_class_memaccess
  memset( cu, 0, sizeof( CodingUnit ) );
  GCC_WARNING_RESET

  cu->minInit    ( unit );
  cu->cs         = this;
  cu->setChType  ( chType );
  cu->setTreeType( treeType );
  cu->setModeType( modeType );
  
  CodingUnit *prevCU = cu;
  std::swap( m_lastCU, prevCU );

  const int currRsAddr = ctuRsAddr( unit.blocks[chType].pos(), chType );

  if( prevCU ) prevCU->next = cu;

  cu->idx = ++m_numCUs;

  uint32_t numCh = getNumberValidChannels( area.chromaFormat );

  CtuData& ctuData = getCtuData( currRsAddr );
  cu->ctuData = &ctuData;

  cu->predBufOff = m_predBufOffset;

  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !cu->blocks[i].valid() )
    {
      continue;
    }

    const int cuArea = cu->blocks[i].area();

    if( i )
    {
      m_predBufOffset += ( cuArea << 1 );
    }
    else
    {
      m_predBufOffset += cuArea;
    }

    const ptrdiff_t  stride = ptrdiff_t( 1 ) << m_ctuWidthLog2[i];
    const Area&      _blk   = cu->blocks[i];
    const UnitScale  scale  = unitScale[i];
    const int        sclX   = scale.scaleHor( _blk.x );
    const int        sclY   = scale.scaleVer( _blk.y );
    const int        sclW   = scale.scaleHor( _blk.width );
    const int        sclH   = scale.scaleVer( _blk.height );

    g_pelBufOP.fillN_CU( ctuData.cuPtr[i] + ( sclX & m_ctuSizeMask[i] ) + ( ( sclY & m_ctuSizeMask[i] ) << m_ctuWidthLog2[i] ), stride, sclW, sclH, cu );

    if( i == chType )
    {
      cu->left   = cuLeft;
      cu->above  = cuAbove;
    }
  }

  PredictionUnit& pu = *cu;
  pu.setChType( chType );

  if( isLuma( chType ) && unit.lheight() >= 8 && unit.lwidth()  >= 8 && unit.Y().area() >= 128 )
  {
    pu.mvdL0SubPuOff     = m_dmvrMvCacheOffset;
    m_dmvrMvCacheOffset += std::max<int>( 1, unit.lwidth() >> DMVR_SUBCU_WIDTH_LOG2 ) * std::max<int>( 1, unit.lheight() >> DMVR_SUBCU_HEIGHT_LOG2 );
  }

  return *cu;
}

TransformUnit& CodingStructure::addTU( const UnitArea &unit, const ChannelType chType, CodingUnit& cu )
{
  TransformUnit* tu;

  if( cu.firstTU.blocks.empty() )
  {
    tu = cu.lastTU = &cu.firstTU;
  }
  else
  {
    tu = m_tuCache.get();

    GCC_WARNING_DISABLE_class_memaccess
    memset( tu, 0, sizeof( TransformUnit ) );
    GCC_WARNING_RESET

    cu.lastTU->next = tu;
    cu.lastTU       = tu;
  }

  tu->idx               = ++m_numTUs;
  tu->cu                =  &cu;
  tu->setChType         (   chType );
  tu->UnitArea::operator=(  unit );

  return *tu;
}

void CodingStructure::addEmptyTUs( Partitioner &partitioner, CodingUnit& cu )
{
  const bool split = partitioner.canSplit( TU_MAX_TR_SPLIT, *this );

  if( split )
  {
    partitioner.splitCurrArea( TU_MAX_TR_SPLIT, *this );

    do
    {
      addTU( partitioner.currArea(), partitioner.chType, cu );
    } while( partitioner.nextPart( *this ) );

    partitioner.exitCurrSplit( *this );
  }
  else
  {
    addTU( partitioner.currArea(), partitioner.chType, cu );
  }
}

CUTraverser CodingStructure::traverseCUs( const UnitArea& unit )
{
  ChannelType lastChan  = unit.chromaFormat != CHROMA_400 ? CH_C : CH_L;
  CodingUnit* firstCU   = getCU( unit.lumaPos(), CH_L );
  CodingUnit* lastCU    = getCU( unit.block( getFirstComponentOfChannel( lastChan ) ).bottomRight(), lastChan );

  CHECKD( !firstCU || !lastCU || ctuRsAddr( firstCU->lumaPos(), CH_L ) != ctuRsAddr( lastCU->blocks[lastChan].pos(), lastChan ), "First CU and/or Last CU non-existent not in the same CTU!" );

  if( lastCU ) lastCU = lastCU->next;

  return CUTraverser( firstCU, lastCU );
}

cCUTraverser CodingStructure::traverseCUs( const UnitArea& unit ) const
{
  ChannelType lastChan      = unit.chromaFormat != CHROMA_400 ? CH_C : CH_L;
  const CodingUnit* firstCU = getCU( unit.lumaPos(), CH_L );
  const CodingUnit* lastCU  = getCU( unit.block( getFirstComponentOfChannel( lastChan ) ).bottomRight(), lastChan );

  CHECKD( !firstCU || !lastCU || ctuRsAddr( firstCU->lumaPos(), CH_L ) != ctuRsAddr( lastCU->blocks[lastChan].pos(), lastChan ), "First CU and/or Last CU non-existent not in the same CTU!" );

  if( lastCU ) lastCU = lastCU->next;

  return cCUTraverser( firstCU, lastCU );
}

// coding utilities

void CodingStructure::create(const ChromaFormat &_chromaFormat, const Area& _area)
{
  createInternals( UnitArea( _chromaFormat, _area ) );
}

void CodingStructure::create(const UnitArea& _unit)
{
  createInternals( _unit );
}

void CodingStructure::createInternals( const UnitArea& _unit )
{
  area = _unit;

  memcpy( unitScale, UnitScaleArray[area.chromaFormat], sizeof( unitScale ) );

  picture = nullptr;
}


void CodingStructure::rebindPicBufs()
{
  if( !picture->m_bufs[PIC_RECONSTRUCTION].bufs.empty() ) m_reco.createFromBuf( picture->m_bufs[PIC_RECONSTRUCTION] );
  else                                                    m_reco.destroy();
  if( !picture->m_bufs[PIC_RECON_WRAP    ].bufs.empty() ) m_rec_wrap.createFromBuf( picture->m_bufs[PIC_RECON_WRAP] );
  else                                                    m_rec_wrap.destroy();
}

void CodingStructure::allocTempInternals()
{
  if( m_ctuDataSize != pcv->sizeInCtus )
  {
    m_ctuDataSize = pcv->sizeInCtus;
    if( m_ctuData ) free( m_ctuData );
    m_ctuData = ( CtuData* ) malloc( m_ctuDataSize * sizeof( CtuData ) );
  }
}

void CodingStructure::deallocTempInternals()
{
  m_numCUs = 0;
  m_numTUs = 0;
  m_lastCU = nullptr;

  m_cuCache.releaseAll();
  m_tuCache.releaseAll();
}

void CodingStructure::initStructData()
{
  m_numCUs = 0;
  m_numTUs = 0;
  m_lastCU = nullptr;

  m_cuCache.releaseAll();
  m_tuCache.releaseAll();

  m_widthInCtus = pcv->widthInCtus;

  m_ctuSizeMask[0] = pcv->maxCUWidthMask >> unitScale[CH_L].posx;
  m_ctuSizeMask[1] = pcv->maxCUWidthMask >> ( getChannelTypeScaleX( CH_C, area.chromaFormat ) + unitScale[CH_C].posx );

  m_ctuWidthLog2[0] = pcv->maxCUWidthLog2 - unitScale[CH_L].posx;
  m_ctuWidthLog2[1] = m_ctuWidthLog2[0]; // same for luma and chroma, because of the 2x2 blocks

  m_dmvrMvCacheOffset = 0;

  m_predBufOffset = 0;

  GCC_WARNING_DISABLE_class_memaccess
  memset( m_ctuData, 0, sizeof( CtuData ) * m_ctuDataSize );
  GCC_WARNING_RESET
}

MotionBuf CodingStructure::getMotionBuf( const Area& _area )
{
  CtuData&        ctuData = getCtuData( ctuRsAddr( _area.pos(), CH_L ) );

  const ptrdiff_t  stride = ptrdiff_t( 1 ) << m_ctuWidthLog2[CH_L];
  const UnitScale  scale  = g_miScaling;

  return MotionBuf( ctuData.motion + inCtuPos( _area, CH_L ), stride, scale.scaleHor( _area.width ), scale.scaleVer( _area.height ) );
}

const CMotionBuf CodingStructure::getMotionBuf( const Area& _area ) const
{
  const CtuData& ctuData  = getCtuData( ctuRsAddr( _area.pos(), CH_L ) );

  const ptrdiff_t  stride = ptrdiff_t( 1 ) << m_ctuWidthLog2[CH_L];
  const UnitScale  scale  = g_miScaling;

  return CMotionBuf( ctuData.motion + inCtuPos( _area, CH_L ), stride, scale.scaleHor( _area.width ), scale.scaleVer( _area.height ) );
}

PelUnitBuf CodingStructure::getPredBuf(const CodingUnit &unit)          
{
  PelUnitBuf ret;
  ret.chromaFormat = unit.chromaFormat;
  ret.bufs.resize_noinit( getNumberValidComponents( unit.chromaFormat ) );

  if( unit.Y().valid() )
  {
    ret.bufs[0].buf    = m_predBuf + unit.predBufOff;
    ret.bufs[0].stride = unit.blocks[0].width;
    ret.bufs[0].width  = unit.blocks[0].width;
    ret.bufs[0].height = unit.blocks[0].height;
  }

  if( isChromaEnabled( unit.chromaFormat ) )
  {
    if( unit.Cb().valid() )
    {
      ret.bufs[1].buf    = m_predBuf + unit.predBufOff + unit.Y().area();
      ret.bufs[1].stride = unit.blocks[1].width;
      ret.bufs[1].width  = unit.blocks[1].width;
      ret.bufs[1].height = unit.blocks[1].height;
    }

    if( unit.Cr().valid() )
    {
      ret.bufs[2].buf    = m_predBuf + unit.predBufOff + unit.Y().area() + unit.Cb().area();
      ret.bufs[2].stride = unit.blocks[2].width;
      ret.bufs[2].width  = unit.blocks[2].width;
      ret.bufs[2].height = unit.blocks[2].height;
    }
  }

  return ret;
}

const CPelUnitBuf CodingStructure::getPredBuf(const CodingUnit &unit) const
{
  CPelUnitBuf ret;
  ret.chromaFormat = unit.chromaFormat;
  ret.bufs.resize( 3 );

  if( unit.Y().valid() )
  {
    ret.bufs[0].buf    = m_predBuf + unit.predBufOff;
    ret.bufs[0].stride = unit.blocks[0].width;
    ret.bufs[0].width  = unit.blocks[0].width;
    ret.bufs[0].height = unit.blocks[0].height;
  }

  if( unit.Cb().valid() )
  {
    ret.bufs[1].buf    = m_predBuf + unit.predBufOff + unit.Y().area();
    ret.bufs[1].stride = unit.blocks[1].width;
    ret.bufs[1].width  = unit.blocks[1].width;
    ret.bufs[1].height = unit.blocks[1].height;
  }

  if( unit.Cr().valid() )
  {
    ret.bufs[2].buf    = m_predBuf + unit.predBufOff + unit.Y().area() + unit.Cb().area();
    ret.bufs[2].stride = unit.blocks[2].width;
    ret.bufs[2].width  = unit.blocks[2].width;
    ret.bufs[2].height = unit.blocks[2].height;
  }

  return ret;
}

const ColocatedMotionInfo& CodingStructure::getColInfo( const Position &pos, const Slice*& pColSlice ) const
{
  const CtuData& ctuData    = getCtuData( ctuRsAddr( pos, CH_L ) );
  const ColocatedMotionInfo&
                 colMi      = ctuData.colMotion[colMotPos( pos )];
                 pColSlice  = ctuData.slice;
  
  return colMi;
}

const CodingUnit* CodingStructure::getCURestricted( const Position &pos, const CodingUnit& curCu, const ChannelType _chType, const CodingUnit* guess ) const
{
  if( guess && guess->blocks[_chType].contains( pos ) ) return guess;

  const int yshift     = pcv->maxCUWidthLog2 - getChannelTypeScaleY( _chType, curCu.chromaFormat );
  const int ydiff      = ( pos.y >> yshift ) - ( curCu.blocks[_chType].y >> yshift ); // ( a <= b ) ==> a - b <= 0
  const int xshift     = pcv->maxCUWidthLog2 - getChannelTypeScaleX( _chType, curCu.chromaFormat );
  const int xdiff      = ( pos.x >> xshift ) - ( curCu.blocks[_chType].x >> xshift );
  const bool sameCTU   = !ydiff && !xdiff;

  const CodingUnit* cu = nullptr;

  if( sameCTU )
  {
    cu = curCu.ctuData->cuPtr[_chType][inCtuPos( pos, _chType )];
  }
  else if( ydiff > 0 || xdiff > ( 1 - sps->getEntropyCodingSyncEnabledFlag() ) )
  {
    return nullptr;
  }
  else
  {
    cu = getCU( pos, _chType );
  }

  if( !cu || cu->idx > curCu.idx ) return nullptr;
  else if( sameCTU ) return cu;

  if( cu->slice->getIndependentSliceIdx() == curCu.slice->getIndependentSliceIdx() && cu->tileIdx == curCu.tileIdx )
  {
    return cu;
  }
  else
  {
    return nullptr;
  }
}

const CodingUnit* CodingStructure::getCURestricted( const Position &pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType ) const
{
  const int yshift     = pcv->maxCUWidthLog2 - getChannelTypeScaleY( _chType, area.chromaFormat );
  const int ydiff      = ( pos.y >> yshift ) - ( curPos.y >> yshift ); // ( a <= b ) ==> a - b <= 0
  const int xshift     = pcv->maxCUWidthLog2 - getChannelTypeScaleX( _chType, area.chromaFormat );
  const int xdiff      = ( pos.x >> xshift ) - ( curPos.x >> xshift );
  const bool sameCTU   = !ydiff && !xdiff;

  const CodingUnit* cu = nullptr;
  
  if( sameCTU )
  {
    return getCU( pos, _chType );
  }
  else if( ydiff > 0 || xdiff > ( 1 - sps->getEntropyCodingSyncEnabledFlag() ) )
  {
    return nullptr;
  }
  else
  {
    cu = getCU( pos, _chType );
  }

  if( cu && cu->slice->getIndependentSliceIdx() == curSliceIdx && cu->tileIdx == curTileIdx )
  {
    return cu;
  }
  else
  {
    return nullptr;
  }
}


void CodingStructure::initVIbcBuf( int numCtuLines, ChromaFormat chromaFormatIDC, int ctuSize )
{
  m_virtualIBCbuf.resize( numCtuLines );
  for( auto &buf: m_virtualIBCbuf )

  if (buf.bufs.empty())
  {
    m_IBCBufferWidth = g_IBCBufferSize / ctuSize;
    buf.create(UnitArea(chromaFormatIDC, Area(0, 0, m_IBCBufferWidth, ctuSize)));
  }

}

void CodingStructure::fillIBCbuffer( CodingUnit &cu, int lineIdx )
{
  for( TransformUnit &tu : TUTraverser( &cu.firstTU, cu.lastTU->next ) )
  {
    for( const CompArea &area : tu.blocks )
    {
      if (!area.valid())
        continue;

      const unsigned int lcuWidth = sps->getMaxCUWidth();
      const int shiftSampleHor = getComponentScaleX(area.compID(), cu.chromaFormat);
      const int shiftSampleVer = getComponentScaleY(area.compID(), cu.chromaFormat);
      const int ctuSizeVerLog2 = getLog2(lcuWidth) - shiftSampleVer;
      const int pux = area.x & ((m_IBCBufferWidth >> shiftSampleHor) - 1);
      const int puy = area.y & (( 1 << ctuSizeVerLog2 ) - 1);
      const CompArea dstArea = CompArea(area.compID(), Position(pux, puy), Size(area.width, area.height));
      CPelBuf srcBuf = getRecoBuf(area);
      PelBuf dstBuf = m_virtualIBCbuf[lineIdx].getBuf(dstArea);

      dstBuf.copyFrom(srcBuf);
    }
  }
}

}
