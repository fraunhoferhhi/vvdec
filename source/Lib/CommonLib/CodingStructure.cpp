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

/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#include "CodingStructure.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"


ThreadSafeCUCache g_globalUnitCache{};

const UnitScale UnitScaleArray[NUM_CHROMA_FORMAT][MAX_NUM_COMPONENT] =
{
  { {2,2}, {0,0}, {0,0} },  // 4:0:0
  { {2,2}, {1,1}, {1,1} },  // 4:2:0
  { {2,2}, {1,2}, {1,2} },  // 4:2:2
  { {2,2}, {2,2}, {2,2} }   // 4:4:4
};

static const int UNIT_PAD_SIZE = 128;

// ---------------------------------------------------------------------------
// coding structure method definitions
// ---------------------------------------------------------------------------

CodingStructure::CodingStructure(std::shared_ptr<CUCache> cuCache, std::shared_ptr<TUCache> tuCache )
  : area      ()
  , picture   ( nullptr )
  , m_cuCache ( cuCache )
  , m_tuCache ( tuCache )
 , m_IBCBufferWidth( 0 )
{
  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    m_cuIdxOrigin[ i ] = nullptr;
    m_cuIdx      [ i ] = nullptr;
  }

  for( int j = 0; j < NUM_EDGE_DIR; j++ )
  {
    m_lfParam[ j ] = nullptr;
  }

  m_motionBuf     = nullptr;
  m_dmvrMvCacheOffset = 0;
}

void CodingStructure::destroy()
{
  picture   = nullptr;

  m_reco.destroy();
  m_rec_wrap.destroy();
  m_pred.destroy();

  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    xFree( m_cuIdxOrigin[ i ] );
    m_cuIdxOrigin[ i ] = nullptr;
    m_cuIdx      [ i ] = nullptr;
  }

  for( int j = 0; j < NUM_EDGE_DIR; j++ )
  {
    xFree( m_lfParam[j] );
    m_lfParam[j] = nullptr;
  }

  xFree( m_motionBuf );
  m_motionBuf = nullptr;

  m_dmvrMvCache.clear();

  m_cuCache->defragment();
  m_tuCache->defragment();
  
  m_virtualIBCbuf.clear();
}

CodingUnit& CodingStructure::addCU( const UnitArea &unit, const ChannelType chType, const TreeType treeType, const ModeType modeType, const CodingUnit *cuLeft, const CodingUnit *cuAbove )
{
  CodingUnit *cu = m_cuCache->get();

  GCC_WARNING_DISABLE_class_memaccess
  memset( cu, 0, sizeof( CodingUnit ) );
  GCC_WARNING_RESET

  cu->minInit    ( unit );
  cu->cs         = this;
  cu->setChType  ( chType );
  cu->setTreeType( treeType );
  cu->setModeType( modeType );

  CodingUnit *prevCU = m_lastCU;

  if( prevCU )
  {
    prevCU->next = cu;
  }

  cu->idx = ++m_numCUs;

  uint32_t numCh = ::getNumberValidChannels( area.chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !cu->blocks[i].valid() )
    {
      continue;
    }

    const int cuArea = cu->blocks[i].area();

    if( i )
    {
      cu->predBuf[1]  = m_predBuf[1];
      m_predBuf  [1] += cuArea;
      cu->predBuf[2]  = m_predBuf[2];
      m_predBuf  [2] += cuArea;
    }
    else
    {
      cu->predBuf[0]  = m_predBuf[0];
      m_predBuf  [0] += cuArea;
    }

    const ptrdiff_t  stride = m_mapOrigSize[i].width;
    const Area      &_blk   = cu-> blocks[i];
    const UnitScale  scale  = unitScale[i];

    g_pelBufOP.fillN_CU( m_cuIdx[i] + stride * scale.scaleVer( _blk.y ) + scale.scaleHor( _blk.x ), stride, scale.scaleHor( _blk.width ), scale.scaleVer( _blk.height ), cu );

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
    CHECKD( m_dmvrMvCacheOffset >= m_dmvrMvCache.size(), "dmvr cache offset out of bounds" )
    pu.mvdL0SubPu       = &m_dmvrMvCache[m_dmvrMvCacheOffset];
    m_dmvrMvCacheOffset += std::max<int>( 1, unit.lwidth() >> DMVR_SUBCU_WIDTH_LOG2 ) * std::max<int>( 1, unit.lheight() >> DMVR_SUBCU_HEIGHT_LOG2 );
  }

  m_lastCU = cu;

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
    tu = m_tuCache->get();

    GCC_WARNING_DISABLE_class_memaccess
    memset( tu, 0, sizeof( TransformUnit ) );
    GCC_WARNING_RESET

    cu.lastTU->next = tu;
    cu.lastTU       = tu;
  }

  tu->cu                =  &cu;
  tu->chType            =  chType;
  tu->UnitArea::operator=( unit );

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

  if( lastCU ) lastCU = lastCU->next;

  return CUTraverser( firstCU, lastCU );
}

cCUTraverser CodingStructure::traverseCUs( const UnitArea& unit ) const
{
  ChannelType lastChan      = unit.chromaFormat != CHROMA_400 ? CH_C : CH_L;
  const CodingUnit* firstCU = getCU( unit.lumaPos(), CH_L );
  const CodingUnit* lastCU  = getCU( unit.block( getFirstComponentOfChannel( lastChan ) ).bottomRight(), lastChan );

  if( lastCU ) lastCU = lastCU->next;

  return cCUTraverser( firstCU, lastCU );
}

// coding utilities

void CodingStructure::create(const ChromaFormat &_chromaFormat, const Area& _area, const bool isTopLayer)
{
  createInternals( UnitArea( _chromaFormat, _area ), isTopLayer );
}

void CodingStructure::create(const UnitArea& _unit, const bool isTopLayer)
{
  createInternals( _unit, isTopLayer );
}

void CodingStructure::createInternals( const UnitArea& _unit, const bool isTopLayer )
{
  area = _unit;

  memcpy( unitScale, UnitScaleArray[area.chromaFormat], sizeof( unitScale ) );

  picture = nullptr;

  const unsigned numCh = ::getNumberValidChannels(area.chromaFormat);

  for( unsigned i = 0; i < numCh; i++ )
  {
    Size allocArea = area.blocks[i].size();
    m_mapSize[i]   = unitScale[i].scale( allocArea );

    const int pad_size = isTopLayer ? UNIT_PAD_SIZE : 0;

    allocArea.width  += 2 * pad_size;
    allocArea.height += 2 * pad_size;

    unsigned _area = unitScale[i].scale( allocArea ).area();

    m_cuIdx[i] = _area > 0 ? ( CodingUnit**    ) xMalloc( CodingUnit*   , _area ) : nullptr;

    m_cuIdxOrigin[i] = m_cuIdx[i];
    m_mapOrigSize[i] = unitScale[i].scale( allocArea );
    m_mapOrigSizeUnscaled[i] = allocArea;

    m_cuIdx[i] = m_cuIdxOrigin[i] + rsAddr( unitScale[i].scale( Position{ pad_size, pad_size } ), m_mapOrigSize[i].width );

    AreaBuf<CodingUnit*>( m_cuIdxOrigin[i], m_mapOrigSize[i].width, m_mapOrigSize[i] ).memset( 0 );
  }

  for( unsigned j = 0; j < NUM_EDGE_DIR; j++ )
  {
    m_lfParam[j] = m_mapSize[CH_L].area() > 0 ? ( LoopFilterParam* ) xMalloc( LoopFilterParam, m_mapSize[CH_L].area() ) : nullptr;
  }

  unsigned _lumaAreaScaled = g_miScaling.scale( area.lumaSize() ).area();
  m_motionBuf = ( MotionInfo* ) xMalloc( MotionInfo, _lumaAreaScaled );

  // for the worst case of all PUs being 8x8 and using DMVR
  unsigned _maxNumDmvrMvs = ( area.lwidth() >> 3 ) * ( area.lheight() >> 3 );
  m_dmvrMvCache.resize( _maxNumDmvrMvs );
}


void CodingStructure::rebindPicBufs()
{
  if( !picture->m_bufs[PIC_RECONSTRUCTION].bufs.empty() ) m_reco.createFromBuf( picture->m_bufs[PIC_RECONSTRUCTION] );
  else                                                    m_reco.destroy();
  if( !picture->m_bufs[PIC_RECON_WRAP    ].bufs.empty() ) m_rec_wrap.createFromBuf( picture->m_bufs[PIC_RECON_WRAP] );
  else                                                    m_rec_wrap.destroy();
  if( !picture->m_bufs[PIC_PREDICTION    ].bufs.empty() ) m_pred.createFromBuf( picture->m_bufs[PIC_PREDICTION] );
  else                                                    m_pred.destroy();
}

void CodingStructure::initStructData()
{
  m_numCUs = 0;
  m_lastCU = nullptr;

  m_cuCache->defragment();
  m_tuCache->defragment();

  int numCh = ::getNumberValidChannels( area.chromaFormat );

  for( int i = 0; i < numCh; i++ )
  {
    AreaBuf<CodingUnit*>( m_cuIdx[i], m_mapOrigSize[i].width, m_mapSize[i] ).memset( 0 );
  }

  MotionBuf   mb    = getMotionBuf();
  MotionInfo* miPtr = mb.buf;

  CHECKD( AMVP_DECIMATION_FACTOR != 2, "AMVP decimation factor does not match!" );

  for( int y = 0; y < mb.height; y += 2 )
  {
    MotionInfo* miLinePtr = miPtr;

    for( int x = 0; x < mb.width; x += 2, miLinePtr += 2 )
    {
      miLinePtr->isInter = false;
    }

    OFFSETY( miPtr, mb.stride, 2 );
  }

  memset( m_lfParam[EDGE_HOR], 0, m_mapSize[CH_L].area() * sizeof( LoopFilterParam ) );
  memset( m_lfParam[EDGE_VER], 0, m_mapSize[CH_L].area() * sizeof( LoopFilterParam ) );

  m_dmvrMvCacheOffset = 0;

  m_predBuf[0] = m_pred.bufs[0].buf;
  m_predBuf[1] = m_pred.bufs[1].buf;
  m_predBuf[2] = m_pred.bufs[2].buf;
}

MotionBuf CodingStructure::getMotionBuf( const Area& _area )
{
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const int  stride   = g_miScaling.scaleHor( _luma.width );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), stride ), stride, miArea.size() );
}

const CMotionBuf CodingStructure::getMotionBuf( const Area& _area ) const
{
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const int  stride   = g_miScaling.scaleHor( _luma.width );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), stride ), stride, miArea.size() );
}

MotionInfo& CodingStructure::getMotionInfo( const Position& pos )
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos  = g_miScaling.scale(pos);

  return *( m_motionBuf + miPos.y * stride + miPos.x );
}

const MotionInfo& CodingStructure::getMotionInfo( const Position& pos ) const
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  //return getMotionBuf().at( g_miScaling.scale( pos - area.lumaPos() ) );
  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor(area.lumaSize().width);
  const Position miPos  = g_miScaling.scale(pos);

  return *( m_motionBuf + miPos.y * stride + miPos.x );
}

LFPBuf CodingStructure::getLoopFilterParamBuf(const DeblockEdgeDir& edgeDir)
{
  return LFPBuf( m_lfParam[edgeDir], m_mapSize[CH_L] );
}

const CLFPBuf CodingStructure::getLoopFilterParamBuf(const DeblockEdgeDir& edgeDir ) const
{
  return CLFPBuf( m_lfParam[edgeDir], m_mapSize[CH_L] );
}

PelUnitBuf CodingStructure::getPredBuf(const CodingUnit &unit)          
{
  PelUnitBuf ret;
  ret.chromaFormat = unit.chromaFormat;
  ret.bufs.resize_noinit( 3 );

  if( unit.Y().valid() )
  {
    ret.bufs[0].buf    = unit.predBuf[0];
    ret.bufs[0].stride = unit.blocks [0].width;
    ret.bufs[0].width  = unit.blocks [0].width;
    ret.bufs[0].height = unit.blocks [0].height;
  }

  if( unit.Cb().valid() )
  {
    ret.bufs[1].buf    = unit.predBuf[1];
    ret.bufs[1].stride = unit.blocks [1].width;
    ret.bufs[1].width  = unit.blocks [1].width;
    ret.bufs[1].height = unit.blocks [1].height;
  }

  if( unit.Cr().valid() )
  {
    ret.bufs[2].buf    = unit.predBuf[2];
    ret.bufs[2].stride = unit.blocks [2].width;
    ret.bufs[2].width  = unit.blocks [2].width;
    ret.bufs[2].height = unit.blocks [2].height;
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
    ret.bufs[0].buf    = unit.predBuf[0];
    ret.bufs[0].stride = unit.blocks[0].width;
    ret.bufs[0].width  = unit.blocks[0].width;
    ret.bufs[0].height = unit.blocks[0].height;
  }

  if( unit.Cb().valid() )
  {
    ret.bufs[1].buf    = unit.predBuf[1];
    ret.bufs[1].stride = unit.blocks[1].width;
    ret.bufs[1].width  = unit.blocks[1].width;
    ret.bufs[1].height = unit.blocks[1].height;
  }

  if( unit.Cr().valid() )
  {
    ret.bufs[2].buf    = unit.predBuf[2];
    ret.bufs[2].stride = unit.blocks[2].width;
    ret.bufs[2].width  = unit.blocks[2].width;
    ret.bufs[2].height = unit.blocks[2].height;
  }

  return ret;
}

const CodingUnit* CodingStructure::getCURestricted( const Position &pos, const CodingUnit& curCu, const ChannelType _chType, const CodingUnit* guess ) const
{
  if( guess && guess->blocks[_chType].contains( pos ) ) return guess;

  const int yshift     = pcv->maxCUWidthLog2 - getChannelTypeScaleY( _chType, curCu.chromaFormat );
  const int ydiff      = ( pos.y >> yshift ) - ( curCu.blocks[_chType].y >> yshift ); // ( a <= b ) ==> a - b <= 0
  const CodingUnit* cu = ydiff <= 0 ? getCU( pos, _chType ) : nullptr;
  const int xshift     = pcv->maxCUWidthLog2 - getChannelTypeScaleX( _chType, curCu.chromaFormat );
  const int xdiff      = ( pos.x >> xshift ) - ( curCu.blocks[_chType].x >> xshift );
  const bool sameCTU   = !ydiff && !xdiff;

  if( cu && cu->idx <= curCu.idx && ( sameCTU || ( cu->slice->getIndependentSliceIdx() == curCu.slice->getIndependentSliceIdx() && cu->tileIdx == curCu.tileIdx ) ) )
  {
    if( xdiff > 0 && sps->getEntropyCodingSyncEnabledFlag() )
    {
      return nullptr;
    }

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
  const CodingUnit* cu = ydiff <= 0 ? getCU( pos, _chType ) : nullptr;
  const int xshift     = pcv->maxCUWidthLog2 - getChannelTypeScaleX( _chType, area.chromaFormat );
  const int xdiff      = ( pos.x >> xshift ) - ( curPos.x >> xshift );
  const bool sameCTU   = !ydiff && !xdiff;

  if( cu && ( sameCTU || ( cu->slice->getIndependentSliceIdx() == curSliceIdx && cu->tileIdx == curTileIdx ) ) )
  {
    if( xdiff > 0 && sps->getEntropyCodingSyncEnabledFlag() )
    {
      return nullptr;
    }

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

      const unsigned int lcuWidth = slice->getSPS()->getMaxCUWidth();
      const int shiftSample = ::getComponentScaleX(area.compID, cu.chromaFormat);
      const int ctuSizeLog2 = getLog2(lcuWidth) - shiftSample;
      const int pux = area.x & ((m_IBCBufferWidth >> shiftSample) - 1);
      const int puy = area.y & (( 1 << ctuSizeLog2 ) - 1);
      const CompArea dstArea = CompArea(area.compID, Position(pux, puy), Size(area.width, area.height));
      CPelBuf srcBuf = getRecoBuf(area);
      PelBuf dstBuf = m_virtualIBCbuf[lineIdx].getBuf(dstArea);

      dstBuf.copyFrom(srcBuf);
    }
  }
}

