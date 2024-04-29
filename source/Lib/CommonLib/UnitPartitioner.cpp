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

/** \file     UnitPartitioner.h
 *  \brief    Provides a class for partitioning management
 */

#include "UnitPartitioner.h"

#include "CodingStructure.h"
#include "Unit.h"
#include "Slice.h"
#include "UnitTools.h"
#include "Picture.h"

namespace vvdec
{

PartLevel::PartLevel()
: split               ( CU_DONT_SPLIT )
, parts               (               )
, idx                 ( 0u            )
, cuAbove             ( nullptr       )
, cuLeft              ( nullptr       )
, modeType            ( MODE_TYPE_ALL )
, qgEnable            ( true          )
, qgChromaEnable      ( true          )
{
}

void PartLevel::init()
{
  split           = CU_DONT_SPLIT;
  idx             = 0u;
  cuAbove         = nullptr;
  cuLeft          = nullptr;
  modeType        = MODE_TYPE_ALL;
  qgEnable        = true;
  qgChromaEnable  = true;
}

//////////////////////////////////////////////////////////////////////////
// Partitioner class
//////////////////////////////////////////////////////////////////////////

SplitSeries Partitioner::getSplitSeries() const
{
  SplitSeries splitSeries = 0;
  SplitSeries depth = 0;

  for( const auto &level : m_partStack )
  {
    if( level.split == CTU_LEVEL ) continue;
    else splitSeries += static_cast< SplitSeries >( level.split ) << ( depth * SPLIT_DMULT );

    depth++;

    if( depth >= 3 ) break;
  }

  return splitSeries;
}

bool Partitioner::isSepTree( const CodingStructure &cs ) const
{
  return treeType != TREE_D || isDualITree;
}

void Partitioner::setCUData( CodingUnit& cu )
{
  cu.depth       = currDepth;
  cu.qtDepth     = currQtDepth;
  cu.splitSeries = getSplitSeries();
}

static void setNeighborCu( PartLevel& level, Partitioner& p, const CodingStructure& cs )
{
  const ChannelType chType = p.treeType == TREE_C ? CH_C : p.chType;
  const Position   &pos    = p.currArea().blocks[chType].pos();

  // get above depth
  level.cuAbove  = cs.getCURestricted( pos.offset( 0, -1 ), pos, p.currSliceIdx, p.currTileIdx, chType );

  // get left depth
  level.cuLeft   = cs.getCURestricted( pos.offset( -1, 0 ), pos, p.currSliceIdx, p.currTileIdx, chType );
}


void Partitioner::initCtu( const UnitArea& ctuArea, const ChannelType _chType, const CodingStructure& cs, const Slice& slice )
{
  this->slice = &slice;
#if _DEBUG
  m_currArea = ctuArea;
#endif
  currDepth   = 0;
  currTrDepth = 0;
  currMtDepth = 0;
  currQtDepth = 0;
  currSubdiv  = 0;
  chType      = _chType;
  currQgPos   = ctuArea.lumaPos();
  currQgChromaPos = ctuArea.chromaFormat != CHROMA_400 ? ctuArea.chromaPos() : Position();
  currImplicitBtDepth = 0;

  currSliceIdx = slice.getIndependentSliceIdx();
  currTileIdx  = cs.pps->getTileIdx( ctuArea.lumaPos() );

  m_partBufIdx = 1;
  m_partStack.resize_noinit( 1 );
  m_partStack.back().split = CTU_LEVEL;
  m_partStack.back().parts = m_partBuf;
  m_partStack.back().parts[0] = ctuArea;
  m_partStack.back().numParts = 1;
  treeType = TREE_D;
  modeType = MODE_TYPE_ALL;

  setNeighborCu( m_partStack.back(), *this, cs );

  const SPS& sps = *cs.sps;
#if GDR_ADJ
  isDualITree = slice.isIntra() && slice.getSPS()->getUseDualITree();
  const int valIdx = slice.isIntra() ? ( !isDualITree? 0 : ( _chType << 1 ) ) : 1;
#else
  isDualITree = slice.isIRAP() && slice.getSPS()->getUseDualITree();
  const int valIdx = slice.isIRAP() ? ( _chType << 1 ) : 1;
#endif

  const unsigned minBtSizeArr[] = { 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize() };
  const unsigned minTtSizeArr[] = { 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize(), 1u << sps.getLog2MinCodingBlockSize() };

  minBtSize = minBtSizeArr[valIdx];
  minTtSize = minTtSizeArr[valIdx];

  if( cs.picHeader->getSplitConsOverrideFlag() )
  {
    maxBTD    = slice.getPicHeader()->getMaxMTTHierarchyDepth( slice.getSliceType(), _chType );
    maxBtSize = slice.getPicHeader()->getMaxBTSize( slice.getSliceType(), _chType );
    maxTtSize = slice.getPicHeader()->getMaxTTSize( slice.getSliceType(), _chType );
    minQtSize = slice.getPicHeader()->getMinQTSize( slice.getSliceType(), _chType );
  }
  else
  {
    maxBTD    = sps.getMaxMTTHierarchyDepths()[valIdx];
    maxBtSize = sps.getMaxBTSizes()[valIdx];
    maxTtSize = sps.getMaxTTSizes()[valIdx];
    minQtSize = sps.getMinQTSizes()[valIdx];
  }

  maxTrSize = cs.sps->getMaxTbSize();
}

void Partitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  const bool isImplicit = !cs.picture->Y().contains( currArea().Y().bottomRight() );
  const UnitArea& area  = currArea();
  bool qgEnable         = currQgEnable();
  bool qgChromaEnable   = currQgChromaEnable();

  PartLevel &last = m_partStack.back();
  m_partStack.resize_noinit( m_partStack.size() + 1 );
  PartLevel& back = m_partStack.back();
  back.init();
  back.parts = &m_partBuf[m_partBufIdx];
  int numParts;

  if     ( split <= CU_TRIV_SPLIT )
    numParts = PartitionerImpl::getCUSubPartitions     ( area, cs, split, back.parts );
  else if( split == TU_MAX_TR_SPLIT )
    numParts = PartitionerImpl::getMaxTuTiling         ( area, cs,        back.parts );
  else if( split >= SBT_VER_HALF_POS0_SPLIT && split <= SBT_HOR_QUAD_POS1_SPLIT )
    numParts = PartitionerImpl::getSbtTuTiling         ( area, cs, split, back.parts );
  else /* if( split == TU_1D_HORZ_SPLIT || split == TU_1D_VERT_SPLIT ) */
    numParts = PartitionerImpl::getTUIntraSubPartitions( area, cs, isDualITree, split, back.parts, treeType );

  back.numParts = numParts;
  m_partBufIdx += numParts;

  CHECK( m_partBufIdx > partBufSize, "Partition buffer overflow" );

  switch( split )
  {
  case CU_QUAD_SPLIT:
    currTrDepth = 0;
    CHECKD( currMtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    currMtDepth = 0;
    currQtDepth++;
    currSubdiv ++;
    break;
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
    currTrDepth = 0;
    if( isImplicit ) currImplicitBtDepth++;
    currMtDepth++;
    break;
  case CU_TRIH_SPLIT:
  case CU_TRIV_SPLIT:
    currTrDepth = 0;
    currMtDepth++;
    currSubdiv ++;
    break;
  case TU_MAX_TR_SPLIT:
  case SBT_VER_HALF_POS0_SPLIT:
  case SBT_VER_HALF_POS1_SPLIT:
  case SBT_HOR_HALF_POS0_SPLIT:
  case SBT_HOR_HALF_POS1_SPLIT:
  case SBT_VER_QUAD_POS0_SPLIT:
  case SBT_VER_QUAD_POS1_SPLIT:
  case SBT_HOR_QUAD_POS0_SPLIT:
  case SBT_HOR_QUAD_POS1_SPLIT:
  case TU_1D_HORZ_SPLIT:
  case TU_1D_VERT_SPLIT:
    currTrDepth++;
    break;
  default:
    THROW_RECOVERABLE( "Unknown split mode" );
    break;
  }

  currDepth++;
  currSubdiv++;
  
  qgEnable       &= ( currSubdiv <= slice->getCuQpDeltaSubdiv() );
  qgChromaEnable &= ( currSubdiv <= slice->getCuChromaQpOffsetSubdiv() );
  if( qgEnable )
    currQgPos = currArea().lumaPos();
  if( qgChromaEnable )
    currQgChromaPos = currArea().chromaPos();
  
  back.qgEnable       = qgEnable;
  back.qgChromaEnable = qgChromaEnable;
  back.split          = split;
  back.modeType       = modeType;
  back.cuAbove        = last.cuAbove;
  back.cuLeft         = last.cuLeft;

#if _DEBUG

  m_currArea = m_partStack.back().parts[0];
#endif
}

void Partitioner::canSplit( const CodingStructure &cs, bool& canNo, bool& canQt, bool& canBh, bool& canBv, bool& canTh, bool& canTv ) const
{
  canNo = canQt = canBh = canTh = canBv = canTv = true;
  bool canBtt = currMtDepth < ( maxBTD + currImplicitBtDepth );

  // the minimal and maximal sizes are given in luma samples
  const CompArea&  area  = currArea().Y();
  const CompArea  *areaC = (chType == CHANNEL_TYPE_CHROMA) ? &(currArea().Cb()) : nullptr;
  const PartLevel& level = m_partStack.back();

  if( isDualITree && ( area.width > 64 || area.height > 64 ) )
  {
    canQt                                 = true;
    canNo = canBh = canTh = canBv = canTv = false;

    return;
  }

  if( treeType == TREE_C )
  {
    canQt = canBh = canTh = canBv = canTv = false;
    return;
  }

  const PartSplit lastSplit   = level.split;
  const bool      isTrInPic   = area.x + area.width  <= cs.picture->lwidth();
  const bool      isBlInPic   = area.y + area.height <= cs.picture->lheight();
  const bool      isImplicit  = !isBlInPic || !isTrInPic;

  // don't allow QT-splitting below a BT split
  if( lastSplit != CTU_LEVEL && lastSplit != CU_QUAD_SPLIT ) canQt = false;
  // minQtSize is in luma samples unit
  const unsigned minQTThreshold = minQtSize;
  if( area.width <= minQTThreshold )                         canQt = false;

  if( areaC && areaC->width <= MIN_DUALTREE_CHROMA_WIDTH ) canQt = false;
  if( isImplicit )
  {
    const bool isBtAllowed = area.width <= maxBtSize && area.height <= maxBtSize
                          && area.width <= MAX_TU_SIZE_FOR_PROFILE && area.height <= MAX_TU_SIZE_FOR_PROFILE
                          && canBtt;
    canNo  = canTh = canTv = false;
    
    canQt |= !isBtAllowed;
    canBh  =  isBtAllowed && !isBlInPic && ( isTrInPic || !canQt );
    canBv  =  isBtAllowed &&  isBlInPic &&  !isTrInPic;
    canBv &= ( !areaC || areaC->width > 4 );
    canQt |= !canBh && !canBv;
    return;
  }

  canBtt &=   area.width >  minBtSize || area.height >  minBtSize   ||   area.width >  minTtSize || area.height >  minTtSize;
  canBtt &= ( area.width <= maxBtSize && area.height <= maxBtSize ) || ( area.width <= maxTtSize && area.height <= maxTtSize );

  if( !canBtt )
  {
    canBh = canTh = canBv = canTv = false;
    return;
  }

  const bool allowModeBt = modeType != MODE_TYPE_INTER || area.area() != 32;
  const bool allowModeTt = modeType != MODE_TYPE_INTER || area.area() != 64;

  if( area.width > maxBtSize || area.height > maxBtSize || !allowModeBt )
  {
    canBh = canBv = false;
  }
  else
  {
    if( ( lastSplit == CU_TRIH_SPLIT || lastSplit == CU_TRIV_SPLIT ) && level.idx == 1 )
    {
      const PartSplit parlSplit = lastSplit == CU_TRIH_SPLIT ? CU_HORZ_SPLIT : CU_VERT_SPLIT;

      canBh = parlSplit != CU_HORZ_SPLIT;
      canBv = parlSplit != CU_VERT_SPLIT;
    }

    // specific check for BT splits
    canBh &= ( area.height > minBtSize && area.height <= maxBtSize );
    canBh &= ( area.width <= MAX_TU_SIZE_FOR_PROFILE || area.height > MAX_TU_SIZE_FOR_PROFILE );

    canBv &= ( area.width > minBtSize && area.width <= maxBtSize );
    canBv &= ( area.width > MAX_TU_SIZE_FOR_PROFILE || area.height <= MAX_TU_SIZE_FOR_PROFILE );
  }

  if( area.width > maxTtSize || area.height > maxTtSize || !allowModeTt || !( area.width <= MAX_TU_SIZE_FOR_PROFILE && area.height <= MAX_TU_SIZE_FOR_PROFILE ) )
  {
    canTh = canTv = false;

    if( !canBh && !canBv ) return;
  }
  else
  {
    canTh &= !( area.height <= 2 * minTtSize );
    canTv &= !( area.width  <= 2 * minTtSize );
  }

  if( areaC )
  {
    canBh &=  ( areaC->width * areaC->height > MIN_DUALTREE_CHROMA_SIZE );
    canTh &=  ( areaC->width * areaC->height > MIN_DUALTREE_CHROMA_SIZE*2 );
    canBv &=  ( areaC->width * areaC->height > MIN_DUALTREE_CHROMA_SIZE   && areaC->width > 4 );
    canTv &=  ( areaC->width * areaC->height > MIN_DUALTREE_CHROMA_SIZE*2 && areaC->width > 8 );
  }
}

bool Partitioner::canSplit( const PartSplit split, const CodingStructure &cs, bool isISP ) const
{
  CHECKD( split < TU_MAX_TR_SPLIT, "This function should only be used for transformation split handling!" );

  if( split == TU_MAX_TR_SPLIT )
  {
    const CompArea& area = currArea().Y();

    return area.width > maxTrSize || area.height > maxTrSize;
  }
  else
  {
    return currTrDepth == 0;
  }
  
  return false;
}

void Partitioner::exitCurrSplit( const CodingStructure& cs )
{
  const PartSplit currSplit = m_partStack.back().split;
  const int       currIndex = m_partStack.back().idx;
  const int       numParts  = m_partStack.back().numParts;

  m_partStack.pop_back();
  m_partBufIdx -= numParts;

  const bool isImplicit     = !cs.picture->Y().contains( currArea().Y().bottomRight() );

  CHECKD( currDepth == 0, "depth is '0', although a split was performed" );

  currDepth --;
  currSubdiv--;

  if( currQgEnable() )
    currQgPos = currArea().lumaPos();
  if( currArea().chromaFormat != CHROMA_400 && currQgChromaEnable() )
    currQgChromaPos = currArea().chromaPos();

#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  if( currSplit == CU_HORZ_SPLIT || currSplit == CU_VERT_SPLIT || currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT )
  {
    CHECKD( currMtDepth == 0, "MT depth is '0', athough a BT split was performed" );

    currMtDepth--;
    if( isImplicit ) currImplicitBtDepth--;
    
    if( ( currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT ) && currIndex != 1 )
    {
      currSubdiv--;
    }
  }
  else if( currSplit == TU_MAX_TR_SPLIT )
  {
    CHECKD( currTrDepth == 0, "TR depth is '0', although a TU split was performed" );

    currTrDepth--;
  }
  else if( currSplit >= SBT_VER_HALF_POS0_SPLIT && currSplit <= SBT_HOR_QUAD_POS1_SPLIT )
  {
    CHECKD( currTrDepth == 0, "TR depth is '0', although a TU split was performed" );

    currTrDepth--;
  }
  else if( currSplit == TU_1D_HORZ_SPLIT || currSplit == TU_1D_VERT_SPLIT )
  {
    currTrDepth--;
  }
  else
  {
    CHECKD( currTrDepth  > 0, "RQT found with QTBT partitioner" );
    CHECKD( currQtDepth == 0, "QT depth is '0', although a QT split was performed" );

    currQtDepth--;
    currSubdiv --;
  }
}

bool Partitioner::nextPart( const CodingStructure &cs, bool autoPop /*= false*/ )
{
        PartLevel& back   =   m_partStack.back();
  const unsigned currIdx  = ++back.idx;
  const PartSplit currSpl =   back.split;

  if( currIdx < back.numParts )
  {
#if _DEBUG
    m_currArea = back.parts[currIdx];
#endif
    if( currSpl <= CU_TRIV_SPLIT )
      setNeighborCu( back, *this, cs );
    
    if( currSpl == CU_TRIH_SPLIT || currSpl == CU_TRIV_SPLIT )
    {
      if( currIdx == 1 ) currSubdiv--;
      else               currSubdiv++;
    }

    if( currQgEnable() )
      currQgPos = currArea().lumaPos();
    if( currQgChromaEnable() )
      currQgChromaPos = currArea().chromaPos();

    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit( cs );
    return false;
  }
}

void Partitioner::updateNeighbors( const CodingStructure& cs )
{
  setNeighborCu( m_partStack.back(), *this, cs );
}

bool Partitioner::hasNextPart() const
{
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().numParts );
}

//////////////////////////////////////////////////////////////////////////
// Partitioner methods describing the actual partitioning logic
//////////////////////////////////////////////////////////////////////////

int PartitionerImpl::getCUSubPartitions( const UnitArea &cuArea, const CodingStructure &cs, const PartSplit _splitType, Partitioning& dst )
{
  const PartSplit splitType = _splitType;

  if( splitType == CU_QUAD_SPLIT )
  {
    Partitioning& sub = dst;

    for( uint32_t i = 0; i < 4; i++ )
    {
      sub[i] = cuArea;

      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 1;
        blk.width  >>= 1;
        if( i >= 2 ) blk.y += blk.height;
        if( i &  1 ) blk.x += blk.width;
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }

    return 4;
  }
  else if( splitType == CU_HORZ_SPLIT )
  {
    Partitioning& sub = dst;

    for (uint32_t i = 0; i < 2; i++)
    {
      sub[i] = cuArea;

      for (auto &blk : sub[i].blocks)
      {
        blk.height >>= 1;
        if (i == 1) blk.y += blk.height;
      }

      CHECK(sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size");
    }

    return 2;
  }
  else if( splitType == CU_VERT_SPLIT )
  {
    Partitioning& sub = dst;

    for( uint32_t i = 0; i < 2; i++ )
    {
      sub[i] = cuArea;

      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;
        if( i == 1 ) blk.x += blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }

    return 2;
  }
  else if( splitType == CU_TRIH_SPLIT )
  {
    Partitioning& sub = dst;

    for( int i = 0; i < 3; i++ )
    {
      sub[i] = cuArea;

      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 1;
        if( ( i + 1 ) & 1 ) blk.height >>= 1;
        if( i == 1 )        blk.y       +=     blk.height / 2;
        if( i == 2 )        blk.y       += 3 * blk.height;
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return 3;
  }
  else if( splitType == CU_TRIV_SPLIT )
  {
    Partitioning& sub = dst;

    for( int i = 0; i < 3; i++ )
    {
      sub[i] = cuArea;

      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;

        if( ( i + 1 ) & 1 ) blk.width >>= 1;
        if( i == 1 )        blk.x      +=     blk.width / 2;
        if( i == 2 )        blk.x      += 3 * blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return 3;
  }
  else
  {
    THROW_RECOVERABLE( "Unknown CU sub-partitioning" );
  }
}

int PartitionerImpl::getTUIntraSubPartitions( const UnitArea &tuArea, const CodingStructure &cs, const bool isDualTree, const PartSplit splitType, Partitioning &sub, const TreeType treeType )
{
  uint32_t nPartitions;
  uint32_t splitDimensionSize = CU::getISPSplitDim( tuArea.lumaSize().width, tuArea.lumaSize().height, splitType );

  if( splitType == TU_1D_HORZ_SPLIT )
  {
    nPartitions = tuArea.lumaSize().height >> getLog2(splitDimensionSize);

    for( uint32_t i = 0; i < nPartitions; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMPONENT_Y];

      blkY.height = splitDimensionSize;
      blkY.y = i > 0 ? sub[i - 1].blocks[COMPONENT_Y].y + splitDimensionSize : blkY.y;

      CHECK( sub[i].lumaSize().height < 1, "the cs split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == TU_1D_VERT_SPLIT )
  {
    nPartitions = tuArea.lumaSize().width >> getLog2(splitDimensionSize);

    for( uint32_t i = 0; i < nPartitions; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMPONENT_Y];

      blkY.width = splitDimensionSize;
      blkY.x = i > 0 ? sub[i - 1].blocks[COMPONENT_Y].x + splitDimensionSize : blkY.x;
      CHECK( sub[i].lumaSize().width < 1, "the split causes the block to be smaller than the minimal TU size" );
    }
  }
  else
  {
    THROW_RECOVERABLE( "Unknown TU sub-partitioning" );
  }
  //we only partition luma, so there is going to be only one chroma tu at the end (unless it is dual tree, in which case there won't be any chroma components)
  uint32_t partitionsWithoutChroma = (cs.area.chromaFormat == CHROMA_400) ? 0 : (isDualTree ? nPartitions : nPartitions - 1);
  for( uint32_t i = 0; i < partitionsWithoutChroma; i++ )
  {
    CompArea& blkCb = sub[i].blocks[COMPONENT_Cb];
    CompArea& blkCr = sub[i].blocks[COMPONENT_Cr];
    blkCb = CompArea();
    blkCr = CompArea();
  }

  return nPartitions;
}


static const int g_rsScanToZ_w4[16] =
{
   0,  1,  4,  5, // wouldn't work for 128x32 blocks, but those are forbidden bcs of VPDU constraints
   2,  3,  6,  7, // correct ordering for 128x64 (TU32)
   8,  9, 12, 13,
  10, 11, 14, 15, // correct ordering for 128x128 (TU32)
};

static const int g_rsScanToZ_w2[8] =
{
   0,  1, // correct ordering for 64x32 (TU32) and 128x64 (TU64)
   2,  3, // correct ordering for 64x64 (TU32) and 128x128 (TU64)
   4,  5,
   6,  7, // correct ordering for 32x64 (TU32) and 64x128 (TU64)
};

static const int g_rsScanToZ_w1[4] =
{
   0, // no tiling, never used
   1, // correct ordering for 64x32 (TU32) and 128x64 (TU64)
   2,
   3, // correct ordering for 128x32 (TU32)
};

static const int* g_rsScanToZ[3] = { g_rsScanToZ_w1, g_rsScanToZ_w2, g_rsScanToZ_w4 };

int PartitionerImpl::getMaxTuTiling( const UnitArea &cuArea, const CodingStructure &cs, Partitioning& dst )
{
  const Size area     = cuArea.lumaSize();
  const int maxTrSize = cs.sps->getMaxTbSize();
  const int numTilesH = std::max<int>( 1, area.width  / maxTrSize );
  const int numTilesV = std::max<int>( 1, area.height / maxTrSize );
  const int numTiles  = numTilesH * numTilesV;
  const int numLog2H  = getLog2( numTilesH );
  const int* rsScanToZ = g_rsScanToZ[numLog2H];

  CHECK( numTiles > MAX_CU_TILING_PARTITIONS, "CU partitioning requires more partitions than available" );

  Partitioning& ret = dst;

  for( int i = 0; i < numTiles; i++ )
  {
    ret[i] = cuArea;

    const int zid = rsScanToZ[i];

    const int y = zid >>         numLog2H;
    const int x = zid & ( ( 1 << numLog2H) - 1 );

    UnitArea& tile = ret[i];

    for( CompArea &comp : tile.blocks )
    {
      if( !comp.valid() ) continue;

      comp.width  /= numTilesH;
      comp.height /= numTilesV;

      comp.x += comp.width  * x;
      comp.y += comp.height * y;
    }
  }

  return numTiles;
}

int PartitionerImpl::getSbtTuTiling( const UnitArea& cuArea, const CodingStructure &cs, const PartSplit splitType, Partitioning& dst )
{
  Partitioning& ret = dst;
  int numTiles      = 2;
  int widthFactor, heightFactor, xOffsetFactor, yOffsetFactor;

  CHECK( !(splitType >= SBT_VER_HALF_POS0_SPLIT && splitType <= SBT_HOR_QUAD_POS1_SPLIT), "wrong" );

  for( int i = 0; i < numTiles; i++ )
  {
    ret[i] = cuArea;

    if( splitType >= SBT_VER_QUAD_POS0_SPLIT )
    {
      if( splitType == SBT_HOR_QUAD_POS0_SPLIT || splitType == SBT_HOR_QUAD_POS1_SPLIT )
      {
        widthFactor   = 4;
        xOffsetFactor = 0;
        heightFactor  = ( ( i == 0 &&        splitType == SBT_HOR_QUAD_POS0_SPLIT ) || ( i == 1 && splitType == SBT_HOR_QUAD_POS1_SPLIT ) ) ? 1 : 3;
        yOffsetFactor =   ( i == 0 ) ? 0 : ( splitType == SBT_HOR_QUAD_POS0_SPLIT ? 1 : 3 );
      }
      else
      {
        widthFactor   = ( ( i == 0 &&        splitType == SBT_VER_QUAD_POS0_SPLIT ) || ( i == 1 && splitType == SBT_VER_QUAD_POS1_SPLIT ) ) ? 1 : 3;
        xOffsetFactor =   ( i == 0 ) ? 0 : ( splitType == SBT_VER_QUAD_POS0_SPLIT ? 1 : 3 );
        heightFactor  = 4;
        yOffsetFactor = 0;
      }
    }
    else
    {
      if( splitType == SBT_HOR_HALF_POS0_SPLIT || splitType == SBT_HOR_HALF_POS1_SPLIT )
      {
        widthFactor   = 4;
        xOffsetFactor = 0;
        heightFactor  = 2;
        yOffsetFactor = ( i == 0 ) ? 0 : 2;
      }
      else
      {
        widthFactor   = 2;
        xOffsetFactor = ( i == 0 ) ? 0 : 2;
        heightFactor  = 4;
        yOffsetFactor = 0;
      }
    }

    UnitArea& tile = ret[i];

    for( CompArea &comp : tile.blocks )
    {
      if( !comp.valid() ) continue;

      comp.x     += ( comp.width  * xOffsetFactor ) >> 2;
      comp.y     += ( comp.height * yOffsetFactor ) >> 2;
      comp.width  = ( comp.width  * widthFactor   ) >> 2;
      comp.height = ( comp.height * heightFactor  ) >> 2;
    }
  }

  return numTiles;
}

}
