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

/** \file     UnitPartitioner.h
 *  \brief    Provides a class for partitioning management
 */

#include "UnitPartitioner.h"

#include "CodingStructure.h"
#include "Unit.h"
#include "Slice.h"
#include "UnitTools.h"
#include "Picture.h"


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

PartLevel::PartLevel( const PartSplit _split, const Partitioning& _parts )
: split               ( _split        )
, parts               ( _parts        )
, idx                 ( 0u            )
, cuAbove             ( nullptr       )
, cuLeft              ( nullptr       )
, modeType            ( MODE_TYPE_ALL )
, qgEnable            ( true          )
, qgChromaEnable      ( true          )
{
}

PartLevel::PartLevel( const PartSplit _split, Partitioning&& _parts )
: split               ( _split                               )
, parts               ( std::forward<Partitioning>( _parts ) )
, idx                 ( 0u                                   )
, cuAbove             ( nullptr                              )
, cuLeft              ( nullptr                              )
, modeType            ( MODE_TYPE_ALL                        )
, qgEnable            ( true                                 )
, qgChromaEnable      ( true                                 )
{
}


void PartLevel::init()
{
  split = CU_DONT_SPLIT;
  idx = 0u;
  cuAbove = nullptr;
  cuLeft = nullptr;
  modeType = MODE_TYPE_ALL;
  qgEnable = true;
  qgChromaEnable = true;
  parts.clear();
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
#if JVET_Q0438_MONOCHROME_BUGFIXES
  currQgChromaPos = ctuArea.chromaFormat != CHROMA_400 ? ctuArea.chromaPos() : Position();
#else
  currQgChromaPos = ctuArea.chromaPos();
#endif
  currImplicitBtDepth = 0;

  currSliceIdx = slice.getIndependentSliceIdx();
  currTileIdx  = cs.pps->getTileIdx( ctuArea.lumaPos() );

  m_partStack.resize_noinit( 1 );
  m_partStack.back().split = CTU_LEVEL;
  m_partStack.back().parts.resize( 1 );
  m_partStack.back().parts[0] = ctuArea;
  treeType = TREE_D;
  modeType = MODE_TYPE_ALL;

  setNeighborCu( m_partStack.back(), *this, cs );

  const SPS& sps = *cs.sps;

  isDualITree = slice.isIRAP() && slice.getSPS()->getUseDualITree();

  const int valIdx = slice.isIRAP() ? ( _chType << 1 ) : 1;

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
    const unsigned maxBtDepthArr[] = { sps.getMaxBTDepthI(), sps.getMaxBTDepth(), sps.getMaxBTDepthIChroma() };
    const unsigned maxBtSizeArr[]  = { sps.getMaxBTSizeI(), sps.getMaxBTSize(), sps.getMaxBTSizeIChroma() };
    const unsigned maxTtSizeArr[]  = { sps.getMaxTTSizeI(), sps.getMaxTTSize(), sps.getMaxTTSizeIChroma() };
    const unsigned minQtSizeArr[]  = { sps.getMinQTSize( I_SLICE, CHANNEL_TYPE_LUMA ), sps.getMinQTSize( B_SLICE, CHANNEL_TYPE_LUMA ), sps.getMinQTSize( I_SLICE, CHANNEL_TYPE_CHROMA ) };

    maxBTD    = maxBtDepthArr[valIdx];
    maxBtSize = maxBtSizeArr [valIdx];
    maxTtSize = maxTtSizeArr [valIdx];
    minQtSize = minQtSizeArr [valIdx];
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

  if     ( split <= CU_TRIV_SPLIT )
    PartitionerImpl::getCUSubPartitions     ( area, cs, split, back.parts );
  else if( split == TU_MAX_TR_SPLIT )
    PartitionerImpl::getMaxTuTiling         ( area, cs,        back.parts );
  else if( split >= SBT_VER_HALF_POS0_SPLIT && split <= SBT_HOR_QUAD_POS1_SPLIT )
    PartitionerImpl::getSbtTuTiling         ( area, cs, split, back.parts );
  else /* if( split == TU_1D_HORZ_SPLIT || split == TU_1D_VERT_SPLIT ) */
    PartitionerImpl::getTUIntraSubPartitions( area, cs, isDualITree, split, back.parts, treeType );

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
    THROW( "Unknown split mode" );
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

  m_currArea = m_partStack.back().parts.front();
#endif
}

void Partitioner::canSplit( const CodingStructure &cs, bool& canNo, bool& canQt, bool& canBh, bool& canBv, bool& canTh, bool& canTv ) const
{
  canNo = canQt = canBh = canTh = canBv = canTv = true;
  bool canBtt = currMtDepth < ( maxBTD + currImplicitBtDepth );

  // the minimal and maximal sizes are given in luma samples
  const CompArea&  area  = currArea().Y();
#if JVET_Q0438_MONOCHROME_BUGFIXES
  const CompArea  *areaC = (chType == CHANNEL_TYPE_CHROMA) ? &(currArea().Cb()) : nullptr;
#else
  const CompArea&  areaC = currArea().Cb();
#endif
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
  const unsigned minQTThreshold = minQtSize >> ( ( cs.area.chromaFormat == CHROMA_400 )
                                                ? 0
                                                : ( ( int ) getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, cs.area.chromaFormat ) - ( int ) getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, cs.area.chromaFormat ) ) );
  if( area.width <= minQTThreshold )                         canQt = false;

#if JVET_Q0438_MONOCHROME_BUGFIXES
  if( areaC && areaC->width <= MIN_DUALTREE_CHROMA_WIDTH ) canQt = false;
#else
  if( chType == CHANNEL_TYPE_CHROMA && areaC.width <= MIN_DUALTREE_CHROMA_WIDTH ) canQt = false;
#endif
  if( isImplicit )
  {
    const bool isBtAllowed = area.width <= maxBtSize && area.height <= maxBtSize && area.width <= MAX_TB_SIZEY && area.height <= MAX_TB_SIZEY;
    canNo  = canTh = canTv = false;
    
    canQt |= !isBtAllowed;
    canBh  =  isBtAllowed && !isBlInPic && ( isTrInPic || !canQt );
    canBv  =  isBtAllowed &&  isBlInPic &&  !isTrInPic;
#if JVET_Q0438_MONOCHROME_BUGFIXES
    canBv &= ( !areaC || areaC->width > 4 );
#else
    if( chType == CHANNEL_TYPE_CHROMA && areaC.width == 4 ) canBv = false;
#endif
    canQt |= !canBh && !canBv;
    return;
  }

  canBtt &= !( area.width <= minBtSize && area.height <= minBtSize && ( ( area.width <= minTtSize && area.height <= minTtSize ) ) );
  canBtt &= !( ( area.width > maxBtSize || area.height > maxBtSize ) && ( area.width > maxTtSize || area.height > maxTtSize ) );

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
  
#if JVET_Q0438_MONOCHROME_BUGFIXES
  if( areaC )
  {
    canBh &=  ( areaC->width * areaC->height > MIN_DUALTREE_CHROMA_SIZE );
    canTh &=  ( areaC->width * areaC->height > MIN_DUALTREE_CHROMA_SIZE*2 );
    canBv &=  ( areaC->width * areaC->height > MIN_DUALTREE_CHROMA_SIZE   && areaC->width > 4 );
    canTv &=  ( areaC->width * areaC->height > MIN_DUALTREE_CHROMA_SIZE*2 && areaC->width > 8 );
  }
#else
  if( chType == CHANNEL_TYPE_CHROMA )
  {
    canBh &=  ( areaC.width * areaC.height > MIN_DUALTREE_CHROMA_SIZE );
    canTh &=  ( areaC.width * areaC.height > MIN_DUALTREE_CHROMA_SIZE*2 );
    canBv &=  ( areaC.width * areaC.height > MIN_DUALTREE_CHROMA_SIZE   && areaC.width > 4 );
    canTv &=  ( areaC.width * areaC.height > MIN_DUALTREE_CHROMA_SIZE*2 && areaC.width > 8 );
  }
#endif
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

  m_partStack.pop_back();

  const bool isImplicit     = !cs.picture->Y().contains( currArea().Y().bottomRight() );

  CHECKD( currDepth == 0, "depth is '0', although a split was performed" );

  currDepth --;
  currSubdiv--;

  if( currQgEnable() )
    currQgPos = currArea().lumaPos();
#if JVET_Q0438_MONOCHROME_BUGFIXES
  if( currArea().chromaFormat != CHROMA_400 && currQgChromaEnable() )
#else
  if( currQgChromaEnable() )
#endif
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

  if( currIdx < back.parts.size() )
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
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().parts.size() );
}

//////////////////////////////////////////////////////////////////////////
// Partitioner methods describing the actual partitioning logic
//////////////////////////////////////////////////////////////////////////

void PartitionerImpl::getCUSubPartitions( const UnitArea &cuArea, const CodingStructure &cs, const PartSplit _splitType, Partitioning& dst )
{
  const PartSplit splitType = _splitType;

  if( splitType == CU_QUAD_SPLIT )
  {
    Partitioning& sub = dst;

    sub.resize( 4, cuArea );

    for( uint32_t i = 0; i < 4; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 1;
        blk.width  >>= 1;
        if( i >= 2 ) blk.y += blk.height;
        if( i &  1 ) blk.x += blk.width;
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }

    return;
  }
  else if( splitType == CU_HORZ_SPLIT )
  {
    Partitioning& sub = dst;

    sub.resize(2, cuArea);

    for (uint32_t i = 0; i < 2; i++)
    {
      for (auto &blk : sub[i].blocks)
      {
        blk.height >>= 1;
        if (i == 1) blk.y += blk.height;
      }

      CHECK(sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size");
    }

    return;
  }
  else if( splitType == CU_VERT_SPLIT )
  {
    Partitioning& sub = dst;

    sub.resize( 2, cuArea );

    for( uint32_t i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;
        if( i == 1 ) blk.x += blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the split causes the block to be smaller than the minimal TU size" );
    }

    return;
  }
  else if( splitType == CU_TRIH_SPLIT )
  {
    Partitioning& sub = dst;

    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 1;
        if( ( i + 1 ) & 1 ) blk.height >>= 1;
        if( i == 1 )        blk.y       +=     blk.height / 2;
        if( i == 2 )        blk.y       += 3 * blk.height;
      }

      CHECK( sub[i].lumaSize().height < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return;
  }
  else if( splitType == CU_TRIV_SPLIT )
  {
    Partitioning& sub = dst;

    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;

        if( ( i + 1 ) & 1 ) blk.width >>= 1;
        if( i == 1 )        blk.x      +=     blk.width / 2;
        if( i == 2 )        blk.x      += 3 * blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TU_SIZE, "the cs split causes the block to be smaller than the minimal TU size" );
    }

    return;
  }
  else
  {
    THROW( "Unknown CU sub-partitioning" );
  }
}

void PartitionerImpl::getTUIntraSubPartitions( const UnitArea &tuArea, const CodingStructure &cs, const bool isDualTree, const PartSplit splitType, Partitioning &sub, const TreeType treeType )
{
  uint32_t nPartitions;
  uint32_t splitDimensionSize = CU::getISPSplitDim( tuArea.lumaSize().width, tuArea.lumaSize().height, splitType );

  if( splitType == TU_1D_HORZ_SPLIT )
  {
    nPartitions = tuArea.lumaSize().height >> getLog2(splitDimensionSize);

    sub.resize( nPartitions );

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

    sub.resize( nPartitions );

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
    THROW( "Unknown TU sub-partitioning" );
  }
  //we only partition luma, so there is going to be only one chroma tu at the end (unless it is dual tree, in which case there won't be any chroma components)
#if JVET_Q0438_MONOCHROME_BUGFIXES
  uint32_t partitionsWithoutChroma = (cs.area.chromaFormat == CHROMA_400) ? 0 : (isDualTree ? nPartitions : nPartitions - 1);
#else
  uint32_t partitionsWithoutChroma = isDualTree ? nPartitions : nPartitions - 1;
#endif
  for( uint32_t i = 0; i < partitionsWithoutChroma; i++ )
  {
    CompArea& blkCb = sub[i].blocks[COMPONENT_Cb];
    CompArea& blkCr = sub[i].blocks[COMPONENT_Cr];
    blkCb = CompArea();
    blkCr = CompArea();
  }
}

static const int g_maxRtGridSize = 3;

static const int g_zScanToX[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  1,  0,  1,  2,  3,  2,  3,
   0,  1,  0,  1,  2,  3,  2,  3,
   4,  5,  4,  5,  6,  7,  6,  7,
   4,  5,  4,  5,  6,  7,  6,  7,
   0,  1,  0,  1,  2,  3,  2,  3,
   0,  1,  0,  1,  2,  3,  2,  3,
   4,  5,  4,  5,  6,  7,  6,  7,
   4,  5,  4,  5,  6,  7,  6,  7,
};
static const int g_zScanToY[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  0,  1,  1,  0,  0,  1,  1,
   2,  2,  3,  3,  2,  2,  3,  3,
   0,  0,  1,  1,  0,  0,  1,  1,
   2,  2,  3,  3,  2,  2,  3,  3,
   4,  4,  5,  5,  4,  4,  5,  5,
   6,  6,  7,  7,  6,  6,  7,  7,
   4,  4,  5,  5,  4,  4,  5,  5,
   6,  6,  7,  7,  6,  6,  7,  7,
};
static const int g_rsScanToZ[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  1,  4,  5, 16, 17, 20, 21,
   2,  3,  6,  7, 18, 19, 22, 23,
   8,  9, 12, 13, 24, 25, 28, 29,
  10, 11, 14, 15, 26, 27, 30, 31,
  32, 33, 36, 37, 48, 49, 52, 53,
  34, 35, 38, 39, 50, 51, 54, 55,
  40, 41, 44, 45, 56, 57, 60, 61,
  42, 43, 46, 47, 58, 59, 62, 63,
};

void PartitionerImpl::getMaxTuTiling( const UnitArea &cuArea, const CodingStructure &cs, Partitioning& dst )
{
  static_assert( MAX_LOG2_DIFF_CU_TR_SIZE <= g_maxRtGridSize, "Z-scan tables are only provided for MAX_LOG2_DIFF_CU_TR_SIZE for up to 3 (8x8 tiling)!" );

  const Size area     = cuArea.lumaSize();
  const int maxTrSize = ( area.width > 64 || area.height > 64 ) ? 64 : cs.sps->getMaxTbSize();
  const int numTilesH = std::max<int>( 1, area.width  / maxTrSize );
  const int numTilesV = std::max<int>( 1, area.height / maxTrSize );
  const int numTiles  = numTilesH * numTilesV;

  CHECK( numTiles > MAX_CU_TILING_PARTITIONS, "CU partitioning requires more partitions than available" );

  Partitioning& ret = dst;
  ret.resize( numTiles, cuArea );

  for( int i = 0; i < numTiles; i++ )
  {
    const int rsy = i / numTilesH;
    const int rsx = i % numTilesH;

    const int x = g_zScanToX[g_rsScanToZ[( rsy << g_maxRtGridSize ) + rsx]];
    const int y = g_zScanToY[g_rsScanToZ[( rsy << g_maxRtGridSize ) + rsx]];

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

  return;
}

void PartitionerImpl::getSbtTuTiling( const UnitArea& cuArea, const CodingStructure &cs, const PartSplit splitType, Partitioning& dst )
{
  Partitioning& ret = dst;
  int numTiles      = 2;
  int widthFactor, heightFactor, xOffsetFactor, yOffsetFactor;

  CHECK( !(splitType >= SBT_VER_HALF_POS0_SPLIT && splitType <= SBT_HOR_QUAD_POS1_SPLIT), "wrong" );

  ret.resize( numTiles, cuArea );

  for( int i = 0; i < numTiles; i++ )
  {
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

  return;
}
