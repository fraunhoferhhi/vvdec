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

#pragma once

#include "Unit.h"

#include "CommonDef.h"

namespace vvdec
{

typedef UnitArea* Partitioning;

//////////////////////////////////////////////////////////////////////////
// PartManager class - manages the partitioning tree
//
// contains the currently processed partitioning area (currArea)
// as well as the all partitioning decisions that led to this area
// being processed (in m_partStack).
//////////////////////////////////////////////////////////////////////////

enum PartSplit
{
  CTU_LEVEL        = 0,
  CU_QUAD_SPLIT,
  CU_HORZ_SPLIT,
  CU_VERT_SPLIT,
  CU_TRIH_SPLIT,
  CU_TRIV_SPLIT,
  TU_MAX_TR_SPLIT,
  TU_NO_ISP,
  TU_1D_HORZ_SPLIT,
  TU_1D_VERT_SPLIT,
  SBT_VER_HALF_POS0_SPLIT,
  SBT_VER_HALF_POS1_SPLIT,
  SBT_HOR_HALF_POS0_SPLIT,
  SBT_HOR_HALF_POS1_SPLIT,
  SBT_VER_QUAD_POS0_SPLIT,
  SBT_VER_QUAD_POS1_SPLIT,
  SBT_HOR_QUAD_POS0_SPLIT,
  SBT_HOR_QUAD_POS1_SPLIT,
  NUM_PART_SPLIT,
  CU_DONT_SPLIT           = 2000  ///< dummy element to indicate no splitting
};



struct PartLevel
{
  PartSplit         split;
  Partitioning      parts;
  unsigned          numParts;
  unsigned          idx;
  const CodingUnit *cuAbove;
  const CodingUnit *cuLeft;
  ModeType          modeType;
  bool              qgEnable;
  bool              qgChromaEnable;

  PartLevel();
  PartLevel( const PartSplit _split, const Partitioning _parts );

  void init();
};

// set depending on max QT / BT possibilities
typedef static_vector<PartLevel, 2 * MAX_CU_DEPTH + 1> PartitioningStack;

class Partitioner
{
protected:
  PartitioningStack m_partStack;
#if _DEBUG
  UnitArea          m_currArea;
#endif
  static const size_t partBufSize = 128;
  UnitArea          m_partBuf[partBufSize];
  ptrdiff_t         m_partBufIdx;

public:

  bool     isDualITree;
  unsigned maxBTD;
  unsigned maxBtSize;
  unsigned minBtSize;
  unsigned maxTtSize;
  unsigned minTtSize;
  unsigned maxTrSize;
  unsigned minQtSize;

  unsigned currDepth;
  unsigned currQtDepth;
  unsigned currTrDepth;
  unsigned currSubdiv;
  unsigned currMtDepth;
  unsigned currSliceIdx;
  unsigned currTileIdx;
  Position currQgPos;
  Position currQgChromaPos;

  unsigned currImplicitBtDepth;
  ChannelType chType;
  TreeType treeType;
  ModeType modeType;

  const Slice* slice;

  const PartLevel& currPartLevel          () const { return m_partStack.back(); }
  const UnitArea&  currArea               () const { return currPartLevel().parts[currPartIdx()]; }
        unsigned   currPartIdx            () const { return currPartLevel().idx; }
  const PartitioningStack& getPartStack   () const { return m_partStack; }
  const bool currQgEnable                 () const { return currPartLevel().qgEnable; }
  const bool currQgChromaEnable           () const { return currPartLevel().qgChromaEnable; }

  SplitSeries getSplitSeries              () const;

  void initCtu                            ( const UnitArea& ctuArea, const ChannelType _chType, const CodingStructure& cs, const Slice& slice );
  void splitCurrArea                      ( const PartSplit split, const CodingStructure &cs );
  void exitCurrSplit                      ( const CodingStructure& cs );
  bool nextPart                           ( const CodingStructure &cs, bool autoPop = false );
  bool hasNextPart                        () const;
  void setCUData                          ( CodingUnit& cu );
  void canSplit                           ( const CodingStructure &cs, bool& canNo, bool& canQt, bool& canBh, bool& canBv, bool& canTh, bool& canTv ) const;
  bool canSplit                           ( const PartSplit split, const CodingStructure &cs, bool isISP = false ) const;
  bool isSepTree                          ( const CodingStructure &cs ) const;
  bool isConsInter                        () const { return modeType == MODE_TYPE_INTER; }
  bool isConsIntra                        () const { return modeType == MODE_TYPE_INTRA; }
  void updateNeighbors                    ( const CodingStructure& cs );
};

//////////////////////////////////////////////////////////////////////////
// Partitioner namespace - contains methods calculating the actual splits
//////////////////////////////////////////////////////////////////////////

namespace PartitionerImpl
{
  int getCUSubPartitions     ( const UnitArea &area, const CodingStructure &cs, const PartSplit splitType, Partitioning& dst );
  int getMaxTuTiling         ( const UnitArea &area, const CodingStructure &cs,                            Partitioning& dst );
  int getTUIntraSubPartitions( const UnitArea &area, const CodingStructure &cs, const bool isDualITree,    const PartSplit splitType, Partitioning &sub, const TreeType treeType );
  int getSbtTuTiling         ( const UnitArea &area, const CodingStructure &cs, const PartSplit splitType, Partitioning& dst );
}

}
