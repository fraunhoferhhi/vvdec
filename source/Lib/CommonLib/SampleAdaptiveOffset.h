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

/** \file     SampleAdaptiveOffset.h
    \brief    sample adaptive offset class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Unit.h"
#include "Reshape.h"

#include <array>

namespace vvdec
{

template<typename T> static inline int sgn( T val )
{
  return ( T( 0 ) < val ) - ( val < T( 0 ) );
}

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define MAX_SAO_TRUNCATED_BITDEPTH     10

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class SampleAdaptiveOffset
{
public:
  SampleAdaptiveOffset() {}
  ~SampleAdaptiveOffset();

  void create( int picWidth, int picHeight, ChromaFormat format, uint32_t maxCUWidth, uint32_t maxCUHeight, uint32_t maxCUDepth, uint32_t bitShift, PelUnitBuf& unitBuf );
  void destroy();

//  void SAOProcess( CodingStructure& cs );
  void SAOProcessCTU    ( CodingStructure& cs, const UnitArea& ctuArea );
  void SAOProcessCTULine( CodingStructure& cs, const UnitArea& lineArea );
  void SAOPrepareCTULine( CodingStructure& cs, const UnitArea& lineArea );

  static int getMaxOffsetQVal( const int channelBitDepth ) { return ( 1 << ( std::min<int>( channelBitDepth, MAX_SAO_TRUNCATED_BITDEPTH ) - 5 ) ) - 1; }   // Table 9-32, inclusive
  void setReshaper(Reshape * p) { m_pcReshape = p; }

protected:
  void deriveLoopFilterBoundaryAvailibility( CodingStructure& cs,
                                             const Position&  pos,
                                             bool&            isLeftAvail,
                                             bool&            isRightAvail,
                                             bool&            isAboveAvail,
                                             bool&            isBelowAvail,
                                             bool&            isAboveLeftAvail,
                                             bool&            isAboveRightAvail,
                                             bool&            isBelowLeftAvail,
                                             bool&            isBelowRightAvail ) const;

  static void offsetBlock_core( const int            channelBitDepth,
                                const ClpRng&        clpRng,
                                int                  typeIdx,
                                int*                 offset,
                                int                  startIdx,
                                const Pel*           srcBlk,
                                Pel*                 resBlk,
                                ptrdiff_t            srcStride,
                                ptrdiff_t            resStride,
                                int                  width,
                                int                  height,
                                bool                 isLeftAvail,
                                bool                 isRightAvail,
                                bool                 isAboveAvail,
                                bool                 isBelowAvail,
                                bool                 isAboveLeftAvail,
                                bool                 isAboveRightAvail,
                                bool                 isBelowLeftAvail,
                                bool                 isBelowRightAvail,
                                std::vector<int8_t>* m_signLineBuf1,
                                std::vector<int8_t>* m_signLineBuf2,
                                bool                 isCtuCrossedByVirtualBoundaries,
                                int                  horVirBndryPos[],
                                int                  verVirBndryPos[],
                                int                  numHorVirBndry,
                                int                  numVerVirBndry );

  void ( *offsetBlock )( const int            channelBitDepth,
                         const ClpRng&        clpRng,
                         int                  typeIdx,
                         int*                 offset,
                         int                  startIdx,
                         const Pel*           srcBlk,
                         Pel*                 resBlk,
                         ptrdiff_t            srcStride,
                         ptrdiff_t            resStride,
                         int                  width,
                         int                  height,
                         bool                 isLeftAvail,
                         bool                 isRightAvail,
                         bool                 isAboveAvail,
                         bool                 isBelowAvail,
                         bool                 isAboveLeftAvail,
                         bool                 isAboveRightAvail,
                         bool                 isBelowLeftAvail,
                         bool                 isBelowRightAvail,
                         std::vector<int8_t>* m_signLineBuf1,
                         std::vector<int8_t>* m_signLineBuf2,
                         bool                 isCtuCrossedByVirtualBoundaries,
                         int                  horVirBndryPos[],
                         int                  verVirBndryPos[],
                         int                  numHorVirBndry,
                         int                  numVerVirBndry );

  void invertQuantOffsets     ( ComponentID compIdx, int typeIdc, int typeAuxInfo, int* dstOffsets, int* srcOffsets ) const;
  void reconstructBlkSAOParam ( SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] ) const;
  int  getMergeList           ( CodingStructure& cs, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] );
  void offsetCTU              ( const UnitArea& area, const CPelUnitBuf& src, PelUnitBuf& res, SAOBlkParam& saoblkParam, CodingStructure& cs, std::vector<int8_t> &signLineBuf1, std::vector<int8_t> &signLineBuf2 );

  static bool isProcessDisabled( int xPos, int yPos, int numVerVirBndry, int numHorVirBndry, int verVirBndryPos[], int horVirBndryPos[] );
  Reshape* m_pcReshape;


#ifdef TARGET_SIMD_X86
  void initSampleAdaptiveOffsetX86();
  template <X86_VEXT vext>
  void _initSampleAdaptiveOffsetX86();
#endif



protected:
  uint32_t   m_offsetStepLog2;   // offset step
  uint32_t   m_numberOfComponents;
  PelUnitBuf m_tempBuf;
};

}
