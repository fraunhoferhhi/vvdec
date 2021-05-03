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

/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#pragma once

#include "CommonDef.h"

#include "Unit.h"
#include "UnitTools.h"

#include <vector>

namespace vvdec
{

struct AlfClassifier
{
  AlfClassifier() {}
  AlfClassifier( uint8_t cIdx, uint8_t tIdx )
    : classIdx( cIdx ), transposeIdx( tIdx )
  {}

  uint8_t classIdx;
  uint8_t transposeIdx;
};

enum Direction
{
  HOR,
  VER,
  DIAG0,
  DIAG1,
  NUM_DIRECTIONS
};

class AdaptiveLoopFilter
{
public:
  static inline int clipALF(const int clip, const short ref, const short val0, const short val1)
  {
    return Clip3<int>(-clip, +clip, val0-ref) + Clip3<int>(-clip, +clip, val1-ref);
  }

  static constexpr int AlfNumClippingValues[MAX_NUM_CHANNEL_TYPE] = { 4, 4 };
  static constexpr int MaxAlfNumClippingValues   = 4;

  static constexpr int m_NUM_BITS                = 8;
  static constexpr int m_CLASSIFICATION_BLK_SIZE = 32;   // non-normative, local buffer size
  static constexpr int m_CLASSIFICATION_ARR_SIZE = m_CLASSIFICATION_BLK_SIZE * m_CLASSIFICATION_BLK_SIZE >> ( MIN_CU_LOG2 << 1 );   // non-normative, local buffer size
  static constexpr int m_ALF_UNUSED_CLASSIDX     = 255;
  static constexpr int m_ALF_UNUSED_TRANSPOSIDX  = 255;

  AdaptiveLoopFilter();
  ~AdaptiveLoopFilter() {}
  void create( const PicHeader* picHeader, const SPS* sps, const PPS* pps, int numThreads );
  void destroy();

  static void filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc,
                             const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs,
                             int vbCTUHeight, int vbPos);

  static void preparePic    ( CodingStructure &cs );
  static void prepareCTU    ( CodingStructure &cs, unsigned col, unsigned line );
         void processCTU    ( CodingStructure &cs, unsigned col, unsigned line, int tid = 0, const ChannelType chType = MAX_NUM_CHANNEL_TYPE );
  static void swapBufs      ( CodingStructure &cs );
  static bool getAlfSkipPic       ( const CodingStructure& cs );
  static void reconstructCoeffAPSs( Slice& slice );
  static void reconstructCoeff    ( AlfSliceParam& alfSliceParam, ChannelType channel, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE] );
protected:
  static void getCompatibleBuffer( const CodingStructure & cs, const CPelUnitBuf & srcBuf, PelStorage & destBuf );

  static void deriveClassificationBlk( AlfClassifier *classifier, const CPelBuf& srcLuma, const Area& blk, const int shift, int vbCTUHeight, int vbPos );
  void ( *m_deriveClassificationBlk )( AlfClassifier *classifier, const CPelBuf& srcLuma, const Area& blk, const int shift, int vbCTUHeight, int vbPos );

  void filterCTU                     ( const CPelUnitBuf& srcBuf, const PelUnitBuf& dstBuf, const uint8_t ctuEnableFlag[3], const uint8_t ctuAlternativeData[2], const ClpRngs& clpRngs, const ChannelType chType, const CodingStructure& cs, int ctuIdx, Position ctuPos, int tid );
  void filterAreaLuma                ( const CPelUnitBuf& srcBuf, const PelUnitBuf& dstBuf, const Area& blk, const Slice* slice, const APS* const* aps, const short filterSetIndex, const ClpRngs& clpRngs, const int tId );
  void filterAreaChroma              ( const CPelUnitBuf& srcBuf, const PelUnitBuf& dstBuf, const Area& blkLuma, const Area& blkChroma, const ComponentID compID, const Slice* slice, const APS* const* aps, const int ctuIdx, const uint8_t ctuComponentEnableFlag, const uint8_t ctuAlternativeData[2], const ClpRngs& clpRngs );

  template<AlfFilterType filtType>
  static void filterBlk              ( const AlfClassifier *classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, const short* filterSet, const short* fClipSet, const ClpRng& clpRng, int vbCTUHeight, int vbPos );
  void ( *m_filterCcAlf )            ( const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, int vbCTUHeight, int vbPos );

  void ( *m_filter5x5Blk )           ( const AlfClassifier *classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, const short* filterSet, const short* fClipSet, const ClpRng& clpRng, int vbCTUHeight, int vbPos );
  void ( *m_filter7x7Blk )           ( const AlfClassifier *classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, const short* filterSet, const short* fClipSet, const ClpRng& clpRng, int vbCTUHeight, int vbPos );

#ifdef TARGET_SIMD_X86
  void initAdaptiveLoopFilterX86();
  template <X86_VEXT vext>
  void _initAdaptiveLoopFilterX86();
#endif

protected:
  bool isCrossedByVirtualBoundaries( const CodingStructure& cs,
                                     const Area& pos,
                                     bool&       clipTop,
                                     bool&       clipBottom,
                                     bool&       clipLeft,
                                     bool&       clipRight,
                                     int&        numHorVirBndry,
                                     int&        numVerVirBndry,
                                     int         horVirBndryPos[],
                                     int         verVirBndryPos[],
                                     int&        rasterSliceAlfPad
                                     );

  static const int        m_fixedFilterSetCoeff   [ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF];
  static const int        m_classToFilterMapping  [NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES];
  short                   m_fixedFilterSetCoeffDec[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_TRANSPOSE_ID * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                   m_clipDefault                                  [MAX_NUM_ALF_TRANSPOSE_ID * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  static const Pel        m_alfClippVls[3][MaxAlfNumClippingValues];
  std::vector<PelStorage> m_tempBuf;
  int           m_inputBitDepth[MAX_NUM_CHANNEL_TYPE] = { 0, 0 };
  int           m_picWidth;
  int           m_picHeight;
  int           m_alfVBLumaPos;
  int           m_alfVBChmaPos;
  int           m_alfVBLumaCTUHeight;
  int           m_alfVBChmaCTUHeight;
  std::vector<std::array<AlfClassifier, m_CLASSIFICATION_ARR_SIZE> > classifier;
};

}
