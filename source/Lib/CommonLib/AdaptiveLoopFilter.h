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

/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#ifndef __ADAPTIVELOOPFILTER__
#define __ADAPTIVELOOPFILTER__

#include "CommonDef.h"

#include "Unit.h"
#include "UnitTools.h"

#include <vector>

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
  static constexpr int m_CLASSIFICATION_BLK_SIZE = 128;   // non-normative, local buffer size
  static constexpr int m_ALF_UNUSED_CLASSIDX     = 255;
  static constexpr int m_ALF_UNUSED_TRANSPOSIDX  = 255;

  AdaptiveLoopFilter();
  ~AdaptiveLoopFilter() {}
  void create( const PicHeader* picHeader, const SPS* sps, const PPS* pps, int numThreads );
  void destroy();

  template<AlfFilterType filtTypeCcAlf>
  static void filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc,
                             const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs,
                             CodingStructure &cs, int vbCTUHeight, int vbPos);

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
  void deriveClassification          ( AlfClassifier *classifier, const CPelBuf& srcLuma, const Area& blk ) const;

  void filterCTU                     ( const CPelUnitBuf & srcBuf, const PelUnitBuf & dstBuf, const uint8_t ctuEnableFlag[3], const uint8_t ctuAlternativeData[2], const ClpRngs & clpRngs, const ChannelType chType, CodingStructure & cs, int ctuIdx, Position ctuPos, int tid );
  template<AlfFilterType filtType>
  static void filterBlk              ( const AlfClassifier *classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, const short* filterSet, const short* fClipSet, const ClpRng& clpRng, const CodingStructure& cs, int vbCTUHeight, int vbPos );

  void (*m_filterCcAlf)              ( const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs,CodingStructure &cs, int vbCTUHeight, int vbPos );

  void ( *m_filter5x5Blk )           ( const AlfClassifier *classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, const short* filterSet, const short* fClipSet, const ClpRng& clpRng, const CodingStructure& cs, int vbCTUHeight, int vbPos );
  void ( *m_filter7x7Blk )           ( const AlfClassifier *classifier, const PelUnitBuf &recDst, const CPelUnitBuf& recSrc, const Area& blk, const ComponentID compId, const short* filterSet, const short* fClipSet, const ClpRng& clpRng, const CodingStructure& cs, int vbCTUHeight, int vbPos );

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
                                     int         verVirBndryPos[] );

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
};

#endif
