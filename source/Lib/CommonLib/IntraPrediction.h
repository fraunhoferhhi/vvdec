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

/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#pragma once

// Include files
#include "Unit.h"
#include "Buffer.h"

#include "MatrixIntraPrediction.h"


namespace vvdec
{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
enum PredBuf
{
  PRED_BUF_UNFILTERED = 0,
  PRED_BUF_FILTERED   = 1,
  NUM_PRED_BUF        = 2
};

static const uint32_t MAX_INTRA_FILTER_DEPTHS=8;

class IntraPrediction
{
private:

  Pel    m_piYuvExt     [NUM_PRED_BUF][( MAX_TU_SIZE_FOR_PROFILE * 2 + 1 + MAX_REF_LINE_IDX ) * ( MAX_TU_SIZE_FOR_PROFILE * 2 + 1 + MAX_REF_LINE_IDX )];
  PelBuf m_pelBufISPBase[NUM_PRED_BUF];
  PelBuf m_pelBufISP    [NUM_PRED_BUF];
  int    m_neighborSize [3];
  int    m_lastCUidx;
  static const uint8_t
       m_aucIntraFilter [MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];

  MatrixIntraPrediction m_matrixIntraPred;

protected:

  ChromaFormat  m_currChromaFormat;

  int m_topRefLength;
  int m_leftRefLength;
  // prediction
  void xPredIntraDc               ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType,                                                                                          const bool enableBoundaryFilter = true, const int mrlIdx = 0 );
  void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const uint32_t dirMode, const ClpRng& clpRng, const SPS& sps,
                                          int  multiRefIdx,
                                    const bool useFilteredPredSamples,
                                          bool &doPDPC,
                                    const bool useISP = false,
                                    const Size cuSize = Size( 0, 0 )
                                   );

  void xPredIntraBDPCM            ( const CPelBuf &pSrc, PelBuf &pDst, const uint32_t dirMode, const ClpRng& clpRng );
  Pel  xGetPredValDc              ( const CPelBuf &pSrc, const Size &dstSize, const int mrlIdx );
  
  void xFillReferenceSamples      ( const CPelBuf &recoBuf,      Pel* refBufUnfiltered, const CompArea &area, const TransformUnit &tu );
  void xFilterReferenceSamples    ( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps, int multiRefIdx, ptrdiff_t predStride = 0 ) const;

  static int getWideAngle         ( int width, int height, int predMode );
  void setReferenceArrayLengths   ( const CompArea &area );

  void destroy                    ();

  void xGetLMParameters           (const CodingUnit &cu, const ComponentID compID, const CompArea& chromaArea, int& a, int& b, int& iShift);
public:
  IntraPrediction();
  ~IntraPrediction();

  void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY);

  // Angular Intra
  void predIntraAng               (const ComponentID compId, PelBuf &piPred, const CodingUnit &cu, const bool useFilteredPredSamples);
  Pel*  getPredictorPtr           (const ComponentID compID, const bool bUseFilteredPredictions = false) { return m_piYuvExt[bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED]; }
  // Cross-component Chroma
  void predIntraChromaLM          (const ComponentID compID, PelBuf &piPred, const CodingUnit &cu, const CompArea& chromaArea, int intraDir);
  void xGetLumaRecPixels          (const CodingUnit &cu, CompArea chromaArea);
  /// set parameters from CU data for accessing intra data
  void initIntraPatternChType     (const TransformUnit &cu, const CompArea &area, const bool bFilterRefSamples = false );
  void initIntraPatternChTypeISP  (const CodingUnit& cu, const CompArea& area, PelBuf& piReco);
  const PelBuf& getISPBuffer      (const bool bUseFilteredPredictions = false) { return m_pelBufISP[bUseFilteredPredictions ? PRED_BUF_FILTERED : PRED_BUF_UNFILTERED]; }

  // Matrix-based intra prediction
  void initIntraMip               (const CodingUnit &cu, const CompArea &area);
  void predIntraMip               (const ComponentID compId, PelBuf &piPred, const CodingUnit &cu);

  static bool useFilteredIntraRefSamples( const ComponentID &compID, const CodingUnit &cu, const UnitArea &tuArea );

  void predBlendIntraCiip         (PelUnitBuf &predUnit, const CodingUnit &cu);

  void ( *IntraPredAngleCore4 )         ( Pel* pDstBuf,const ptrdiff_t dstStride,Pel* refMain,int width,int height,int deltaPos,int intraPredAngle,const TFilterCoeff *ff,const bool useCubicFilter,const ClpRng& clpRng);
  void ( *IntraPredAngleCore8 )         ( Pel* pDstBuf,const ptrdiff_t dstStride,Pel* refMain,int width,int height,int deltaPos,int intraPredAngle,const TFilterCoeff *ff,const bool useCubicFilter,const ClpRng& clpRng);

  void( *IntraPredAngleChroma4 )        ( int16_t* pDst, const ptrdiff_t dstStride, int16_t* pBorder, int width, int height, int deltaPos, int intraPredAngle );
  void( *IntraPredAngleChroma8 )        ( int16_t* pDst, const ptrdiff_t dstStride, int16_t* pBorder, int width, int height, int deltaPos, int intraPredAngle );

  void  ( *IntraPredSampleFilter8 )     ( Pel *ptrSrc,const ptrdiff_t  srcStride,PelBuf &piPred,const uint32_t uiDirMode,const ClpRng& clpRng );
  void  ( *IntraPredSampleFilter16 )    ( Pel *ptrSrc,const ptrdiff_t  srcStride,PelBuf &piPred,const uint32_t uiDirMode,const ClpRng& clpRng );
  void  ( *xPredIntraPlanar )           ( const CPelBuf &pSrc, PelBuf &pDst, const SPS& sps );

  void ( *GetLumaRecPixel420 )          ( const int width,const int height, const Pel* pRecSrc0,const ptrdiff_t iRecStride,Pel* pDst0,const ptrdiff_t iDstStride );

#if ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_X86 )
  void initIntraPredictionX86();
  template <X86_VEXT vext>
  void _initIntraPredictionX86();
#endif
};

}
