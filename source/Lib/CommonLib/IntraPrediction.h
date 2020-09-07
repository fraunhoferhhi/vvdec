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

/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#ifndef __INTRAPREDICTION__
#define __INTRAPREDICTION__


// Include files
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"

#include "MatrixIntraPrediction.h"

//! \ingroup CommonLib
//! \{

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

  Pel* m_piYuvExt[MAX_NUM_COMPONENT][NUM_PRED_BUF];
  PelBuf m_pelBufISPBase[2];
  PelBuf m_pelBufISP[2];
  int  m_iYuvExtSize;

  Pel* m_yuvExt2[MAX_NUM_COMPONENT][4];
  int  m_yuvExtSize2;

  static const uint8_t m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS];

  unsigned m_auShiftLM[32]; // Table for substituting division operation by multiplication

  Pel* m_piTemp;
  Pel* m_pMdlmTemp; // for MDLM mode
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
  
  void xFillReferenceSamples      ( const CPelBuf &recoBuf,      Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu ) const;
  void xFilterReferenceSamples    ( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps, int multiRefIdx, ptrdiff_t predStride = 0 ) const;

  static int getWideAngle         ( int width, int height, int predMode );
  void setReferenceArrayLengths   ( const CompArea &area );

  void destroy                    ();

  void xFilterGroup               ( Pel* pMulDst[], int i, Pel const* const piSrc, int iRecStride, bool bAboveAvaillable, bool bLeftAvaillable);
  void xGetLMParameters(const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, int& a, int& b, int& iShift);
public:
  IntraPrediction();
  virtual ~IntraPrediction();

  void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY);

  // Angular Intra
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool useFilteredPredSamples );
  Pel*  getPredictorPtr           (const ComponentID compID, const bool bUseFilteredPredictions = false) { return m_piYuvExt[compID][bUseFilteredPredictions?PRED_BUF_FILTERED:PRED_BUF_UNFILTERED]; }
  // Cross-component Chroma
  void predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir);
  void xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea);
  /// set parameters from CU data for accessing intra data
  void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const bool bFilterRefSamples = false );
  void initIntraPatternChTypeISP  (const CodingUnit& cu, const CompArea& area, PelBuf& piReco);
  const PelBuf& getISPBuffer      (const bool bUseFilteredPredictions = false) { return m_pelBufISP[bUseFilteredPredictions ? PRED_BUF_FILTERED : PRED_BUF_UNFILTERED]; }

  // Matrix-based intra prediction
  void initIntraMip               (const PredictionUnit &pu, const CompArea &area);
  void predIntraMip               (const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);

  static bool getUseFilterRef           ( const int predMode, const int dirMode );
  static bool useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, const UnitArea &tuArea );

  void geneWeightedPred           (const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf);
  Pel* getPredictorPtr2           (const ComponentID compID, uint32_t idx) { return m_yuvExt2[compID][idx]; }
  void switchBuffer               (const PredictionUnit &pu, ComponentID compID, PelBuf srcBuff, Pel *dst);
  void geneIntrainterPred         (const CodingUnit &cu);

  void ( *IntraPredAngleCore4 )         ( Pel* pDstBuf,const ptrdiff_t dstStride,Pel* refMain,int width,int height,int deltaPos,int intraPredAngle,const TFilterCoeff *ff,const bool useCubicFilter,const ClpRng& clpRng);
  void ( *IntraPredAngleCore8 )         ( Pel* pDstBuf,const ptrdiff_t dstStride,Pel* refMain,int width,int height,int deltaPos,int intraPredAngle,const TFilterCoeff *ff,const bool useCubicFilter,const ClpRng& clpRng);

  void( *IntraPredAngleChroma4 )      ( int16_t* pDst, const ptrdiff_t dstStride, int16_t* pBorder, int width, int height, int deltaPos, int intraPredAngle );
  void( *IntraPredAngleChroma8 )      ( int16_t* pDst, const ptrdiff_t dstStride, int16_t* pBorder, int width, int height, int deltaPos, int intraPredAngle );

  void  ( *IntraPredSampleFilter8 )      (Pel *ptrSrc,const ptrdiff_t  srcStride,PelBuf &piPred,const uint32_t uiDirMode,const ClpRng& clpRng);
  void  ( *IntraPredSampleFilter16 )      (Pel *ptrSrc,const ptrdiff_t  srcStride,PelBuf &piPred,const uint32_t uiDirMode,const ClpRng& clpRng);
  void  ( *xPredIntraPlanar)            ( const CPelBuf &pSrc, PelBuf &pDst, const SPS& sps );

  void ( *GetLumaRecPixel420)  (const int width,const int height, const Pel* pRecSrc0,const ptrdiff_t iRecStride,Pel* pDst0,const ptrdiff_t iDstStride);

#if ENABLE_SIMD_OPT_INTRAPRED
  void initIntraPredictionX86();
  template <X86_VEXT vext>
  void _initIntraPredictionX86();

#endif

};

//! \}

#endif // __INTRAPREDICTION__
