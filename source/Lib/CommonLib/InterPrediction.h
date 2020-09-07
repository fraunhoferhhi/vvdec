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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#ifndef __INTERPREDICTION__
#define __INTERPREDICTION__


// Include files
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"
#include "ContextModelling.h"
// forward declaration
class Mv;

//! \ingroup CommonLib
//! \{


// ====================================================================================================================
// Class definition
// ====================================================================================================================

class InterPrediction : public WeightPrediction
{
protected:
  InterpolationFilter  m_if;

  Pel*                 m_acYuvPred            [    NUM_REF_PIC_LIST_01][MAX_NUM_COMPONENT];
  Pel*                 m_filteredBlockTmp     [2 * NUM_REF_PIC_LIST_01][MAX_NUM_COMPONENT];

  ChromaFormat         m_currChromaFormat = NUM_CHROMA_FORMAT;

  RdCost*              m_pcRdCost = nullptr;

  int                  m_iRefListIdx = -1;
  PelStorage           m_geoPartBuf;
  Mv                   m_storedMv[( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];

  Pel*                 m_gradX0  = nullptr;
  Pel*                 m_gradY0  = nullptr;
  Pel*                 m_gradX1  = nullptr;
  Pel*                 m_gradY1  = nullptr;
  bool                 m_subPuMC = false;

  UnitArea             m_currCuArea;

  /*buffers for bilinear Filter data for DMVR refinement*/
  Pel*                 m_cYuvPredTempDMVRL0 = nullptr;
  Pel*                 m_cYuvPredTempDMVRL1 = nullptr;
  int                  m_biLinearBufStride;
  /*buffers for padded data*/
  PelUnitBuf           m_cYuvRefBuffDMVRL0;
  PelUnitBuf           m_cYuvRefBuffDMVRL1;
  Pel*                 m_cRefSamplesDMVRL0[MAX_NUM_COMPONENT];
  Pel*                 m_cRefSamplesDMVRL1[MAX_NUM_COMPONENT];
  Mv m_pSearchOffset[25] = { Mv(-2,-2), Mv(-1,-2), Mv(0,-2), Mv(1,-2), Mv(2,-2),
                             Mv(-2,-1), Mv(-1,-1), Mv(0,-1), Mv(1,-1), Mv(2,-1),
                             Mv(-2, 0), Mv(-1, 0), Mv(0, 0), Mv(1, 0), Mv(2, 0),
                             Mv(-2, 1), Mv(-1, 1), Mv(0, 1), Mv(1, 1), Mv(2, 1),
                             Mv(-2, 2), Mv(-1, 2), Mv(0, 2), Mv(1, 2), Mv(2, 2) };
  uint64_t             m_SADsArray[((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)];

  Pel                  m_gradBuf[2][(AFFINE_MIN_BLOCK_SIZE + 2) * (AFFINE_MIN_BLOCK_SIZE + 2)];
  int                  m_dMvBuf[2][16 * 2];
  int                  m_IBCBufferWidth;
  PelStorage           m_IBCBuffer;

  void xIntraBlockCopy          (PredictionUnit &pu, PelUnitBuf &predBuf, const ComponentID compID);

  void            applyBiOptFlow(const PredictionUnit &pu, const PelUnitBuf &yuvSrc0, const PelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths);

  void xPredInterUni            ( const PredictionUnit& pu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi
                                  , const bool& bioApplied
                                  , const bool luma, const bool chroma
  );
  void xPredInterBi             ( PredictionUnit& pu, PelUnitBuf &pcYuvPred );
  template<bool altSrc, bool altSize>
  void xPredInterBlk            ( const ComponentID&    compID,
                                  const PredictionUnit& pu,
                                  const Picture*        refPic,
                                  Mv                    mv,
                                  PelBuf&               dstPic,
                                  bool                  bi,
                                  const ClpRng&         clpRng,
                                  bool                  bioApplied,
                                  bool                  isIBC,
                                  bool                  wrapRef,
                                  SizeType              dmvrWidth    = 0,
                                  SizeType              dmvrHeight   = 0,
                                  bool                  bilinearMC   = false,
                                  Pel*                  srcPadBuf    = NULL,
                                  ptrdiff_t             srcPadStride = 0 );

  void (*BiOptFlow)             ( const Pel* srcY0,const Pel* srcY1,const Pel* gradX0,const Pel* gradX1,const Pel* gradY0,const Pel* gradY1,const int width,const int height,Pel* dstY,const ptrdiff_t dstStride,const int shiftNum,const int  offset,const int  limit, const ClpRng& clpRng, const int bitDepth ) = nullptr;
  void (*BioGradFilter)         (       Pel* pSrc, ptrdiff_t srcStride,  int width, int height, ptrdiff_t gradStride, Pel* gradX, Pel* gradY, const int bitDepth ) = nullptr;

  void( *PaddBIO )              ( const Pel* refPel, Pel* dstPel, unsigned width, const int shift );

  void xWeightedAverage         ( const PredictionUnit& pu, const PelUnitBuf& pcYuvSrc0, const PelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied );
  void( *profGradFilter )       ( Pel* pSrc, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride, Pel* gradX, Pel* gradY, const int bitDepth );
  void( *applyPROF[2] )         ( Pel* dst, ptrdiff_t dstStride, const Pel* src, const Pel* gradX, const Pel* gradY, const int* dMvX, const int* dMvY, int shiftNum, Pel offset, const ClpRng& clpRng );
  void( *roundIntVector )       ( int* v, int size, unsigned int nShift, const int dmvLimit );
#if JVET_R0058
  void( *clipMv )               ( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps );
#endif

  void xPredAffineBlk           ( const ComponentID& compID, const PredictionUnit& pu, const Picture* refPic, const RefPicList refPicList, PelUnitBuf& dstPic, bool bi, const ClpRng& clpRng, const std::pair<int, int> scalingRatio = SCALE_1X );
  static bool xCheckIdenticalMotion
                                ( const PredictionUnit& pu );

  void xSubPuMC                 ( PredictionUnit& pu, PelUnitBuf& predBuf );
  void xSubPuBio                ( PredictionUnit& pu, PelUnitBuf& predBuf );
  void destroy();

public:
  InterPrediction();
  virtual ~InterPrediction();

  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize);

  // inter
  void    motionCompensation  (PredictionUnit &pu, PelUnitBuf& predBuf, const bool luma = true, const bool chroma = true);

  void    motionCompensationGeo      ( PredictionUnit &pu, PelUnitBuf &predBuf );
  void    weightedGeoBlk             ( PredictionUnit &pu, const uint8_t splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 );
  void    xPrefetch                  ( PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma );
  void    xPad                       ( PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma );
  void    xFinalPaddedMCForDMVR      ( PredictionUnit& pu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1, PelUnitBuf &pcPad0, PelUnitBuf &pcPad1, const bool bioApplied, const Mv startMV[NUM_REF_PIC_LIST_01] );
  void xBIPMVRefine(DistParam &cDistParam, const Pel *pRefL0, const Pel *pRefL1, uint64_t& minCost, int16_t *deltaMV, uint64_t *pSADsArray);
  void xinitMC(PredictionUnit& pu, const ClpRngs &clpRngs);
  void xProcessDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied );
  static bool isSubblockVectorSpreadOverLimit( int a, int b, int c, int d, int predType );
  void xFillIBCBuffer(CodingUnit &cu);
#if JVET_O1170_CHECK_BV_AT_DECODER
  void resetIBCBuffer(const ChromaFormat chromaFormatIDC, const int ctuSize);
  void resetVPDUforIBC(const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos);
  bool isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv);
#endif
  void xPredInterBlkRPR( const std::pair<int, int>& scalingRatio, const PPS& pps, const ComponentID& compID, const ChromaFormat chFmt, const Picture* refPic, const Mv& mv, const Position blkPos, const int dstWidth, const int dstHeight, Pel* dst, const ptrdiff_t dstStride, const bool bi, const bool wrapRef, const ClpRng& clpRng, const int filterIndex, const bool useAltHpelIf = false );
#if ENABLE_SIMD_OPT_BIO

  void initInterPredictionX86();
  template <X86_VEXT vext>
  void _initInterPredictionX86();
#endif
};

//! \}

#endif // __INTERPREDICTION__
