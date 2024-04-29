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

/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#pragma once

// Include files
#include "InterpolationFilter.h"
#include "WeightPrediction.h"

#include "Buffer.h"
#include "Unit.h"
#include "Picture.h"

#include "RdCost.h"

namespace vvdec
{

// forward declaration
class Mv;

//! \ingroup CommonLib
//! \{


// ====================================================================================================================
// Class definition
// ====================================================================================================================

class InterPrediction : public WeightPrediction
{
  friend class TrQuant; // for the access to the shared buffers m_acYuvPRed
protected:
  InterpolationFilter  m_if;

  Pel                  m_gradX0[BIO_TEMP_BUFFER_SIZE];
  Pel                  m_gradY0[BIO_TEMP_BUFFER_SIZE];
  Pel                  m_gradX1[BIO_TEMP_BUFFER_SIZE];
  Pel                  m_gradY1[BIO_TEMP_BUFFER_SIZE];

  Pel                  m_bdofBlock[NUM_REF_PIC_LIST_01][( MAX_BDOF_APPLICATION_REGION + ( 2 * BIO_ALIGN_SIZE + BIO_ALIGN_SIZE ) + 16 )
                                                      * ( MAX_BDOF_APPLICATION_REGION + ( 2 * BIO_EXTEND_SIZE + 2 ) + 2 )];
  Pel                  m_acYuvPred[MAX_NUM_COMPONENT][MAX_CU_SIZE * MAX_CU_SIZE];
  // TrQuant will use m_acYuvPred as its internal buffers
  Pel                  m_tmpBlock[MAX_CU_SIZE * (MAX_CU_SIZE + NTAPS_LUMA)];

  /*buffers for padded data*/
  Pel                  m_cRefSamplesDMVRL0[MAX_NUM_COMPONENT][(DMVR_SUBCU_WIDTH + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA) * (DMVR_SUBCU_HEIGHT + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA)];
  Pel                  m_cRefSamplesDMVRL1[MAX_NUM_COMPONENT][(DMVR_SUBCU_WIDTH + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA) * (DMVR_SUBCU_HEIGHT + (2 * DMVR_NUM_ITERATION) + NTAPS_LUMA)];
  /*buffers for bilinear Filter data for DMVR refinement*/
  Pel                  m_cYuvPredTempDMVRL0[(MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION))];
  Pel                  m_cYuvPredTempDMVRL1[(MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION)) * (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION))];
  int                  m_biLinearBufStride;
  const Mv             m_pSearchOffset[25] = { Mv(-2,-2), Mv(-1,-2), Mv(0,-2), Mv(1,-2), Mv(2,-2),
                                               Mv(-2,-1), Mv(-1,-1), Mv(0,-1), Mv(1,-1), Mv(2,-1),
                                               Mv(-2, 0), Mv(-1, 0), Mv(0, 0), Mv(1, 0), Mv(2, 0),
                                               Mv(-2, 1), Mv(-1, 1), Mv(0, 1), Mv(1, 1), Mv(2, 1),
                                               Mv(-2, 2), Mv(-1, 2), Mv(0, 2), Mv(1, 2), Mv(2, 2) };
  Distortion           m_SADsArray[((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)];

  ChromaFormat         m_currChromaFormat = NUM_CHROMA_FORMAT;

  RdCost*              m_pcRdCost = nullptr;

  int                  m_iRefListIdx = -1;
  Mv                   m_storedMv[( MAX_CU_SIZE * MAX_CU_SIZE ) >> ( MIN_CU_LOG2 << 1 )];
  bool                 m_subPuMC = false;

  UnitArea             m_currCuArea;
  int                  m_IBCBufferWidth;
  PelStorage           m_IBCBuffer;

  void xIntraBlockCopy          (CodingUnit &cu, PelUnitBuf &predBuf, const ComponentID compID);

  void applyBiOptFlow           (const CodingUnit &cu, const PelUnitBuf &yuvSrc0, const PelUnitBuf &yuvSrc1, const int &refIdx0, const int &refIdx1, PelUnitBuf &yuvDst, const BitDepths &clipBitDepths);

  void xPredInterUni            ( const CodingUnit& cu, const RefPicList& eRefPicList, PelUnitBuf& pcYuvPred, const bool& bi
                                  , const bool& bioApplied
                                  , const bool luma, const bool chroma
  );
  void xPredInterBi             ( CodingUnit& cu, PelUnitBuf &pcYuvPred );
  template<bool altSrc, bool altSize>
  void xPredInterBlk            ( const ComponentID&    compID,
                                  const CodingUnit& cu,
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

  void xWeightedAverage         ( const CodingUnit& cu, const PelUnitBuf& pcYuvSrc0, const PelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied );
  void( *profGradFilter )       ( Pel* pSrc, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride, Pel* gradX, Pel* gradY, const int bitDepth );
  void( *applyPROF[2] )         ( Pel* dst, ptrdiff_t dstStride, const Pel* src, const Pel* gradX, const Pel* gradY, const int* dMvX, const int* dMvY, int shiftNum, Pel offset, const ClpRng& clpRng );
  void( *roundIntVector )       ( int* v, int size, unsigned int nShift, const int dmvLimit );
  void( *clipMv )               ( Mv& rcMv, const Position& pos, const struct Size& size, const SPS& sps, const PPS& pps );
  void( *prefetchPad[3] )       ( const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height );

  void xPredAffineBlk           ( const ComponentID& compID, const CodingUnit& cu, const Picture* refPic, const RefPicList refPicList, PelUnitBuf& dstPic, bool bi, const ClpRng& clpRng, const std::pair<int, int> scalingRatio = SCALE_1X );
  static bool xCheckIdenticalMotion
                                ( const CodingUnit& cu );

  void xSubPuMC                 ( CodingUnit& cu, PelUnitBuf& predBuf );
  void xSubPuBio                ( CodingUnit& cu, PelUnitBuf& predBuf );
  void destroy();

public:
  InterPrediction();
  virtual ~InterPrediction();

  void    init                (RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize);

  // inter
  void    motionCompensation  (CodingUnit &cu, PelUnitBuf& predBuf, const bool luma = true, const bool chroma = true);

  void    motionCompensationGeo      ( CodingUnit &cu, PelUnitBuf &predBuf );
  void    weightedGeoBlk             ( CodingUnit &cu, const uint8_t splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1 );

  static bool isSubblockVectorSpreadOverLimit( int a, int b, int c, int d, int predType );

private:
  void    xPrefetchPad               ( CodingUnit& cu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma );
  void    xFinalPaddedMCForDMVR      ( CodingUnit& cu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1, PelUnitBuf &pcPad0, PelUnitBuf &pcPad1, const bool bioApplied, const Mv startMV[NUM_REF_PIC_LIST_01] );
  void xBIPMVRefine(DistParam &cDistParam, const Pel *pRefL0, const Pel *pRefL1, Distortion& minCost, int16_t *deltaMV, Distortion *pSADsArray);
  void xinitMC(CodingUnit& cu, const ClpRngs &clpRngs);
  void xProcessDMVR(CodingUnit& cu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied );
#if JVET_O1170_CHECK_BV_AT_DECODER
  void resetIBCBuffer(const ChromaFormat chromaFormatIDC, const int ctuSize);
  void resetVPDUforIBC(const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos);
  bool isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv);
#endif
  void xPredInterBlkRPR( const std::pair<int, int>& scalingRatio, const PPS& pps, const ComponentID& compID, const ChromaFormat chFmt, const Picture* refPic, const Mv& mv, const Position blkPos, const int dstWidth, const int dstHeight, Pel* dst, const ptrdiff_t dstStride, const bool bi, const bool wrapRef, const ClpRng& clpRng, const int filterIndex, const bool useAltHpelIf = false );
#if ENABLE_SIMD_OPT_INTER && defined( TARGET_SIMD_X86 )

  void initInterPredictionX86();
  template <X86_VEXT vext>
  void _initInterPredictionX86();
#endif
};

}
