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

/** \file     MatrixIntraPrediction.h
\brief    matrix-based intra prediction class (header)
*/

#ifndef __MATRIXINTRAPPREDICTION__
#define __MATRIXINTRAPPREDICTION__


#include "Unit.h"

static const int MIP_MAX_INPUT_SIZE             =  8;
static const int MIP_MAX_REDUCED_OUTPUT_SAMPLES = 64;


namespace Mip
{
  class PredictorMIP
  {
  public:
    PredictorMIP();
    void             deriveBoundaryData(const CPelBuf& src, const Area& block, const int bitDepth);
    void             getPrediction     (int* const result, const int modeIdx, const bool transpose, const int bitDepth);
  private:
    int m_reducedBoundary          [MIP_MAX_INPUT_SIZE]; // downsampled             boundary of a block
    int m_reducedBoundaryTransposed[MIP_MAX_INPUT_SIZE]; // downsampled, transposed boundary of a block

    int                                    m_inputOffset;
    int                                    m_inputOffsetTransp;
    int m_refSamplesTop            [MIP_MAX_WIDTH       ];             // top  reference samples for upsampling
    int m_refSamplesLeft           [MIP_MAX_HEIGHT     ];            // left reference samples for upsampling
    Size m_blockSize;
    int  m_sizeId;
    int  m_reducedBdrySize;
    int  m_reducedPredSize;
    unsigned int m_upsmpFactorHor;
    unsigned int m_upsmpFactorVer;

    void initPredBlockParams(const Size& block);

    static void boundaryDownsampling1D(int* reducedDst, const int* const fullSrc, const SizeType srcLen, const SizeType dstLen);

    void predictionUpsampling( int* const dst, const int* const src ) const;
    static void predictionUpsampling1D( int* const dst, const int* const src, const int* const bndry,
                                        const SizeType srcSizeUpsmpDim, const SizeType srcSizeOrthDim,
                                        const SizeType srcStep, const SizeType srcStride,
                                        const SizeType dstStep, const SizeType dstStride,
                                        const SizeType bndryStep,
                                        const unsigned int upsmpFactor );

    const uint8_t* getMatrixData(const int modeIdx) const;


    void computeReducedPred( int*const result, const int* const input, 
                             const uint8_t* matrix,
                             const bool transpose, const int bitDepth );
  };
}

class MatrixIntraPrediction
{
public:
  MatrixIntraPrediction();

  Mip::PredictorMIP m_predictorMip;
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  void prepareInputForPred( const CPelBuf &src, const Area& puArea, const int bitDepth, const ComponentID compId );
  void predBlock( const Size &puSize, const int modeIdx, PelBuf &dst, const bool transpose, const int bitDepth, const ComponentID compId );
#else
  void prepareInputForPred(const CPelBuf &src, const Area& puArea, const int bitDepth);
  void predBlock( const Size &puSize, const int modeIdx, PelBuf &dst, const bool transpose, const int bitDepth );
#endif
private:
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
    ComponentID m_component = MAX_NUM_COMPONENT;
#endif

  int m_mipResult[MIP_MAX_WIDTH * MIP_MAX_HEIGHT];
};


#endif //__MATRIXINTRAPPREDICTION__
