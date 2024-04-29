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

/** \file     MatrixIntraPrediction.h
\brief    matrix-based intra prediction class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Common.h"
#include "Buffer.h"

namespace vvdec
{

static const int MIP_MAX_INPUT_SIZE             =  8;
static const int MIP_MAX_REDUCED_OUTPUT_SAMPLES = 64;


namespace Mip
{
  class PredictorMIP
  {
  public:
    PredictorMIP();
    void             deriveBoundaryData(const CPelBuf& src, const Area& block, const int bitDepth);
    void             getPrediction     (Pel* const result, const int modeIdx, const bool transpose, const int bitDepth);
  private:
    Pel m_reducedBoundary          [MIP_MAX_INPUT_SIZE]; // downsampled             boundary of a block
    Pel m_reducedBoundaryTransposed[MIP_MAX_INPUT_SIZE]; // downsampled, transposed boundary of a block

    int                                    m_inputOffset;
    int                                    m_inputOffsetTransp;
    Pel m_refSamplesTop            [MIP_MAX_WIDTH       ];             // top  reference samples for upsampling
    Pel m_refSamplesLeft           [MIP_MAX_HEIGHT     ];            // left reference samples for upsampling
    Size m_blockSize;
    int  m_sizeId;
    int  m_reducedBdrySize;
    int  m_reducedPredSize;
    unsigned int m_upsmpFactorHor;
    unsigned int m_upsmpFactorVer;

    void initPredBlockParams(const Size& block);

    static void boundaryDownsampling1D(Pel* reducedDst, const Pel* const fullSrc, const SizeType srcLen, const SizeType dstLen);

    void predictionUpsampling( Pel* const dst, const Pel* const src ) const;
    static void predictionUpsampling1D( Pel* const dst, const Pel* const src, const Pel* const bndry,
                                        const SizeType srcSizeUpsmpDim, const SizeType srcSizeOrthDim,
                                        const SizeType srcStep, const SizeType srcStride,
                                        const SizeType dstStep, const SizeType dstStride,
                                        const SizeType bndryStep,
                                        const unsigned int upsmpFactor );

    const uint8_t* getMatrixData(const int modeIdx) const;


    void computeReducedPred( Pel* const result, const Pel* const input,
                             const uint8_t* matrix,
                             const bool transpose, const int bitDepth );
  };
}

class MatrixIntraPrediction
{
public:
  MatrixIntraPrediction();

  Mip::PredictorMIP m_predictorMip;
  void prepareInputForPred( const CPelBuf &src, const Area& puArea, const int bitDepth, const ComponentID compId );
  void predBlock( const Size &puSize, const int modeIdx, PelBuf &dst, const bool transpose, const int bitDepth, const ComponentID compId, Pel* const resultMip );
private:
  ComponentID m_component = MAX_NUM_COMPONENT;
};

}
