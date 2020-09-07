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
