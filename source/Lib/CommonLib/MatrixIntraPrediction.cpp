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

/** \file     MatrixIntraPrediction.cpp
\brief    matrix-based intra prediction class
*/

#include "MatrixIntraPrediction.h"
#include "dtrace_next.h"

#include "UnitTools.h"
#include "MipData.h"


namespace Mip
{
  PredictorMIP::PredictorMIP():
    m_blockSize( 0, 0 ),
    m_sizeId( 0 ),
    m_reducedBdrySize( 0 ),
    m_reducedPredSize( 0 ),
    m_upsmpFactorHor( 0 ),
    m_upsmpFactorVer( 0 )
  {
  }

  void PredictorMIP::deriveBoundaryData(const CPelBuf& src, const Area& block, const int bitDepth)
  {
    // Step 1: Save block size and calculate dependent values
    initPredBlockParams(block);
  
    // Step 2: Get the input data (left and top reference samples)
    //    m_refSamplesTop.resize(block.width);
    for (int x = 0; x < block.width; x++)
    {
      m_refSamplesTop[x] = src.at(x + 1, 0);
    }
  
    //    m_refSamplesLeft.resize(block.height);
    for (int y = 0; y < block.height; y++)
    {
      m_refSamplesLeft[y] = src.at(0, y + 1);
    }
  
    // Step 3: Compute the reduced boundary via Haar-downsampling (input for the prediction)
    //    m_reducedBoundary          .resize( m_reducedBoundarySize.width + m_reducedBoundarySize.height );
    //    m_reducedBoundaryTransposed.resize( m_reducedBoundarySize.width + m_reducedBoundarySize.height );
  
    //    m_reducedBoundary          .resize( inputSize );
    //    m_reducedBoundaryTransposed.resize( inputSize );
  
    int* const topReduced = m_reducedBoundary;
    boundaryDownsampling1D( topReduced, m_refSamplesTop, block.width, m_reducedBdrySize );
  
    int* const leftReduced = m_reducedBoundary + m_reducedBdrySize;
    boundaryDownsampling1D( leftReduced, m_refSamplesLeft, block.height, m_reducedBdrySize );
  
    int* const leftReducedTransposed = m_reducedBoundaryTransposed;
    int* const topReducedTransposed  = m_reducedBoundaryTransposed + m_reducedBdrySize;
    for( int x = 0; x < m_reducedBdrySize; x++ )
    {
      topReducedTransposed[x] = topReduced[x];
    }
    for( int y = 0; y < m_reducedBdrySize; y++ )
    {
      leftReducedTransposed[y] = leftReduced[y];
    }

  // Step 4: Rebase the reduced boundary
    const int inputSize = 2 * m_reducedBdrySize;
  
    m_inputOffset       = m_reducedBoundary[0];
    m_inputOffsetTransp = m_reducedBoundaryTransposed[0];
  
    const bool hasFirstCol = (m_sizeId < 2);
    m_reducedBoundary          [0] = hasFirstCol ? ((1 << (bitDepth - 1)) - m_inputOffset      ) : 0; // first column of matrix not needed for large blocks
    m_reducedBoundaryTransposed[0] = hasFirstCol ? ((1 << (bitDepth - 1)) - m_inputOffsetTransp) : 0;
    for (int i = 1; i < inputSize; i++)
    {
      m_reducedBoundary          [i] -= m_inputOffset;
      m_reducedBoundaryTransposed[i] -= m_inputOffsetTransp;
    }
  }

  void PredictorMIP::getPrediction(int* const result, const int modeIdx, const bool transpose, const int bitDepth)
  {
    const bool needUpsampling = ( m_upsmpFactorHor > 1 ) || ( m_upsmpFactorVer > 1 );

  const uint8_t* matrix = getMatrixData(modeIdx);

    int bufReducedPred[MIP_MAX_REDUCED_OUTPUT_SAMPLES];
    int* const       reducedPred     = needUpsampling ? bufReducedPred : result;
    const int* const reducedBoundary = transpose ? m_reducedBoundaryTransposed : m_reducedBoundary;
    computeReducedPred( reducedPred, reducedBoundary, matrix, transpose, bitDepth );
    if( needUpsampling )
    {
      predictionUpsampling( result, reducedPred );
    }
  }


  void PredictorMIP::initPredBlockParams(const Size& block)
  {
    m_blockSize = block;
    // init size index
    m_sizeId = getMipSizeId( m_blockSize );

    // init reduced boundary size
    m_reducedBdrySize = (m_sizeId == 0) ? 2 : 4;

    // init reduced prediction size
    m_reducedPredSize = ( m_sizeId < 2 ) ? 4 : 8;


    // init upsampling factors
    m_upsmpFactorHor = m_blockSize.width  / m_reducedPredSize;
    m_upsmpFactorVer = m_blockSize.height / m_reducedPredSize;

    CHECKD( (m_upsmpFactorHor < 1) || ((m_upsmpFactorHor & (m_upsmpFactorHor - 1)) != 0), "Need power of two horizontal upsampling factor." );
    CHECKD( (m_upsmpFactorVer < 1) || ((m_upsmpFactorVer & (m_upsmpFactorVer - 1)) != 0), "Need power of two vertical upsampling factor." );
  }



void PredictorMIP::boundaryDownsampling1D(int* reducedDst, const int* const fullSrc, const SizeType srcLen, const SizeType dstLen)
{
  if (dstLen < srcLen)
  {
    // Create reduced boundary by downsampling
      const SizeType downsmpFactor = srcLen / dstLen;
      const int log2DownsmpFactor = getLog2(downsmpFactor);
      const int roundingOffset = (1 << (log2DownsmpFactor - 1));

      SizeType srcIdx = 0;
      for( SizeType dstIdx = 0; dstIdx < dstLen; dstIdx++ )
      {
        int sum = 0;
        for( int k = 0; k < downsmpFactor; k++ )
        {
          sum += fullSrc[srcIdx++];
        }
        reducedDst[dstIdx] = (sum + roundingOffset) >> log2DownsmpFactor;
      }
  }
  else
  {
    // Copy boundary if no downsampling is needed
    for (SizeType i = 0; i < dstLen; ++i)
    {
      reducedDst[i] = fullSrc[i];
    }
  }
}


  void PredictorMIP::predictionUpsampling1D( int* const dst, const int* const src, const int* const bndry,
                                              const SizeType srcSizeUpsmpDim, const SizeType srcSizeOrthDim,
                                              const SizeType srcStep, const SizeType srcStride,
                                              const SizeType dstStep, const SizeType dstStride,
                                              const SizeType bndryStep,
                                              const unsigned int upsmpFactor )
  {
    const int log2UpsmpFactor = getLog2( upsmpFactor );
    CHECKD( upsmpFactor <= 1, "Upsampling factor must be at least 2." );
    const int roundingOffset = 1 << (log2UpsmpFactor - 1);

    SizeType idxOrthDim = 0;
    const int* srcLine = src;
    int* dstLine = dst;
    const int* bndryLine = bndry + bndryStep - 1;
    while( idxOrthDim < srcSizeOrthDim )
    {
      SizeType idxUpsmpDim = 0;
      const int* before = bndryLine;
      const int* behind = srcLine;
      int* currDst = dstLine;
      while( idxUpsmpDim < srcSizeUpsmpDim )
      {
        SizeType pos = 1;
        int scaledBefore = ( *before ) << log2UpsmpFactor;
        int scaledBehind = 0;
        while( pos <= upsmpFactor )
        {
          scaledBefore -= *before;
          scaledBehind += *behind;
          *currDst = (scaledBefore + scaledBehind + roundingOffset) >> log2UpsmpFactor;

          pos++;
          currDst += dstStep;
        }

        idxUpsmpDim++;
        before = behind;
        behind += srcStep;
      }

      idxOrthDim++;
      srcLine += srcStride;
      dstLine += dstStride;
      bndryLine += bndryStep;
    }
  }


  void PredictorMIP::predictionUpsampling( int* const dst, const int* const src ) const
  {
    const int* verSrc     = src;
    SizeType   verSrcStep = m_blockSize.width;
  
    if( m_upsmpFactorHor > 1 )
    {
      int* const horDst = dst + (m_upsmpFactorVer - 1) * m_blockSize.width;
      verSrc = horDst;
      verSrcStep *= m_upsmpFactorVer;
  
      predictionUpsampling1D( horDst, src, m_refSamplesLeft,
                              m_reducedPredSize, m_reducedPredSize,
                              1, m_reducedPredSize, 1, verSrcStep,
                              m_upsmpFactorVer, m_upsmpFactorHor );
    }
  
    if( m_upsmpFactorVer > 1 )
    {
      predictionUpsampling1D( dst, verSrc, m_refSamplesTop,
                              m_reducedPredSize, m_blockSize.width,
                              verSrcStep, 1, m_blockSize.width, 1,
                              1, m_upsmpFactorVer );
    }
  }

const uint8_t* PredictorMIP::getMatrixData(const int modeIdx) const
{
    switch( m_sizeId )
    {
      case 0: return &mipMatrix4x4[modeIdx][0][0];

      case 1: return &mipMatrix8x8[modeIdx][0][0];

      case 2: return &mipMatrix16x16[modeIdx][0][0];
    
      default: THROW( "Invalid mipSizeId" );
    }
}

  void PredictorMIP::computeReducedPred( int*const result, const int* const input,
                                          const uint8_t* matrix,
                                          const bool transpose, const int bitDepth )
  {
    const int inputSize = 2 * m_reducedBdrySize;

    // use local buffer for transposed result
    int resBufTransposed[MIP_MAX_REDUCED_OUTPUT_SAMPLES];
    int*const resPtr = (transpose) ? resBufTransposed : result;

    int sum = 0;
    for( int i = 0; i < inputSize; i++ ) { sum += input[i]; }
  const int offset = (1 << (MIP_SHIFT_MATRIX - 1)) - MIP_OFFSET_MATRIX * sum;
    CHECK( inputSize != 4 * (inputSize >> 2), "Error, input size not divisible by four" );

    const uint8_t *weight = matrix;
    const int   inputOffset = transpose ? m_inputOffsetTransp : m_inputOffset;

    const bool redSize = (m_sizeId == 2);
    int posRes = 0;
    for( int y = 0; y < m_reducedPredSize; y++ )
    {
      for( int x = 0; x < m_reducedPredSize; x++ )
      {
        if( redSize ) weight -= 1;
        int tmp0 = redSize ? 0 : (input[0] * weight[0]);
        int tmp1 = input[1] * weight[1];
        int tmp2 = input[2] * weight[2];
        int tmp3 = input[3] * weight[3];
        for (int i = 4; i < inputSize; i += 4)
        {
          tmp0 += input[i]     * weight[i];
          tmp1 += input[i + 1] * weight[i + 1];
          tmp2 += input[i + 2] * weight[i + 2];
          tmp3 += input[i + 3] * weight[i + 3];
        }
        resPtr[posRes++] = ClipBD<int>(((tmp0 + tmp1 + tmp2 + tmp3 + offset) >> MIP_SHIFT_MATRIX) + inputOffset, bitDepth);

        weight += inputSize;
      }
}

    if( transpose )
    {
      for( int y = 0; y < m_reducedPredSize; y++ )
      {
        for( int x = 0; x < m_reducedPredSize; x++ )
        {
          result[ y * m_reducedPredSize + x ] = resPtr[ x * m_reducedPredSize + y ];
        }
      }
    }
  }
} // namespace Mip

MatrixIntraPrediction::MatrixIntraPrediction()
{
}

#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
void MatrixIntraPrediction::prepareInputForPred(const CPelBuf &src, const Area& puArea, const int bitDepth, const ComponentID compId)
#else
void MatrixIntraPrediction::prepareInputForPred(const CPelBuf &src, const Area& puArea, const int bitDepth)
#endif
{
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  m_component = compId;
#endif

  m_predictorMip.deriveBoundaryData(src, puArea, bitDepth);
}

#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
void MatrixIntraPrediction::predBlock( const Size &puSize, const int intraMode, PelBuf& dst, const bool transpose, const int bitDepth, const ComponentID compId )
#else
void MatrixIntraPrediction::predBlock( const Size &puSize, const int intraMode, PelBuf& dst, const bool transpose, const int bitDepth )
#endif
{
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  CHECK( m_component != compId, "Boundary has not been prepared for this component." );
#endif
  int* const resultMip = m_mipResult;

  m_predictorMip.getPrediction( resultMip, intraMode, transpose, bitDepth );

  for( int y = 0; y < puSize.height; y++ )
  {
    int* const resultLine = &resultMip[y * puSize.width];
    Pel*       dstLine    = dst.bufAt( 0, y );

    for( int x = 0; x < puSize.width; x += 4 )
    {
      dstLine[x + 0] = Pel( resultLine[x + 0] );
      dstLine[x + 1] = Pel( resultLine[x + 1] );
      dstLine[x + 2] = Pel( resultLine[x + 2] );
      dstLine[x + 3] = Pel( resultLine[x + 3] );
    }
  }
}

