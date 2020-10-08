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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"

#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"

#include <memory.h>
#include <array>

#include "CommonLib/InterpolationFilter.h"
#include "CommonLib/TimeProfiler.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint8_t IntraPrediction::m_aucIntraFilter[MAX_NUM_CHANNEL_TYPE][MAX_INTRA_FILTER_DEPTHS] =
{
  { // Luma
    24, //   1xn
    24, //   2xn
    24, //   4xn
    14, //   8xn
    2,  //  16xn
    0,  //  32xn
    0,  //  64xn
    0,  // 128xn
  },
  { // Chroma
    40, //   1xn
    40, //   2xn
    40, //   4xn
    28, //   8xn
    4,  //  16xn
    0,  //  32xn
    0,  //  64xn
    0,  // 128xn
  }
};

const TFilterCoeff g_intraGaussFilter[32][4] = {
  { 16, 32, 16,  0 },
  { 16, 32, 16,  0 },
  { 15, 31, 17,  1 },
  { 15, 31, 17,  1 },
  { 14, 30, 18,  2 },
  { 14, 30, 18,  2 },
  { 13, 29, 19,  3 },
  { 13, 29, 19,  3 },
  { 12, 28, 20,  4 },
  { 12, 28, 20,  4 },
  { 11, 27, 21,  5 },
  { 11, 27, 21,  5 },
  { 10, 26, 22,  6 },
  { 10, 26, 22,  6 },
  {  9, 25, 23,  7 },
  {  9, 25, 23,  7 },
  {  8, 24, 24,  8 },
  {  8, 24, 24,  8 },
  {  7, 23, 25,  9 },
  {  7, 23, 25,  9 },
  {  6, 22, 26, 10 },
  {  6, 22, 26, 10 },
  {  5, 21, 27, 11 },
  {  5, 21, 27, 11 },
  {  4, 20, 28, 12 },
  {  4, 20, 28, 12 },
  {  3, 19, 29, 13 },
  {  3, 19, 29, 13 },
  {  2, 18, 30, 14 },
  {  2, 18, 30, 14 },
  {  1, 17, 31, 15 },
  {  1, 17, 31, 15 },
};

void GetLumaRecPixel420Core (const int width,const int height, const Pel* pRecSrc0,const ptrdiff_t iRecStride,Pel* pDst0,const ptrdiff_t iDstStride)
{
  for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x ++ )
      {
        pDst0[x + 0 ] = (   pRecSrc0[( (x + 0 ) << 1 )    ] * 2
                          + pRecSrc0[( (x + 0 ) << 1 ) + 1] * 1
                          + pRecSrc0[( (x + 0 ) << 1 ) - 1] * 1
                          + pRecSrc0[( (x + 0 ) << 1 ) + iRecStride] * 2
                          + pRecSrc0[( (x + 0 ) << 1 ) + 1 + iRecStride] * 1
                          + pRecSrc0[( (x + 0 ) << 1 ) - 1 + iRecStride] * 1
                          + 4 ) >> 3;
      }
      pDst0 += iDstStride;
      pRecSrc0 += (iRecStride<<1);
    }
}

/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra coding).
 */

//NOTE: Bit-Limit - 24-bit source
void xPredIntraPlanarCore( const CPelBuf &pSrc, PelBuf &pDst, const SPS& sps )
{
  // with some optimizations gcc gives spurious "-Wmaybe-uninitialized" warnings here
  GCC_WARNING_DISABLE_maybe_uninitialized

  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;
  const uint32_t log2W  = getLog2( width );
  const uint32_t log2H  = getLog2( height );
  int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  const uint32_t offset = 1 << (log2W + log2H);
  // Get left and above reference column and row
  for( int k = 0; k < width + 1; k++ )
  {
    topRow[k] = pSrc.at( k + 1, 0 );
  }

  for( int k = 0; k < height + 1; k++ )
  {
    leftColumn[k] = pSrc.at( 0, k + 1 );
  }

  // Prepare intermediate variables used in interpolation
  int bottomLeft = leftColumn[height];
  int topRight = topRow[width];

  for( int k = 0; k < width; k++ )
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] << log2H;
  }

  for( int k = 0; k < height; k++ )
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] << log2W;
  }

  const uint32_t finalShift = 1 + log2W + log2H;
  const ptrdiff_t stride     = pDst.stride;
  Pel*       pred       = pDst.buf;
  for( int y = 0; y < height; y++, pred += stride )
  {
    int horPred = leftColumn[y];

    for( int x = 0; x < width; x++ )
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      int vertPred = topRow[x];
      pred[x]      = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;

    }
  }
  GCC_WARNING_RESET
}

void  IntraPredSampleFilterCore(Pel *ptrSrc,const ptrdiff_t  srcStride,PelBuf &piPred,const uint32_t uiDirMode,const ClpRng& clpRng)
{
  const CPelBuf srcBuf  ( ptrSrc, ( SizeType ) srcStride, ( SizeType ) srcStride );
  const int     iWidth  = piPred.width;
  const int     iHeight = piPred.height;
  PelBuf        dstBuf  = piPred;

  const int scale = ((getLog2(iWidth) - 2 + getLog2(iHeight) - 2 + 2) >> 2);
  CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");
  
#if 1
  if( uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX )
  {
    for( int y = 0; y < iHeight; y++ )
    {
      const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
//      const Pel left = srcBuf.at(y + 1, 1);
      const Pel left = srcBuf.at(0, y + 1 );
      for (int x = 0; x < iWidth; x++)
      {
        const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
        const Pel top   = srcBuf.at(x + 1, 0);
        const Pel val   = dstBuf.at(x, y);
        dstBuf.at(x, y) = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
      }
    }
  }
#else
  const int lev[4]={std::min(3,iWidth),std::min(6,iWidth),std::min(12,iWidth),std::min(24,iWidth)};
  if (uiDirMode == PLANAR_IDX)
  {
    for (int y = 0; y < iHeight; y++)
    {
      int wT = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel left = srcBuf.at(0, y + 1);
      if (wT)
      {
        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          dstBuf.at(x, y) = ClipPel((wL * left + wT * top + (64 - wL - wT) * dstBuf.at(x, y) + 32) >> 6, clpRng);

        }
      }
      else
      {
        for (int x = 0; x < lev[scale]; x++)   // bis wL 0 ist, das ist bei x lev[scale]
        {
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          dstBuf.at(x, y) = ClipPel((wL * left + (64 - wL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }
      }
    }
  }
  else if (uiDirMode == DC_IDX)
  {
    const Pel topLeft = srcBuf.at(0, 0);
    for (int y = 0; y < iHeight; y++)
    {
      int wT = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel left = srcBuf.at(0, y + 1);
      if (wT)
      {

        for (int x = 0; x < iWidth; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          int wTL = (wL >> 4) + (wT >> 4);
          dstBuf.at(x, y) = ClipPel((wL * left + wT * top - wTL * topLeft + (64 - wL - wT + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
          }
      }
      else
      {
        for (int x = 0; x < lev[scale]; x++)
        {
          const Pel top = srcBuf.at(x + 1, 0);
          int wL = 32 >> std::min(31, ((x << 1) >> scale));
          int wTL = (wL >> 4) + (wT >> 4);
          dstBuf.at(x, y) = ClipPel((wL * left + wT * top - wTL * topLeft + (64 - wL - wT + wTL) * dstBuf.at(x, y) + 32) >> 6, clpRng);
        }

      }
    }
  }
#endif
}

template<typename T>
void IntraPredAngleCore(T* pDstBuf,const ptrdiff_t dstStride,T* refMain,int width,int height,int deltaPos,int intraPredAngle,const TFilterCoeff *ff,const bool useCubicFilter,const ClpRng& clpRng)
{
    for (int y = 0; y<height; y++ )
    {
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & ( 32 - 1 );

      Pel p[4];

      int refMainIndex = deltaInt + 1;

      const TFilterCoeff *f = &ff[deltaFract << 2];

      for( int x = 0; x < width; x++, refMainIndex++ )
      {
        p[0] = refMain[refMainIndex - 1];
        p[1] = refMain[refMainIndex    ];
        p[2] = refMain[refMainIndex + 1];
        p[3] = refMain[refMainIndex + 2];

        pDstBuf[y*dstStride + x] = static_cast<Pel>((static_cast<int>(f[0] * p[0]) + static_cast<int>(f[1] * p[1]) + static_cast<int>(f[2] * p[2]) + static_cast<int>(f[3] * p[3]) + 32) >> 6);

        if( useCubicFilter ) // only cubic filter has negative coefficients and requires clipping
        {
          pDstBuf[y*dstStride + x] = ClipPel( pDstBuf[y*dstStride + x], clpRng );
        }
      }
      deltaPos += intraPredAngle;
    }
}

template<typename T>
void IntraPredAngleChroma(T* pDstBuf,const ptrdiff_t dstStride,int16_t* pBorder,int width,int height,int deltaPos,int intraPredAngle)
{
  for (int y = 0; y<height; y++)
  {
    const int deltaInt   = deltaPos >> 5;
    const int deltaFract = deltaPos & (32 - 1);

    // Do linear filtering
    const Pel *pRM = pBorder + deltaInt + 1;
    int lastRefMainPel = *pRM++;

    for( int x = 0; x < width; pRM++, x++ )
    {
      int thisRefMainPel = *pRM;
      pDstBuf[x + 0] = ( Pel ) ( ( ( 32 - deltaFract )*lastRefMainPel + deltaFract*thisRefMainPel + 16 ) >> 5 );
      lastRefMainPel = thisRefMainPel;
    }
    deltaPos += intraPredAngle;
    pDstBuf += dstStride;
  }

}

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction()
  : m_currChromaFormat( NUM_CHROMA_FORMAT )
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      m_piYuvExt[ch][buf] = nullptr;
    }
  }
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < 4; buf++)
    {
      m_yuvExt2[ch][buf] = nullptr;
    }
  }

  m_piTemp = nullptr;
  m_pMdlmTemp = nullptr;


  IntraPredAngleCore4 = IntraPredAngleCore;
  IntraPredAngleCore8 = IntraPredAngleCore;
  IntraPredAngleChroma4 = IntraPredAngleChroma;
  IntraPredAngleChroma8 = IntraPredAngleChroma;

  IntraPredSampleFilter8 = IntraPredSampleFilterCore;
  IntraPredSampleFilter16 = IntraPredSampleFilterCore;

  xPredIntraPlanar = xPredIntraPlanarCore;

  GetLumaRecPixel420 = GetLumaRecPixel420Core;
}

IntraPrediction::~IntraPrediction()
{
  destroy();
}

void IntraPrediction::destroy()
{
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
    {
      delete[] m_piYuvExt[ch][buf];
      m_piYuvExt[ch][buf] = nullptr;
    }
  }
  for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
  {
    for (uint32_t buf = 0; buf < 4; buf++)
    {
      delete[] m_yuvExt2[ch][buf];
      m_yuvExt2[ch][buf] = nullptr;
    }
  }

  delete[] m_piTemp;
  m_piTemp = nullptr;
  delete[] m_pMdlmTemp;
  m_pMdlmTemp = nullptr;
}

void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  if (m_yuvExt2[COMPONENT_Y][0] != nullptr && m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  if (m_piYuvExt[COMPONENT_Y][PRED_BUF_UNFILTERED] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    m_iYuvExtSize = (MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * (MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX);

    for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (uint32_t buf = 0; buf < NUM_PRED_BUF; buf++)
      {
        m_piYuvExt[ch][buf] = new Pel[m_iYuvExtSize];
      }
    }
  }

  if (m_yuvExt2[COMPONENT_Y][0] == nullptr) // check if first is null (in which case, nothing initialised yet)
  {
    m_yuvExtSize2 = (MAX_CU_SIZE) * (MAX_CU_SIZE);

    for (uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++)
    {
      for (uint32_t buf = 0; buf < 4; buf++)
      {
        m_yuvExt2[ch][buf] = new Pel[m_yuvExtSize2];
      }
    }
  }

  int shift = bitDepthY + 4;
  for (int i = 32; i < 64; i++)
  {
    m_auShiftLM[i - 32] = ((1 << shift) + i / 2) / i;
  }
  if (m_piTemp == nullptr)
  {
    m_piTemp = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
  }
  if (m_pMdlmTemp == nullptr)
  {
    m_pMdlmTemp = new Pel[(2 * MAX_CU_SIZE + 1)*(2 * MAX_CU_SIZE + 1)];//MDLM will use top-above and left-below samples.
  }

#if   ENABLE_SIMD_OPT_INTRAPRED
  initIntraPredictionX86();
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc( const CPelBuf &pSrc, const Size &dstSize, const int mrlIdx )
{
  CHECK( dstSize.width == 0 || dstSize.height == 0, "Empty area provided" );

  int idx, sum = 0;
  Pel dcVal;
  const int  width     = dstSize.width;
  const int  height    = dstSize.height;
  const auto denom     = (width == height) ? (width << 1) : std::max(width,height);
  const auto divShift  = getLog2(denom);
  const auto divOffset = (denom >> 1);

  if( width >= height )
  {
    for( idx = 0; idx < width; idx++ )
    {
      sum += pSrc.at( mrlIdx + 1 + idx, 0 );
    }
  }
  if( width <= height )
  {
    for( idx = 0; idx < height; idx++ )
    {
      sum += pSrc.at( 0, mrlIdx + 1 + idx );
    }
  }

  dcVal = (sum + divOffset) >> divShift;
  return dcVal;
}

int IntraPrediction::getWideAngle( int width, int height, int predMode )
{
  if ( predMode > DC_IDX && predMode <= VDIA_IDX )
  {
    int modeShift[] = { 0, 6, 10, 12, 14, 15 };
    int deltaSize = abs(getLog2(width) - getLog2(height));
    if (width > height && predMode < 2 + modeShift[deltaSize])
    {
      predMode += (VDIA_IDX - 1);
    }
    else if (height > width && predMode > VDIA_IDX - modeShift[deltaSize])
    {
      predMode -= (VDIA_IDX - 1);
    }
  }
  return predMode;
}

void IntraPrediction::setReferenceArrayLengths( const CompArea &area )
{
  // set Top and Left reference samples length
  const int  width    = area.width;
  const int  height   = area.height;

  m_leftRefLength     = (height << 1);
  m_topRefLength      = (width << 1);

}



void IntraPrediction::predIntraAng( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool useFilteredPredSamples )
{
  const ComponentID    compID       = MAP_CHROMA( compId );
  const ChannelType    channelType  = toChannelType( compID );
  const int            iWidth       = piPred.width;
  const int            iHeight      = piPred.height;
  const Size           cuSize       = Size( pu.blocks[compId].width, pu.blocks[compId].height );
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  CHECK( CU::isMIP(pu, toChannelType(compId)), "We should not get here for MIP." );
#endif
  const uint32_t       uiDirMode    = isLuma( compId ) && pu.bdpcmMode() ? BDPCM_IDX : !isLuma(compId) && pu.bdpcmModeChroma() ? BDPCM_IDX : PU::getFinalIntraMode(pu, channelType);
  
  CHECKD( iWidth == 2, "Width of 2 is not supported" );

  const int  multiRefIdx = ( compID == COMPONENT_Y ) ? pu.multiRefIdx() : 0;
  const bool useISP      = pu.ispMode() && isLuma( compID );
  const int srcStride    = m_topRefLength  + 1 + multiRefIdx;
  const int srcHStride   = m_leftRefLength + 1 + multiRefIdx;
  const ClpRng& clpRng   ( pu.slice->clpRng( compID ) );
        bool doPDPC      = ( iWidth >= MIN_TB_SIZEY && iHeight >= MIN_TB_SIZEY ) && multiRefIdx == 0;

  const PelBuf& srcBuf = pu.ispMode() && isLuma(compID) ? getISPBuffer( useFilteredPredSamples ) : PelBuf(getPredictorPtr(compID, useFilteredPredSamples), srcStride, srcHStride);

  switch (uiDirMode)
  {
    case(PLANAR_IDX): xPredIntraPlanar(srcBuf, piPred, *pu.cs->sps); break;
    case(DC_IDX):     xPredIntraDc    (srcBuf, piPred, channelType, false, multiRefIdx); break;
    case(BDPCM_IDX):  xPredIntraBDPCM(srcBuf, piPred, isLuma(compID) ? pu.bdpcmMode() : pu.bdpcmModeChroma(), clpRng); break;
    case(2):
    case(DIA_IDX):
    case(VDIA_IDX):
      if (getWideAngle(useISP ? cuSize.width : iWidth, useISP ? cuSize.height : iHeight, uiDirMode) == static_cast<int>(uiDirMode)) // check if uiDirMode is not wide-angle
      {
        xPredIntraAng(srcBuf, piPred, channelType, uiDirMode, clpRng, *pu.cs->sps, multiRefIdx, useFilteredPredSamples, doPDPC, useISP, cuSize );
        break;
      }
    default:          xPredIntraAng(srcBuf, piPred, channelType, uiDirMode, clpRng, *pu.cs->sps, multiRefIdx, useFilteredPredSamples, doPDPC, useISP, cuSize); break;
  }

  if( doPDPC && (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX ) )
  {
    if (iWidth>8)
      IntraPredSampleFilter16(srcBuf.buf,srcBuf.stride,piPred,uiDirMode,clpRng);
    else
      IntraPredSampleFilter8(srcBuf.buf,srcBuf.stride,piPred,uiDirMode,clpRng);
  }
}
void IntraPrediction::predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir)
{
  int  iLumaStride = 0;
  PelBuf Temp;
  if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX))
  {
    iLumaStride = 2 * MAX_CU_SIZE + 1;
    Temp = PelBuf(m_pMdlmTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  else
  {
    iLumaStride = MAX_CU_SIZE + 1;
    Temp = PelBuf(m_piTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  int a, b, iShift;
  xGetLMParameters(pu, compID, chromaArea, a, b, iShift);

  ////// final prediction
  piPred.copyFrom(Temp);
  piPred.linearTransform(a, iShift, b, true, pu.slice->clpRng(compID));
}

void IntraPrediction::xFilterGroup(Pel* pMulDst[], int i, Pel const * const piSrc, int iRecStride, bool bAboveAvaillable, bool bLeftAvaillable)
{
  pMulDst[0][i] = (piSrc[1] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[1][i] = (piSrc[iRecStride] + piSrc[iRecStride + 1] + 1) >> 1;

  pMulDst[3][i] = (piSrc[0] + piSrc[1] + 1) >> 1;

  pMulDst[2][i] = (piSrc[0] + piSrc[1] + piSrc[iRecStride] + piSrc[iRecStride + 1] + 2) >> 2;

}



void IntraPrediction::xPredIntraDc( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter, const int mrlIdx )
{
  const Pel dcval = xGetPredValDc( pSrc, pDst, mrlIdx );
  pDst.fill( dcval );
}

// Function for deriving the angular Intra predictions
void IntraPredAngleCore(Pel *pDstBuf,const int dstStride,Pel* refMain,int width,int height,int deltaPos,int intraPredAngle,const TFilterCoeff *ff,const bool useCubicFilter,const ClpRng& clpRng)
{
  for (int y = 0; y<height; y++ )
  {
    const int deltaInt   = deltaPos >> 5;
    const int deltaFract = deltaPos & ( 32 - 1 );

    Pel p[4];

    int refMainIndex = deltaInt + 1;

    const TFilterCoeff *f = &ff[deltaFract << 2];

    for( int x = 0; x < width; x++, refMainIndex++ )
    {
      p[0] = refMain[refMainIndex - 1];
      p[1] = refMain[refMainIndex    ];
      p[2] = refMain[refMainIndex + 1];
      p[3] = refMain[refMainIndex + 2];

      pDstBuf[y*dstStride + x] = static_cast<Pel>((static_cast<int>(f[0] * p[0]) + static_cast<int>(f[1] * p[1]) + static_cast<int>(f[2] * p[2]) + static_cast<int>(f[3] * p[3]) + 32) >> 6);

      if( useCubicFilter ) // only cubic filter has negative coefficients and requires clipping
      {
        pDstBuf[y*dstStride + x] = ClipPel( pDstBuf[y*dstStride + x], clpRng );
      }
    }
    deltaPos += intraPredAngle;
  }
}


/** Function for deriving the simplified angular intra predictions.
 *
 * This function derives the prediction samples for the angular mode based on the prediction direction indicated by
 * the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
 * the reference row above the block in the case of vertical prediction or displacement of the rightmost column
 * of the block and reference column left from the block in the case of the horizontal prediction. The displacement
 * is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
 * the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
 * from the extended main reference.
 */
//NOTE: Bit-Limit - 25-bit source

void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const uint32_t dirMode, const ClpRng& clpRng, const SPS& sps,
                                           int      multiRefIdx,
                                     const bool     useFilteredPredSamples ,
                                           bool     &doPDPC,
                                     const bool     useISP,
                                     const Size     cuSize
                                    )
{
  int width =int(pDst.width);
  int height=int(pDst.height);

  CHECK( !( dirMode > DC_IDX && dirMode < NUM_LUMA_MODE ), "Invalid intra dir" );
  int              predMode           = useISP ? getWideAngle( cuSize.width, cuSize.height, dirMode ) : getWideAngle( width, height, dirMode );
  const bool       bIsModeVer         = predMode >= DIA_IDX;
  const int        intraPredAngleMode = (bIsModeVer) ? predMode - VER_IDX : -(predMode - HOR_IDX);
  const int        absAngMode         = abs(intraPredAngleMode);
  const int        signAng            = intraPredAngleMode < 0 ? -1 : 1;

  // Set bitshifts and scale the angle parameter to block size
  static const int angTable[32]    = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };
  static const int invAngTable[32] = {
    0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024, 910, 819, 712, 630, 565,
    512, 468,   420,  364,  321,  287,  256,  224,  191,  161,  128,  96,  64,  48,  32,  16
  };   // (512 * 32) / Angle
  int invAngle                    = invAngTable[absAngMode];
  int absAng                      = angTable   [absAngMode];
  int intraPredAngle              = signAng * absAng;

  Pel* refMain;
  Pel* refSide;

  Pel  refAbove[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
  Pel  refLeft [2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    for (int x = 0; x <= width + 1 + multiRefIdx; x++)
    {
      refAbove[x + height] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= height + 1 + multiRefIdx; y++)
    {
      refLeft[y + width] = pSrc.at(0, y);
    }
    refMain = bIsModeVer ? refAbove + height : refLeft + width;
    refSide = bIsModeVer ? refLeft  + width : refAbove + height;

    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * invAngle + 256) >> 9, sizeSide)];
    }
  }
  else
  {
    //for (int x = 0; x <= m_topRefLength + multiRefIdx; x++)
    //{
    //  refAbove[x] = pSrc.at(x, 0);
    //}
    memcpy( refAbove, pSrc.buf, ( m_topRefLength + multiRefIdx + 1 ) * sizeof( Pel ) );
    for (int y = 0; y <= m_leftRefLength + multiRefIdx; y++)
    {
      refLeft[y] = pSrc.at(0, y);
    }

    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;

    // Extend main reference to right using replication
    const int log2Ratio = getLog2(width) - getLog2(height);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex  = (multiRefIdx << s) + 2;
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength + multiRefIdx];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
  }

  // swap width/height if we are doing a horizontal mode:
  Pel tempArray[MAX_TB_SIZEY*MAX_TB_SIZEY];
  const ptrdiff_t dstStride = bIsModeVer ? pDst.stride : MAX_TB_SIZEY;
  Pel *pDstBuf = bIsModeVer ? pDst.buf : tempArray;
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }

  // compensate for line offset in reference line buffers
  refMain += multiRefIdx;
  refSide += multiRefIdx;

  if( intraPredAngle == 0 )  // pure vertical or pure horizontal
  {
    if( doPDPC )
    {
      const int scale = ( ( getLog2( width ) - 2 + getLog2( height ) - 2 + 2 ) >> 2 );
      CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");
      const int lev[4]={std::min(3,width),std::min(6,width),std::min(12,width),std::min(24,width)};

      const Pel topLeft = pSrc.at(0, 0);
      for( int y = 0; y < height; y++ )
      {
        const Pel  left =  refSide[y + 1];
              Pel *line = &pDstBuf[y * dstStride];
        for( int x = 0; x < lev[scale]; x++ )
        {
          int wL = 32 >> std::min( 31, ( ( x << 1 ) >> scale ) );
          *line++ = ClipPel( ( wL * ( left - topLeft ) + ( refMain[x + 1] << 6 ) + 32 ) >> 6, clpRng );
        }
        memcpy( line, refMain + lev[scale] + 1, ( width - lev[scale] ) * sizeof( Pel ) );
      }
    }
    else
    {
      for( int y = 0; y < height; y++ )
      {
        memcpy( pDstBuf + y * dstStride, refMain + 1, width * sizeof( Pel ) );
      }

    }
  }
  else
  {
    Pel *pDsty=pDstBuf;

    if( !(0 == (absAng & 0x1F)) )
    {
      if( isLuma(channelType) )
      {
        int deltaPos = intraPredAngle * (1 + multiRefIdx);
        bool interpolationFlag = false, filterFlag = false;
        const int diff = std::min<int>( abs( predMode - HOR_IDX ), abs( predMode - VER_IDX ) );
        const int log2Size = ((getLog2(width) + getLog2(height)) >> 1);
        CHECKD( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
        filterFlag = (diff > m_aucIntraFilter[channelType][log2Size]);

        if( filterFlag )
        {
          const bool isRefFilter = 0 == ( absAng & 0x1F );
          interpolationFlag = !isRefFilter;
        }
        const bool useCubicFilter = useISP ? true : ( !interpolationFlag || multiRefIdx > 0 );
        const TFilterCoeff *f              = (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(0) : g_intraGaussFilter[0];
        if( ( width & 7 ) == 0 )
        {
          IntraPredAngleCore8(pDstBuf,dstStride,refMain,width,height,deltaPos,intraPredAngle,f,useCubicFilter,clpRng);

        }
        else if( ( width & 3 ) == 0 )
        {
          IntraPredAngleCore4(pDstBuf,dstStride,refMain,width,height,deltaPos,intraPredAngle,f,useCubicFilter,clpRng);
        }
        else
        {
          CHECK( !useISP, "should not happen" );

          for (int y = 0; y<height; y++ )
          {
            const int deltaInt   = deltaPos >> 5;
            const int deltaFract = deltaPos & ( 32 - 1 );

            Pel p[4];

            int refMainIndex = deltaInt + 1;

            const TFilterCoeff *ff = &f[deltaFract << 2];

            for( int x = 0; x < width; x++, refMainIndex++ )
            {
              p[0] = refMain[refMainIndex - 1];
              p[1] = refMain[refMainIndex    ];
              p[2] = refMain[refMainIndex + 1];
              p[3] = refMain[refMainIndex + 2];

              pDstBuf[y*dstStride + x] = static_cast<Pel>((static_cast<int>(ff[0] * p[0]) + static_cast<int>(ff[1] * p[1]) + static_cast<int>(ff[2] * p[2]) + static_cast<int>(ff[3] * p[3]) + 32) >> 6);

              if( useCubicFilter ) // only cubic filter has negative coefficients and requires clipping
              {
                pDstBuf[y*dstStride + x] = ClipPel( pDstBuf[y*dstStride + x], clpRng );
              }
            }
            deltaPos += intraPredAngle;
          }
        }

      }
      else
      {
        int deltaPos = intraPredAngle * (1 + multiRefIdx);
        if ( width >=8 )
        {
          IntraPredAngleChroma8(pDstBuf,dstStride,refMain,width,height,deltaPos,intraPredAngle);
        }
        else if( width == 4 )
        {
          IntraPredAngleChroma4(pDstBuf,dstStride,refMain,width,height,deltaPos,intraPredAngle);
        }
        else
        {
          IntraPredAngleChroma(pDstBuf,dstStride,refMain,width,height,deltaPos,intraPredAngle);
        }
      }

    }
    else
    {
      for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y<height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
      {
        const int deltaInt   = deltaPos >> 5;
        // Just copy the integer samples
        memcpy(pDsty,refMain  + deltaInt + 1,width*sizeof(Pel));
      }
    }

    pDsty=pDstBuf;
    for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y<height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      int angularScale = 0;
      if( intraPredAngle < 0 )
      {
        doPDPC = false;
      }
      else if( intraPredAngle > 0 )
      {
        const int sideSize = predMode >= DIA_IDX ? pDst.height : pDst.width;
        const int maxScale = 2;
        
        angularScale = std::min(maxScale, getLog2(sideSize) - (getLog2(3 * invAngle - 2) - 8));
        doPDPC &= angularScale >= 0;
      }
        
      if( doPDPC )
      {
        int invAngleSum = 256;

        for (int x = 0; x < std::min(3 << angularScale, width); x++)
        {
          invAngleSum += invAngle;

          int wL   = 32 >> (2 * x >> angularScale);
          Pel left = refSide[y + (invAngleSum >> 9) + 1];
          pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
        }
      }
    }
  }

  // Flip the block if this is the horizontal mode
  if( !bIsModeVer )
  {
    pDst.transposedFrom( CPelBuf( pDstBuf, dstStride, width, height ) );
  }
}

void IntraPrediction::xPredIntraBDPCM(const CPelBuf &pSrc, PelBuf &pDst, const uint32_t dirMode, const ClpRng& clpRng )
{
  const int wdt = pDst.width;
  const int hgt = pDst.height;

  const ptrdiff_t strideP = pDst.stride;
  const ptrdiff_t strideS = pSrc.stride;

  CHECK( !( dirMode == 1 || dirMode == 2 ), "Incorrect BDPCM mode parameter." );

  Pel* pred = &pDst.buf[0];
  if( dirMode == 1 )
  {
    Pel  val;
    for( int y = 0; y < hgt; y++ )
    {
      val = pSrc.buf[(y + 1) * strideS];
      for( int x = 0; x < wdt; x++ )
      {
        pred[x] = val;
      }
      pred += strideP;
    }
  }
  else
  {
    for( int y = 0; y < hgt; y++ )
    {
      for( int x = 0; x < wdt; x++ )
      {
        pred[x] = pSrc.buf[x + 1];
      }
      pred += strideP;
    }
  }
}

void IntraPrediction::geneWeightedPred( const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf )
{
  const int            width      = pred.width;
  const int            height     = pred.height;
  const ptrdiff_t      srcStride  = width;
  const ptrdiff_t      dstStride  = pred.stride;
  Pel*                 dstBuf     = pred.buf;

  CHECKD( width == 2, "Width of 2 is not supported" );

  const CodingUnit &cu = pu;
  const Position posBL = pu.Y().bottomLeft();
  const Position posTR = pu.Y().topRight();

  const CodingUnit *cuLeft  = cu.cs->getCURestricted( posBL.offset( -1, 0 ), cu, CHANNEL_TYPE_LUMA, cu.left );
  const CodingUnit *cuAbove = cu.cs->getCURestricted( posTR.offset( 0, -1 ), cu, CHANNEL_TYPE_LUMA, cu.above );

  const bool isNeigh0Intra = cuLeft  && ( CU::isIntra( *cuLeft ) );
  const bool isNeigh1Intra = cuAbove && ( CU::isIntra( *cuAbove ) );

  const int wIntra = 3 - !isNeigh0Intra  - !isNeigh1Intra;
  const int wMerge = 3 - !!isNeigh0Intra - !!isNeigh1Intra;

  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x += 4 )
    {
      dstBuf[y*dstStride + x + 0] = ( wMerge * dstBuf[y*dstStride + x + 0] + wIntra * srcBuf[y*srcStride + x + 0] + 2 ) >> 2;
      dstBuf[y*dstStride + x + 1] = ( wMerge * dstBuf[y*dstStride + x + 1] + wIntra * srcBuf[y*srcStride + x + 1] + 2 ) >> 2;
      dstBuf[y*dstStride + x + 2] = ( wMerge * dstBuf[y*dstStride + x + 2] + wIntra * srcBuf[y*srcStride + x + 2] + 2 ) >> 2;
      dstBuf[y*dstStride + x + 3] = ( wMerge * dstBuf[y*dstStride + x + 3] + wIntra * srcBuf[y*srcStride + x + 3] + 2 ) >> 2;
    }
  }
}

void IntraPrediction::switchBuffer(const PredictionUnit &pu, ComponentID compID, PelBuf srcBuff, Pel *dst)
{
  Pel  *src = srcBuff.bufAt(0, 0);
  int compWidth = compID == COMPONENT_Y ? pu.Y().width : pu.Cb().width;
  int compHeight = compID == COMPONENT_Y ? pu.Y().height : pu.Cb().height;
  for (int i = 0; i < compHeight; i++)
  {
    memcpy(dst, src, compWidth * sizeof(Pel));
    src += srcBuff.stride;
    dst += compWidth;
  }
}

void IntraPrediction::geneIntrainterPred( const CodingUnit &cu )
{
  if( !cu.ciipFlag() )
  {
    return;
  }
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_INTRAPRED, *cu.cs, compID );

  const PredictionUnit &pu = cu;

  PelUnitBuf predBuf;
  predBuf.bufs.resize( 3 );

#if JVET_Q0438_MONOCHROME_BUGFIXES
  int maxCompID = 1;
  if( isChromaEnabled( pu.chromaFormat ) )
  {
    maxCompID = MAX_NUM_COMPONENT;
  }
  for( int currCompID = 0; currCompID < maxCompID; currCompID++ )
#else
  for( int currCompID = 0; currCompID < 3; currCompID++ )
#endif
  {
    if( pu.chromaSize().width <= 2 && currCompID > 0 ) continue;

    const ComponentID currCompID2 = (ComponentID) currCompID;

    predBuf.bufs[currCompID] = PelBuf( getPredictorPtr2( currCompID2, 0 ), cu.blocks[currCompID] );
  }

  const bool isUseFilter = IntraPrediction::useFilteredIntraRefSamples( COMPONENT_Y, pu, cu );
  initIntraPatternChType( cu, pu.Y(), isUseFilter );
  predIntraAng( COMPONENT_Y, predBuf.Y(), pu, isUseFilter );

#if JVET_Q0438_MONOCHROME_BUGFIXES
  if( pu.chromaSize().width > 2 && isChromaEnabled( pu.chromaFormat ) )
#else
  if( pu.chromaSize().width > 2 )
#endif
  {
    initIntraPatternChType( cu, pu.Cb(), false );
    predIntraAng( COMPONENT_Cb, predBuf.Cb(), pu, false );

    initIntraPatternChType( cu, pu.Cr(), false );
    predIntraAng( COMPONENT_Cr, predBuf.Cr(), pu, false );
  }
}

inline bool isAboveLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT );
inline int  isAboveAvailable      ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags, const CodingUnit* startCU = nullptr );
inline int  isLeftAvailable       ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags, const CodingUnit* startCU = nullptr, int tuHeight = 0 );
inline int  isAboveRightAvailable ( const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags );
inline int  isBelowLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags );

void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool bFilterRefSamples)
{
  CHECK( area.width == 2, "Width of 2 is not supported" );
  const CodingStructure& cs   = *cu.cs;

  Pel *refBufUnfiltered   = m_piYuvExt[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered     = m_piYuvExt[area.compID][PRED_BUF_FILTERED];

  setReferenceArrayLengths( area );

  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );
  // ----- Step 2: filtered reference samples -----
  if( bFilterRefSamples )
  {
    xFilterReferenceSamples( refBufUnfiltered, refBufFiltered, area, *cs.sps , cu.multiRefIdx() );
  }
}

void IntraPrediction::initIntraPatternChTypeISP(const CodingUnit& cu, const CompArea& area, PelBuf& recBuf)
{
  const CodingStructure& cs = *cu.cs;

  const Position &posLT = area.pos();
  bool isLeftAvail  = nullptr != cs.getCURestricted( posLT.offset( -1, 0 ), cu, CH_L, posLT.x == cu.lx() ? cu.left : &cu );
  bool isAboveAvail = nullptr != cs.getCURestricted( posLT.offset( 0, -1 ), cu, CH_L, posLT.y == cu.ly() ? cu.left : &cu );

  // ----- Step 1: unfiltered reference samples -----
  if( cu.blocks[area.compID].x == area.x && cu.blocks[area.compID].y == area.y )
  {
    Pel* refBufUnfiltered = m_piYuvExt[area.compID][PRED_BUF_UNFILTERED];
    // With the first subpartition all the CU reference samples are fetched at once in a single call to xFillReferenceSamples
    if( cu.ispMode() == HOR_INTRA_SUBPARTITIONS )
    {
      m_leftRefLength = cu.Y().height << 1;
      m_topRefLength  = cu.Y().width + area.width;
    }
    else //if (cu.ispMode() == VER_INTRA_SUBPARTITIONS)
    {
      m_leftRefLength = cu.Y().height + area.height;
      m_topRefLength  = cu.Y().width << 1;
    }

    const int srcStride = m_topRefLength + 1;
    const int srcHStride = m_leftRefLength + 1;

    m_pelBufISP[0] = m_pelBufISPBase[0] = PelBuf(m_piYuvExt[area.compID][PRED_BUF_UNFILTERED], srcStride, srcHStride);
    m_pelBufISP[1] = m_pelBufISPBase[1] = PelBuf(m_piYuvExt[area.compID][PRED_BUF_FILTERED], srcStride, srcHStride);

    xFillReferenceSamples(cs.picture->getRecoBuf(cu.Y()), refBufUnfiltered, cu.Y(), cu);

    // After having retrieved all the CU reference samples, the number of reference samples is now adjusted for the current subpartition
    m_topRefLength = cu.blocks[area.compID].width + area.width;
    m_leftRefLength = cu.blocks[area.compID].height + area.height;
  }
  else
  {
    //Now we only need to fetch the newly available reconstructed samples from the previously coded TU
    Position tuPos = area;
    tuPos.relativeTo(cu.Y());
    m_pelBufISP[0] = m_pelBufISPBase[0].subBuf(tuPos, area.size());
    m_pelBufISP[1] = m_pelBufISPBase[1].subBuf(tuPos, area.size());

    PelBuf& dstBuf = m_pelBufISP[0];

    m_topRefLength = cu.blocks[area.compID].width + area.width;
    m_leftRefLength = cu.blocks[area.compID].height + area.height;

    const int predSizeHor = m_topRefLength;
    const int predSizeVer = m_leftRefLength;
    if (cu.ispMode() == HOR_INTRA_SUBPARTITIONS)
    {
      Pel* src = recBuf.bufAt(0, -1);
      Pel* dst = dstBuf.bufAt(1, 0);
      for (int i = 0; i < area.width; i++)
      {
        dst[i] = src[i];
      }
      Pel sample = src[area.width - 1];
      dst += area.width;
      for (int i = 0; i < predSizeHor - area.width; i++)
      {
        dst[i] = sample;
      }
      if (!isLeftAvail) //if left is not avaible, then it is necessary to fetch these samples for each subpartition
      {
        Pel* dst = dstBuf.bufAt(0, 0);
        Pel  sample = src[0];
        for (int i = 0; i < predSizeVer + 1; i++)
        {
          *dst = sample;
          dst += dstBuf.stride;
        }
      }
    }
    else
    {
      Pel* src = recBuf.bufAt(-1, 0);
      Pel* dst = dstBuf.bufAt(0, 1);
      for (int i = 0; i < area.height; i++)
      {
        *dst = *src;
        src += recBuf.stride;
        dst += dstBuf.stride;
      }
      Pel sample = src[-recBuf.stride];
      for (int i = 0; i < predSizeVer - area.height; i++)
      {
        *dst = sample;
        dst += dstBuf.stride;
      }

      if (!isAboveAvail) //if above is not avaible, then it is necessary to fetch these samples for each subpartition
      {
        Pel* dst = dstBuf.bufAt(0, 0);
        Pel  sample = recBuf.at(-1, 0);
        for (int i = 0; i < predSizeHor + 1; i++)
        {
          dst[i] = sample;
        }
      }
    }
  }
}

void IntraPrediction::xFillReferenceSamples( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu ) const
{
  const ChannelType      chType = toChannelType( area.compID );
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int multiRefIdx         = (area.compID == COMPONENT_Y) ? cu.multiRefIdx() : 0;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
  const int  predSize           = m_topRefLength;
  const int  predHSize          = m_leftRefLength;
  const int  predStride         = predSize + 1 + multiRefIdx;

  const int  unitWidth          = tuWidth  <= 2 && cu.ispMode() && isLuma(area.compID) ? tuWidth  : pcv.minCUWidth  >> getComponentScaleX(area.compID, sps.getChromaFormatIdc());
  const int  unitHeight         = tuHeight <= 2 && cu.ispMode() && isLuma(area.compID) ? tuHeight : pcv.minCUHeight >> getComponentScaleY(area.compID, sps.getChromaFormatIdc());

  const int  totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported" );

  // ----- Step 1: analyze neighborhood -----
  const Position posLT          = area.pos();

  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset( neighborFlags, 0, totalUnits );

  const CodingUnit *aboveLeftCu = cu.cs->getCURestricted( posLT.offset( -1, -1 ), cu, chType, cu.left ? cu.left : cu.above );
  neighborFlags[totalLeftUnits] = !!aboveLeftCu;
  int numIntraNeighbor = aboveLeftCu ? 1 : 0;
  numIntraNeighbor    += isAboveAvailable( cu, chType, posLT, numAboveUnits + numAboveRightUnits, unitWidth,  neighborFlags + totalLeftUnits + 1, area.y == cu.blocks[area.compID].y ? cu.above : &cu );
  numIntraNeighbor    += isLeftAvailable ( cu, chType, posLT, numLeftUnits  + numLeftBelowUnits,  unitHeight, neighborFlags + totalLeftUnits - 1, area.x == cu.blocks[area.compID].x ? cu.left  : &cu, tuHeight );

  // ----- Step 2: fill reference samples (depending on neighborhood) -----
  CHECK((predHSize + 1) * predStride > m_iYuvExtSize, "Reference sample area not supported");

  const Pel*  srcBuf    = recoBuf.buf;
  const ptrdiff_t srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth( chType ) - 1);

  if( numIntraNeighbor == 0 )
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize + multiRefIdx; j++) { ptrDst[j] = valueDC; }
    for (int i = 1; i <= predHSize + multiRefIdx; i++) { ptrDst[i*predStride] = valueDC; }
  }
  else if( numIntraNeighbor == totalUnits )
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    for (int j = 0; j <= predSize + multiRefIdx; j++) { ptrDst[j] = ptrSrc[j]; }
    ptrSrc = srcBuf - multiRefIdx * srcStride - (1 + multiRefIdx);
    for (int i = 1; i <= predHSize + multiRefIdx; i++) { ptrDst[i*predStride] = *(ptrSrc); ptrSrc += srcStride; }
  }
  else // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    ptrDst = refBufUnfiltered;
    if (neighborFlags[totalLeftUnits])
    {
      ptrDst[0] = ptrSrc[0];
      for (int i = 1; i <= multiRefIdx; i++)
      {
        ptrDst[i] = ptrSrc[i];
        ptrDst[i*predStride] = ptrSrc[i*srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += (1 + multiRefIdx) * srcStride;
    ptrDst += (1 + multiRefIdx) * predStride;
    for (int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx--)
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i*predStride] = ptrSrc[i*srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight * predStride;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = (predHSize % unitHeight == 0) ? unitHeight : predHSize % unitHeight;
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i*predStride] = ptrSrc[i*srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
    ptrDst = refBufUnfiltered + 1 + multiRefIdx;
    for (int unitIdx = totalLeftUnits + 1; unitIdx < totalUnits - 1; unitIdx++)
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = (predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth;
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = 0;
      int firstAvailCol = 0;
      if (firstAvailUnit < totalLeftUnits)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + multiRefIdx;
      }
      else if (firstAvailUnit == totalLeftUnits)
      {
        firstAvailRow = multiRefIdx;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - 1) * unitWidth + 1 + multiRefIdx;
      }
      const Pel firstAvailSample = ptrDst[firstAvailCol + firstAvailRow * predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize + multiRefIdx;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i*predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit]) // samples not available
      {
        // last available sample
        int lastAvailRow = 0;
        int lastAvailCol = 0;
        if (lastAvailUnit < totalLeftUnits)
        {
          lastAvailRow = (totalLeftUnits - lastAvailUnit - 1) * unitHeight + multiRefIdx + 1;
        }
        else if (lastAvailUnit == totalLeftUnits)
        {
          lastAvailCol = multiRefIdx;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits) * unitWidth + multiRefIdx;
        }
        const Pel lastAvailSample = ptrDst[lastAvailCol + lastAvailRow * predStride];

        // fill current unit with last available sample
        if (currUnit < totalLeftUnits)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i*predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits)
        {
          for (int i = 1; i < multiRefIdx + 1; i++)
          {
            ptrDst[i*predStride] = lastAvailSample;
          }
          for (int j = 0; j < multiRefIdx + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit = (currUnit == totalUnits - 1) ? ((predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth) : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
}
  
}

void IntraPrediction::xFilterReferenceSamples( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps, int multiRefIdx, ptrdiff_t stride ) const
{
  if (area.compID != COMPONENT_Y)
  {
    multiRefIdx = 0;
  }
  const int       predSize   = m_topRefLength  + multiRefIdx;
  const int       predHSize  = m_leftRefLength + multiRefIdx;
  const ptrdiff_t predStride = stride == 0 ? predSize + 1 : stride;



  // Regular reference sample filter
  const Pel *piSrcPtr  = refBufUnfiltered + (predStride * predHSize); // bottom left
        Pel *piDestPtr = refBufFiltered   + (predStride * predHSize); // bottom left

  // bottom left (not filtered)
  *piDestPtr = *piSrcPtr;
  piDestPtr -= predStride;
  piSrcPtr  -= predStride;
  //left column (bottom to top)
  for( int i = 1; i < predHSize; i++, piDestPtr -= predStride, piSrcPtr -= predStride)
  {
    *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[-predStride] + 2) >> 2;
  }
  //top-left
  *piDestPtr = (piSrcPtr[predStride] + 2 * piSrcPtr[0] + piSrcPtr[1] + 2) >> 2;
  piDestPtr++;
  piSrcPtr++;
  //top row (left-to-right)
  for( uint32_t i=1; i < predSize; i++, piDestPtr++, piSrcPtr++ )
  {
    *piDestPtr = (piSrcPtr[1] + 2 * piSrcPtr[0] + piSrcPtr[-1] + 2) >> 2;
  }
  // top right (not filtered)
  *piDestPtr=*piSrcPtr;
}

bool IntraPrediction::getUseFilterRef( const int predMode, const int dirMode )
{
  static const int angTable[32]    = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };

  const int     intraPredAngleMode = (predMode >= DIA_IDX) ? predMode - VER_IDX : -(predMode - HOR_IDX);

  const int     absAngMode         = abs(intraPredAngleMode);
  const int     absAng             = angTable  [absAngMode];

  return 0 == (absAng & 0x1F);
}

bool IntraPrediction::useFilteredIntraRefSamples( const ComponentID &compID, const PredictionUnit &pu, const UnitArea &tuArea )
{
  //const SPS         &sps    = *pu.cs->sps;
  const ChannelType  chType = toChannelType( compID );

  // high level conditions
  //if( sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag() )  { return false; }
  //if( !isLuma( chType ) )                                           { return false; }
  //if( pu.ispMode() && isLuma(compID) )                              { return false; }
  //if( CU::isMIP( pu, chType ) )                                     { return false; }
  if( pu.multiRefIdx() )                                            { return false; }
  if( pu.bdpcmMode() )                                              { return false; }

  // pred. mode related conditions
  const int dirMode = PU::getFinalIntraMode( pu, chType );
  int predMode = getWideAngle(tuArea.blocks[compID].width, tuArea.blocks[compID].height, dirMode);
  if (dirMode == DC_IDX)                                            { return false; }
  if (dirMode == PLANAR_IDX)
  {
    return tuArea.blocks[compID].area() > 32 ? true : false;
  }

  bool filterFlag = false;
  {
    const int diff = std::min<int>( abs( predMode - HOR_IDX ), abs( predMode - VER_IDX ) );
    int log2Size = ( ( getLog2( tuArea.blocks[compID].width ) + getLog2( tuArea.blocks[compID].height ) ) >> 1 );
    CHECKD( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
    filterFlag = (diff > m_aucIntraFilter[chType][log2Size]);
  }

  if (filterFlag)
  {
    const bool isRefFilter = getUseFilterRef( predMode, dirMode );
    CHECKD( tuArea.blocks[compID].width * tuArea.blocks[compID].height <= 32, "DCT-IF interpolation filter is always used for 4x4, 4x8, and 8x4 luma CB" );
    return isRefFilter;
  }
  else
  {
    return false;
  }
}


bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT)
{
  const CodingStructure& cs       = *cu.cs;
  const Position refPos           = posLT.offset( -1, -1 );
  const CodingUnit* pcCUAboveLeft = cs.getCURestricted( refPos, cu, chType, cu.left ? cu.left : cu.above );

  return pcCUAboveLeft ? true : false;
}

int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags, const CodingUnit* pcCUAbove)
{
  const CodingStructure& cs = *cu.cs;
  bool *pbValidFlags        = bValidFlags;
  int iNumIntra             = 0;
  int maxDx                 = uiNumUnitsInPU * unitWidth;
  int rightXAbove           = pcCUAbove ? ( pcCUAbove->blocks[chType].x + pcCUAbove->blocks[chType].width ) : -1;
  Position refPos           = posLT.offset( 0, -1 );

  for( uint32_t dx = 0; dx < maxDx; dx += unitWidth, refPos.x += unitWidth )
  {
    if( !pcCUAbove || refPos.x >= rightXAbove )
    {
      pcCUAbove   = cs.getCURestricted( refPos, cu, chType, pcCUAbove ? nullptr : cu.above );
      rightXAbove = pcCUAbove ? ( pcCUAbove->blocks[chType].x + pcCUAbove->blocks[chType].width ) : -1;
    }

    if( pcCUAbove )
    {
      iNumIntra++;
      *pbValidFlags++ = true;
    }
    else
    {
      return iNumIntra;
    }
  }

  return iNumIntra;
}

int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags, const CodingUnit* pcCULeft, int tuHeight)
{
  const CodingStructure& cs = *cu.cs;
  bool *pbValidFlags        = bValidFlags;
  int iNumIntra             = 0;
  int maxDy                 = uiNumUnitsInPU * unitHeight;
  int bottomYLeft           = pcCULeft ? ( pcCULeft->blocks[chType].y + pcCULeft->blocks[chType].height ) : -1;
  Position refPos           = posLT.offset( -1, 0 );

  bool check    = false;
  int  bttmYlft = 0;

  if( pcCULeft && *pcCULeft == cu && !cu.ispMode() )
  {
    bttmYlft = posLT.y + tuHeight - 1;
    check = true;
  }

  bool avail = true;
  for( uint32_t dy = 0; dy < maxDy; dy += unitHeight, refPos.y += unitHeight )
  {
    if( !pcCULeft || refPos.y >= bottomYLeft )
    {
      pcCULeft    = cs.getCURestricted( refPos, cu, chType, pcCULeft ? nullptr : cu.left );
      bottomYLeft = pcCULeft ? ( pcCULeft->blocks[chType].y + pcCULeft->blocks[chType].height ) : -1;
    }

    if( check && refPos.y >= bttmYlft )
    {
      //we are inside the same cu with more than one tu and the bottom left tu is not reconstructed yet
      avail = false;
    }
    if( pcCULeft && avail )
    {
      iNumIntra++;
      *pbValidFlags-- = true;
    }
    else
    {
      return iNumIntra;
    }
  }

  return iNumIntra;
}

int isAboveRightAvailable( const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  bool *pbValidFlags        = bValidFlags;
  int iNumIntra             = 0;
  int maxDx                 = uiNumUnitsInPU * unitWidth;
  int rightXAbove           = -1;
  Position refPos           = posRT.offset( unitWidth, -1 );
  const CodingUnit *pcCUAbove = nullptr;

  for( uint32_t dx = 0; dx < maxDx; dx += unitWidth, refPos.x += unitWidth )
  {
    if( !pcCUAbove || refPos.x >= rightXAbove )
    {
      pcCUAbove   = cs.getCURestricted( refPos, cu, chType, pcCUAbove ? nullptr : cu.above );
      rightXAbove = pcCUAbove ? ( pcCUAbove->blocks[chType].x + pcCUAbove->blocks[chType].width ) : -1;
    }

    if( pcCUAbove )
    {
      iNumIntra++;
      *pbValidFlags++ = true;
    }
    else
    {
      return iNumIntra;
    }
  }
  return iNumIntra;
}

int isBelowLeftAvailable( const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;
  bool *pbValidFlags        = bValidFlags;
  int iNumIntra             = 0;
  int maxDy                 = uiNumUnitsInPU * unitHeight;
  int bottomYLeft           = -1;
  Position refPos           = posLB.offset( -1, unitHeight );
  const CodingUnit *pcCULeft = nullptr;

  bool avail = true;
  for( uint32_t dy = 0; dy < maxDy; dy += unitHeight, refPos.y += unitHeight )
  {
    if( !pcCULeft || refPos.y >= bottomYLeft )
    {
      pcCULeft    = cs.getCURestricted( refPos, cu, chType, pcCULeft ? nullptr : cu.left );
      bottomYLeft = pcCULeft ? ( pcCULeft->blocks[chType].y + pcCULeft->blocks[chType].height ) : -1;
    }

    if( pcCULeft && avail )
    {
      iNumIntra++;
      *pbValidFlags-- = true;
    }
    else
    {
      return iNumIntra;
    }
  }

  return iNumIntra;
}
// LumaRecPixels
void IntraPrediction::xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea)
{
  int iDstStride = 0;
  Pel* pDst0 = 0;
  int curChromaMode = pu.intraDir[1];
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    iDstStride = 2 * MAX_CU_SIZE + 1;
    pDst0      = m_pMdlmTemp + iDstStride + 1;
  }
  else
  {
    iDstStride = MAX_CU_SIZE + 1;
    pDst0      = m_piTemp + iDstStride + 1; //MMLM_SAMPLE_NEIGHBOR_LINES;
  }
  //assert 420 chroma subsampling
  CompArea lumaArea = CompArea( COMPONENT_Y, chromaArea.lumaPos( pu.chromaFormat),
                                recalcSize( pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size() ) );//needed for correct pos/size (4x4 Tus)


  CHECK( lumaArea.width  == chromaArea.width  && CHROMA_444 != pu.chromaFormat, "" );
  CHECK( lumaArea.height == chromaArea.height && CHROMA_444 != pu.chromaFormat && CHROMA_422 != pu.chromaFormat, "" );

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  CPelBuf Src = pu.cs->picture->getRecoBuf( lumaArea );
  Pel const* pRecSrc0   = Src.bufAt( 0, 0 );
  ptrdiff_t iRecStride  = Src.stride;
  int logSubWidthC  = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
  int logSubHeightC = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.chromaFormat);


  ptrdiff_t iRecStride2 = iRecStride << logSubHeightC;    // TODO: really Height here? not Width?
  const int mult        =          1 << logSubWidthC ;
  const CodingUnit& lumaCU = isChroma( pu.chType() ) ? *pu.cs->getCU( lumaArea.pos(), CH_L ) : pu;
  const CodingUnit&     cu = pu;

  const CompArea& area = isChroma( pu.chType() ) ? chromaArea : lumaArea;

  const uint32_t uiTuWidth  = area.width;
  const uint32_t uiTuHeight = area.height;

  int iBaseUnitSize = ( 1 << MIN_CU_LOG2 );

  const int  iUnitWidth       = iBaseUnitSize >> getComponentScaleX( area.compID, pu.chromaFormat );
  const int  iUnitHeight      = iBaseUnitSize >> getComponentScaleY( area.compID, pu.chromaFormat );
  const int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const int  iAboveUnits      = iTUWidthInUnits;
  const int  iLeftUnits       = iTUHeightInUnits;
  const int  chromaUnitWidth  = iBaseUnitSize >> getComponentScaleX(COMPONENT_Cb, pu.chromaFormat);
  const int  chromaUnitHeight = iBaseUnitSize >> getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);
  const int  topTemplateSampNum  = 2 * uiCWidth; // for MDLM, the number of template samples is 2W or 2H.
  const int  leftTemplateSampNum = 2 * uiCHeight;
  CHECKD( !( m_topRefLength >= topTemplateSampNum ), "Error!" );
  CHECKD( !( m_leftRefLength >= leftTemplateSampNum ), "Error!" );
  const int  totalAboveUnits = (topTemplateSampNum + (chromaUnitWidth - 1)) / chromaUnitWidth;
  const int  totalLeftUnits  = (leftTemplateSampNum + (chromaUnitHeight - 1)) / chromaUnitHeight;
  const int  totalUnits      = totalLeftUnits + totalAboveUnits + 1;
  const int  aboveRightUnits = totalAboveUnits - iAboveUnits;
  const int  leftBelowUnits  = totalLeftUnits - iLeftUnits;

  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits  = 0;
  bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(bNeighborFlags, 0, totalUnits);

  const CodingUnit& chromaCU = isChroma( pu.chType() ) ? cu : lumaCU;

  int availlableUnit = isLeftAvailable( chromaCU, toChannelType( area.compID ), area.pos(), iLeftUnits, iUnitHeight, bNeighborFlags + iLeftUnits + leftBelowUnits - 1, cu.left );

  const bool bLeftAvaillable = availlableUnit == iTUHeightInUnits;

  availlableUnit = isAboveAvailable( chromaCU, toChannelType( area.compID ), area.pos(), iAboveUnits, iUnitWidth, bNeighborFlags + iLeftUnits + leftBelowUnits + 1, cu.above );

  const bool bAboveAvaillable = availlableUnit == iTUWidthInUnits;

  bool checkIt = true;
  Position posLB = area.bottomLeftComp( pu.chromaFormat, area.compID );
  if( chromaCU.blocks[pu.chType()].contains( posLB.offset( -1, 1 ) ) )
  {
    checkIt = false;
  }
  if( bLeftAvaillable && checkIt ) // if left is not available, then the below left is not available
  {
    avaiLeftBelowUnits = isBelowLeftAvailable( chromaCU, toChannelType( area.compID ), area.bottomLeftComp( pu.chromaFormat, area.compID ), leftBelowUnits, iUnitHeight, bNeighborFlags + leftBelowUnits - 1 );
  }

  if( bAboveAvaillable ) // if above is not available, then  the above right is not available.
  {
    avaiAboveRightUnits = isAboveRightAvailable( chromaCU, toChannelType( area.compID ), area.topRightComp( pu.chromaFormat, area.compID ), aboveRightUnits, iUnitWidth, bNeighborFlags + iLeftUnits + leftBelowUnits + iAboveUnits + 1 );
  }

  Pel*       pDst  = nullptr;
  Pel const* piSrc = nullptr;

  bool isFirstRowOfCtu            = ( lumaArea.y & ((pu.cs->sps)->getCTUSize() - 1) ) == 0;
  const ptrdiff_t strOffset       = ( CHROMA_444 == pu.chromaFormat ) ? 0 : iRecStride;

  int c0_3tap = 2, c1_3tap = 1, c2_3tap = 1,                                        offset_3tap = 2, shift_3tap = 2; //sum = 4
  int c0_5tap = 1, c1_5tap = 4, c2_5tap = 1, c3_5tap = 1, c4_5tap = 1,              offset_5tap = 4, shift_5tap = 3; //sum = 8
  int c0_6tap = 2, c1_6tap = 1, c2_6tap = 1, c3_6tap = 2, c4_6tap = 1, c5_6tap = 1, offset_6tap = 4, shift_6tap = 3; //sum = 8

  switch (pu.chromaFormat)
  {
    case CHROMA_422: //overwrite filter coefficient values for 422
      c0_3tap = 2, c1_3tap = 1, c2_3tap = 1,                                        offset_3tap = 2, shift_3tap = 2; //sum = 4
      c0_5tap = 0, c1_5tap = 1, c2_5tap = 0, c3_5tap = 0, c4_5tap = 0,              offset_5tap = 0, shift_5tap = 0; //sum = 1
      c0_6tap = 2, c1_6tap = 1, c2_6tap = 1, c3_6tap = 0, c4_6tap = 0, c5_6tap = 0, offset_6tap = 2, shift_6tap = 2; //sum = 4
      break;

    case CHROMA_444:  //overwrite filter coefficient values for 444
      c0_3tap = 1, c1_3tap = 0, c2_3tap = 0,                                        offset_3tap = 0, shift_3tap = 0; //sum = 1
      c0_5tap = 0, c1_5tap = 1, c2_5tap = 0, c3_5tap = 0, c4_5tap = 0,              offset_5tap = 0, shift_5tap = 0; //sum = 1
      c0_6tap = 1, c1_6tap = 0, c2_6tap = 0, c3_6tap = 0, c4_6tap = 0, c5_6tap = 0, offset_6tap = 0, shift_6tap = 0; //sum = 1
      break;

    default:
      break;
  }

  if( bAboveAvaillable )
  {
    pDst  = pDst0    - iDstStride;
    int addedAboveRight = 0;
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
    {
      addedAboveRight = avaiAboveRightUnits*chromaUnitWidth;
    }
    for (int i = 0; i < uiCWidth + addedAboveRight; i++)
    {
      if (isFirstRowOfCtu)
      {
        piSrc = pRecSrc0 - iRecStride;

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + addedAboveRight - 1 + logSubWidthC))
        {
          pDst[i] = (piSrc[mult * i] * c0_3tap + piSrc[mult * i] * c1_3tap + piSrc[mult * i + 1] * c2_3tap + offset_3tap) >> shift_3tap;
        }
        else
        {
          pDst[i] = (piSrc[mult * i] * c0_3tap + piSrc[mult * i - 1] * c1_3tap + piSrc[mult * i + 1] * c2_3tap + offset_3tap) >> shift_3tap;
        }
      }
      else if( pu.cs->sps->getCclmCollocatedChromaFlag() )
      {
        piSrc = pRecSrc0 - iRecStride2;

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + addedAboveRight - 1 + logSubWidthC))
        {
          pDst[i] = (piSrc[mult * i - strOffset] * c0_5tap
                  +  piSrc[mult * i]             * c1_5tap + piSrc[mult * i] * c2_5tap + piSrc[mult * i + 1] * c3_5tap
                  +  piSrc[mult * i + strOffset] * c4_5tap
                  +  offset_5tap) >> shift_5tap;
        }
        else
        {
          pDst[i] = (piSrc[mult * i - strOffset] * c0_5tap
                  +  piSrc[mult * i]             * c1_5tap + piSrc[mult * i - 1] * c2_5tap + piSrc[mult * i + 1] * c3_5tap
                  +  piSrc[mult * i + strOffset] * c4_5tap
                  +  offset_5tap) >> shift_5tap;
        }
      }
      else
      {
        piSrc = pRecSrc0 - iRecStride2;

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + addedAboveRight - 1 + logSubWidthC))
        {
          pDst[i] = ((piSrc[mult * i]            * c0_6tap + piSrc[mult * i]             * c1_6tap + piSrc[mult * i + 1]             * c2_6tap)
                  + (piSrc[mult * i + strOffset] * c3_6tap + piSrc[mult * i + strOffset] * c4_6tap + piSrc[mult * i + 1 + strOffset] * c5_6tap)
                  + offset_6tap) >> shift_6tap;
        }
        else
        {
          pDst[i] = ((piSrc[mult * i]            * c0_6tap + piSrc[mult * i - 1]             * c1_6tap + piSrc[mult * i + 1]             * c2_6tap)
                  + (piSrc[mult * i + strOffset] * c3_6tap + piSrc[mult * i - 1 + strOffset] * c4_6tap + piSrc[mult * i + 1 + strOffset] * c5_6tap)
                  + offset_6tap) >> shift_6tap;
        }
      }
    }
  }

  if( bLeftAvaillable )
  {
    pDst  = pDst0    - 1;

    piSrc = pRecSrc0 - 2 - logSubWidthC;

    int addedLeftBelow = 0;
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
    {
      addedLeftBelow = avaiLeftBelowUnits*chromaUnitHeight;
    }
    for (int j = 0; j < uiCHeight + addedLeftBelow; j++)
    {
      if( pu.cs->sps->getCclmCollocatedChromaFlag() )
      {
        if ((j == 0 && !bAboveAvaillable) || (j == uiCHeight + addedLeftBelow - 1 + logSubWidthC))
        {
          pDst[0] = ( piSrc[1            ] * c0_5tap
                    + piSrc[1            ] * c1_5tap + piSrc[0] * c2_5tap + piSrc[2] * c3_5tap
                    + piSrc[1 + strOffset] * c4_5tap
                    + offset_5tap ) >> shift_5tap;
        }
        else
        {
          pDst[0] = ( piSrc[1 - strOffset] * c0_5tap
                    + piSrc[1            ] * c1_5tap + piSrc[0] * c2_5tap + piSrc[2] * c3_5tap
                    + piSrc[1 + strOffset] * c4_5tap
                    + offset_5tap ) >> shift_5tap;
        }
      }
      else
      {
        pDst[0] = ((piSrc[1]             * c0_6tap + piSrc[0]         * c1_6tap + piSrc[2]             * c2_6tap)
                +  (piSrc[1 + strOffset] * c3_6tap + piSrc[strOffset] * c4_6tap + piSrc[2 + strOffset] * c5_6tap)
                +   offset_6tap) >> shift_6tap;
      }

      piSrc += iRecStride2;
      pDst  += iDstStride;
    }
  }

  if( pu.cs->sps->getCclmCollocatedChromaFlag() )
  {
    // TODO: unroll loop
    for( int j = 0; j < uiCHeight; j++ )
    {
      for( int i = 0; i < uiCWidth; i++ )
      {
        if( i == 0 && !bLeftAvaillable )
        {
          if( j == 0 && !bAboveAvaillable )
          {
            pDst0[i] = (pRecSrc0[mult * i] * c0_5tap
                     +  pRecSrc0[mult * i] * c1_5tap + pRecSrc0[mult * i] * c2_5tap + pRecSrc0[mult * i + 1] * c3_5tap
                     +  pRecSrc0[mult * i + strOffset] * c4_5tap
                     +  offset_5tap) >> shift_5tap;
          }
          else
          {
            pDst0[i] = (pRecSrc0[mult * i - strOffset] * c0_5tap
                     +  pRecSrc0[mult * i] * c1_5tap + pRecSrc0[mult * i] * c2_5tap + pRecSrc0[mult * i + 1] * c3_5tap
                     +  pRecSrc0[mult * i + strOffset] * c4_5tap
                     +  offset_5tap) >> shift_5tap;
          }
        }
        else if( j == 0 && !bAboveAvaillable )
        {
          pDst0[i] = (pRecSrc0[mult * i] * c0_5tap
                   +  pRecSrc0[mult * i] * c1_5tap + pRecSrc0[mult * i - 1] * c2_5tap + pRecSrc0[mult * i + 1] * c3_5tap
                   +  pRecSrc0[mult * i + strOffset] * c4_5tap
                   +  offset_5tap) >> shift_5tap;
        }
        else
        {
          pDst0[i] = (pRecSrc0[mult * i - strOffset] * c0_5tap
                   +  pRecSrc0[mult * i]             * c1_5tap + pRecSrc0[mult * i - 1] * c2_5tap + pRecSrc0[mult * i + 1] * c3_5tap
                   +  pRecSrc0[mult * i + strOffset] * c4_5tap
                   +  offset_5tap) >> shift_5tap;
        }
      }
      pDst0    += iDstStride;
      pRecSrc0 += iRecStride2;
    }
    return;
  }




#define GET_LUMA_REC_PIX_INC   \
  pDst0 += iDstStride;         \
  pRecSrc0 += iRecStride2

#define GET_LUMA_REC_PIX_OP2(ADDR)                                                    \
  pDst0[ADDR] = (   pRecSrc0[( (ADDR) << logSubWidthC )    ]              * c0_6tap   \
                  + pRecSrc0[( (ADDR) << logSubWidthC ) + 1]              * c1_6tap   \
                  + pRecSrc0[( (ADDR) << logSubWidthC ) - 1]              * c2_6tap   \
                  + pRecSrc0[( (ADDR) << logSubWidthC )     + iRecStride] * c3_6tap   \
                  + pRecSrc0[( (ADDR) << logSubWidthC ) + 1 + iRecStride] * c4_6tap   \
                  + pRecSrc0[( (ADDR) << logSubWidthC ) - 1 + iRecStride] * c5_6tap   \
                  + offset_6tap ) >> shift_6tap

#define GET_LUMA_REC_PIX_OP1(ADDR)                                                        \
  pDst0[ADDR] = !(ADDR) ? (   pRecSrc0[( (ADDR) << logSubWidthC )    ]              * c0_6tap   \
                            + pRecSrc0[( (ADDR) << logSubWidthC ) + 1]              * c1_6tap   \
                            + pRecSrc0[( (ADDR) << logSubWidthC )]              * c2_6tap   \
                            + pRecSrc0[( (ADDR) << logSubWidthC )     + iRecStride] * c3_6tap   \
                            + pRecSrc0[( (ADDR) << logSubWidthC ) + 1 + iRecStride] * c4_6tap   \
                            + pRecSrc0[( (ADDR) << logSubWidthC ) + iRecStride] * c5_6tap   \
                            + offset_6tap ) >> shift_6tap                                       \
                        : GET_LUMA_REC_PIX_OP2(ADDR)
  
  int width  = uiCWidth;
  int height = uiCHeight;

GCC_WARNING_DISABLE_sequence_point
  if( bLeftAvaillable )
  {
    if( pu.chromaFormat == CHROMA_420 )
    {
      GetLumaRecPixel420( width, height, pRecSrc0, iRecStride, pDst0, iDstStride );
      //      SIZE_AWARE_PER_EL_OP( GET_LUMA_REC_PIX_OP2, GET_LUMA_REC_PIX_INC );
    }
    else  //TODO add SIMD for 422,444
    {
      SIZE_AWARE_PER_EL_OP( GET_LUMA_REC_PIX_OP2, GET_LUMA_REC_PIX_INC );
    }
  }
  else
  {
    SIZE_AWARE_PER_EL_OP( GET_LUMA_REC_PIX_OP1, GET_LUMA_REC_PIX_INC );
  }
  GCC_WARNING_RESET
}

#undef GET_LUMA_REC_PIX_INC
#undef GET_LUMA_REC_PIX_OP1
#undef GET_LUMA_REC_PIX_OP2
#undef SIZE_AWARE_PER_EL_OP

void IntraPrediction::xGetLMParameters(const PredictionUnit &pu, const ComponentID compID,
                                              const CompArea &chromaArea,
                                              int &a, int &b, int &iShift)
{
  CHECK(compID == COMPONENT_Y, "");

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  const Position posLT = chromaArea;

  const CodingUnit      &cu =  pu;
  const CodingStructure &cs = *cu.cs;

  const SPS &        sps           = *cs.sps;
  const uint32_t     tuWidth     = chromaArea.width;
  const uint32_t     tuHeight    = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const int baseUnitSize = 1 << MIN_CU_LOG2;
  const int unitWidth    = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);
  const int unitHeight   = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);

  const int tuWidthInUnits  = tuWidth / unitWidth;
  const int tuHeightInUnits = tuHeight / unitHeight;
  const int aboveUnits      = tuWidthInUnits;
  const int leftUnits       = tuHeightInUnits;
  int topTemplateSampNum = 2 * cWidth; // for MDLM, the template sample number is 2W or 2H;
  int leftTemplateSampNum = 2 * cHeight;
  CHECKD( !(m_topRefLength >= topTemplateSampNum),   "Error!" );
  CHECKD( !(m_leftRefLength >= leftTemplateSampNum), "Error!" );
  int totalAboveUnits = (topTemplateSampNum + (unitWidth - 1)) / unitWidth;
  int totalLeftUnits = (leftTemplateSampNum + (unitHeight - 1)) / unitHeight;
  int totalUnits = totalLeftUnits + totalAboveUnits + 1;
  int aboveRightUnits = totalAboveUnits - aboveUnits;
  int leftBelowUnits = totalLeftUnits - leftUnits;
  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  int avaiAboveUnits = 0;
  int avaiLeftUnits = 0;

  int curChromaMode = pu.intraDir[1];
  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  bool aboveAvailable, leftAvailable;

  int availableUnit = isAboveAvailable( cu, CHANNEL_TYPE_CHROMA, posLT, aboveUnits, unitWidth, neighborFlags + leftUnits + leftBelowUnits + 1, cu.above );
  aboveAvailable = availableUnit == tuWidthInUnits;

  availableUnit = isLeftAvailable( cu, CHANNEL_TYPE_CHROMA, posLT, leftUnits, unitHeight, neighborFlags + leftUnits + leftBelowUnits - 1, cu.left );
  leftAvailable = availableUnit == tuHeightInUnits;

  bool checkIt = true;
  Position posLB = chromaArea.bottomLeftComp( pu.chromaFormat, chromaArea.compID );
  if( leftAvailable && cu.blocks[pu.chType()].contains( posLB.offset( -1, 1 ) ) )
  {
    checkIt = false;
    avaiLeftUnits = tuHeightInUnits;
  }

  if (leftAvailable && checkIt) // if left is not available, then the below left is not available
  {
    avaiLeftUnits = tuHeightInUnits;
    avaiLeftBelowUnits = isBelowLeftAvailable( cu, CHANNEL_TYPE_CHROMA, chromaArea.bottomLeftComp( nChromaFormat, chromaArea.compID ), leftBelowUnits, unitHeight, neighborFlags + leftBelowUnits - 1 );
  }
  if (aboveAvailable) // if above is not available, then  the above right is not available.
  {
    avaiAboveUnits = tuWidthInUnits;
    avaiAboveRightUnits = isAboveRightAvailable( cu, CHANNEL_TYPE_CHROMA, chromaArea.topRightComp( nChromaFormat, chromaArea.compID ), aboveRightUnits, unitWidth, neighborFlags + leftUnits + leftBelowUnits + aboveUnits + 1 );
  }
  Pel *srcColor0, *curChroma0;
  int  srcStride, curStride;

  PelBuf temp;
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    srcStride = 2 * MAX_CU_SIZE + 1;
    temp      = PelBuf(m_pMdlmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
    srcStride = MAX_CU_SIZE + 1;
    temp      = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  srcColor0 = temp.bufAt(0, 0);
  curChroma0 = getPredictorPtr(compID);

  curStride = m_topRefLength + 1;

  curChroma0 += curStride + 1;

  unsigned internalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  int minLuma[2] = {  MAX_INT, 0 };
  int maxLuma[2] = { -MAX_INT, 0 };

  Pel *src = srcColor0 - srcStride;
  Pel *cur = curChroma0 - curStride;
  int actualTopTemplateSampNum = 0;
  int actualLeftTemplateSampNum = 0;
  if (curChromaMode == MDLM_T_IDX)
  {
    leftAvailable = 0;
    avaiAboveRightUnits = avaiAboveRightUnits > (cHeight/unitWidth) ?  cHeight/unitWidth : avaiAboveRightUnits;
    actualTopTemplateSampNum = unitWidth*(avaiAboveUnits + avaiAboveRightUnits);
  }
  else if (curChromaMode == MDLM_L_IDX)
  {
    aboveAvailable = 0;
    avaiLeftBelowUnits = avaiLeftBelowUnits > (cWidth/unitHeight) ? cWidth/unitHeight : avaiLeftBelowUnits;
    actualLeftTemplateSampNum = unitHeight*(avaiLeftUnits + avaiLeftBelowUnits);
  }
  else if (curChromaMode == LM_CHROMA_IDX)
  {
    actualTopTemplateSampNum = cWidth;
    actualLeftTemplateSampNum = cHeight;
  }
  int startPos[2]; //0:Above, 1: Left
  int pickStep[2];

  int aboveIs4 = leftAvailable  ? 0 : 1;
  int leftIs4 =  aboveAvailable ? 0 : 1;

  startPos[0] = actualTopTemplateSampNum >> (2 + aboveIs4);
  pickStep[0] = std::max(1, actualTopTemplateSampNum >> (1 + aboveIs4));

  startPos[1] = actualLeftTemplateSampNum >> (2 + leftIs4);
  pickStep[1] = std::max(1, actualLeftTemplateSampNum >> (1 + leftIs4));

  Pel selectLumaPix[4] = { 0, 0, 0, 0 };
  Pel selectChromaPix[4] = { 0, 0, 0, 0 };

  int cntT, cntL;
  cntT = cntL = 0;
  int cnt = 0;
  if (aboveAvailable)
  {
    cntT = std::min(actualTopTemplateSampNum, (1 + aboveIs4) << 1);
    src = srcColor0 - srcStride;
    cur = curChroma0 - curStride;
    for (int pos = startPos[0]; cnt < cntT; pos += pickStep[0], cnt++)
    {
      selectLumaPix[cnt] = src[pos];
      selectChromaPix[cnt] = cur[pos];
    }
  }

  if (leftAvailable)
  {
    cntL = std::min(actualLeftTemplateSampNum, ( 1 + leftIs4 ) << 1 );
    src = srcColor0 - 1;
    cur = curChroma0 - 1;
    for (int pos = startPos[1], cnt = 0; cnt < cntL; pos += pickStep[1], cnt++)
    {
      selectLumaPix[cnt + cntT] = src[pos * srcStride];
      selectChromaPix[cnt+ cntT] = cur[pos * curStride];
    }
  }
  cnt = cntL + cntT;

  if (cnt == 2)
  {
    selectLumaPix[3] = selectLumaPix[0]; selectChromaPix[3] = selectChromaPix[0];
    selectLumaPix[2] = selectLumaPix[1]; selectChromaPix[2] = selectChromaPix[1];
    selectLumaPix[0] = selectLumaPix[1]; selectChromaPix[0] = selectChromaPix[1];
    selectLumaPix[1] = selectLumaPix[3]; selectChromaPix[1] = selectChromaPix[3];
  }

  int minGrpIdx[2] = { 0, 2 };
  int maxGrpIdx[2] = { 1, 3 };
  int *tmpMinGrp = minGrpIdx;
  int *tmpMaxGrp = maxGrpIdx;
  if (selectLumaPix[tmpMinGrp[0]] > selectLumaPix[tmpMinGrp[1]]) std::swap(tmpMinGrp[0], tmpMinGrp[1]);
  if (selectLumaPix[tmpMaxGrp[0]] > selectLumaPix[tmpMaxGrp[1]]) std::swap(tmpMaxGrp[0], tmpMaxGrp[1]);
  if (selectLumaPix[tmpMinGrp[0]] > selectLumaPix[tmpMaxGrp[1]]) std::swap(tmpMinGrp, tmpMaxGrp);       // TODO: really? not std::swap(tmpMinGrp[0], tmpMaxGrp[1]); ?
  if (selectLumaPix[tmpMinGrp[1]] > selectLumaPix[tmpMaxGrp[0]]) std::swap(tmpMinGrp[1], tmpMaxGrp[0]);

  minLuma[0] = (selectLumaPix  [tmpMinGrp[0]] + selectLumaPix  [tmpMinGrp[1]] + 1) >> 1;
  minLuma[1] = (selectChromaPix[tmpMinGrp[0]] + selectChromaPix[tmpMinGrp[1]] + 1) >> 1;
  maxLuma[0] = (selectLumaPix  [tmpMaxGrp[0]] + selectLumaPix  [tmpMaxGrp[1]] + 1) >> 1;
  maxLuma[1] = (selectChromaPix[tmpMaxGrp[0]] + selectChromaPix[tmpMaxGrp[1]] + 1) >> 1;

  if (leftAvailable || aboveAvailable)
  {
    int diff = maxLuma[0] - minLuma[0];
    if (diff > 0)
    {
      int diffC = maxLuma[1] - minLuma[1];
      int x = getLog2( diff );
      static const uint8_t DivSigTable[1 << 4] = {
        // 4bit significands - 8 ( MSB is omitted )
        0,  7,  6,  5,  5,  4,  4,  3,  3,  2,  2,  1,  1,  1,  1,  0
      };
      int normDiff = (diff << 4 >> x) & 15;
      int v = DivSigTable[normDiff] | 8;
      x += normDiff != 0;

      int y = getLog2( abs( diffC ) ) + 1;
      int add = 1 << y >> 1;
      a = (diffC * v + add) >> y;
      iShift = 3 + x - y;
      if ( iShift < 1 ) {
        iShift = 1;
        a = ( (a == 0)? 0: (a < 0)? -15 : 15 );   // a=Sign(a)*15
      }
      b = minLuma[1] - ((a * minLuma[0]) >> iShift);
    }
    else
    {
      a = 0;
      b = minLuma[1];
      iShift = 0;
    }
  }
  else
  {
    a = 0;

    b = 1 << (internalBitDepth - 1);

    iShift = 0;
  }
}

void IntraPrediction::initIntraMip( const PredictionUnit &pu, const CompArea &area )
{
  CHECK( area.width > MIP_MAX_WIDTH || area.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP" );

  // prepare input (boundary) data for prediction
//  CHECK( m_ipaParam.refFilterFlag, "ERROR: unfiltered refs expected for MIP" );
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  Pel *ptrSrc = getPredictorPtr( area.compID );
#else
  Pel *ptrSrc = getPredictorPtr( COMPONENT_Y );
#endif
  const int srcStride  = m_topRefLength  + 1; //TODO: check this if correct
  const int srcHStride = m_leftRefLength + 1;

#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  m_matrixIntraPred.prepareInputForPred( CPelBuf( ptrSrc, srcStride, srcHStride ), area, pu.slice->getSPS()->getBitDepth( toChannelType( area.compID ) ), area.compID );
#else
  m_matrixIntraPred.prepareInputForPred( CPelBuf( ptrSrc, srcStride, srcHStride ), area, pu.slice->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) );
#endif
}

void IntraPrediction::predIntraMip( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu )
{
#if !JVET_R0350_MIP_CHROMA_444_SINGLETREE
  CHECK( compId != COMPONENT_Y, "Error: chroma not supported" );
#endif
  CHECK( piPred.width > MIP_MAX_WIDTH || piPred.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP" );
  CHECK( piPred.width != (1 << getLog2(piPred.width)) || piPred.height != (1 << getLog2(piPred.height)), "Error: expecting blocks of size 2^M x 2^N" );

  // generate mode-specific prediction
#if JVET_R0350_MIP_CHROMA_444_SINGLETREE
  uint32_t modeIdx       = MAX_NUM_MIP_MODE;
  bool     transposeFlag = false;
  if( compId == COMPONENT_Y )
  {
    modeIdx       = pu.intraDir[CHANNEL_TYPE_LUMA];
    transposeFlag = pu.mipTransposedFlag();
  }
  else
  {
    const PredictionUnit &coLocatedLumaPU = PU::getCoLocatedLumaPU(pu);

    CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "Error: MIP is only supported for chroma with DM_CHROMA.");
    CHECK(!coLocatedLumaPU.mipFlag(), "Error: Co-located luma CU should use MIP.");

    modeIdx       = coLocatedLumaPU.intraDir[CHANNEL_TYPE_LUMA];
    transposeFlag = coLocatedLumaPU.mipTransposedFlag();
  }

  CHECK(modeIdx >= getNumModesMip(piPred), "Error: Wrong MIP mode index");

  const int bitDepth = pu.slice->getSPS()->getBitDepth( toChannelType( compId ) );
  m_matrixIntraPred.predBlock( piPred, modeIdx, piPred, transposeFlag, bitDepth, compId );
#else
  const int bitDepth = pu.slice->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA );
  m_matrixIntraPred.predBlock( piPred, pu.intraDir[CHANNEL_TYPE_LUMA], piPred, pu.mipTransposedFlag(), bitDepth );
#endif
}

//! \}
