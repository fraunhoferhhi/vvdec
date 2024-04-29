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

namespace vvdec
{

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

IntraPrediction::IntraPrediction() : m_currChromaFormat( NUM_CHROMA_FORMAT )
{
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
}

void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if (m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;

  std::fill_n( m_neighborSize, 3, 0 );
  m_lastCUidx = -1;

#if ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_X86 )
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



void IntraPrediction::predIntraAng( const ComponentID compID, PelBuf &piPred, const CodingUnit &cu, const bool useFilteredPredSamples )
{
  const ChannelType    channelType  = toChannelType( compID );
  const int            iWidth       = piPred.width;
  const int            iHeight      = piPred.height;
  const Size           cuSize       = Size( cu.blocks[compID].width, cu.blocks[compID].height );
  CHECK( CU::isMIP(cu, toChannelType(compID)), "We should not get here for MIP." );
  const uint32_t       uiDirMode    = isLuma( compID ) && cu.bdpcmMode() ? BDPCM_IDX : !isLuma(compID) && cu.bdpcmModeChroma() ? BDPCM_IDX : PU::getFinalIntraMode(cu, channelType);

  CHECKD( iWidth == 2, "Width of 2 is not supported" );

  const int     multiRefIdx = ( compID == COMPONENT_Y ) ? cu.multiRefIdx() : 0;
  const bool    useISP      = cu.ispMode() && isLuma( compID );
  const int     srcStride   = m_topRefLength  + 1 + multiRefIdx;
  const int     srcHStride  = m_leftRefLength + 1 + multiRefIdx;
  const ClpRng& clpRng      ( cu.slice->clpRng( compID ) );
        bool    doPDPC      = ( iWidth >= MIN_TB_SIZEY && iHeight >= MIN_TB_SIZEY ) && multiRefIdx == 0;

  const PelBuf& srcBuf = cu.ispMode() && isLuma(compID) ? getISPBuffer( useFilteredPredSamples ) : PelBuf(getPredictorPtr(compID, useFilteredPredSamples), srcStride, srcHStride);

  switch (uiDirMode)
  {
    case(PLANAR_IDX): xPredIntraPlanar(srcBuf, piPred, *cu.sps); break;
    case(DC_IDX):     xPredIntraDc    (srcBuf, piPred, channelType, false, multiRefIdx); break;
    case(BDPCM_IDX):  xPredIntraBDPCM(srcBuf, piPred, isLuma(compID) ? cu.bdpcmMode() : cu.bdpcmModeChroma(), clpRng); break;
    case(2):
    case(DIA_IDX):
    case(VDIA_IDX):
      if (getWideAngle(useISP ? cuSize.width : iWidth, useISP ? cuSize.height : iHeight, uiDirMode) == static_cast<int>(uiDirMode)) // check if uiDirMode is not wide-angle
      {
        xPredIntraAng(srcBuf, piPred, channelType, uiDirMode, clpRng, *cu.sps, multiRefIdx, useFilteredPredSamples, doPDPC, useISP, cuSize );
        break;
      }
    default:          xPredIntraAng(srcBuf, piPred, channelType, uiDirMode, clpRng, *cu.sps, multiRefIdx, useFilteredPredSamples, doPDPC, useISP, cuSize); break;
  }

  if( doPDPC && (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX ) )
  {
    if (iWidth>8)
      IntraPredSampleFilter16(srcBuf.buf,srcBuf.stride,piPred,uiDirMode,clpRng);
    else
      IntraPredSampleFilter8(srcBuf.buf,srcBuf.stride,piPred,uiDirMode,clpRng);
  }
}

void IntraPrediction::predIntraChromaLM( const ComponentID compID, PelBuf& piPred, const CodingUnit& cu, const CompArea& chromaArea, int intraDir )
{
  int  iLumaStride = 0;
  PelBuf Temp;
  if( (intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX) )
  {
    iLumaStride = 2 * MAX_TU_SIZE_FOR_PROFILE + 1;
    Temp = PelBuf( m_piYuvExt[1] + iLumaStride + 1, iLumaStride, Size( chromaArea ) );
  }
  else
  {
    iLumaStride = MAX_TU_SIZE_FOR_PROFILE + 1;
    Temp = PelBuf( m_piYuvExt[1] + iLumaStride + 1, iLumaStride, Size( chromaArea ) );
  }
  int a, b, iShift;
  xGetLMParameters( cu, compID, chromaArea, a, b, iShift );

  ////// final prediction
  piPred.copyFrom( Temp );
  piPred.linearTransform( a, iShift, b, true, cu.slice->clpRng( compID ) );
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

void IntraPrediction::predBlendIntraCiip( PelUnitBuf &predUnit, const CodingUnit &cu )
{
  int maxCompID = 1;

  if( isChromaEnabled( cu.chromaFormat ) && cu.chromaSize().width > 2 )
  {
    maxCompID = MAX_NUM_COMPONENT;
  }

  for( int currCompID = 0; currCompID < maxCompID; currCompID++ )
  {
    PelBuf&              pred   = predUnit.bufs[ currCompID ];
    const int            width  = pred.width;
    const int            height = pred.height;
    const ptrdiff_t      srcStride = width;
    const ptrdiff_t      dstStride = pred.stride;
    Pel*                 dstBuf    = pred.buf;
    const bool           isUseFilter = currCompID == 0 && IntraPrediction::useFilteredIntraRefSamples( COMPONENT_Y, cu, cu );
    Pel*                 srcBuf    = m_piYuvExt[!isUseFilter];
    PelBuf               srcAreaBuf( srcBuf, srcStride, width, height );

    {
      PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_INTRAPRED, *cu.cs, compID );

      initIntraPatternChType( cu.firstTU, cu.blocks[currCompID], isUseFilter );
      predIntraAng( ComponentID( currCompID ), srcAreaBuf, cu, isUseFilter );
    }

    CHECKD( width == 2, "Width of 2 is not supported" );

    const Position posBL = cu.Y().bottomLeft();
    const Position posTR = cu.Y().topRight();

    const CodingUnit* cuLeft  = cu.cs->getCURestricted( posBL.offset( -1, 0 ), cu, CHANNEL_TYPE_LUMA, cu.left );
    const CodingUnit* cuAbove = cu.cs->getCURestricted( posTR.offset( 0, -1 ), cu, CHANNEL_TYPE_LUMA, cu.above );

    const bool isNeigh0Intra = cuLeft  && ( CU::isIntra( *cuLeft ) );
    const bool isNeigh1Intra = cuAbove && ( CU::isIntra( *cuAbove ) );

    const int wIntra = 3 - !isNeigh0Intra - !isNeigh1Intra;
    const int wMerge = 3 - !!isNeigh0Intra - !!isNeigh1Intra;

    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x += 4 )
      {
        dstBuf[y * dstStride + x + 0] = ( wMerge * dstBuf[y * dstStride + x + 0] + wIntra * srcBuf[y * srcStride + x + 0] + 2 ) >> 2;
        dstBuf[y * dstStride + x + 1] = ( wMerge * dstBuf[y * dstStride + x + 1] + wIntra * srcBuf[y * srcStride + x + 1] + 2 ) >> 2;
        dstBuf[y * dstStride + x + 2] = ( wMerge * dstBuf[y * dstStride + x + 2] + wIntra * srcBuf[y * srcStride + x + 2] + 2 ) >> 2;
        dstBuf[y * dstStride + x + 3] = ( wMerge * dstBuf[y * dstStride + x + 3] + wIntra * srcBuf[y * srcStride + x + 3] + 2 ) >> 2;
      }
    }
  }
}

inline int isAboveAvailable(const TransformUnit &tu, const ChannelType &chType, const Position &posLT,
                            const uint32_t uiNumUnitsInPU, const uint32_t unitWidth);
inline int isLeftAvailable(const TransformUnit &tu, const ChannelType &chType, const Position &posLT,
                           const uint32_t uiNumUnitsInPU, const uint32_t unitWidth);

void IntraPrediction::initIntraPatternChType(const TransformUnit &tu, const CompArea &area, const bool bFilterRefSamples)
{
  CHECK( area.width == 2, "Width of 2 is not supported" );
  const CodingStructure& cs   = *tu.cu->cs;

  Pel *refBufUnfiltered   = m_piYuvExt[PRED_BUF_UNFILTERED];
  Pel *refBufFiltered     = m_piYuvExt[PRED_BUF_FILTERED];

  setReferenceArrayLengths( area );

  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, tu );
  // ----- Step 2: filtered reference samples -----
  if( bFilterRefSamples )
  {
    xFilterReferenceSamples( refBufUnfiltered, refBufFiltered, area, *cs.sps , tu.cu->multiRefIdx() );
  }
}

void IntraPrediction::initIntraPatternChTypeISP(const CodingUnit& cu, const CompArea& area, PelBuf& recBuf)
{
  const CodingStructure& cs = *cu.cs;

  const Position &posLT = area.pos();
  bool isLeftAvail  = nullptr != cs.getCURestricted( posLT.offset( -1, 0 ), cu, CH_L, posLT.x == cu.lx() ? cu.left : &cu );
  bool isAboveAvail = nullptr != cs.getCURestricted( posLT.offset( 0, -1 ), cu, CH_L, posLT.y == cu.ly() ? cu.left : &cu );

  // ----- Step 1: unfiltered reference samples -----
  if( cu.blocks[area.compID()].x == area.x && cu.blocks[area.compID()].y == area.y )
  {
    Pel* refBufUnfiltered = m_piYuvExt[PRED_BUF_UNFILTERED];
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

    m_pelBufISP[0] = m_pelBufISPBase[0] = PelBuf(m_piYuvExt[PRED_BUF_UNFILTERED], srcStride, srcHStride);
    m_pelBufISP[1] = m_pelBufISPBase[1] = PelBuf(m_piYuvExt[PRED_BUF_FILTERED], srcStride, srcHStride);

    xFillReferenceSamples( cs.picture->getRecoBuf( cu.Y() ), refBufUnfiltered, cu.Y(), isLuma( area.compID() ) ? cu.firstTU : *cu.lastTU );

    // After having retrieved all the CU reference samples, the number of reference samples is now adjusted for the current subpartition
    m_topRefLength = cu.blocks[area.compID()].width + area.width;
    m_leftRefLength = cu.blocks[area.compID()].height + area.height;
  }
  else
  {
    //Now we only need to fetch the newly available reconstructed samples from the previously coded TU
    Position tuPos = area;
    tuPos.relativeTo(cu.Y());
    m_pelBufISP[0] = m_pelBufISPBase[0].subBuf(tuPos, area.size());
    m_pelBufISP[1] = m_pelBufISPBase[1].subBuf(tuPos, area.size());

    PelBuf& dstBuf = m_pelBufISP[0];

    m_topRefLength = cu.blocks[area.compID()].width + area.width;
    m_leftRefLength = cu.blocks[area.compID()].height + area.height;

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

void IntraPrediction::xFillReferenceSamples( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const TransformUnit &tu )
{
  const ChannelType      chType = toChannelType( area.compID() );
  const CodingUnit      &cu     = *tu.cu;
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int multiRefIdx         = (area.compID() == COMPONENT_Y) ? cu.multiRefIdx() : 0;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
  const int  predSize           = m_topRefLength;
  const int  predHSize          = m_leftRefLength;
  const int  predStride         = predSize + 1 + multiRefIdx;

  const int  csx                = getChannelTypeScaleX( chType, pcv.chrFormat );
  const int  csy                = getChannelTypeScaleY( chType, pcv.chrFormat );

  const int  unitWidth          = pcv.minCUWidth  >> csx;
  const int  unitHeight         = pcv.minCUHeight >> csy;

  const int  totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits      = tuWidth / unitWidth;
  const int  numLeftUnits       = tuHeight / unitHeight;
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported" );

  // ----- Step 1: analyze neighborhood -----
  if( m_lastCUidx == tu.cu->idx && area.compID() != getFirstComponentOfChannel( tu.cu->chType() ) )
  {
  }
  else
  {
    const Position posLT = area.pos();
  
    const bool sameCTUx  = !!( posLT.x & ( pcv.maxCUWidthMask  >> csx ) );
    const bool sameCTUy  = !!( posLT.y & ( pcv.maxCUHeightMask >> csy ) );
    const bool sameCTU   = sameCTUx && sameCTUy;

    m_neighborSize[0]    = sameCTU ? 1 : !!cu.cs->getCURestricted( posLT.offset( -1, -1 ), cu, chType, cu.left ? cu.left : cu.above );

    if( cu.above || area.y > tu.cu->blocks[chType].y )
    {
      m_neighborSize[1] = numAboveUnits;

      Position posAR{ posLT.x + ( PosType ) area.width, posLT.y };
      m_neighborSize[1] += isAboveAvailable( tu, chType, posAR, numAboveRightUnits, unitWidth );
    }
    else
      m_neighborSize[1] = 0;

    if( cu.left || area.x > tu.cu->blocks[chType].x )
    {
      m_neighborSize[2] = numLeftUnits;

      Position posLB{ posLT.x, posLT.y + ( PosType ) area.height };
      m_neighborSize[2] += isLeftAvailable( tu, chType, posLB, numLeftBelowUnits, unitHeight );
    }
    else
      m_neighborSize[2] = 0;

    m_lastCUidx = tu.cu->idx;
  }

  int numIntraNeighbor = m_neighborSize[0] + m_neighborSize[1] + m_neighborSize[2];

  // ----- Step 2: fill reference samples (depending on neighborhood) -----
  const Pel*  srcBuf    = recoBuf.buf;
  const ptrdiff_t srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth() - 1);

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
    if ( m_neighborSize[2] > 0) {  // left is available
      // Fill left & below-left samples if available (downwards)
      ptrSrc = srcBuf - (1 + multiRefIdx);
      ptrDst = refBufUnfiltered + (1 + multiRefIdx) * predStride;
      int tmpSize = m_neighborSize[2] * unitHeight;
      tmpSize = std::min(tmpSize, predHSize);
      for (int i = 0; i < tmpSize; i++) {
        ptrDst[i * predStride] = ptrSrc[i * srcStride];
      }

      // pad
      Pel tmpPixel = ptrDst[(tmpSize - 1) * predStride];
      for (int i = tmpSize; i < predHSize; i++) {
        ptrDst[i * predStride] = tmpPixel;
      }

      // Fill top-left sample(s) if available
      if ( m_neighborSize[0]) {
        ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
        ptrDst = refBufUnfiltered;
        memcpy(ptrDst, ptrSrc, sizeof(Pel) * (multiRefIdx + 1));
        for (int i = 1; i <= multiRefIdx; i++) {
          ptrDst[i * predStride] = ptrSrc[i * srcStride];
        }
      } else {                                // pad
        ptrSrc = srcBuf - (1 + multiRefIdx);  // left pixel
        ptrDst = refBufUnfiltered;
        tmpPixel = ptrSrc[0];
        ptrDst[0] = tmpPixel;
        for (int i = 1; i <= multiRefIdx; i++) {
          ptrDst[i] = tmpPixel;
          ptrDst[i * predStride] = tmpPixel;
        }
      }

      // Fill above & above-right samples if available (left-to-right)
      if ( m_neighborSize[1]) {
        ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
        ptrDst = refBufUnfiltered + 1 + multiRefIdx;
        tmpSize = m_neighborSize[1] * unitWidth;
        tmpSize = std::min(tmpSize, predSize);
        memcpy(ptrDst, ptrSrc, tmpSize * sizeof(Pel));
        // pad
        Pel tmpPixel = ptrDst[tmpSize - 1];
        for (int i = tmpSize; i < predSize; i++) {
          ptrDst[i] = tmpPixel;
        }
      } else {  // all not available, pad
        ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
        ptrDst = refBufUnfiltered + 1 + multiRefIdx;
        Pel tmpPixel = ptrDst[-1];
        std::fill_n(ptrDst, predSize, tmpPixel);
      }
    } else {  // left is not available, top must be available
      // Fill above & above-right samples (left-to-right)
      ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
      ptrDst = refBufUnfiltered + 1 + multiRefIdx;
      int tmpSize = m_neighborSize[1] * unitWidth;
      tmpSize = std::min(tmpSize, predSize);
      memcpy(ptrDst, ptrSrc, tmpSize * sizeof(Pel));
      // pad
      Pel tmpPixel = ptrDst[tmpSize - 1];
      for (int i = tmpSize; i < predSize; i++) {
        ptrDst[i] = tmpPixel;
      }

      tmpPixel = ptrSrc[0];
      // pad top-left sample(s)
      ptrDst = refBufUnfiltered;
      ptrDst[0] = tmpPixel;
      for (int i = 1; i <= multiRefIdx; i++) {
        ptrDst[i] = tmpPixel;
        ptrDst[i * predStride] = tmpPixel;
      }

      // pad left sample(s)
      ptrDst = refBufUnfiltered + (1 + multiRefIdx) * predStride;
      for (int i = 0; i < predHSize; i++) {
        ptrDst[i * predStride] = tmpPixel;
      }
    }
  }
}

void IntraPrediction::xFilterReferenceSamples( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea &area, const SPS &sps, int multiRefIdx, ptrdiff_t stride ) const
{
  if (area.compID() != COMPONENT_Y)
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

static bool getUseFilterRef( const int predMode )
{
  static const int angTable[32]    = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };

  const int     intraPredAngleMode = ( predMode >= DIA_IDX ) ? predMode - VER_IDX : -( predMode - HOR_IDX );

  const int     absAngMode         = abs(intraPredAngleMode);
  const int     absAng             = angTable   [absAngMode];

  return 0 == ( absAng & 0x1F );
}

bool IntraPrediction::useFilteredIntraRefSamples( const ComponentID &compID, const CodingUnit &cu, const UnitArea &tuArea )
{
  //const SPS         &sps    = *cu.sps;
  const ChannelType  chType = toChannelType( compID );

  // high level conditions
  //if( sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag() )  { return false; }
  //if( !isLuma( chType ) )                                           { return false; }
  //if( cu.ispMode() && isLuma(compID) )                              { return false; }
  //if( CU::isMIP( cu, chType ) )                                     { return false; }
  if( cu.multiRefIdx() )                                            { return false; }
  if( cu.bdpcmMode() )                                              { return false; }

  // pred. mode related conditions
  const int dirMode = PU::getFinalIntraMode( cu, chType );
  if (dirMode == DC_IDX)                                            { return false; }
  if (dirMode == PLANAR_IDX)
  {
    return tuArea.blocks[compID].area() > 32 ? true : false;
  }

  const int predMode = getWideAngle(tuArea.blocks[compID].width, tuArea.blocks[compID].height, dirMode);
  const int diff     = std::min<int>( abs( predMode - HOR_IDX ), abs( predMode - VER_IDX ) );
  const int log2Size = ( ( getLog2( tuArea.blocks[compID].width ) + getLog2( tuArea.blocks[compID].height ) ) >> 1 );
  CHECKD( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
  return diff > m_aucIntraFilter[ chType ][ log2Size ] && getUseFilterRef( predMode );
}

static inline TransformUnit const* getTU( const CodingUnit& cu, const Position& pos, const ChannelType chType )
{
  const TransformUnit* ptu = &cu.firstTU;

  if( !ptu->next ) return ptu;

  while( !( ptu->blocks[chType].x + ptu->blocks[chType].width > pos.x && ptu->blocks[chType].y + ptu->blocks[chType].height > pos.y ) )
  {
    ptu = ptu->next;
  }

  return ptu;
}

int isAboveAvailable(const TransformUnit &tu, const ChannelType &chType, const Position &posLT,
                     const uint32_t uiNumUnitsInPU, const uint32_t unitWidth)
{
  const CodingUnit      &cu = *tu.cu;
  const CodingStructure &cs = *cu.cs;

  int maxDx = uiNumUnitsInPU * unitWidth;
  Position refPos = posLT.offset(0, -1);
  const TransformUnit *pcTUAbove = nullptr;
  const int currTUIdx = tu.idx;
  int dx = 0;

  while( dx < maxDx )
  {
    const CodingUnit* cuAbove = cs.getCURestricted( refPos, cu, chType, pcTUAbove ? nullptr : cu.above );

    if( !cuAbove ) break;
    pcTUAbove = getTU( *cuAbove, refPos, chType );
    if( cuAbove->ctuData == cu.ctuData && pcTUAbove->idx >= currTUIdx ) break;

    int diff  = ( int ) pcTUAbove->blocks[chType].width - refPos.x + pcTUAbove->blocks[chType].x;
    dx       += diff;
    refPos.x += diff;
  }

  int neighborSize = dx / unitWidth;
  neighborSize = std::min<int>( neighborSize, uiNumUnitsInPU );
  return neighborSize;
}

int isLeftAvailable(const TransformUnit &tu, const ChannelType &chType, const Position &posLT,
                    const uint32_t uiNumUnitsInPU, const uint32_t unitHeight)
{
  const CodingUnit      &cu = *tu.cu;
  const CodingStructure &cs = *cu.cs;

  int maxDy = uiNumUnitsInPU * unitHeight;
  Position refPos = posLT.offset(-1, 0);
  const TransformUnit *pcTULeft = nullptr;
  int currTUIdx = tu.idx;
  int dy = 0;

  while( dy < maxDy )
  {
    const CodingUnit* cuLeft = cs.getCURestricted( refPos, cu, chType, pcTULeft ? nullptr : cu.left );

    if( !cuLeft ) break;
    pcTULeft = getTU( *cuLeft, refPos, chType );
    if( cuLeft->ctuData == cu.ctuData && pcTULeft->idx >= currTUIdx ) break;

    int diff  = ( int ) pcTULeft->blocks[chType].height - refPos.y + pcTULeft->blocks[chType].y;
    dy       += diff;
    refPos.y += diff;
  }

  int neighborSize = dy / unitHeight;
  neighborSize = std::min<int>( neighborSize, uiNumUnitsInPU );
  return neighborSize;
}
// LumaRecPixels
NO_THREAD_SANITIZE void IntraPrediction::xGetLumaRecPixels(const CodingUnit &cu, CompArea chromaArea)
{
  int iDstStride = 0;
  Pel* pDst0 = 0;
  int curChromaMode = cu.intraDir[1];
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    iDstStride = 2 * MAX_TU_SIZE_FOR_PROFILE + 1;
    pDst0      = m_piYuvExt[1] + iDstStride + 1;
  }
  else
  {
    iDstStride = MAX_TU_SIZE_FOR_PROFILE + 1;
    pDst0      = m_piYuvExt[1] + iDstStride + 1; //MMLM_SAMPLE_NEIGHBOR_LINES;
  }
  //assert 420 chroma subsampling
  CompArea lumaArea = CompArea( COMPONENT_Y, chromaArea.lumaPos( cu.chromaFormat),
                                recalcSize( cu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size() ) );//needed for correct pos/size (4x4 Tus)


  CHECK( lumaArea.width  == chromaArea.width  && CHROMA_444 != cu.chromaFormat, "" );
  CHECK( lumaArea.height == chromaArea.height && CHROMA_444 != cu.chromaFormat && CHROMA_422 != cu.chromaFormat, "" );

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  CPelBuf Src = cu.cs->picture->getRecoBuf( lumaArea );
  Pel const* pRecSrc0   = Src.bufAt( 0, 0 );
  ptrdiff_t iRecStride  = Src.stride;
  int logSubWidthC  = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, cu.chromaFormat);
  int logSubHeightC = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, cu.chromaFormat);


  ptrdiff_t iRecStride2 = iRecStride << logSubHeightC;    // TODO: really Height here? not Width?
  const int mult        =          1 << logSubWidthC ;

  const CompArea& area = isChroma( cu.chType() ) ? chromaArea : lumaArea;

  const uint32_t uiTuWidth  = area.width;
  const uint32_t uiTuHeight = area.height;

  int iBaseUnitSize = ( 1 << MIN_CU_LOG2 );

  const int  iUnitWidth       = iBaseUnitSize >> getComponentScaleX( area.compID(), cu.chromaFormat );
  const int  iUnitHeight      = iBaseUnitSize >> getComponentScaleY( area.compID(), cu.chromaFormat );
  const int  iTUWidthInUnits  = uiTuWidth  / iUnitWidth;
  const int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const int  iAboveUnits      = iTUWidthInUnits;
  const int  iLeftUnits       = iTUHeightInUnits;
  const int  chromaUnitWidth  = iBaseUnitSize >> getComponentScaleX(COMPONENT_Cb, cu.chromaFormat);
  const int  chromaUnitHeight = iBaseUnitSize >> getComponentScaleY(COMPONENT_Cb, cu.chromaFormat);
  const int  topTemplateSampNum  = 2 * uiCWidth; // for MDLM, the number of template samples is 2W or 2H.
  const int  leftTemplateSampNum = 2 * uiCHeight;
  CHECKD( !( m_topRefLength >= topTemplateSampNum ), "Error!" );
  CHECKD( !( m_leftRefLength >= leftTemplateSampNum ), "Error!" );
  int totalAboveUnits = (curChromaMode == MDLM_T_IDX) ? (topTemplateSampNum + (chromaUnitWidth - 1)) / chromaUnitWidth : iAboveUnits;
  int totalLeftUnits = (curChromaMode == MDLM_L_IDX) ? (leftTemplateSampNum + (chromaUnitHeight - 1)) / chromaUnitHeight : iLeftUnits;

  const int  availlableLeftUnit = ( cu.left || chromaArea.x > cu.blocks[CH_C].x ) ? totalLeftUnits : 0;
  const bool bLeftAvaillable    = availlableLeftUnit >= iTUHeightInUnits;
  
  const int  availlableAboveUnit = ( cu.above || chromaArea.y > cu.blocks[CH_C].y ) ? totalAboveUnits : 0;
  const bool bAboveAvaillable    = availlableAboveUnit >= iTUWidthInUnits;

  Pel*       pDst  = nullptr;
  Pel const* piSrc = nullptr;

  bool isFirstRowOfCtu            = ( lumaArea.y & ( cu.sps->getCTUSize() - 1) ) == 0;
  const ptrdiff_t strOffset       = ( CHROMA_444 == cu.chromaFormat ) ? 0 : iRecStride;

  int c0_3tap = 2, c1_3tap = 1, c2_3tap = 1,                                        offset_3tap = 2, shift_3tap = 2; //sum = 4
  int c0_5tap = 1, c1_5tap = 4, c2_5tap = 1, c3_5tap = 1, c4_5tap = 1,              offset_5tap = 4, shift_5tap = 3; //sum = 8
  int c0_6tap = 2, c1_6tap = 1, c2_6tap = 1, c3_6tap = 2, c4_6tap = 1, c5_6tap = 1, offset_6tap = 4, shift_6tap = 3; //sum = 8

  switch (cu.chromaFormat)
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
    int avaiAboveSizes = availlableAboveUnit * chromaUnitWidth;
    for (int i = 0; i < avaiAboveSizes; i++)
    {
      if (isFirstRowOfCtu)
      {
        piSrc = pRecSrc0 - iRecStride;

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + avaiAboveSizes - 1 + logSubWidthC))
        {
          pDst[i] = (piSrc[mult * i] * c0_3tap + piSrc[mult * i] * c1_3tap + piSrc[mult * i + 1] * c2_3tap + offset_3tap) >> shift_3tap;
        }
        else
        {
          pDst[i] = (piSrc[mult * i] * c0_3tap + piSrc[mult * i - 1] * c1_3tap + piSrc[mult * i + 1] * c2_3tap + offset_3tap) >> shift_3tap;
        }
      }
      else if( cu.sps->getCclmCollocatedChromaFlag() )
      {
        piSrc = pRecSrc0 - iRecStride2;

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + avaiAboveSizes - 1 + logSubWidthC))
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

        if ((i == 0 && !bLeftAvaillable) || (i == uiCWidth + avaiAboveSizes - 1 + logSubWidthC))
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

    int availlableLeftSizes = availlableLeftUnit * chromaUnitHeight;
    for (int j = 0; j < availlableLeftSizes; j++)
    {
      if( cu.sps->getCclmCollocatedChromaFlag() )
      {
        if ((j == 0 && !bAboveAvaillable) || (j == uiCHeight + availlableLeftSizes - 1 + logSubWidthC))
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

  if( cu.sps->getCclmCollocatedChromaFlag() )
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

#define GET_LUMA_REC_PIX_OP1(ADDR)                          \
  if( !(ADDR) )                                             \
  {                                                         \
    pDst0[0] = (   pRecSrc0[0    ]              * c0_6tap   \
                 + pRecSrc0[0 + 1]              * c1_6tap   \
                 + pRecSrc0[0]                  * c2_6tap   \
                 + pRecSrc0[0     + iRecStride] * c3_6tap   \
                 + pRecSrc0[0 + 1 + iRecStride] * c4_6tap   \
                 + pRecSrc0[0     + iRecStride] * c5_6tap   \
                 + offset_6tap ) >> shift_6tap;             \
  }                                                         \
  else                                                      \
  {                                                         \
    GET_LUMA_REC_PIX_OP2(ADDR);                             \
  }

  int width  = uiCWidth;
  int height = uiCHeight;

  if( bLeftAvaillable )
  {
    if( cu.chromaFormat == CHROMA_420 )
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
}

#undef GET_LUMA_REC_PIX_INC
#undef GET_LUMA_REC_PIX_OP1
#undef GET_LUMA_REC_PIX_OP2
#undef SIZE_AWARE_PER_EL_OP

void IntraPrediction::xGetLMParameters(const CodingUnit &cu, const ComponentID compID,
                                              const CompArea &chromaArea,
                                              int &a, int &b, int &iShift)
{
  CHECK(compID == COMPONENT_Y, "");

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  const Position posLT = chromaArea;

  const CodingStructure &cs = *cu.cs;

  const SPS &        sps           = *cs.sps;
  const uint32_t     tuWidth     = chromaArea.width;
  const uint32_t     tuHeight    = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const int baseUnitSize = 1 << MIN_CU_LOG2;
  const int unitWidth    = baseUnitSize >> getComponentScaleX(chromaArea.compID(), nChromaFormat);
  const int unitHeight   = baseUnitSize >> getComponentScaleX(chromaArea.compID(), nChromaFormat);

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
  int aboveRightUnits = totalAboveUnits - aboveUnits;
  int leftBelowUnits = totalLeftUnits - leftUnits;

  int curChromaMode = cu.intraDir[1];
  bool aboveAvailable = 0, leftAvailable = 0;

  const TransformUnit& tu = *getTU( cu, chromaArea.pos(), CH_C );

  Pel *srcColor0, *curChroma0;
  int  srcStride, curStride;

  PelBuf temp;
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    srcStride = 2 * MAX_TU_SIZE_FOR_PROFILE + 1;
    temp      = PelBuf(m_piYuvExt[1] + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
    srcStride = MAX_TU_SIZE_FOR_PROFILE + 1;
    temp      = PelBuf(m_piYuvExt[1] + srcStride + 1, srcStride, Size(chromaArea));
  }
  srcColor0 = temp.bufAt(0, 0);
  curChroma0 = getPredictorPtr(compID);

  curStride = m_topRefLength + 1;

  curChroma0 += curStride + 1;

  unsigned internalBitDepth = sps.getBitDepth();

  int minLuma[2] = {  MAX_INT, 0 };
  int maxLuma[2] = { -MAX_INT, 0 };

  Pel *src = srcColor0 - srcStride;
  Pel *cur = curChroma0 - curStride;
  int actualTopTemplateSampNum = 0;
  int actualLeftTemplateSampNum = 0;
  if( curChromaMode == MDLM_T_IDX )
  {
    int avaiAboveUnits = 0;

    if( tu.cu->above || chromaArea.y > tu.cu->blocks[CH_C].y )
    {
      avaiAboveUnits  = aboveUnits;
      aboveRightUnits = aboveRightUnits > ( cHeight / unitWidth ) ? cHeight / unitWidth : aboveRightUnits;
      avaiAboveUnits += isAboveAvailable( tu, CHANNEL_TYPE_CHROMA, { posLT.x + ( PosType ) cWidth, posLT.y }, aboveRightUnits, unitWidth );
    }

    aboveAvailable           = avaiAboveUnits >= tuWidthInUnits;
    actualTopTemplateSampNum = unitWidth * avaiAboveUnits;
  }
  else if( curChromaMode == MDLM_L_IDX )
  {
    int avaiLeftUnits = 0;
    
    if( tu.cu->left || chromaArea.x > tu.cu->blocks[CH_C].x )
    {
      avaiLeftUnits  = leftUnits;
      leftBelowUnits = leftBelowUnits > ( cWidth / unitHeight ) ? cWidth / unitHeight : leftBelowUnits;
      avaiLeftUnits += isLeftAvailable( tu, CHANNEL_TYPE_CHROMA, { posLT.x, posLT.y + ( PosType ) cHeight }, leftBelowUnits, unitHeight );
    }
    
    leftAvailable             = avaiLeftUnits >= tuHeightInUnits;
    actualLeftTemplateSampNum = unitHeight * avaiLeftUnits;
  }
  else if( curChromaMode == LM_CHROMA_IDX )
  {
    aboveAvailable = tu.cu->above || chromaArea.y > tu.cu->blocks[CH_C].y;
    leftAvailable  = tu.cu->left || chromaArea.x > tu.cu->blocks[CH_C].x;
    actualTopTemplateSampNum =  cWidth;
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

      int y = diffC == 0 ? 0 : getLog2( abs( diffC ) ) + 1;
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

void IntraPrediction::initIntraMip( const CodingUnit &cu, const CompArea &area )
{
  CHECK( area.width > MIP_MAX_WIDTH || area.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP" );

  // prepare input (boundary) data for prediction
//  CHECK( m_ipaParam.refFilterFlag, "ERROR: unfiltered refs expected for MIP" );
  Pel *ptrSrc = getPredictorPtr( area.compID() );
  const int srcStride  = m_topRefLength  + 1; //TODO: check this if correct
  const int srcHStride = m_leftRefLength + 1;

  m_matrixIntraPred.prepareInputForPred( CPelBuf( ptrSrc, srcStride, srcHStride ), area, cu.sps->getBitDepth(), area.compID() );
}

void IntraPrediction::predIntraMip( const ComponentID compId, PelBuf &piPred, const CodingUnit &cu )
{
  CHECK( piPred.width > MIP_MAX_WIDTH || piPred.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP" );
  CHECK( piPred.width != (1 << getLog2(piPred.width)) || piPred.height != (1 << getLog2(piPred.height)), "Error: expecting blocks of size 2^M x 2^N" );

  // generate mode-specific prediction
  uint32_t modeIdx       = MAX_NUM_MIP_MODE;
  bool     transposeFlag = false;
  if( compId == COMPONENT_Y )
  {
    modeIdx       = cu.intraDir[CHANNEL_TYPE_LUMA];
    transposeFlag = cu.mipTransposedFlag();
  }
  else
  {
    const CodingUnit &coLocatedLumaPU = PU::getCoLocatedLumaPU(cu);

    CHECK(cu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "Error: MIP is only supported for chroma with DM_CHROMA.");
    CHECK(!coLocatedLumaPU.mipFlag(), "Error: Co-located luma CU should use MIP.");

    modeIdx       = coLocatedLumaPU.intraDir[CHANNEL_TYPE_LUMA];
    transposeFlag = coLocatedLumaPU.mipTransposedFlag();
  }

  CHECK(modeIdx >= getNumModesMip(piPred), "Error: Wrong MIP mode index");

  const int bitDepth = cu.sps->getBitDepth();
  m_matrixIntraPred.predBlock( piPred, modeIdx, piPred, transposeFlag, bitDepth, compId, m_piYuvExt[0] );
}

}
