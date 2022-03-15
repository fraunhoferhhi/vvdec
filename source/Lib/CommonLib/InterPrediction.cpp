/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

#include "InterPrediction.h"

#include "Buffer.h"
#include "UnitTools.h"
#include "CommonLib/TimeProfiler.h"

#include <memory.h>
#include <algorithm>
#include <cmath>

namespace vvdec
{

template<bool bi>
void applyPROFCore( Pel* dst, ptrdiff_t dstStride, const Pel* src, const Pel* gradX, const Pel* gradY, const int* dMvX, const int* dMvY, int shiftNum, Pel offset, const ClpRng& clpRng )
{
  static constexpr ptrdiff_t srcStride = 6;
  static constexpr int width = 4;
  static constexpr int height = 4;

  int idx = 0;
  const int dILimit = 1 << std::max<int>(clpRng.bd + 1, 13);

  for (int h = 0; h < height; h++)
  {
    for (int w = 0; w < width; w++)
    {
      int32_t dI = dMvX[idx] * gradX[w] + dMvY[idx] * gradY[w];
      dI = Clip3(-dILimit, dILimit - 1, dI);
      dst[w] = src[w] + dI;
      if (!bi)
      {
        dst[w] = (dst[w] + offset) >> shiftNum;
        dst[w] = ClipPel(dst[w], clpRng);
      }
      idx++;
    }
    gradX += 4;
    gradY += 4;
    dst += dstStride;
    src += srcStride;
  }
}


static inline int rightShiftMSB(int numer, int denom)
{
  int     d;
  int msbIdx = 0;
  for (msbIdx = 0; msbIdx<32; msbIdx++)
  {
    if (denom < ((int)1 << msbIdx))
    {
      break;
    }
  }
  int shiftIdx = msbIdx - 1;
  d = (numer >> shiftIdx);

  return d;
}

void addBIOAvg4(const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel *dst, ptrdiff_t dstStride, const Pel *gradX0, const Pel *gradX1, const Pel *gradY0, const Pel*gradY1, ptrdiff_t gradStride, int width, int height, int tmpx, int tmpy, int shift, int offset, const ClpRng& clpRng)
{
  int b = 0;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 4)
    {
      b = tmpx * (gradX0[x] - gradX1[x]) + tmpy * (gradY0[x] - gradY1[x]);
      dst[x] = ClipPel((int16_t)rightShift((src0[x] + src1[x] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 1] - gradX1[x + 1]) + tmpy * (gradY0[x + 1] - gradY1[x + 1]);
      dst[x + 1] = ClipPel((int16_t)rightShift((src0[x + 1] + src1[x + 1] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 2] - gradX1[x + 2]) + tmpy * (gradY0[x + 2] - gradY1[x + 2]);
      dst[x + 2] = ClipPel((int16_t)rightShift((src0[x + 2] + src1[x + 2] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 3] - gradX1[x + 3]) + tmpy * (gradY0[x + 3] - gradY1[x + 3]);
      dst[x + 3] = ClipPel((int16_t)rightShift((src0[x + 3] + src1[x + 3] + b + offset), shift), clpRng);
    }
    dst += dstStride;       src0 += src0Stride;     src1 += src1Stride;
    gradX0 += gradStride; gradX1 += gradStride; gradY0 += gradStride; gradY1 += gradStride;
  }
}

void calcBIOSums(const Pel* srcY0Tmp, const Pel* srcY1Tmp, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, int xu, int yu, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGY_GX)
{
  int shift4 = 4;
  int shift5 = 1;

  for (int y = 0; y < 6; y++)
  {
    for (int x = 0; x < 6; x++)
    {
      int tmpGX = (gradX0[x] + gradX1[x]) >> shift5;
      int tmpGY = (gradY0[x] + gradY1[x]) >> shift5;
      int tmpDI = (int)((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4));
      *sumAbsGX += (tmpGX < 0 ? -tmpGX : tmpGX);
      *sumAbsGY += (tmpGY < 0 ? -tmpGY : tmpGY);
      *sumDIX += (tmpGX < 0 ? -tmpDI : (tmpGX == 0 ? 0 : tmpDI));
      *sumDIY += (tmpGY < 0 ? -tmpDI : (tmpGY == 0 ? 0 : tmpDI));
      *sumSignGY_GX += (tmpGY < 0 ? -tmpGX : (tmpGY == 0 ? 0 : tmpGX));

    }
    srcY1Tmp += src1Stride;
    srcY0Tmp += src0Stride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
  }
}

void BiOptFlowCore(const Pel* srcY0,const Pel* srcY1,const Pel* gradX0,const Pel* gradX1,const Pel* gradY0,const Pel* gradY1,const int width,const int height,Pel* dstY,const ptrdiff_t dstStride,const int shiftNum,const int  offset,const int  limit, const ClpRng& clpRng, const int bitDepth)
{
  int           widthG  = width      + BIO_ALIGN_SIZE;
  int           stridePredMC = width + BIO_ALIGN_SIZE;
  int           offsetPos = widthG*BIO_EXTEND_SIZE + BIO_EXTEND_SIZE;
  int xUnit = (width >> 2);
  int yUnit = (height >> 2);

  const Pel*    srcY0Temp;
  const Pel*    srcY1Temp;
  int OffPos;
  Pel *dstY0;
  for (int yu = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++)
    {
      OffPos=offsetPos + ((yu*widthG + xu) << 2);

      {
        int tmpx = 0, tmpy = 0;
        int sumAbsGX = 0, sumAbsGY = 0, sumDIX = 0, sumDIY = 0;
        int sumSignGY_GX = 0;

        const Pel* pGradX0Tmp = gradX0 + (xu << 2) + (yu << 2) * widthG;
        const Pel* pGradX1Tmp = gradX1 + (xu << 2) + (yu << 2) * widthG;
        const Pel* pGradY0Tmp = gradY0 + (xu << 2) + (yu << 2) * widthG;
        const Pel* pGradY1Tmp = gradY1 + (xu << 2) + (yu << 2) * widthG;
        const Pel* SrcY1Tmp = srcY1 + (xu << 2) + (yu << 2) * stridePredMC;
        const Pel* SrcY0Tmp = srcY0 + (xu << 2) + (yu << 2) * stridePredMC;

        calcBIOSums(SrcY0Tmp, SrcY1Tmp, pGradX0Tmp, pGradX1Tmp, pGradY0Tmp, pGradY1Tmp, xu, yu, stridePredMC, stridePredMC, widthG, bitDepth, &sumAbsGX, &sumAbsGY, &sumDIX, &sumDIY, &sumSignGY_GX);
        tmpx = (sumAbsGX == 0 ? 0 : rightShiftMSB(sumDIX << 2, sumAbsGX));
        tmpx = Clip3(-limit, limit, tmpx);
        int     mainsGxGy = sumSignGY_GX >> 12;
        int     secsGxGy = sumSignGY_GX & ((1 << 12) - 1);
        int     tmpData = tmpx * mainsGxGy;
        tmpData = ((tmpData << 12) + tmpx*secsGxGy) >> 1;
        tmpy = (sumAbsGY == 0 ? 0 : rightShiftMSB(((sumDIY << 2) - tmpData), sumAbsGY));
        tmpy = Clip3(-limit, limit, tmpy);
        srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu*stridePredMC + xu) << 2);
        srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu*stridePredMC + xu) << 2);
        dstY0 = dstY + ((yu*dstStride + xu) << 2);
        addBIOAvg4(srcY0Temp, stridePredMC, srcY1Temp, stridePredMC, dstY0, dstStride, gradX0 + OffPos, gradX1 + OffPos, gradY0 + OffPos, gradY1 + OffPos, widthG, (1 << 2), (1 << 2), (int)tmpx, (int)tmpy, shiftNum, offset, clpRng);
      }
    }  // xu
  }  // yu


}

template<bool PAD = true>
void gradFilterCore(Pel* pSrc, ptrdiff_t srcStride, int width, int height, ptrdiff_t gradStride, Pel* gradX, Pel* gradY, const int bitDepth)
{
  Pel* srcTmp   = PAD ? pSrc  + srcStride  + 1 : pSrc;
  Pel* gradXTmp = PAD ? gradX + gradStride + 1 : gradX;
  Pel* gradYTmp = PAD ? gradY + gradStride + 1 : gradY;
  int  shift1 = 6;
  
  const int widthInside  = PAD ? width  - 2 * BIO_EXTEND_SIZE : width;
  const int heightInside = PAD ? height - 2 * BIO_EXTEND_SIZE : height;

  for (int y = 0; y < heightInside; y++)
  {
    for (int x = 0; x < widthInside; x++)
    {
      gradYTmp[x] = ( srcTmp[x + srcStride] >> shift1 ) - ( srcTmp[x - srcStride] >> shift1 );
      gradXTmp[x] = ( srcTmp[x + 1] >> shift1 ) - ( srcTmp[x - 1] >> shift1 );
    }
    gradXTmp += gradStride;
    gradYTmp += gradStride;
    srcTmp += srcStride;
  }

  if (PAD)
  {
    gradXTmp = gradX + gradStride + 1;
    gradYTmp = gradY + gradStride + 1;
    srcTmp   = pSrc  + srcStride  + 1;

    for (int y = 0; y < heightInside; y++)
    {
      gradXTmp[-1] = gradXTmp[0];
      gradXTmp[width - 2 * BIO_EXTEND_SIZE] = gradXTmp[width - 2 * BIO_EXTEND_SIZE - 1];
      gradXTmp += gradStride;

      gradYTmp[-1] = gradYTmp[0];
      gradYTmp[width - 2 * BIO_EXTEND_SIZE] = gradYTmp[width - 2 * BIO_EXTEND_SIZE - 1];
      gradYTmp += gradStride;
      
      srcTmp[-1] = srcTmp[0];
      srcTmp[width - 2 * BIO_EXTEND_SIZE] = srcTmp[width - 2 * BIO_EXTEND_SIZE - 1];
      srcTmp += srcStride;
    }

    gradXTmp = gradX + gradStride;
    gradYTmp = gradY + gradStride;
    srcTmp   = pSrc  + srcStride;

    ::memcpy(gradXTmp - gradStride, gradXTmp, sizeof(Pel)*(width));
    ::memcpy(gradXTmp + (height - 2 * BIO_EXTEND_SIZE)*gradStride, gradXTmp + (height - 2 * BIO_EXTEND_SIZE - 1)*gradStride, sizeof(Pel)*(width));
    ::memcpy(gradYTmp - gradStride, gradYTmp, sizeof(Pel)*(width));
    ::memcpy(gradYTmp + (height - 2 * BIO_EXTEND_SIZE)*gradStride, gradYTmp + (height - 2 * BIO_EXTEND_SIZE - 1)*gradStride, sizeof(Pel)*(width));
    ::memcpy(srcTmp   - srcStride , srcTmp, sizeof(Pel)*(width));
    ::memcpy(srcTmp   + (height - 2 * BIO_EXTEND_SIZE)*srcStride , srcTmp   + (height - 2 * BIO_EXTEND_SIZE - 1)*srcStride , sizeof(Pel)*(width));
  }
}

void PaddBIOCore(const Pel* refPel,Pel* dstPel,unsigned width,const int shift)
{
#define LFTSHFT(y,shift) y<<shift           // simplification because shift is never < 0

  for( int w = 0; w < width + 2 * BIO_EXTEND_SIZE; w++ )
  {
    Pel val   = LFTSHFT( refPel[w], shift );
    dstPel[w] = val - (Pel)IF_INTERNAL_OFFS;
  }


}

template<int padSize>
void paddingCore(Pel *ptr, ptrdiff_t stride, int width, int height)
{
  /*left and right padding*/
  Pel *ptrTemp1 = ptr;
  Pel *ptrTemp2 = ptr + (width - 1);
  ptrdiff_t offset = 0;
  for (int i = 0; i < height; i++)
  {
    offset = stride * i;
    for (int j = 1; j <= padSize; j++)
    {
      *(ptrTemp1 - j + offset) = *(ptrTemp1 + offset);
      *(ptrTemp2 + j + offset) = *(ptrTemp2 + offset);
    }
  }
  /*Top and Bottom padding*/
  int numBytes = (width + padSize + padSize) * sizeof(Pel);
  ptrTemp1 = (ptr - padSize);
  ptrTemp2 = (ptr + (stride * (height - 1)) - padSize);
  for (int i = 1; i <= padSize; i++)
  {
    memcpy(ptrTemp1 - (i * stride), (ptrTemp1), numBytes);
    memcpy(ptrTemp2 + (i * stride), (ptrTemp2), numBytes);
  }
}

template<int padSize>
void prefetchPadCore( const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height )
{
  g_pelBufOP.copyBuffer( ( const char* ) src, srcStride * sizeof( Pel ), ( char* ) dst, dstStride * sizeof( Pel ), width * sizeof( Pel ), height );

  paddingCore<padSize>( dst, dstStride, width, height );
}

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================


InterPrediction::InterPrediction()
  : BiOptFlow     ( BiOptFlowCore )
  , BioGradFilter ( gradFilterCore )
  , profGradFilter( gradFilterCore<false> )
  , roundIntVector( nullptr )
{
  clipMv = clipMvInPic;

  m_currChromaFormat = NUM_CHROMA_FORMAT;
}

InterPrediction::~InterPrediction()
{
  destroy();
}

void InterPrediction::destroy()
{
  m_IBCBuffer.destroy();
}

void InterPrediction::init( RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize )
{
  m_pcRdCost = pcRdCost;

  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_currChromaFormat != chromaFormatIDC )
  {
    destroy();
    m_currChromaFormat = NUM_CHROMA_FORMAT;
  }

  if( m_currChromaFormat == NUM_CHROMA_FORMAT ) // check if first is null (in which case, nothing initialised yet)
  {
    VALGRIND_MEMCLEAR( m_bdofBlock );
    VALGRIND_MEMCLEAR( m_tmpBlock  );

    m_iRefListIdx = -1;

    VALGRIND_MEMCLEAR( m_gradX0 );
    VALGRIND_MEMCLEAR( m_gradY0 );
    VALGRIND_MEMCLEAR( m_gradX1 );
    VALGRIND_MEMCLEAR( m_gradY1 );

    m_if.initInterpolationFilter( true );

    applyPROF[0] = applyPROFCore<0>;
    applyPROF[1] = applyPROFCore<1>;
    PaddBIO      = PaddBIOCore;
    prefetchPad[0] = prefetchPadCore<2>; // luma
    prefetchPad[1] = prefetchPadCore<2>; // chroma for 444 and 422
    prefetchPad[2] = prefetchPadCore<1>; // chroma for 420
#if ENABLE_SIMD_OPT_INTER && defined( TARGET_SIMD_X86 )
    initInterPredictionX86();
#endif
  }

  if( m_IBCBuffer.bufs.empty() )
  {
    m_IBCBufferWidth = g_IBCBufferSize / ctuSize;
    m_IBCBuffer.create( UnitArea( chromaFormatIDC, Area( 0, 0, m_IBCBufferWidth, ctuSize ) ) );
  }
  m_currChromaFormat = chromaFormatIDC;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion( const PredictionUnit &pu )
{
  const Slice &slice = *pu.slice;

  if( slice.isInterB() && !pu.pps->getWPBiPred() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPOC( REF_PIC_LIST_0, pu.refIdx[0] );
      int RefPOCL1 = slice.getRefPOC( REF_PIC_LIST_1, pu.refIdx[1] );

      if( RefPOCL0 == RefPOCL1 )
      {
        if( !pu.affineFlag() )
        {
          if( pu.mv[0][0] == pu.mv[1][0] )
          {
            return true;
          }
        }
        else
        {
          if( pu.mv[0][0] == pu.mv[1][0] && pu.mv[0][1] == pu.mv[1][1] && ( pu.affineType() == AFFINEMODEL_4PARAM || pu.mv[0][2] == pu.mv[1][2] ) )
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

void InterPrediction::xSubPuMC( PredictionUnit& pu, PelUnitBuf& predBuf )
{
  // compute the location of the current PU
  const Position puPos    = pu.lumaPos();
  const Size puSize       = pu.lumaSize();

  const int numPartLine   = std::max<SizeType>( puSize.width  >> ATMVP_SUB_BLOCK_SIZE, 1u );
  const int numPartCol    = std::max<SizeType>( puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u );
  const int puHeight      = numPartCol  == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  const int puWidth       = numPartLine == 1 ? puSize.width  : 1 << ATMVP_SUB_BLOCK_SIZE;

  CodingUnit      subCu;
  PredictionUnit& subPu = subCu;

  subCu.cs           = pu.cs;
  subCu.slice        = pu.slice;
  subCu.pps          = pu.pps;
  subCu.sps          = pu.sps;
  subCu.setChType    ( pu.chType() );
  subCu.setPredMode  ( pu.predMode() );
  subCu.UnitArea::operator=( pu );

  subPu.setMergeType ( MRG_TYPE_DEFAULT_N );
  subPu.setAffineFlag( false );
  subPu.setGeoFlag   ( false );
  subPu.setBcwIdx    ( pu.BcwIdx() );
  subPu.setImv       ( pu.imv() );
  subPu.setSmvdMode  ( pu.smvdMode() );

  // join sub-pus containing the same motion
  bool verMC    = puSize.height > puSize.width;
  int  fstStart = ( !verMC ? puPos.y : puPos.x );
  int  secStart = ( !verMC ? puPos.x : puPos.y );
  int  fstEnd   = ( !verMC ? puPos.y + puSize.height : puPos.x + puSize.width  );
  int  secEnd   = ( !verMC ? puPos.x + puSize.width  : puPos.y + puSize.height );
  int  fstStep  = ( !verMC ? puHeight : puWidth  );
  int  secStep  = ( !verMC ? puWidth  : puHeight );

#if RPR_FIX
  bool scaled = pu.slice->getRefPic( REF_PIC_LIST_0, 0 )->isRefScaled( pu.pps ) || ( pu.slice->getSliceType() == B_SLICE ? pu.slice->getRefPic( REF_PIC_LIST_1, 0 )->isRefScaled( pu.pps ) : false );
#endif
  
  m_subPuMC = true;

  for( int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep )
  {
    for( int secDim = secStart; secDim < secEnd; secDim += secStep )
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = pu.getMotionInfo( Position{ x, y } );

      int length = secStep;
      int later   = secDim + secStep;

      while( later < secEnd )
      {
        const MotionInfo &laterMi = !verMC ? pu.getMotionInfo( Position{ later, fstDim } ) : pu.getMotionInfo( Position{ fstDim, later } );
#if RPR_FIX
        if( !scaled && laterMi == curMi )
#else
        if( laterMi == curMi )
#endif
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }

      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu = curMi;

      if( !verMC && ( dx & 15 ) && dx > 16 )
      {
        int dxPart = dx & ~15;

        new ( &static_cast< UnitArea& >( subPu ) ) UnitArea( pu.chromaFormat, Area( x, y, dxPart, dy ) );
        PelUnitBuf subPredBuf = predBuf.subBuf( UnitAreaRelative( pu, subPu ) );

        motionCompensation( subPu, subPredBuf );
        x  += dxPart;
        dx -= dxPart;
      }
      else if( verMC && ( dy & 15 ) && dy > 16 )
      {
        int dyPart = dy & ~15;

        new ( &static_cast< UnitArea& >( subPu ) ) UnitArea( pu.chromaFormat, Area( x, y, dx, dyPart ) );
        PelUnitBuf subPredBuf = predBuf.subBuf( UnitAreaRelative( pu, subPu ) );

        motionCompensation( subPu, subPredBuf );

        y  += dyPart;
        dy -= dyPart;
      }

      new ( &static_cast< UnitArea& >( subPu ) ) UnitArea( pu.chromaFormat, Area( x, y, dx, dy ) );
      PelUnitBuf subPredBuf = predBuf.subBuf( UnitAreaRelative( pu, subPu ) );

      motionCompensation( subPu, subPredBuf );

      secDim = later - secStep;
    }
  }
  m_subPuMC = false;
}

void InterPrediction::xSubPuBio(PredictionUnit& pu, PelUnitBuf& predBuf )
{
  // compute the location of the current PU
  const Position puPos = pu.lumaPos();
  const Size puSize    = pu.lumaSize();
  
  CodingUnit      subCu;
  PredictionUnit& subPu = subCu;

  subCu.cs           = pu.cs;
  subCu.slice        = pu.slice;
  subCu.pps          = pu.pps;
  subCu.sps          = pu.sps;
  subCu.setChType    ( pu.chType() );
  subCu.setPredMode  ( pu.predMode() );

  subPu.setMergeType ( pu.mergeType() );
  subPu.setMmvdFlag  ( pu.mmvdFlag() );
  subPu.setMergeFlag ( pu.mergeFlag() );
  subPu.setCiipFlag  ( pu.ciipFlag() );
  subPu.setGeoFlag   ( pu.geoFlag() );
//  subPu.mvRefine = pu.mvRefine;
  subPu.setAffineFlag( pu.affineFlag() );
  subPu.refIdx[0]    = pu.refIdx[0];
  subPu.refIdx[1]    = pu.refIdx[1];
  subPu.setBcwIdx    ( pu.BcwIdx() );
  subPu.setImv       ( pu.imv() );
  subPu.setSmvdMode  ( pu.smvdMode() );

  const int  subPuHeight = std::min<int>( MAX_BDOF_APPLICATION_REGION, puSize.height );
  const int  subPuWidth  = std::min<int>( MAX_BDOF_APPLICATION_REGION, puSize.width );

  const int  csy = getChannelTypeScaleY( CH_C, pu.chromaFormat );
  const int  csx = getChannelTypeScaleX( CH_C, pu.chromaFormat );

  const int  subPuHeightC = subPuHeight >> csy;
  const int  subPuWidthC  = subPuWidth  >> csx;
  
  PelUnitBuf pcMbBuf1( pu.chromaFormat, PelBuf( m_acYuvPred[0], subPuWidth, subPuHeight ), PelBuf( m_acYuvPred[1], subPuWidthC, subPuHeightC ), PelBuf( m_acYuvPred[2], subPuWidthC, subPuHeightC ) );
  
  PelUnitBuf subPredBuf = predBuf.subBuf( UnitAreaRelative( pu, UnitArea( pu.chromaFormat, Area( 0, 0, subPuWidth, subPuHeight ) ) ) );
                                                                   
  for( int y = puPos.y, dy = 0; y < puPos.y + puSize.height; y += subPuHeight, dy += subPuHeight )
  {
    for( int x = puPos.x, dx = 0; x < puPos.x + puSize.width; x += subPuWidth, dx += subPuWidth )
    {
      const MotionInfo &curMi = pu.getMotionInfo( Position{ x, y } );

      new ( &static_cast< UnitArea& >( subPu ) ) UnitArea( pu.chromaFormat, Area( x, y, subPuWidth, subPuHeight ) );
      subPu = curMi;

      subPredBuf  .bufs[0].buf = GET_OFFSET( predBuf.bufs[0].buf, predBuf.bufs[0].stride, dx,        dy );
      if( isChromaEnabled( pu.chromaFormat ) )
      {
        subPredBuf.bufs[1].buf = GET_OFFSET( predBuf.bufs[1].buf, predBuf.bufs[1].stride, dx >> csx, dy >> csy );
        subPredBuf.bufs[2].buf = GET_OFFSET( predBuf.bufs[2].buf, predBuf.bufs[2].stride, dx >> csx, dy >> csy );
      }

      CHECKD( pu.refIdx[0] < 0 || pu.refIdx[1] < 0, "Bi-prediction required for BDOF!" );

      PelUnitBuf& pcMbBuf0 = subPredBuf;

      m_iRefListIdx = REF_PIC_LIST_0;
      xPredInterUni( subPu, REF_PIC_LIST_0, pcMbBuf0, true, true, true, true );
      m_iRefListIdx = REF_PIC_LIST_1;
      xPredInterUni( subPu, REF_PIC_LIST_1, pcMbBuf1, true, true, true, true );

      xWeightedAverage( subPu, pcMbBuf0, pcMbBuf1, subPredBuf, pu.sps->getBitDepths(), pu.slice->clpRngs(), true );
    }
  }
}

void InterPrediction::xPredInterUni( const PredictionUnit &pu, const RefPicList &eRefPicList, PelUnitBuf &pcYuvPred, const bool &bi, const bool &bioApplied, const bool luma, const bool chroma )
{
  const SPS &    sps     = *pu.sps;
  const int      iRefIdx = pu.refIdx[eRefPicList];
  const bool     isIBC   = CU::isIBC( pu );
  const Picture *refPic  = isIBC ? pu.slice->getPic() : pu.slice->getRefPic( eRefPicList, iRefIdx );
        bool     affine  = pu.affineFlag();
  Mv             mv[3];
  const bool scaled       = refPic ? refPic->isRefScaled( pu.pps ) : false;
  const auto scalingRatio = pu.slice->getScalingRatio( eRefPicList, iRefIdx );

  CHECKD( !CU::isIBC( pu ) && pu.lwidth() == 4 && pu.lheight() == 4, "invalid 4x4 inter blocks" );

  if( affine )
  {
    CHECK( iRefIdx < 0, "iRefIdx incorrect." );

    mv[0] = pu.mv[eRefPicList][0];
    mv[1] = pu.mv[eRefPicList][1];
    mv[2] = pu.mv[eRefPicList][2];
  }
  else
  {
    mv[0] = pu.mv[eRefPicList][0];

    CHECK( !refPic, "xPredInterUni missing ref pic" );

    if( !isIBC && !scaled )
    {
      clipMv( mv[0], m_currCuArea.lumaPos(), m_currCuArea.lumaSize(), sps, *pu.pps );
    }
  }

  const bool wrapRef = !isIBC && pu.sps->getUseWrapAround() && wrapClipMv( mv[0], pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );

  for( uint32_t comp = COMPONENT_Y; comp < pcYuvPred.bufs.size(); comp++ )
  {
    const ComponentID compID = ComponentID( comp );

    if( compID == COMPONENT_Y && !luma   ) continue;
    if( compID != COMPONENT_Y && !chroma ) continue;

    if( affine )
    {
      CHECK( bioApplied, "BIO is not allowed with affine" );
      m_iRefListIdx = eRefPicList;
      xPredAffineBlk( compID, pu, refPic, eRefPicList, pcYuvPred, bi, pu.slice->clpRng( compID ), pu.slice->getScalingRatio( eRefPicList, iRefIdx ) );
    }
    else
    {
      if( !isIBC && scaled )
      {
        xPredInterBlkRPR( scalingRatio, *pu.pps, compID, pu.chromaFormat, refPic, mv[0], pu.blocks[compID], pcYuvPred.bufs[compID].width, pcYuvPred.bufs[compID].height, pcYuvPred.bufs[compID].buf, pcYuvPred.bufs[compID].stride, bi, wrapRef, pu.slice->clpRng( compID ), 0, pu.imv() == IMV_HPEL );
        CHECKD( bioApplied, "BDOF should be disabled with RPR" );
      }
      else
      {
        xPredInterBlk<false, false>( compID, pu, refPic->unscaledPic, mv[0], pcYuvPred.bufs[compID], bi, pu.slice->clpRng( compID ), bioApplied, isIBC, wrapRef );
      }
    }
  }
}

void InterPrediction::xPredInterBi( PredictionUnit& pu, PelUnitBuf &pcYuvPred )
{
  const Slice &slice = *pu.slice;
  const PPS   &pps   = *pu.pps;

  PelUnitBuf& pcMbBuf0 = pcYuvPred;
  PelUnitBuf  pcMbBuf1 = isChromaEnabled( pu.chromaFormat ) ? PelUnitBuf( pu.chromaFormat, PelBuf( m_acYuvPred[0], pcYuvPred.Y() ), PelBuf( m_acYuvPred[1], pcYuvPred.Cb() ), PelBuf( m_acYuvPred[2], pcYuvPred.Cr() ) ) : PelUnitBuf( pu.chromaFormat, PelBuf( m_acYuvPred[0], pcYuvPred.Y() ) );

  const bool isBiPred = pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0;

  if( isBiPred )
  {
    m_iRefListIdx = REF_PIC_LIST_0;
    xPredInterUni( pu, REF_PIC_LIST_0, pcMbBuf0, true, false, true, true );
    m_iRefListIdx = REF_PIC_LIST_1;
    xPredInterUni( pu, REF_PIC_LIST_1, pcMbBuf1, true, false, true, true );
  }
  else
  {
    m_iRefListIdx = pu.refIdx[0] >= 0 ? REF_PIC_LIST_0 : REF_PIC_LIST_1;

    if( !pu.geoFlag() && ( ( pps.getUseWP() && slice.getSliceType() == P_SLICE ) || ( pps.getWPBiPred() && slice.getSliceType() == B_SLICE ) ) )
    {
      xPredInterUni( pu, RefPicList( m_iRefListIdx ), pcMbBuf0, true, false, true, true );
    }
    else
    {
      xPredInterUni( pu, RefPicList( m_iRefListIdx ), pcYuvPred, pu.geoFlag(), false, true, true );
    }
  }

#ifndef NDEBUG
  for( uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    if( pu.refIdx[refList] < 0 )
    {
      continue;
    }

    RefPicList eRefPicList = refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

    CHECKD( CU::isIBC( pu ) && eRefPicList != REF_PIC_LIST_0, "Invalid interdir for ibc mode" );
    CHECKD( CU::isIBC( pu ) && pu.refIdx[refList] != MAX_NUM_REF, "Invalid reference index for ibc mode" );
    CHECKD( CU::isInter( pu ) && pu.refIdx[refList] >= slice.getNumRefIdx( eRefPicList ), "Invalid reference index" );
  }

#endif
  if( !pu.geoFlag() )
  {
    if( pps.getWPBiPred() && slice.getSliceType() == B_SLICE && pu.BcwIdx() == BCW_DEFAULT )
    {
      xWeightedPredictionBi( pu, pcMbBuf0, isBiPred ? pcMbBuf1 : pcMbBuf0, pcYuvPred );
    }
    else if( pps.getUseWP() && slice.getSliceType() == P_SLICE )
    {
      xWeightedPredictionUni( pu, pcMbBuf0, REF_PIC_LIST_0, pcYuvPred, -1 );
    }
    else if( isBiPred )
    {
      xWeightedAverage( pu, pcMbBuf0, pcMbBuf1, pcYuvPred, slice.getSPS()->getBitDepths(), slice.clpRngs(), false );
    }
  }
}

template<bool altSrc, bool altSize>
void InterPrediction::xPredInterBlk( const ComponentID&    compID,
                                     const PredictionUnit& pu,
                                     const Picture*        refPic,
                                     Mv                    mv,
                                     PelBuf&               dstPic,
                                     bool                  bi,
                                     const ClpRng&         clpRng,
                                     bool                  bioApplied,
                                     bool                  isIBC,
                                     bool                  wrapRef,
                                     SizeType              dmvrWidth,
                                     SizeType              dmvrHeight,
                                     bool                  bilinearMC,
                                     Pel*                  srcPadBuf,
                                     ptrdiff_t             srcPadStride )
{
  CHECK( srcPadBuf == NULL && altSrc, "wrong" );
  
  const ChromaFormat  chFmt = pu.chromaFormat;
  const bool          rndRes = !bi;

  const int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleX(compID, chFmt);
  const int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleY(compID, chFmt);
  
  const bool useAltHpelIf = pu.imv() == IMV_HPEL;

  const int xFrac     = !isIBC ? mv.hor & ( ( 1 << shiftHor ) - 1 ) : 0;
  const int yFrac     = !isIBC ? mv.ver & ( ( 1 << shiftVer ) - 1 ) : 0;

  const Pel* refPtr    = nullptr;
  ptrdiff_t  refStride = 0;
  if( pu.pps->getNumSubPics() > 1 && pu.pps->getSubPicFromCU( pu ).getTreatedAsPicFlag() )
  {
    const int subPicIdx = pu.pps->getSubPicFromCU( pu ).getSubPicIdx();
    refPtr              = altSrc ? srcPadBuf    : refPic->getSubPicBufPtr   ( subPicIdx, compID, wrapRef );
    refStride           = altSrc ? srcPadStride : refPic->getSubPicBufStride( subPicIdx, compID, wrapRef );
  }
  else
  {
    refPtr    = altSrc ? srcPadBuf    : refPic->getRecoBufPtr   ( compID, wrapRef );
    refStride = altSrc ? srcPadStride : refPic->getRecoBufStride( compID, wrapRef );
  }

  if( !altSrc )
  {
    OFFSET( refPtr, refStride, pu.blocks[compID].x + ( mv.hor >> shiftHor ), pu.blocks[compID].y + ( mv.ver >> shiftVer ) );
  }

  unsigned width, height;

  if( altSize )
  {
    width  = dmvrWidth;
    height = dmvrHeight;
  }
  else
  {
    width  = dstPic.width;
    height = dstPic.height;
  }

  Pel* dstBuf;
  ptrdiff_t dstStride;

  if( bioApplied && compID == COMPONENT_Y )
  {
    // change MC output
    dstStride = width + BIO_ALIGN_SIZE;
    dstBuf    = m_bdofBlock[m_iRefListIdx] + 2 * dstStride + 1;
  }
  else
  {
    dstBuf    = dstPic.buf;
    dstStride = dstPic.stride;
  }

    
  if( yFrac == 0 )
  {
    m_if.filterHor( compID, refPtr, refStride, dstBuf, dstStride, width, height, xFrac, rndRes, chFmt, clpRng, bilinearMC, bilinearMC, useAltHpelIf );
  }
  else if( xFrac == 0 )
  {
    m_if.filterVer( compID, refPtr, refStride, dstBuf, dstStride, width, height, yFrac, true, rndRes, chFmt, clpRng, bilinearMC, bilinearMC, useAltHpelIf );
  }
  else if( bilinearMC )
  {
    m_if.filterN2_2D( compID, refPtr, refStride, dstBuf, dstStride, width, height, xFrac, yFrac, chFmt, clpRng );
  }
  else if( width == 4 && height == 4 && !bilinearMC )
  {
    m_if.filter4x4( compID, refPtr, refStride, dstBuf, dstStride, 4, 4, xFrac, yFrac, rndRes, chFmt, clpRng );
  }
  else if( !bilinearMC && ( width & 15 ) == 0 )
  {
    for( int dx = 0; dx < width; dx += 16 )
    {
      m_if.filter16x16( compID, refPtr + dx, refStride, dstBuf + dx, dstStride, 16, height, xFrac, yFrac, rndRes, chFmt, clpRng, useAltHpelIf );
    }
  }
  else if( !bilinearMC && ( width & 7 ) == 0 )
  {
    for( int dx = 0; dx < width; dx += 8 )
    {
      m_if.filter8x8( compID, refPtr + dx, refStride, dstBuf + dx, dstStride, 8, height, xFrac, yFrac, rndRes, chFmt, clpRng, useAltHpelIf );
    }
  }
  else
  {
    Pel *tmpBuf = m_tmpBlock;
    ptrdiff_t tmpStride = dmvrWidth ? dmvrWidth : width;

    int vFilterSize = bilinearMC ? NTAPS_BILINEAR : isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA;

    m_if.filterHor( compID, GET_OFFSETY( refPtr, refStride, -( ( vFilterSize >> 1 ) - 1 ) ), refStride, tmpBuf, tmpStride, width, height + vFilterSize - 1, xFrac, false,         chFmt, clpRng, bilinearMC, bilinearMC, useAltHpelIf );
    m_if.filterVer( compID, GET_OFFSETY( tmpBuf, tmpStride,    ( vFilterSize >> 1 ) - 1 ),   tmpStride, dstBuf, dstStride, width, height,                   yFrac, false, rndRes, chFmt, clpRng, bilinearMC, bilinearMC, useAltHpelIf );
  }

  if( bioApplied && compID == COMPONENT_Y )
  {
    const int   shift   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
    const int   xOffset = ( xFrac < 8 ) ? 1 : 0;
    const int   yOffset = ( yFrac < 8 ) ? 1 : 0;
    const Pel*  refPel  = refPtr + ( 1 - yOffset ) * refStride - xOffset;
    Pel*        dstPel  = m_bdofBlock[m_iRefListIdx] + 2 * dstStride;

    for( int h = 0; h < height; h++ )
    {
      dstPel[0]         = ( refPel[0        ] << shift ) - ( Pel ) IF_INTERNAL_OFFS;
      dstPel[width + 1] = ( refPel[width + 1] << shift ) - ( Pel ) IF_INTERNAL_OFFS;

      refPel += refStride;
      dstPel += dstStride;
    }

    refPel = refPtr - yOffset * refStride - xOffset;
    dstPel = m_bdofBlock[m_iRefListIdx] + dstStride;

    PaddBIO( refPel, dstPel, width, shift );
    
    refPel = refPtr + ( height + 1 - yOffset ) * refStride - xOffset;
    dstPel = m_bdofBlock[m_iRefListIdx] + ( height + 2 * BIO_EXTEND_SIZE ) * dstStride;

    PaddBIO( refPel, dstPel, width, shift );
  }
}

bool InterPrediction::isSubblockVectorSpreadOverLimit( int a, int b, int c, int d, int predType )
{
  int s4 = ( 4 << 11 );
  int filterTap = 6;

  if ( predType == 3 )
  {
    int refBlkWidth  = std::max( std::max( 0, 4 * a + s4 ), std::max( 4 * c, 4 * a + 4 * c + s4 ) ) - std::min( std::min( 0, 4 * a + s4 ), std::min( 4 * c, 4 * a + 4 * c + s4 ) );
    int refBlkHeight = std::max( std::max( 0, 4 * b ), std::max( 4 * d + s4, 4 * b + 4 * d + s4 ) ) - std::min( std::min( 0, 4 * b ), std::min( 4 * d + s4, 4 * b + 4 * d + s4 ) );
    refBlkWidth  = ( refBlkWidth >> 11 ) + filterTap + 3;
    refBlkHeight = ( refBlkHeight >> 11 ) + filterTap + 3;

    if ( refBlkWidth * refBlkHeight > ( filterTap + 9 ) * ( filterTap + 9 ) )
    {
      return true;
    }
  }
  else
  {
    int refBlkWidth  = std::max( 0, 4 * a + s4 ) - std::min( 0, 4 * a + s4 );
    int refBlkHeight = std::max( 0, 4 * b ) - std::min( 0, 4 * b );
    refBlkWidth  = ( refBlkWidth >> 11 ) + filterTap + 3;
    refBlkHeight = ( refBlkHeight >> 11 ) + filterTap + 3;
    if ( refBlkWidth * refBlkHeight > ( filterTap + 9 ) * ( filterTap + 5 ) )
    {
      return true;
    }

    refBlkWidth  = std::max( 0, 4 * c ) - std::min( 0, 4 * c );
    refBlkHeight = std::max( 0, 4 * d + s4 ) - std::min( 0, 4 * d + s4 );
    refBlkWidth  = ( refBlkWidth >> 11 ) + filterTap + 3;
    refBlkHeight = ( refBlkHeight >> 11 ) + filterTap + 3;
    if ( refBlkWidth * refBlkHeight > ( filterTap + 5 ) * ( filterTap + 9 ) )
    {
      return true;
    }
  }
  return false;
}

#define CALC_AFFINE_MV_ON_THE_FLY 0

void InterPrediction::xPredAffineBlk( const ComponentID&        compID,
                                      const PredictionUnit&     pu,
                                      const Picture*            refPic,
                                      const RefPicList          refPicList,
                                      PelUnitBuf&               dstPic,
                                      bool                      bi,
                                      const ClpRng&             clpRng,
                                      const std::pair<int, int> scalingRatio
                                      )
{
  const ChromaFormat chFmt = pu.chromaFormat;
  const int iScaleX = getComponentScaleX( compID, chFmt );
  const int iScaleY = getComponentScaleY( compID, chFmt );

  const int chromaScaleX = getChannelTypeScaleX( CH_C, chFmt );
  const int chromaScaleY = getChannelTypeScaleY( CH_C, chFmt );

  const int shiftX = 4 + iScaleX;
  const int shiftY = 4 + iScaleY;
  const int maskX  = ( 1 << shiftX ) - 1;
  const int maskY  = ( 1 << shiftY ) - 1;

  // get affine sub-block width and height
  const int width  = pu.lwidth();
  const int height = pu.lheight();

  static constexpr int blockWidth  = AFFINE_MIN_BLOCK_SIZE;
  static constexpr int blockHeight = AFFINE_MIN_BLOCK_SIZE;

  const int MVBUFFER_SIZE = ( width / AFFINE_MIN_BLOCK_SIZE ) >> chromaScaleX;

  const int cxWidth  = width  >> iScaleX;
  const int cxHeight = height >> iScaleY;
  const SPS &sps    = *pu.sps;
  const int iMvShift = 4;
  const int iOffset  = 8;
  const int iHorMax = ( pu.pps->getPicWidthInLumaSamples()  + iOffset -       pu.lx() - 1 ) << iMvShift;
  const int iHorMin = (  -(int)pu.cs->pcv->maxCUWidth       - iOffset -  (int)pu.lx() + 1 ) << iMvShift;
  const int iVerMax = ( pu.pps->getPicHeightInLumaSamples() + iOffset -       pu.ly() - 1 ) << iMvShift;
  const int iVerMin = (  -(int)pu.cs->pcv->maxCUHeight      - iOffset -  (int)pu.ly() + 1 ) << iMvShift;

  const int shift = MAX_CU_DEPTH;

  const Mv &affLT = pu.mv[refPicList][0];
  const Mv &affRT = pu.mv[refPicList][1];
  const Mv &affLB = pu.mv[refPicList][2];

  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;

  deltaMvHorX = ( affRT - affLT ).getHor() << ( shift - getLog2( width ) );
  deltaMvHorY = ( affRT - affLT ).getVer() << ( shift - getLog2( width ) );

  if( pu.affineType() == AFFINEMODEL_6PARAM )
  {
    deltaMvVerX = ( affLB - affLT ).getHor() << ( shift - getLog2( height ) );
    deltaMvVerY = ( affLB - affLT ).getVer() << ( shift - getLog2( height ) );
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY =  deltaMvHorX;
  }

#if CALC_AFFINE_MV_ON_THE_FLY
  const int mvScaleHor = affLT.getHor() << shift;
  const int mvScaleVer = affLT.getVer() << shift;

  static const int halfBW = AFFINE_MIN_BLOCK_SIZE >> 1;
  static const int halfBH = AFFINE_MIN_BLOCK_SIZE >> 1;

#endif
  const bool subblkMVSpreadOverLimit = InterPrediction::isSubblockVectorSpreadOverLimit( deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY, pu.interDir() );

  const bool refPicScaled = refPic->isRefScaled( pu.pps );

  PelBuf &dstBuf = dstPic.bufs[compID];

#if !CALC_AFFINE_MV_ON_THE_FLY
  const CMotionBuf mb       = pu.getMotionBuf();
  const MotionInfo* curMi   = mb.buf;
  const ptrdiff_t miStride  = mb.stride;
#endif
  Mv* chromaMvFld = m_storedMv;

  if( isLuma( compID ) )
  {
    GCC_WARNING_DISABLE_class_memaccess
    memset( m_storedMv, 0, MVBUFFER_SIZE * ( g_miScaling.scaleVer( height ) >> chromaScaleY ) * sizeof( Mv ) );
    GCC_WARNING_RESET
  }

  bool enablePROF = ( sps.getUsePROF() ) && ( compID == COMPONENT_Y );
  enablePROF &= (! pu.cs->picHeader->getDisProfFlag() );
  enablePROF &= !( ( pu.affineType() == AFFINEMODEL_6PARAM && affLT == affRT && affLT == affLB ) || ( pu.affineType() == AFFINEMODEL_4PARAM && affLT == affRT ) );
  enablePROF &= !subblkMVSpreadOverLimit;
  enablePROF &= !refPicScaled;

  bool isLast = enablePROF ? false : !bi;

  Pel gradX[36];
  Pel gradY[36];

  static constexpr int dstExtW = blockWidth  + PROF_BORDER_EXT_W * 2;
  static constexpr int dstExtH = blockHeight + PROF_BORDER_EXT_H * 2;
  PelBuf dstExtBuf( m_bdofBlock[0], dstExtW, dstExtH );

  int dMvScaleHor[16];
  int dMvScaleVer[16];

  if (enablePROF)
  {
    int* dMvH = dMvScaleHor;
    int* dMvV = dMvScaleVer;
    int quadHorX = deltaMvHorX << 2;
    int quadHorY = deltaMvHorY << 2;
    int quadVerX = deltaMvVerX << 2;
    int quadVerY = deltaMvVerY << 2;

    dMvH[0] = ((deltaMvHorX + deltaMvVerX) << 1) - ((quadHorX + quadVerX) << 1);
    dMvV[0] = ((deltaMvHorY + deltaMvVerY) << 1) - ((quadHorY + quadVerY) << 1);

    for (int w = 1; w < blockWidth; w++)
    {
      dMvH[w] = dMvH[w - 1] + quadHorX;
      dMvV[w] = dMvV[w - 1] + quadHorY;
    }

    dMvH += blockWidth;
    dMvV += blockWidth;
    for (int h = 1; h < blockHeight; h++)
    {
      for (int w = 0; w < blockWidth; w++)
      {
        dMvH[w] = dMvH[w - blockWidth] + quadVerX;
        dMvV[w] = dMvV[w - blockWidth] + quadVerY;
      }
      dMvH += blockWidth;
      dMvV += blockWidth;
    }

    const int mvShift  = 8;
    const int dmvLimit = ( 1 << 5 ) - 1;

    if (!roundIntVector)
    {
      for (int idx = 0; idx < blockWidth * blockHeight; idx++)
      {
        roundAffineMv(dMvScaleHor[idx], dMvScaleVer[idx], mvShift);
        dMvScaleHor[idx] = Clip3( -dmvLimit, dmvLimit, dMvScaleHor[idx] );
        dMvScaleVer[idx] = Clip3( -dmvLimit, dmvLimit, dMvScaleVer[idx] );
      }
    }
    else
    {
      int sz = blockWidth * blockHeight;
      roundIntVector(dMvScaleHor, sz, mvShift, dmvLimit);
      roundIntVector(dMvScaleVer, sz, mvShift, dmvLimit);
    }
  }
  
#if CALC_AFFINE_MV_ON_THE_FLY
  int mvhor, mvver;

  if( subblkMVSpreadOverLimit )
  {
    mvhor = mvScaleHor + deltaMvHorX * ( width >> 1 ) + deltaMvVerX * ( height >> 1 );
    mvver = mvScaleVer + deltaMvHorY * ( width >> 1 ) + deltaMvVerY * ( height >> 1 );
    roundAffineMv( mvhor, mvver, shift );
    mv.hor = mvhor; mv.ver = mvver;
    mv.clipToStorageBitDepth();
  }
#endif

  std::array<const Pel*, 2> refBuf{ nullptr, nullptr };
  std::array<ptrdiff_t, 2>  refBufStride{ 0, 0 };
  if( pu.pps->getNumSubPics() > 1 && pu.pps->getSubPicFromCU( pu ).getTreatedAsPicFlag() )
  {
    const int subPicIdx = pu.pps->getSubPicFromCU( pu ).getSubPicIdx();
    refBuf              = { refPic->getSubPicBufPtr   ( subPicIdx, compID, false ), 0 /*refPic->getSubPicBufPtr   ( subPicIdx, compID, true )*/ };
    refBufStride        = { refPic->getSubPicBufStride( subPicIdx, compID, false ), 0 /*refPic->getSubPicBufStride( subPicIdx, compID, true )*/ };
  }
  else
  {
    if( pu.sps->getUseWrapAround() )
    {
      refBuf       = { refPic->getRecoBufPtr   ( compID, false ), refPic->getRecoBufPtr   ( compID, true ) };
      refBufStride = { refPic->getRecoBufStride( compID, false ), refPic->getRecoBufStride( compID, true ) };
    }
    else
    {
      refBuf      [0] = refBuf      [1] = refPic->getRecoBufPtr   ( compID, false );
      refBufStride[0] = refBufStride[1] = refPic->getRecoBufStride( compID, false );
    }
  }

  const int puPosX = pu.blocks[compID].x, puPosY = pu.blocks[compID].y;

  // get prediction block by block
  for ( int h = 0; h < cxHeight; h += blockHeight )
  {
#if !CALC_AFFINE_MV_ON_THE_FLY
    const MotionInfo* lineMi = curMi;

#endif
    for ( int w = 0; w < cxWidth; w += blockWidth )
    {
      Position mvPos{ w >> 2, h >> 2 };
      
      int iMvScaleTmpHor;
      int iMvScaleTmpVer;

      if( isLuma( compID ) || chFmt == CHROMA_444 )
      {
#if CALC_AFFINE_MV_ON_THE_FLY
        if( !subblkMVSpreadOverLimit )
        {
          mvhor = mvScaleHor + deltaMvHorX * ( halfBW + w ) + deltaMvVerX * ( halfBH + h );
          mvver = mvScaleVer + deltaMvHorY * ( halfBW + w ) + deltaMvVerY * ( halfBH + h );
          roundAffineMv( mvhor, mvver, shift );
          mv.hor = mvhor; mv.ver = mvver;
          mv.clipToStorageBitDepth();
        }
#else   
        const Mv& mv = lineMi->mv[refPicList];

        iMvScaleTmpHor = mv.hor;
        iMvScaleTmpVer = mv.ver;
#endif

        if( chFmt != CHROMA_400 && chFmt != CHROMA_444 && ( ( ( mvPos.x ^ mvPos.y ) & 1 ) == 0 || chFmt != CHROMA_420 ) )
        {
          Mv &chromaMv = *GET_OFFSET( chromaMvFld, MVBUFFER_SIZE, mvPos.x >> chromaScaleX, mvPos.y >> chromaScaleY );
          chromaMv.hor += iMvScaleTmpHor;
          chromaMv.ver += iMvScaleTmpVer;
        }
      }
      else
      {
        Mv& mv = *GET_OFFSET( chromaMvFld, MVBUFFER_SIZE, mvPos.x, mvPos.y );
        iMvScaleTmpHor = mv.hor << ( 1 - ( chromaScaleX | chromaScaleY ) );
        iMvScaleTmpVer = mv.ver << ( 1 - ( chromaScaleX | chromaScaleY ) );
        roundAffineMv( iMvScaleTmpHor, iMvScaleTmpVer, 1 );
      }

      bool wrapRef = false;

      if ( refPic->isWrapAroundEnabled( pu.pps ) )
      {
        Mv tmpMv(iMvScaleTmpHor, iMvScaleTmpVer);
        wrapRef = wrapClipMv( tmpMv, Position( pu.Y().x + ( w << iScaleX ), pu.Y().y + ( h << iScaleY ) ), Size( blockWidth << iScaleX, blockHeight << iScaleY ), sps, *pu.pps );
        iMvScaleTmpHor = tmpMv.getHor();
        iMvScaleTmpVer = tmpMv.getVer();
      }
      else
      {
        iMvScaleTmpHor = std::min<int>( iHorMax, std::max<int>( iHorMin, iMvScaleTmpHor ) );
        iMvScaleTmpVer = std::min<int>( iVerMax, std::max<int>( iVerMin, iMvScaleTmpVer ) );
      }

      CHECKD( !refPic, "Should not be null" );
      if( refPicScaled )
      {
        xPredInterBlkRPR( scalingRatio, *pu.pps, compID, pu.chromaFormat, refPic, Mv( iMvScaleTmpHor, iMvScaleTmpVer ), pu.blocks[compID].offset( w, h ), blockWidth, blockHeight, dstPic.bufs[compID].buf + w + h * dstPic.bufs[compID].stride, dstPic.bufs[compID].stride, bi, wrapRef, clpRng, 2 );
        CHECKD( enablePROF, "PROF should be disabled with RPR" );
      }
      else
      {
        const int xInt  = iMvScaleTmpHor >> shiftX;
        const int xFrac = iMvScaleTmpHor &  maskX;
        const int yInt  = iMvScaleTmpVer >> shiftY;
        const int yFrac = iMvScaleTmpVer &  maskY;

        const Pel*      refBufPtr = refBuf      [wrapRef];
        const ptrdiff_t refStride = refBufStride[wrapRef];
        OFFSET( refBufPtr, refStride, puPosX + xInt + w, puPosY + yInt + h );

        Pel* dst;

        ptrdiff_t dstStride;

        if( enablePROF )
        {
          dst       = dstExtBuf.buf + 1 + dstExtBuf.stride;
          dstStride = dstExtBuf.stride;
        }
        else
        {
          dst       = dstBuf.bufAt( w, h );
          dstStride = dstBuf.stride;
        }

        if( xFrac && yFrac )
        {
          m_if.filter4x4( compID, refBufPtr, refStride, dst, dstStride, 4, 4, xFrac, yFrac, isLast, chFmt, clpRng );
        }
        else if( yFrac == 0 )
        {
          m_if.filterHor( compID, refBufPtr, refStride, dst, dstStride, blockWidth, blockHeight, xFrac, isLast, chFmt, clpRng );
        }
        else// if( xFrac == 0 )
        {
          m_if.filterVer( compID, refBufPtr, refStride, dst, dstStride, blockWidth, blockHeight, yFrac, true, isLast, chFmt, clpRng );
        }

        if( enablePROF )
        {
          const Pel shift   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
          const int xOffset = xFrac >> 3;
          const int yOffset = yFrac >> 3;

          CHECKD( shift < 0, "Shift need to be positive!" );
          static_assert( PROF_BORDER_EXT_H == BIO_EXTEND_SIZE, "PROF and BIO extension need to be equal!" );
          static_assert( PROF_BORDER_EXT_W == BIO_EXTEND_SIZE, "PROF and BIO extension need to be equal!" );

          const ptrdiff_t refOffset = ( blockHeight + 1 ) * refStride;
          const ptrdiff_t dstOffset = ( blockHeight + 1 ) * dstStride;

          const Pel* refPel = refBufPtr - ( 1 - yOffset ) * refStride + xOffset - 1;
                Pel* dstPel = dst - 1 - dstStride;

          PaddBIO( refPel,             dstPel,             blockWidth, shift );
          PaddBIO( refPel + refOffset, dstPel + dstOffset, blockWidth, shift );

          refPel = refBufPtr + yOffset * refStride + xOffset;
          dstPel = dst;
          for( int ph = 0; ph < 4; ph++, refPel += refStride, dstPel += dstStride )
          {
            dstPel[        -1] = ( refPel[        -1] << shift ) - Pel( IF_INTERNAL_OFFS );
            dstPel[blockWidth] = ( refPel[blockWidth] << shift ) - Pel( IF_INTERNAL_OFFS );
          }

          profGradFilter( dst, dstStride, blockWidth, blockHeight, AFFINE_MIN_BLOCK_SIZE, gradX, gradY, clpRng.bd );

          Pel *dstY = dstBuf.buf + w + dstBuf.stride * h;
          const Pel offset   = ( 1 << ( shift- 1 ) ) + IF_INTERNAL_OFFS;
          applyPROF[bi]( dstY, dstBuf.stride, dst, gradX, gradY, dMvScaleHor, dMvScaleVer, shift, offset, clpRng );
        }
      }
#if !CALC_AFFINE_MV_ON_THE_FLY

      INCX( lineMi, miStride );
#endif
    }
#if !CALC_AFFINE_MV_ON_THE_FLY

    INCY( curMi, miStride );
#endif
  }
}

void InterPrediction::applyBiOptFlow( const PredictionUnit &pu,
                                      const PelUnitBuf &    yuvSrc0,
                                      const PelUnitBuf &    yuvSrc1,
                                      const int &           refIdx0,
                                      const int &           refIdx1,
                                      PelUnitBuf &          yuvDst,
                                      const BitDepths &     clipBitDepths )
{
  const int height  = yuvDst.Y().height;
  const int width   = yuvDst.Y().width;
  int       heightG = height + 2 * BIO_EXTEND_SIZE;
  int       widthG  = width  + 2 * BIO_EXTEND_SIZE;

  Pel *gradX0 = m_gradX0;
  Pel *gradX1 = m_gradX1;
  Pel *gradY0 = m_gradY0;
  Pel *gradY1 = m_gradY1;

  int        stridePredMC = width + BIO_ALIGN_SIZE;
  const Pel *srcY0        = m_bdofBlock[0] + stridePredMC;
  const Pel *srcY1        = m_bdofBlock[1] + stridePredMC;

  Pel *           dstY      = yuvDst.Y().buf;
  const ptrdiff_t dstStride = yuvDst.Y().stride;

  const int       bitDepth  = clipBitDepths.recon[toChannelType( COMPONENT_Y )];

  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    Pel *dstTempPtr = m_bdofBlock[refList] + stridePredMC;
    Pel *gradY      = ( refList == 0 ) ? m_gradY0 : m_gradY1;
    Pel *gradX      = ( refList == 0 ) ? m_gradX0 : m_gradX1;
    BioGradFilter( dstTempPtr, stridePredMC, widthG, heightG, width + BIO_ALIGN_SIZE, gradX, gradY, bitDepth );
  }

  const ClpRng &clpRng   = pu.slice->clpRng( COMPONENT_Y );
  const int     shiftNum = IF_INTERNAL_PREC + 1 - bitDepth;
  const int     offset   = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;
  const int     limit    = ( 1 << 4 ) - 1;


  BiOptFlow( srcY0,
             srcY1,
             gradX0,
             gradX1,
             gradY0,
             gradY1,
             width,
             height,
             dstY,
             dstStride,
             shiftNum,
             offset,
             limit,
             clpRng,
             bitDepth
            );
}

void InterPrediction::xWeightedAverage(const PredictionUnit& pu, const PelUnitBuf& pcYuvSrc0, const PelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const BitDepths& clipBitDepths, const ClpRngs& clpRngs, const bool& bioApplied )
{
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];

  CHECKD( !( iRefIdx0 >= 0 && iRefIdx1 >= 0 ), "xWeightedAverage should only be called for BI-predicted blocks!" );

  if( pu.BcwIdx() != BCW_DEFAULT && !pu.ciipFlag() )
  {
    CHECK( bioApplied, "Bcw is disallowed with BIO" );
    pcYuvDst.addWeightedAvg( pcYuvSrc0, pcYuvSrc1, clpRngs, g_BcwInternBcw[pu.BcwIdx()] );
    return;
  }

  if( bioApplied )
  {
    applyBiOptFlow( pu, pcYuvSrc0, pcYuvSrc1, iRefIdx0, iRefIdx1, pcYuvDst, clipBitDepths );
  }

  pcYuvDst.addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs, bioApplied );
}


void InterPrediction::motionCompensation( PredictionUnit &pu, PelUnitBuf &predBuf, const bool luma, const bool chroma )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_MOTCOMP, *pu.cs, luma ? CH_L: CH_C );
  m_currCuArea = pu;

  if( pu.slice->getSliceType() != I_SLICE && pu.slice->getRefPic( REF_PIC_LIST_0, 0 )->subPictures.size() > 1 )
  {
    clipMv = clipMvInSubpic;
  }
  else
  {
    clipMv = clipMvInPic;
  }

  if( CU::isIBC( pu ) )
  {
    CHECK( !luma, "IBC only for Chroma is not allowed." );
    xIntraBlockCopy( pu, predBuf, COMPONENT_Y );
    if( chroma )
    {
      xIntraBlockCopy( pu, predBuf, COMPONENT_Cb );
      xIntraBlockCopy( pu, predBuf, COMPONENT_Cr );
    }
    return;
  }

  // else, go with regular MC below
  const PPS &pps            = *pu.pps;

  CHECKD( !pu.affineFlag() && pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 && ( pu.lwidth() + pu.lheight() == 12 ), "invalid 4x8/8x4 bi-predicted blocks" );
  WPScalingParam *wp0;
  WPScalingParam *wp1;
  int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
  int refIdx1 = pu.refIdx[REF_PIC_LIST_1];
  pu.slice->getWpScaling( REF_PIC_LIST_0, refIdx0, wp0 );
  pu.slice->getWpScaling( REF_PIC_LIST_1, refIdx1, wp1 );
  bool bioApplied    = false;
  const Slice &slice = *pu.slice;

  if( pu.sps->getUseBIO() && ( !pu.cs->picHeader->getDisBdofFlag() ) )
  {

    if( pu.affineFlag() || m_subPuMC || pu.ciipFlag() || pu.smvdMode() || ( pu.sps->getUseBcw() && pu.BcwIdx() != BCW_DEFAULT ) )
    {
      bioApplied = false;
    }
    else
    {
      const bool biocheck0 = !((wp0[COMPONENT_Y].bPresentFlag || wp0[COMPONENT_Cb].bPresentFlag || wp0[COMPONENT_Cr].bPresentFlag || wp1[COMPONENT_Y].bPresentFlag || wp1[COMPONENT_Cb].bPresentFlag || wp1[COMPONENT_Cr].bPresentFlag) && slice.getSliceType() == B_SLICE);
      const bool biocheck1 = !( pps.getUseWP() && slice.getSliceType() == P_SLICE );

      if( biocheck0 && biocheck1 && PU::isBiPredFromDifferentDirEqDistPoc( pu ) && pu.Y().height >= 8 && pu.Y().width >= 8 && pu.Y().area() >= 128 )
      {
        bioApplied = true;
      }
    }
  }
    
  bool dmvrApplied = !m_subPuMC && PU::checkDMVRCondition( pu );
  bool refIsScaled = ( refIdx0 < 0 ? false : pu.slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->isRefScaled( pu.pps ) ) ||
                     ( refIdx1 < 0 ? false : pu.slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->isRefScaled( pu.pps ) );

  dmvrApplied = dmvrApplied && !refIsScaled;
  bioApplied  = bioApplied  && !refIsScaled;

  if( pu.mergeType() != MRG_TYPE_SUBPU_ATMVP && bioApplied && !dmvrApplied )
  {
    xSubPuBio( pu, predBuf );
  }
  else if( dmvrApplied )
  {
    pu.setDmvrCondition( true );
    xProcessDMVR( pu, predBuf, slice.clpRngs(), bioApplied );
  }
  else if( pu.mergeType() == MRG_TYPE_SUBPU_ATMVP )
  {
    xSubPuMC( pu, predBuf );
  }
  else if( xCheckIdenticalMotion( pu ) )
  {
    xPredInterUni( pu, REF_PIC_LIST_0, predBuf, false, false , true, true );
  }
  else
  {
    CHECKD( bioApplied, "BIO should not be applied here!" );
    xPredInterBi( pu, predBuf );
  }
}

void InterPrediction::motionCompensationGeo( PredictionUnit &pu, PelUnitBuf &predBuf )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_MOTCOMP, *pu.cs, CH_L );

  if( pu.slice->getSliceType() != I_SLICE && pu.slice->getRefPic( REF_PIC_LIST_0, 0 )->subPictures.size() > 1 )
  {
    clipMv = clipMvInSubpic;
  }
  else
  {
    clipMv = clipMvInPic;
  }

  const UnitArea localUnitArea( pu.cs->area.chromaFormat, Area( 0, 0, pu.lwidth(), pu.lheight() ) );

  PelUnitBuf tmpGeoBuf0 = isChromaEnabled( pu.chromaFormat ) ? PelUnitBuf( pu.chromaFormat, PelBuf( m_acYuvPred[0], localUnitArea.Y() ), PelBuf( m_acYuvPred[1], localUnitArea.Cb() ), PelBuf( m_acYuvPred[2], localUnitArea.Cr() ) ) : PelUnitBuf( pu.chromaFormat, PelBuf( m_acYuvPred[0], localUnitArea.Y() ) );

  uint8_t locInterDir = pu.interDirrefIdxGeo0() >> 4;
  CHECKD( !( locInterDir == 1 || locInterDir == 2 ), "Should not happen" );
  pu.mv  [REF_PIC_LIST_0][0] = locInterDir == 1 ? pu.mv[0][1] : Mv();
  pu.mv  [REF_PIC_LIST_1][0] = locInterDir == 1 ? Mv() : pu.mv[0][1];
  pu.refIdx [REF_PIC_LIST_0] = locInterDir == 1 ? pu.interDirrefIdxGeo0() & 15 : -1;
  pu.refIdx [REF_PIC_LIST_1] = locInterDir == 1 ? -1 : pu.interDirrefIdxGeo0() & 15;
  pu.mvpIdx [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx [REF_PIC_LIST_1] = NOT_VALID;
  motionCompensation( pu, tmpGeoBuf0, true, isChromaEnabled( pu.chromaFormat ) );

  locInterDir = pu.interDirrefIdxGeo1() >> 4;
  CHECKD( !( locInterDir == 1 || locInterDir == 2 ), "Should not happen" );
  pu.mv  [REF_PIC_LIST_0][0] = locInterDir == 1 ? pu.mv[1][1] : Mv();
  pu.mv  [REF_PIC_LIST_1][0] = locInterDir == 1 ? Mv() : pu.mv[1][1];
  pu.refIdx [REF_PIC_LIST_0] = locInterDir == 1 ? pu.interDirrefIdxGeo1() & 15 : -1;
  pu.refIdx [REF_PIC_LIST_1] = locInterDir == 1 ? -1 : pu.interDirrefIdxGeo1() & 15;
  pu.mvpIdx [REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx [REF_PIC_LIST_1] = NOT_VALID;
  motionCompensation( pu, predBuf, true, isChromaEnabled( pu.chromaFormat ) );

  const uint8_t splitDir = pu.geoSplitDir;
  weightedGeoBlk( pu, splitDir, isChromaEnabled( pu.chromaFormat ) ? MAX_NUM_CHANNEL_TYPE : CHANNEL_TYPE_LUMA, predBuf, tmpGeoBuf0, predBuf );
}

void InterPrediction::weightedGeoBlk( PredictionUnit &pu, const uint8_t splitDir, int32_t channel, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  if( channel == CHANNEL_TYPE_LUMA )
  {
    m_if.weightedGeoBlk( pu, pu.lumaSize().width, pu.lumaSize().height, COMPONENT_Y, splitDir, predDst, predSrc0, predSrc1, pu.slice->clpRngs() );
  }
  else if( channel == CHANNEL_TYPE_CHROMA )
  {
    m_if.weightedGeoBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, predDst, predSrc0, predSrc1, pu.slice->clpRngs() );
    m_if.weightedGeoBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, predDst, predSrc0, predSrc1, pu.slice->clpRngs() );
  }
  else
  {
    m_if.weightedGeoBlk( pu, pu.lumaSize().width,   pu.lumaSize().height,   COMPONENT_Y,  splitDir, predDst, predSrc0, predSrc1, pu.slice->clpRngs() );
    if( isChromaEnabled( pu.chromaFormat ) )
    {
      m_if.weightedGeoBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cb, splitDir, predDst, predSrc0, predSrc1, pu.slice->clpRngs() );
      m_if.weightedGeoBlk( pu, pu.chromaSize().width, pu.chromaSize().height, COMPONENT_Cr, splitDir, predDst, predSrc0, predSrc1, pu.slice->clpRngs() );
    }
  }
}


void InterPrediction::xPrefetchPad( PredictionUnit& pu, PelUnitBuf &pcPad, RefPicList refId, bool forLuma )
{
  int width, height;
  Mv cMv;

  const Picture* refPic = pu.slice->getRefPic( refId, pu.refIdx[refId] )->unscaledPic;

  static constexpr int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  const bool wrapRefEnbld = refPic->isWrapAroundEnabled( pu.pps );
  const bool subPicAsPic  = pu.pps->getNumSubPics() > 1 && pu.pps->getSubPicFromCU( pu ).getTreatedAsPicFlag();

  const ChannelType chType = forLuma ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA;

  int filtersize = isLuma( chType ) ? NTAPS_LUMA : NTAPS_CHROMA;
  cMv = pu.mv[refId][0];
  
  const int mvshiftTempHor        = mvShift + getChannelTypeScaleX( chType, pu.chromaFormat );
  const int mvshiftTempVer        = mvShift + getChannelTypeScaleY( chType, pu.chromaFormat );
  cMv                      += Mv( -( ( ( filtersize >> 1 ) - 1 ) << mvshiftTempHor ),
                                  -( ( ( filtersize >> 1 ) - 1 ) << mvshiftTempVer ) );
  bool wrapRef = false;

  if( wrapRefEnbld )
  {
    wrapRef = wrapClipMv( cMv, pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );
  }
  else
  {
    clipMv( cMv, pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );
  }

  cMv.hor >>= mvshiftTempHor;
  cMv.ver >>= mvshiftTempVer;

  if( isLuma( chType ) )
  {
    pcPad.bufs[COMPONENT_Y]
      .stride  = pcPad.bufs[COMPONENT_Y].width + ( 2 * DMVR_NUM_ITERATION ) + filtersize;
    width      = pcPad.bufs[COMPONENT_Y].width;
    height     = pcPad.bufs[COMPONENT_Y].height;
    ptrdiff_t
      offset   = DMVR_NUM_ITERATION * ( pcPad.bufs[COMPONENT_Y].stride + 1 );

    width      += filtersize - 1;
    height     += filtersize - 1;

    CPelBuf refBuf = subPicAsPic ? refPic->getSubPicBuf( pu.pps->getSubPicFromCU( pu ).getSubPicIdx(), COMPONENT_Y, wrapRef ) : refPic->getRecoBuf( COMPONENT_Y, wrapRef );

    Position   Rec_offset = pu.lumaPos().offset( cMv.hor, cMv.ver );
    const Pel* refBufPtr  = refBuf.bufAt( Rec_offset );

    PelBuf& dstBuf = pcPad.Y();
    prefetchPad[0]( refBufPtr, refBuf.stride, dstBuf.buf + offset, dstBuf.stride, width, height );
  }
  else
  {
    pcPad.bufs[COMPONENT_Cb]
      .stride  = pcPad.bufs[COMPONENT_Cb].width + ( 2 * DMVR_NUM_ITERATION ) + filtersize;
    pcPad.bufs[COMPONENT_Cr]
      .stride  = pcPad.bufs[COMPONENT_Cb].stride;
    width      = pcPad.bufs[COMPONENT_Cb].width;
    height     = pcPad.bufs[COMPONENT_Cb].height;
    ptrdiff_t
      offsetCb = DMVR_NUM_ITERATION * ( pcPad.bufs[COMPONENT_Cb].stride + 1 );
    ptrdiff_t
      offsetCr = DMVR_NUM_ITERATION * ( pcPad.bufs[COMPONENT_Cr].stride + 1 );

    width      += filtersize - 1;
    height     += filtersize - 1;

    CPelBuf refBufCb = subPicAsPic ? refPic->getSubPicBuf( pu.pps->getSubPicFromCU( pu ).getSubPicIdx(), COMPONENT_Cb, wrapRef ) : refPic->getRecoBuf( COMPONENT_Cb, wrapRef );
    CPelBuf refBufCr = subPicAsPic ? refPic->getSubPicBuf( pu.pps->getSubPicFromCU( pu ).getSubPicIdx(), COMPONENT_Cr, wrapRef ) : refPic->getRecoBuf( COMPONENT_Cr, wrapRef );

    Position   Rec_offset     = pu.blocks[COMPONENT_Cb].pos().offset( cMv.hor, cMv.ver );
    const Pel* refBufPtr  [2] = { refBufCb.bufAt( Rec_offset ), refBufCr.bufAt( Rec_offset ) };
    const ptrdiff_t stride[2] = { refBufCb.stride, refBufCr.stride };
          Pel* dstBufPtr  [2] = { pcPad.Cb().buf + offsetCb, pcPad.Cr().buf + offsetCr };
    const ptrdiff_t dstStr[2] = { pcPad.Cb().stride, pcPad.Cr().stride };

    const int idx = getChannelTypeScaleY( CH_C, pu.chromaFormat );

    prefetchPad[1+idx]( refBufPtr[0], stride[0], dstBufPtr[0], dstStr[0], width, height );
    prefetchPad[1+idx]( refBufPtr[1], stride[1], dstBufPtr[1], dstStr[1], width, height );
  }
}

inline int32_t div_for_maxq7(int64_t N, int64_t D)
{
  int32_t sign, q;
  sign = 0;
  if (N < 0)
  {
    sign = 1;
    N = -N;
  }

  q = 0;
  D = (D << 3);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  D = (D >> 1);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  if (N >= (D >> 1))
    q++;

  if (sign)
    return (-q);
  return(q);
}

void xSubPelErrorSrfc(uint64_t *sadBuffer, int32_t *deltaMv)
{
  int64_t numerator, denominator;
  int32_t mvDeltaSubPel;
  int32_t mvSubPelLvl = 4;/*1: half pel, 2: Qpel, 3:1/8, 4: 1/16*/
                                                        /*horizontal*/
    numerator = (int64_t)((sadBuffer[1] - sadBuffer[3]) << mvSubPelLvl);
    denominator = (int64_t)((sadBuffer[1] + sadBuffer[3] - (sadBuffer[0] << 1)));

    if (0 != denominator)
    {
      if ((sadBuffer[1] != sadBuffer[0]) && (sadBuffer[3] != sadBuffer[0]))
      {
        mvDeltaSubPel = div_for_maxq7(numerator, denominator);
        deltaMv[0] = (mvDeltaSubPel);
      }
      else
      {
        if (sadBuffer[1] == sadBuffer[0])
        {
          deltaMv[0] = -8;// half pel
        }
        else
        {
          deltaMv[0] = 8;// half pel
        }
      }
    }

    /*vertical*/
    numerator = (int64_t)((sadBuffer[2] - sadBuffer[4]) << mvSubPelLvl);
    denominator = (int64_t)((sadBuffer[2] + sadBuffer[4] - (sadBuffer[0] << 1)));
    if (0 != denominator)
    {
      if ((sadBuffer[2] != sadBuffer[0]) && (sadBuffer[4] != sadBuffer[0]))
      {
        mvDeltaSubPel = div_for_maxq7(numerator, denominator);
        deltaMv[1] = (mvDeltaSubPel);
      }
      else
      {
        if (sadBuffer[2] == sadBuffer[0])
        {
          deltaMv[1] = -8;// half pel
        }
        else
        {
          deltaMv[1] = 8;// half pel
        }
      }
    }

  return;
}

void InterPrediction::xBIPMVRefine( DistParam &cDistParam, const Pel *pRefL0, const Pel *pRefL1, Distortion& minCost, int16_t *deltaMV, Distortion *pSADsArray)
{
  const ptrdiff_t refStride = m_biLinearBufStride;

  const Pel *pRefL0Orig = pRefL0;
  const Pel *pRefL1Orig = pRefL1;

  for (int ver = -2; ver <= 2; ver++) {
    const int initHor = -2;
    const ptrdiff_t offset = initHor + ver * refStride;
    pRefL0 = pRefL0Orig + offset;
    pRefL1 = pRefL1Orig - offset;
    cDistParam.org.buf = pRefL0;
    cDistParam.cur.buf = pRefL1;

    cDistParam.distFuncX5(cDistParam, pSADsArray, ver != 0);

    for (int hor = -2; hor <= 2; hor++, pSADsArray++) {
      uint64_t cost = *pSADsArray;

      if (cost < minCost) {
        minCost = cost;
        deltaMV[0] = hor;
        deltaMV[1] = ver;
      }
    }
  }
}

void InterPrediction::xFinalPaddedMCForDMVR(PredictionUnit& pu, PelUnitBuf &pcYuvSrc0, PelUnitBuf &pcYuvSrc1, PelUnitBuf &pcPad0, PelUnitBuf &pcPad1, const bool bioApplied, const Mv mergeMV[NUM_REF_PIC_LIST_01] )
{
  ptrdiff_t offset;
  int deltaIntMvX, deltaIntMvY;

  /*always high precision MVs are used*/
  const int mvShift      = MV_FRACTIONAL_BITS_INTERNAL;
  const ClpRngs clp      = pu.slice->clpRngs();
  const int numValidComp = getNumberValidComponents( pu.chromaFormat );

  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    PelUnitBuf &pcYUVTemp = k == 0 ? pcYuvSrc0 : pcYuvSrc1;
    PelUnitBuf &pcPadTemp = k == 0 ? pcPad0    : pcPad1;

    RefPicList refId = (RefPicList)k;
    Mv cMv = pu.mv[refId][0];
    m_iRefListIdx = refId;
    const Picture* refPic = pu.slice->getRefPic( refId, pu.refIdx[refId] )->unscaledPic;
    Mv cMvClipped( cMv );
    clipMv( cMvClipped, pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );
    const bool wrapRef = pu.pps->getUseWrapAround() && wrapClipMv( cMvClipped, pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );

    Mv startMv = mergeMV[refId];

    for( int compID = 0; compID < numValidComp; compID++ )
    {
      const int mvshiftTempHor = mvShift + getComponentScaleX( (ComponentID)compID, pu.chromaFormat );
      const int mvshiftTempVer = mvShift + getComponentScaleY( (ComponentID)compID, pu.chromaFormat );
      deltaIntMvX = ( cMv.getHor() >> mvshiftTempHor ) - ( startMv.getHor() >> mvshiftTempHor );
      deltaIntMvY = ( cMv.getVer() >> mvshiftTempVer ) - ( startMv.getVer() >> mvshiftTempVer );

      if( deltaIntMvX || deltaIntMvY )
      {
        ptrdiff_t pcPadstride       = pcPadTemp.bufs[compID].stride;
        const int leftPixelExtra    = compID == COMPONENT_Y ? ( NTAPS_LUMA >> 1 ) - 1 : ( NTAPS_CHROMA >> 1 ) - 1;

        CHECKD( ( abs( deltaIntMvX ) > DMVR_NUM_ITERATION ) || ( abs( deltaIntMvY ) > DMVR_NUM_ITERATION ), "not expected DMVR movement" );

        offset  = ( DMVR_NUM_ITERATION + leftPixelExtra ) * ( pcPadstride + 1 );
        offset += ( deltaIntMvY ) * pcPadstride;
        offset += ( deltaIntMvX );
        Pel *srcBufPelPtr = pcPadTemp.bufs[compID].buf + offset;

        xPredInterBlk<true , false>( ComponentID( compID ), pu, refPic, cMvClipped, pcYUVTemp.bufs[compID], true, clp, bioApplied, false, wrapRef, 0, 0, 0, srcBufPelPtr, pcPadstride );
      }
      else
      {
        xPredInterBlk<false, false>( ComponentID( compID ), pu, refPic, cMvClipped, pcYUVTemp.bufs[compID], true, clp, bioApplied, false, wrapRef, 0, 0, 0 );
      }
    }
  }
}

void xDMVRSubPixelErrorSurface( bool notZeroCost, int16_t *totalDeltaMV, int16_t *deltaMV, Distortion*pSADsArray )
{
  int sadStride = ( ( ( 2 * DMVR_NUM_ITERATION ) + 1 ) );
  uint64_t sadbuffer[5];

  if( notZeroCost && ( abs( totalDeltaMV[0] ) != ( 2 << MV_FRACTIONAL_BITS_INTERNAL ) ) && ( abs( totalDeltaMV[1] ) != ( 2 << MV_FRACTIONAL_BITS_INTERNAL ) ) )
  {
    int32_t tempDeltaMv[2] = { 0,0 };
    sadbuffer[0] = pSADsArray[0];
    sadbuffer[1] = pSADsArray[-1];
    sadbuffer[2] = pSADsArray[-sadStride];
    sadbuffer[3] = pSADsArray[1];
    sadbuffer[4] = pSADsArray[sadStride];
    xSubPelErrorSrfc(sadbuffer, tempDeltaMv);
    totalDeltaMV[0] += tempDeltaMv[0];
    totalDeltaMV[1] += tempDeltaMv[1];
  }
}

void InterPrediction::xinitMC( PredictionUnit& pu, const ClpRngs &clpRngs )
{
  /*use merge MV as starting MV*/
  Mv mergeMVL0(pu.mv[REF_PIC_LIST_0][0]);
  Mv mergeMVL1(pu.mv[REF_PIC_LIST_1][0]);

  /*Clip the starting MVs*/
  clipMv( mergeMVL0, pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );
  clipMv( mergeMVL1, pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );

  const bool wrapRefL0 = pu.pps->getUseWrapAround() && wrapClipMv( mergeMVL0, pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );
  const bool wrapRefL1 = pu.pps->getUseWrapAround() && wrapClipMv( mergeMVL1, pu.lumaPos(), pu.lumaSize(), *pu.sps, *pu.pps );

  static constexpr int sizeExt = DMVR_NUM_ITERATION << 1;

  const int extWidth  = pu.lwidth()  + sizeExt;
  const int extHeight = pu.lheight() + sizeExt;

  /*L0 MC for refinement*/
  {
    const Picture* refPic = pu.slice->getRefPic( L0, pu.refIdx[L0] )->unscaledPic;

    PelBuf yuvPredTempL0( m_cYuvPredTempDMVRL0, m_biLinearBufStride, extWidth, extHeight );

    mergeMVL0.hor -= ( DMVR_NUM_ITERATION << MV_FRACTIONAL_BITS_INTERNAL );
    mergeMVL0.ver -= ( DMVR_NUM_ITERATION << MV_FRACTIONAL_BITS_INTERNAL );

    xPredInterBlk<false, true>( COMPONENT_Y, pu, refPic, mergeMVL0, yuvPredTempL0, true, clpRngs, false, false, wrapRefL0, extWidth, extHeight, true );
  }

  /*L1 MC for refinement*/
  {
    const Picture* refPic = pu.slice->getRefPic( L1, pu.refIdx[L1] )->unscaledPic;

    PelBuf yuvPredTempL1( m_cYuvPredTempDMVRL1, m_biLinearBufStride, extWidth, extHeight );

    mergeMVL1.hor -= ( DMVR_NUM_ITERATION << MV_FRACTIONAL_BITS_INTERNAL );
    mergeMVL1.ver -= ( DMVR_NUM_ITERATION << MV_FRACTIONAL_BITS_INTERNAL );

    xPredInterBlk<false, true>( COMPONENT_Y, pu, refPic, mergeMVL1, yuvPredTempL1, true, clpRngs, false, false, wrapRefL1, extWidth, extHeight, true );
  }
}

void InterPrediction::xProcessDMVR( PredictionUnit& pu, PelUnitBuf &pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied )
{
  /*Always High Precision*/
  static constexpr int mvShift  = MV_FRACTIONAL_BITS_INTERNAL;
         const     int mvShiftX = mvShift + getChannelTypeScaleX( CH_C, pu.chromaFormat );
         const     int mvShiftY = mvShift + getChannelTypeScaleY( CH_C, pu.chromaFormat );

  /*use merge MV as starting MV*/
  Mv mergeMv[] = { pu.mv[REF_PIC_LIST_0][0] , pu.mv[REF_PIC_LIST_1][0] };

  m_biLinearBufStride = ( pu.lwidth() + ( 2 * DMVR_NUM_ITERATION ) );

  xinitMC( pu, clpRngs );

  int dy = std::min<int>( pu.lumaSize().height, DMVR_SUBCU_HEIGHT );
  int dx = std::min<int>( pu.lumaSize().width,  DMVR_SUBCU_WIDTH );

  Position puPos = pu.lumaPos();
  BitDepths bds  = pu.sps->getBitDepths();

  int  bioEnabledThres = ( 2 * dy * dx );
  bool bioAppliedSubblk;
  {
    int num = 0;
    
    int scaleX = getComponentScaleX( COMPONENT_Cb, pu.chromaFormat );
    int scaleY = getComponentScaleY( COMPONENT_Cb, pu.chromaFormat );
    // point mc buffer to cetre point to avoid multiplication to reach each iteration to the begining
    Pel *biLinearPredL0 = m_cYuvPredTempDMVRL0 + ( DMVR_NUM_ITERATION * m_biLinearBufStride ) + DMVR_NUM_ITERATION;
    Pel *biLinearPredL1 = m_cYuvPredTempDMVRL1 + ( DMVR_NUM_ITERATION * m_biLinearBufStride ) + DMVR_NUM_ITERATION;
    
    CodingUnit      subCu;
    PredictionUnit& subPu = subCu;

    subPu                   = pu;
    subPu.UnitArea::operator=( UnitArea( pu.chromaFormat, Area( puPos.x, puPos.y, dx, dy ) ) );

    PelUnitBuf cYuvRefBuffDMVRL0 = isChromaEnabled( pu.chromaFormat ) ? PelUnitBuf( pu.chromaFormat, PelBuf( m_cRefSamplesDMVRL0[0], subPu.Y() ), PelBuf( m_cRefSamplesDMVRL0[1], subPu.Cb() ), PelBuf( m_cRefSamplesDMVRL0[2], subPu.Cr() ) ) : PelUnitBuf( pu.chromaFormat, PelBuf( m_cRefSamplesDMVRL0[0], subPu.Y() ) );
    PelUnitBuf cYuvRefBuffDMVRL1 = isChromaEnabled( pu.chromaFormat ) ? PelUnitBuf( pu.chromaFormat, PelBuf( m_cRefSamplesDMVRL1[0], subPu.Y() ), PelBuf( m_cRefSamplesDMVRL1[1], subPu.Cb() ), PelBuf( m_cRefSamplesDMVRL1[2], subPu.Cr() ) ) : PelUnitBuf( pu.chromaFormat, PelBuf( m_cRefSamplesDMVRL1[0], subPu.Y() ) );

    PelUnitBuf srcPred1 = isChromaEnabled( pu.chromaFormat ) ? PelUnitBuf( pu.chromaFormat, PelBuf( m_acYuvPred[0], subPu.Y() ), PelBuf( m_acYuvPred[1], subPu.Cb() ), PelBuf( m_acYuvPred[2], subPu.Cr() ) ) : PelUnitBuf( pu.chromaFormat, PelBuf( m_acYuvPred[0], subPu.Y() ) );

    DistParam cDistParam;
    m_pcRdCost->setDistParam( cDistParam, nullptr, nullptr, m_biLinearBufStride, m_biLinearBufStride, clpRngs.bd, dx, dy, 1 );
    
    PelUnitBuf subPredBuf = pcYuvDst.subBuf( UnitAreaRelative( pu, subPu ) );
    const ptrdiff_t dstStride[MAX_NUM_COMPONENT] = { pcYuvDst.bufs[COMPONENT_Y].stride,
                                                     isChromaEnabled(pu.chromaFormat) ? pcYuvDst.bufs[COMPONENT_Cb].stride : 0,
                                                     isChromaEnabled(pu.chromaFormat) ? pcYuvDst.bufs[COMPONENT_Cr].stride : 0};
    for( int y = puPos.y, yStart = 0; y < ( puPos.y + pu.lumaSize().height ); y = y + dy, yStart = yStart + dy )
    {
      for( int x = puPos.x, xStart = 0; x < ( puPos.x + pu.lumaSize().width ); x = x + dx, xStart = xStart + dx )
      {
        subPu.mv[0][0]    = pu.mv[0][0]; subPu.mv[1][0]    = pu.mv[1][0];
        subPu.blocks[0].x = x;           subPu.blocks[0].y = y;
        if( isChromaEnabled( subPu.chromaFormat ) )
        {
          subPu.blocks[1].x = x >> scaleX; subPu.blocks[1].y = y >> scaleY;
          subPu.blocks[2].x = x >> scaleX; subPu.blocks[2].y = y >> scaleY;
        }

        Distortion *pSADsArray = &m_SADsArray[( ( ( 2 * DMVR_NUM_ITERATION ) + 1 ) * ( ( 2 * DMVR_NUM_ITERATION ) + 1 ) ) >> 1];

        Pel *biPredSubPuL0 = biLinearPredL0 + xStart + yStart * m_biLinearBufStride;
        Pel *biPredSubPuL1 = biLinearPredL1 + xStart + yStart * m_biLinearBufStride;

        cDistParam.cur.buf = biPredSubPuL0;
        cDistParam.org.buf = biPredSubPuL1;

        Distortion minCost = cDistParam.distFunc( cDistParam );
      
        bool notZeroCost        = true;
        int16_t totalDeltaMV[2] = { 0, 0 };
        int16_t deltaMV[2]      = { 0, 0 };

        minCost >>= 1;
        minCost  -= ( minCost >> 2 );

        if( minCost < ( dx * dy ) )
        {
          notZeroCost = false;
        }
        else
        {
          pSADsArray[0] = minCost;

          xBIPMVRefine( cDistParam, biPredSubPuL0, biPredSubPuL1, minCost, deltaMV, m_SADsArray );

          if( deltaMV[0] != 0 || deltaMV[1] != 0 )
          {
            pSADsArray += deltaMV[1] * ( 2 * DMVR_NUM_ITERATION + 1 ) + deltaMV[0];
          }
        }

        bioAppliedSubblk  = minCost < bioEnabledThres ? false : bioApplied;
        totalDeltaMV[0]   = deltaMV[0] << mvShift;
        totalDeltaMV[1]   = deltaMV[1] << mvShift;
        xDMVRSubPixelErrorSurface( notZeroCost, totalDeltaMV, deltaMV, pSADsArray );

        Mv &curDMv = pu.cs->m_dmvrMvCache[pu.mvdL0SubPuOff + num];
        curDMv = Mv( totalDeltaMV[0], totalDeltaMV[1] );
        
        Mv mv0 = mergeMv[REF_PIC_LIST_0] + curDMv; mv0.clipToStorageBitDepth();
        Mv mv1 = mergeMv[REF_PIC_LIST_1] - curDMv; mv1.clipToStorageBitDepth();

        if( ( mv0.hor >> mvShift ) != ( mergeMv[0].hor >> mvShift ) || ( mv0.ver >> mvShift ) != ( mergeMv[0].ver >> mvShift ) )
        {
          xPrefetchPad( subPu, cYuvRefBuffDMVRL0, REF_PIC_LIST_0, true );
        }

        if( isChromaEnabled( pu.chromaFormat ) && ( ( mv0.hor >> mvShiftX ) != ( mergeMv[0].hor >> mvShiftX ) || ( mv0.ver >> mvShiftY ) != ( mergeMv[0].ver >> mvShiftY ) ) )
        {
          xPrefetchPad( subPu, cYuvRefBuffDMVRL0, REF_PIC_LIST_0, false );
        }

        if( ( mv1.hor >> mvShift ) != ( mergeMv[1].hor >> mvShift ) || ( mv1.ver >> mvShift ) != ( mergeMv[1].ver >> mvShift ) )
        {
          xPrefetchPad( subPu, cYuvRefBuffDMVRL1, REF_PIC_LIST_1, true );
        }
        if( isChromaEnabled( pu.chromaFormat ) && ( ( mv1.hor >> mvShiftX ) != ( mergeMv[1].hor >> mvShiftX ) || ( mv1.ver >> mvShiftY ) != ( mergeMv[1].ver >> mvShiftY ) ) )
        {
          xPrefetchPad( subPu, cYuvRefBuffDMVRL1, REF_PIC_LIST_1, false );
        }

        subPu.mv[0][0] = mv0;
        subPu.mv[1][0] = mv1;

        subPredBuf.bufs[COMPONENT_Y].buf    = pcYuvDst.bufs[COMPONENT_Y].buf  +   xStart +                 yStart             * dstStride[COMPONENT_Y];

        if( isChromaEnabled( pu.chromaFormat ) )
        {
          subPredBuf.bufs[COMPONENT_Cb].buf = pcYuvDst.bufs[COMPONENT_Cb].buf + ( xStart >> scaleX ) + ( ( yStart >> scaleY ) * dstStride[COMPONENT_Cb] );
          subPredBuf.bufs[COMPONENT_Cr].buf = pcYuvDst.bufs[COMPONENT_Cr].buf + ( xStart >> scaleX ) + ( ( yStart >> scaleY ) * dstStride[COMPONENT_Cr] );
        }

        PelUnitBuf& srcPred0 = subPredBuf;

        xFinalPaddedMCForDMVR( subPu, srcPred0, srcPred1, cYuvRefBuffDMVRL0, cYuvRefBuffDMVRL1, bioAppliedSubblk, mergeMv );

        xWeightedAverage( subPu, srcPred0, srcPred1, subPredBuf, bds, clpRngs, bioAppliedSubblk );
        num++;
      }
    }
  }
}

void InterPrediction::xFillIBCBuffer(CodingUnit &cu)
{
  for( TransformUnit &tu : TUTraverser( &cu.firstTU, cu.lastTU->next ) )
  {
    for( const CompArea &area : tu.blocks )
    {
      if (!area.valid())
        continue;

      const unsigned int lcuWidth = cu.sps->getMaxCUWidth();
      const int shiftSample = getComponentScaleX(area.compID(), cu.chromaFormat);
      const int ctuSizeLog2 = getLog2(lcuWidth) - shiftSample;
      const int pux = area.x & ((m_IBCBufferWidth >> shiftSample) - 1);
      const int puy = area.y & (( 1 << ctuSizeLog2 ) - 1);
      const CompArea dstArea = CompArea(area.compID(), Position(pux, puy), Size(area.width, area.height));
      CPelBuf srcBuf = cu.cs->getRecoBuf(area);
      PelBuf dstBuf = m_IBCBuffer.getBuf(dstArea);

      dstBuf.copyFrom(srcBuf);
    }
  }
}

void InterPrediction::xIntraBlockCopy( PredictionUnit &pu, PelUnitBuf &predBuf, const ComponentID compID )
{
  const unsigned int lcuWidth = pu.sps->getMaxCUWidth();
  const int shiftSampleHor = getComponentScaleX( compID, pu.chromaFormat );
  const int shiftSampleVer = getComponentScaleY( compID, pu.chromaFormat );
  const int ctuSizeVerLog2 = getLog2(lcuWidth) - shiftSampleVer;
  Mv bv = pu.mv[REF_PIC_LIST_0][0];
  bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  int refx, refy;
  if (compID == COMPONENT_Y)
  {
    refx = pu.Y().x + bv.hor;
    refy = pu.Y().y + bv.ver;
  }
  else
  {//Cb or Cr
    refx = pu.Cb().x + (bv.hor >> shiftSampleHor);
    refy = pu.Cb().y + (bv.ver >> shiftSampleVer);
  }
  refx &= ((m_IBCBufferWidth >> shiftSampleHor) - 1);
  refy &= ((1 << ctuSizeVerLog2) - 1);

  int lineIdx = pu.lumaPos().y / pu.slice->getSPS()->getMaxCUHeight();
  
  if (refx + predBuf.bufs[compID].width <= (m_IBCBufferWidth >> shiftSampleHor))
  {
    const CompArea srcArea = CompArea(compID, Position(refx, refy), Size(predBuf.bufs[compID].width, predBuf.bufs[compID].height));
    const CPelBuf refBuf = pu.cs->m_virtualIBCbuf[lineIdx].getBuf( srcArea );   //m_IBCBuffer.getBuf(srcArea);
    predBuf.bufs[compID].copyFrom(refBuf);
  }
  else
  {//wrap around
    int width = (m_IBCBufferWidth >> shiftSampleHor) - refx;
    CompArea srcArea = CompArea(compID, Position(refx, refy), Size(width, predBuf.bufs[compID].height));
    CPelBuf srcBuf = pu.cs->m_virtualIBCbuf[lineIdx].getBuf( srcArea );   //m_IBCBuffer.getBuf(srcArea);
    PelBuf dstBuf = PelBuf(predBuf.bufs[compID].bufAt(Position(0, 0)), predBuf.bufs[compID].stride, Size(width, predBuf.bufs[compID].height));
    dstBuf.copyFrom(srcBuf);

    width = refx + predBuf.bufs[compID].width - (m_IBCBufferWidth >> shiftSampleHor);
    srcArea = CompArea(compID, Position(0, refy), Size(width, predBuf.bufs[compID].height));
    srcBuf = pu.cs->m_virtualIBCbuf[lineIdx].getBuf( srcArea );   //m_IBCBuffer.getBuf(srcArea);
    dstBuf = PelBuf(predBuf.bufs[compID].bufAt(Position((m_IBCBufferWidth >> shiftSampleHor) - refx, 0)), predBuf.bufs[compID].stride, Size(width, predBuf.bufs[compID].height));
    dstBuf.copyFrom(srcBuf);
  }
}

#if JVET_O1170_CHECK_BV_AT_DECODER
void InterPrediction::resetIBCBuffer(const ChromaFormat chromaFormatIDC, const int ctuSize)
{
  const UnitArea area = UnitArea(chromaFormatIDC, Area(0, 0, m_IBCBufferWidth, ctuSize));
  m_IBCBuffer.getBuf(area).fill(-1);
}

void InterPrediction::resetVPDUforIBC(const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos)
{
  const UnitArea area = UnitArea(chromaFormatIDC, Area(xPos & (m_IBCBufferWidth - 1), yPos & (ctuSize - 1), vSize, vSize));
  m_IBCBuffer.getBuf(area).fill(-1);
}

bool InterPrediction::isLumaBvValid(const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv)
{
  if(((yCb + yBv) & (ctuSize - 1)) + height > ctuSize)
  {
    return false;
  }
  int refTLx = xCb + xBv;
  int refTLy = (yCb + yBv) & (ctuSize - 1);
  PelBuf buf = m_IBCBuffer.Y();
  for(int x = 0; x < width; x += 4)
  {
    for(int y = 0; y < height; y += 4)
    {
      if(buf.at((x + refTLx) & (m_IBCBufferWidth - 1), y + refTLy) == -1) return false;
      if(buf.at((x + 3 + refTLx) & (m_IBCBufferWidth - 1), y + refTLy) == -1) return false;
      if(buf.at((x + refTLx) & (m_IBCBufferWidth - 1), y + 3 + refTLy) == -1) return false;
      if(buf.at((x + 3 + refTLx) & (m_IBCBufferWidth - 1), y + 3 + refTLy) == -1) return false;
    }
  }
  return true;
}
#endif

void InterPrediction::xPredInterBlkRPR( const std::pair<int, int>& scalingRatio, const PPS& pps, const ComponentID& compID, const ChromaFormat chFmt, const Picture* refPic, const Mv& mv, const Position blkPos, const int dstWidth, const int dstHeight, Pel* dst, const ptrdiff_t dstStride, const bool bi, const bool wrapRef, const ClpRng& clpRng, const int filterIndex, const bool useAltHpelIf )
{ 
  const bool          rndRes = !bi;

  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleX( compID, chFmt );
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleY( compID, chFmt );

  int width  = dstWidth;
  int height = dstHeight;
  CPelBuf refBuf;
  const Pel* refPtr;
  ptrdiff_t  refStride;

  int row, col;

  int refPicWidth = refPic->lwidth();
  int refPicHeight = refPic->lheight();

  int xFilter = filterIndex;
  int yFilter = filterIndex;
  const int rprThreshold1 = ( 1 << SCALE_RATIO_BITS ) * 5 / 4; 
  const int rprThreshold2 = ( 1 << SCALE_RATIO_BITS ) * 7 / 4;
  if( filterIndex == 0 )
  {
    if( scalingRatio.first > rprThreshold2 )
    {
      xFilter = 4;
    }
    else if( scalingRatio.first > rprThreshold1 )
    {
      xFilter = 3;
    }

    if( scalingRatio.second > rprThreshold2 )
    {
      yFilter = 4;
    }
    else if( scalingRatio.second > rprThreshold1 )
    {
      yFilter = 3;
    }
  }
  if (filterIndex == 2)
  {
    if (isLuma(compID))
    {
      if (scalingRatio.first > rprThreshold2)
      {
        xFilter = 6;
      }
      else if (scalingRatio.first > rprThreshold1)
      {
        xFilter = 5;
      }

      if (scalingRatio.second > rprThreshold2)
      {
        yFilter = 6;
      }
      else if (scalingRatio.second > rprThreshold1)
      {
        yFilter = 5;
      }
    }
    else
    {
      if (scalingRatio.first > rprThreshold2)
      {
        xFilter = 4;
      }
      else if (scalingRatio.first > rprThreshold1)
      {
        xFilter = 3;
      }

      if (scalingRatio.second > rprThreshold2)
      {
        yFilter = 4;
      }
      else if (scalingRatio.second > rprThreshold1)
      {
        yFilter = 3;
      }
    }
  }

  const int posShift = SCALE_RATIO_BITS - 4;
  int stepX = ( scalingRatio.first + 8 ) >> 4;
  int stepY = ( scalingRatio.second + 8 ) >> 4;
  int64_t x0Int;
  int64_t y0Int;
  int offX = 1 << ( posShift - shiftHor - 1 );
  int offY = 1 << ( posShift - shiftVer - 1 );

  const int64_t posX = ( ( blkPos.x << getComponentScaleX( compID, chFmt ) ) - ( pps.getScalingWindow().getWindowLeftOffset() * SPS::getWinUnitX( chFmt ) ) ) >> getComponentScaleX( compID, chFmt );
  const int64_t posY = ( ( blkPos.y << getComponentScaleY( compID, chFmt ) ) - ( pps.getScalingWindow().getWindowTopOffset()  * SPS::getWinUnitY( chFmt ) ) ) >> getComponentScaleY( compID, chFmt );

  int addX = isLuma( compID ) ? 0 : int( 1 - refPic->cs->sps->getHorCollocatedChromaFlag() ) * 8 * ( scalingRatio.first - SCALE_1X.first );
  int addY = isLuma( compID ) ? 0 : int( 1 - refPic->cs->sps->getVerCollocatedChromaFlag() ) * 8 * ( scalingRatio.second - SCALE_1X.second );

  x0Int = ( ( posX << ( 4 + getComponentScaleX( compID, chFmt ) ) ) + mv.getHor() ) * (int64_t)scalingRatio.first + addX;
  x0Int = SIGN( x0Int ) * ( ( llabs( x0Int ) + ( (long long)1 << ( 7 + getComponentScaleX( compID, chFmt ) ) ) ) >> ( 8 + getComponentScaleX( compID, chFmt ) ) ) + ( ( refPic->slices[0]->getPPS()->getScalingWindow().getWindowLeftOffset() * SPS::getWinUnitX( chFmt ) ) << ( ( posShift - getComponentScaleX( compID, chFmt ) ) ) );

  y0Int = ( ( posY << ( 4 + getComponentScaleY( compID, chFmt ) ) ) + mv.getVer() ) * (int64_t)scalingRatio.second + addY;
  y0Int = SIGN( y0Int ) * ( ( llabs( y0Int ) + ( (long long)1 << ( 7 + getComponentScaleY( compID, chFmt ) ) ) ) >> ( 8 + getComponentScaleY( compID, chFmt ) ) ) + ( ( refPic->slices[0]->getPPS()->getScalingWindow().getWindowTopOffset() * SPS::getWinUnitY( chFmt ) ) << ( ( posShift - getComponentScaleY( compID, chFmt ) ) ) );
  
  const int extSize = isLuma( compID ) ? 1 : 2;

  int vFilterSize = isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA;

  int yInt0 = ( (int32_t)y0Int + offY ) >> posShift;
  yInt0 = std::min( std::max( -(NTAPS_LUMA / 2), yInt0 ), ( refPicHeight >> getComponentScaleY( compID, chFmt ) ) + (NTAPS_LUMA / 2) );

  int xInt0 = ( (int32_t)x0Int + offX ) >> posShift;
  xInt0 = std::min( std::max( -(NTAPS_LUMA / 2), xInt0 ), ( refPicWidth >> getComponentScaleX( compID, chFmt ) ) + (NTAPS_LUMA / 2) );

  int refHeight = ((((int32_t)y0Int + (height-1) * stepY) + offY ) >> posShift) - ((((int32_t)y0Int + 0 * stepY) + offY ) >> posShift) + 1;
  refHeight = std::max<int>( 1, refHeight );
  
  CHECK( MAX_CU_SIZE * MAX_SCALING_RATIO < refHeight + vFilterSize - 1 + extSize, "Buffer size is not enough, increase MAX_SCALING_RATIO" );

  Pel buffer[( MAX_CU_SIZE + 16 ) * ( MAX_CU_SIZE * MAX_SCALING_RATIO + 16 )];

  int tmpStride = width;
  int xInt = 0, yInt = 0;

  refBuf = refPic->getRecoBuf( compID, wrapRef );

  for( col = 0; col < width; col++ )
  {
    int posX = (int32_t)x0Int + col * stepX;
    xInt = ( posX + offX ) >> posShift;
    xInt = std::min( std::max( -(NTAPS_LUMA / 2), xInt ), ( refPicWidth >> getComponentScaleX( compID, chFmt ) ) + (NTAPS_LUMA / 2) );
    int xFrac = ( ( posX + offX ) >> ( posShift - shiftHor ) ) & ( ( 1 << shiftHor ) - 1 );

    CHECK( xInt0 > xInt, "Wrong horizontal starting point" );

    refPtr    = refBuf.bufAt( xInt, yInt0 );
    refStride = refBuf.stride;

    m_if.filterHor( compID, GET_OFFSETY( refPtr, refStride, -( ( vFilterSize >> 1 ) - 1 ) ), refStride, GET_OFFSETX( buffer, tmpStride, col ), tmpStride, 1, refHeight + vFilterSize - 1 + extSize, xFrac, false, chFmt, clpRng, xFilter, false, useAltHpelIf && scalingRatio.first == 1 << SCALE_RATIO_BITS );
  }

  for( row = 0; row < height; row++ )
  {
    int posY = (int32_t)y0Int + row * stepY;
    yInt = ( posY + offY ) >> posShift;
    yInt = std::min( std::max( -(NTAPS_LUMA / 2), yInt ), ( refPicHeight >> getComponentScaleY( compID, chFmt ) ) + (NTAPS_LUMA / 2) );
    int yFrac = ( ( posY + offY ) >> ( posShift - shiftVer ) ) & ( ( 1 << shiftVer ) - 1 );

    CHECK( yInt0 > yInt, "Wrong vertical starting point" );

    m_if.filterVer( compID, GET_OFFSETY( buffer, tmpStride, ( yInt - yInt0 ) + ( ( vFilterSize >> 1 ) - 1 ) ), tmpStride, dst + row * dstStride, dstStride, width, 1, yFrac, false, rndRes, chFmt, clpRng, yFilter, false, useAltHpelIf && scalingRatio.second == 1 << SCALE_RATIO_BITS );
  }
}

}
