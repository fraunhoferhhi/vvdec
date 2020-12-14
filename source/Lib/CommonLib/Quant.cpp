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

Copyright (c) 2018-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
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

/** \file     Quant.cpp
    \brief    transform and quantization class
*/

#include "Quant.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"

#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>



//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================


// ====================================================================================================================
// QpParam constructor
// ====================================================================================================================

QpParam::QpParam( const TransformUnit& tu, const ComponentID &compIDX, const bool allowACTQpoffset )
{
  const ComponentID compID = MAP_CHROMA( compIDX );
  const ChannelType chType = toChannelType( compID );
  const SPS        &sps    = *tu.cu->cs->sps;
  const int     qpBdOffset = sps.getQpBDOffset( chType );
  const bool useJQP        = isChroma( compID ) && TU::getICTMode( tu, false ) == 2;
  const ComponentID jCbCr  = useJQP ? JOINT_CbCr : compID;
  
  int baseQp;
  int qpy        = tu.cu->qp;
  //bool skip      = tu.mtsIdx[compID] == MTS_SKIP;

  if( isLuma( compID ) )
  {
    baseQp = qpy + qpBdOffset;
  }
  else
  {
    const PPS &pps  = *tu.cu->slice->getPPS();
    int
    chromaQpOffset  = pps.getQpOffset                    ( jCbCr );
    chromaQpOffset += tu.cu->slice->getSliceChromaQpDelta( jCbCr );
    chromaQpOffset += pps.getPpsRangeExtension().getChromaQpOffsetListEntry( tu.cu->chromaQpAdj ).u.offset[int( jCbCr ) - 1];

    int qpi = Clip3( -qpBdOffset, MAX_QP, qpy );
    baseQp  = sps.getMappedChromaQpValue( jCbCr, qpi );
    baseQp  = Clip3( 0, MAX_QP + qpBdOffset, baseQp + chromaQpOffset + qpBdOffset );
  }

  if( allowACTQpoffset && tu.cu->colorTransform() )
  {
    baseQp += DELTA_QP_ACT[jCbCr];
    baseQp  = Clip3( 0, MAX_QP + qpBdOffset, baseQp );
  }

  // TODO: ensure clip not needed for non-ACT

  //if( !skip )
  {
    Qps [0] = baseQp;
    pers[0] = baseQp / 6;
    rems[0] = baseQp - ( pers[0] << 2 ) - ( pers[0] << 1 );
  }
  //else
  {
    int internalMinusInputBitDepth = sps.getInternalMinusInputBitDepth( chType );
    int baseQpTS                   = std::max( baseQp, 4 + 6 * internalMinusInputBitDepth );
    Qps [1] = baseQpTS;
    pers[1] = baseQpTS / 6;
    rems[1] = baseQpTS - ( pers[1] << 2 ) - ( pers[1] << 1 );;
  }
}

// ====================================================================================================================
// Quant class member functions
// ====================================================================================================================

static void DeQuantCore(const int maxX,const int restX,const int maxY,const int scale,const TCoeffSig*const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum)
{


  const int inputMinimum = -(inputMaximum+1);
  const TCoeff transformMinimum = -(transformMaximum);

  if (rightShift>0)
  {
    const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);


    for( int y = 0, n = 0; y <= maxY; y++)
    {
      for( int x = 0; x <= maxX; x++, n++ )
      {
        const TCoeff level = piQCoef[x + y * piQCfStride];

        if( level )
        {
          const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
          Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;
          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
        }
      }
      n += restX;
    }
  }
  else  // rightshift <0
  {
    int leftShift = -rightShift;
    for( int y = 0, n = 0; y <= maxY; y++)
    {
      for( int x = 0; x <= maxX; x++, n++ )
      {
        const TCoeff level = piQCoef[x + y * piQCfStride];

        if( level )
        {
          const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
          const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
        }
      }
      n += restX;
    }

  }

}

static void DeQuantPCMCore(const int maxX,const int restX,const int maxY,const int scale,TCoeff   *const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum)
{
  const int inputMinimum = -(inputMaximum+1);
  const TCoeff transformMinimum = -(transformMaximum);


  if (rightShift > 0)
  {
    const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);
    for( int y = 0, n = 0; y <= maxY; y++)
    {
      for( int x = 0; x <= maxX; x++, n++ )
      {
        const TCoeff level = piQCoef[x + y * piQCfStride];
        if( level )
        {
          const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
          const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
        }
      }
      n += restX;
    }
  }
  else
  {
    int leftShift = -rightShift;
    for( int y = 0, n = 0; y <= maxY; y++)
    {
      for( int x = 0; x <= maxX; x++, n++ )
      {
        const TCoeff level = piQCoef[x + y * piQCfStride];
        if( level )
        {
          const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
          const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
        }
      }
      n += restX;
    }
  }
}



Quant::Quant()
{
  xInitScalingList( nullptr );
  DeQuant= DeQuantCore;
  DeQuantPCM = DeQuantPCMCore;
#if   ENABLE_SIMD_OPT_QUANT
  initQuantX86();
#endif

}

Quant::~Quant()
{
  xDestroyScalingList();
}

void invResDPCM( const TransformUnit &tu, const ComponentID &compID, CoeffBuf &dstBuf )
{
  const CompArea &rect = tu.blocks[compID];
  const int      wdt = rect.width;
  const int      hgt = rect.height;
  const CCoeffSigBuf coeffs = tu.cu->cs->getRecoBuf( tu.block( compID ) );

  const int      maxLog2TrDynamicRange = tu.cu->cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff   inputMinimum   = -(1 << maxLog2TrDynamicRange);
  const TCoeff   inputMaximum   =  (1 << maxLog2TrDynamicRange) - 1;

  const TCoeffSig* coef = &coeffs.buf[0];
  TCoeff* dst = &dstBuf.buf[0];

  if( isLuma( compID ) ? tu.cu->bdpcmMode() == 1 : tu.cu->bdpcmModeChroma() == 1 )
  {
    for( int y = 0; y < hgt; y++ )
    {
      dst[0] = coef[0];
      for( int x = 1; x < wdt; x++ )
      {
        dst[x] = Clip3(inputMinimum, inputMaximum, dst[x - 1] + coef[x]);
      }
      coef += coeffs.stride;
      dst += dstBuf.stride;
    }
  }
  else
  {
    for( int x = 0; x < wdt; x++ )
    {
      dst[x] = coef[x];
    }
    for( int y = 0; y < hgt - 1; y++ )
    {
      for( int x = 0; x < wdt; x++ )
      {
        dst[dstBuf.stride + x] = Clip3(inputMinimum, inputMaximum, dst[x] + coef[coeffs.stride + x]);
      }
      coef += coeffs.stride;
      dst += dstBuf.stride;
    }
  }
}

void Quant::dequant(   const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  const SPS            *sps                = tu.cu->cs->sps.get();
  const CompArea       &area               = tu.blocks[compID];
  const CCoeffSigBuf    coeffBuf           = tu.cu->cs->getRecoBuf( tu.block( compID ) );
  const TCoeffSig*const piQCoef            = coeffBuf.buf;
  const size_t          piQCfStride        = coeffBuf.stride;
        TCoeff   *const piCoef             = dstCoeff.buf;
  const int             maxLog2TrDynamicRange = sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff          transformMinimum   = -(1 << maxLog2TrDynamicRange);
  const TCoeff          transformMaximum   =  (1 << maxLog2TrDynamicRange) - 1;
  const bool            isTransformSkip    = ( tu.mtsIdx[compID] == MTS_SKIP );
  setUseScalingList( tu.cu->slice->getExplicitScalingListUsed() );
#if JVET_P0365_SCALING_MATRIX_LFNST
  const bool            disableSMForLFNST  = tu.cu->slice->getExplicitScalingListUsed() ? sps->getDisableScalingMatrixForLfnstBlks() : false;
  const bool            isLfnstApplied     = tu.cu->lfnstIdx() > 0 && ( CU::isSepTree( *tu.cu ) ? true : isLuma( compID ) );
#if JVET_R0380_SCALING_MATRIX_DISABLE_YCC_OR_RGB
  const bool            disableSMForACT    = tu.cu->cs->sps->getScalingMatrixForAlternativeColourSpaceDisabledFlag() && tu.cu->cs->sps->getScalingMatrixDesignatedColourSpaceFlag() == tu.cu->colorTransform();
  const bool            enableScalingLists = getUseScalingList(isTransformSkip, isLfnstApplied, disableSMForLFNST, disableSMForACT);
#else
  const bool            enableScalingLists = getUseScalingList(isTransformSkip, isLfnstApplied, disableSMForLFNST);
#endif
#else
  const bool            enableScalingLists = getUseScalingList(isTransformSkip);
#endif
  const int             scalingListType    = getScalingListType(tu.cu->predMode(), compID);
  const int             channelBitDepth    = sps->getBitDepth(toChannelType(compID));

  int maxX, maxY;


  if( ( tu.cu->bdpcmMode() && isLuma(compID) ) || ( tu.cu->bdpcmModeChroma() && isChroma(compID) ) )
  {
    invResDPCM( tu, compID, dstCoeff );
    maxX = area.width - 1;
    maxY = area.height - 1;
  }
  else
  {
    maxX = tu.maxScanPosX[compID];
    maxY = tu.maxScanPosY[compID];
  }
  
  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  // Represents scaling through forward transform
  const bool bClipTransformShiftTo0 = false;// tu.mtsIdx[compID] != 1 && sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag();
  const int  originalTransformShift = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);
  const bool needSqrtAdjustment     = TU::needsBlockSizeTrafoScale( tu, compID );
  const int  iTransformShift        = (bClipTransformShiftTo0 ? std::max<int>(0, originalTransformShift) : originalTransformShift) + (needSqrtAdjustment?-1:0);
  const bool depQuant = tu.cu->slice->getDepQuantEnabledFlag() && ( tu.mtsIdx[compID] != MTS_SKIP );
  const int  QP_per   = depQuant ? ( ( cQP.Qp( isTransformSkip ) + 1 ) / 6 )        : cQP.per( isTransformSkip );
  const int  QP_rem   = depQuant ? (   cQP.Qp( isTransformSkip ) + 1 - 6 * QP_per ) : cQP.rem( isTransformSkip );
  const int  rightShift = (IQUANT_SHIFT + ( depQuant ? 1 : 0 ) - ((isTransformSkip ? 0 : iTransformShift) + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  {
    const uint32_t uiLog2TrWidth  = getLog2(area.width);
    const uint32_t uiLog2TrHeight = getLog2(area.height);

    int *piDequantCoef        = getDequantCoeff(scalingListType, QP_rem, uiLog2TrWidth, uiLog2TrHeight);
    int scale     = g_InvQuantScales[needSqrtAdjustment?1:0][QP_rem];

    const int scaleBits = ( IQUANT_SHIFT + 1 );

    //from the dequantisation equation:
    //iCoeffQ                         = Intermediate_Int((int64_t(clipQCoef) * scale + iAdd) >> rightShift);
    //(sizeof(Intermediate_Int) * 8)  =                    inputBitDepth   + scaleBits      - rightShift
    const uint32_t         targetInputBitDepth = std::min<uint32_t>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    const int restX = area.width - maxX - 1;

    if (!enableScalingLists)
    {
      if( ( tu.cu->bdpcmMode() && isLuma( compID ) ) || ( tu.cu->bdpcmModeChroma() && isChroma( compID ) ) )
      {
        TCoeff* dst = &dstCoeff.buf[0];
        DeQuantPCM(maxX,restX,maxY,scale,dst,dstCoeff.stride,piCoef,rightShift,inputMaximum,transformMaximum);

      }
      else
      {
        DeQuant(maxX,restX,maxY,scale,piQCoef,piQCfStride,piCoef,rightShift,inputMaximum,transformMaximum);
      }
    }
    else
    {
      if( ( tu.cu->bdpcmMode() && isLuma(compID) ) || ( tu.cu->bdpcmModeChroma() && isChroma(compID) ) )
      {
        TCoeff* dst = &dstCoeff.buf[0];

        if (rightShift > 0)
        {
          const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);
          for( int y = 0, n = 0; y <= maxY; y++)
          {
            for( int x = 0; x <= maxX; x++, n++ )
            {
              const TCoeff level = dst[x + y * dstCoeff.stride];

              if( level )
              {
                scale = piDequantCoef[n];

                const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
                const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

                piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
              }
            }
            n += restX;
          }
        }
        else
        {
          int leftShift = -rightShift;
          for( int y = 0, n = 0; y <= maxY; y++)
          {
            for( int x = 0; x <= maxX; x++, n++ )
            {
              const TCoeff level = dst[x + y * dstCoeff.stride];

              if( level )
              {
                scale = piDequantCoef[n];

                const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
                const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

                piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
              }
            }
            n += restX;
          }
        }
      }
      else
      {
        if (rightShift > 0)
        {
          const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);

          for( int y = 0, n = 0; y <= maxY; y++)
          {
            for( int x = 0; x <= maxX; x++, n++ )
            {
              const TCoeff level = piQCoef[x + y * piQCfStride];

              if( level )
              {
                scale = piDequantCoef[n];

                const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
                Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;

                piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
              }
            }
            n += restX;
          }
        }
        else
        {
          int leftShift = -rightShift;
          for( int y = 0, n = 0; y <= maxY; y++)
          {
            for( int x = 0; x <= maxX; x++, n++ )
            {
              const TCoeff level = piQCoef[x + y * piQCfStride];

              if( level )
              {
                scale = piDequantCoef[n];

                const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
                const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;

                piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
              }
            }
            n += restX;
          }
        }
      }
    }
  }
}

/** set quantized matrix coefficient for decode
 * \param scalingList quantized matrix address
 * \param format      chroma format
 */
void Quant::setScalingListDec( ScalingList &scalingList )
{
  const int minimumQp = 0;
  const int maximumQp = SCALING_LIST_REM_NUM;

  int scalingListId = 0;
  int recScalingListId = 0;
  for (uint32_t size = SCALING_LIST_FIRST_CODED; size <= SCALING_LIST_LAST_CODED; size++)
  {
    for(uint32_t list = 0; list < SCALING_LIST_NUM; list++)
    {
#if JVET_R0166_SCALING_LISTS_CHROMA_444
      if( size == SCALING_LIST_2x2 && list < 4 )   // skip 2x2 luma
#else
      if( (size == SCALING_LIST_2x2 && list < 4) || (size == SCALING_LIST_64x64 && list % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES)) != 0) )   // skip 2x2 luma
#endif
        continue;
#if JVET_R0166_SCALING_LISTS_CHROMA_444
      scalingListId = g_scalingListId[size][list];
#endif
        for( int qp = minimumQp; qp < maximumQp; qp++ )
        {
          xSetScalingListDec(scalingList, list, size, qp, scalingListId);
        }
#if !JVET_R0166_SCALING_LISTS_CHROMA_444
      scalingListId++;
#endif
    }
  }
  //based on square result and apply downsample technology
  for (uint32_t sizew = 0; sizew <= SCALING_LIST_LAST_CODED; sizew++) //7
  {
    for (uint32_t sizeh = 0; sizeh <= SCALING_LIST_LAST_CODED; sizeh++) //7
    {
      if (sizew == sizeh || (sizew == SCALING_LIST_1x1 && sizeh<SCALING_LIST_4x4) || (sizeh == SCALING_LIST_1x1 && sizew<SCALING_LIST_4x4)) continue;
      for (uint32_t list = 0; list < SCALING_LIST_NUM; list++) //9
      {
        int largerSide = (sizew > sizeh) ? sizew : sizeh;
#if !JVET_R0166_SCALING_LISTS_CHROMA_444
        if (largerSide == SCALING_LIST_64x64 && list % (SCALING_LIST_NUM / (NUMBER_OF_PREDICTION_MODES)) != 0) continue;
#endif
        if (largerSide < SCALING_LIST_4x4) printf("Rectangle Error !\n");
#if JVET_R0166_SCALING_LISTS_CHROMA_444
        recScalingListId = g_scalingListId[largerSide][list];
#else
        recScalingListId = SCALING_LIST_NUM * (largerSide - 2) + 2 + (list / ((largerSide == SCALING_LIST_64x64) ? 3 : 1));
#endif
        for (int qp = minimumQp; qp < maximumQp; qp++)
        {
          xSetRecScalingListDec(scalingList, list, sizew, sizeh, qp, recScalingListId);
        }
      }
    }
  }
}

/** set quantized matrix coefficient for decode
 * \param scalingList quantaized matrix address
 * \param listId List index
 * \param sizeId size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
void Quant::xSetScalingListDec(const ScalingList &scalingList, uint32_t listId, uint32_t sizeId, int qp, uint32_t scalingListId)
{
  uint32_t width  = g_vvcScalingListSizeX[sizeId];
  uint32_t height = g_vvcScalingListSizeX[sizeId];
  uint32_t ratio  = g_vvcScalingListSizeX[sizeId]/std::min(MAX_MATRIX_SIZE_NUM,(int)g_vvcScalingListSizeX[sizeId]);
  int *dequantcoeff;
  const int *coeff = scalingList.getScalingListAddress(scalingListId);

  dequantcoeff = getDequantCoeff(listId, qp, sizeId, sizeId);

  const int blockIsNotPowerOf4 = ( getLog2( width ) + getLog2( height ) ) & 1;
  int invQuantScale = g_InvQuantScales[blockIsNotPowerOf4][qp];

  processScalingListDec(coeff,
                        dequantcoeff,
                        invQuantScale,
                        height, width, ratio,
                        std::min(MAX_MATRIX_SIZE_NUM, (int)g_vvcScalingListSizeX[sizeId]),
                        scalingList.getScalingListDC(scalingListId));
}

/** set quantized matrix coefficient for decode
* \param scalingList quantaized matrix address
* \param listId List index
* \param sizeId size index
* \param qp Quantization parameter
* \param format chroma format
*/
void Quant::xSetRecScalingListDec(const ScalingList &scalingList, uint32_t listId, uint32_t sizeIdw, uint32_t sizeIdh, int qp, uint32_t scalingListId)
{
  if (sizeIdw == sizeIdh) return;
  uint32_t width = g_vvcScalingListSizeX[sizeIdw];
  uint32_t height = g_vvcScalingListSizeX[sizeIdh];
  uint32_t largeSideId = (sizeIdw > sizeIdh) ? sizeIdw : sizeIdh;  //16

  const int *coeff = scalingList.getScalingListAddress(scalingListId);
  int *dequantcoeff;
  dequantcoeff = getDequantCoeff(listId, qp, sizeIdw, sizeIdh);
  const int blockIsNotPowerOf4 = ( getLog2( width ) + getLog2( height ) ) & 1;
  int invQuantScale = g_InvQuantScales[blockIsNotPowerOf4][qp];
  processScalingListDec(coeff,
                        dequantcoeff,
                        invQuantScale,
                        height, width, (largeSideId>3) ? 2 : 1,
                        (largeSideId >= 3 ? 8 : 4),
                        scalingList.getScalingListDC(scalingListId));
}

/** set quantized matrix coefficient for decode
 * \param coeff quantaized matrix address
 * \param dequantcoeff quantaized matrix address
 * \param invQuantScales IQ(QP%6))
 * \param height height
 * \param width width
 * \param ratio ratio for upscale
 * \param sizuNum matrix size
 * \param dc dc parameter
 */
void Quant::processScalingListDec( const int *coeff, int *dequantcoeff, int invQuantScales, uint32_t height, uint32_t width, uint32_t ratio, int sizuNum, uint32_t dc)
{
  if (height != width)
  {
    for (uint32_t j = 0; j<height; j++)
    {
      for (uint32_t i = 0; i<width; i++)
      {
        if (i >= JVET_C0024_ZERO_OUT_TH || j >= JVET_C0024_ZERO_OUT_TH)
        {
          dequantcoeff[j*width + i] = 0;
          continue;
        }
        int ratioWH = height > width   ? height / width   : width   / height;
        int ratioH  = height / sizuNum ? height / sizuNum : sizuNum / height;
        int ratioW  = width / sizuNum  ? width  / sizuNum : sizuNum / width;
        //sizeNum = 8/4
        if (height > width)
        {
          dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j / ratioH) + ((i * ratioWH) / ratioH)];
        }
        else //ratioH < ratioW
        {
          dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * ((j * ratioWH) / ratioW) + (i / ratioW)];
        }
        int largeOne = std::max( width, height );
        if( largeOne > 8 )
        {
          dequantcoeff[0] = invQuantScales * dc;
        }
      }
    }
    return;
  }
  for(uint32_t j=0;j<height;j++)
  {
    for(uint32_t i=0;i<width;i++)
    {
      dequantcoeff[j*width + i] = invQuantScales * coeff[sizuNum * (j / ratio) + i / ratio];
    }
  }

  if(ratio > 1)
  {
    dequantcoeff[0] = invQuantScales * dc;
  }
}

/** initialization process of scaling list array
 */
void Quant::xInitScalingList( const Quant* other )
{
  size_t numQuants = 0;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
        {
          m_dequantCoef [sizeIdX][sizeIdY][listId][qp] = &m_dequantCoefBuf[numQuants];
          numQuants += g_vvcScalingListSizeX[sizeIdX] * g_vvcScalingListSizeX[sizeIdY];
        } // listID loop
      }
    }
  }
}

/** destroy quantization matrix array
 */
void Quant::xDestroyScalingList()
{
}

void Quant::init( Slice *slice )
{
  if( slice->getExplicitScalingListUsed() )
  {
    std::shared_ptr<APS> scalingListAPS = slice->getPicHeader()->getScalingListAPS();
    if( slice->getNalUnitLayerId() != scalingListAPS->getLayerId() )
    {
      CHECK( scalingListAPS->getLayerId() > slice->getNalUnitLayerId(), "Layer Id of APS cannot be greater than layer Id of VCL NAL unit the refer to it" );
      CHECK( slice->getSPS()->getVPSId() == 0, "VPSId of the referred SPS cannot be 0 when layer Id of APS and layer Id of current slice are different" );
      for( int i = 0; i < slice->getVPS()->getNumOutputLayerSets(); i++ )
      {
        bool isCurrLayerInOls = false;
        bool isRefLayerInOls = false;
        for( int j = slice->getVPS()->getNumLayersInOls(i) - 1; j >= 0; j-- )
        {
          if( slice->getVPS()->getLayerIdInOls(i, j) == slice->getNalUnitLayerId() )
          {
            isCurrLayerInOls = true;
          }
          if( slice->getVPS()->getLayerIdInOls(i, j) == scalingListAPS->getLayerId() )
          {
            isRefLayerInOls = true;
          }
        }
        CHECK( isCurrLayerInOls && !isRefLayerInOls, "When VCL NAl unit in layer A refers to APS in layer B, all OLS that contains layer A shall also contains layer B" );
      }
    }
    ScalingList scalingList = scalingListAPS->getScalingList();
    setScalingListDec(scalingList);
    setUseScalingList(true);
  }
  else
  {
    setUseScalingList( false );
  }
}

//! \}

