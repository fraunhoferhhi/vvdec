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

namespace vvdec
{

// ====================================================================================================================
// QpParam constructor
// ====================================================================================================================

QpParam::QpParam( const TransformUnit& tu, const ComponentID& compID, const bool allowACTQpoffset )
{
  const SPS&        sps        = *tu.cu->sps;
  const int         qpBdOffset = sps.getQpBDOffset();
  const bool        useJQP     = isChroma( compID ) && TU::getICTMode( tu, false ) == 2;
  const ComponentID jCbCr      = useJQP ? JOINT_CbCr : compID;

  int baseQp;
  int qpy        = tu.cu->qp;
  //bool skip      = tu.mtsIdx[compID] == MTS_SKIP;

  if( isLuma( compID ) )
  {
    baseQp = qpy + qpBdOffset;
  }
  else
  {
    const PPS &pps  = *tu.cu->pps;
    int
    chromaQpOffset  = pps.getQpOffset                    ( jCbCr );
    chromaQpOffset += tu.cu->slice->getSliceChromaQpDelta( jCbCr );
    chromaQpOffset += pps.getChromaQpOffsetListEntry( tu.cu->chromaQpAdj ).get( jCbCr );

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
    int internalMinusInputBitDepth = sps.getInternalMinusInputBitDepth();
    int baseQpTS                   = std::max( baseQp, 4 + 6 * internalMinusInputBitDepth );
    Qps [1] = baseQpTS;
    pers[1] = baseQpTS / 6;
    rems[1] = baseQpTS - ( pers[1] << 2 ) - ( pers[1] << 1 );;
  }
}

// ====================================================================================================================
// Quant class member functions
// ====================================================================================================================

static void DeQuantCore( const int              maxX,
                         const int              restX,
                         const int              maxY,
                         const int              scale,
                         const TCoeffSig* const piQCoef,
                         const size_t           piQCfStride,
                               TCoeff* const    piCoef,
                         const int              rightShift,
                         const int              inputMaximum,
                         const TCoeff           transformMaximum )
{
  const int    inputMinimum     = -( inputMaximum     + 1 );
  const TCoeff transformMinimum = -( transformMaximum + 1 );

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
          const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) *(1<<leftShift);

          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
        }
      }
      n += restX;
    }

  }
}

static void DeQuantPCMCore( const int     maxX,
                            const int     restX,
                            const int     maxY,
                            const int     scale,
                            TCoeff* const piQCoef,
                            const size_t  piQCfStride,
                            TCoeff* const piCoef,
                            const int     rightShift,
                            const int     inputMaximum,
                            const TCoeff  transformMaximum )
{
  const int    inputMinimum     = -( inputMaximum + 1 );
  const TCoeff transformMinimum = -( transformMaximum );

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
          const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) *(1<< leftShift);

          piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
        }
      }
      n += restX;
    }
  }
}

Quant::Quant( const Quant* other ) : m_dequantCoefBuf( nullptr ), m_ownDequantCoeff( false )
{
  xInitScalingList( other );

  DeQuant    = DeQuantCore;
  DeQuantPCM = DeQuantPCMCore;
#if ENABLE_SIMD_OPT_QUANT && defined( TARGET_SIMD_X86 )

  initQuantX86();
#endif
}

Quant::~Quant()
{
  xDestroyScalingList();
}

void invResDPCM( const TransformUnit &tu, const ComponentID &compID, CoeffBuf &dstBuf )
{
  const CompArea&    rect   = tu.blocks[compID];
  const int          wdt    = rect.width;
  const int          hgt    = rect.height;
  const CCoeffSigBuf coeffs = tu.cu->cs->getRecoBuf( tu.block( compID ) );

  const int    maxLog2TrDynamicRange = tu.cu->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff inputMinimum          = -(1 << maxLog2TrDynamicRange);
  const TCoeff inputMaximum          =  (1 << maxLog2TrDynamicRange) - 1;

  const TCoeffSig* coef = &coeffs.buf[0];
  TCoeff*          dst  = &dstBuf.buf[0];

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

static inline int getTransformShift( const int channelBitDepth, const Size size, const int maxLog2TrDynamicRange )
{
  return maxLog2TrDynamicRange - channelBitDepth - ( ( getLog2( size.width ) + getLog2( size.height ) ) >> 1 );
}

static inline int getScalingListType( const PredMode predMode, const ComponentID compID )
{
  return ( predMode == MODE_INTRA ? 0 : MAX_NUM_COMPONENT ) + compID;
}

void Quant::dequant( const TransformUnit& tu, CoeffBuf& dstCoeff, const ComponentID& compID, const QpParam& cQP )
{
  const SPS*             sps                   = tu.cu->sps;
  const CompArea&        area                  = tu.blocks[compID];
  const CCoeffSigBuf     coeffBuf              = tu.cu->cs->getRecoBuf( tu.block( compID ) );
  const TCoeffSig* const piQCoef               = coeffBuf.buf;
  const size_t           piQCfStride           = coeffBuf.stride;
        TCoeff* const    piCoef                = dstCoeff.buf;
  const int              maxLog2TrDynamicRange = sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const TCoeff           transformMinimum      = -( 1 << maxLog2TrDynamicRange );
  const TCoeff           transformMaximum      =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const bool             isTransformSkip       = ( tu.mtsIdx( compID ) == MTS_SKIP );
  setUseScalingList( tu.cu->slice->getExplicitScalingListUsed() );
  const bool             disableSMForLFNST     = tu.cu->slice->getExplicitScalingListUsed() ? sps->getDisableScalingMatrixForLfnstBlks() : false;
  const bool             isLfnstApplied        = tu.cu->lfnstIdx() > 0 && ( CU::isSepTree( *tu.cu ) ? true : isLuma( compID ) );
  const bool             disableSMForACT       = tu.cu->sps->getScalingMatrixForAlternativeColourSpaceDisabledFlag() && tu.cu->sps->getScalingMatrixDesignatedColourSpaceFlag() == tu.cu->colorTransform();
  const bool             enableScalingLists    = getUseScalingList( isTransformSkip, isLfnstApplied, disableSMForLFNST, disableSMForACT );
  const int              scalingListType       = getScalingListType( tu.cu->predMode(), compID );
  const int              channelBitDepth       = sps->getBitDepth();

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
  const bool depQuant = tu.cu->slice->getDepQuantEnabledFlag() && ( tu.mtsIdx( compID ) != MTS_SKIP );
  const int  QP_per   = depQuant ? ( ( cQP.Qp( isTransformSkip ) + 1 ) / 6 )        : cQP.per( isTransformSkip );
  const int  QP_rem   = depQuant ? (   cQP.Qp( isTransformSkip ) + 1 - 6 * QP_per ) : cQP.rem( isTransformSkip );
  const int  rightShift = (IQUANT_SHIFT + ( depQuant ? 1 : 0 ) - ((isTransformSkip ? 0 : iTransformShift) + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  {
    const uint32_t uiLog2TrWidth  = getLog2(area.width);
    const uint32_t uiLog2TrHeight = getLog2(area.height);

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
      int* piDequantCoef = getDequantCoeff( scalingListType, uiLog2TrWidth, uiLog2TrHeight );
      int  scaleQP       = g_InvQuantScales[needSqrtAdjustment ? 1 : 0][QP_rem];

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
                scale = piDequantCoef[n] * scaleQP;

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
                scale = piDequantCoef[n] * scaleQP;

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
                scale = piDequantCoef[n] * scaleQP;

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
                scale = piDequantCoef[n] * scaleQP;

                const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, level));
                const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) *(1<< leftShift);

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
void Quant::setScalingListDec( const ScalingList& scalingList )
{
  int scalingListId    = 0;
  int recScalingListId = 0;
  bool anyChange = false;
  for( uint32_t size = SCALING_LIST_FIRST_CODED; size <= SCALING_LIST_LAST_CODED; size++ )
  {
    for( uint32_t list = 0; list < SCALING_LIST_NUM; list++ )
    {
      if( size == SCALING_LIST_2x2 && list < 4 )   // skip 2x2 luma
      {
        continue;
      }
      scalingListId = g_scalingListId[size][list];
      anyChange |= xSetScalingListDec( scalingList, list, size, scalingListId );
    }
  }
  if( !anyChange ) return;
  // based on square result and apply downsample technology
  for( uint32_t sizew = 0; sizew <= SCALING_LIST_LAST_CODED; sizew++ )   // 7
  {
    for( uint32_t sizeh = 0; sizeh <= SCALING_LIST_LAST_CODED; sizeh++ )   // 7
    {
      if( sizew == sizeh || ( sizew == SCALING_LIST_1x1 && sizeh < SCALING_LIST_4x4 ) || ( sizeh == SCALING_LIST_1x1 && sizew < SCALING_LIST_4x4 ) )
      {
        continue;
      }
      for( uint32_t list = 0; list < SCALING_LIST_NUM; list++ )   // 9
      {
        int largerSide = ( sizew > sizeh ) ? sizew : sizeh;
        CHECK( largerSide < SCALING_LIST_4x4, "Rectangle Error!" );
        recScalingListId = g_scalingListId[largerSide][list];
        xSetRecScalingListDec( scalingList, list, sizew, sizeh, recScalingListId );
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
bool Quant::xSetScalingListDec(const ScalingList &scalingList, uint32_t listId, uint32_t sizeId, uint32_t scalingListId)
{
  const uint32_t width  = g_vvcScalingListSizeX[sizeId];
  const uint32_t height = g_vvcScalingListSizeX[sizeId];
#if defined( __SANITIZE_ADDRESS__ )   // work around a bug in GCC address-sanitizer, when building with -fsanitize=address, but without -fsanitize=undefined
  volatile
#endif
  const uint32_t ratio  = g_vvcScalingListSizeX[sizeId]/std::min(MAX_MATRIX_SIZE_NUM,(int)g_vvcScalingListSizeX[sizeId]);

  const int *coeff = scalingList.getScalingListAddress(scalingListId);
  int *dequantcoeff = getDequantCoeff(listId, sizeId, sizeId);

  return
  processScalingListDec(coeff,
                        dequantcoeff,
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
void Quant::xSetRecScalingListDec(const ScalingList &scalingList, uint32_t listId, uint32_t sizeIdw, uint32_t sizeIdh, uint32_t scalingListId)
{
  if (sizeIdw == sizeIdh) return;

  const uint32_t width  = g_vvcScalingListSizeX[sizeIdw];
  const uint32_t height = g_vvcScalingListSizeX[sizeIdh];
  const uint32_t largeSideId = (sizeIdw > sizeIdh) ? sizeIdw : sizeIdh;  //16

  const int *coeff = scalingList.getScalingListAddress(scalingListId);
  int *dequantcoeff = getDequantCoeff(listId, sizeIdw, sizeIdh);

  processScalingListDec(coeff,
                        dequantcoeff,
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
bool Quant::processScalingListDec( const int *coeff, int *dequantcoeff, uint32_t height, uint32_t width, uint32_t ratio, int sizuNum, uint32_t dc)
{
  if (height != width)
  {
    const int hl2 = getLog2( height );
    const int wl2 = getLog2( width );
    const int sl2 = getLog2( sizuNum );

    const int loopH = std::min<int>( height, JVET_C0024_ZERO_OUT_TH );
    const int loopW = std::min<int>( width,  JVET_C0024_ZERO_OUT_TH );
    
    const int ratioWH = height > width   ? hl2 - wl2 : wl2 - hl2;
    const int ratioH  = height / sizuNum ? hl2 - sl2 : sl2 - hl2;
    const int ratioW  = width / sizuNum  ? wl2 - sl2 : sl2 - wl2;

    if( height > width )
    {
      for( uint32_t j = 0; j < loopH; j += ( 1 << ratioH ) )
      {
        for( uint32_t i = 0; i < loopW; i++ )
        {
          dequantcoeff[j * width + i] = coeff[sizuNum * ( j >> ratioH ) + ( ( i << ratioWH ) >> ratioH )];
        }

        const int* src = &dequantcoeff[j * width];
        for( int jj = 1; jj < ( 1 << ratioH ); jj++ )
        {
          memcpy( &dequantcoeff[( j + jj ) * width], src, loopW * sizeof( int ) );
        }
      }
    }
    else
    {
      for( uint32_t j = 0; j < loopH; j++ )
      {
        for( uint32_t i = 0; i < loopW; i += ( 1 << ratioW ) )
        {
          const int coeffi = coeff[sizuNum * ( ( j << ratioWH ) >> ratioW ) + ( i >> ratioW )];
          for( uint32_t ii = 0; ii < ( 1 << ratioW ); ii++ )
          {
            dequantcoeff[j * width + i + ii] = coeffi;
          }
        }
      }
    }

    const int largeOne = std::max( width, height );
    if( largeOne > 8 )
    {
      dequantcoeff[0] = dc;
    }
    return true;
  }

  bool anyChange = false;

  const int rl2   = getLog2( ratio );
  const int loopH = std::min<int>( height, JVET_C0024_ZERO_OUT_TH );
  const int loopW = std::min<int>( width, JVET_C0024_ZERO_OUT_TH );

  for( uint32_t j = 0; j < loopH; j += ( 1 << rl2 ) )
  {
    for( uint32_t i = 0; i < loopW; i += ( 1 << rl2 ) )
    {
      const int coeffi = coeff[sizuNum * ( j >> rl2 ) + ( i >> rl2 )];
      anyChange |= coeffi != dequantcoeff[j * width + i];
      for( uint32_t ii = 0; anyChange && ii < ( 1 << rl2 ); ii++ )
      {
        dequantcoeff[j * width + i + ii] = coeffi;
      }
    }

    const int* src = &dequantcoeff[j * width];
    for( int jj = 1; jj < ( 1 << rl2 ); jj++ )
    {
      memcpy( &dequantcoeff[( j + jj ) * width], src, loopW * sizeof( int ) );
    }
  }

  if( ratio > 1 )
  {
    anyChange |= dequantcoeff[0] != dc;
    dequantcoeff[0] = dc;
  }

  return anyChange;
}

static constexpr int g_numScalingListCoeffs = 96774;

/** initialization process of scaling list array
 */
void Quant::xInitScalingList( const Quant* other )
{
  if( other )
  {
    m_dequantCoefBuf  = other->m_dequantCoefBuf;
    m_ownDequantCoeff = false;
  }
  else
  {
    m_dequantCoefBuf  = new int[g_numScalingListCoeffs];
    m_ownDequantCoeff = true;
  }

  size_t numQuants = 0;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        m_dequantCoef [sizeIdX][sizeIdY][listId] = &m_dequantCoefBuf[numQuants];
        numQuants += g_vvcScalingListSizeX[sizeIdX] * g_vvcScalingListSizeX[sizeIdY];
      } // listID loop
    }
  }

  CHECK( numQuants != g_numScalingListCoeffs, "Incorrect size of scaling list entries number!" );
}

/** destroy quantization matrix array
 */
void Quant::xDestroyScalingList()
{
  if( m_ownDequantCoeff )
  {
    delete[] m_dequantCoefBuf;
  }

  m_ownDequantCoeff = false;
  m_dequantCoefBuf  = nullptr;
}

void Quant::init( const Picture *pic )
{
  const Slice* scalingListSlice = nullptr;

  for( const Slice* slice : pic->slices )
  {
    if( slice->getExplicitScalingListUsed() )
    {
      scalingListSlice = slice;
      break;
    }
  }

  const Slice* slice = scalingListSlice;

  if( slice && slice->getExplicitScalingListUsed() )
  {
    const std::shared_ptr<const APS> scalingListAPS = slice->getPicHeader()->getScalingListAPS();
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
    const ScalingList& scalingList = scalingListAPS->getScalingList();
    if( m_ownDequantCoeff )
    {
      memset( m_dequantCoefBuf, 0, sizeof( int ) * g_numScalingListCoeffs );
      setScalingListDec( scalingList );
    }
    setUseScalingList(true);
  }
  else
  {
    setUseScalingList( false );
  }
}

}
