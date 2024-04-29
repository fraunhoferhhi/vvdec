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

/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "TimeProfiler.h"
#include "Quant.h"
#include "InterPrediction.h"


#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>

#include "Quant.h"

namespace vvdec
{

InvTrans *fastInvTrans[NUM_TRANS_TYPE][g_numTransformMatrixSizes] =
{
  { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64 },
  { nullptr,            fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, nullptr },
  { nullptr,            fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, nullptr },
};

//! \ingroup CommonLib
//! \{

static void invLfnstNxNCore( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  int             maxLog2TrDynamicRange =  15;
  const TCoeff    outputMinimum         = -( 1 << maxLog2TrDynamicRange );
  const TCoeff    outputMaximum         =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const int8_t*   trMat                 =  ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ];
  const int       trSize                =  ( size > 4 ) ? 48 : 16;
  int             resi;
  int*            out                   =  dst;

  CHECK( index > 2, "wrong" );

  for( int j = 0; j < trSize; j++, trMat += 16 )
  {
    resi = 0;
    const int8_t* trMatTmp = trMat;
    int*          srcPtr   = src;

    for( int i = 0; i < zeroOutSize; i++ )
    {
      resi += *srcPtr++ * *trMatTmp++;
    }

    *out++ = Clip3( outputMinimum, outputMaximum, ( int ) ( resi + 64 ) >> 7 );
  }
}

static inline int64_t square( const int d ) { return d * (int64_t)d; }

template<int signedMode> void invTransformCbCr( PelBuf &resCb, PelBuf &resCr )
{
  Pel*  cb  = resCb.buf;
  Pel*  cr  = resCr.buf;
  for( SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride )
  {
    for( SizeType x = 0; x < resCb.width; x++ )
    {
      if      ( signedMode ==  1 )  { cr[x] =  cb[x] >> 1;  }
      else if ( signedMode == -1 )  { cr[x] = -cb[x] >> 1;  }
      else if ( signedMode ==  2 )  { cr[x] =  cb[x]; }
      else if ( signedMode == -2 )  { cr[x] = -cb[x]; }
      else if ( signedMode ==  3 )  { cb[x] =  cr[x] >> 1; }
      else if ( signedMode == -3 )  { cb[x] = -cr[x] >> 1; }
    }
  }
}

// ====================================================================================================================
// TrQuant class member functions
// ====================================================================================================================

TrQuant::TrQuant( class InterPrediction* ip, const TrQuant* other ) : Quant( other )
{
  // allocate temporary buffers
  m_invICT      = m_invICTMem + maxAbsIctMode;
  m_invICT[ 0]  = invTransformCbCr< 0>;
  m_invICT[ 1]  = invTransformCbCr< 1>;
  m_invICT[-1]  = invTransformCbCr<-1>;
  m_invICT[ 2]  = invTransformCbCr< 2>;
  m_invICT[-2]  = invTransformCbCr<-2>;
  m_invICT[ 3]  = invTransformCbCr< 3>;
  m_invICT[-3]  = invTransformCbCr<-3>;

  m_invLfnstNxN = invLfnstNxNCore;

  static_assert( sizeof( ip->m_acYuvPred[0] ) > sizeof( TCoeff ) * ( MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE + MEMORY_ALIGN_DEF_SIZE ), "Buffer to small to be reused!" );
  static_assert( sizeof( ip->m_acYuvPred[1] ) > sizeof( TCoeff ) * ( MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE + MEMORY_ALIGN_DEF_SIZE ), "Buffer to small to be reused!" );
  static_assert( sizeof( ip->m_acYuvPred[2] ) > sizeof( TCoeff ) * ( MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE + MEMORY_ALIGN_DEF_SIZE ), "Buffer to small to be reused!" );

  char* tmp  = ( char* ) ip->m_acYuvPred[0];
  char* blk  = ( char* ) ip->m_acYuvPred[1];
  char* dqnt = ( char* ) ip->m_acYuvPred[2];

  m_tmp  = ( TCoeff* ) ( ( ptrdiff_t ) tmp  + ( MEMORY_ALIGN_DEF_SIZE - ( ( ptrdiff_t ) tmp  & ( MEMORY_ALIGN_DEF_SIZE - 1 ) ) ) );
  m_blk  = ( TCoeff* ) ( ( ptrdiff_t ) blk  + ( MEMORY_ALIGN_DEF_SIZE - ( ( ptrdiff_t ) blk  & ( MEMORY_ALIGN_DEF_SIZE - 1 ) ) ) );
  m_dqnt = ( TCoeff* ) ( ( ptrdiff_t ) dqnt + ( MEMORY_ALIGN_DEF_SIZE - ( ( ptrdiff_t ) dqnt & ( MEMORY_ALIGN_DEF_SIZE - 1 ) ) ) );

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_TCOEFF_OPS
  initTrQuantX86();
#endif
}

void TrQuant::xDeQuant(const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_PARSERESIDUALS, *tu.cu->cs, compID );
  dequant( tu, dstCoeff, compID, cQP );
}

void TrQuant::init( const Picture *pic )
{
  Quant::init( pic );
}

uint32_t TrQuant::getLFNSTIntraMode( int wideAngPredMode )
{
  uint32_t intraMode;

  if( wideAngPredMode < 0 )
  {
    intraMode = ( uint32_t ) ( wideAngPredMode + ( NUM_EXT_LUMA_MODE >> 1 ) + NUM_LUMA_MODE );
  }
  else if( wideAngPredMode >= NUM_LUMA_MODE )
  {
    intraMode = ( uint32_t ) ( wideAngPredMode + ( NUM_EXT_LUMA_MODE >> 1 ) );
  }
  else
  {
    intraMode = ( uint32_t ) wideAngPredMode;
  }

  return intraMode;
}

bool TrQuant::getTransposeFlag( uint32_t intraMode )
{
  return ( ( intraMode >= NUM_LUMA_MODE ) && ( intraMode >= ( NUM_LUMA_MODE + ( NUM_EXT_LUMA_MODE >> 1 ) ) ) ) ||
         ( ( intraMode <  NUM_LUMA_MODE ) && ( intraMode >  DIA_IDX ) );
}

void TrQuant::xInvLfnst( TransformUnit &tu, const ComponentID& compID )
{
  const CompArea& area     = tu.blocks[ compID ];
  const uint32_t  width    = area.width;
  const uint32_t  height   = area.height;
  const uint32_t  lfnstIdx = tu.cu->lfnstIdx();

  if( lfnstIdx && tu.mtsIdx( compID ) != MTS_SKIP && ( CU::isSepTree( *tu.cu ) ? true : isLuma( compID ) ) )
  {
    const bool whge3     = width >= 8 && height >= 8;
    const uint16_t* scan = whge3 ? g_coefTopLeftDiagScan8x8[ g_sizeIdxInfo.idxFrom( width ) ] : g_scanOrder[ SCAN_GROUPED_4x4 ][ g_sizeIdxInfo.idxFrom( width ) ][ g_sizeIdxInfo.idxFrom( height ) ];
    uint32_t intraMode   = 0;

    if( CU::isMIP( *tu.cu, toChannelType( compID ) ) )
    {
      intraMode = PLANAR_IDX;
    }
    else
    {
      intraMode = PU::isLMCMode( tu.cu->intraDir[toChannelType( compID )] ) ? PU::getCoLocatedIntraLumaMode( *tu.cu ) : PU::getFinalIntraMode( *tu.cu, toChannelType( compID ) );
    }

    CHECKD( intraMode > NUM_INTRA_MODE, "Invalid intra mode" );
    CHECKD( lfnstIdx >= 3, "Invalid LFNST index" );

    intraMode                     = getLFNSTIntraMode( PU::getWideAngIntraMode( tu, intraMode, compID ) );
    bool          transposeFlag   = getTransposeFlag( intraMode );
    const int     sbSize          = whge3 ? 8 : 4;
    bool          tu4x4Flag       = ( width == 4 && height == 4 );
    bool          tu8x8Flag       = ( width == 8 && height == 8 );
    TCoeff*       lfnstTemp;
    TCoeff*       coeffTemp;
    int y;
    lfnstTemp = m_tempInMatrix; // inverse spectral rearrangement
    coeffTemp = m_dqnt;
    TCoeff * dst = lfnstTemp;
    for( y = 0; y < 16; y++ )
    {
      *dst++ = coeffTemp[ scan[y] ];
    }

    m_invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, ( tu4x4Flag || tu8x8Flag ) ? 8 : 16 );

    lfnstTemp = m_tempOutMatrix; // inverse spectral rearrangement

    if( transposeFlag )
    {
      if( sbSize == 4 )
      {
        for( y = 0; y < 4; y++ )
        {
          coeffTemp[ 0 ] = lfnstTemp[ 0 ];  coeffTemp[ 1 ] = lfnstTemp[  4 ];
          coeffTemp[ 2 ] = lfnstTemp[ 8 ];  coeffTemp[ 3 ] = lfnstTemp[ 12 ];
          lfnstTemp++;
          coeffTemp += width;
        }
      }
      else // ( sbSize == 8 )
      {
        for( y = 0; y < 8; y++ )
        {
          coeffTemp[ 0 ] = lfnstTemp[  0 ];  coeffTemp[ 1 ] = lfnstTemp[  8 ];
          coeffTemp[ 2 ] = lfnstTemp[ 16 ];  coeffTemp[ 3 ] = lfnstTemp[ 24 ];
          if( y < 4 )
          {
            coeffTemp[ 4 ] = lfnstTemp[ 32 ];  coeffTemp[ 5 ] = lfnstTemp[ 36 ];
            coeffTemp[ 6 ] = lfnstTemp[ 40 ];  coeffTemp[ 7 ] = lfnstTemp[ 44 ];
          }
          lfnstTemp++;
          coeffTemp += width;
        }
      }
    }
    else
    {
      for( y = 0; y < sbSize; y++ )
      {
        uint32_t uiStride = ( y < 4 ) ? sbSize : 4;
        ::memcpy( coeffTemp, lfnstTemp, uiStride * sizeof( TCoeff ) );
        lfnstTemp += uiStride;
        coeffTemp += width;
      }
    }

    tu.maxScanPosX[compID] = std::max<int>( tu.maxScanPosX[compID], std::min<int>( width  - 1, 7 ) );
    tu.maxScanPosY[compID] = std::max<int>( tu.maxScanPosY[compID], std::min<int>( height - 1, 7 ) );
  }
}

void TrQuant::invTransformNxN( TransformUnit &tu, const ComponentID &compID, PelBuf &pResi, const QpParam &cQP )
{
  CompArea &area    = tu.blocks[compID];
  uint32_t uiWidth  = area.width;
  uint32_t uiHeight = area.height;

  CoeffBuf coeff( m_dqnt, uiWidth, uiHeight );
  coeff.memset( 0 );

  xDeQuant( tu, coeff, compID, cQP );

  DTRACE_COEFF_BUF( D_TCOEFF, coeff, tu, tu.cu->predMode(), compID );

  if( tu.cu->sps->getUseLFNST() )
  {
    xInvLfnst( tu, compID );
  }

  if( tu.mtsIdx( compID )== 1 )
  {
    xITransformSkip( coeff, pResi, tu, compID );
  }
  else
  {
    xIT( tu, compID, coeff, pResi );
  }

  DTRACE_PEL_BUF( D_RESIDUALS, pResi, tu, tu.cu->predMode(), compID);
}

void TrQuant::invTransformICT( const TransformUnit &tu, PelBuf &resCb, PelBuf &resCr )
{
  CHECKD( Size(resCb) != Size(resCr), "resCb and resCr have different sizes" );
  ( *m_invICT[TU::getICTMode( tu, tu.cu->cs->picHeader->getJointCbCrSignFlag() )] )( resCb, resCr );
}

// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------

void TrQuant::getTrTypes( const TransformUnit& tu, const ComponentID compID, int &trTypeHor, int &trTypeVer )
{
  const bool isCuIntra     = CU::isIntra( *tu.cu );
  const bool isCompLuma    = isLuma( compID );
  const bool isImplicitMTS = isCuIntra && isCompLuma && tu.cu->sps->getUseImplicitMTS() && tu.cu->lfnstIdx() == 0 && tu.cu->mipFlag() == 0;
  const bool isISP         = isCuIntra && isCompLuma && tu.cu->ispMode();

  if( isISP && tu.cu->lfnstIdx() )
  {
    return;
  }

  const int lwidth  = tu.lwidth();
  const int lheight = tu.lheight();

  if( !tu.cu->sps->getUseMTS() )
    return;

  if( isImplicitMTS || isISP )
  {
    bool widthDstOk   = lwidth  >= 4 && lwidth  <= 16;
    bool heightDstOk  = lheight >= 4 && lheight <= 16;

    if( widthDstOk )
      trTypeHor = DST7;
    if( heightDstOk )
      trTypeVer = DST7;

    return;
  }

  const bool isCuInter     = CU::isInter( *tu.cu ) && isCompLuma;
  const bool isExplicitMTS = isCuIntra ? tu.cu->sps->getUseIntraMTS() && isCompLuma : tu.cu->sps->getUseInterMTS() && isCuInter;
  const bool isSBT         = isCuInter && tu.cu->sbtInfo();

  if( isSBT )
  {
    const uint8_t sbtIdx = CU::getSbtIdx( *tu.cu );
    const uint8_t sbtPos = CU::getSbtPos( *tu.cu );

    if( sbtIdx == SBT_VER_HALF || sbtIdx == SBT_VER_QUAD )
    {
      CHECK( lwidth > MTS_INTER_MAX_CU_SIZE, "wrong" );
      if( lheight > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if( sbtPos == SBT_POS0 )  { trTypeHor = DCT8;  trTypeVer = DST7; }
        else                      { trTypeHor = DST7;  trTypeVer = DST7; }
      }
    }
    else
    {
      CHECK( lheight > MTS_INTER_MAX_CU_SIZE, "wrong" );
      if( lwidth > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if( sbtPos == SBT_POS0 )  { trTypeHor = DST7;  trTypeVer = DCT8; }
        else                      { trTypeHor = DST7;  trTypeVer = DST7; }
      }
    }
    return;
  }
  else if( isExplicitMTS )
  {
    if (tu.mtsIdx( compID ) > MTS_SKIP)
    {
      int indHor = (tu.mtsIdx( compID ) - MTS_DST7_DST7) & 1;
      int indVer = (tu.mtsIdx( compID ) - MTS_DST7_DST7) >> 1;
      trTypeHor = indHor ? DCT8 : DST7;
      trTypeVer = indVer ? DCT8 : DST7;
    }
  }
}

void TrQuant::xIT( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual )
{
  const int      width                  = pCoeff.width;
  const int      height                 = pCoeff.height;
  const unsigned maxLog2TrDynamicRange  = tu.cu->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cu->sps->getBitDepth();
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift;
  const TCoeff   clipMinimum            = -( 1 << maxLog2TrDynamicRange );
  const TCoeff   clipMaximum            =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const uint32_t transformWidthIndex    = getLog2(width ) - 1;                                // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = getLog2(height) - 1;                                // nLog2HeightMinus1, since transform start from 2-point

  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

  getTrTypes( tu, compID, trTypeHor, trTypeVer );

  if( tu.maxScanPosX[compID] == 0 && tu.maxScanPosY[compID] == 0 && trTypeHor == DCT2 && trTypeVer == DCT2 )
  {
    int dcVal = 0;

    if( width > 1 && height > 1 )
    {
      const int shift_1st = TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC;
      const int shift_2nd = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;

      dcVal = ( ( pCoeff.buf[0] * ( 1 << TRANSFORM_MATRIX_SHIFT ) ) + ( 1 << ( shift_1st - 1 ) ) ) >> shift_1st;
      dcVal = ( ( dcVal * ( 1 << TRANSFORM_MATRIX_SHIFT ) ) + ( 1 << ( shift_2nd - 1 ) ) ) >> shift_2nd;
    }
    else
    {
      const int shift = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange ) - bitDepth + COM16_C806_TRANS_PREC;
      dcVal = ( ( pCoeff.buf[0] * ( 1 << TRANSFORM_MATRIX_SHIFT ) ) + ( 1 << ( shift - 1 ) ) ) >> shift;
    }

    pResidual.fill( dcVal );
    return;
  }

  const int skipWidth  = std::max<int>( ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0, width  - tu.maxScanPosX[compID] - 1 );
  const int skipHeight = std::max<int>( ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0, height - tu.maxScanPosY[compID] - 1 );

  TCoeff *block = m_blk;
  int shiftlast;

  if( width > 1 && height > 1 ) //2-D transform
  {
    const int      shift_1st              =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; // 1 has been added to shift_1st at the expense of shift_2nd
    const int      shift_2nd              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift_1st < 0, "Negative shift" );
    CHECK( shift_2nd < 0, "Negative shift" );
    TCoeff *tmp   = m_tmp;
    fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, tmp, shift_1st, width, skipWidth, skipHeight, true,  clipMinimum, clipMaximum);
    fastInvTrans[trTypeHor][transformWidthIndex] (tmp,      block, shift_2nd, height,         0, skipWidth, false, clipMinimum, clipMaximum);
    shiftlast = shift_2nd;
  }
  else if( width == 1 ) //1-D vertical transform
  {
    int shift = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECK( ( transformHeightIndex < 0 ), "There is a problem with the height." );
    fastInvTrans[trTypeVer][transformHeightIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipHeight, false, clipMinimum, clipMaximum );
    shiftlast = shift + 1;
  }
  else //if(iHeight == 1) //1-D horizontal transform
  {
    const int      shift              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECK( ( transformWidthIndex < 0 ), "There is a problem with the width." );
    fastInvTrans[trTypeHor][transformWidthIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipWidth, false, clipMinimum, clipMaximum );
    shiftlast = shift + 1;
  }

  int round = 1 << ( shiftlast - 1 );
  g_tCoeffOps.cpyResiClip[getLog2( width )]( block, pResidual.buf, pResidual.stride, width, height, clipMinimum, clipMaximum, round, shiftlast );
}

/** Wrapper function between HM interface and core NxN transform skipping
 */
void TrQuant::xITransformSkip(const CCoeffBuf     &pCoeff,
                                    PelBuf        &pResidual,
                              const TransformUnit &tu,
                              const ComponentID   &compID)
{
  const CompArea &area  = tu.blocks[compID];
  const int width       = area.width;
  const int height      = area.height;
  
  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      pResidual.at( x, y ) = Pel( pCoeff.at( x, y ) );
    }
  }
}

}
