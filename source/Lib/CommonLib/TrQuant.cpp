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

/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"

#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "CommonLib/TimeProfiler.h"


#include "dtrace_buffer.h"

#include <stdlib.h>
#include <limits>
#include <memory.h>

#include "Quant.h"


InvTrans *fastInvTrans[NUM_TRANS_TYPE][g_numTransformMatrixSizes] =
{
  { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64 },
  { nullptr,            fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, nullptr },
  { nullptr,            fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, nullptr },
};

//! \ingroup CommonLib
//! \{

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
TrQuant::TrQuant()
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

  m_tmp  = ( TCoeff* ) xMalloc( TCoeff, MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE );
  m_blk  = ( TCoeff* ) xMalloc( TCoeff, MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE );
  m_dqnt = ( TCoeff* ) xMalloc( TCoeff, MAX_TU_SIZE_FOR_PROFILE * MAX_TU_SIZE_FOR_PROFILE );
}

TrQuant::~TrQuant()
{
  // delete temporary buffers
  if( m_tmp )
  {
    xFree( m_tmp );
    m_tmp = nullptr;
  }

  if( m_blk )
  {
    xFree( m_blk );
    m_blk = nullptr;
  }

  if( m_dqnt )
  {
    xFree( m_dqnt );
    m_dqnt = nullptr;
  }
}


void TrQuant::xDeQuant(const TransformUnit &tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_PARSERESIDUALS, *tu.cu->cs, compID );
  dequant( tu, dstCoeff, compID, cQP );
}

void TrQuant::init( Slice *slice )
{
  Quant::init( slice );
}

void TrQuant::invLfnstNxN( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  int             maxLog2TrDynamicRange =  15;
  const TCoeff    outputMinimum         = -( 1 << maxLog2TrDynamicRange );
  const TCoeff    outputMaximum         =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const int8_t*   trMat                 =  ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ];
  const int       trSize                =  ( size > 4 ) ? 48 : 16;
  int             resi;
  int*            out                   =  dst;

  CHECK( index > 2, "wrong" );

  for( int j = 0; j < trSize; j++, trMat++ )
  {
    resi = 0;
    const int8_t* trMatTmp = trMat;
    int*          srcPtr   = src;

    for( int i = 0; i < zeroOutSize; i++, trMatTmp += trSize )
    {
      resi += *srcPtr++ * *trMatTmp;
    }

    *out++ = Clip3( outputMinimum, outputMaximum, ( int ) ( resi + 64 ) >> 7 );
  }
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

  if( lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && ( CU::isSepTree( *tu.cu ) ? true : isLuma( compID ) ) )
  {
    const bool whge3     = width >= 8 && height >= 8;
    const uint32_t* scan = whge3 ? g_coefTopLeftDiagScan8x8[ g_sizeIdxInfo.idxFrom( width ) ] : g_scanOrder[ SCAN_GROUPED_4x4 ][ SCAN_DIAG ][ g_sizeIdxInfo.idxFrom( width ) ][ g_sizeIdxInfo.idxFrom( height ) ];
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

    invLfnstNxN( m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[ intraMode ], lfnstIdx - 1, sbSize, ( tu4x4Flag || tu8x8Flag ) ? 8 : 16 );

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

  if( tu.cu->cs->sps->getUseLFNST() )
  {
    xInvLfnst( tu, compID );
  }

  if( tu.mtsIdx[compID] == 1 )
  {
    xITransformSkip( coeff, pResi, tu, compID );
  }
  else
  {
    xIT( tu, compID, coeff, pResi );
  }

  DTRACE_PEL_BUF( D_RESIDUALS, pResi, tu, tu.cu->predMode(), compID);
  invRdpcmNxN( tu, compID, pResi );
}

void TrQuant::invRdpcmNxN(TransformUnit& tu, const ComponentID &compID, PelBuf &pcResidual)
{
  const CompArea &area    = tu.blocks[compID];

  if( CU::isRDPCMEnabled( *tu.cu ) && tu.mtsIdx[compID] == 1 )
  {
    const uint32_t uiWidth  = area.width;
    const uint32_t uiHeight = area.height;

    RDPCMMode rdpcmMode = RDPCM_OFF;

    if (tu.cu->predMode() == MODE_INTRA)
    {
      const ChannelType chType = toChannelType(compID);
      const uint32_t uiChFinalMode = PU::getFinalIntraMode( *tu.cu, chType );

      if (uiChFinalMode == VER_IDX || uiChFinalMode == HOR_IDX)
      {
        rdpcmMode = (uiChFinalMode == VER_IDX) ? RDPCM_VER : RDPCM_HOR;
      }
    }
    else  // not intra case
    {
      rdpcmMode = RDPCMMode(tu.rdpcm[compID]);
    }

    const TCoeff pelMin = (TCoeff) std::numeric_limits<Pel>::min();
    const TCoeff pelMax = (TCoeff) std::numeric_limits<Pel>::max();

    if (rdpcmMode == RDPCM_VER)
    {
      for (uint32_t uiX = 0; uiX < uiWidth; uiX++)
      {
        TCoeff accumulator = pcResidual.at(uiX, 0); // 32-bit accumulator

        for (uint32_t uiY = 1; uiY < uiHeight; uiY++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
    else if (rdpcmMode == RDPCM_HOR)
    {
      for (uint32_t uiY = 0; uiY < uiHeight; uiY++)
      {
        TCoeff accumulator = pcResidual.at(0, uiY);

        for (uint32_t uiX = 1; uiX < uiWidth; uiX++)
        {
          accumulator            += pcResidual.at(uiX, uiY);
          pcResidual.at(uiX, uiY) = (Pel) Clip3<TCoeff>(pelMin, pelMax, accumulator);
        }
      }
    }
  }
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
  const bool isImplicitMTS = isCuIntra && isCompLuma && tu.cu->cs->sps->getUseImplicitMTS() && tu.cu->lfnstIdx() == 0 && tu.cu->mipFlag() == 0;
  const bool isISP         = isCuIntra && isCompLuma && tu.cu->ispMode();

  if( isISP && tu.cu->lfnstIdx() )
  {
    return;
  }

  const int lwidth  = tu.lwidth();
  const int lheight = tu.lheight();

  if( !tu.cu->cs->sps->getUseMTS() )
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
  const bool isExplicitMTS = isCuIntra ? tu.cu->cs->sps->getUseIntraMTS() && isCompLuma : tu.cu->cs->sps->getUseInterMTS() && isCuInter;
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
    if (tu.mtsIdx[compID] > MTS_SKIP)
    {
      int indHor = (tu.mtsIdx[compID] - MTS_DST7_DST7) & 1;
      int indVer = (tu.mtsIdx[compID] - MTS_DST7_DST7) >> 1;
      trTypeHor = indHor ? DCT8 : DST7;
      trTypeVer = indVer ? DCT8 : DST7;
    }
  }
}

void TrQuant::xIT( const TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pCoeff, PelBuf &pResidual )
{
  const int      width                  = pCoeff.width;
  const int      height                 = pCoeff.height;
  const unsigned maxLog2TrDynamicRange  = tu.cu->cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cu->cs->sps->getBitDepth(              toChannelType( compID ) );
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift;
  const TCoeff   clipMinimum            = -( 1 << maxLog2TrDynamicRange );
  const TCoeff   clipMaximum            =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const uint32_t transformWidthIndex    = getLog2(width ) - 1;                                // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = getLog2(height) - 1;                                // nLog2HeightMinus1, since transform start from 2-point

  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

  getTrTypes( tu, compID, trTypeHor, trTypeVer );

  const int skipWidth  = std::max<int>( ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0, width  - tu.maxScanPosX[compID] - 1 );
  const int skipHeight = std::max<int>( ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0, height - tu.maxScanPosY[compID] - 1 );

  TCoeff *block = m_blk;

  if( width > 1 && height > 1 ) //2-D transform
  {
    const int      shift_1st              =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; // 1 has been added to shift_1st at the expense of shift_2nd
    const int      shift_2nd              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift_1st < 0, "Negative shift" );
    CHECK( shift_2nd < 0, "Negative shift" );
    TCoeff *tmp   = m_tmp;
    fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, tmp, shift_1st, width, skipWidth, skipHeight, clipMinimum, clipMaximum);
    fastInvTrans[trTypeHor][transformWidthIndex] (tmp,      block, shift_2nd, height,         0, skipWidth, clipMinimum, clipMaximum);
  }
  else if( width == 1 ) //1-D vertical transform
  {
    int shift = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECK( ( transformHeightIndex < 0 ), "There is a problem with the height." );
    fastInvTrans[trTypeVer][transformHeightIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipHeight, clipMinimum, clipMaximum );
  }
  else //if(iHeight == 1) //1-D horizontal transform
  {
    const int      shift              = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    CHECK( ( transformWidthIndex < 0 ), "There is a problem with the width." );
    fastInvTrans[trTypeHor][transformWidthIndex]( pCoeff.buf, block, shift + 1, 1, 0, skipWidth, clipMinimum, clipMaximum );
  }

#if ENABLE_SIMD_TCOEFF_OPS
  if( width & 3 )
#endif //ENABLE_SIMD_TCOEFF_OPS
  {
    Pel       *dst    = pResidual.buf;
    ptrdiff_t  stride = pResidual.stride;

    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        dst[x] = ( Pel ) *block++;
      }

      dst += stride;
    }
  }
#if ENABLE_SIMD_TCOEFF_OPS
  else if( width & 7 )
  {
    g_tCoeffOps.cpyResi4( block, pResidual.buf, pResidual.stride, width, height );
  }
  else
  {
    g_tCoeffOps.cpyResi8( block, pResidual.buf, pResidual.stride, width, height );
  }
#endif //ENABLE_SIMD_TCOEFF_OPS
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


//! \}
