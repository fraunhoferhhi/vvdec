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

/** \file     LoopFilter.cpp
    \brief    deblocking filter
*/

#include "LoopFilter.h"
#include "Slice.h"
#include "Mv.h"
#include "Unit.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"

#include "Quant.h"

#ifdef TARGET_SIMD_X86
#include "CommonDefX86.h"

#include <simde/x86/sse4.1.h>
#endif

namespace vvdec
{

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define DEBLOCK_SMALLEST_BLOCK  8


#define DEFAULT_INTRA_TC_OFFSET 2 ///< Default intra TC offset

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint16_t LoopFilter::sm_tcTable[MAX_QP + 1 + DEFAULT_INTRA_TC_OFFSET] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,4,4,4,4,5,5,5,5,7,7,8,9,10,10,11,13,14,15,17,19,21,24,25,29,33,36,41,45,51,57,64,71,80,89,100,112,125,141,157,177,198,222,250,280,314,352,395
};
const uint8_t LoopFilter::sm_betaTable[MAX_QP + 1] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,6,7,8,9,10,11,12,13,14,15,16,17,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64
  , 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88
};

// ====================================================================================================================
// utility functions
// ====================================================================================================================


#define BsSet( val, compIdx ) (   ( val ) << ( ( compIdx ) << 1 ) )
#define BsGet( val, compIdx ) ( ( ( val ) >> ( ( compIdx ) << 1 ) ) & 3 )

static const int dbCoeffs7[7] = { 59, 50, 41, 32, 23, 14,  5 };
static const int dbCoeffs5[5] = { 58, 45, 32, 19,  6 };
static const int dbCoeffs3[3] = { 53, 32, 11 };

static inline void xBilinearFilter( Pel* srcP, Pel* srcQ, ptrdiff_t offset, int refMiddle, int refP, int refQ, int numberPSide, int numberQSide, const int* dbCoeffsP, const int* dbCoeffsQ, int tc )
{
  int src;
  const char tc7[7] = { 6, 5, 4, 3, 2, 1, 1 };
  const char tc3[3] = { 6, 4, 2 };
  const char *tcP = ( numberPSide == 3 ) ? tc3 : tc7;
  const char *tcQ = ( numberQSide == 3 ) ? tc3 : tc7;

  for( int pos = 0; pos < numberPSide; pos++ )
  {
    src = srcP[-offset * pos];
    int cvalue = ( tc * tcP[pos] ) >> 1;
    srcP[-offset * pos] = Clip3( src - cvalue, src + cvalue, ( ( refMiddle * dbCoeffsP[pos] + refP * ( 64 - dbCoeffsP[pos] ) + 32 ) >> 6 ) );
  }

  for( int pos = 0; pos < numberQSide; pos++ )
  {
    src = srcQ[ offset * pos];
    int cvalue = ( tc * tcQ[pos] ) >> 1;
    srcQ[ offset * pos] = Clip3( src - cvalue, src + cvalue, ( ( refMiddle * dbCoeffsQ[pos] + refQ * ( 64 - dbCoeffsQ[pos] ) + 32 ) >> 6 ) );
  }
}

void xFilteringPandQCore( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc )
{
  CHECK( numberPSide <= 3 && numberQSide <= 3, "Short filtering in long filtering function" );
  CHECK( numberPSide != 3 && numberPSide != 5 && numberPSide != 7, "invalid numberPSide" );
  CHECK( numberQSide != 3 && numberQSide != 5 && numberQSide != 7, "invalid numberQSide" );

  const int*       dbCoeffsP    = numberPSide == 7 ? dbCoeffs7 : ( numberPSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const int*       dbCoeffsQ    = numberQSide == 7 ? dbCoeffs7 : ( numberQSide == 5 ) ? dbCoeffs5 : dbCoeffs3;

  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    Pel* srcP = src + step * i - offset;
    Pel* srcQ = src + step * i;


    int refP = 0;
    int refQ = 0;
    int refMiddle = 0;

    switch( numberPSide )
    {
    case 7: refP = ( srcP[-6 * offset] + srcP[-7 * offset] + 1 ) >> 1; break;
    case 3: refP = ( srcP[-2 * offset] + srcP[-3 * offset] + 1 ) >> 1; break;
    case 5: refP = ( srcP[-4 * offset] + srcP[-5 * offset] + 1 ) >> 1; break;
    }

    switch( numberQSide )
    {
    case 7: refQ = ( srcQ[6 * offset] + srcQ[7 * offset] + 1 ) >> 1; break;
    case 3: refQ = ( srcQ[2 * offset] + srcQ[3 * offset] + 1 ) >> 1; break;
    case 5: refQ = ( srcQ[4 * offset] + srcQ[5 * offset] + 1 ) >> 1; break;
    }

    if( numberPSide == numberQSide )
    {
      if( numberPSide == 5 )
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] ) + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + 8 ) >> 4;
      }
      else
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] ) + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + +srcP[-6 * offset] + srcQ[6 * offset] + 8 ) >> 4;
      }
    }
    else
    {
      Pel* srcPt = srcP;
      Pel* srcQt = srcQ;
      ptrdiff_t offsetP = -offset;
      ptrdiff_t offsetQ = offset;

      int newNumberQSide = numberQSide;
      int newNumberPSide = numberPSide;

      if( numberQSide > numberPSide )
      {
        std::swap( srcPt, srcQt );
        std::swap( offsetP, offsetQ );
        newNumberQSide = numberPSide;
        newNumberPSide = numberQSide;
      }

      if( newNumberPSide == 7 && newNumberQSide == 5 )
      {
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] ) + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + srcP[-4 * offset] + srcQ[4 * offset] + srcP[-5 * offset] + srcQ[5 * offset] + 8 ) >> 4;
      }
      else if( newNumberPSide == 7 && newNumberQSide == 3 )
      {
        refMiddle = ( 2 * ( srcPt[0] + srcQt[0] ) + srcQt[0] + 2 * ( srcQt[offsetQ] + srcQt[2 * offsetQ] ) + srcPt[offsetP] + srcQt[offsetQ] + srcPt[2 * offsetP] + srcPt[3 * offsetP] + srcPt[4 * offsetP] + srcPt[5 * offsetP] + srcPt[6 * offsetP] + 8 ) >> 4;
      }
      else //if (newNumberPSide == 5 && newNumberQSide == 3)
      {
        refMiddle = ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2 * offset] + srcQ[2 * offset] + srcP[-3 * offset] + srcQ[3 * offset] + 4 ) >> 3;
      }
    }

    xBilinearFilter( srcP, srcQ, offset, refMiddle, refP, refQ, numberPSide, numberQSide, dbCoeffsP, dbCoeffsQ, tc );
    
  }
}

/**
- Deblocking for the luminance component with strong or weak filter
.
\param piSrc           pointer to picture data
\param iOffset         offset value for picture data
\param tc              tc value
\param sw              decision strong/weak filter
\param bPartPNoFilter  indicator to disable filtering on partP
\param bPartQNoFilter  indicator to disable filtering on partQ
\param iThrCut         threshold value for weak filter decision
\param bFilterSecondP  decision weak filter/no filter for partP
\param bFilterSecondQ  decision weak filter/no filter for partQ
\param bitDepthLuma    luma bit depth
*/
void xPelFilterLumaCorePel( Pel* piSrc, const ptrdiff_t iOffset, const int tc, const bool sw, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng )
{
  const Pel m0  = piSrc[-4 * iOffset];
  const Pel m1  = piSrc[-3 * iOffset];
  const Pel m2  = piSrc[-2 * iOffset];
  const Pel m3  = piSrc[-1 * iOffset];
  const Pel m4  = piSrc[ 0          ];
  const Pel m5  = piSrc[ 1 * iOffset];
  const Pel m6  = piSrc[ 2 * iOffset];
  const Pel m7  = piSrc[ 3 * iOffset];

  const char tc3[3] = { 3, 2, 1 };

  if( sw )
  {
    piSrc[-3 * iOffset ] = Clip3( m1 - tc3[2] * tc, m1 + tc3[2] * tc, ( 2*m0 + 3*m1 +   m2 +   m3 +   m4                      + 4 ) >> 3 );
    piSrc[-2 * iOffset ] = Clip3( m2 - tc3[1] * tc, m2 + tc3[1] * tc, (          m1 +   m2 +   m3 +   m4                      + 2 ) >> 2 );
    piSrc[-1 * iOffset ] = Clip3( m3 - tc3[0] * tc, m3 + tc3[0] * tc, (          m1 + 2*m2 + 2*m3 + 2*m4 +   m5               + 4 ) >> 3 );
    piSrc[ 0           ] = Clip3( m4 - tc3[0] * tc, m4 + tc3[0] * tc, (                 m2 + 2*m3 + 2*m4 + 2*m5 +   m6        + 4 ) >> 3 );
    piSrc[ 1 * iOffset ] = Clip3( m5 - tc3[1] * tc, m5 + tc3[1] * tc, (                        m3 +   m4 +   m5 +   m6        + 2 ) >> 2 );
    piSrc[ 2 * iOffset ] = Clip3( m6 - tc3[2] * tc, m6 + tc3[2] * tc, (                        m3 +   m4 +   m5 + 3*m6 + 2*m7 + 4 ) >> 3 );
  }
  else
  {
    /* Weak filter */
    int delta = ( 9 * ( m4 - m3 ) - 3 * ( m5 - m2 ) + 8 ) >> 4;

    if ( abs(delta) < iThrCut )
    {
      delta = Clip3( -tc, tc, delta );
      const int tc2 = tc >> 1;

      piSrc[-iOffset * 1] = ClipPel( m3 + delta, clpRng);
      if( bFilterSecondP )
      {
        const int delta1 = Clip3( -tc2, tc2, ( ( ( ( m1 + m3 + 1 ) >> 1 ) - m2 + delta ) >> 1 ) );
        piSrc[-iOffset * 2] = ClipPel( m2 + delta1, clpRng);
      }

      piSrc[0]        = ClipPel( m4 - delta, clpRng);
      if( bFilterSecondQ )
      {
        const int delta2 = Clip3( -tc2, tc2, ( ( ( ( m6 + m4 + 1 ) >> 1 ) - m5 - delta ) >> 1 ) );
        piSrc[iOffset] = ClipPel( m5 + delta2, clpRng);
      }
    }
  }
}

inline void xPelFilterLumaCore( Pel* piSrc, const ptrdiff_t step, const ptrdiff_t offset, const int tc, const bool sw, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng )
{
  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    xPelFilterLumaCorePel( piSrc + step * i, offset, tc, sw, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );
  }
}

/**
- Deblocking of one line/column for the chrominance component
.
\param piSrc           pointer to picture data
\param iOffset         offset value for picture data
\param tc              tc value
\param bPartPNoFilter  indicator to disable filtering on partP
\param bPartQNoFilter  indicator to disable filtering on partQ
\param bitDepthChroma  chroma bit depth
*/
static inline void xPelFilterChroma( Pel* piSrc, const ptrdiff_t iOffset, const int tc, const bool sw, const ClpRng& clpRng, const bool largeBoundary, const bool isChromaHorCTBBoundary )
{
  int delta;

  const Pel m0 = piSrc[-iOffset * 4];
  const Pel m1 = piSrc[-iOffset * 3];
  const Pel m2 = piSrc[-iOffset * 2];
  const Pel m3 = piSrc[-iOffset * 1];
  const Pel m4 = piSrc[           0];
  const Pel m5 = piSrc[ iOffset * 1];
  const Pel m6 = piSrc[ iOffset * 2];
  const Pel m7 = piSrc[ iOffset * 3];

  if( sw )
  {
    if( isChromaHorCTBBoundary )
    {
      piSrc[-iOffset * 1] = Clip3( m3 - tc, m3 + tc, ( ( 3 * m2 + 2 * m3 +     m4 +     m5 +     m6          + 4 ) >> 3 ) ); // p0
      piSrc[           0] = Clip3( m4 - tc, m4 + tc, ( ( 2 * m2 +     m3 + 2 * m4 +     m5 +     m6 +     m7 + 4 ) >> 3 ) ); // q0
      piSrc[ iOffset * 1] = Clip3( m5 - tc, m5 + tc, ( (     m2 +     m3 +     m4 + 2 * m5 +     m6 + 2 * m7 + 4 ) >> 3 ) ); // q1
      piSrc[ iOffset * 2] = Clip3( m6 - tc, m6 + tc, ( (              m3 +     m4 +     m5 + 2 * m6 + 3 * m7 + 4 ) >> 3 ) ); // q2
    }
    else
    {
      //Pel val = m0 + ( ( m0 + m1 ) << 1 ) + m2 + m3 + m4 + 4;
      //piSrc[-iOffset * 3] = Clip3( m1 - tc, m1 + tc, val >> 3 ); // p2
      //val -= m0 + m1 - m2 - m5;
      //piSrc[-iOffset * 2] = Clip3( m2 - tc, m2 + tc, val >> 3 ); // p1
      //val -= m0 + m2 - m3 - m6;
      //piSrc[-iOffset * 1] = Clip3( m3 - tc, m3 + tc, val >> 3 ); // p0
      //val -= m0 + m3 - m4 - m7;
      //piSrc[           0] = Clip3( m4 - tc, m4 + tc, val >> 3 ); // q0
      //val -= m1 + m4 - m5 - m7;
      //piSrc[ iOffset * 1] = Clip3( m5 - tc, m5 + tc, val >> 3 ); // q1
      //val -= m2 + m5 - m6 - m7;
      //piSrc[ iOffset * 2] = Clip3( m6 - tc, m6 + tc, val >> 3 ); // q2
      piSrc[-iOffset * 3] = Clip3( m1 - tc, m1 + tc, ( ( 3 * m0 + 2 * m1 +     m2 +     m3 +     m4 +                            4 ) >> 3 ) ); // p2
      piSrc[-iOffset * 2] = Clip3( m2 - tc, m2 + tc, ( ( 2 * m0 +     m1 + 2 * m2 +     m3 +     m4 +     m5 +                   4 ) >> 3 ) ); // p1
      piSrc[-iOffset * 1] = Clip3( m3 - tc, m3 + tc, ( (     m0 +     m1 +     m2 + 2 * m3 +     m4 +     m5 +     m6 +          4 ) >> 3 ) ); // p0
      piSrc[           0] = Clip3( m4 - tc, m4 + tc, ( (              m1 +     m2 +     m3 + 2 * m4 +     m5 +     m6 +     m7 + 4 ) >> 3 ) ); // q0
      piSrc[ iOffset * 1] = Clip3( m5 - tc, m5 + tc, ( (                       m2 +     m3 +     m4 + 2 * m5 +     m6 + 2 * m7 + 4 ) >> 3 ) ); // q1
      piSrc[ iOffset * 2] = Clip3( m6 - tc, m6 + tc, ( (                                m3 +     m4 +     m5 + 2 * m6 + 3 * m7 + 4 ) >> 3 ) ); // q2
    }
  }
  else
  {
    delta           = Clip3( -tc, tc, ( ( ( ( m4 - m3 ) *4 ) + m2 - m5 + 4 ) >> 3 ) );

    piSrc[-iOffset] = ClipPel( m3 + delta, clpRng );
    piSrc[       0] = ClipPel( m4 - delta, clpRng );
  }
}
// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

LoopFilter::LoopFilter()
{
  xPelFilterLuma  = xPelFilterLumaCore;
  xFilteringPandQ = xFilteringPandQCore;

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_DBLF
  initLoopFilterX86();
#endif
}

LoopFilter::~LoopFilter()
{}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void LoopFilter::calcFilterStrengthsCTU( CodingStructure& cs, const int ctuRsAddr )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_DBFILTER );

  for( auto &currCU : cs.traverseCUs( ctuRsAddr ) )
  {
    if( currCU.slice->getDeblockingFilterDisable() )
    {
      return;
    }

    calcFilterStrengths( currCU );
  }
}

void LoopFilter::loopFilterCTU( CodingStructure &cs, const ChannelType chType, const int ctuCol, const int ctuLine, const int offset, const DeblockEdgeDir edgeDir ) const
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_DBFILTER );
  const PreCalcValues &pcv = *cs.pcv;

  const bool frstLine = ctuLine == 0;

  const int ly = frstLine ? 0 : ( ctuLine * pcv.maxCUHeight + ( offset ) );
  const int lh = frstLine ? pcv.maxCUHeight + ( offset ) : pcv.maxCUHeight;

  if( ly >= pcv.lumaHeight )
  {
    return;
  }

  const UnitArea ctuArea = clipArea( UnitArea( pcv.chrFormat, Area( ctuCol << pcv.maxCUWidthLog2, ly, pcv.maxCUWidth, lh ) ), *cs.picture );

  if( edgeDir == NUM_EDGE_DIR || edgeDir == EDGE_VER )
  {
    xDeblockCtuArea<EDGE_VER>( cs, ctuArea, chType );
  }
  if( edgeDir == NUM_EDGE_DIR || edgeDir == EDGE_HOR )
  {
    xDeblockCtuArea<EDGE_HOR>( cs, ctuArea, chType );
  }

#if ENABLE_TRACING
  for( unsigned x = 0; x < pcv.widthInCtus; x++ )
  {
    const UnitArea ctuArea( pcv.chrFormat, Area( x << pcv.maxCUWidthLog2, ly, pcv.maxCUWidth, pcv.maxCUWidth ) );
    DTRACE    ( g_trace_ctx, D_CRC, "CTU %d %d", ctuArea.Y().x, ctuArea.Y().y );
    DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );
  }
#endif
}
// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/**
 Deblocking filter process in CU-based (the same function as conventional's)

 \param cu               the CU to be deblocked
 \param edgeDir          the direction of the edge in block boundary (horizontal/vertical), which is added newly
*/
template<DeblockEdgeDir edgeDir>
void LoopFilter::xDeblockCtuArea( CodingStructure& cs, const UnitArea& area, const ChannelType chType ) const
{
  if( cs.getCtuData( cs.ctuRsAddr( area.lumaPos(), CH_L ) ).cuPtr[0][0]->slice->getDeblockingFilterDisable() )
  {
    return;
  }

  const PreCalcValues& pcv = *cs.pcv;

  const CtuData& ctuData       = cs.getCtuData( cs.ctuRsAddr( area.lumaPos(), CH_L ) );
  const Slice& slice           = *ctuData.slice;
  
  bool doLuma   =   chType == MAX_NUM_CHANNEL_TYPE || isLuma  ( chType );
  bool doChroma = ( chType == MAX_NUM_CHANNEL_TYPE || isChroma( chType ) ) && pcv.chrFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid();
  static constexpr int incx = 4;
  static constexpr int incy = 4;

  const int csx = getChannelTypeScaleX( CH_C, pcv.chrFormat );
  const int csy = getChannelTypeScaleY( CH_C, pcv.chrFormat );

  const ptrdiff_t lfpStride = cs.get4x4MapStride();

  const Position lumaPos = area.lumaPos();
  const Position chrmPos = doChroma ? area.chromaPos() : lumaPos;

  if( doLuma )
  {
    LoopFilterParam const* lfpPtr = cs.getLFPMapPtr( edgeDir, cs.ctuRsAddr( area.Y().pos(), CH_L ) );

    for( int dy = 0; dy < area.lheight(); dy += incy )
    {
      LoopFilterParam const* lineLfpPtr = lfpPtr;
    
      for( int dx = 0; dx < area.lwidth(); dx += incx, lineLfpPtr++ )
      {
        const uint8_t bs = lineLfpPtr->bs;

        if( BsGet( bs, COMPONENT_Y ) )
        {
          xEdgeFilterLuma<edgeDir>( cs, { lumaPos.x + dx, lumaPos.y + dy }, *lineLfpPtr, slice );
        }
      }

      OFFSETY( lfpPtr, lfpStride, 1 );
    }
  }

  if( doChroma )
  {
    LoopFilterParam const* lfpPtr = cs.getLFPMapPtr( edgeDir, cs.ctuRsAddr( area.Y().pos(), CH_L ) );

    const int cincy = edgeDir == EDGE_VER ? ( incy >> csy )
                                          : ( ( DEBLOCK_SMALLEST_BLOCK << csy ) / incy * ( incy >> csy ) );
    const int cincx = edgeDir == EDGE_HOR ? ( incx >> csx )
                                          : ( ( DEBLOCK_SMALLEST_BLOCK << csx ) / incx * ( incx >> csx ) );
    const int cstepy = cincy / ( incy >> csy );
    const int cstepx = cincx / ( incx >> csx );

    for( int cdy = 0; cdy < area.chromaSize().height; cdy += cincy )
    {
      LoopFilterParam const* lineLfpPtr = lfpPtr;

      for( int cdx = 0; cdx < area.chromaSize().width; cdx += cincx, lineLfpPtr += cstepx )
      {
        const uint8_t bs = lineLfpPtr->bs;

        if( BsGet( bs, COMPONENT_Cb ) | BsGet( bs, COMPONENT_Cr ) )
        {
          xEdgeFilterChroma<edgeDir>( cs, { chrmPos.x + cdx, chrmPos.y + cdy }, *lineLfpPtr, slice );
        }
      }

      OFFSETY( lfpPtr, lfpStride, cstepy );
    }
  }
}

void LoopFilter::calcFilterStrengths( const CodingUnit& cu ) const
{
  // reenable this, if not checking in outer loop (currently calcFilterStrengthsCTU):
  //  if( cu.slice->getDeblockingFilterDisable() )
  //  {
  //    return;
  //  }

  const PreCalcValues& pcv = *cu.cs->pcv;
  const Area area          = cu.blocks[cu.chType()];

  bool horEdgeFilter    = false;
  bool verEdgeFilter    = false;
  int  numHorVirBndry   = 0;
  int  numVerVirBndry   = 0;
  int  horVirBndryPos[] = { 0, 0, 0 };
  int  verVirBndryPos[] = { 0, 0, 0 };

  const uint8_t channelScaleX = getChannelTypeScaleX( cu.chType(), cu.chromaFormat );
  const uint8_t channelScaleY = getChannelTypeScaleY( cu.chType(), cu.chromaFormat );

  const bool isCuCrossedByVirtualBoundaries = isCrossedByVirtualBoundaries( cu.cs->picHeader.get(),
                                                                            Area( area.x << channelScaleX, area.y << channelScaleY, area.width << channelScaleX, area.height << channelScaleY ),
                                                                            numHorVirBndry, numVerVirBndry,
                                                                            horVirBndryPos, verVirBndryPos );
  if( isCuCrossedByVirtualBoundaries )
  {
    CHECK( numHorVirBndry >= (int)( sizeof(horVirBndryPos) / sizeof(horVirBndryPos[0]) ), "Too many virtual boundaries" );
    CHECK( numHorVirBndry >= (int)( sizeof(verVirBndryPos) / sizeof(verVirBndryPos[0]) ), "Too many virtual boundaries" );
  }
  

  static constexpr int subBlockSize = 8;
  const CodingUnit& currPU  = cu;
  const Area& areaPu            = area;
  LFCUParam stLFCUParam         { xGetLoopfilterParam( cu ) };
  const UnitScale scaling       = cu.cs->getScaling( UnitScale::LF_PARAM_MAP, cu.chType());
  CtuData& ctuData              = *cu.ctuData;
  // for SUBPU ATMVP and Affine, more PU deblocking needs to be found, for ISP the chroma block will be deferred to the last luma block,
  // so the processing order is different. For all other cases the boundary strenght can be directly obtained in the TU loop.
  const bool refineBs     = ( currPU.mergeFlag() && currPU.mergeType() == MRG_TYPE_SUBPU_ATMVP ) || currPU.affineFlag() || cu.ispMode();

  const int maskBlkX = ~( ( 1 << scaling.posx ) - 1 );
  const int maskBlkY = ~( ( 1 << scaling.posy ) - 1 );

  const bool pqCuSameCtuHor = ( area.y & ( pcv.maxCUHeightMask >> getChannelTypeScaleY( cu.chType(), pcv.chrFormat ) ) ) > 0;
  const bool pqCuSameCtuVer = ( area.x & ( pcv.maxCUWidthMask  >> getChannelTypeScaleX( cu.chType(), pcv.chrFormat ) ) ) > 0;

  for( auto currTU = &cu.firstTU; currTU; currTU = currTU->next )
  {
    const Area& areaTu = currTU->blocks[currTU->chType()];

    verEdgeFilter = ( areaTu.x & maskBlkX ) == area.x ? stLFCUParam.leftEdge : true;
    horEdgeFilter = ( areaTu.y & maskBlkY ) == area.y ? stLFCUParam.topEdge  : true;

    const bool pqSameCtuHor = ( areaTu.y & maskBlkY ) == area.y ? pqCuSameCtuHor : true;
    const bool pqSameCtuVer = ( areaTu.x & maskBlkX ) == area.x ? pqCuSameCtuVer : true;

    if( isCuCrossedByVirtualBoundaries )
    {
      xDeriveEdgefilterParam( Position( ( areaTu.x & maskBlkX ) << channelScaleX, ( areaTu.y & maskBlkY ) << channelScaleY ),
                              numVerVirBndry, numHorVirBndry,
                              verVirBndryPos, horVirBndryPos,
                              verEdgeFilter,  horEdgeFilter );
    }

    xSetMaxFilterLengthPQFromTransformSizes<EDGE_VER>( cu, *currTU, verEdgeFilter, !refineBs, ctuData, pqSameCtuVer );
    xSetMaxFilterLengthPQFromTransformSizes<EDGE_HOR>( cu, *currTU, horEdgeFilter, !refineBs, ctuData, pqSameCtuHor );
  }

  if( !refineBs ) return;

  if( ( currPU.mergeFlag() && currPU.mergeType() == MRG_TYPE_SUBPU_ATMVP ) || currPU.affineFlag() )
  {
    CHECK( cu.chType() != CH_L, "This path is only valid for single tree blocks!" );

    for( int off = subBlockSize; off < areaPu.width; off += subBlockSize )
    {
      const Area mvBlockV( cu.Y().x + off, cu.Y().y, subBlockSize, cu.Y().height );
      verEdgeFilter = true;
      if( isCuCrossedByVirtualBoundaries )
      {
        xDeriveEdgefilterParam( mvBlockV,
                                numVerVirBndry, 0,
                                verVirBndryPos, horVirBndryPos,
                                verEdgeFilter, horEdgeFilter );
      }

      xSetEdgeFilterInsidePu<EDGE_VER>( cu, mvBlockV, verEdgeFilter, ctuData );
    }

    xSetMaxFilterLengthPQForCodingSubBlocks<EDGE_VER>( cu, ctuData );

    for( int off = subBlockSize; off < areaPu.height; off += subBlockSize )
    {
      const Area mvBlockH( cu.Y().x, cu.Y().y + off, cu.Y().width, subBlockSize );
      horEdgeFilter = true;
      if( isCuCrossedByVirtualBoundaries )
      {
        xDeriveEdgefilterParam( mvBlockH,
                                0, numHorVirBndry,
                                verVirBndryPos, horVirBndryPos,
                                verEdgeFilter, horEdgeFilter );
      }

      xSetEdgeFilterInsidePu<EDGE_HOR>( cu, mvBlockH, horEdgeFilter, ctuData );
    }

    xSetMaxFilterLengthPQForCodingSubBlocks<EDGE_HOR>( cu, ctuData );
  }

#if ENABLE_SIMD_DBLF && defined( TARGET_SIMD_X86 )
  const bool           useSimd = read_x86_extension_flags() > x86_simd::SCALAR;
#else
  const bool           useSimd = false;
#endif
  const unsigned uiPelsInPartX = pcv.minCUWidth >> channelScaleX;
  const unsigned uiPelsInPartY = pcv.minCUHeight >> channelScaleY;
  const ptrdiff_t       lfpPos = cu.cs->inCtuPos( area.pos(), cu.chType() );

  const CodingUnit* cuP        = cu.left;
  const ChannelType chType     = cu.chType();

  {
    LoopFilterParam* lfpPtrV   = cu.cs->getLFPMapPtr( EDGE_VER, cu.cs->ctuRsAddr( area.pos(), cu.chType() ) );
    ptrdiff_t        lfpStride = cu.cs->get4x4MapStride();
                     lfpPtrV  += lfpPos;

    for( int y = 0; y < area.height; y += uiPelsInPartY )
    {
      LoopFilterParam* lineLfpPtrV = lfpPtrV;
      
      cuP = cuP && cuP->blocks[chType].y + cuP->blocks[chType].height > area.y + y ? cuP : cu.cs->getCU( Position{ area.x - 1, area.y + y }, chType );

      for( int x = 0; x < area.width; x += uiPelsInPartX )
      {
        if( lineLfpPtrV->filterEdge( cu.chType() ) ) xGetBoundaryStrengthSingle<EDGE_VER>( *lineLfpPtrV, cu, Position{ area.x + x, area.y + y }, x ? cu : *cuP, ctuData, x ? true : pqCuSameCtuVer, useSimd );

        lineLfpPtrV->bs &= ~BsSet( 3, MAX_NUM_COMPONENT );

        INCX( lineLfpPtrV, lfpStride );
      }

      INCY( lfpPtrV, lfpStride );
    }
  }

  cuP = cu.above;

  {
    LoopFilterParam* lfpPtrH   = cu.cs->getLFPMapPtr( EDGE_HOR, cu.cs->ctuRsAddr( area.pos(), cu.chType() ) );
    ptrdiff_t        lfpStride = cu.cs->get4x4MapStride();
                     lfpPtrH  += lfpPos;

    for( int y = 0; y < area.height; y += uiPelsInPartY )
    {
      LoopFilterParam* lineLfpPtrH = lfpPtrH;

      for( int x = 0; x < area.width; x += uiPelsInPartX )
      {
        cuP = ( y || ( cuP && cuP->blocks[chType].x + cuP->blocks[chType].width > area.x + x ) ) ? cuP : cu.cs->getCU( Position{ area.x + x, area.y - 1 }, chType );

        if( lineLfpPtrH->filterEdge( cu.chType() ) ) xGetBoundaryStrengthSingle<EDGE_HOR>( *lineLfpPtrH, cu, Position{ area.x + x, area.y + y }, y ? cu : *cuP, ctuData, y ? true : pqCuSameCtuHor, useSimd );

        lineLfpPtrH->bs &= ~BsSet( 3, MAX_NUM_COMPONENT );

        INCX( lineLfpPtrH, lfpStride );
      }

      INCY( lfpPtrH, lfpStride );
    }
  }
}

inline void LoopFilter::xDeriveEdgefilterParam( const Position pos, const int numVerVirBndry, const int numHorVirBndry, const int verVirBndryPos[], const int horVirBndryPos[], bool &verEdgeFilter, bool &horEdgeFilter ) const
{
  for (int i = 0; i < numVerVirBndry; i++)
  {
    if (verVirBndryPos[i] == pos.x)
    {
      verEdgeFilter = false;
      break;
    }
  }

  for (int i = 0; i < numHorVirBndry; i++)
  {
    if (horVirBndryPos[i] == pos.y)
    {
      horEdgeFilter = false;
      break;
    }
  }
}

template<DeblockEdgeDir edgeDir> inline PosType parlPos( const Position& );
template<DeblockEdgeDir edgeDir> inline PosType perpPos( const Position& );

template<DeblockEdgeDir edgeDir> inline SizeType parlSize( const Size& );
template<DeblockEdgeDir edgeDir> inline SizeType perpSize( const Size& );

template<> inline PosType parlPos<EDGE_HOR>( const Position& pos ) { return pos.x; }
template<> inline PosType perpPos<EDGE_HOR>( const Position& pos ) { return pos.y; }
template<> inline PosType parlPos<EDGE_VER>( const Position& pos ) { return pos.y; }
template<> inline PosType perpPos<EDGE_VER>( const Position& pos ) { return pos.x; }

template<> inline SizeType parlSize<EDGE_HOR>( const Size& size ) { return size.width; }
template<> inline SizeType perpSize<EDGE_HOR>( const Size& size ) { return size.height; }
template<> inline SizeType parlSize<EDGE_VER>( const Size& size ) { return size.height; }
template<> inline SizeType perpSize<EDGE_VER>( const Size& size ) { return size.width; }

template<DeblockEdgeDir edgeDir>
void LoopFilter::xSetMaxFilterLengthPQForCodingSubBlocks( const CodingUnit& cu, CtuData& ctuData ) const
{
  static constexpr
        int subBlockSize = 8;
  const int minCUWidth   = cu.cs->pcv->minCUWidth;
  const int minCUHeight  = cu.cs->pcv->minCUHeight;
  const int xInc         = edgeDir ? minCUWidth   : subBlockSize;
  const int yInc         = edgeDir ? subBlockSize : minCUHeight;
  
  LoopFilterParam* lfpPtrL   = ctuData.lfParam[edgeDir] + cu.cs->inCtuPos( cu.lumaPos(), CH_L );
  ptrdiff_t        lfpStride = cu.cs->get4x4MapStride();
  const UnitScale  scaling   = cu.cs->getScaling( UnitScale::LF_PARAM_MAP, CHANNEL_TYPE_LUMA );

  for( int y = 0; y < cu.Y().height; y += yInc )
  {
    LoopFilterParam* lfpPtr = lfpPtrL;

    for( int x = 0; x < cu.Y().width; x += xInc )
    {
      const PosType    perpVal   = edgeDir ? y : x;

      uint8_t maxFLP, maxFLQ;
      uint8_t te = 0;
      if( lfpPtr->sideMaxFiltLength & 128 )
      {
        te = 128;
        maxFLQ = std::min<int>( lfpPtr->sideMaxFiltLength        & 7, 5 );
        maxFLP =              ( lfpPtr->sideMaxFiltLength >> 4 ) & 7;

        if( perpVal > 0 )
        {
          maxFLP = std::min<int>( maxFLP, 5 );
        }
      }
      else if( perpVal > 0 && ( GET_OFFSET( lfpPtr, lfpStride, -    ( 1 - edgeDir ), -    edgeDir )->sideMaxFiltLength & 128 || ( perpVal + 4 >= perpSize<edgeDir>( cu.Y() ) ) || GET_OFFSET( lfpPtr, lfpStride,     ( 1 - edgeDir ),     edgeDir )->sideMaxFiltLength & 128 ) )
      {
        maxFLP = maxFLQ = 1;
      }
      else if( perpVal > 0 && ( (edgeDir ? (y == 8) : (x == 8)) || GET_OFFSET( lfpPtr, lfpStride, -2 * ( 1 - edgeDir ), -2 * edgeDir )->sideMaxFiltLength & 128 || ( perpVal + 8 >= perpSize<edgeDir>( cu.Y() ) ) || GET_OFFSET( lfpPtr, lfpStride, 2 * ( 1 - edgeDir ), 2 * edgeDir )->sideMaxFiltLength & 128 ) )
      {
        maxFLP = maxFLQ = 2;
      }
      else
      {
        maxFLP = maxFLQ = 3;
      }
      unsigned newVal = maxFLP;
      newVal <<= 4;
      newVal += maxFLQ;
      newVal += te;

      lfpPtr->sideMaxFiltLength = newVal;

      OFFSETX( lfpPtr, lfpStride, scaling.scaleHor( xInc ) );
    }

    OFFSETY( lfpPtrL, lfpStride, scaling.scaleVer( yInc ) );
  }
}

static inline TransformUnit const* getTU( const CodingUnit& cu, const Position& pos, const ChannelType chType )
{
  const TransformUnit* ptu = &cu.firstTU;

  while( !( ptu->blocks[chType].x + ptu->blocks[chType].width > pos.x && ptu->blocks[chType].y + ptu->blocks[chType].height > pos.y ) )
  {
    ptu = ptu->next;
  }

  return ptu;
}

template<DeblockEdgeDir edgeDir>
void LoopFilter::xSetMaxFilterLengthPQFromTransformSizes( const CodingUnit& cu, const TransformUnit& currTU, const bool bValue, bool deriveBdStrngt, CtuData& ctuData, bool pqSameCtu ) const
{
  const PreCalcValues &pcv = *cu.cs->pcv;

  ChannelType start  = CH_L;
  ChannelType end    = CH_C;

  const bool      dt = CU::isSepTree( cu );
#if ENABLE_SIMD_DBLF && defined( TARGET_SIMD_X86 )
  const bool useSimd = read_x86_extension_flags() > x86_simd::SCALAR;
#else
  const bool useSimd = false;
#endif

  if( dt )
  {
    if( cu.chType() == CH_L )
    {
      end = CH_L;
    }
    else
    {
      start = CH_C;
    }
  }

  if( start != end && ( !isChromaEnabled( pcv.chrFormat ) || !currTU.Cb().valid() ) )
  {
    end = CH_L;
  }

  const int csx    = ( start != end ) ? getChannelTypeScaleX( CH_C, pcv.chrFormat ) : 0;
  const int csy    = ( start != end ) ? getChannelTypeScaleY( CH_C, pcv.chrFormat ) : 0;
  const Area& area = currTU.blocks[end];

  for( int ct = start; ct <= end; ct++ )
  {
    const ChannelType ch  = ( ChannelType ) ct;
    const bool        vld = isLuma( ch ) ? currTU.Y().valid() : currTU.Cb().valid();

    if( vld && perpPos<edgeDir>( currTU.blocks[ch] ) != 0 )
    {
      LoopFilterParam* lfpPtr    = ctuData.lfParam[edgeDir] + cu.cs->inCtuPos( currTU.blocks[ct], ch );
      ptrdiff_t lfpStride        = cu.cs->get4x4MapStride();

      const int         inc      = edgeDir ? pcv.minCUWidth  >> getChannelTypeScaleX( ch, cu.chromaFormat )
                                           : pcv.minCUHeight >> getChannelTypeScaleY( ch, cu.chromaFormat );

      const CodingUnit* cuNeigh  = edgeDir ? cu.above : cu.left;
      const CodingUnit* cuP      = ( cuNeigh && perpPos<edgeDir>( currTU.blocks[ch   ] ) == perpPos<edgeDir>( cu.blocks[ch   ] ) ) ? cuNeigh : &cu;
      const CodingUnit* cuPfstCh = ( cuNeigh && perpPos<edgeDir>( currTU.blocks[start] ) == perpPos<edgeDir>( cu.blocks[start] ) ) ? cuNeigh : &cu;
      const int         incFst   = edgeDir ? pcv.minCUWidth  >> getChannelTypeScaleX( ChannelType( start ), cu.chromaFormat )
                                           : pcv.minCUHeight >> getChannelTypeScaleY( ChannelType( start ), cu.chromaFormat );

      if( cuP == &cu && ( edgeDir ? cu.above : cu.left ) == nullptr && ( edgeDir ? cu.blocks[ch].pos().y : cu.blocks[ch].pos().x ) > 0 ) //TODO: check for !pps.getLoopFilterAcrossSlicesEnabledFlag() || !pps.getLoopFilterAcrossTilesEnabledFlag()
      {
        const Position posP   { currTU.blocks[   ch].x - ( 1 - edgeDir ), currTU.blocks[   ch].y - edgeDir };
        const Position posPfst{ currTU.blocks[start].x - ( 1 - edgeDir ), currTU.blocks[start].y - edgeDir };
        cuP      = cuP     ->blocks[   ch].contains( posP    ) ? cuP      : cu.cs->getCU( posP,    ch );
        cuPfstCh = cuPfstCh->blocks[start].contains( posPfst ) ? cuPfstCh : cu.cs->getCU( posPfst, start );
      }

      bool bSameCUTUSize = perpPos<edgeDir>( currTU.blocks[ch] ) == perpPos<edgeDir>( cu.blocks[ch] );
      if( ct == end && deriveBdStrngt )
      {
        if( start != end )
        {
          for( int d = 0, dFst = 0; d < parlSize<edgeDir>( currTU.blocks[ch] );  )
          {
            const Position  posQ     { currTU.blocks[ch].x + edgeDir * d, currTU.blocks[ch].y + ( 1 - edgeDir ) * d };
            const Position  posP     = posQ.offset( -( 1 - edgeDir ), -edgeDir );
            const int sizeQSide      = perpSize<edgeDir>( currTU.blocks[ch] );
                            cuP      = parlPos<edgeDir>( cuP->blocks[ch] ) + parlSize<edgeDir>( cuP->blocks[ch] ) > parlPos<edgeDir>( posP )? cuP: cu.cs->getCU( posP, ch );
            const Position  posPfst  = currTU.blocks[start].offset( edgeDir ? dFst : -1, edgeDir ? -1 : dFst );
                            cuPfstCh = parlPos<edgeDir>( cuPfstCh->blocks[start] ) + parlSize<edgeDir>( cuPfstCh->blocks[start] ) > parlPos<edgeDir>( posPfst ) ? cuPfstCh : cu.cs->getCU( posPfst, ChannelType( start ) );
            const TransformUnit &tuP = cuP->firstTU.next == nullptr ? cuP->firstTU : *getTU( *cuP, posP, ch );
            const int sizePSide      = perpSize<edgeDir>( tuP.blocks[ch] );
            LoopFilterParam& lfp     = *lfpPtr;

            lfp.setFilterCMFL( ( sizeQSide >= 8 && sizePSide >= 8 ) ? 1 : 0 );
            if( bValue )
              xGetBoundaryStrengthSingle<edgeDir>( lfp, cu, Position( ( area.x + edgeDir * d ) << csx, ( area.y + ( 1 - edgeDir ) * d ) << csy ), *cuPfstCh, ctuData, pqSameCtu, useSimd );
            lfp.bs &= ~BsSet( 3, MAX_NUM_COMPONENT );

            if( !CU::isIntra( cu ) && !CU::isIntra( *cuP ) && cuP == cuPfstCh && cu.geoFlag() == false && cuP->geoFlag() == false )
            {
              int distance = parlPos<edgeDir>( tuP.blocks[ch] ) + parlSize<edgeDir>( tuP.blocks[ch] ) - parlPos<edgeDir>( posQ );
              if( distance > parlSize<edgeDir>( currTU.blocks[ch] ) - d )
                distance = parlSize<edgeDir>( currTU.blocks[ch] ) - d; // range check
              if( distance > inc && !cuP->affineFlag() && !( cuP->mergeFlag() && cuP->mergeType() == MRG_TYPE_SUBPU_ATMVP ) )
              {
                LoopFilterParam param = lfp;
                for( int j = inc; j < distance; j += inc, d += inc, dFst += incFst )
                {
                  OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
                  *lfpPtr = param;
                }
              }
            }
            OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
            d    += inc;
            dFst += incFst;
          }
        }
        else
        {
          // old logical
          for( int d = 0, dFst = 0; d < parlSize<edgeDir>( currTU.blocks[ch] ); d += inc, dFst += incFst )
          {
            const Position  posQ     { currTU.blocks[ch].x + edgeDir * d, currTU.blocks[ch].y + ( 1 - edgeDir ) * d };
            const Position  posP     = posQ.offset( -( 1 - edgeDir ), -edgeDir );
            const Position  posPfst  = currTU.blocks[start].offset( edgeDir ? dFst : -1, edgeDir ? -1 : dFst );
            const int sizeQSide      = perpSize<edgeDir>( currTU.blocks[ch] );
                            cuP      =                parlPos<edgeDir>( cuP->     blocks[ch   ] ) + parlSize<edgeDir>( cuP->     blocks[ch   ] ) > parlPos<edgeDir>( posP )    ? cuP      : cu.cs->getCU( posP,                 ch );
                            cuPfstCh = start != end ? parlPos<edgeDir>( cuPfstCh->blocks[start] ) + parlSize<edgeDir>( cuPfstCh->blocks[start] ) > parlPos<edgeDir>( posPfst ) ? cuPfstCh : cu.cs->getCU( posPfst, ChannelType( start ) )
                                                    : cuP;
            const TransformUnit &tuP = cuP->firstTU.next == nullptr ? cuP->firstTU : *getTU( *cuP, posP, ch );
            const int sizePSide      = perpSize<edgeDir>( tuP.blocks[ch] );
            LoopFilterParam& lfp     = *lfpPtr;

            lfp.setFilterEdge( cu.chType(), bValue );
            if( ( lfp.bs || bSameCUTUSize ) && bValue )
            {
              lfp.bs |= BsSet( 3, MAX_NUM_COMPONENT );
            }
            else
            {
              lfp.bs |= BsSet( 1, MAX_NUM_COMPONENT );
            }
            if( ch == CHANNEL_TYPE_LUMA )
            {
              uint8_t maxFLPQ;
              
              bool smallBlock = ( sizePSide <= 4 ) || ( sizeQSide <= 4 );
              if( smallBlock )
              {
                maxFLPQ = 17;
              }
              else
              {
                maxFLPQ   = ( sizePSide >= 32 ) ? ( cuP->affineFlag() ? 5 : 7 ) : 3;
                maxFLPQ <<= 4;
                maxFLPQ  += ( sizeQSide >= 32 ) ? 7 : 3;
              }
              
              maxFLPQ    += 128;
              lfp.sideMaxFiltLength = maxFLPQ;
            }
            else
            {
              lfp.setFilterCMFL( ( sizeQSide >= 8 && sizePSide >= 8 ) ? 1 : 0 );
            }
            
            if( bValue )
              xGetBoundaryStrengthSingle<edgeDir>( lfp, cu, Position( ( area.x + edgeDir * d ) << csx, ( area.y + ( 1 - edgeDir ) * d ) << csy ), *cuPfstCh, ctuData, pqSameCtu, useSimd );
            lfp.bs &= ~BsSet( 3, MAX_NUM_COMPONENT );
            OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
          }
        }
      }
      else
      {
        for( int d = 0; d < parlSize<edgeDir>( currTU.blocks[ch] ); )
        {
          const Position  posQ     { currTU.blocks[ch].x + edgeDir * d, currTU.blocks[ch].y + ( 1 - edgeDir ) * d };
          const Position  posP     = posQ.offset( -( 1 - edgeDir ), -edgeDir );
          const int sizeQSide      = perpSize<edgeDir>( currTU.blocks[ch] );
                          cuP      = parlPos<edgeDir>( cuP->blocks[ch] ) + parlSize<edgeDir>( cuP->blocks[ch] ) > parlPos<edgeDir>( posP )? cuP:cu.cs->getCU( posP,                 ch );
          const TransformUnit &tuP = cuP->firstTU.next == nullptr ? cuP->firstTU : *getTU( *cuP, posP, ch );
          const int sizePSide      = perpSize<edgeDir>( tuP.blocks[ch] );

          int distance = parlPos<edgeDir>( tuP.blocks[ch] ) + parlSize<edgeDir>( tuP.blocks[ch] ) - parlPos<edgeDir>(posQ);
          if( distance > parlSize<edgeDir>( currTU.blocks[ch] ) - d )
            distance = parlSize<edgeDir>( currTU.blocks[ch] ) - d; // range check

          if( ch == CHANNEL_TYPE_LUMA )
          {
            LoopFilterParam& lfp = *lfpPtr;
            lfp.setFilterEdge( cu.chType(), bValue );

            if( bValue )
            {
              if( lfp.bs || bSameCUTUSize )
              {
                lfp.bs |= BsSet( 3, MAX_NUM_COMPONENT );
              }
              else
              {
                lfp.bs |= BsSet( 1, MAX_NUM_COMPONENT );
              }

              uint8_t maxFLPQ;
              bool smallBlock = ( sizePSide <= 4 ) || ( sizeQSide <= 4 );
              if( smallBlock )
              {
                maxFLPQ = 17;
              }
              else
              {
                maxFLPQ   = ( sizePSide >= 32 ) ? ( cuP->affineFlag() ? 5 : 7 ) : 3;
                maxFLPQ <<= 4;
                maxFLPQ  += ( sizeQSide >= 32 ) ? 7 : 3;
              }
              maxFLPQ    += 128;
              lfp.sideMaxFiltLength = maxFLPQ;
            }

            if( distance > inc )
            {
              LoopFilterParam tmp = lfp;
              for (int j = inc; j < distance; j += inc, d += inc)
              {
                OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
                *lfpPtr = tmp;
              }
            }
            d += inc;
            OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
          }
          else
          {
            for( int j = 0; j < distance; j += inc, d += inc )
            {
              LoopFilterParam& lfp = *lfpPtr;
              if( ct == start )
              {
                lfp.setFilterEdge( cu.chType(), bValue );

                if( bValue )
                {
                  if( lfp.bs || bSameCUTUSize )
                  {
                    lfp.bs |= BsSet( 3, MAX_NUM_COMPONENT );
                  }
                  else
                  {
                    lfp.bs |= BsSet( 1, MAX_NUM_COMPONENT );
                  }
                }
              }

              lfp.setFilterCMFL( ( sizeQSide >= 8 && sizePSide >= 8 ) ? 1 : 0 );
              OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
            }
          }
        }
      }
    }
  }
}

template<DeblockEdgeDir edgeDir>
void LoopFilter::xSetEdgeFilterInsidePu( const CodingUnit &cu, const Area &area, const bool bValue, CtuData& ctuData ) const
{
  const PreCalcValues &  pcv       = *cu.cs->pcv;
  LoopFilterParam*       lfpPtr    =  ctuData.lfParam[edgeDir] + cu.cs->inCtuPos( area.pos(), cu.chType() );
  ptrdiff_t              lfpStride =  cu.cs->get4x4MapStride();

  const int inc = edgeDir ? pcv.minCUWidth  >> getChannelTypeScaleX( cu.chType(), cu.chromaFormat )
                          : pcv.minCUHeight >> getChannelTypeScaleY( cu.chType(), cu.chromaFormat );

  if (bValue)
  {
    for( int d = 0; d < parlSize<edgeDir>( area ); d += inc )
    {
      lfpPtr->setFilterEdge( cu.chType(), 1 );
       if( lfpPtr->bs)
       {
         lfpPtr->bs |= BsSet( 3, MAX_NUM_COMPONENT );
       }
       OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
    }
  }
  else
  {
    for( int d = 0; d < parlSize<edgeDir>( area ); d += inc )
    {
      lfpPtr->setFilterEdge( cu.chType(), 0 );
      OFFSET( lfpPtr, lfpStride, edgeDir, ( 1 - edgeDir ) );
    }
  }
}

LFCUParam LoopFilter::xGetLoopfilterParam( const CodingUnit& cu ) const
{
  const Position pos = cu.blocks[cu.chType()].pos();

  const PPS& pps = *cu.pps;
  const SPS& sps = *cu.sps;

  LFCUParam stLFCUParam;   ///< status structure
  if( pos.x > 0 )
  {
    const CodingUnit* cuLeft = cu.left ? cu.left : cu.cs->getCU( pos.offset( -1, 0 ), cu.chType() ) ;
    const bool loopFilterAcrossSubPicEnabledFlagLeft = !sps.getSubPicInfoPresentFlag() ||
                                                       ( pps.getSubPicFromCU( cu ).getloopFilterAcrossSubPicEnabledFlag() &&
                                                         pps.getSubPicFromCU( *cuLeft ).getloopFilterAcrossSubPicEnabledFlag() );

    stLFCUParam.leftEdge = CU::isAvailable( cu, *cuLeft, !pps.getLoopFilterAcrossSlicesEnabledFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag(), !loopFilterAcrossSubPicEnabledFlagLeft );
  }

  if( pos.y > 0 )
  {
    const CodingUnit* cuAbove = cu.above ? cu.above : cu.cs->getCU( pos.offset( 0, -1 ), cu.chType() );
    const bool loopFilterAcrossSubPicEnabledFlagTop = !sps.getSubPicInfoPresentFlag() ||
                                                      ( pps.getSubPicFromCU( cu ).getloopFilterAcrossSubPicEnabledFlag() &&
                                                        pps.getSubPicFromCU( *cuAbove ).getloopFilterAcrossSubPicEnabledFlag() );

    stLFCUParam.topEdge = CU::isAvailable( cu, *cuAbove, !pps.getLoopFilterAcrossSlicesEnabledFlag(), !pps.getLoopFilterAcrossTilesEnabledFlag(), !loopFilterAcrossSubPicEnabledFlagTop );
  }
  return stLFCUParam;
}

template<DeblockEdgeDir edgeDir>
void LoopFilter::xGetBoundaryStrengthSingle( LoopFilterParam& lfp, const CodingUnit& cuQ, const Position &localPos, const CodingUnit& cuP, CtuData& ctuData, bool pqSameCtu, bool useSimd ) const
{
  const Slice      &sliceQ = *cuQ.slice;
  const ChannelType chType = cuQ.chType();
  const Position    &cuPos = cuQ.blocks[chType].pos();
  const Position     &posQ = localPos;
  const Position      posP { posQ.x - !edgeDir, posQ.y - edgeDir };

  const TransformUnit &tuQ = cuQ.firstTU.next == nullptr ? cuQ.firstTU : *getTU( cuQ, posQ, chType );
  const TransformUnit &tuP = cuP.firstTU.next == nullptr ? cuP.firstTU : *getTU( cuP, posP, chType ); //TODO: check this: based on chType of the current cu, because cuQ.chType and cuP.chType are not the same when local dual-tree is applied
  
  const bool hasLuma   = cuQ.Y(). valid();
  const bool hasChroma = isChromaEnabled( cuQ.chromaFormat ) && cuQ.Cb().valid();

  bool cuPcIsIntra = false;
  int  chrmBS      = 2;

  if( hasLuma )
  {
    lfp.qp[0] = ( cuQ.qp + cuP.qp + 1 ) >> 1;
  }

  if( hasChroma )
  {
    const int qpBdOffset2     = cuQ.sps->getQpBDOffset() << 1;
    const bool isPQDiffCh     = !chType && cuP.treeType() != TREE_D;
    const TransformUnit &tuQc = cuQ.ispMode() ? *cuQ.lastTU : tuQ;
    const Position      posPc = isPQDiffCh ? recalcPosition( cuQ.chromaFormat, chType, CH_C, posP ) : Position();
    const CodingUnit    &cuPc = isPQDiffCh ? *cuQ.cs->getCU(       posPc, CH_C ) :   cuP;
    const TransformUnit &tuPc = isPQDiffCh ? *        getTU( cuPc, posPc, CH_C ) : ( cuP.ispMode() ? *cuP.lastTU : tuP );

    cuPcIsIntra = CU::isIntra( cuPc );
    
    lfp.qp[1] = ( ( tuPc.chromaQp[0] + tuQc.chromaQp[0] - qpBdOffset2 + 1 ) >> 1 );
    lfp.qp[2] = ( ( tuPc.chromaQp[1] + tuQc.chromaQp[1] - qpBdOffset2 + 1 ) >> 1 );
    
    if( cuPcIsIntra )
    {
      chrmBS  = ( MODE_INTRA == cuPc.predMode() && cuPc.bdpcmModeChroma() ) && ( MODE_INTRA == cuQ.predMode() && cuQ.bdpcmModeChroma() ) ? 0 : 2;
    }
  }

  const int bsMask = ( hasLuma   ? BsSet( 3, COMPONENT_Y ) :  0 ) |
                                   BsSet( 3, MAX_NUM_COMPONENT  ) |
                     ( hasChroma ? BsSet( 3, COMPONENT_Cb ) : 0 ) |
                     ( hasChroma ? BsSet( 3, COMPONENT_Cr ) : 0 );

  //-- Set BS for Intra MB : BS = 4 or 3
  if( MODE_INTRA == cuP.predMode() || MODE_INTRA == cuQ.predMode() )
  {
    const int edgeIdx = ( perpPos<edgeDir>( localPos ) - perpPos<edgeDir>( cuPos ) ) / 4;

    int bsY = cuP.bdpcmMode() && cuQ.bdpcmMode()? 0 : 2;

    if( cuQ.ispMode() && edgeIdx )
    {
      lfp.bs |= BsSet( bsY, COMPONENT_Y ) & bsMask;
    }
    else
    {
      lfp.bs |= ( BsSet( bsY, COMPONENT_Y ) + BsSet( chrmBS, COMPONENT_Cb ) + BsSet( chrmBS, COMPONENT_Cr ) ) & bsMask;
    }

    return;
  }
  else if( cuPcIsIntra )
  {
    lfp.bs |= ( BsSet( chrmBS, COMPONENT_Cb ) + BsSet( chrmBS, COMPONENT_Cr ) );
  }

  if( ( lfp.bs & bsMask ) && ( cuP.ciipFlag() || cuQ.ciipFlag() ) )
  {
    lfp.bs |= ( BsSet( 2, COMPONENT_Y ) + BsSet( 2, COMPONENT_Cb ) + BsSet( 2, COMPONENT_Cr ) ) & bsMask;

    return;
  }

  unsigned tmpBs = 0;
  //-- Set BS for not Intra MB : BS = 2 or 1 or 0
  // Y
  if( lfp.bs & bsMask )
  {
    int cbfSum = tuQ.cbf | tuP.cbf;
    tmpBs += BsSet( cbfSum & 1, COMPONENT_Y );
    if( !( MODE_INTRA != cuP.predMode() && MODE_INTRA != cuQ.predMode() && cuPcIsIntra ) )
    {
      bool jointChr = tuQ.jointCbCr || tuP.jointCbCr;
      cbfSum >>= 1;
      tmpBs += BsSet( (cbfSum & 1) | jointChr , COMPONENT_Cb );
      cbfSum >>= 1;
      tmpBs += BsSet( (cbfSum & 1) | jointChr, COMPONENT_Cr );
    }
  }

  if( BsGet( tmpBs, COMPONENT_Y ) == 1 )
  {
    lfp.bs |= tmpBs & bsMask;

    return;
  }

  if( cuP.ciipFlag() || cuQ.ciipFlag() )
  {
    lfp.bs |= 1 & bsMask;

    return;
  }

  if( !hasLuma )
  {
    lfp.bs |= tmpBs & bsMask;
    return;
  }

  if( BsGet( lfp.bs, MAX_NUM_COMPONENT ) != 0 && BsGet( lfp.bs, MAX_NUM_COMPONENT ) != 3 )
  {
    lfp.bs |= tmpBs & bsMask;
    return;
  }

  if( hasChroma )
  {
    lfp.bs |= tmpBs & bsMask;
  }

  if( cuP.predMode() != cuQ.predMode() && hasLuma )
  {
    lfp.bs |= 1 & bsMask;
    return;
  }

  const ptrdiff_t pqDiff    = edgeDir ? int( cuQ.cs->get4x4MapStride() ) : 1;
  const ptrdiff_t inCtuPosQ = cuQ.cs->inCtuPos( posQ, chType );

  // and now the pred
  const MotionInfo&     miQ    = ctuData.motion[inCtuPosQ];
  const MotionInfo&     miP    = !pqSameCtu ? cuP.getMotionInfo( posP ) : ctuData.motion[inCtuPosQ - pqDiff];
  const Slice&          sliceP = *cuP.slice;

  static constexpr int nThreshold = ( 1 << MV_FRACTIONAL_BITS_INTERNAL ) >> 1;

  if( sliceQ.isInterB() || sliceP.isInterB() )
  {
    const Picture *piRefP0 = CU::isIBC( cuP ) ? sliceP.getPic() : isMotionValid( miP.miRefIdx[0], MI_NOT_VALID ) ? sliceP.getRefPic( REF_PIC_LIST_0, miP.miRefIdx[0] ) : nullptr;
    const Picture *piRefP1 = CU::isIBC( cuP ) ? nullptr         : isMotionValid( miP.miRefIdx[1], MI_NOT_VALID ) ? sliceP.getRefPic( REF_PIC_LIST_1, miP.miRefIdx[1] ) : nullptr;
    const Picture *piRefQ0 = CU::isIBC( cuQ ) ? sliceQ.getPic() : isMotionValid( miQ.miRefIdx[0], MI_NOT_VALID ) ? sliceQ.getRefPic( REF_PIC_LIST_0, miQ.miRefIdx[0] ) : nullptr;
    const Picture *piRefQ1 = CU::isIBC( cuQ ) ? nullptr         : isMotionValid( miQ.miRefIdx[1], MI_NOT_VALID ) ? sliceQ.getRefPic( REF_PIC_LIST_1, miQ.miRefIdx[1] ) : nullptr;

    unsigned uiBs = 0;

    const bool refQ0valid = !!piRefQ0;
    const bool refQ1valid = !!piRefQ1;
    const bool refP0valid = !!piRefP0;
    const bool refP1valid = !!piRefP1;

    //th can be optimized
    if( ( piRefP0 == piRefQ0 && piRefP1 == piRefQ1 ) || ( piRefP0 == piRefQ1 && piRefP1 == piRefQ0 ) )
    {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_DBLF
      if( useSimd )
      {
        const __m128i xmvP = _mm_unpacklo_epi64( refP0valid ? _mm_loadu_si64( ( const __m128i* ) &miP.mv[0] ) : _mm_setzero_si128(), refP1valid ? _mm_loadu_si64( ( const __m128i* ) &miP.mv[1] ) : _mm_setzero_si128() );
        const __m128i xmvQ = _mm_unpacklo_epi64( refQ0valid ? _mm_loadu_si64( ( const __m128i* ) &miQ.mv[0] ) : _mm_setzero_si128(), refQ1valid ? _mm_loadu_si64( ( const __m128i* ) &miQ.mv[1] ) : _mm_setzero_si128() );
        const __m128i xth  = _mm_set1_epi32( nThreshold - 1 );

        if( piRefP0 != piRefP1 )   // Different L0 & L1
        {
          if( piRefP0 == piRefQ0 )
          {
            __m128i
            xdiff = _mm_sub_epi32  ( xmvQ, xmvP );
            xdiff = _mm_abs_epi32  ( xdiff );
            xdiff = _mm_cmpgt_epi32( xdiff, xth );
            uiBs  = _mm_testz_si128( xdiff, xdiff ) ? 0 : 1;
          }
          else
          {
            __m128i
            xmvQ1 = _mm_shuffle_epi32( xmvQ, ( 2 << 0 ) + ( 3 <<  2 ) + ( 0 << 4 ) + ( 1 << 6 ) );
            __m128i
            xdiff = _mm_sub_epi32  ( xmvQ1, xmvP );
            xdiff = _mm_abs_epi32  ( xdiff );
            xdiff = _mm_cmpgt_epi32( xdiff, xth );
            uiBs  = _mm_testz_si128( xdiff, xdiff ) ? 0 : 1;
          }
        }
        else
        {
          __m128i
          xmvQ1 = _mm_shuffle_epi32( xmvQ, ( 2 << 0 ) + ( 3 << 2 ) + ( 0 << 4 ) + ( 1 << 6 ) );
          __m128i
          xdiff = _mm_sub_epi32( xmvQ1, xmvP );
          xdiff = _mm_abs_epi32( xdiff );
          xdiff = _mm_cmpgt_epi32( xdiff, xth );
          uiBs  = _mm_testz_si128( xdiff, xdiff ) ? 0 : 1;

          xdiff = _mm_sub_epi32( xmvQ, xmvP );
          xdiff = _mm_abs_epi32( xdiff );
          xdiff = _mm_cmpgt_epi32( xdiff, xth );
          uiBs &= _mm_testz_si128( xdiff, xdiff ) ? 0 : 1;
        }
      }
      else
#endif
      {
        Mv mvP[2] = { { 0, 0 }, { 0, 0 } }, mvQ[2] = { { 0, 0 }, { 0, 0 } };

        if( refP0valid ) { mvP[0] = miP.mv[0]; }
        if( refP1valid ) { mvP[1] = miP.mv[1]; }
        if( refQ0valid ) { mvQ[0] = miQ.mv[0]; }
        if( refQ1valid ) { mvQ[1] = miQ.mv[1]; }

        if( piRefP0 != piRefP1 )   // Different L0 & L1
        {
          if( piRefP0 == piRefQ0 )
          {
            uiBs = ( ( abs( mvQ[0].getHor() - mvP[0].getHor() ) >= nThreshold ) || ( abs( mvQ[0].getVer() - mvP[0].getVer() ) >= nThreshold ) ||
                     ( abs( mvQ[1].getHor() - mvP[1].getHor() ) >= nThreshold ) || ( abs( mvQ[1].getVer() - mvP[1].getVer() ) >= nThreshold ) )
                 ? 1 : 0;
          }
          else
          {
            uiBs = ( ( abs( mvQ[1].getHor() - mvP[0].getHor() ) >= nThreshold ) || ( abs( mvQ[1].getVer() - mvP[0].getVer() ) >= nThreshold ) ||
                     ( abs( mvQ[0].getHor() - mvP[1].getHor() ) >= nThreshold ) || ( abs( mvQ[0].getVer() - mvP[1].getVer() ) >= nThreshold ) )
                 ? 1 : 0;
          }
        }
        else
        {
          uiBs = ( ( abs( mvQ[0].getHor() - mvP[0].getHor() ) >= nThreshold ) || ( abs( mvQ[0].getVer() - mvP[0].getVer() ) >= nThreshold ) ||
                   ( abs( mvQ[1].getHor() - mvP[1].getHor() ) >= nThreshold ) || ( abs( mvQ[1].getVer() - mvP[1].getVer() ) >= nThreshold ) )
                  &&
                 ( ( abs( mvQ[1].getHor() - mvP[0].getHor() ) >= nThreshold ) || ( abs( mvQ[1].getVer() - mvP[0].getVer() ) >= nThreshold ) ||
                   ( abs( mvQ[0].getHor() - mvP[1].getHor() ) >= nThreshold ) || ( abs( mvQ[0].getVer() - mvP[1].getVer() ) >= nThreshold ) )
               ? 1 : 0;
        }
      }
    }
    else // for all different Ref_Idx
    {
      uiBs = 1;
    }

    lfp.bs |= ( uiBs + tmpBs ) & bsMask;

    return;
  }

  // pcSlice->isInterP()
  CHECK( CU::isInter( cuP ) && isMotionInvalid( miP.miRefIdx[0], MI_NOT_VALID ), "Invalid reference picture list index" );
  CHECK( CU::isInter( cuP ) && isMotionInvalid( miQ.miRefIdx[0], MI_NOT_VALID ), "Invalid reference picture list index" );

  const Picture *piRefP0 = ( CU::isIBC( cuP ) ? sliceP.getPic() : sliceP.getRefPic( REF_PIC_LIST_0, miP.miRefIdx[0] ) );
  const Picture *piRefQ0 = ( CU::isIBC( cuQ ) ? sliceQ.getPic() : sliceQ.getRefPic( REF_PIC_LIST_0, miQ.miRefIdx[0] ) );

  if( piRefP0 != piRefQ0 )
  {
    lfp.bs |= ( tmpBs + 1 ) & bsMask;

    return;
  }

  Mv mvP0 = miP.mv[0];
  Mv mvQ0 = miQ.mv[0];

  lfp.bs |= ( ( ( abs( mvQ0.getHor() - mvP0.getHor() ) >= nThreshold ) || ( abs( mvQ0.getVer() - mvP0.getVer() ) >= nThreshold ) ) ? ( tmpBs + 1 ) : tmpBs ) & bsMask;
}

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
void LoopFilter::deriveLADFShift( const Pel* src, const ptrdiff_t stride, int& shift, const DeblockEdgeDir edgeDir, const SPS& sps ) const
{
  int32_t lumaLevel = 0;
  shift = sps.getLadfQpOffset(0);

  if (edgeDir == EDGE_VER)
  {
    lumaLevel = (src[0] + src[3*stride] + src[-1] + src[3*stride - 1]) >> 2;
  }
  else // (edgeDir == EDGE_HOR)
  {
    lumaLevel = (src[0] + src[3] + src[-stride] + src[-stride + 3]) >> 2;
  }

  for ( int k = 1; k < sps.getLadfNumIntervals(); k++ )
  {
    const int th = sps.getLadfIntervalLowerBound( k );
    if ( lumaLevel > th )
    {
      shift = sps.getLadfQpOffset( k );
    }
    else
    {
      break;
    }
  }
}
#endif

template<bool isChromaHorCTBBoundary = false>
static inline int xCalcDP( const Pel* piSrc, const ptrdiff_t iOffset )
{
  if( isChromaHorCTBBoundary )
  {
    return abs( piSrc[-iOffset * 2] - 2 * piSrc[-iOffset * 2] + piSrc[-iOffset] );
  }
  else
  {
    return abs( piSrc[-iOffset * 3] - 2 * piSrc[-iOffset * 2] + piSrc[-iOffset] );
  }
}

static inline int xCalcDQ( const Pel* piSrc, const ptrdiff_t iOffset )
{
  return abs( piSrc[0] - 2 * piSrc[iOffset] + piSrc[iOffset * 2] );
}

static inline bool xUseStrongFiltering( const Pel* piSrc, const ptrdiff_t iOffset, const int d, const int beta, const int tc, bool sidePisLarge = false, bool sideQisLarge = false, int maxFilterLengthP = 7, int maxFilterLengthQ = 7, bool isChromaHorCTBBoundary = false )
{
  const Pel m3 = piSrc[-1 * iOffset];
  const Pel m4 = piSrc[ 0          ];

  if( !( d < ( beta >> 2 ) && abs( m3 - m4 ) < ( ( tc * 5 + 1 ) >> 1 ) ) ) return false;

  const Pel m0 = piSrc[-4 * iOffset];
  const Pel m7 = piSrc[ 3 * iOffset];

  const Pel m2 = piSrc[-iOffset * 2];
  int       sp3      = abs( m0 - m3 );
  if (isChromaHorCTBBoundary)
  {
    sp3 = abs( m2 - m3 );
  }
  int       sq3      = abs( m7 - m4 );
  const int d_strong = sp3 + sq3;

  if( sidePisLarge || sideQisLarge )
  {
    if( sidePisLarge )
    {
      const Pel mP4 = piSrc[-iOffset * maxFilterLengthP - iOffset];
      if( maxFilterLengthP == 7 )
      {
        const Pel mP5 = piSrc[-iOffset * 5];
        const Pel mP6 = piSrc[-iOffset * 6];
        const Pel mP7 = piSrc[-iOffset * 7];;
        sp3 = sp3 + abs(mP5 - mP6 - mP7 + mP4);
      }
      sp3 = ( sp3 + abs( m0 - mP4 ) + 1 ) >> 1;
    }
    if( sideQisLarge )
    {
      const Pel m11 = piSrc[ iOffset * maxFilterLengthQ];
      if (maxFilterLengthQ == 7)
      {
        const Pel m8 = piSrc[iOffset * 4];
        const Pel m9 = piSrc[iOffset * 5];
        const Pel m10 = piSrc[iOffset * 6];;
        sq3 = sq3 + abs(m8 - m9 - m10 + m11);
      }
      sq3 = ( sq3 + abs( m11 - m7 ) + 1 ) >> 1;
    }
    return ((sp3 + sq3) < (beta*3 >> 5)) && (d < (beta >> 4)) && (abs(m3 - m4) < ((tc * 5 + 1) >> 1));
  }
  else
  {
    return d_strong < ( beta >> 3 );
  }
}

template<DeblockEdgeDir edgeDir>
void LoopFilter::xEdgeFilterLuma( CodingStructure& cs, const Position& pos, const LoopFilterParam& lfp, const Slice &slice ) const
{
  PelBuf          picYuvRec    = cs.getRecoBuf( COMPONENT_Y );
  Pel *           piSrc        = picYuvRec.bufAt( pos );
  const ptrdiff_t iStride      = picYuvRec.stride;
  
  const int       bitDepthLuma = slice.getSPS()->getBitDepth();
  const ClpRng&         clpRng = slice.clpRng( COMPONENT_Y );

  const int  betaOffsetDiv2    = slice.getDeblockingFilterBetaOffsetDiv2();
  const int  tcOffsetDiv2      = slice.getDeblockingFilterTcOffsetDiv2();

  ptrdiff_t offset, srcStep;

  if( edgeDir == EDGE_VER )
  {
    offset   = 1;
    srcStep  = iStride;
  }
  else  // (edgeDir == EDGE_HOR)
  {
    offset   = iStride;
    srcStep  = 1;
  }

#if ENABLE_SIMD_DBLF && defined( TARGET_SIMD_X86 )
  if( edgeDir == EDGE_VER )
  {
    _mm_prefetch( (char *) &piSrc[0 * iStride - 4], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[1 * iStride - 4], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[2 * iStride - 4], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[3 * iStride - 4], _MM_HINT_T0 );
  }
  else
  {
    _mm_prefetch( (char *) &piSrc[( 0 - 4 ) * iStride], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 1 - 4 ) * iStride], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 2 - 4 ) * iStride], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 3 - 4 ) * iStride], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 4 - 4 ) * iStride], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 5 - 4 ) * iStride], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 6 - 4 ) * iStride], _MM_HINT_T0 );
    _mm_prefetch( (char *) &piSrc[( 7 - 4 ) * iStride], _MM_HINT_T0 );
  }
#endif // ENABLE_SIMD_OPT

  const unsigned uiBs = BsGet(lfp.bs, COMPONENT_Y);

  if( !uiBs )
  {
    return;
  }

  int iQP = lfp.qp[0];
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  if ( slice.getSPS()->getLadfEnabled() )
  {
    int iShift = 0;
    deriveLADFShift( piSrc, iStride, iShift, edgeDir, *slice.getSPS() );
    iQP += iShift;
  }

#endif
  const int maxFilterLengthP = ( lfp.sideMaxFiltLength >> 4 ) &   7;
  const int maxFilterLengthQ =   lfp.sideMaxFiltLength        &   7;

  bool sidePisLarge = maxFilterLengthP > 3;
  bool sideQisLarge = maxFilterLengthQ > 3;

  if( edgeDir == EDGE_HOR && ( pos.y & cs.pcv->maxCUHeightMask ) == 0 )
  {
    sidePisLarge = false;
  }

  const int iIndexTC  = Clip3( 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, int( iQP + DEFAULT_INTRA_TC_OFFSET * ( uiBs - 1 ) + ( tcOffsetDiv2 *2 ) ) );
  const int iIndexB   = Clip3( 0, MAX_QP, iQP + ( betaOffsetDiv2 *2 ) );

  const int iTc = bitDepthLuma < 10 ? ((sm_tcTable[iIndexTC] + (1 << (9 - bitDepthLuma))) >> (10 - bitDepthLuma)) : ((sm_tcTable[iIndexTC]) << (bitDepthLuma - 10));
  const int iBeta     = sm_betaTable[iIndexB ] << ( bitDepthLuma - 8 );
  const int iSideThreshold = ( iBeta + ( iBeta >> 1 ) ) >> 3;
  const int iThrCut   = iTc * 10;

  static constexpr bool bPartPNoFilter = false;
  static constexpr bool bPartQNoFilter = false;

  if( !bPartPNoFilter || !bPartQNoFilter )
  {
    const Pel* piSrc0 = piSrc;
    const Pel* piSrc3 = piSrc + 3 * srcStep;

    const int dp0 = xCalcDP( piSrc0, offset );
    const int dq0 = xCalcDQ( piSrc0, offset );
    const int dp3 = xCalcDP( piSrc3, offset );
    const int dq3 = xCalcDQ( piSrc3, offset );
    const int d0 = dp0 + dq0;
    const int d3 = dp3 + dq3;

    if( sidePisLarge || sideQisLarge )
    {
      const ptrdiff_t off3 = 3 * offset;
      const int dp0L = sidePisLarge ? ( ( dp0 + xCalcDP( piSrc0 - off3, offset ) + 1 ) >> 1 ) : dp0;
      const int dq0L = sideQisLarge ? ( ( dq0 + xCalcDQ( piSrc0 + off3, offset ) + 1 ) >> 1 ) : dq0;
      const int dp3L = sidePisLarge ? ( ( dp3 + xCalcDP( piSrc3 - off3, offset ) + 1 ) >> 1 ) : dp3;
      const int dq3L = sideQisLarge ? ( ( dq3 + xCalcDQ( piSrc3 + off3, offset ) + 1 ) >> 1 ) : dq3;

      const int d0L = dp0L + dq0L;
      const int d3L = dp3L + dq3L;

      const int dL = d0L + d3L;

      if( dL < iBeta )
      {
        // adjust decision so that it is not read beyond p5 is maxFilterLengthP is 5 and q5 if maxFilterLengthQ is 5
        const bool swL = xUseStrongFiltering( piSrc0, offset, 2 * d0L, iBeta, iTc, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ )
                      && xUseStrongFiltering( piSrc3, offset, 2 * d3L, iBeta, iTc, sidePisLarge, sideQisLarge, maxFilterLengthP, maxFilterLengthQ );
        if( swL )
        {
          xFilteringPandQ( piSrc, srcStep, offset, sidePisLarge ? maxFilterLengthP : 3, sideQisLarge ? maxFilterLengthQ : 3, iTc );

          return;
        }
      }
    }

    // if( dL >= iBet || !swL )
    {
      const int dp = dp0 + dp3;
      const int dq = dq0 + dq3;
      const int d  = d0  + d3;

      if( d < iBeta )
      {
        bool bFilterP = false;
        bool bFilterQ = false;

        if( maxFilterLengthP > 1 && maxFilterLengthQ > 1 )
        {
          bFilterP = ( dp < iSideThreshold );
          bFilterQ = ( dq < iSideThreshold );
        }

        bool sw = false;

        if( maxFilterLengthP > 2 && maxFilterLengthQ > 2 )
        {
          sw = xUseStrongFiltering( piSrc0, offset, 2 * d0, iBeta, iTc )
            && xUseStrongFiltering( piSrc3, offset, 2 * d3, iBeta, iTc );
        }

        xPelFilterLuma( piSrc, srcStep, offset, iTc, sw, iThrCut, bFilterP, bFilterQ, clpRng );
      }
    }
  }
}

template<DeblockEdgeDir edgeDir>
void LoopFilter::xEdgeFilterChroma( CodingStructure &cs, const Position &pos, const LoopFilterParam& lfp, const Slice& slice ) const
{
  const PreCalcValues &pcv               = *cs.pcv;

  const ChromaFormat nChromaFormat       = pcv.chrFormat;
  const int          csy                 = getChannelTypeScaleY( CH_C, nChromaFormat );
  const unsigned     uiPelsInPartChromaH = pcv.minCUWidth  >> getChannelTypeScaleX( CH_C, nChromaFormat );
  const unsigned     uiPelsInPartChromaV = pcv.minCUHeight >> csy;

  PelBuf             picYuvRecCb         = cs.getRecoBuf( COMPONENT_Cb );
  PelBuf             picYuvRecCr         = cs.getRecoBuf( COMPONENT_Cr );
  Pel *              piSrcCb             = picYuvRecCb.bufAt( pos );
  Pel *              piSrcCr             = picYuvRecCr.bufAt( pos );
  const ptrdiff_t    iStride             = picYuvRecCb.stride;
  const SPS &        sps                 = *cs.sps;
  const int          bitDepthChroma      = sps.getBitDepth();

  const int tcOffsetDiv2[2]              = { slice.getDeblockingFilterCbTcOffsetDiv2(),   slice.getDeblockingFilterCrTcOffsetDiv2() };
  const int betaOffsetDiv2[2]            = { slice.getDeblockingFilterCbBetaOffsetDiv2(), slice.getDeblockingFilterCrBetaOffsetDiv2() };

  ptrdiff_t offset, srcStep;
  unsigned  uiLoopLength;

  if( edgeDir == EDGE_VER )
  {
    offset       = 1;
    srcStep      = iStride;
    uiLoopLength = uiPelsInPartChromaV;
  }
  else
  {
    offset       = iStride;
    srcStep      = 1;
    uiLoopLength = uiPelsInPartChromaH;
  }

  unsigned bS[2];

  unsigned tmpBs = lfp.bs;
  bS[0] = BsGet( tmpBs, COMPONENT_Cb );
  bS[1] = BsGet( tmpBs, COMPONENT_Cr );

  if( bS[0] <= 0 && bS[1] <= 0 )
  {
    return;
  }

  bool largeBoundary = lfp.filterCMFL();
  bool isChromaHorCTBBoundary = false;

  if( edgeDir == EDGE_HOR && ( pos.y & ( pcv.maxCUHeightMask >> csy ) ) == 0 )
  {
    isChromaHorCTBBoundary = true;
  }

  static constexpr bool bPartPNoFilter = false;
  static constexpr bool bPartQNoFilter = false;

  if( !bPartPNoFilter || !bPartQNoFilter )
  for( unsigned chromaIdx = 0; chromaIdx < 2; chromaIdx++ )
  {
    if( bS[chromaIdx] == 2 || ( largeBoundary && bS[chromaIdx] == 1 ) )
    {
      const ClpRng& clpRng( cs.picture->slices[0]->clpRng( ComponentID( chromaIdx + 1 ) ) );

      int iQP = lfp.qp[chromaIdx + 1];

      const int iIndexTC = Clip3<int>( 0, MAX_QP + DEFAULT_INTRA_TC_OFFSET, iQP + DEFAULT_INTRA_TC_OFFSET * ( bS[chromaIdx] - 1 ) + ( tcOffsetDiv2[chromaIdx] *2 ) );
      const int iTc = bitDepthChroma < 10 ? ((sm_tcTable[iIndexTC] + (1 << (9 - bitDepthChroma))) >> (10 - bitDepthChroma)) : ((sm_tcTable[iIndexTC]) << (bitDepthChroma - 10));
      Pel* piSrcChroma   = chromaIdx == 0 ? piSrcCb : piSrcCr;

      if( largeBoundary )
      {
        const int iBitdepthScale = 1 << ( bitDepthChroma - 8 );

        const int indexB = Clip3<int>( 0, MAX_QP, iQP + ( betaOffsetDiv2[chromaIdx] *2 ) );
        const int beta   = sm_betaTable[indexB] * iBitdepthScale;

        const int dp0 = isChromaHorCTBBoundary ? xCalcDP<true>( piSrcChroma, offset ) : xCalcDP( piSrcChroma, offset );
        const int dq0 = xCalcDQ( piSrcChroma, offset );
        const int subSamplingShift = ( edgeDir == EDGE_VER ) ? getChannelTypeScaleY( CH_C, nChromaFormat ) : getChannelTypeScaleX( CH_C, nChromaFormat );
        const int dp3 = isChromaHorCTBBoundary
                        ? ( ( subSamplingShift == 1 ) ? xCalcDP<true>(piSrcChroma + srcStep, offset) : xCalcDP<true>(piSrcChroma + srcStep*3, offset ) )
                        : ( ( subSamplingShift == 1 ) ? xCalcDP      (piSrcChroma + srcStep, offset) : xCalcDP      (piSrcChroma + srcStep*3, offset ) );
        const int dq3 = ( subSamplingShift == 1 ) ? xCalcDQ(piSrcChroma + srcStep, offset) : xCalcDQ(piSrcChroma + srcStep*3, offset);

        const int d0 = dp0 + dq0;
        const int d3 = dp3 + dq3;
        const int d  = d0  + d3;

        if( d < beta )
        {
          const bool sw = xUseStrongFiltering( piSrcChroma, offset, 2 * d0, beta, iTc, false, false, 7, 7, isChromaHorCTBBoundary )
                       && xUseStrongFiltering( piSrcChroma + ( ( subSamplingShift == 1 ) ? srcStep : srcStep*3 ), offset, 2 * d3, beta, iTc, false, false, 7, 7, isChromaHorCTBBoundary );

          for( unsigned uiStep = 0; uiStep < uiLoopLength; uiStep++ )
          {
            xPelFilterChroma( piSrcChroma + srcStep * uiStep, offset, iTc, sw, clpRng, largeBoundary, isChromaHorCTBBoundary );
          }

          continue;
        }
      }

      for( unsigned uiStep = 0; uiStep < uiLoopLength; uiStep++ )
      {
        xPelFilterChroma( piSrcChroma + srcStep * uiStep, offset, iTc, false, clpRng, largeBoundary, isChromaHorCTBBoundary );
      }
    }
  }
}

}
