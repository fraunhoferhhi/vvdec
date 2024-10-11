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

/** \file     AdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "AdaptiveLoopFilter.h"

#include "CodingStructure.h"
#include "Picture.h"
#include "UnitTools.h"
#include "CommonLib/TimeProfiler.h"
#include <array>
#include <cmath>
#include <mutex>

namespace vvdec
{

constexpr int AdaptiveLoopFilter::AlfNumClippingValues[];

AdaptiveLoopFilter::AdaptiveLoopFilter()
{
  m_deriveClassificationBlk = deriveClassificationBlk;
  m_filterCcAlf             = filterBlkCcAlf;
  m_filterCcAlfBoth         = filterBlkCcAlfBoth;
  m_filter5x5Blk            = filterBlk<ALF_FILTER_5>;
  m_filter7x7Blk            = filterBlk<ALF_FILTER_7>;

#if ENABLE_SIMD_OPT_ALF
# ifdef TARGET_SIMD_X86
  initAdaptiveLoopFilterX86();
# endif
#endif

  for( int filterSetIndex = 0; filterSetIndex < NUM_FIXED_FILTER_SETS; filterSetIndex++ )
  {
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
    {
      const int fixedFilterIdx = m_classToFilterMapping[filterSetIndex][classIdx];
      for( int i = 0; i < MAX_NUM_ALF_LUMA_COEFF - 1; i++ )
      {
        m_fixedFilterSetCoeffDec[filterSetIndex][classIdx * MAX_NUM_ALF_LUMA_COEFF + i] = m_fixedFilterSetCoeff[fixedFilterIdx][i];
      }
      m_fixedFilterSetCoeffDec[filterSetIndex][classIdx * MAX_NUM_ALF_LUMA_COEFF + MAX_NUM_ALF_LUMA_COEFF - 1] = ( 1 << ( m_NUM_BITS - 1 ) );

      for( int tranposeIdx = 1; tranposeIdx < 4; tranposeIdx++ )
      {
        short* coef = m_fixedFilterSetCoeffDec[filterSetIndex] + classIdx * MAX_NUM_ALF_LUMA_COEFF;
        short* txcf = m_fixedFilterSetCoeffDec[filterSetIndex] + classIdx * MAX_NUM_ALF_LUMA_COEFF + tranposeIdx * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF;

        if( tranposeIdx == 1 )
        {
          const short tcff[] = { coef[9], coef[4], coef[10], coef[8], coef[1], coef[5], coef[11], coef[7], coef[3], coef[0], coef[2], coef[6], coef[12] };
          memcpy( txcf, tcff, sizeof( tcff ) );
        }
        else if( tranposeIdx == 2 )
        {
          const short tcff[] = { coef[0], coef[3], coef[2], coef[1], coef[8], coef[7], coef[6], coef[5], coef[4], coef[9], coef[10], coef[11], coef[12] };
          memcpy( txcf, tcff, sizeof( tcff ) );
        }
        else if( tranposeIdx == 3 )
        {
          const short tcff[] = { coef[9], coef[8], coef[10], coef[4], coef[3], coef[7], coef[11], coef[5], coef[1], coef[0], coef[2], coef[6], coef[12] };
          memcpy( txcf, tcff, sizeof( tcff ) );
        }
      }
    }
  }
}

bool AdaptiveLoopFilter::isClipOrCrossedByVirtualBoundaries( const CodingStructure& cs,
                                                             const Area&            area,
                                                             bool&                  clipTop,
                                                             bool&                  clipBottom,
                                                             bool&                  clipLeft,
                                                             bool&                  clipRight,
                                                             int&                   numHorVirBndry,
                                                             int&                   numVerVirBndry,
                                                             int                    horVirBndryPos[],
                                                             int                    verVirBndryPos[],
                                                             int&                   rasterSliceAlfPad
)
{
  clipTop        = false;
  clipBottom     = false;
  clipLeft       = false;
  clipRight      = false;
  numHorVirBndry = 0;
  numVerVirBndry = 0;

  const PPS*       pps       = cs.pps.get();
  const SPS*       sps       = cs.sps.get();
  const PicHeader* picHeader = cs.picHeader.get();

  if( picHeader->getVirtualBoundariesPresentFlag() )
  {
    for( int i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++ )
    {
      if( picHeader->getVirtualBoundariesPosY( i ) == area.y )
      {
        clipTop = true;
      }
      else if( picHeader->getVirtualBoundariesPosY( i ) == area.y + area.height )
      {
        clipBottom = true;
      }
      else if( area.y < picHeader->getVirtualBoundariesPosY( i ) && picHeader->getVirtualBoundariesPosY( i ) < area.y + area.height )
      {
        horVirBndryPos[numHorVirBndry++] = picHeader->getVirtualBoundariesPosY( i );
      }
    }
    for( int i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++ )
    {
      if( picHeader->getVirtualBoundariesPosX( i ) == area.x )
      {
        clipLeft = true;
      }
      else if( picHeader->getVirtualBoundariesPosX( i ) == area.x + area.width )
      {
        clipRight = true;
      }
      else if( area.x < picHeader->getVirtualBoundariesPosX( i ) && picHeader->getVirtualBoundariesPosX( i ) < area.x + area.width )
      {
        verVirBndryPos[numVerVirBndry++] = picHeader->getVirtualBoundariesPosX( i );
      }
    }
  }

  int               ctuSize = sps->getCTUSize();
  const Position    currCtuPos( area.x, area.y );
  const CodingUnit* currCtu = cs.getCU( currCtuPos, CHANNEL_TYPE_LUMA );
  bool loopFilterAcrossSubPicEnabledFlag = true;
  bool loopFilterAcrossTilesEnabledFlag = true;
  bool loopFilterAcrossSlicesEnabledFlag = true;

  if( sps->getSubPicInfoPresentFlag() )
  {
    loopFilterAcrossSubPicEnabledFlag = pps->getSubPicFromPos( currCtuPos ).getloopFilterAcrossSubPicEnabledFlag();
  }

  if( pps->getNumTiles() > 1 )
  {
    loopFilterAcrossTilesEnabledFlag = pps->getLoopFilterAcrossTilesEnabledFlag();
  }

  if( currCtu->slice->getNumCtuInSlice() != cs.pcv->sizeInCtus )
  {
    loopFilterAcrossSlicesEnabledFlag = pps->getLoopFilterAcrossSlicesEnabledFlag();
  }

  bool restrictAny    = !loopFilterAcrossSlicesEnabledFlag || !loopFilterAcrossTilesEnabledFlag || !loopFilterAcrossSubPicEnabledFlag;
  bool restrictSlices = !loopFilterAcrossSlicesEnabledFlag;

  // top
  if( area.y >= ctuSize && clipTop == false && restrictAny )
  {
    const Position    prevCtuPos( area.x, area.y - ctuSize );
    const CodingUnit* prevCtu = cs.getCU( prevCtuPos, CHANNEL_TYPE_LUMA );
    if( !CU::isAvailable( *currCtu,
                          *prevCtu,
                          !loopFilterAcrossSlicesEnabledFlag,
                          !loopFilterAcrossTilesEnabledFlag,
                          !loopFilterAcrossSubPicEnabledFlag ) )
    {
      clipTop = true;
    }
  }

  // bottom
  if( area.y + ctuSize < cs.pcv->lumaHeight && clipBottom == false && restrictAny )
  {
    const Position    nextCtuPos( area.x, area.y + ctuSize );
    const CodingUnit* nextCtu = cs.getCU( nextCtuPos, CHANNEL_TYPE_LUMA );
    if( !CU::isAvailable( *currCtu,
                          *nextCtu,
                          !loopFilterAcrossSlicesEnabledFlag,
                          !loopFilterAcrossTilesEnabledFlag,
                          !loopFilterAcrossSubPicEnabledFlag ) )
    {
      clipBottom = true;
    }
  }

  // left
  if( area.x >= ctuSize && clipLeft == false && restrictAny )
  {
    const Position    prevCtuPos( area.x - ctuSize, area.y );
    const CodingUnit* prevCtu = cs.getCU( prevCtuPos, CHANNEL_TYPE_LUMA );
    if( !CU::isAvailable( *currCtu,
                          *prevCtu,
                          !loopFilterAcrossSlicesEnabledFlag,
                          !loopFilterAcrossTilesEnabledFlag,
                          !loopFilterAcrossSubPicEnabledFlag ) )
    {
      clipLeft = true;
    }
  }

  // right
  if( area.x + ctuSize < cs.pcv->lumaWidth && clipRight == false && restrictAny )
  {
    const Position    nextCtuPos( area.x + ctuSize, area.y );
    const CodingUnit* nextCtu = cs.getCU( nextCtuPos, CHANNEL_TYPE_LUMA );

    if( !CU::isAvailable( *currCtu,
                          *nextCtu,
                          !loopFilterAcrossSlicesEnabledFlag,
                          !loopFilterAcrossTilesEnabledFlag,
                          !loopFilterAcrossSubPicEnabledFlag ) )
    {
      clipRight = true;
    }
  }

  rasterSliceAlfPad = 0;
  if( !clipTop && !clipLeft && restrictSlices )
  {
    //top-left CTU
    if ( area.x >= ctuSize && area.y >= ctuSize )
    {
      const Position prevCtuPos( area.x - ctuSize, area.y - ctuSize );
      const CodingUnit *prevCtu = cs.getCU( prevCtuPos, CHANNEL_TYPE_LUMA );
      if ( !loopFilterAcrossSlicesEnabledFlag && !CU::isSameSlice( *currCtu, *prevCtu ) )
      {
        rasterSliceAlfPad = 1;
      }
    }
  }

  if( !clipBottom && !clipRight && restrictSlices )
  {
    //bottom-right CTU
    if ( area.x + ctuSize < cs.pcv->lumaWidth && area.y + ctuSize < cs.pcv->lumaHeight )
    {
      const Position nextCtuPos( area.x + ctuSize, area.y + ctuSize );
      const CodingUnit *nextCtu = cs.getCU( nextCtuPos, CHANNEL_TYPE_LUMA );
      if ( !loopFilterAcrossSlicesEnabledFlag && !CU::isSameSlice( *currCtu, *nextCtu ) )
      {
        rasterSliceAlfPad += 2;
      }
    }
  }
  
  return numHorVirBndry > 0 || numVerVirBndry > 0 || clipTop || clipBottom || clipLeft || clipRight || rasterSliceAlfPad;
}

const int AdaptiveLoopFilter::m_fixedFilterSetCoeff[ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF] =
{
  {  0,   0,   2,  -3,   1,  -4,   1,   7,  -1,   1,  -1,   5,  0 },
  {  0,   0,   0,   0,   0,  -1,   0,   1,   0,   0,  -1,   2,  0 },
  {  0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0,  0 },
  {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  -1,   1,  0 },
  {  2,   2,  -7,  -3,   0,  -5,  13,  22,  12,  -3,  -3,  17,  0 },
  { -1,   0,   6,  -8,   1,  -5,   1,  23,   0,   2,  -5,  10,  0 },
  {  0,   0,  -1,  -1,   0,  -1,   2,   1,   0,   0,  -1,   4,  0 },
  {  0,   0,   3, -11,   1,   0,  -1,  35,   5,   2,  -9,   9,  0 },
  {  0,   0,   8,  -8,  -2,  -7,   4,   4,   2,   1,  -1,  25,  0 },
  {  0,   0,   1,  -1,   0,  -3,   1,   3,  -1,   1,  -1,   3,  0 },
  {  0,   0,   3,  -3,   0,  -6,   5,  -1,   2,   1,  -4,  21,  0 },
  { -7,   1,   5,   4,  -3,   5,  11,  13,  12,  -8,  11,  12,  0 },
  { -5,  -3,   6,  -2,  -3,   8,  14,  15,   2,  -7,  11,  16,  0 },
  {  2,  -1,  -6,  -5,  -2,  -2,  20,  14,  -4,   0,  -3,  25,  0 },
  {  3,   1,  -8,  -4,   0,  -8,  22,   5,  -3,   2, -10,  29,  0 },
  {  2,   1,  -7,  -1,   2, -11,  23,  -5,   0,   2, -10,  29,  0 },
  { -6,  -3,   8,   9,  -4,   8,   9,   7,  14,  -2,   8,   9,  0 },
  {  2,   1,  -4,  -7,   0,  -8,  17,  22,   1,  -1,  -4,  23,  0 },
  {  3,   0,  -5,  -7,   0,  -7,  15,  18,  -5,   0,  -5,  27,  0 },
  {  2,   0,   0,  -7,   1, -10,  13,  13,  -4,   2,  -7,  24,  0 },
  {  3,   3, -13,   4,  -2,  -5,   9,  21,  25,  -2,  -3,  12,  0 },
  { -5,  -2,   7,  -3,  -7,   9,   8,   9,  16,  -2,  15,  12,  0 },
  {  0,  -1,   0,  -7,  -5,   4,  11,  11,   8,  -6,  12,  21,  0 },
  {  3,  -2,  -3,  -8,  -4,  -1,  16,  15,  -2,  -3,   3,  26,  0 },
  {  2,   1,  -5,  -4,  -1,  -8,  16,   4,  -2,   1,  -7,  33,  0 },
  {  2,   1,  -4,  -2,   1, -10,  17,  -2,   0,   2, -11,  33,  0 },
  {  1,  -2,   7, -15, -16,  10,   8,   8,  20,  11,  14,  11,  0 },
  {  2,   2,   3, -13, -13,   4,   8,  12,   2,  -3,  16,  24,  0 },
  {  1,   4,   0,  -7,  -8,  -4,   9,   9,  -2,  -2,   8,  29,  0 },
  {  1,   1,   2,  -4,  -1,  -6,   6,   3,  -1,  -1,  -3,  30,  0 },
  { -7,   3,   2,  10,  -2,   3,   7,  11,  19,  -7,   8,  10,  0 },
  {  0,  -2,  -5,  -3,  -2,   4,  20,  15,  -1,  -3,  -1,  22,  0 },
  {  3,  -1,  -8,  -4,  -1,  -4,  22,   8,  -4,   2,  -8,  28,  0 },
  {  0,   3, -14,   3,   0,   1,  19,  17,   8,  -3,  -7,  20,  0 },
  {  0,   2,  -1,  -8,   3,  -6,   5,  21,   1,   1,  -9,  13,  0 },
  { -4,  -2,   8,  20,  -2,   2,   3,   5,  21,   4,   6,   1,  0 },
  {  2,  -2,  -3,  -9,  -4,   2,  14,  16,   3,  -6,   8,  24,  0 },
  {  2,   1,   5, -16,  -7,   2,   3,  11,  15,  -3,  11,  22,  0 },
  {  1,   2,   3, -11,  -2,  -5,   4,   8,   9,  -3,  -2,  26,  0 },
  {  0,  -1,  10,  -9,  -1,  -8,   2,   3,   4,   0,   0,  29,  0 },
  {  1,   2,   0,  -5,   1,  -9,   9,   3,   0,   1,  -7,  20,  0 },
  { -2,   8,  -6,  -4,   3,  -9,  -8,  45,  14,   2, -13,   7,  0 },
  {  1,  -1,  16, -19,  -8,  -4,  -3,   2,  19,   0,   4,  30,  0 },
  {  1,   1,  -3,   0,   2, -11,  15,  -5,   1,   2,  -9,  24,  0 },
  {  0,   1,  -2,   0,   1,  -4,   4,   0,   0,   1,  -4,   7,  0 },
  {  0,   1,   2,  -5,   1,  -6,   4,  10,  -2,   1,  -4,  10,  0 },
  {  3,   0,  -3,  -6,  -2,  -6,  14,   8,  -1,  -1,  -3,  31,  0 },
  {  0,   1,   0,  -2,   1,  -6,   5,   1,   0,   1,  -5,  13,  0 },
  {  3,   1,   9, -19, -21,   9,   7,   6,  13,   5,  15,  21,  0 },
  {  2,   4,   3, -12, -13,   1,   7,   8,   3,   0,  12,  26,  0 },
  {  3,   1,  -8,  -2,   0,  -6,  18,   2,  -2,   3, -10,  23,  0 },
  {  1,   1,  -4,  -1,   1,  -5,   8,   1,  -1,   2,  -5,  10,  0 },
  {  0,   1,  -1,   0,   0,  -2,   2,   0,   0,   1,  -2,   3,  0 },
  {  1,   1,  -2,  -7,   1,  -7,  14,  18,   0,   0,  -7,  21,  0 },
  {  0,   1,   0,  -2,   0,  -7,   8,   1,  -2,   0,  -3,  24,  0 },
  {  0,   1,   1,  -2,   2, -10,  10,   0,  -2,   1,  -7,  23,  0 },
  {  0,   2,   2, -11,   2,  -4,  -3,  39,   7,   1, -10,   9,  0 },
  {  1,   0,  13, -16,  -5,  -6,  -1,   8,   6,   0,   6,  29,  0 },
  {  1,   3,   1,  -6,  -4,  -7,   9,   6,  -3,  -2,   3,  33,  0 },
  {  4,   0, -17,  -1,  -1,   5,  26,   8,  -2,   3, -15,  30,  0 },
  {  0,   1,  -2,   0,   2,  -8,  12,  -6,   1,   1,  -6,  16,  0 },
  {  0,   0,   0,  -1,   1,  -4,   4,   0,   0,   0,  -3,  11,  0 },
  {  0,   1,   2,  -8,   2,  -6,   5,  15,   0,   2,  -7,   9,  0 },
  {  1,  -1,  12, -15,  -7,  -2,   3,   6,   6,  -1,   7,  30,  0 },
};
const int AdaptiveLoopFilter::m_classToFilterMapping[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES] =
{
  {  8,   2,   2,   2,   3,   4,  53,   9,   9,  52,   4,   4,   5,   9,   2,   8,  10,   9,   1,   3,  39,  39,  10,   9,  52 },
  { 11,  12,  13,  14,  15,  30,  11,  17,  18,  19,  16,  20,  20,   4,  53,  21,  22,  23,  14,  25,  26,  26,  27,  28,  10 },
  { 16,  12,  31,  32,  14,  16,  30,  33,  53,  34,  35,  16,  20,   4,   7,  16,  21,  36,  18,  19,  21,  26,  37,  38,  39 },
  { 35,  11,  13,  14,  43,  35,  16,   4,  34,  62,  35,  35,  30,  56,   7,  35,  21,  38,  24,  40,  16,  21,  48,  57,  39 },
  { 11,  31,  32,  43,  44,  16,   4,  17,  34,  45,  30,  20,  20,   7,   5,  21,  22,  46,  40,  47,  26,  48,  63,  58,  10 },
  { 12,  13,  50,  51,  52,  11,  17,  53,  45,   9,  30,   4,  53,  19,   0,  22,  23,  25,  43,  44,  37,  27,  28,  10,  55 },
  { 30,  33,  62,  51,  44,  20,  41,  56,  34,  45,  20,  41,  41,  56,   5,  30,  56,  38,  40,  47,  11,  37,  42,  57,   8 },
  { 35,  11,  23,  32,  14,  35,  20,   4,  17,  18,  21,  20,  20,  20,   4,  16,  21,  36,  46,  25,  41,  26,  48,  49,  58 },
  { 12,  31,  59,  59,   3,  33,  33,  59,  59,  52,   4,  33,  17,  59,  55,  22,  36,  59,  59,  60,  22,  36,  59,  25,  55 },
  { 31,  25,  15,  60,  60,  22,  17,  19,  55,  55,  20,  20,  53,  19,  55,  22,  46,  25,  43,  60,  37,  28,  10,  55,  52 },
  { 12,  31,  32,  50,  51,  11,  33,  53,  19,  45,  16,   4,   4,  53,   5,  22,  36,  18,  25,  43,  26,  27,  27,  28,  10 },
  {  5,   2,  44,  52,   3,   4,  53,  45,   9,   3,   4,  56,   5,   0,   2,   5,  10,  47,  52,   3,  63,  39,  10,   9,  52 },
  { 12,  34,  44,  44,   3,  56,  56,  62,  45,   9,  56,  56,   7,   5,   0,  22,  38,  40,  47,  52,  48,  57,  39,  10,   9 },
  { 35,  11,  23,  14,  51,  35,  20,  41,  56,  62,  16,  20,  41,  56,   7,  16,  21,  38,  24,  40,  26,  26,  42,  57,  39 },
  { 33,  34,  51,  51,  52,  41,  41,  34,  62,   0,  41,  41,  56,   7,   5,  56,  38,  38,  40,  44,  37,  42,  57,  39,  10 },
  { 16,  31,  32,  15,  60,  30,   4,  17,  19,  25,  22,  20,   4,  53,  19,  21,  22,  46,  25,  55,  26,  48,  63,  58,  55 },
};


const Pel AdaptiveLoopFilter::m_alfClippVls[3][MaxAlfNumClippingValues] =
{
  {  256,  32,  8, 2 },
  {  512,  64, 16, 4 },
  { 1024, 128, 32, 8 },
};

void AdaptiveLoopFilter::create( const PicHeader* picHeader, const SPS* sps, const PPS* pps, int numThreads, PelUnitBuf& unitBuf )
{
  auto inputBitDepth = sps->getBitDepths().recon;
  if( m_inputBitDepth != inputBitDepth )
  {
    const auto clippVl = m_alfClippVls[inputBitDepth - 8][0];
    for( unsigned i = 0; i < sizeof( m_clipDefault ) / sizeof( m_clipDefault[0] ); i++ )
    {
      m_clipDefault[i] = clippVl;
    }
  }
  m_inputBitDepth = inputBitDepth;
  m_picWidth  = pps->getPicWidthInLumaSamples();
  m_picHeight = pps->getPicHeightInLumaSamples();
  const int  maxCUWidth  = sps->getMaxCUWidth();
  const int  maxCUHeight = sps->getMaxCUHeight();
  const auto format      = sps->getChromaFormatIdc();

  m_alfVBLumaCTUHeight = maxCUHeight;
  m_alfVBChmaCTUHeight = maxCUHeight >> getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, format );

  m_alfVBLumaPos = m_alfVBLumaCTUHeight - ALF_VB_POS_ABOVE_CTUROW_LUMA;
  m_alfVBChmaPos = m_alfVBChmaCTUHeight - ALF_VB_POS_ABOVE_CTUROW_CHMA;

  CHECK( m_inputBitDepth > 10 , "m_alfClippingValues or m_alfClippVls needs to be enabled/adjusted" );

  bool loopFilterAcrossSubPicEnabledFlag = true;
  if( sps->getSubPicInfoPresentFlag() )
  {
    for( int i = 0; i < sps->getNumSubPics(); ++i )
    {
      if( !sps->getLoopFilterAcrossSubpicEnabledFlag( i ) )
      {
        loopFilterAcrossSubPicEnabledFlag = false;
        break;
      }
    }
  }

  if( picHeader->getVirtualBoundariesPresentFlag() || !pps->getLoopFilterAcrossSlicesEnabledFlag() || !pps->getLoopFilterAcrossTilesEnabledFlag()
      || !loopFilterAcrossSubPicEnabledFlag )
  {
    m_tempBuf.resize( std::max( 1, numThreads ) );
    for( auto &buf: m_tempBuf )
    {
      if( buf.chromaFormat!=format || buf.Y()!=Size( maxCUWidth, maxCUHeight ) )
      {
        buf.destroy();
        buf.create( format, Size( maxCUWidth, maxCUHeight ), maxCUWidth, 2 * MAX_ALF_PADDING_SIZE, 0, false );
      }
    }
  }

  classifier.resize( std::max( 1, numThreads ) );

  m_alfBuf = unitBuf;
}

void AdaptiveLoopFilter::destroy()
{
  m_tempBuf.clear();
  classifier.clear();
}

void AdaptiveLoopFilter::prepareCTU( CodingStructure &cs, unsigned col, unsigned line )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_ALF, cs, CH_L );
  // border-extend the buffer per ctu-line
  const int PEL_EXT_SIZE = MAX_ALF_FILTER_LENGTH/2 + ( MAX_ALF_FILTER_LENGTH/2 % 2 );  // PEL_EXT_SIZE needs to be divisible by 2

  PelUnitBuf recYuv = cs.getRecoBuf();
  const UnitArea pelExtUnitArea = getCtuArea( cs, col, line, true );

  recYuv.subBuf( pelExtUnitArea ).extendBorderPel( PEL_EXT_SIZE, col == 0, col == cs.pcv->widthInCtus - 1, line == 0, line == cs.pcv->heightInCtus - 1 );
}


void AdaptiveLoopFilter::processCTU( CodingStructure & cs, unsigned col, unsigned line, int tid, const ChannelType chType )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_ALF, cs, CH_L );

  const UnitArea ctuArea( getCtuArea( cs, col, line, true ) );
  CPelUnitBuf    recYuv = cs.getRecoBuf().subBuf( ctuArea );
  PelUnitBuf     dstYuv = m_alfBuf.subBuf( ctuArea );

  const unsigned ctuIdx  = line * cs.pcv->widthInCtus + col;
  CtuAlfData currAlfData = cs.getCtuData( col, line ).alfParam;
  currAlfData.alfCtuEnableFlag[1] += currAlfData.ccAlfFilterControl[0] > 0 ? 2 : 0;
  currAlfData.alfCtuEnableFlag[2] += currAlfData.ccAlfFilterControl[1] > 0 ? 2 : 0;

  filterCTU( recYuv, dstYuv, currAlfData, cs.picture->slices[0]->getClpRngs(), chType, cs, ctuIdx, ctuArea.lumaPos(), tid );
}

bool AdaptiveLoopFilter::getAlfSkipPic( const CodingStructure & cs )
{
  if( cs.pps->getAlfInfoInPhFlag() )
  {
    // ph_alf_enabled_flag, if false, implies chroma ALF and CCALF are also off
    return !cs.picHeader->getAlfEnabledFlag( COMPONENT_Y );
  }
  else if( cs.picture->slices.size() == 1 )
  {
    // sh_alf_enabled_flag, if false, implies chroma ALF and CCALF are also off
    return !cs.picture->slices[0]->getAlfEnabledFlag( COMPONENT_Y );
  }

  return false;
}

void AdaptiveLoopFilter::filterAreaLuma( const CPelUnitBuf& srcBuf,
                                               PelUnitBuf&  dstBuf,
                                         const Area&        blk,
                                         const Slice*       slice,
                                         const APS* const*  aps,
                                         const short        filterSetIndex,
                                         const ClpRngs&     clpRngs,
                                         const int          tId )

{
  const short* coeff = nullptr;
  const short* clip  = nullptr;
  if( filterSetIndex >= NUM_FIXED_FILTER_SETS )
  {
    CHECK( slice->getNumAlfAps() <= ( filterSetIndex - NUM_FIXED_FILTER_SETS ), "deduemm" );
    const int apsIdx = slice->getAlfApsIdsLuma()[filterSetIndex - NUM_FIXED_FILTER_SETS];

    const APS* curAPS = aps[apsIdx];   // TODO: check this
    CHECK( curAPS == NULL, "invalid APS" );
    const AlfSliceParam& alfSliceParam = curAPS->getAlfAPSParam();
    coeff                              = alfSliceParam.lumaCoeffFinal;
    clip                               = alfSliceParam.lumaClippFinal;
  }
  else
  {
    coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
    clip  = m_clipDefault;
  }

  const int bottom = blk.y + blk.height;
  const int right = blk.x + blk.width;

  for( int i = blk.y; i < bottom; i += m_CLASSIFICATION_BLK_SIZE )
  {
    int nHeight = std::min( i + m_CLASSIFICATION_BLK_SIZE, bottom ) - i;

    for( int j = blk.x; j < right; j += m_CLASSIFICATION_BLK_SIZE )
    {
      int nWidth = std::min( j + m_CLASSIFICATION_BLK_SIZE, right ) - j;
      
      m_deriveClassificationBlk( classifier[tId].data(), srcBuf.Y(),     Area( j, i, nWidth, nHeight ), m_inputBitDepth + 4,               m_alfVBLumaCTUHeight, m_alfVBLumaPos );
      m_filter7x7Blk           ( classifier[tId].data(), dstBuf, srcBuf, Area( j, i, nWidth, nHeight ), COMPONENT_Y, coeff, clip, clpRngs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );
    }
  }
}

void AdaptiveLoopFilter::filterAreaChroma( const CPelUnitBuf& srcBuf,
                                                 PelUnitBuf&  dstBuf,
                                           const Area&        blkChroma,
                                           const ComponentID  compID,
                                           const Slice*       slice,
                                           const APS* const*  aps,
                                           const CtuAlfData&  ctuAlfData,
                                           const ClpRngs&     clpRngs )

{
  if( ctuAlfData.alfCtuEnableFlag[compID] & 1 )
  {
    const int  apsIdxChroma = slice->getAlfApsIdChroma();
    const APS* curAPS       = aps[apsIdxChroma];
    CHECK( curAPS == NULL, "invalid APS" );
    const AlfSliceParam& alfSliceParam = curAPS->getAlfAPSParam();

    const uint8_t altIdx = ctuAlfData.alfCtuAlternative[compID - 1];

    m_filter5x5Blk( nullptr,
                    dstBuf,
                    srcBuf,
                    blkChroma,
                    compID,
                    alfSliceParam.chromaCoeff    + altIdx * MAX_NUM_ALF_CHROMA_COEFF,
                    alfSliceParam.chrmClippFinal + altIdx * MAX_NUM_ALF_CHROMA_COEFF,
                    clpRngs,
                    m_alfVBChmaCTUHeight,
                    m_alfVBChmaPos );
  }
  else
  {
#if ALF_FIX
    dstBuf.get( compID ).subBuf( blkChroma ).copyFrom( srcBuf.get( compID ).subBuf( blkChroma ) );
#else
    dstBuf.get( compID ).copyFrom( srcBuf.get( compID ) );
#endif
  }
}

void AdaptiveLoopFilter::filterAreaChromaCc( const CPelUnitBuf& srcBuf,
                                                   PelUnitBuf&  dstBuf,
                                             const Area&        blkLuma,
                                             const Area&        blkChroma,
                                             const ComponentID  compID,
                                             const Slice*       slice,
                                             const APS* const*  aps,
                                             const CtuAlfData&  ctuAlfData,
                                             const ClpRngs&     clpRngs )

{
  if( slice->getCcAlfEnabledFlag( compID ) )
  {
    const int filterIdx = ctuAlfData.ccAlfFilterControl[compID - 1];

    if( filterIdx != 0 )
    {
      const int      apsIdx      = compID == 1 ? slice->getCcAlfCbApsId()
                                               : slice->getCcAlfCrApsId();
      const int16_t* filterCoeff = aps[apsIdx]->getCcAlfAPSParam().ccAlfCoeff[compID - 1][filterIdx - 1];

      m_filterCcAlf( dstBuf.get( compID ), srcBuf, blkChroma, blkLuma, compID, filterCoeff, clpRngs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );
    }
  }
}

void AdaptiveLoopFilter::filterAreaChromaBothCc( const CPelUnitBuf& srcBuf,
                                                       PelUnitBuf&  dstBuf,
                                                 const Area&        blkLuma,
                                                 const Area&        blkChroma,
                                                 const Slice*       slice,
                                                 const APS* const*  aps,
                                                 const CtuAlfData&  ctuAlfData,
                                                 const ClpRngs&     clpRngs )

{
  const int filterIdxCb =
      slice->getCcAlfEnabledFlag(COMPONENT_Cb) ? ctuAlfData.ccAlfFilterControl[COMPONENT_Cb - 1] : 0;
  const int filterIdxCr =
      slice->getCcAlfEnabledFlag(COMPONENT_Cr) ? ctuAlfData.ccAlfFilterControl[COMPONENT_Cr - 1] : 0;
  
  if( filterIdxCb && filterIdxCr )
  {
    const Area blk( Position( 0, 0 ), Size( srcBuf.get( COMPONENT_Cb ) ) );
    int apsIdxCb = slice->getCcAlfCbApsId();
    const int16_t* filterCoeffCb =
        slice->getAlfAPSs()[apsIdxCb]->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cb - 1][filterIdxCb - 1];
    int apsIdxCr = slice->getCcAlfCrApsId();
    const int16_t* filterCoeffCr =
        slice->getAlfAPSs()[apsIdxCr]->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cr - 1][filterIdxCr - 1];

    m_filterCcAlfBoth( dstBuf.get( COMPONENT_Cb ), dstBuf.get( COMPONENT_Cr ), srcBuf, blkChroma, blkLuma, filterCoeffCb, filterCoeffCr, clpRngs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );

  }
  else
  {
    if( filterIdxCb )
    {
      const Area blk( Position( 0, 0 ), Size( srcBuf.get( COMPONENT_Cb ) ) );
      int apsIdx = slice->getCcAlfCbApsId();
      const int16_t* filterCoeff =
          slice->getAlfAPSs()[apsIdx]->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cb - 1][filterIdxCb - 1];

      m_filterCcAlf( dstBuf.get( COMPONENT_Cb ), srcBuf, blkChroma, blkLuma, COMPONENT_Cb, filterCoeff, clpRngs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );
    }

    if( filterIdxCr )
    {
      const Area blk( Position( 0, 0 ), Size( srcBuf.get( COMPONENT_Cr ) ) );
      int apsIdx = slice->getCcAlfCrApsId();
      const int16_t* filterCoeff =
          slice->getAlfAPSs()[apsIdx]->getCcAlfAPSParam().ccAlfCoeff[COMPONENT_Cr - 1][filterIdxCr - 1];

      m_filterCcAlf( dstBuf.get( COMPONENT_Cr ), srcBuf, blkChroma, blkLuma, COMPONENT_Cr, filterCoeff, clpRngs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );
    }
  }
}

void AdaptiveLoopFilter::filterCTU( const CPelUnitBuf&     srcBuf,
                                           PelUnitBuf&     dstBuf,
                                    const CtuAlfData&      ctuAlfData,
                                    const ClpRngs&         clpRngs,
                                    const ChannelType      chType,
                                    const CodingStructure& cs,
                                    int                    ctuIdx,
                                    Position               ctuPos,
                                    int                    tid )
{
  const Slice*         slice          = cs.getCtuData( ctuIdx ).cuPtr[0][0]->slice;
  const APS* const*    aps            = slice->getAlfAPSs();
  const PreCalcValues& pcv            = *cs.pcv;

  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int  numHorVirBndry   = 0;
  int  numVerVirBndry   = 0;
  int  horVirBndryPos[] = { 0, 0, 0 };
  int  verVirBndryPos[] = { 0, 0, 0 };

  int rasterSliceAlfPad = 0;
  bool isCrssByVBs = isClipOrCrossedByVirtualBoundaries( cs,
                                                         Area( ctuPos, Size( srcBuf.Y() ) ),
                                                         clipTop, clipBottom, clipLeft, clipRight,
                                                         numHorVirBndry, numVerVirBndry,
                                                         horVirBndryPos, verVirBndryPos,
                                                         rasterSliceAlfPad );
  if( isCrssByVBs )
  {
    CHECK( numHorVirBndry >= (int)( sizeof(horVirBndryPos) / sizeof(horVirBndryPos[0]) ), "Too many virtual boundaries" );
    CHECK( numHorVirBndry >= (int)( sizeof(verVirBndryPos) / sizeof(verVirBndryPos[0]) ), "Too many virtual boundaries" );
  }

  const int width  = ( ctuPos.x + pcv.maxCUWidth  > pcv.lumaWidth  ) ? ( pcv.lumaWidth  - ctuPos.x ) : pcv.maxCUWidth;
  const int height = ( ctuPos.y + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - ctuPos.y ) : pcv.maxCUHeight;

  const int numComp = getNumberValidComponents( pcv.chrFormat );
  
  if( !isCrssByVBs )
  {
    for( int compIdx = 0; compIdx < numComp; compIdx++ )
    {
      const ComponentID compID = ComponentID( compIdx );

      if( chType < MAX_NUM_CHANNEL_TYPE && toChannelType( compID ) != chType )
        continue;
  #if ALF_FIX
      if( !ctuAlfData.alfCtuEnableFlag[compIdx] && ( compIdx == 0 || !slice->getCcAlfEnabledFlag( compIdx ) ) )
  #else
      if( !ctuAlfData.alfCtuEnableFlag[compIdx] )
  #endif
      {
        // unfiltered blocks just need to be copied to the destination
        dstBuf.get( compID ).copyFrom( srcBuf.get( compID ) );
        continue;
      }

      if( compID == COMPONENT_Y )
      {
        const Area blk( Position( 0, 0 ), Size( srcBuf.get( compID ) ) );
        const short filterSetIndex = ctuAlfData.alfCtbFilterIndex;
        filterAreaLuma( srcBuf, dstBuf, blk, slice, aps, filterSetIndex, clpRngs, tid );
      }
      else
      {
        const Area blkLuma  ( Position( 0, 0 ), Size( width, height ) );
        const Area blkChroma( Position( 0, 0 ), Size( srcBuf.get( compID ) ) );

        filterAreaChroma( srcBuf, dstBuf, blkChroma, compID, slice, aps, ctuAlfData, clpRngs );
      }
    }
    
    // has chroma
    if( numComp > 1 )
    {
      const Area blkLuma  ( Position( 0, 0 ), Size( width, height ) );
      const Area blkChroma( Position( 0, 0 ), Size( srcBuf.get( COMPONENT_Cb ) ) );
      filterAreaChromaBothCc( srcBuf, dstBuf, blkLuma, blkChroma, slice, aps, ctuAlfData, clpRngs );
    }
  }
  else
  {
    for( int compIdx = 0; compIdx < numComp; compIdx++ )
    {
      const ComponentID compID = ComponentID( compIdx );

      if( chType < MAX_NUM_CHANNEL_TYPE && toChannelType( compID ) != chType )
        continue;
  #if ALF_FIX
      if( !ctuAlfData.alfCtuEnableFlag[compIdx] && ( compIdx == 0 || !slice->getCcAlfEnabledFlag( compIdx ) ) )
  #else
      if( !ctuAlfData.alfCtuEnableFlag[compIdx] )
  #endif
      {
        // unfiltered blocks just need to be copied to the destination
        dstBuf.get( compID ).copyFrom( srcBuf.get( compID ) );
        continue;
      }

      // isCrssByVBs
      {
        const int chromaScaleX = getComponentScaleX( compID, srcBuf.chromaFormat );
        const int chromaScaleY = getComponentScaleY( compID, srcBuf.chromaFormat );

        int yStart = ctuPos.y;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int  yEnd  = i == numHorVirBndry ? ctuPos.y + srcBuf.Y().height : horVirBndryPos[i];
          const int  h     = yEnd - yStart;
          const bool clipT = ( i == 0              && clipTop )    || ( i > 0 )              || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd   == pcv.lumaHeight );

          int xStart = ctuPos.x;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int  xEnd  = j == numVerVirBndry ? ctuPos.x + srcBuf.Y().width : verVirBndryPos[j];
            const int  w     = xEnd - xStart;
            const bool clipL = ( j == 0              && clipLeft )  || ( j > 0 )              || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd  == pcv.lumaWidth );

            const int padL = clipL ? 0 : MAX_ALF_PADDING_SIZE;
            const int padR = clipR ? 0 : MAX_ALF_PADDING_SIZE;
            const int padT = clipT ? 0 : MAX_ALF_PADDING_SIZE;
            const int padB = clipB ? 0 : MAX_ALF_PADDING_SIZE;

            const int yInSrc = yStart - ctuPos.y;
            const int xInSrc = xStart - ctuPos.x;

            const Size     paddedSize( w + padL + padR, h + padT + padB );
            const Position posInSrc( xInSrc - padL, yInSrc - padT );

            if( compID == COMPONENT_Y || !slice->getCcAlfEnabledFlag( compIdx ) )
            {
              auto tmpSubBuf = m_tempBuf[tid].subBuf( Area( posInSrc, paddedSize ) ).bufs[compID];
              tmpSubBuf.copyFrom( srcBuf.subBuf( Area( posInSrc, paddedSize ) ).bufs[compID] );

              // pad top-left unavailable samples for raster slice
              if( xStart == ctuPos.x && yStart == ctuPos.y && ( rasterSliceAlfPad & 1 ) )
              {
                tmpSubBuf.padBorderPel( MAX_ALF_PADDING_SIZE, MAX_ALF_PADDING_SIZE, 1 );
              }

              // pad bottom-right unavailable samples for raster slice
              if( xEnd == ctuPos.x + width && yEnd == ctuPos.y + height && ( rasterSliceAlfPad & 2 ) )
              {
                tmpSubBuf.padBorderPel( MAX_ALF_PADDING_SIZE, MAX_ALF_PADDING_SIZE, 2 );
              }
              tmpSubBuf.extendBorderPel( MAX_ALF_PADDING_SIZE );
            }
            else
            {
              auto tmpSubBuf = m_tempBuf[tid].subBuf( Area( posInSrc, paddedSize ) );
              tmpSubBuf.copyFrom( srcBuf.subBuf( Area( posInSrc, paddedSize ) ) );

              // pad top-left unavailable samples for raster slice
              if( xStart == ctuPos.x && yStart == ctuPos.y && ( rasterSliceAlfPad & 1 ) )
              {
                tmpSubBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
              }

              // pad bottom-right unavailable samples for raster slice
              if( xEnd == ctuPos.x + width && yEnd == ctuPos.y + height && ( rasterSliceAlfPad & 2 ) )
              {
                tmpSubBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
              }
              tmpSubBuf.extendBorderPel( MAX_ALF_PADDING_SIZE );
            }

            if( compID == COMPONENT_Y )
            {
              const Area blk( xInSrc, yInSrc, w, h );
              const short filterSetIndex = ctuAlfData.alfCtbFilterIndex;
              filterAreaLuma( m_tempBuf[tid], dstBuf, blk, slice, aps, filterSetIndex, clpRngs, tid );
            }
            else
            {
              const Area blkLuma ( Position( xInSrc,                 yInSrc ),                 Size( w,                 h ) );
              const Area blkChoma( Position( xInSrc >> chromaScaleX, yInSrc >> chromaScaleY ), Size( w >> chromaScaleX, h >> chromaScaleY ) );

              filterAreaChroma( m_tempBuf[tid], dstBuf, blkChoma, compID, slice, aps, ctuAlfData, clpRngs );
              filterAreaChromaCc( m_tempBuf[tid], dstBuf, blkLuma, blkChoma, compID, slice, aps, ctuAlfData, clpRngs );
            }
            xStart = xEnd;
          }
          yStart = yEnd;
        }
      }
    }
  }
}

void AdaptiveLoopFilter::reconstructCoeffAPSs( Slice& slice )
{
  const SPS*  sps = slice.getSPS();
  const APS** aps = slice.getAlfAPSs();

  // luma
  if( slice.getAlfEnabledFlag( COMPONENT_Y ) )
  {
    for( int i = 0; i < slice.getNumAlfAps(); i++ )
    {
      int        apsIdx = slice.getAlfApsIdsLuma()[i];
      const APS* curAPS = aps[apsIdx];
      CHECK( curAPS == NULL, "invalid APS" );

      AlfSliceParam& alfSliceParamTmp = curAPS->getMutableAlfAPSParam();
      reconstructCoeff( alfSliceParamTmp, CHANNEL_TYPE_LUMA, sps->getBitDepths().recon );
      curAPS->releaseMutableAlfAPSParam( alfSliceParamTmp );
    }
  }

  // chroma
  if( slice.getAlfEnabledFlag( COMPONENT_Cb ) || slice.getAlfEnabledFlag( COMPONENT_Cr ) )
  {
    int        apsIdxChroma = slice.getAlfApsIdChroma();
    const APS* curAPS       = aps[apsIdxChroma];
    CHECK( curAPS == NULL, "invalid APS" );

    AlfSliceParam& alfSliceParamTmp = curAPS->getMutableAlfAPSParam();
    reconstructCoeff( alfSliceParamTmp, CHANNEL_TYPE_CHROMA, sps->getBitDepths().recon );
    curAPS->releaseMutableAlfAPSParam( alfSliceParamTmp );
  }
}

void AdaptiveLoopFilter::reconstructCoeff( AlfSliceParam& alfSliceParam, ChannelType channel, const int inputBitDepth )
{
  if( isChroma( channel ) && alfSliceParam.chrmFinalDone )
  {
    return;
  }
  if( isLuma( channel ) && alfSliceParam.lumaFinalDone )
  {
    return;
  }

  const AlfFilterType filterType     = isLuma( channel ) ? ALF_FILTER_7 : ALF_FILTER_5;
  const int           numClasses     = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  const int           numCoeff       = filterType == ALF_FILTER_5 ? 7 : 13;
  const int           numCoeffMinus1 = numCoeff - 1;


  if( isChroma( channel ) )
  {
    const int numAlts =  alfSliceParam.numAlternativesChroma;
    for( int altIdx = 0; altIdx < numAlts; ++ altIdx )
    {
      for( int coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx )
      {
        const int clipIdx = alfSliceParam.nonLinearFlagChroma ? alfSliceParam.chromaClipp[altIdx * MAX_NUM_ALF_CHROMA_COEFF + coeffIdx] : 0;
        alfSliceParam.chrmClippFinal[altIdx * MAX_NUM_ALF_CHROMA_COEFF + coeffIdx] = m_alfClippVls[inputBitDepth - 8][clipIdx];
      }
      alfSliceParam.chrmClippFinal[altIdx * MAX_NUM_ALF_CHROMA_COEFF + numCoeffMinus1] = m_alfClippVls[inputBitDepth - 8][0];
    }
    alfSliceParam.chrmFinalDone = true;
    return;
  }

  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    int filterIdx      = alfSliceParam.filterCoeffDeltaIdx[classIdx];
    memcpy( alfSliceParam.lumaCoeffFinal + classIdx * MAX_NUM_ALF_LUMA_COEFF, alfSliceParam.lumaCoeff + filterIdx * MAX_NUM_ALF_LUMA_COEFF, sizeof( int16_t ) * numCoeffMinus1 );
    alfSliceParam.lumaClippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = m_alfClippVls[inputBitDepth - 8][0];
    for( int coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx )
    {
      const int clipIdx = alfSliceParam.nonLinearFlagLuma ? alfSliceParam.lumaClipp[filterIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx] : 0;
      alfSliceParam.lumaClippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx] = m_alfClippVls[inputBitDepth - 8][clipIdx];
    }
#if ALF_PRE_TRANSPOSE

    for( int tranposeIdx = 1; tranposeIdx < 4; tranposeIdx++ )
    {
      short* coef = alfSliceParam.lumaCoeffFinal + classIdx * MAX_NUM_ALF_LUMA_COEFF;
      short* txcf = alfSliceParam.lumaCoeffFinal + classIdx * MAX_NUM_ALF_LUMA_COEFF + tranposeIdx * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF;
    
      short* coefClp = alfSliceParam.lumaClippFinal + classIdx * MAX_NUM_ALF_LUMA_COEFF;
      short* txcfClp = alfSliceParam.lumaClippFinal + classIdx * MAX_NUM_ALF_LUMA_COEFF + tranposeIdx * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF;
    
      if( tranposeIdx == 1 )
      {
        short tcff[] = { coef[9], coef[4], coef[10], coef[8], coef[1], coef[5], coef[11], coef[7], coef[3], coef[0], coef[2], coef[6], coef[12] };
        memcpy( txcf, tcff, sizeof( tcff ) );
        short tcffClp[] = { coefClp[9], coefClp[4], coefClp[10], coefClp[8], coefClp[1], coefClp[5], coefClp[11], coefClp[7], coefClp[3], coefClp[0], coefClp[2], coefClp[6], coefClp[12] };
        memcpy( txcfClp, tcffClp, sizeof( tcffClp ) );
      }
      else if( tranposeIdx == 2 )
      {
        short tcff[] = { coef[0], coef[3], coef[2], coef[1], coef[8], coef[7], coef[6], coef[5], coef[4], coef[9], coef[10], coef[11], coef[12] };
        memcpy( txcf, tcff, sizeof( tcff ) );
        short tcffClp[] = { coefClp[0], coefClp[3], coefClp[2], coefClp[1], coefClp[8], coefClp[7], coefClp[6], coefClp[5], coefClp[4], coefClp[9], coefClp[10], coefClp[11], coefClp[12] };
        memcpy( txcfClp, tcffClp, sizeof( tcffClp ) );
      }
      else if( tranposeIdx == 3 )
      {
        short tcff[] = { coef[9], coef[8], coef[10], coef[4], coef[3], coef[7], coef[11], coef[5], coef[1], coef[0], coef[2], coef[6], coef[12] };
        memcpy( txcf, tcff, sizeof( tcff ) );
        short tcffClp[] = { coefClp[9], coefClp[8], coefClp[10], coefClp[4], coefClp[3], coefClp[7], coefClp[11], coefClp[5], coefClp[1], coefClp[0], coefClp[2], coefClp[6], coefClp[12] };
        memcpy( txcfClp, tcffClp, sizeof( tcffClp ) );
      }
    }
#endif
  }

  alfSliceParam.lumaFinalDone = true;
}

void AdaptiveLoopFilter::deriveClassificationBlk( AlfClassifier *classifier, const CPelBuf& srcLuma, const Area& blk, const int shift, int vbCTUHeight, int vbPos )
{
  static const int th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
  const ptrdiff_t stride  = srcLuma.stride;
  const Pel* src          = srcLuma.buf;
  const int maxActivity   = 15;

  const int fl   = 2;
  const int flP1 = fl + 1;
  const int fl2  = 2 * fl;


  const int height      = blk.height + fl2;
  const int width       = blk.width + fl2;
  const int posX        = blk.x;
  const int posY        = blk.y;
  const int startHeight = posY - flP1;

  using array2d = int[(m_CLASSIFICATION_BLK_SIZE + 5)/2][(m_CLASSIFICATION_BLK_SIZE + 5)/2];
#if ASAN_WORKAROUND
  std::vector<array2d> laplacian(NUM_DIRECTIONS);   // put the laplacian array on the heap, otherwise we have stack-overflows (we have a fast SIMD-implementation so the performance doesn't matter here)
#else
  std::array<array2d, NUM_DIRECTIONS> laplacian;
#endif

  for( int i = 0; i < height; i += 2 )
  {
    ptrdiff_t yoffset = ( i + 1 + startHeight ) * stride - flP1;

    const Pel *src0 = &src[yoffset - stride];
    const Pel *src1 = &src[yoffset];
    const Pel *src2 = &src[yoffset + stride];
    const Pel *src3 = &src[yoffset + stride * 2];
    if( ( blk.y - 2 + i ) > 0 &&
        ( blk.y - 2 + i ) % vbCTUHeight == vbPos - 2 )
    {
      src3 = &src[yoffset + stride];
    }
    else if( ( blk.y - 2 + i ) > 0 &&
             ( blk.y - 2 + i ) % vbCTUHeight == vbPos )
    {
      src0 = &src[yoffset];
    }
    int* pYver  = laplacian[VER]  [i/2];
    int* pYhor  = laplacian[HOR]  [i/2];
    int* pYdig0 = laplacian[DIAG0][i/2];
    int* pYdig1 = laplacian[DIAG1][i/2];

    for( int j = 0; j < width; j += 2 )
    {
      int pixY          = j + 1 + posX;
      const Pel *pY     = src1 + pixY;
      const Pel* pYdown = src0 + pixY;
      const Pel* pYup   = src2 + pixY;
      const Pel* pYup2  = src3 + pixY;

      const Pel y0      = pY[0] << 1;
      const Pel yup1    = pYup[1] << 1;

      pYver [j/2] = abs( y0 - pYdown[ 0] - pYup  [0] ) + abs( yup1 - pY   [1] - pYup2[1] );
      pYhor [j/2] = abs( y0 - pY    [ 1] - pY   [-1] ) + abs( yup1 - pYup [2] - pYup [0] );
      pYdig0[j/2] = abs( y0 - pYdown[-1] - pYup  [1] ) + abs( yup1 - pY   [0] - pYup2[2] );
      pYdig1[j/2] = abs( y0 - pYup  [-1] - pYdown[1] ) + abs( yup1 - pYup2[0] - pY   [2] );

      if( j > 4 && ( j - 6 ) % 4 == 0 )
      {
        int jM6 = (j - 6);
        int jM4 = (j - 4);
        int jM2 = (j - 2);

        pYver [jM6/2] += pYver [jM4/2] + pYver [jM2/2] + pYver [j/2];
        pYhor [jM6/2] += pYhor [jM4/2] + pYhor [jM2/2] + pYhor [j/2];
        pYdig0[jM6/2] += pYdig0[jM4/2] + pYdig0[jM2/2] + pYdig0[j/2];
        pYdig1[jM6/2] += pYdig1[jM4/2] + pYdig1[jM2/2] + pYdig1[j/2];
      }
    }
  }

  // classification block size
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  for( int i = 0; i < blk.height; i += clsSizeY )
  {
    int* pYver   = laplacian[VER]  [(i    )/2];
    int* pYver2  = laplacian[VER]  [(i + 2)/2];
    int* pYver4  = laplacian[VER]  [(i + 4)/2];
    int* pYver6  = laplacian[VER]  [(i + 6)/2];

    int* pYhor   = laplacian[HOR]  [(i    )/2];
    int* pYhor2  = laplacian[HOR]  [(i + 2)/2];
    int* pYhor4  = laplacian[HOR]  [(i + 4)/2];
    int* pYhor6  = laplacian[HOR]  [(i + 6)/2];

    int* pYdig0  = laplacian[DIAG0][(i    )/2];
    int* pYdig02 = laplacian[DIAG0][(i + 2)/2];
    int* pYdig04 = laplacian[DIAG0][(i + 4)/2];
    int* pYdig06 = laplacian[DIAG0][(i + 6)/2];

    int* pYdig1  = laplacian[DIAG1][(i    )/2];
    int* pYdig12 = laplacian[DIAG1][(i + 2)/2];
    int* pYdig14 = laplacian[DIAG1][(i + 4)/2];
    int* pYdig16 = laplacian[DIAG1][(i + 6)/2];

    for( int j = 0; j < blk.width; j += clsSizeX )
    {
      int sumV = 0; int sumH = 0; int sumD0 = 0; int sumD1 = 0;
      if( ( i + blk.y ) % vbCTUHeight == vbPos - 4 )
      {
        sumV  = pYver [j/2] + pYver2 [j/2] + pYver4 [j/2];
        sumH  = pYhor [j/2] + pYhor2 [j/2] + pYhor4 [j/2];
        sumD0 = pYdig0[j/2] + pYdig02[j/2] + pYdig04[j/2];
        sumD1 = pYdig1[j/2] + pYdig12[j/2] + pYdig14[j/2];
      }
      else if( ( i + blk.y ) % vbCTUHeight == vbPos )
      {
        sumV  = pYver2 [j/2] + pYver4 [j/2] + pYver6 [j/2];
        sumH  = pYhor2 [j/2] + pYhor4 [j/2] + pYhor6 [j/2];
        sumD0 = pYdig02[j/2] + pYdig04[j/2] + pYdig06[j/2];
        sumD1 = pYdig12[j/2] + pYdig14[j/2] + pYdig16[j/2];
      }
      else
      {
        sumV  = pYver [j/2] + pYver2 [j/2] + pYver4 [j/2] + pYver6 [j/2];
        sumH  = pYhor [j/2] + pYhor2 [j/2] + pYhor4 [j/2] + pYhor6 [j/2];
        sumD0 = pYdig0[j/2] + pYdig02[j/2] + pYdig04[j/2] + pYdig06[j/2];
        sumD1 = pYdig1[j/2] + pYdig12[j/2] + pYdig14[j/2] + pYdig16[j/2];
      }
      int tempAct  = sumV + sumH;
      int activity = 0;
      if( ( i + blk.y ) % vbCTUHeight == vbPos - 4 ||
          ( i + blk.y ) % vbCTUHeight == vbPos )
      {
        activity = (Pel)Clip3<int>(0, maxActivity, (tempAct * 96) >> shift);
      }
      else
      {
        activity = (Pel)Clip3<int>(0, maxActivity, (tempAct * 64) >> shift);
      }
      int classIdx = th[activity];

      int hv1, hv0, d1, d0, hvd1, hvd0;
      int mainDirection, secondaryDirection, dirTempHV, dirTempD;
      if( sumV > sumH )
      {
        hv1 = sumV;
        hv0 = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1 = sumH;
        hv0 = sumV;
        dirTempHV = 3;
      }

      if( sumD0 > sumD1 )
      {
        d1 = sumD0;
        d0 = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1 = sumD1;
        d0 = sumD0;
        dirTempD = 2;
      }
      if( (uint32_t)d1 * (uint32_t)hv0 > (uint32_t)hv1 * (uint32_t)d0 )
      {
        hvd1 = d1;
        hvd0 = d0;
        mainDirection = dirTempD;
        secondaryDirection = dirTempHV;
      }
      else
      {
        hvd1 = hv1;
        hvd0 = hv0;
        mainDirection = dirTempHV;
        secondaryDirection = dirTempD;
      }

      int directionStrength = 0;
      if( hvd1 > 2 * hvd0 )
      {
        directionStrength = 1;
      }
      if( hvd1 * 2 > 9 * hvd0 )
      {
        directionStrength = 2;
      }

      if( directionStrength )
      {
        classIdx += ( ( ( mainDirection & 0x1 ) << 1 ) + directionStrength ) * 5;
      }

      static const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx = transposeTable[mainDirection * 2 + ( secondaryDirection >> 1 )];

      classifier[( i / 4 ) * ( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE / 4 ) + j / 4] = AlfClassifier( classIdx, transposeIdx );
    }
  }
}

template<AlfFilterType filtType>
void AdaptiveLoopFilter::filterBlk( const AlfClassifier* classifier,
                                    const PelUnitBuf&    recDst,
                                    const CPelUnitBuf&   recSrc,
                                    const Area&          blk,
                                    const ComponentID    compId,
                                    const short*         filterSet,
                                    const short*         fClipSet,
                                    const ClpRng&        clpRng,
                                    int                  vbCTUHeight,
                                    int                  vbPos )
{
  const bool bChroma = isChroma( compId );

  if( bChroma )
  {
    CHECK( filtType != 0, "Chroma needs to have filtType == 0" );
  }

  const CPelBuf srcLuma = recSrc.get( compId );
         PelBuf dstLuma = recDst.get( compId );

  const ptrdiff_t srcStride = srcLuma.stride;
  const ptrdiff_t dstStride = dstLuma.stride;

  const int startHeight = blk.y;
  const int endHeight   = blk.y + blk.height;
  const int startWidth  = blk.x;
  const int endWidth    = blk.x + blk.width;

  const Pel* src = srcLuma.buf;
        Pel* dst = dstLuma.buf + startHeight * dstStride;

  const short *filterCoeff = filterSet;
  const short *filterClipp = fClipSet;

  const int shift   = m_NUM_BITS - 1;
  const int offset  = 1 << ( shift - 1 );

  const int clsSizeY = 4;
  const int clsSizeX = 4;

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth  % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth  - startWidth )  % clsSizeX, "Wrong endWidth in filtering" );

  ptrdiff_t dstStride2 = dstStride * clsSizeY;
  ptrdiff_t srcStride2 = srcStride * clsSizeY;

  const Pel* pImgYPad0 = src + startHeight * srcStride + startWidth;
  const Pel* pImgYPad1 = pImgYPad0 + srcStride;
  const Pel* pImgYPad2 = pImgYPad0 - srcStride;
  const Pel* pImgYPad3 = pImgYPad1 + srcStride;
  const Pel* pImgYPad4 = pImgYPad2 - srcStride;
  const Pel* pImgYPad5 = pImgYPad3 + srcStride;
  const Pel* pImgYPad6 = pImgYPad4 - srcStride;

  Pel* pRec0 = dst   + startWidth;
  Pel* pRec1 = pRec0 + dstStride;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    for( int j = 0; j < endWidth - startWidth; j += clsSizeX )
    {
      if( !bChroma )
      {
        const AlfClassifier &cl = classifier[( i / 4 ) * ( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE / 4 ) + j / 4];
        filterCoeff = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF + cl.transposeIdx * MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES;
        filterClipp = fClipSet  + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF + cl.transposeIdx * MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES;
      }
      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        const Pel* pImg0 = pImgYPad0 + j + ii * srcStride;
        const Pel* pImg1 = pImgYPad1 + j + ii * srcStride;
        const Pel* pImg2 = pImgYPad2 + j + ii * srcStride;
        const Pel* pImg3 = pImgYPad3 + j + ii * srcStride;
        const Pel* pImg4 = pImgYPad4 + j + ii * srcStride;
        const Pel* pImg5 = pImgYPad5 + j + ii * srcStride;
        const Pel* pImg6 = pImgYPad6 + j + ii * srcStride;

        pRec1 = pRec0 + j + ii * dstStride;

        if ((startHeight + i + ii) % vbCTUHeight < vbPos && ((startHeight + i + ii) % vbCTUHeight >= vbPos - (bChroma ? 2 : 4))) //above
        {
          pImg1 = ((startHeight + i + ii) % vbCTUHeight == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = ((startHeight + i + ii) % vbCTUHeight >= vbPos - 2) ? pImg1 : pImg3;
          pImg5 = ((startHeight + i + ii) % vbCTUHeight >= vbPos - 3) ? pImg3 : pImg5;

          pImg2 = ((startHeight + i + ii) % vbCTUHeight == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = ((startHeight + i + ii) % vbCTUHeight >= vbPos - 2) ? pImg2 : pImg4;
          pImg6 = ((startHeight + i + ii) % vbCTUHeight >= vbPos - 3) ? pImg4 : pImg6;
        }
        else if ((startHeight + i + ii) % vbCTUHeight >= vbPos && ((startHeight + i + ii) % vbCTUHeight <= vbPos + (bChroma ? 1 : 3))) //bottom
        {
          pImg2 = ((startHeight + i + ii) % vbCTUHeight == vbPos    ) ? pImg0 : pImg2;
          pImg4 = ((startHeight + i + ii) % vbCTUHeight <= vbPos + 1) ? pImg2 : pImg4;
          pImg6 = ((startHeight + i + ii) % vbCTUHeight <= vbPos + 2) ? pImg4 : pImg6;

          pImg1 = ((startHeight + i + ii) % vbCTUHeight == vbPos    ) ? pImg0 : pImg1;
          pImg3 = ((startHeight + i + ii) % vbCTUHeight <= vbPos + 1) ? pImg1 : pImg3;
          pImg5 = ((startHeight + i + ii) % vbCTUHeight <= vbPos + 2) ? pImg3 : pImg5;
        }
        bool isNearVBabove = (startHeight + i + ii) % vbCTUHeight < vbPos && ((startHeight + i + ii) % vbCTUHeight >= vbPos - 1);
        bool isNearVBbelow = (startHeight + i + ii) % vbCTUHeight >= vbPos && ((startHeight + i + ii) % vbCTUHeight <= vbPos);
        for( int jj = 0; jj < clsSizeX; jj++ )
        {
          int sum = 0;
          const Pel curr = pImg0[+0];
          if( filtType == ALF_FILTER_7 )
          {
            sum += filterCoeff[0] * ( clipALF(filterClipp[0], curr, pImg5[+0], pImg6[+0]) );
            sum += filterCoeff[1] * ( clipALF(filterClipp[1], curr, pImg3[+1], pImg4[-1]) );
            sum += filterCoeff[2] * ( clipALF(filterClipp[2], curr, pImg3[+0], pImg4[+0]) );
            sum += filterCoeff[3] * ( clipALF(filterClipp[3], curr, pImg3[-1], pImg4[+1]) );
            sum += filterCoeff[4] * ( clipALF(filterClipp[4], curr, pImg1[+2], pImg2[-2]) );
            sum += filterCoeff[5] * ( clipALF(filterClipp[5], curr, pImg1[+1], pImg2[-1]) );
            sum += filterCoeff[6] * ( clipALF(filterClipp[6], curr, pImg1[+0], pImg2[+0]) );
            sum += filterCoeff[7] * ( clipALF(filterClipp[7], curr, pImg1[-1], pImg2[+1]) );
            sum += filterCoeff[8] * ( clipALF(filterClipp[8], curr, pImg1[-2], pImg2[+2]) );
            sum += filterCoeff[9] * ( clipALF(filterClipp[9], curr, pImg0[+3], pImg0[-3]) );
            sum += filterCoeff[10] * ( clipALF(filterClipp[10], curr, pImg0[+2], pImg0[-2]) );
            sum += filterCoeff[11] * ( clipALF(filterClipp[11], curr, pImg0[+1], pImg0[-1]) );
          }
          else
          {
            sum += filterCoeff[0] * ( clipALF(filterClipp[0], curr, pImg3[+0], pImg4[+0]) );
            sum += filterCoeff[1] * ( clipALF(filterClipp[1], curr, pImg1[+1], pImg2[-1]) );
            sum += filterCoeff[2] * ( clipALF(filterClipp[2], curr, pImg1[+0], pImg2[+0]) );
            sum += filterCoeff[3] * ( clipALF(filterClipp[3], curr, pImg1[-1], pImg2[+1]) );
            sum += filterCoeff[4] * ( clipALF(filterClipp[4], curr, pImg0[+2], pImg0[-2]) );
            sum += filterCoeff[5] * ( clipALF(filterClipp[5], curr, pImg0[+1], pImg0[-1]) );
          }
          if (!(isNearVBabove || isNearVBbelow))
          {
            sum = ( sum + offset ) >> shift;
          }
          else
          {
            sum = (sum + (1 << ((shift + 3) - 1))) >> (shift + 3);
          }
          sum += curr;
          pRec1[jj] = ClipPel( sum, clpRng );

          pImg0++;
          pImg1++;
          pImg2++;
          pImg3++;
          pImg4++;
          pImg5++;
          pImg6++;
        }
      }
    }

    pRec0     += dstStride2;
    pRec1     += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
    pImgYPad6 += srcStride2;
  }
}

void AdaptiveLoopFilter::filterBlkCcAlf( const PelBuf&      dstBuf,
                                         const CPelUnitBuf& recSrc,
                                         const Area&        blkDst,
                                         const Area&        blkSrc,
                                         const ComponentID  compId,
                                         const int16_t*     filterCoeff,
                                         const ClpRngs&     clpRngs,
                                         int                vbCTUHeight,
                                         int                vbPos )
{
  CHECK( 1 << getLog2(vbCTUHeight) != vbCTUHeight, "Not a power of 2");

  CHECK(!isChroma(compId), "Must be chroma");

  const int  clsSizeY      = 4;
  const int  clsSizeX      = 4;
  const int  startHeight   = blkDst.y;
  const int  endHeight     = blkDst.y + blkDst.height;
  const int  startWidth    = blkDst.x;
  const int  endWidth      = blkDst.x + blkDst.width;
  const auto nChromaFormat = recSrc.chromaFormat;
  const int  scaleX        = getComponentScaleX( compId, nChromaFormat );
  const int  scaleY        = getComponentScaleY( compId, nChromaFormat );

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  CPelBuf         srcBuf     = recSrc.get(COMPONENT_Y);
  const ptrdiff_t lumaStride = srcBuf.stride;
  const Pel *     lumaPtr    = srcBuf.buf + blkSrc.y * lumaStride + blkSrc.x;

  const ptrdiff_t chromaStride = dstBuf.stride;
  Pel *           chromaPtr    = dstBuf.buf + blkDst.y * chromaStride + blkDst.x;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    for( int j = 0; j < endWidth - startWidth; j += clsSizeX )
    {
      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        int row       = ii;
        int col       = j;
        Pel *srcSelf  = chromaPtr + col + row * chromaStride;

        ptrdiff_t offset1 = lumaStride;
        ptrdiff_t offset2 = -lumaStride;
        ptrdiff_t offset3 = 2 * lumaStride;
        row <<= scaleY;
        col <<= scaleX;
        const Pel *srcCross = lumaPtr + col + row * lumaStride;

        int pos = ((startHeight + i + ii) << scaleY) & (vbCTUHeight - 1);
        if (scaleY == 0 && (pos == vbPos || pos == vbPos + 1))
        {
          continue;
        }
        if (pos == (vbPos - 2) || pos == (vbPos + 1))
        {
          offset3 = offset1;
        }
        else if (pos == (vbPos - 1) || pos == vbPos)
        {
          offset1 = 0;
          offset2 = 0;
          offset3 = 0;
        }

        for (int jj = 0; jj < clsSizeX; jj++)
        {
          const int jj2     = (jj << scaleX);
          const int offset0 = 0;

          int sum = 0;
          const Pel currSrcCross = srcCross[offset0 + jj2];
          sum += filterCoeff[0] * (srcCross[offset2 + jj2    ] - currSrcCross);
          sum += filterCoeff[1] * (srcCross[offset0 + jj2 - 1] - currSrcCross);
          sum += filterCoeff[2] * (srcCross[offset0 + jj2 + 1] - currSrcCross);
          sum += filterCoeff[3] * (srcCross[offset1 + jj2 - 1] - currSrcCross);
          sum += filterCoeff[4] * (srcCross[offset1 + jj2    ] - currSrcCross);
          sum += filterCoeff[5] * (srcCross[offset1 + jj2 + 1] - currSrcCross);
          sum += filterCoeff[6] * (srcCross[offset3 + jj2    ] - currSrcCross);

          sum = (sum + ((1 << 7 ) >> 1)) >> 7; //m_scaleBits = 7
          const int offset = 1 << clpRngs.bd >> 1;
          sum = ClipPel(sum + offset, clpRngs) - offset;
          sum += srcSelf[jj];
          srcSelf[jj] = ClipPel(sum, clpRngs);
        }
      }
    }

    chromaPtr += chromaStride * clsSizeY;

    lumaPtr += lumaStride * clsSizeY << getComponentScaleY(compId, nChromaFormat);
  }
}

void AdaptiveLoopFilter::filterBlkCcAlfBoth( const PelBuf& dstBufCb, const PelBuf& dstBufCr, const CPelUnitBuf &recSrc, const Area &blkDst,
                                            const Area &blkSrc, const int16_t* filterCoeffCb, const int16_t* filterCoeffCr,
                                            const ClpRngs &clpRngs, int vbCTUHeight, int vbPos )
{
  CHECK(1 << getLog2(vbCTUHeight) != vbCTUHeight, "Not a power of 2");

  ChromaFormat nChromaFormat = recSrc.chromaFormat;
  const int clsSizeY = 4;
  const int clsSizeX = 4;
  const int startHeight = blkDst.y;
  const int endHeight = blkDst.y + blkDst.height;
  const int startWidth = blkDst.x;
  const int endWidth = blkDst.x + blkDst.width;
  const int scaleX = getComponentScaleX(COMPONENT_Cb, nChromaFormat);
  const int scaleY = getComponentScaleY(COMPONENT_Cb, nChromaFormat);

  CHECK(startHeight % clsSizeY, "Wrong startHeight in filtering");
  CHECK(startWidth % clsSizeX, "Wrong startWidth in filtering");
  CHECK((endHeight - startHeight) % clsSizeY, "Wrong endHeight in filtering");
  CHECK((endWidth - startWidth) % clsSizeX, "Wrong endWidth in filtering");

  CPelBuf srcBuf = recSrc.get(COMPONENT_Y);
  const ptrdiff_t lumaStride = srcBuf.stride;
  const Pel* lumaPtr = srcBuf.buf + blkSrc.y * lumaStride + blkSrc.x;

  const ptrdiff_t cbStride = dstBufCb.stride;
  const ptrdiff_t crStride = dstBufCr.stride;
  Pel* cbPtr = dstBufCb.buf + blkDst.y * cbStride + blkDst.x;
  Pel* crPtr = dstBufCr.buf + blkDst.y * crStride + blkDst.x;

  for (int i = 0; i < endHeight - startHeight; i += clsSizeY) {
    for (int j = 0; j < endWidth - startWidth; j += clsSizeX) {
      for (int ii = 0; ii < clsSizeY; ii++) {
        int row = ii;
        int col = j;
        Pel* srcSelfCb = cbPtr + col + row * cbStride;
        Pel* srcSelfCr = crPtr + col + row * crStride;
        
        ptrdiff_t offset1 = lumaStride;
        ptrdiff_t offset2 = -lumaStride;
        ptrdiff_t offset3 = 2 * lumaStride;
        row <<= scaleY;
        col <<= scaleX;
        const Pel* srcCross = lumaPtr + col + row * lumaStride;
        
        int pos = ((startHeight + i + ii) << scaleY) & (vbCTUHeight - 1);
        if (scaleY == 0 && (pos == vbPos || pos == vbPos + 1)) {
          continue;
        }
        if (pos == (vbPos - 2) || pos == (vbPos + 1)) {
          offset3 = offset1;
        } else if (pos == (vbPos - 1) || pos == vbPos) {
          offset1 = 0;
          offset2 = 0;
          offset3 = 0;
        }
        
        for (int jj = 0; jj < clsSizeX; jj++) {
          const int jj2 = (jj << scaleX);
          const int offset0 = 0;
          
          int sumCb = 0, sumCr = 0;
          const int currSrcCross = srcCross[offset0 + jj2];
          sumCb += filterCoeffCb[0] * (srcCross[offset2 + jj2] - currSrcCross);
          sumCb += filterCoeffCb[1] * (srcCross[offset0 + jj2 - 1] - currSrcCross);
          sumCb += filterCoeffCb[2] * (srcCross[offset0 + jj2 + 1] - currSrcCross);
          sumCb += filterCoeffCb[3] * (srcCross[offset1 + jj2 - 1] - currSrcCross);
          sumCb += filterCoeffCb[4] * (srcCross[offset1 + jj2] - currSrcCross);
          sumCb += filterCoeffCb[5] * (srcCross[offset1 + jj2 + 1] - currSrcCross);
          sumCb += filterCoeffCb[6] * (srcCross[offset3 + jj2] - currSrcCross);
          
          sumCr += filterCoeffCr[0] * (srcCross[offset2 + jj2] - currSrcCross);
          sumCr += filterCoeffCr[1] * (srcCross[offset0 + jj2 - 1] - currSrcCross);
          sumCr += filterCoeffCr[2] * (srcCross[offset0 + jj2 + 1] - currSrcCross);
          sumCr += filterCoeffCr[3] * (srcCross[offset1 + jj2 - 1] - currSrcCross);
          sumCr += filterCoeffCr[4] * (srcCross[offset1 + jj2] - currSrcCross);
          sumCr += filterCoeffCr[5] * (srcCross[offset1 + jj2 + 1] - currSrcCross);
          sumCr += filterCoeffCr[6] * (srcCross[offset3 + jj2] - currSrcCross);
          
          sumCb = (sumCb + ((1 << 7) >> 1)) >> 7;  // m_scaleBits = 7
          sumCr = (sumCr + ((1 << 7) >> 1)) >> 7;  // m_scaleBits = 7
          const int offset = 1 << clpRngs.bd >> 1;
          sumCb = ClipPel(sumCb + offset, clpRngs) - offset;
          sumCr = ClipPel(sumCr + offset, clpRngs) - offset;
          sumCb += srcSelfCb[jj];
          sumCr += srcSelfCr[jj];
          srcSelfCb[jj] = ClipPel(sumCb, clpRngs);
          srcSelfCr[jj] = ClipPel(sumCr, clpRngs);
        }
      }
    }

    cbPtr += cbStride * clsSizeY;
    crPtr += crStride * clsSizeY;

    lumaPtr += lumaStride * clsSizeY << getComponentScaleY(COMPONENT_Cb, nChromaFormat);
  }
}

}
