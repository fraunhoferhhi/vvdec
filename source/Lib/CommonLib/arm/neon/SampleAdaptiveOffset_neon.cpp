/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/** \file     SampleAdaptiveOffset_neon.cpp
    \brief    Neon sample adaptive offset
*/

#include "../CommonDefARM.h"
#include "CommonLib/SampleAdaptiveOffset.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_SAO

#include "tbl_neon.h"

namespace vvdec
{

static inline bool isHorProcessDisabled( int yPos, int numHorVirBndry, const int horVirBndryPos[] )
{
  CHECKD( numHorVirBndry >= 3, "Too many virtual boundaries" );

  for( int i = 0; i < numHorVirBndry; i++ )
  {
    if( yPos == horVirBndryPos[i] || yPos == horVirBndryPos[i] - 1 )
    {
      return true;
    }
  }
  return false;
}

static inline void setVerProcessDisabledMask( int width, int numVerVirBndry, const int verVirBndryPos[],
                                              uint16_t bndmask[MAX_CU_SIZE] )
{
  CHECKD( numVerVirBndry >= 3, "Too many virtual boundaries" );

  for( int i = 0; i < numVerVirBndry; i++ )
  {
    if( verVirBndryPos[i] >= 0 && verVirBndryPos[i] < width )
    {
      bndmask[verVirBndryPos[i]] = 0xffff; // Set mask to preserve original source sample.
    }

    if( verVirBndryPos[i] - 1 >= 0 && verVirBndryPos[i] - 1 < width )
    {
      bndmask[verVirBndryPos[i] - 1] = 0xffff; // Set mask to preserve original source sample.
    }
  }
}

static inline int8x16_t signDiff_neon( const int16x8_t lhs, const int16x8_t rhs )
{
  const uint16x8_t gt = vcgtq_s16( lhs, rhs );
  const uint16x8_t lt = vcltq_s16( lhs, rhs );

  return vsubq_s8( vreinterpretq_s8_u16( lt ), vreinterpretq_s8_u16( gt ) );
}

static inline int16x8_t clipPel_neon( const int16x8_t val, const int channelBitDepth )
{
  const int16x8_t vmin = vdupq_n_s16( 0 );
  const int16x8_t vmax = vdupq_n_s16( ( 1 << channelBitDepth ) - 1 );

  return vmaxq_s16( vminq_s16( val, vmax ), vmin );
}

static inline int16x8_t applyOffset_BO_neon( const int16x8_t src, const int8x8_t index, const int8x8_t offsetTbl,
                                             const int channelBitDepth )
{
  const int8x8_t offset = vtbl1_s8( offsetTbl, index );
  return clipPel_neon( vaddw_s8( src, offset ), channelBitDepth );
}

static inline int16x8_t applyOffset_EO_neon( const int16x8_t src, const int8x16_t index, const int8x16_t offsetTbl,
                                             const int channelBitDepth )
{
  const uint8x16_t index8 = vreinterpretq_u8_s8( index );
  const int16x8_t offset = vshrq_n_s16( vreinterpretq_s16_s8( vvdec_vqtbl1q_s8( offsetTbl, index8 ) ), 8 );
  return clipPel_neon( vaddq_s16( src, offset ), channelBitDepth );
}

static inline int8x8_t loadOffsetTable_BO_neon( const int32_t offset[MAX_NUM_SAO_CLASSES], int startIdx )
{
  CHECKD( startIdx < 0 || startIdx >= MAX_NUM_SAO_CLASSES, "Invalid start index" );

  static constexpr uint8_t offsetLut8[4][8] =
  {
    // Use 255 to fill the rest of the table as 0.
    {  0,  4, 8 , 12, 255, 255, 255, 255 },
    {  4,  8, 12, 16, 255, 255, 255, 255 },
    {  8, 12, 16, 20, 255, 255, 255, 255 },
    { 12, 16, 20, 24, 255, 255, 255, 255 },
  };

  int8x8_t offset0123;
  if( startIdx <= MAX_NUM_SAO_CLASSES - SAO_NUM_OFFSETS )
  {
    int8x16_t offsetTbl8 = vreinterpretq_s8_s32( vld1q_s32( offset + startIdx ) );
    offset0123 = vvdec_vqtbl1_s8( offsetTbl8, vld1_u8( offsetLut8[0] ) );
  }
  else
  {
    int8x16x2_t offsetTbl8;
    offsetTbl8.val[0] = vreinterpretq_s8_s32( vld1q_s32( offset + MAX_NUM_SAO_CLASSES - SAO_NUM_OFFSETS ) );
    offsetTbl8.val[1] = vreinterpretq_s8_s32( vld1q_s32( offset ) );

    // adjustIdx can only be 1, 2, 3.
    const unsigned adjustIdx = startIdx - ( MAX_NUM_SAO_CLASSES - SAO_NUM_OFFSETS );
    offset0123 = vvdec_vqtbl2_s8( offsetTbl8, vld1_u8( offsetLut8[adjustIdx] ) );
  }

  return offset0123;
}

static inline int8x16_t loadOffsetTable_EO_neon( const int32_t offset[5] )
{
  const int16x8_t offset0123 = vreinterpretq_s16_s32( vld1q_s32( offset ) );
  const int16x8_t offset4 = vreinterpretq_s16_s32( vsetq_lane_s32( offset[4], vdupq_n_s32( 0 ), 0 ) );

  const int16x8_t offset01234 = vuzpq_s16( offset0123, offset4 ).val[0];

  return vcombine_s8( vmovn_s16( offset01234 ), vdup_n_s8( 0 ) );
}

template<int ChannelBitDepth>
static void offsetBlock_BO_neon( const int* offset, int startIdx, const Pel* srcBlk, Pel* resBlk, ptrdiff_t srcStride,
                                 ptrdiff_t resStride, int width, int height )
{
  static constexpr int shiftBits = ChannelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
  static_assert( shiftBits >= 1 && shiftBits <= 8, "Invalid shift value" );
  const int8x8_t offsetTbl = loadOffsetTable_BO_neon( offset, startIdx );
  const int8x8_t base = vdup_n_s8( startIdx );
  const int8x8_t mask = vdup_n_s8( MAX_NUM_SAO_CLASSES - 1 );

  const Pel* srcLine = srcBlk;
  Pel* resLine = resBlk;

  do
  {
    int x = 0;
    do
    {
      const int16x8_t src = vld1q_s16( srcLine + x );
      int8x8_t band = vshrn_n_s16( src, shiftBits );
      band = vsub_s8( band, base );
      band = vand_s8( band, mask );

      int16x8_t res = applyOffset_BO_neon( src, band, offsetTbl, ChannelBitDepth );
      vst1q_s16( resLine + x, res );

      x += 8;
    } while( x != width );

    srcLine += srcStride;
    resLine += resStride;
  } while( --height != 0 );
}

template<int SaoType, bool ApplyVerBndryMask>
static void offsetBlock_EO_process_neon( const int channelBitDepth, const int* offset, const Pel* srcBlk, Pel* resBlk,
                                         ptrdiff_t srcStride, ptrdiff_t resStride, int width, int startY, int endY,
                                         bool hasHorBndry, const int horVirBndryPos[], int numHorVirBndry,
                                         const uint16_t bndmask[MAX_CU_SIZE] )
{
  static_assert( SaoType == SAO_TYPE_EO_0   || SaoType == SAO_TYPE_EO_90 ||
                 SaoType == SAO_TYPE_EO_135 || SaoType == SAO_TYPE_EO_45, "Unsupported SAO EO type" );

  static constexpr int prevRowOffset = SaoType == SAO_TYPE_EO_0   ?  0 : -1;
  static constexpr int prevColOffset = SaoType == SAO_TYPE_EO_0   ? -1 :
                                       SaoType == SAO_TYPE_EO_90  ?  0 :
                                       SaoType == SAO_TYPE_EO_135 ? -1 :  1;
  static constexpr int nextRowOffset = SaoType == SAO_TYPE_EO_0   ?  0 :  1;
  static constexpr int nextColOffset = SaoType == SAO_TYPE_EO_0   ?  1 :
                                       SaoType == SAO_TYPE_EO_90  ?  0 :
                                       SaoType == SAO_TYPE_EO_135 ?  1 : -1;

  const int8x16_t offsetTbl = loadOffsetTable_EO_neon( offset );
  const int8x16_t baseOffset = vdupq_n_s8( 2 );

  const Pel* srcLine = srcBlk + startY * srcStride;
  Pel* resLine = resBlk + startY * resStride;

  int y = startY;
  do
  {
    if( hasHorBndry && isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
    {
      srcLine += srcStride;
      resLine += resStride;
      continue;
    }

    const Pel* prevLine = srcLine + prevRowOffset * srcStride + prevColOffset;
    const Pel* nextLine = srcLine + nextRowOffset * srcStride + nextColOffset;

    int x = 0;
    do
    {
      const int16x8_t src = vld1q_s16( srcLine + x );
      const int16x8_t prev = vld1q_s16( prevLine + x );
      const int16x8_t next = vld1q_s16( nextLine + x );

      const int8x16_t signPrev = signDiff_neon( src, prev );
      const int8x16_t signNext = signDiff_neon( src, next );
      const int8x16_t edgeType = vaddq_s8( signPrev, signNext );
      // edgeType: -2, -1, 0, 1, 2 -> offsetIdx: 0, 1, 2, 3, 4
      const int8x16_t offsetIdx = vaddq_s8( baseOffset, edgeType );

      int16x8_t res = applyOffset_EO_neon( src, offsetIdx, offsetTbl, channelBitDepth );
      if( ApplyVerBndryMask )
      {
        const uint16x8_t mask = vld1q_u16( bndmask + x );
        res = vbslq_s16( mask, src, res );
      }
      vst1q_s16( resLine + x, res );

      x += 8;
    } while( x != width );

    srcLine += srcStride;
    resLine += resStride;
  } while( ++y != endY );
}

template<int SaoType>
static void offsetBlock_EO_neon( const int channelBitDepth, const int* offset, const Pel* srcBlk, Pel* resBlk,
                                 ptrdiff_t srcStride, ptrdiff_t resStride, int width, int startY, int endY,
                                 const int horVirBndryPos[], int numHorVirBndry, const int verVirBndryPos[],
                                 int numVerVirBndry )
{
  static_assert( SaoType == SAO_TYPE_EO_0   || SaoType == SAO_TYPE_EO_90 ||
                 SaoType == SAO_TYPE_EO_135 || SaoType == SAO_TYPE_EO_45, "Unsupported SAO EO type" );

  static constexpr bool checkHorBndry = SaoType != SAO_TYPE_EO_0;
  static constexpr bool checkVerBndry = SaoType != SAO_TYPE_EO_90;

  const bool hasHorBndry = checkHorBndry && numHorVirBndry > 0;
  const bool hasVerBndry = checkVerBndry && numVerVirBndry > 0;

  if( hasVerBndry )
  {
    uint16_t bndmask[MAX_CU_SIZE] = { 0 };
    setVerProcessDisabledMask( width, numVerVirBndry, verVirBndryPos, bndmask );

    offsetBlock_EO_process_neon<SaoType, true>( channelBitDepth, offset, srcBlk, resBlk, srcStride, resStride, width,
                                                startY, endY, hasHorBndry, horVirBndryPos, numHorVirBndry, bndmask );
  }
  else
  {
    offsetBlock_EO_process_neon<SaoType, false>( channelBitDepth, offset, srcBlk, resBlk, srcStride, resStride, width,
                                                 startY, endY, hasHorBndry, horVirBndryPos, numHorVirBndry, nullptr );
  }
}

void offsetBlock_neon( const int channelBitDepth, const ClpRng& clpRng, int typeIdx, int* offset, int startIdx,
                       const Pel* srcBlk, Pel* resBlk, ptrdiff_t srcStride, ptrdiff_t resStride, int width, int height,
                       bool isLeftAvail, bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail,
                       bool isAboveRightAvail, bool isBelowLeftAvail, bool isBelowRightAvail,
                       std::vector<int8_t>* m_signLineBuf1, std::vector<int8_t>* m_signLineBuf2,
                       bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[],
                       int numHorVirBndry, int numVerVirBndry )
{
  CHECKD( width <= 0 || ( width & 7 ), "Width must be positive and a multiple of 8" );
  CHECKD( width > MAX_CU_SIZE, "Width must not exceed MAX_CU_SIZE" );
  CHECKD( height <= 0, "Height must be positive" );

  if( typeIdx == SAO_TYPE_BO )
  {
    CHECKD( channelBitDepth != 8 && channelBitDepth != 10, "Unsupported channel bit depth" );

    switch( channelBitDepth )
    {
    case 8:
      offsetBlock_BO_neon<8>( offset, startIdx, srcBlk, resBlk, srcStride, resStride, width, height );
      break;
    case 10:
    default:
      offsetBlock_BO_neon<10>( offset, startIdx, srcBlk, resBlk, srcStride, resStride, width, height );
    }
    return;
  }

  CHECKD( isCtuCrossedByVirtualBoundaries != ( numHorVirBndry > 0 || numVerVirBndry > 0 ),
          "Virtual boundary flag is inconsistent with virtual boundary counts" );

  switch( typeIdx )
  {
  case SAO_TYPE_EO_0:
    if( !isLeftAvail || !isRightAvail )
    {
      break;
    }
    offsetBlock_EO_neon<SAO_TYPE_EO_0>( channelBitDepth, offset, srcBlk, resBlk, srcStride, resStride, width, 0, height,
                                        horVirBndryPos, numHorVirBndry, verVirBndryPos, numVerVirBndry );
    return;

  case SAO_TYPE_EO_45:
    CHECKD( isAboveRightAvail && ( !isAboveAvail || !isRightAvail ),
            "Above-right is available without above and right availability" );
    CHECKD( isBelowLeftAvail && ( !isBelowAvail || !isLeftAvail ),
            "Below-left is available without below and left availability" );
    if( !isAboveRightAvail || !isBelowLeftAvail )
    {
      break;
    }
    offsetBlock_EO_neon<SAO_TYPE_EO_45>( channelBitDepth, offset, srcBlk, resBlk, srcStride, resStride, width, 0,
                                         height, horVirBndryPos, numHorVirBndry, verVirBndryPos, numVerVirBndry );
    return;

  case SAO_TYPE_EO_90:
  {
    const int startY = isAboveAvail ? 0 : 1;
    const int endY = isBelowAvail ? height : height - 1;
    offsetBlock_EO_neon<SAO_TYPE_EO_90>( channelBitDepth, offset, srcBlk, resBlk, srcStride, resStride, width, startY,
                                         endY, horVirBndryPos, numHorVirBndry, verVirBndryPos, numVerVirBndry );
    return;
  }

  case SAO_TYPE_EO_135:
    CHECKD( isAboveLeftAvail && ( !isAboveAvail || !isLeftAvail ),
            "Above-left is available without above and left availability" );
    CHECKD( isBelowRightAvail && ( !isBelowAvail || !isRightAvail ),
            "Below-right is available without below and right availability" );
    if( !isAboveLeftAvail || !isBelowRightAvail )
    {
      break;
    }
    offsetBlock_EO_neon<SAO_TYPE_EO_135>( channelBitDepth, offset, srcBlk, resBlk, srcStride, resStride, width, 0,
                                          height, horVirBndryPos, numHorVirBndry, verVirBndryPos, numVerVirBndry );
    return;

  default:
    THROW_FATAL( "Not a supported SAO type\n" );
  }

  // Fall back to the scalar core implementation for cases the Neon path does not handle.
  SampleAdaptiveOffset::offsetBlock_core(
      channelBitDepth, clpRng, typeIdx, offset, startIdx, srcBlk, resBlk, srcStride, resStride, width, height,
      isLeftAvail, isRightAvail, isAboveAvail, isBelowAvail, isAboveLeftAvail, isAboveRightAvail, isBelowLeftAvail,
      isBelowRightAvail, m_signLineBuf1, m_signLineBuf2, isCtuCrossedByVirtualBoundaries, horVirBndryPos,
      verVirBndryPos, numHorVirBndry, numVerVirBndry );
}

template<>
void SampleAdaptiveOffset::_initSampleAdaptiveOffsetARM<NEON>()
{
  offsetBlock = offsetBlock_neon;
}

} // namespace vvdec

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_SAO
