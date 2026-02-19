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
#include <arm_neon.h>

#include "../AdaptiveLoopFilter.h"
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "mem_neon.h"
#include "sum_neon.h"
#include "tbl_neon.h"
#include "transpose_neon.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_ALF
namespace vvdec
{

constexpr int ALF_7x7_VB_FOLD_MIN_DIST = -2;
constexpr int ALF_7x7_VB_FOLD_MAX_DIST = 3;

constexpr int ALF_5x5_VB_FOLD_MIN_DIST = -1;
constexpr int ALF_5x5_VB_FOLD_MAX_DIST = 2;

typedef struct ALFGroupParam
{
  int16x8_t clipA, negClipA;
  int16x8_t clipB, negClipB;
  int16x4_t coeff;
} ALFGroupParam;

static inline int16x8_t clip3_neon( const int16x8_t val, const int16x8_t minVal, const int16x8_t maxVal )
{
  return vmaxq_s16( vminq_s16( val, maxVal ), minVal );
}

static inline void processALF_CoeffPair_neon( const Pel* ptr0, const Pel* ptr1, const Pel* ptr2, const Pel* ptr3,
                                              const int16x8_t curr, const int16x8_t clipA, const int16x8_t clipB,
                                              const int16x8_t negClipA, const int16x8_t negClipB, int16x8_t& accA,
                                              int16x8_t& accB )
{
  const int16x8_t v0 = vld1q_s16( ptr0 );
  const int16x8_t v1 = vld1q_s16( ptr1 );
  const int16x8_t v2 = vld1q_s16( ptr2 );
  const int16x8_t v3 = vld1q_s16( ptr3 );

  int16x8_t diff0 = vsubq_s16( v0, curr );
  int16x8_t diff1 = vsubq_s16( v1, curr );
  int16x8_t diff2 = vsubq_s16( v2, curr );
  int16x8_t diff3 = vsubq_s16( v3, curr );

  diff0 = clip3_neon( diff0, negClipA, clipA );
  diff1 = clip3_neon( diff1, negClipA, clipA );
  diff2 = clip3_neon( diff2, negClipB, clipB );
  diff3 = clip3_neon( diff3, negClipB, clipB );

  accA = vaddq_s16( diff0, diff1 );
  accB = vaddq_s16( diff2, diff3 );
}

template<bool isFoldingRequired>
static inline void processALF7x7Row_neon( const Pel* pImg0, Pel* pDst, const int distance, const ptrdiff_t srcStride,
                                          const ALFGroupParam param[6], const int clpRngMax )
{
  constexpr int SHIFT = AdaptiveLoopFilter::m_NUM_BITS - 1;

  const Pel* pImg1 = pImg0 + srcStride; // y+1
  const Pel* pImg2 = pImg0 - srcStride; // y-1
  const Pel* pImg3 = pImg1 + srcStride; // y+2
  const Pel* pImg4 = pImg2 - srcStride; // y-2
  const Pel* pImg5 = pImg3 + srcStride; // y+3
  const Pel* pImg6 = pImg4 - srcStride; // y-3

  if( isFoldingRequired )
  {
    // When the current line is near the VB (vbPos), some of rows access could point across the CTU boundary.
    // Distance = 0 or 1, reuse current row (pImg0).
    // Distance = 2 or -1, reuse previous folded row (pImg1/pImg2).
    // Distance = 3 or -2, reuse previous folded row (pImg3/pImg4).
    if( distance > 0 && distance <= ALF_7x7_VB_FOLD_MAX_DIST ) // Above.
    {
      pImg1 = distance == 1 ? pImg0 : pImg1;
      pImg3 = distance <= 2 ? pImg1 : pImg3;
      pImg5 = distance <= 3 ? pImg3 : pImg5;

      pImg2 = distance == 1 ? pImg0 : pImg2;
      pImg4 = distance <= 2 ? pImg2 : pImg4;
      pImg6 = distance <= 3 ? pImg4 : pImg6;
    }
    else if( distance <= 0 && distance >= ALF_7x7_VB_FOLD_MIN_DIST ) // Bottom.
    {
      pImg1 = distance == 0 ? pImg0 : pImg1;
      pImg3 = distance >= -1 ? pImg1 : pImg3;
      pImg5 = distance >= -2 ? pImg3 : pImg5;

      pImg2 = distance == 0 ? pImg0 : pImg2;
      pImg4 = distance >= -1 ? pImg2 : pImg4;
      pImg6 = distance >= -2 ? pImg4 : pImg6;
    }
  }

  const int16x8_t curr = vld1q_s16( pImg0 );

  int16x8_t a, b;
  processALF_CoeffPair_neon( pImg5 + 0, pImg6 + 0, pImg3 + 1, pImg4 - 1, curr, param[0].clipA, param[0].clipB,
                             param[0].negClipA, param[0].negClipB, a, b );
  int32x4_t accLo = vmull_lane_s16( vget_low_s16( a ), param[0].coeff, 0 );
  int32x4_t accHi = vmull_lane_s16( vget_high_s16( a ), param[1].coeff, 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b ), param[0].coeff, 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b ), param[1].coeff, 1 );

  processALF_CoeffPair_neon( pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1, curr, param[1].clipA, param[1].clipB,
                             param[1].negClipA, param[1].negClipB, a, b );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( a ), param[0].coeff, 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a ), param[1].coeff, 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b ), param[0].coeff, 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b ), param[1].coeff, 3 );

  processALF_CoeffPair_neon( pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1, curr, param[2].clipA, param[2].clipB,
                             param[2].negClipA, param[2].negClipB, a, b );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( a ), param[2].coeff, 0 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a ), param[3].coeff, 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b ), param[2].coeff, 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b ), param[3].coeff, 1 );

  processALF_CoeffPair_neon( pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1, curr, param[3].clipA, param[3].clipB,
                             param[3].negClipA, param[3].negClipB, a, b );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( a ), param[2].coeff, 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a ), param[3].coeff, 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b ), param[2].coeff, 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b ), param[3].coeff, 3 );

  processALF_CoeffPair_neon( pImg1 - 2, pImg2 + 2, pImg0 + 3, pImg0 - 3, curr, param[4].clipA, param[4].clipB,
                             param[4].negClipA, param[4].negClipB, a, b );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( a ), param[4].coeff, 0 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a ), param[5].coeff, 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b ), param[4].coeff, 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b ), param[5].coeff, 1 );

  processALF_CoeffPair_neon( pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1, curr, param[5].clipA, param[5].clipB,
                             param[5].negClipA, param[5].negClipB, a, b );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( a ), param[4].coeff, 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a ), param[5].coeff, 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b ), param[4].coeff, 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b ), param[5].coeff, 3 );

  int16x8_t acc;
  if( isFoldingRequired && distance >= 0 && distance <= 1 )
  {
    // Weaker filter, closer to VB.
    acc = vcombine_s16( vrshrn_n_s32( accLo, SHIFT + 3 ), vrshrn_n_s32( accHi, SHIFT + 3 ) );
  }
  else
  {
    // Regular filter strength.
    acc = vcombine_s16( vrshrn_n_s32( accLo, SHIFT ), vrshrn_n_s32( accHi, SHIFT ) );
  }

  acc = vqaddq_s16( acc, curr );
  int16x8_t dst = clip3_neon( acc, vdupq_n_s16( 0 ), vdupq_n_s16( clpRngMax ) );

  vst1q_s16( pDst, dst );
}

void Filter7x7Blk_neon( const AlfClassifier* classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc,
                        const Area& blk, const ComponentID compId, const short* filterSet, const short* fClipSet,
                        const ClpRng& clpRng, int vbCTUHeight, int vbPos )
{
  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const ptrdiff_t srcStride = srcLuma.stride;
  const ptrdiff_t dstStride = dstLuma.stride;

  const int startHeight = blk.y;
  const int width = blk.width;
  const int height = blk.height;

  const Pel* src = srcLuma.buf + blk.y * srcStride + blk.x;
  Pel* dst = dstLuma.buf + blk.y * dstStride + blk.x;

  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 4;

  CHECKD( startHeight % STEP_Y, "Wrong startHeight in filtering!" );
  CHECKD( width % STEP_X, "Width must be multiple of 8!" );
  CHECKD( height % STEP_Y, "Height must be multiple of 4!" );

  const int clpRngMax = clpRng.max();

  for( int i = 0; i < height; i += STEP_Y )
  {
    int yVbPos = ( startHeight + i ) & ( vbCTUHeight - 1 ); // Row’s position inside its CTU.

    auto calculateNextVbPosDist = [&yVbPos, vbPos, vbCTUHeight]() -> int
    {
      int distance = vbPos - yVbPos;
      if( ++yVbPos == vbCTUHeight )
      {
        yVbPos = 0;
      }
      return distance;
    };

    int VbDistance[STEP_Y];
    VbDistance[0] = calculateNextVbPosDist();
    VbDistance[1] = calculateNextVbPosDist();
    VbDistance[2] = calculateNextVbPosDist();
    VbDistance[3] = calculateNextVbPosDist();

    const bool foldingRequired =
        ( VbDistance[0] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[0] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[1] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[1] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[2] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[2] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[3] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[3] <= ALF_7x7_VB_FOLD_MAX_DIST );

    int cl_index = ( i / 4 ) * ( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE / 4 );
    const Pel* pImg0 = src;
    Pel* pDst = dst;

    int j = width;
    do
    {
      const AlfClassifier& cl0 = classifier[cl_index];
      const int index0 =
          cl0.classIdx * MAX_NUM_ALF_LUMA_COEFF + cl0.transposeIdx * MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES;
      const AlfClassifier& cl1 = classifier[cl_index + 1];
      const int index1 =
          cl1.classIdx * MAX_NUM_ALF_LUMA_COEFF + cl1.transposeIdx * MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES;

      const short* filterCoeff0 = filterSet + index0;
      const short* filterClip0 = fClipSet + index0;
      const short* filterCoeff1 = filterSet + index1;
      const short* filterClip1 = fClipSet + index1;

      ALFGroupParam param[MAX_NUM_ALF_LUMA_COEFF / 2];

      // Copy all 12 coeffs and clips.
      const int16x8_t cl0_04 = vld1q_s16( filterClip0 + 0 );
      const int16x8_t cl1_04 = vld1q_s16( filterClip1 + 0 );
      const int16x4_t cl0_8 = vld1_s16( filterClip0 + 8 );
      const int16x4_t cl1_8 = vld1_s16( filterClip1 + 8 );

      const int16x4_t cl0_0 = vget_low_s16( cl0_04 );
      const int16x4_t cl0_4 = vget_high_s16( cl0_04 );
      const int16x4_t cl1_0 = vget_low_s16( cl1_04 );
      const int16x4_t cl1_4 = vget_high_s16( cl1_04 );

      param[0].clipA = vcombine_s16( vdup_lane_s16( cl0_0, 0 ), vdup_lane_s16( cl1_0, 0 ) );
      param[0].clipB = vcombine_s16( vdup_lane_s16( cl0_0, 1 ), vdup_lane_s16( cl1_0, 1 ) );
      param[1].clipA = vcombine_s16( vdup_lane_s16( cl0_0, 2 ), vdup_lane_s16( cl1_0, 2 ) );
      param[1].clipB = vcombine_s16( vdup_lane_s16( cl0_0, 3 ), vdup_lane_s16( cl1_0, 3 ) );
      param[2].clipA = vcombine_s16( vdup_lane_s16( cl0_4, 0 ), vdup_lane_s16( cl1_4, 0 ) );
      param[2].clipB = vcombine_s16( vdup_lane_s16( cl0_4, 1 ), vdup_lane_s16( cl1_4, 1 ) );
      param[3].clipA = vcombine_s16( vdup_lane_s16( cl0_4, 2 ), vdup_lane_s16( cl1_4, 2 ) );
      param[3].clipB = vcombine_s16( vdup_lane_s16( cl0_4, 3 ), vdup_lane_s16( cl1_4, 3 ) );
      param[4].clipA = vcombine_s16( vdup_lane_s16( cl0_8, 0 ), vdup_lane_s16( cl1_8, 0 ) );
      param[4].clipB = vcombine_s16( vdup_lane_s16( cl0_8, 1 ), vdup_lane_s16( cl1_8, 1 ) );
      param[5].clipA = vcombine_s16( vdup_lane_s16( cl0_8, 2 ), vdup_lane_s16( cl1_8, 2 ) );
      param[5].clipB = vcombine_s16( vdup_lane_s16( cl0_8, 3 ), vdup_lane_s16( cl1_8, 3 ) );

      param[0].negClipA = vnegq_s16( param[0].clipA );
      param[0].negClipB = vnegq_s16( param[0].clipB );
      param[1].negClipA = vnegq_s16( param[1].clipA );
      param[1].negClipB = vnegq_s16( param[1].clipB );
      param[2].negClipA = vnegq_s16( param[2].clipA );
      param[2].negClipB = vnegq_s16( param[2].clipB );
      param[3].negClipA = vnegq_s16( param[3].clipA );
      param[3].negClipB = vnegq_s16( param[3].clipB );
      param[4].negClipA = vnegq_s16( param[4].clipA );
      param[4].negClipB = vnegq_s16( param[4].clipB );
      param[5].negClipA = vnegq_s16( param[5].clipA );
      param[5].negClipB = vnegq_s16( param[5].clipB );

      const int16x8_t c0_04 = vld1q_s16( filterCoeff0 + 0 );
      const int16x8_t c1_04 = vld1q_s16( filterCoeff1 + 0 );
      param[0].coeff = vget_low_s16( c0_04 );
      param[2].coeff = vget_high_s16( c0_04 );
      param[1].coeff = vget_low_s16( c1_04 );
      param[3].coeff = vget_high_s16( c1_04 );
      param[4].coeff = vld1_s16( filterCoeff0 + 8 );
      param[5].coeff = vld1_s16( filterCoeff1 + 8 );

      if( foldingRequired )
      {
        for( int k = 0; k < STEP_Y; k++ )
        {
          processALF7x7Row_neon<true>( pImg0 + k * srcStride, pDst + k * dstStride, VbDistance[k], srcStride, param,
                                       clpRngMax );
        }
      }
      else
      {
        for( int k = 0; k < STEP_Y; k++ )
        {
          processALF7x7Row_neon<false>( pImg0 + k * srcStride, pDst + k * dstStride, 0, srcStride, param, clpRngMax );
        }
      }

      j -= STEP_X;
      cl_index += 2;
      pImg0 += STEP_X;
      pDst += STEP_X;
    } while( j != 0 );

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
  }
}

static inline int16x8_t processALF_5x5_NearVB_neon( const Pel* pImg, const short* coeff, const short* clip,
                                                    const int clpRngMax )
{
  constexpr int SHIFT = AdaptiveLoopFilter::m_NUM_BITS - 1 + 3; // Weaker filter near VB.

  const int16x8_t curr = vld1q_s16( pImg + 0 );
  const int16x8_t v0p1 = vld1q_s16( pImg + 1 );
  const int16x8_t v0m1 = vld1q_s16( pImg - 1 );
  const int16x8_t v0p2 = vld1q_s16( pImg + 2 );
  const int16x8_t v0m2 = vld1q_s16( pImg - 2 );

  const int16x8_t diff_v0p1 = vsubq_s16( v0p1, curr );
  const int16x8_t diff_v0m1 = vsubq_s16( v0m1, curr );
  const int16x8_t diff_v0p2 = vsubq_s16( v0p2, curr );
  const int16x8_t diff_v0m2 = vsubq_s16( v0m2, curr );

  const int16x8_t diff_v0p1_c1 = clip3_neon( diff_v0p1, vdupq_n_s16( -clip[1] ), vdupq_n_s16( clip[1] ) );
  const int16x8_t diff_v0m1_c1 = clip3_neon( diff_v0m1, vdupq_n_s16( -clip[1] ), vdupq_n_s16( clip[1] ) );
  const int16x8_t diff_v0m1_c3 = clip3_neon( diff_v0m1, vdupq_n_s16( -clip[3] ), vdupq_n_s16( clip[3] ) );
  const int16x8_t diff_v0p1_c3 = clip3_neon( diff_v0p1, vdupq_n_s16( -clip[3] ), vdupq_n_s16( clip[3] ) );

  const int16x8_t acc1 = vaddq_s16( diff_v0p1_c1, diff_v0m1_c1 );
  const int16x8_t acc3 = vaddq_s16( diff_v0m1_c3, diff_v0p1_c3 );

  int32x4_t accLo = vmull_n_s16( vget_low_s16( acc1 ), coeff[1] );
  int32x4_t accHi = vmull_n_s16( vget_high_s16( acc1 ), coeff[1] );
  accLo = vmlal_n_s16( accLo, vget_low_s16( acc3 ), coeff[3] );
  accHi = vmlal_n_s16( accHi, vget_high_s16( acc3 ), coeff[3] );

  const int16x8_t diff_v0p2_c4 = clip3_neon( diff_v0p2, vdupq_n_s16( -clip[4] ), vdupq_n_s16( clip[4] ) );
  const int16x8_t diff_v0m2_c4 = clip3_neon( diff_v0m2, vdupq_n_s16( -clip[4] ), vdupq_n_s16( clip[4] ) );
  const int16x8_t diff_v0p1_c5 = clip3_neon( diff_v0p1, vdupq_n_s16( -clip[5] ), vdupq_n_s16( clip[5] ) );
  const int16x8_t diff_v0m1_c5 = clip3_neon( diff_v0m1, vdupq_n_s16( -clip[5] ), vdupq_n_s16( clip[5] ) );

  const int16x8_t acc4 = vaddq_s16( diff_v0p2_c4, diff_v0m2_c4 );
  const int16x8_t acc5 = vaddq_s16( diff_v0p1_c5, diff_v0m1_c5 );

  accLo = vmlal_n_s16( accLo, vget_low_s16( acc4 ), coeff[4] );
  accHi = vmlal_n_s16( accHi, vget_high_s16( acc4 ), coeff[4] );
  accLo = vmlal_n_s16( accLo, vget_low_s16( acc5 ), coeff[5] );
  accHi = vmlal_n_s16( accHi, vget_high_s16( acc5 ), coeff[5] );

  int16x8_t acc = vcombine_s16( vrshrn_n_s32( accLo, SHIFT ), vrshrn_n_s32( accHi, SHIFT ) );
  acc = vqaddq_s16( acc, curr );
  return clip3_neon( acc, vdupq_n_s16( 0 ), vdupq_n_s16( clpRngMax ) );
}

template<bool isFoldingRequired>
static inline int16x8_t processALF5x5Row_neon( const Pel* pImg0, const int distance, const ptrdiff_t srcStride,
                                               const short* coeff, const short* clip, const int clpRngMax )
{
  constexpr int SHIFT = AdaptiveLoopFilter::m_NUM_BITS - 1;

  const Pel* pImg1 = pImg0 + srcStride; // y+1
  const Pel* pImg2 = pImg0 - srcStride; // y-1
  const Pel* pImg3 = pImg1 + srcStride; // y+2
  const Pel* pImg4 = pImg2 - srcStride; // y-2

  if( isFoldingRequired )
  {
    // When the current line is near the VB (vbPos), some of rows access could point across the CTU boundary.
    // Distance = 0 or 1, reuse current row (pImg0).
    // Distance = 2 or -1, reuse previous folded row (pImg1/pImg2).
    if( distance == 0 || distance == 1 )
    {
      // All point to the same row.
      return processALF_5x5_NearVB_neon( pImg0, coeff, clip, clpRngMax );
    }

    if( distance == ALF_5x5_VB_FOLD_MIN_DIST || distance == ALF_5x5_VB_FOLD_MAX_DIST )
    {
      pImg3 = pImg1;
      pImg4 = pImg2;
    }
  }

  const int16x8_t curr = vld1q_s16( pImg0 );

  int16x8_t a, b;
  processALF_CoeffPair_neon( pImg3 + 0, pImg4 + 0, pImg1 + 1, pImg2 - 1, curr, vdupq_n_s16( clip[0] ),
                             vdupq_n_s16( clip[1] ), vdupq_n_s16( -clip[0] ), vdupq_n_s16( -clip[1] ), a, b );
  int32x4_t accLo = vmull_n_s16( vget_low_s16( a ), coeff[0] );
  int32x4_t accHi = vmull_n_s16( vget_high_s16( a ), coeff[0] );
  accLo = vmlal_n_s16( accLo, vget_low_s16( b ), coeff[1] );
  accHi = vmlal_n_s16( accHi, vget_high_s16( b ), coeff[1] );

  processALF_CoeffPair_neon( pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1, curr, vdupq_n_s16( clip[2] ),
                             vdupq_n_s16( clip[3] ), vdupq_n_s16( -clip[2] ), vdupq_n_s16( -clip[3] ), a, b );
  accLo = vmlal_n_s16( accLo, vget_low_s16( a ), coeff[2] );
  accHi = vmlal_n_s16( accHi, vget_high_s16( a ), coeff[2] );
  accLo = vmlal_n_s16( accLo, vget_low_s16( b ), coeff[3] );
  accHi = vmlal_n_s16( accHi, vget_high_s16( b ), coeff[3] );

  processALF_CoeffPair_neon( pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1, curr, vdupq_n_s16( clip[4] ),
                             vdupq_n_s16( clip[5] ), vdupq_n_s16( -clip[4] ), vdupq_n_s16( -clip[5] ), a, b );
  accLo = vmlal_n_s16( accLo, vget_low_s16( a ), coeff[4] );
  accHi = vmlal_n_s16( accHi, vget_high_s16( a ), coeff[4] );
  accLo = vmlal_n_s16( accLo, vget_low_s16( b ), coeff[5] );
  accHi = vmlal_n_s16( accHi, vget_high_s16( b ), coeff[5] );

  int16x8_t acc = vcombine_s16( vrshrn_n_s32( accLo, SHIFT ), vrshrn_n_s32( accHi, SHIFT ) );
  acc = vqaddq_s16( acc, curr );
  return clip3_neon( acc, vdupq_n_s16( 0 ), vdupq_n_s16( clpRngMax ) );
}

void Filter5x5Blk_neon( const AlfClassifier*, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc, const Area& blk,
                        const ComponentID compId, const short* filterSet, const short* fClipSet, const ClpRng& clpRng,
                        int vbCTUHeight, int vbPos )
{
  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const ptrdiff_t srcStride = srcLuma.stride;
  const ptrdiff_t dstStride = dstLuma.stride;

  const int startHeight = blk.y;
  const int width = blk.width;
  const int height = blk.height;

  const Pel* src = srcLuma.buf + blk.y * srcStride + blk.x;
  Pel* dst = dstLuma.buf + blk.y * dstStride + blk.x;

  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 4;

  CHECKD( !isChroma( compId ), "ALF 5x5 filter is for chroma only!" );
  CHECKD( startHeight % STEP_Y, "Wrong startHeight in filtering!" );
  CHECKD( width % 4, "Width must be multiple of 4!" );
  CHECKD( height % 4, "Height must be multiple of 4!" );

  const int clpRngMax = clpRng.max();

  for( int i = 0; i < height; i += STEP_Y )
  {
    int yVbPos = ( startHeight + i ) & ( vbCTUHeight - 1 ); // Row’s position inside its CTU.

    auto calculateNextVbPosDist = [&yVbPos, vbPos, vbCTUHeight]() -> int
    {
      int distance = vbPos - yVbPos;
      if( ++yVbPos == vbCTUHeight )
      {
        yVbPos = 0;
      }
      return distance;
    };

    int VbDistance[STEP_Y];
    VbDistance[0] = calculateNextVbPosDist();
    VbDistance[1] = calculateNextVbPosDist();
    VbDistance[2] = calculateNextVbPosDist();
    VbDistance[3] = calculateNextVbPosDist();
    const bool foldingRequired =
        ( VbDistance[0] >= ALF_5x5_VB_FOLD_MIN_DIST && VbDistance[0] <= ALF_5x5_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[1] >= ALF_5x5_VB_FOLD_MIN_DIST && VbDistance[1] <= ALF_5x5_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[2] >= ALF_5x5_VB_FOLD_MIN_DIST && VbDistance[2] <= ALF_5x5_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[3] >= ALF_5x5_VB_FOLD_MIN_DIST && VbDistance[3] <= ALF_5x5_VB_FOLD_MAX_DIST );

    const Pel* pImg0 = src;
    Pel* pDst = dst;

    for( int j = 0; j < ( width & ~7 ); j += STEP_X )
    {
      int16x8_t dst0, dst1, dst2, dst3;

      if( foldingRequired )
      {
        dst0 = processALF5x5Row_neon<true>( pImg0 + 0 * srcStride, VbDistance[0], srcStride, filterSet, fClipSet,
                                            clpRngMax );
        dst1 = processALF5x5Row_neon<true>( pImg0 + 1 * srcStride, VbDistance[1], srcStride, filterSet, fClipSet,
                                            clpRngMax );
        dst2 = processALF5x5Row_neon<true>( pImg0 + 2 * srcStride, VbDistance[2], srcStride, filterSet, fClipSet,
                                            clpRngMax );
        dst3 = processALF5x5Row_neon<true>( pImg0 + 3 * srcStride, VbDistance[3], srcStride, filterSet, fClipSet,
                                            clpRngMax );
      }
      else
      {
        dst0 = processALF5x5Row_neon<false>( pImg0 + 0 * srcStride, 0, srcStride, filterSet, fClipSet, clpRngMax );
        dst1 = processALF5x5Row_neon<false>( pImg0 + 1 * srcStride, 0, srcStride, filterSet, fClipSet, clpRngMax );
        dst2 = processALF5x5Row_neon<false>( pImg0 + 2 * srcStride, 0, srcStride, filterSet, fClipSet, clpRngMax );
        dst3 = processALF5x5Row_neon<false>( pImg0 + 3 * srcStride, 0, srcStride, filterSet, fClipSet, clpRngMax );
      }

      vst1q_s16( pDst + 0 * dstStride, dst0 );
      vst1q_s16( pDst + 1 * dstStride, dst1 );
      vst1q_s16( pDst + 2 * dstStride, dst2 );
      vst1q_s16( pDst + 3 * dstStride, dst3 );

      pImg0 += STEP_X;
      pDst += STEP_X;
    }

    if( width & 7 )
    {
      int16x8_t dst0, dst1, dst2, dst3;

      if( foldingRequired )
      {
        dst0 = processALF5x5Row_neon<true>( pImg0 + 0 * srcStride, VbDistance[0], srcStride, filterSet, fClipSet,
                                            clpRngMax );
        dst1 = processALF5x5Row_neon<true>( pImg0 + 1 * srcStride, VbDistance[1], srcStride, filterSet, fClipSet,
                                            clpRngMax );
        dst2 = processALF5x5Row_neon<true>( pImg0 + 2 * srcStride, VbDistance[2], srcStride, filterSet, fClipSet,
                                            clpRngMax );
        dst3 = processALF5x5Row_neon<true>( pImg0 + 3 * srcStride, VbDistance[3], srcStride, filterSet, fClipSet,
                                            clpRngMax );
      }
      else
      {
        dst0 = processALF5x5Row_neon<false>( pImg0 + 0 * srcStride, 0, srcStride, filterSet, fClipSet, clpRngMax );
        dst1 = processALF5x5Row_neon<false>( pImg0 + 1 * srcStride, 0, srcStride, filterSet, fClipSet, clpRngMax );
        dst2 = processALF5x5Row_neon<false>( pImg0 + 2 * srcStride, 0, srcStride, filterSet, fClipSet, clpRngMax );
        dst3 = processALF5x5Row_neon<false>( pImg0 + 3 * srcStride, 0, srcStride, filterSet, fClipSet, clpRngMax );
      }

      vst1_s16( pDst + 0 * dstStride, vget_low_s16( dst0 ) );
      vst1_s16( pDst + 1 * dstStride, vget_low_s16( dst1 ) );
      vst1_s16( pDst + 2 * dstStride, vget_low_s16( dst2 ) );
      vst1_s16( pDst + 3 * dstStride, vget_low_s16( dst3 ) );
    }

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
  }
}

void DeriveClassificationBlk_neon( AlfClassifier* classifier, const CPelBuf& srcLuma, const Area& blk, const int shift,
                                   int vbCTUHeight, int vbPos )
{
  static_assert( sizeof( AlfClassifier ) == 2, "ALFClassifier type must be 16 bits wide!" );
  CHECKD( blk.width % 8, "Width must be multiple of 8!" );
  CHECKD( blk.height % 4, "Height must be multiple of 4!" );

  const ptrdiff_t stride = srcLuma.stride;
  static constexpr int maxActivity = 15;

  const int heightExtended = ( blk.height + 4 ) >> 1;
  const int widthExtended = blk.width + 4;

  static constexpr uint16_t mask[] = { 0xFFFFu, 0, 0xFFFFu, 0, 0xFFFFu, 0, 0xFFFFu, 0 };
  const uint16x8_t even_odd_mask = vld1q_u16( mask );

  // widthExtended is multiple of 4, when unrolling by 8,
  // laplace buffer is long enough to hold widthExtended+4 pixels.
  uint16_t laplacian[( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4 ) >> 1]
                    [AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 8];

  int pos = blk.pos().y - 2;

  const Pel* src = srcLuma.buf + ( blk.y - 3 ) * stride + blk.x - 3;

  // First pass - compute the laplacian values for each direction and store in laplacian[][].
  int i = 0;
  do
  {
    const Pel* src0 = src + 0 * stride; // row - 1
    const Pel* src1 = src + 1 * stride; // row + 0
    const Pel* src2 = src + 2 * stride; // row + 1
    const Pel* src3 = src + 3 * stride; // row + 2
    uint16_t* dst = &laplacian[i][0];

    // When close to a vertical boundary between CTUs, must not read across that boundary.
    // Instead, clamps the reference rows.
    if( pos > 0 )
    {
      int posInCTU = pos & ( vbCTUHeight - 1 );
      int distance = vbPos - posInCTU;

      src0 = distance == 0 ? src1 : src0;
      src3 = distance == 2 ? src2 : src3;
    }
    uint16x8_t prev = vdupq_n_u16( 0 );

    int j = 0;
    do
    {
      int16x8_t x0 = vld1q_s16( src0 ); // [ y0p0, y0p1, y0p2, y0p3, y0p4, y0p5, y0p6, y0p7 ]
      int16x8_t x1 = vld1q_s16( src1 ); // [ y1p0, y1p1, y1p2, y1p3, y1p4, y1p5, y1p6, y1p7 ]
      int16x8_t x2 = vld1q_s16( src2 ); // [ y2p0, y2p1, y2p2, y2p3, y2p4, y2p5, y2p6, y2p7 ]
      int16x8_t x3 = vld1q_s16( src3 ); // [ y3p0, y3p1, y3p2, y3p3, y3p4, y3p5, y3p6, y3p7 ]

      int16x8_t evenR0_oddR1 = vbslq_s16( even_odd_mask, x0, x1 ); // [ y0p0, y1p1, y0p2, y1p3, y0p4, y1p5, y0p6, y1p7 ]
      int16x8_t evenR1_oddR2 = vbslq_s16( even_odd_mask, x1, x2 ); // [ y1p0, y2p1, y1p2, y2p3, y1p4, y2p5, y1p6, y2p7 ]
      int16x8_t evenR2_oddR3 = vbslq_s16( even_odd_mask, x2, x3 ); // [ y2p0, y3p1, y2p2, y3p3, y2p4, y3p5, y2p6, y3p7 ]

      int16x8_t x4 = vld1q_s16( src0 + 2 ); // [ y0p2, y0p3, y0p4, y0p5, y0p6, y0p7, y0p8, y0p9 ]
      int16x8_t x5 = vld1q_s16( src1 + 2 ); // [ y1p2, y1p3, y1p4, y1p5, y1p6, y1p7, y1p8, y1p9 ]
      int16x8_t x6 = vld1q_s16( src2 + 2 ); // [ y2p2, y2p3, y2p4, y2p5, y2p6, y2p7, y2p8, y2p9 ]
      int16x8_t x7 = vld1q_s16( src3 + 2 ); // [ y3p2, y3p3, y3p4, y3p5, y3p6, y3p7, y3p8, y3p9 ]

      int16x8_t evenR4_oddR5 = vbslq_s16( even_odd_mask, x4, x5 ); // [ y0p2, y1p3, y0p4, y1p5, y0p6, y1p7, y0p8, y1p9 ]
      int16x8_t evenR5_oddR6 = vbslq_s16( even_odd_mask, x5, x6 ); // [ y1p2, y2p3, y1p4, y2p5, y1p6, y2p7, y1p8, y2p9 ]
      int16x8_t evenR6_oddR7 = vbslq_s16( even_odd_mask, x6, x7 ); // [ y2p2, y3p3, y2p4, y3p5, y2p6, y3p7, y2p8, y3p9 ]
      int16x8_t evenR5_oddR0 = vbslq_s16( even_odd_mask, x5, x0 ); // [ y1p2, y0p1, y1p4, y0p3, y1p6, y0p5, y1p8, y0p7 ]
      int16x8_t evenR7_oddR2 = vbslq_s16( even_odd_mask, x7, x2 ); // [ y3p2, y2p1, y3p4, y2p3, y3p6, y2p5, y3p8, y2p7 ]
      int16x8_t center = vbslq_s16( even_odd_mask, x6, x1 );       // [ y2p2, y1p1, y2p4, y1p3, y2p6, y1p5, y2p8, y1p7 ]

      int16x8_t c = vshlq_n_s16( center, 1 );
      int16x8_t d = vrev32q_s16( c );

      int16x8_t r5r0_r7r2 = vaddq_s16( evenR5_oddR0, evenR7_oddR2 );
      int16x8_t r1r2_r5r6 = vaddq_s16( evenR1_oddR2, evenR5_oddR6 );
      int16x8_t r0r1_r6r7 = vaddq_s16( evenR0_oddR1, evenR6_oddR7 );
      int16x8_t r4r5_r2r3 = vaddq_s16( evenR4_oddR5, evenR2_oddR3 );

      // Calculate |2 * center − neighbor1 − neighbor2| (for two locations) summed.
      // Vertical   : |2*center - (up + down)|
      // Horizontal : |2*center - (left + right)|
      // Diag0      : |2*center - (up-left + down-right)|
      // Diag1      : |2*center - (up-right + down-left)|

      // ver = [ abs(2*y2p2 - (y1p2 + y3p2)), abs(2*y1p1 - (y0p1 + y2p1)), => Lane 0+1 corresponds to vert at j=0
      //         abs(2*y2p4 - (y1p4 + y3p4)), abs(2*y2p4 - (y1p4 + y3p4)), => Lane 2+3 corresponds to vert at j=2
      //         abs(2*y2p6 - (y1p6 + y3p6)), abs(2*y1p5 - (y0p5 + y2p5)), => Lane 4+5 corresponds to vert at j=4
      //         abs(2*y2p8 - (y1p8 + y3p8)), abs(2*y1p7 - (y0p7 + y2p7)) ]=> Lane 6+7 corresponds to vert at j=6
      uint16x8_t ver = vreinterpretq_u16_s16( vabdq_s16( c, r5r0_r7r2 ) );

      // hor = [ abs(2*y1p1 - (y1p0 + y1p2)), abs( 2*y2p2 - (y2p1 + y2p3)),  => Lane 0+1 corresponds to horiz at j=0
      //         abs(2*y1p3 - (y1p2 + y1p4)), abs( 2*y2p4 - (y2p3 + y2p5)),  => Lane 2+3 corresponds to horiz at j=2
      //         abs(2*y1p5 - (y1p4 + y1p6)), abs( 2*y2p6 - (y2p5 + y2p7)),  => Lane 4+5 corresponds to horiz at j=4
      //         abs(2*y1p7 - (y1p6 + y1p8)), abs( 2*y2p8 - (y2p7 + y2p9)) ] => Lane 6+7 corresponds to horiz at j=6
      uint16x8_t horiz = vreinterpretq_u16_s16( vabdq_s16( d, r1r2_r5r6 ) );

      // diag0 = [ abs(2*y1p1 - (y0p0 + y2p2)), abs(2*y2p2 - (y1p1 + y3p3)),  => Lane 0+1 corresponds to diag0 at j=0
      //           abs(2*y1p3 - (y0p2 + y2p4)), abs(2*y2p4 - (y1p3 + y3p5)),  => Lane 2+3 corresponds to diag0 at j=2
      //           abs(2*y1p5 - (y0p4 + y2p6)), abs(2*y2p6 - (y1p5 + y3p7)),  => Lane 4+5 corresponds to diag0 at j=4
      //           abs(2*y1p7 - (y0p6 + y2p8)), abs(2*y2p8 - (y1p7 + y3p9)) ] => Lane 6+7 corresponds to diag0 at j=6
      uint16x8_t dig0 = vreinterpretq_u16_s16( vabdq_s16( d, r0r1_r6r7 ) );

      // diag1 = [ abs(2*y1p1 - (y0p2 + y2p0)), abs(2*y2p2 - (y1p3 + y3p1)),  => Lane 0+1 corresponds to diag1 at j=0
      //           abs(2*y1p3 - (y0p4 + y2p2)), abs(2*y2p4 - (y1p5 + y3p3)),  => Lane 2+3 corresponds to diag1 at j=2
      //           abs(2*y1p5 - (y0p6 + y2p4)), abs(2*y2p6 - (y1p7 + y3p5)),  => Lane 4+5 corresponds to diag1 at j=4
      //           abs(2*y1p7 - (y0p8 + y2p6)), abs(2*y2p8 - (y1p9 + y3p7)) ] => Lane 6+7 corresponds to diag1 at j=6
      uint16x8_t dig1 = vreinterpretq_u16_s16( vabdq_s16( d, r4r5_r2r3 ) );

      uint16x8_t hv = vvdec_vpaddq_u16( ver, horiz );
      uint16x8_t d01 = vvdec_vpaddq_u16( dig0, dig1 );

      // Sum 4 neighboring cells and store the result in the leftmost one.
      // [ (V0+V1), (V2+V3), (H0+H1), (H2+H3), (D00+D01), (D02+D03), (D10+D11), (D12+D13) ]
      uint16x8_t all = vvdec_vpaddq_u16( hv, d01 );

      uint16x8_t t = vbslq_u16( even_odd_mask, all, prev ); //[ all0, prev1, all2, prev3, all4, prev5, all6, prev7 ]

      // [ (V0+V1)+(old V2+V3), (H0+H1)+(old H2+H3), (D00+D01)+(old D02+D03), (D10+D11)+(old D12+D13),
      //   (V0+V1+V2+V3), (H0+H1+H2+H3), (D00+D01+D02+D03), (D10+D11+D12+D13)
      uint16x8_t out = vvdec_vpaddq_u16( t, all ); // for j == 0, out[0..3] are unused.

      // laplacian[i][j] corresponds to [VER, HOR, DIAG0, DIAG1, VER, HOR, DIAG0, DIAG1]
      vst1q_u16( dst, out );

      prev = all;

      src0 += 8;
      src1 += 8;
      src2 += 8;
      src3 += 8;
      dst += 8;
      j += 8;
    } while( j < widthExtended );

    pos += 2;
    src += stride * 2;
  } while( ++i != heightExtended );

  // Second pass.
  static constexpr uint8_t th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
  const uint8x16_t tbl = vld1q_u8( th );

  static constexpr uint8_t idx_arr[16] = { 0, 16, 4, 20, 8, 24, 12, 28, 0, 0, 0, 0, 0, 0, 0, 0 };
  const uint8x16_t idx = vld1q_u8( idx_arr );

  static constexpr int clsSizeY = 8;
  static constexpr int clsSizeX = 8;

  uint16_t* row0 = laplacian[0] + 4;
  uint16_t* row1 = laplacian[1] + 4;
  uint16_t* row2 = laplacian[2] + 4;
  uint16_t* row3 = laplacian[3] + 4;
  uint16_t* row4 = laplacian[4] + 4;
  uint16_t* row5 = laplacian[5] + 4;

  static constexpr int lapRowAdvance = ( clsSizeY >> 1 ) * ( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 8 );

  i = 0;
  do
  {
    int yVbPos0 = ( i + blk.y ) & ( vbCTUHeight - 1 ); // Row’s position inside its CTU.
    int distance0 = vbPos - yVbPos0;

    int yVbPos1 = ( i + 4 + blk.y ) & ( vbCTUHeight - 1 );
    int distance1 = vbPos - yVbPos1;

    bool near0_vbPos = distance0 == 0;
    bool near0_vbPosM4 = distance0 == 4;
    bool near1_vbPos = distance1 == 0;
    bool near1_vbPosM4 = distance1 == 4;

    const uint16x8_t mask_x0 = near0_vbPos ? vdupq_n_u16( 0xFFFFu ) : vdupq_n_u16( 0 );
    const uint16x8_t mask_x3 = near0_vbPosM4 ? vdupq_n_u16( 0xFFFFu ) : vdupq_n_u16( 0 );
    const uint16x8_t mask_x4 = near1_vbPos ? vdupq_n_u16( 0xFFFFu ) : vdupq_n_u16( 0 );
    const uint16x8_t mask_x7 = near1_vbPosM4 ? vdupq_n_u16( 0xFFFFu ) : vdupq_n_u16( 0 );

    uint32_t scale0 = near0_vbPos || near0_vbPosM4 ? 96 : 64;
    uint32_t scale1 = near1_vbPos || near1_vbPosM4 ? 96 : 64;
    uint32_t scale[4] = { scale0, scale0, scale1, scale1 };
    const uint32x4_t vScale = vld1q_u32( scale );

    AlfClassifier* clPtr1 = &classifier[( i + 0 ) / 4 * ( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE / 4 )];
    AlfClassifier* clPtr2 = &classifier[( i + 4 ) / 4 * ( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE / 4 )];

    int j = 0;
    do
    {
      uint16x8_t x0 = vld1q_u16( &row0[j] ); // [VER0, HOR0, DIAG00, DIAG10, VER1, HOR1, DIAG01, DIAG11]
      uint16x8_t x1 = vld1q_u16( &row1[j] );
      uint16x8_t x2 = vld1q_u16( &row2[j] );
      uint16x8_t x3 = vld1q_u16( &row3[j] );
      uint16x8_t x4 = x2; // [VER2, HOR2, DIAG02, DIAG12, VER3, HOR3, DIAG03, DIAG13] for second row.
      uint16x8_t x5 = x3;
      uint16x8_t x6 = vld1q_u16( &row4[j] );
      uint16x8_t x7 = vld1q_u16( &row5[j] );

      // Deal with vertical boundary crossing within the classification block.
      // If the block is just above VB, use 3 rows above the boundary.
      // If the block is just below VB, use 3 rows below the boundary.
      // Otherwise, use all 4 rows.
      x0 = vbicq_u16( x0, mask_x0 );
      x3 = vbicq_u16( x3, mask_x3 );
      x4 = vbicq_u16( x4, mask_x4 );
      x7 = vbicq_u16( x7, mask_x7 );

      uint32x4_t sum01_lo = vaddl_u16( vget_low_u16( x0 ), vget_low_u16( x1 ) );
      uint32x4_t sum01_hi = vaddl_u16( vget_high_u16( x0 ), vget_high_u16( x1 ) );
      uint32x4_t sum23_lo = vaddl_u16( vget_low_u16( x2 ), vget_low_u16( x3 ) );
      uint32x4_t sum23_hi = vaddl_u16( vget_high_u16( x2 ), vget_high_u16( x3 ) );
      uint32x4_t sum45_lo = vaddl_u16( vget_low_u16( x4 ), vget_low_u16( x5 ) );
      uint32x4_t sum45_hi = vaddl_u16( vget_high_u16( x4 ), vget_high_u16( x5 ) );
      uint32x4_t sum67_lo = vaddl_u16( vget_low_u16( x6 ), vget_low_u16( x7 ) );
      uint32x4_t sum67_hi = vaddl_u16( vget_high_u16( x6 ), vget_high_u16( x7 ) );

      uint32x4_t sum0123_lo = vaddq_u32( sum01_lo, sum23_lo ); // [VER0, HOR0, DIAG00, DIAG10]
      uint32x4_t sum0123_hi = vaddq_u32( sum01_hi, sum23_hi ); // [VER1, HOR1, DIAG01, DIAG11]
      uint32x4_t sum4567_lo = vaddq_u32( sum45_lo, sum67_lo ); // [VER2, HOR2, DIAG02, DIAG12]
      uint32x4_t sum4567_hi = vaddq_u32( sum45_hi, sum67_hi ); // [VER3, HOR3, DIAG03, DIAG13]

      uint32x4_t sumV, sumH, sumD0, sumD1;
      transpose_4x4_u32( sum0123_lo, sum0123_hi, sum4567_lo, sum4567_hi, sumV, sumH, sumD0, sumD1 );

      // After transpose, we have:
      // sumV = [VER0, VER1, VER2, VER3]
      // sumH = [HOR0, HOR1, HOR2, HOR3]
      // sumD0 = [DIAG00, DIAG01, DIAG02, DIAG03]
      // sumD1 = [DIAG10, DIAG11, DIAG12, DIAG13]

      // Compare vertical vs horizontal, pick the stronger HV direction.
      uint32x4_t hv1 = vmaxq_u32( sumV, sumH ); // Stronger of V/H.
      uint32x4_t hv0 = vminq_u32( sumV, sumH ); // Weaker of V/H.

      // Compare diagonal0 vs diagonal1, pick stronger diagonal direction.
      uint32x4_t d1 = vmaxq_u32( sumD0, sumD1 ); // Stronger of D0/D1.
      uint32x4_t d0 = vminq_u32( sumD0, sumD1 ); // Weaker of D0/D1.

      // Check the mainDirection be the HV direction or the diagonal direction.
      uint32x4_t hv0_d1 = vmulq_u32( hv0, d1 );
      uint32x4_t hv1_d0 = vmulq_u32( hv1, d0 );
      // d1 * hv0 > hv1 * d0
      uint32x4_t mainDirection =
          vcgeq_u32( hv1_d0, hv0_d1 ); // If H/V are more directional, mainDirection=0xFFFFFFFF, else 0x0.

      uint32x4_t hvd1 = vbslq_u32( mainDirection, hv1, d1 );
      uint32x4_t hvd0 = vbslq_u32( mainDirection, hv0, d0 );

      uint32x4_t hvd1_n2 = vshlq_n_u32( hvd1, 1 );                                  // hvd1 * 2
      uint32x4_t hvd0_n2 = vshlq_n_u32( hvd0, 1 );                                  // hvd0 * 2
      uint32x4_t hvd0_n9 = vmulq_n_u32( hvd0, 9 );                                  // hvd0 * 9
      int32x4_t strength1 = vreinterpretq_s32_u32( vcgtq_u32( hvd1, hvd0_n2 ) );    // hvd1 > 2 * hvd0
      int32x4_t strength2 = vreinterpretq_s32_u32( vcgtq_u32( hvd1_n2, hvd0_n9 ) ); // hvd1 * 2 > 9 * hvd0
      int32x4_t strengthSum = vaddq_s32( strength1, strength2 );

      // If mainDirection is diagonal,
      //    strength1 : add 5 to classIdx.
      //    strength2 : add 10 to classIdx.
      // If mainDirection is horizontal/vertical,
      //    strength1 : add 15 to classIdx.
      //    strength2 : add 20 to classIdx.
      // Rewrite the logic as below (if strength2 is true, then strength1 is also true):
      //    if( strength1 ) classIdx += 5.
      //    if( strength2 ) classIdx += 5.
      //    if( mainDirection == HV && strength1 ) classIdx += 10.

      // mainDirection true means HV choosen.
      int32x4_t cond3 = vandq_s32( vreinterpretq_s32_u32( mainDirection ), strength1 );

      uint32x4_t activity = vaddq_u32( sumV, sumH );
      activity = vmulq_u32( activity, vScale );
      activity = vshlq_u32( activity, vdupq_n_s32( -shift ) );
      activity = vminq_u32( activity, vdupq_n_u32( maxActivity ) ); // Range: [0..15]
      uint8x16_t idx8 = vreinterpretq_u8_u32( activity );
      // Upper 3 lanes are zeros, retrieves zero from tbl for those lanes, doesn't matter since they won't be used.
      int32x4_t classIdx = vreinterpretq_s32_u8( vvdec_vqtbl1q_u8( tbl, idx8 ) ); // Range: [0..4]

      classIdx = vmlsq_n_s32( classIdx, strengthSum, 5 );
      classIdx = vmlsq_n_s32( classIdx, cond3, 10 );

      // Calculate transposeIdx from mainDirection and secondaryDirection.
      // For a pair of mainDirection and secondaryDirection, transposeIdx is unique.
      // | mainDirection | secondaryDirection | transposeIdx |
      // | ------------- | ------------------ | -----------  |
      // | DIAG0 (0)     | V (1)              |  0           |
      // | DIAG0 (0)     | H (3)              |  1           |
      // | V (1)         | DIAG0 (0)          |  0           |
      // | V (1)         | DIAG1 (2)          |  2           |
      // | DIAG1 (2)     | V (1)              |  2           |
      // | DIAG1 (2)     | H (3)              |  3           |
      // | H (3)         | DIAG0 (0)          |  1           |
      // | H (3)         | DIAG1 (2)          |  3           |

      // If V stronger, hvMask = -1; if H stronger, hvMask = 0.
      // If DIAG0 stronger, dMask = -1; if DIAG1 stronger, dMask = 0.
      int32x4_t hvMask = vreinterpretq_s32_u32( vcgtq_u32( sumV, sumH ) );
      int32x4_t dMask = vreinterpretq_s32_u32( vcgtq_u32( sumD0, sumD1 ) );

      // transposeIdx = 3 + hvMask + 2 * dMask
      int32x4_t transposeIdx = vaddq_s32( vaddq_s32( vdupq_n_s32( 3 ), hvMask ), vshlq_n_s32( dMask, 1 ) );

      const uint8x16x2_t ct = { vreinterpretq_u8_s32( classIdx ), vreinterpretq_u8_s32( transposeIdx ) };
      uint32x4_t classifier = vreinterpretq_u32_u8( vvdec_vqtbl2q_u8( ct, idx ) ); // [c0,t0,c1,t1,c2,t2,c3,t3]

      // Lanes 2, 3 are unused.
      store_unaligned_u32_4x1<0>( ( void* )clPtr1, classifier );
      store_unaligned_u32_4x1<1>( ( void* )clPtr2, classifier );

      clPtr1 += 2;
      clPtr2 += 2;
      j += clsSizeX;
    } while( j < blk.width );

    row0 += lapRowAdvance;
    row1 += lapRowAdvance;
    row2 += lapRowAdvance;
    row3 += lapRowAdvance;
    row4 += lapRowAdvance;
    row5 += lapRowAdvance;
    i += clsSizeY;
  } while( i < blk.height );
}

template<>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterARM<NEON>()
{
  m_deriveClassificationBlk = DeriveClassificationBlk_neon;
  m_filter7x7Blk = Filter7x7Blk_neon;
  m_filter5x5Blk = Filter5x5Blk_neon;
}

} // namespace vvdec
#endif
//! \}
