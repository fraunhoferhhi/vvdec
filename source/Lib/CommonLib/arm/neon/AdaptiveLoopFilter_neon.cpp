/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

template<>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterARM<NEON>()
{
  m_filter7x7Blk = Filter7x7Blk_neon;
  m_filter5x5Blk = Filter5x5Blk_neon;
}

} // namespace vvdec
#endif
//! \}
