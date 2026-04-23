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

#include "../LoopFilter.h"
#include "CommonDefARM.h"
#include "mem_neon.h"
#include "tbl_neon.h"
#include "transpose_neon.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_DBLF

namespace vvdec
{

static inline int16x4_t clip3_neon( const int16x4_t value, const int16x4_t minValue, const int16x4_t maxValue )
{
  return vmax_s16( vmin_s16( value, maxValue ), minValue );
}

static inline int16x4_t clipPel_neon( const int16x4_t value, const ClpRng& clpRng )
{
  const int16x4_t vmin = vdup_n_s16( 0 );
  const int16x4_t vmax = vdup_n_s16( clpRng.max() );
  return clip3_neon( value, vmin, vmax );
}

static inline void xPelFilterLumaWeakCore_neon( int16x4_t& vm1, int16x4_t& vm2, int16x4_t& vm3, int16x4_t& vm4,
                                                int16x4_t& vm5, int16x4_t& vm6, const int tc, const int iThrCut,
                                                const bool bFilterSecondP, const bool bFilterSecondQ,
                                                const ClpRng& clpRng )
{
  const int16x4_t tcVec = vdup_n_s16(  tc );
  const int16x4_t tcNeg = vdup_n_s16( -tc );
  const int16x4_t tcHalf = vdup_n_s16( tc >> 1 );
  const int16x4_t tcHalfN = vneg_s16( tcHalf );

  const int16x4_t vm3Orig = vm3;
  const int16x4_t vm4Orig = vm4;

  const int16x4_t diff0 = vsub_s16( vm4, vm3 );
  const int16x4_t diff1 = vsub_s16( vm5, vm2 );

  CHECKD( clpRng.bd > 10, "Bit-depth > 10 not supported" );
  int16x4_t delta = vmul_n_s16( diff0, 9 ); // Works for up to 10-bit.
  delta = vmls_n_s16( delta, diff1, 3 );
  delta = vrshr_n_s16( delta, 4 );

  const uint16x4_t applyMask = vcgt_s16( vdup_n_s16( iThrCut ), vabs_s16( delta ) );

  delta = clip3_neon( delta, tcNeg, tcVec );

  const int16x4_t vm3New = clipPel_neon( vadd_s16( vm3, delta ), clpRng );
  vm3 = vbsl_s16( applyMask, vm3New, vm3 );

  if( bFilterSecondP )
  {
    const int16x4_t avgP = vrhadd_s16( vm1, vm3Orig );
    int16x4_t deltaP = vhadd_s16( vsub_s16( avgP, vm2 ), delta );
    deltaP = clip3_neon( deltaP, tcHalfN, tcHalf );
    const int16x4_t vm2New = clipPel_neon( vadd_s16( vm2, deltaP ), clpRng );
    vm2 = vbsl_s16( applyMask, vm2New, vm2 );
  }

  const int16x4_t vm4New = clipPel_neon( vsub_s16( vm4, delta ), clpRng );
  vm4 = vbsl_s16( applyMask, vm4New, vm4 );

  if( bFilterSecondQ )
  {
    const int16x4_t avgQ = vrhadd_s16( vm6, vm4Orig );
    int16x4_t deltaQ = vhsub_s16( vsub_s16( avgQ, vm5 ), delta );
    deltaQ = clip3_neon( deltaQ, tcHalfN, tcHalf );
    const int16x4_t vm5New = clipPel_neon( vadd_s16( vm5, deltaQ ), clpRng );
    vm5 = vbsl_s16( applyMask, vm5New, vm5 );
  }
}

static inline void xPelFilterLumaStrongHor_neon( Pel* piSrc, const ptrdiff_t offset, const int tc )
{
  const int16x4_t m0 = vld1_s16( &piSrc[-4 * offset] );
  int16x4_t       m1 = vld1_s16( &piSrc[-3 * offset] );
  int16x4_t       m2 = vld1_s16( &piSrc[-2 * offset] );
  int16x4_t       m3 = vld1_s16( &piSrc[-1 * offset] );
  int16x4_t       m4 = vld1_s16( &piSrc[ 0 * offset] );
  int16x4_t       m5 = vld1_s16( &piSrc[ 1 * offset] );
  int16x4_t       m6 = vld1_s16( &piSrc[ 2 * offset] );
  const int16x4_t m7 = vld1_s16( &piSrc[ 3 * offset] );

  const int16x4_t m1234 = vadd_s16( vadd_s16( m1, m2 ), vadd_s16( m3, m4 ) );
  const int16x4_t m3456 = vadd_s16( vadd_s16( m5, m6 ), vadd_s16( m3, m4 ) );

  const int16x4_t r1 = vmla_n_s16( m1234, vadd_s16( m0, m1 ), 2 );
  const int16x4_t r2 = m1234;
  const int16x4_t r3 = vmla_n_s16( vsub_s16( m5, m1 ), m1234, 2 );
  const int16x4_t r4 = vmla_n_s16( vsub_s16( m2, m6 ), m3456, 2 );
  const int16x4_t r5 = m3456;
  const int16x4_t r6 = vmla_n_s16( m3456, vadd_s16( m6, m7 ), 2 );

  const int16x4_t tc0 = vdup_n_s16( 3 * tc );
  const int16x4_t tc1 = vdup_n_s16( 2 * tc );
  const int16x4_t tc2 = vdup_n_s16( tc );

  m1 = clip3_neon( vrshr_n_s16( r1, 3 ), vsub_s16( m1, tc2 ), vadd_s16( m1, tc2 ) );
  m2 = clip3_neon( vrshr_n_s16( r2, 2 ), vsub_s16( m2, tc1 ), vadd_s16( m2, tc1 ) );
  m3 = clip3_neon( vrshr_n_s16( r3, 3 ), vsub_s16( m3, tc0 ), vadd_s16( m3, tc0 ) );
  m4 = clip3_neon( vrshr_n_s16( r4, 3 ), vsub_s16( m4, tc0 ), vadd_s16( m4, tc0 ) );
  m5 = clip3_neon( vrshr_n_s16( r5, 2 ), vsub_s16( m5, tc1 ), vadd_s16( m5, tc1 ) );
  m6 = clip3_neon( vrshr_n_s16( r6, 3 ), vsub_s16( m6, tc2 ), vadd_s16( m6, tc2 ) );

  vst1_s16( &piSrc[-3 * offset], m1 );
  vst1_s16( &piSrc[-2 * offset], m2 );
  vst1_s16( &piSrc[-1 * offset], m3 );
  vst1_s16( &piSrc[ 0 * offset], m4 );
  vst1_s16( &piSrc[ 1 * offset], m5 );
  vst1_s16( &piSrc[ 2 * offset], m6 );
}

static inline void xPelFilterLumaStrongVer_neon( Pel* piSrc, const ptrdiff_t step, const int tc )
{
  static constexpr uint8_t shuffleIdx[5][16] =
  {
    // 88 index is used to fill the unused elements with 0.
    { 88, 88, 0, 1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 88, 88 },
    { 88, 88, 2, 3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 88, 88 },
    { 88, 88, 4, 5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 88, 88 },
    { 88, 88, 6, 7,  8,  9, 10, 11, 12, 13, 88, 88,  6,  7, 88, 88 },
    { 88, 88, 8, 9, 88, 88,  2,  3,  4,  5,  6,  7,  8,  9, 88, 88 },
  };
  const uint8x16_t shuffle0 = vld1q_u8( shuffleIdx[0] );
  const uint8x16_t shuffle1 = vld1q_u8( shuffleIdx[1] );
  const uint8x16_t shuffle2 = vld1q_u8( shuffleIdx[2] );
  const uint8x16_t shuffle3 = vld1q_u8( shuffleIdx[3] );
  const uint8x16_t shuffle4 = vld1q_u8( shuffleIdx[4] );

  static constexpr int16_t coeff[3][8] =
  {
    { 0, 2, 1, 2, 2, 1, 1, 0 },
    { 0, 3, 1, 2, 2, 1, 3, 0 },
    { 0, 1, 1, 2, 2, 1, 2, 0 },
  };
  const int16x8_t coeff0 = vld1q_s16( coeff[0] );
  const int16x8_t coeff1 = vld1q_s16( coeff[1] );
  const int16x8_t coeff2 = vld1q_s16( coeff[2] );

  static constexpr int16_t neg_shift[8] = { 0, -3, -2, -3, -3, -2, -3, 0 };
  const int16x8_t lshift = vld1q_s16( neg_shift );

  static constexpr int16_t tc3[8] = { 0, 1, 2, 3, 3, 2, 1, 0 };
  const int16x8_t tc_max = vmulq_s16( vld1q_s16( tc3 ), vdupq_n_s16( tc ) );
  const int16x8_t tc_min = vnegq_s16( tc_max );

  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    int8x16_t s = vreinterpretq_s8_s16( vld1q_s16( &piSrc[-4 + i * step] ) );
    int16x8_t s0 = vreinterpretq_s16_s8( vvdec_vqtbl1q_s8( s, shuffle0 ) );
    int16x8_t s1 = vreinterpretq_s16_s8( vvdec_vqtbl1q_s8( s, shuffle1 ) );
    int16x8_t s2 = vreinterpretq_s16_s8( vvdec_vqtbl1q_s8( s, shuffle2 ) );
    int16x8_t s3 = vreinterpretq_s16_s8( vvdec_vqtbl1q_s8( s, shuffle3 ) );
    int16x8_t s4 = vreinterpretq_s16_s8( vvdec_vqtbl1q_s8( s, shuffle4 ) );

    int16x8_t sum = vaddq_s16( s3, s4 );
    sum = vmlaq_s16( sum, s0, coeff0 );
    sum = vmlaq_s16( sum, s1, coeff1 );
    sum = vmlaq_s16( sum, s2, coeff2 );
    sum = vrshlq_s16( sum, lshift );
    sum = vsubq_s16( sum, s1 );

    sum = vminq_s16( tc_max, vmaxq_s16( tc_min, sum ) );
    int16x8_t d = vaddq_s16( sum, vreinterpretq_s16_s8( s ) );
    vst1q_s16( &piSrc[-4 + i * step], d );
  }
}

static inline void xPelFilterLumaWeakHor_neon( Pel* piSrc, const ptrdiff_t offset, const int tc, const int iThrCut,
                                               const bool bFilterSecondP, const bool bFilterSecondQ,
                                               const ClpRng& clpRng )
{
  int16x4_t vm1 = vld1_s16( &piSrc[-3 * offset] );
  int16x4_t vm2 = vld1_s16( &piSrc[-2 * offset] );
  int16x4_t vm3 = vld1_s16( &piSrc[-1 * offset] );
  int16x4_t vm4 = vld1_s16( &piSrc[ 0 * offset] );
  int16x4_t vm5 = vld1_s16( &piSrc[ 1 * offset] );
  int16x4_t vm6 = vld1_s16( &piSrc[ 2 * offset] );

  xPelFilterLumaWeakCore_neon( vm1, vm2, vm3, vm4, vm5, vm6, tc, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );

  if( bFilterSecondP )
  {
    vst1_s16( &piSrc[-2 * offset], vm2 );
  }
  vst1_s16( &piSrc[-1 * offset], vm3 );
  vst1_s16( &piSrc[ 0 * offset], vm4 );
  if( bFilterSecondQ )
  {
    vst1_s16( &piSrc[1 * offset], vm5 );
  }
}

static inline void xPelFilterLumaWeakVer_neon( Pel* piSrc, const ptrdiff_t step, const int tc, const int iThrCut,
                                               const bool bFilterSecondP, const bool bFilterSecondQ,
                                               const ClpRng& clpRng )
{
  int16x8_t row0 = load_s16x6( &piSrc[-3 + 0 * step] );
  int16x8_t row1 = load_s16x6( &piSrc[-3 + 1 * step] );
  int16x8_t row2 = load_s16x6( &piSrc[-3 + 2 * step] );
  int16x8_t row3 = load_s16x6( &piSrc[-3 + 3 * step] );

  int16x8_t col0, col1, col2, col3;
  transpose_concat_8x4_s16( row0, row1, row2, row3, col0, col1, col2, col3 );

  int16x4_t vm1 = vget_low_s16 ( col0 );
  int16x4_t vm2 = vget_high_s16( col0 );
  int16x4_t vm3 = vget_low_s16 ( col1 );
  int16x4_t vm4 = vget_high_s16( col1 );
  int16x4_t vm5 = vget_low_s16 ( col2 );
  int16x4_t vm6 = vget_high_s16( col2 );

  xPelFilterLumaWeakCore_neon( vm1, vm2, vm3, vm4, vm5, vm6, tc, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );

  int16x4_t d0, d1, d2, d3;
  transpose_4x4_s16( vm2, vm3, vm4, vm5, d0, d1, d2, d3 );

  vst1_s16( &piSrc[0 * step - 2], d0 );
  vst1_s16( &piSrc[1 * step - 2], d1 );
  vst1_s16( &piSrc[2 * step - 2], d2 );
  vst1_s16( &piSrc[3 * step - 2], d3 );
}

void xPelFilterLuma_neon( Pel* piSrc, const ptrdiff_t step, const ptrdiff_t offset, const int tc, const bool sw,
                          const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ,
                          const ClpRng& clpRng )
{
  if( sw )
  {
    if( offset == 1 )
    {
      xPelFilterLumaStrongVer_neon( piSrc, step, tc );
    }
    else
    {
      xPelFilterLumaStrongHor_neon( piSrc, offset, tc );
    }
  }
  else
  {
    if( offset == 1 )
    {
      xPelFilterLumaWeakVer_neon( piSrc, step, tc, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );
    }
    else
    {
      xPelFilterLumaWeakHor_neon( piSrc, offset, tc, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );
    }
  }
}

template<>
void LoopFilter::_initLoopFilterARM<NEON>()
{
  xPelFilterLuma = xPelFilterLuma_neon;
}

} // namespace vvdec

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_DBLF
