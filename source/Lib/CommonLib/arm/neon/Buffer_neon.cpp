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


-------------------------------------------------------------------------------------------
*/

/**
 * \file Buffer_neon.cpp
 * \brief NEON buffer operations.
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "../BufferARM.h"
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "tbl_neon.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCIF

namespace vvdec
{

void addAvg4_neon( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel* dst,
                   ptrdiff_t dstStride, int width, int height, int rshift, int offset, const ClpRng& clpRng )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 4 || width & 3, "Width must be >= 4 and a multiple of 4" );
  CHECKD( offset > 16448, "Offset must be <= 16448" ); // Max: (1 << (rshift - 1)) + 2 * (1 << 13), where rshift=7.

  const int lshift = -rshift;
  const uint16x4_t max = vdup_n_u16( clpRng.max() );

  do
  {
    int w = 0;
    do
    {
      int16x4_t s0 = vld1_s16( src0 + w );
      int16x4_t s1 = vld1_s16( src1 + w );

      int32x4_t sum_32 = vaddl_s16( s0, s1 );
      sum_32 = vaddq_s32( sum_32, vdupq_n_s32( offset ) );
      sum_32 = vshlq_s32( sum_32, vdupq_n_s32( lshift ) );

      uint16x4_t sum_16 = vqmovun_s32( sum_32 );
      sum_16 = vmin_u16( sum_16, max );

      vst1_s16( dst + w, vreinterpret_s16_u16( sum_16 ) );
      w += 4;
    } while( w != width );

    src0 += src0Stride;
    src1 += src1Stride;
    dst += dstStride;
  } while( --height != 0 );
}

void addAvg8_neon( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel* dst,
                   ptrdiff_t dstStride, int width, int height, int rshift, int offset, const ClpRng& clpRng )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 8 || width & 7, "Width must be >= 8 and a multiple of 8" );
  CHECKD( offset > 16448, "Offset must be <= 16448" ); // Max: (1 << (rshift - 1)) + 2 * (1 << 13), where rshift=7.

  const int lshift = -rshift;
  const uint16x8_t max = vdupq_n_u16( clpRng.max() );

  do
  {
    int w = 0;
    do
    {
      int16x8_t s0 = vld1q_s16( src0 + w );
      int16x8_t s1 = vld1q_s16( src1 + w );

      int32x4_t sum_32_lo = vaddl_s16( vget_low_s16( s0 ), vget_low_s16( s1 ) );
      int32x4_t sum_32_hi = vaddl_s16( vget_high_s16( s0 ), vget_high_s16( s1 ) );

      sum_32_lo = vaddq_s32( sum_32_lo, vdupq_n_s32( offset ) );
      sum_32_lo = vshlq_s32( sum_32_lo, vdupq_n_s32( lshift ) );

      sum_32_hi = vaddq_s32( sum_32_hi, vdupq_n_s32( offset ) );
      sum_32_hi = vshlq_s32( sum_32_hi, vdupq_n_s32( lshift ) );

      uint16x8_t sum_16 = vcombine_u16( vqmovun_s32( sum_32_lo ), vqmovun_s32( sum_32_hi ) );
      sum_16 = vminq_u16( sum_16, max );

      vst1q_s16( dst + w, vreinterpretq_s16_u16( sum_16 ) );
      w += 8;
    } while( w != width );

    src0 += src0Stride;
    src1 += src1Stride;
    dst += dstStride;
  } while( --height != 0 );
}

void addAvg16_neon( const Pel* src0, ptrdiff_t src0Stride, const Pel* src1, ptrdiff_t src1Stride, Pel* dst,
                    ptrdiff_t dstStride, int width, int height, int rshift, int offset, const ClpRng& clpRng )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 16 || width & 15, "Width must be >= 16 and a multiple of 16" );
  CHECKD( offset > 16448, "Offset must be <= 16448" ); // Max: (1 << (rshift - 1)) + 2 * (1 << 13), where rshift=7.

  const int lshift = -rshift;
  uint16x8_t max = vdupq_n_u16( clpRng.max() );

  do
  {
    int w = 0;
    do
    {
      int16x8_t s0_lo = vld1q_s16( src0 + w + 0 );
      int16x8_t s0_hi = vld1q_s16( src0 + w + 8 );
      int16x8_t s1_lo = vld1q_s16( src1 + w + 0 );
      int16x8_t s1_hi = vld1q_s16( src1 + w + 8 );

      int32x4_t sum_32_lo0 = vaddl_s16( vget_low_s16( s0_lo ), vget_low_s16( s1_lo ) );
      int32x4_t sum_32_lo1 = vaddl_s16( vget_high_s16( s0_lo ), vget_high_s16( s1_lo ) );
      int32x4_t sum_32_hi0 = vaddl_s16( vget_low_s16( s0_hi ), vget_low_s16( s1_hi ) );
      int32x4_t sum_32_hi1 = vaddl_s16( vget_high_s16( s0_hi ), vget_high_s16( s1_hi ) );

      sum_32_lo0 = vaddq_s32( sum_32_lo0, vdupq_n_s32( offset ) );
      sum_32_lo1 = vaddq_s32( sum_32_lo1, vdupq_n_s32( offset ) );
      sum_32_hi0 = vaddq_s32( sum_32_hi0, vdupq_n_s32( offset ) );
      sum_32_hi1 = vaddq_s32( sum_32_hi1, vdupq_n_s32( offset ) );

      sum_32_lo0 = vshlq_s32( sum_32_lo0, vdupq_n_s32( lshift ) );
      sum_32_lo1 = vshlq_s32( sum_32_lo1, vdupq_n_s32( lshift ) );
      sum_32_hi0 = vshlq_s32( sum_32_hi0, vdupq_n_s32( lshift ) );
      sum_32_hi1 = vshlq_s32( sum_32_hi1, vdupq_n_s32( lshift ) );

      uint16x8_t sum_16_lo = vcombine_u16( vqmovun_s32( sum_32_lo0 ), vqmovun_s32( sum_32_lo1 ) );
      uint16x8_t sum_16_hi = vcombine_u16( vqmovun_s32( sum_32_hi0 ), vqmovun_s32( sum_32_hi1 ) );

      sum_16_lo = vminq_u16( sum_16_lo, max );
      sum_16_hi = vminq_u16( sum_16_hi, max );

      vst1q_s16( dst + w + 0, vreinterpretq_s16_u16( sum_16_lo ) );
      vst1q_s16( dst + w + 8, vreinterpretq_s16_u16( sum_16_hi ) );
      w += 16;
    } while( w != width );

    src0 += src0Stride;
    src1 += src1Stride;
    dst += dstStride;
  } while( --height != 0 );
}

void rspFwdCore_neon( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const int bd, const Pel OrgCW,
                      const Pel* LmcsPivot, const Pel* ScaleCoeff, const Pel* InputPivot )
{
  int shift = getLog2( OrgCW );

  if( ( width & 7 ) == 0 )
  {
    int8x16x2_t mLmcsPivot = vld2q_s8( ( const int8_t* )LmcsPivot );
    int8x16x2_t mInputPivot = vld2q_s8( ( const int8_t* )InputPivot );
    int8x16x2_t mScaleCoeff = vld2q_s8( ( const int8_t* )ScaleCoeff );

    const int16x8_t mMin = vdupq_n_s16( 0 );
    const int16x8_t mMax = vdupq_n_s16( ( 1 << bd ) - 1 );

    const uint8_t idx4idx_array[16] = { 0, 2, 4, 6, 8, 10, 12, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    const uint8x16_t idx4idx = vld1q_u8( idx4idx_array );

    do
    {
      int w = 0;
      do
      {
        const int16x8_t xsrc = vld1q_s16( &ptr[w] );

        // ( idxY = ptr[w] >> shift ) range is [0, 15]. Convert idxY to 8-bit so we can use the 8-bit vqtbl1q lookup.
        const uint8x16_t idxY =
            vvdec_vqtbl1q_u8( vreinterpretq_u8_s16( vshlq_s16( xsrc, vdupq_n_s16( -shift ) ) ), idx4idx );

        const int8x16_t xlmc_s8 =
            vzipq_s8( vvdec_vqtbl1q_s8( mLmcsPivot.val[0], idxY ), vvdec_vqtbl1q_s8( mLmcsPivot.val[1], idxY ) ).val[0];
        const int16x8_t xlmc = vreinterpretq_s16_s8( xlmc_s8 );

        const int8x16_t xinp_s8 =
            vzipq_s8( vvdec_vqtbl1q_s8( mInputPivot.val[0], idxY ), vvdec_vqtbl1q_s8( mInputPivot.val[1], idxY ) ).val[0];
        const int16x8_t xinp = vreinterpretq_s16_s8( xinp_s8 );

        const int8x16_t xscl_s8 =
            vzipq_s8( vvdec_vqtbl1q_s8( mScaleCoeff.val[0], idxY ), vvdec_vqtbl1q_s8( mScaleCoeff.val[1], idxY ) ).val[0];
        const int16x8_t xscl = vreinterpretq_s16_s8( xscl_s8 );

        int16x8_t diff = vqsubq_s16( xsrc, xinp );

        int32x4_t mul_lo = vmull_s16( vget_low_s16( diff ), vget_low_s16( xscl ) );
        int32x4_t mul_hi = vmull_s16( vget_high_s16( diff ), vget_high_s16( xscl ) );

        int16x8_t xtmp1 = vcombine_s16( vrshrn_n_s32( mul_lo, 11 ), vrshrn_n_s32( mul_hi, 11 ) );

        xtmp1 = vaddq_s16( xlmc, xtmp1 );

        xtmp1 = vminq_s16( xtmp1, mMax );
        xtmp1 = vmaxq_s16( xtmp1, mMin );

        vst1q_s16( &ptr[w], xtmp1 );

        w += 8;
      } while( w < width );
      ptr += ptrStride;
    } while( --height != 0 );
  }
  else
  {
    int idxY;

#define RSP_FWD_OP( ADDR )                                                                                             \
  {                                                                                                                    \
    idxY = ptr[ADDR] >> shift;                                                                                         \
    ptr[ADDR] = static_cast<Pel>( ClipBD<int>(                                                                         \
        LmcsPivot[idxY] + ( ( ScaleCoeff[idxY] * ( ptr[ADDR] - InputPivot[idxY] ) + ( 1 << 10 ) ) >> 11 ), bd ) );     \
  }
#define RSP_FWD_INC ptr += ptrStride;

    SIZE_AWARE_PER_EL_OP( RSP_FWD_OP, RSP_FWD_INC )

#undef RSP_FWD_OP
#undef RSP_FWD_INC
  }
}

template<>
void PelBufferOps::_initPelBufOpsARM<NEON>()
{
  addAvg4 = addAvg4_neon;
  addAvg8 = addAvg8_neon;
  addAvg16 = addAvg16_neon;
  applyLut = applyLut_SIMD<NEON>;
  rspFwd = rspFwdCore_neon;
}

} // namespace vvdec
#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCIF
//! \}
