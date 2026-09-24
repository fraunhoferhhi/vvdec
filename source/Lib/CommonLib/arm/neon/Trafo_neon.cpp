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
/**
 * \file Trafo_neon.cpp
 * \brief Neon implementation of selected TCoeffOps kernels.
 */

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/TrQuant.h"
#include "TrQuant_EMT.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_TCOEFF_OPS

namespace vvdec
{

template<unsigned trSize>
void fastInvCore_neon( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned lines, unsigned reducedLines,
                       unsigned rows )
{
  static_assert( trSize % 4 == 0, "trSize should be a multiple of four" );
  CHECKD( rows == 0, "rows should be non-zero" );

  unsigned i = 0;
  for( ; i != ( reducedLines & ~3U ); i += 4 )
  {
    unsigned j = 0;
    do
    {
      const TCoeff* srci = src + i;
      const TMatrixCoeff* itj = it + j;
      TCoeff* dstij = dst + i * trSize + j;

      int32x4_t d0 = vld1q_s32( dstij + 0 * trSize );
      int32x4_t d1 = vld1q_s32( dstij + 1 * trSize );
      int32x4_t d2 = vld1q_s32( dstij + 2 * trSize );
      int32x4_t d3 = vld1q_s32( dstij + 3 * trSize );

      unsigned k = rows;
      do
      {
        int16x4_t s = vmovn_s32( vld1q_s32( srci ) );
        int16x4_t c = vld1_s16( itj );

        d0 = vmlal_lane_s16( d0, c, s, 0 );
        d1 = vmlal_lane_s16( d1, c, s, 1 );
        d2 = vmlal_lane_s16( d2, c, s, 2 );
        d3 = vmlal_lane_s16( d3, c, s, 3 );

        srci += lines;
        itj += trSize;
      } while( --k != 0 );

      vst1q_s32( dstij + 0 * trSize, d0 );
      vst1q_s32( dstij + 1 * trSize, d1 );
      vst1q_s32( dstij + 2 * trSize, d2 );
      vst1q_s32( dstij + 3 * trSize, d3 );

      j += 4;
    } while( j != trSize );
  }

  for( ; i != reducedLines; ++i )
  {
    unsigned j = 0;
    do
    {
      const TCoeff* srci = src + i;
      const TMatrixCoeff* itj = it + j;
      TCoeff* dstij = dst + i * trSize + j;

      int32_t d0 = *dstij;
      unsigned k = rows;
      do
      {
        d0 += *srci * *itj;

        srci += lines;
        itj += trSize;
      } while( --k != 0 );

      *dstij = d0;
    } while( ++j != trSize );
  }
}

template<>
void TCoeffOps::_initTCoeffOpsARM<NEON>()
{
  fastInvCore[0] = fastInvCore_neon<4>;
  fastInvCore[1] = fastInvCore_neon<8>;
  fastInvCore[2] = fastInvCore_neon<16>;
  fastInvCore[3] = fastInvCore_neon<32>;
  fastInvCore[4] = fastInvCore_neon<64>;
}

// Portable (ARMv7 NEON and AArch64) horizontal reduction of four int32x4_t
// row accumulators into one int32x4_t of per-row sums: [sum(a0),sum(a1),sum(a2),sum(a3)].
// vpaddq_s32 would be shorter but is AArch64-only; this project still builds an
// ARMv7 NEON target (source/Lib/vvdec/CMakeLists.txt), so this reduction avoids it.
static inline int32x4_t hsum4x4( int32x4_t a0, int32x4_t a1, int32x4_t a2, int32x4_t a3 )
{
  int32x2_t r0 = vadd_s32( vget_low_s32( a0 ), vget_high_s32( a0 ) );
  int32x2_t r1 = vadd_s32( vget_low_s32( a1 ), vget_high_s32( a1 ) );
  int32x2_t r2 = vadd_s32( vget_low_s32( a2 ), vget_high_s32( a2 ) );
  int32x2_t r3 = vadd_s32( vget_low_s32( a3 ), vget_high_s32( a3 ) );

  int32x2_t s01 = vpadd_s32( r0, r1 );
  int32x2_t s23 = vpadd_s32( r2, r3 );

  return vcombine_s32( s01, s23 );
}

// NEON implementation of the inverse LFNST matrix-vector product
// (scalar reference: TrQuant.cpp::invLfnstNxNCore).
// zeroOutSize (the number of active input coefficients) is always 8 or 16 in
// this decoder; the matrix row stride is always 16 bytes regardless.
static void invLfnstNxNCore_neon( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize )
{
  CHECK( index > 2 || ( zeroOutSize != 8 && zeroOutSize != 16 ), "Wrong parameters" );

  static constexpr int32_t outputMinimum = -( 1 << 15 );
  static constexpr int32_t outputMaximum =  ( 1 << 15 ) - 1;

  const int8_t* trMat  = ( size > 4 ) ? g_lfnst8x8[ mode ][ index ][ 0 ] : g_lfnst4x4[ mode ][ index ][ 0 ];
  const int     trSize = ( size > 4 ) ? 48 : 16;
  const bool    wide   = ( zeroOutSize == 16 );
  int*          out    = dst;

  // Load the active source coefficients once and narrow to signed 16-bit.
  // Lossless: dequantized coefficients feeding LFNST are clipped to
  // [-32768,32767] (transform dynamic range 15, see Quant.cpp/Slice.h).
  int16x8_t vsrc0 = vcombine_s16( vmovn_s32( vld1q_s32( src ) ), vmovn_s32( vld1q_s32( src + 4 ) ) );
  int16x8_t vsrc1 = vdupq_n_s16( 0 );
  if( wide )
  {
    vsrc1 = vcombine_s16( vmovn_s32( vld1q_s32( src + 8 ) ), vmovn_s32( vld1q_s32( src + 12 ) ) );
  }

  const int32x4_t vmin = vdupq_n_s32( outputMinimum );
  const int32x4_t vmax = vdupq_n_s32( outputMaximum );
  const int32x4_t v64  = vdupq_n_s32( 64 );

  for( int j = 0; j < trSize; j += 4, out += 4 )
  {
    int32x4_t acc[4];

    for( int k = 0; k < 4; k++ )
    {
      const int8_t* row = trMat + k * 16;

      int16x8_t vtr0 = vmovl_s8( vld1_s8( row ) );

      int32x4_t a = vmull_s16( vget_low_s16( vsrc0 ), vget_low_s16( vtr0 ) );
                a = vmlal_s16( a, vget_high_s16( vsrc0 ), vget_high_s16( vtr0 ) );

      if( wide )
      {
        int16x8_t vtr1 = vmovl_s8( vld1_s8( row + 8 ) );
        a = vmlal_s16( a, vget_low_s16( vsrc1 ), vget_low_s16( vtr1 ) );
        a = vmlal_s16( a, vget_high_s16( vsrc1 ), vget_high_s16( vtr1 ) );
      }

      acc[k] = a;
    }

    trMat += 64; // 4 rows * 16 bytes/row, regardless of zeroOutSize

    int32x4_t sums = hsum4x4( acc[0], acc[1], acc[2], acc[3] );

    sums = vaddq_s32( sums, v64 );
    sums = vshrq_n_s32( sums, 7 );
    sums = vminq_s32( vmaxq_s32( sums, vmin ), vmax );

    vst1q_s32( out, sums );
  }
}

template<>
void TrQuant::_initTrQuantARM<NEON>()
{
  m_invLfnstNxN = invLfnstNxNCore_neon;
}

} // namespace vvdec

#endif
