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

} // namespace vvdec

#endif
