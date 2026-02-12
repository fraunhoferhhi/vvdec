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
 * \file Buffer_sve2.cpp
 * \brief SVE2 implementation of buffer operations (fixed 128-bit vectors).
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "CommonLib/CommonDef.h"
#include "CommonLib/arm/BufferARM.h"
#include "CommonLib/arm/CommonDefARM.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM_SVE2 ) && ENABLE_SIMD_OPT_BUFFER

#include "CommonLib/arm/neon/mem_neon.h"
#include "CommonLib/arm/sve2/tbl_sve2.h"

namespace vvdec
{

void rspFwdCore_sve2( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const int bd, const Pel OrgCW,
                      const Pel* LmcsPivot, const Pel* ScaleCoeff, const Pel* InputPivot )
{
  CHECKD( svcnth() != 8, "rspFwdCore_sve2 expects 8 halfword (128-bit) SVE vectors, but svcnth() == " << svcnth() );

  int shift = getLog2( OrgCW );

  const int16x8x2_t mLmcsPivot = load_s16x8_x2( LmcsPivot );
  const int16x8x2_t mInputPivot = load_s16x8_x2( InputPivot );
  const int16x8x2_t mScaleCoeff = load_s16x8_x2( ScaleCoeff );

  const int16x8_t mMin = vdupq_n_s16( 0 );
  const int16x8_t mMax = vdupq_n_s16( ( 1 << bd ) - 1 );

  if( ( width & 7 ) == 0 )
  {
    do
    {
      int w = 0;
      do
      {
        const int16x8_t xsrc = vld1q_s16( &ptr[w] );

        // ( idxY = ptr[w] >> shift ) range is [0, 15].
        uint16x8_t idxY = vreinterpretq_u16_s16( vshlq_s16( xsrc, vdupq_n_s16( -shift ) ) );

        const int16x8_t xlmc = vvdec_svtbl2_s16( mLmcsPivot, idxY );
        const int16x8_t xinp = vvdec_svtbl2_s16( mInputPivot, idxY );
        const int16x8_t xscl = vvdec_svtbl2_s16( mScaleCoeff, idxY );

        int16x8_t diff = vqsubq_s16( xsrc, xinp );

        // vqrdmlahq uses Q15 rounding: (a * b) >> 15. We need (diff * xscl) >> 11,
        // so pre-scale diff by 2^4 to make (diff << 4) * xscl >> 15 == diff * xscl >> 11.
        int16x8_t diff_shift4 = vshlq_n_s16( diff, 4 );
        int16x8_t xtmp1 = vqrdmlahq_s16( xlmc, diff_shift4, xscl );

        xtmp1 = vminq_s16( xtmp1, mMax );
        xtmp1 = vmaxq_s16( xtmp1, mMin );

        vst1q_s16( &ptr[w], xtmp1 );

        w += 8;
      } while( w < width );
      ptr += ptrStride;
    } while( --height != 0 );
  }
  else // width == 4
  {
    CHECKD( width != 4, "rspFwdCore_sve2 else path expects width == 4" );

    do
    {
      const int16x4_t xsrc = vld1_s16( ptr );

      // ( idxY = ptr[w] >> shift ) range is [0, 15].
      uint16x4_t idxY_half = vreinterpret_u16_s16( vshl_s16( xsrc, vdup_n_s16( -shift ) ) );
      uint16x8_t idxY = vcombine_u16( idxY_half, vdup_n_u16( 0 ) );

      const int16x4_t xlmc = vget_low_s16( vvdec_svtbl2_s16( mLmcsPivot, idxY ) );
      const int16x4_t xinp = vget_low_s16( vvdec_svtbl2_s16( mInputPivot, idxY ) );
      const int16x4_t xscl = vget_low_s16( vvdec_svtbl2_s16( mScaleCoeff, idxY ) );

      int16x4_t diff = vqsub_s16( xsrc, xinp );

      // vqrdmlah uses Q15 rounding: (a * b) >> 15. We need (diff * xscl) >> 11,
      // so pre-scale diff by 2^4 to make (diff << 4) * xscl >> 15 == diff * xscl >> 11.
      int16x4_t diff_shift4 = vshl_n_s16( diff, 4 );
      int16x4_t xtmp1 = vqrdmlah_s16( xlmc, diff_shift4, xscl );

      xtmp1 = vmin_s16( xtmp1, vget_low_s16( mMax ) );
      xtmp1 = vmax_s16( xtmp1, vget_low_s16( mMin ) );

      vst1_s16( ptr, xtmp1 );

      ptr += ptrStride;
    } while( --height != 0 );
  }
}

template<>
void PelBufferOps::_initPelBufOpsARM<SVE2>()
{
  // Enable only if the CPU supports 8 halfword (128-bit) SVE vectors.
  if( svcnth() == 8 )
  {
    rspFwd = rspFwdCore_sve2;
  }
}

} // namespace vvdec

#endif // defined( TARGET_SIMD_ARM_SVE2 ) && ENABLE_SIMD_OPT_BUFFER

//! \}
