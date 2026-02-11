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
 * \file Buffer_neon_rdm.cpp
 * \brief Neon buffer operations with RDM (Rounding Doubling Multiply).
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "CommonLib/CommonDef.h"
#include "CommonLib/arm/BufferARM.h"
#include "CommonLib/arm/CommonDefARM.h"
#include "CommonLib/arm/neon/mem_neon.h"
#include "CommonLib/arm/neon/tbl_neon.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM_RDM ) && ENABLE_SIMD_OPT_BUFFER

namespace vvdec
{

void rspFwdCore_neon_rdm( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const int bd, const Pel OrgCW,
                          const Pel* LmcsPivot, const Pel* ScaleCoeff, const Pel* InputPivot )
{
  int shift = getLog2( OrgCW );

  const int8_t* lmcsPivotBytes = ( const int8_t* )LmcsPivot;
  const int8_t* inputPivotBytes = ( const int8_t* )InputPivot;
  const int8_t* scaleCoeffBytes = ( const int8_t* )ScaleCoeff;
  const int8x16x2_t mLmcsPivot = load_s8x16_x2( lmcsPivotBytes );
  const int8x16x2_t mInputPivot = load_s8x16_x2( inputPivotBytes );
  const int8x16x2_t mScaleCoeff = load_s8x16_x2( scaleCoeffBytes );

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
        // Multiply by 2 so we can use vqtbl2q lookup with 32-byte (16 * 2) tables.
        uint8x16_t idxY = vreinterpretq_u8_s16( vshlq_s16( xsrc, vdupq_n_s16( -shift ) ) );
        idxY = vshlq_n_u8( idxY, 1 );
        idxY = vtrnq_u8( idxY, vaddq_u8( idxY, vdupq_n_u8( 1 ) ) ).val[0]; // Add 1 for odd positions.

        const int8x16_t xlmc_s8 = vvdec_vqtbl2q_s8( mLmcsPivot, idxY );
        const int16x8_t xlmc = vreinterpretq_s16_s8( xlmc_s8 );

        const int8x16_t xinp_s8 = vvdec_vqtbl2q_s8( mInputPivot, idxY );
        const int16x8_t xinp = vreinterpretq_s16_s8( xinp_s8 );

        const int8x16_t xscl_s8 = vvdec_vqtbl2q_s8( mScaleCoeff, idxY );
        const int16x8_t xscl = vreinterpretq_s16_s8( xscl_s8 );

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
    CHECKD( width != 4, "rspFwdCore_neon_rdm else path expects width == 4" );

    do
    {
      const int16x4_t xsrc = vld1_s16( ptr );

      // ( idxY = ptr[w] >> shift ) range is [0, 15].
      // Multiply by 2 so we can use vqtbl2 lookup with 32-byte (16 * 2) tables.
      uint8x8_t idxY = vreinterpret_u8_s16( vshl_s16( xsrc, vdup_n_s16( -shift ) ) );
      idxY = vshl_n_u8( idxY, 1 );
      idxY = vtrn_u8( idxY, vadd_u8( idxY, vdup_n_u8( 1 ) ) ).val[0]; // Add 1 for odd positions.

      const int8x8_t xlmc_s8 = vvdec_vqtbl2_s8( mLmcsPivot, idxY );
      const int16x4_t xlmc = vreinterpret_s16_s8( xlmc_s8 );

      const int8x8_t xinp_s8 = vvdec_vqtbl2_s8( mInputPivot, idxY );
      const int16x4_t xinp = vreinterpret_s16_s8( xinp_s8 );

      const int8x8_t xscl_s8 = vvdec_vqtbl2_s8( mScaleCoeff, idxY );
      const int16x4_t xscl = vreinterpret_s16_s8( xscl_s8 );

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
void PelBufferOps::_initPelBufOpsARM<NEON_RDM>()
{
  rspFwd = rspFwdCore_neon_rdm;
}

} // namespace vvdec

#endif // defined( TARGET_SIMD_ARM_RDM ) && ENABLE_SIMD_OPT_BUFFER
//! \}
