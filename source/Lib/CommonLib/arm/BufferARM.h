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

/** \file     BufferARM.h
    \brief    SIMD averaging.
*/

//! \ingroup CommonLib
//! \{

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP 1

#include "CommonDefARM.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/InterpolationFilter.h"
#include "CommonLib/Unit.h"

#if ENABLE_SIMD_OPT_BUFFER
#  ifdef TARGET_SIMD_ARM

namespace vvdec
{

#  if __ARM_ARCH >= 8

template<ARM_VEXT vext>
void rspBcwCore_SIMD( Pel*       ptr,
                      ptrdiff_t  ptrStride,
                      int        width,
                      int        height,
                      const int  bd,
                      const int  minBin,
                      const int  maxBin,
                      const Pel* LmcsPivot,
                      const Pel* InvScCoeff,
                      const Pel* InputPivot )
{
  const int effMaxBin = maxBin < PIC_CODE_CW_BINS - 1 ? maxBin + 1 : maxBin;

  if( ( width & 7 ) == 0 )
  {
    int8x16x2_t     xtmp1         = vld2q_s8( (const signed char*) &InputPivot[ 0 ] );
    const int8x16_t mInputPivotLo = xtmp1.val[ 0 ];
    const int8x16_t mInputPivotHi = xtmp1.val[ 1 ];

    xtmp1                         = vld2q_s8( (const signed char*) &InvScCoeff[ 0 ] );
    const int8x16_t mScaleCoeffLo = xtmp1.val[ 0 ];
    const int8x16_t mScaleCoeffHi = xtmp1.val[ 1 ];

    const int16x8_t mMin = vdupq_n_s16( 0 );
    const int16x8_t mMax = vdupq_n_s16( ( 1 << bd ) - 1 );

    int16x8_t xtmp2;

    const uint8x16_t idx4idx = { 0, 2, 4, 6, 8, 10, 12, 14, 0, 0, 0, 0, 0, 0, 0, 0 };

    for( int y = 0; y < height; y += 4 )
    {
      for( int x = 0; x < width; x += 8 )
      {
        const int16x8_t xsrc0 = vld1q_s16( ptr + x + 0 * ptrStride );
        const int16x8_t xsrc1 = vld1q_s16( ptr + x + 1 * ptrStride );
        const int16x8_t xsrc2 = vld1q_s16( ptr + x + 2 * ptrStride );
        const int16x8_t xsrc3 = vld1q_s16( ptr + x + 3 * ptrStride );

        // die kleinste Zahl aus dem aktuellen Bildbereich extrahieren
        const Pel min = vminvq_s16( vpminq_s16( vpminq_s16( xsrc0, xsrc1 ), vpminq_s16( xsrc2, xsrc3 ) ) );

        int i = minBin;
        switch( effMaxBin - minBin )
        {
        default:
        case 15: if( min < LmcsPivot[++i] ) { break; };
        case 14: if( min < LmcsPivot[++i] ) { break; };
        case 13: if( min < LmcsPivot[++i] ) { break; };
        case 12: if( min < LmcsPivot[++i] ) { break; };
        case 11: if( min < LmcsPivot[++i] ) { break; };
        case 10: if( min < LmcsPivot[++i] ) { break; };
        case  9: if( min < LmcsPivot[++i] ) { break; };
        case  8: if( min < LmcsPivot[++i] ) { break; };
        case  7: if( min < LmcsPivot[++i] ) { break; };
        case  6: if( min < LmcsPivot[++i] ) { break; };
        case  5: if( min < LmcsPivot[++i] ) { break; };
        case  4: if( min < LmcsPivot[++i] ) { break; };
        case  3: if( min < LmcsPivot[++i] ) { break; };
        case  2: if( min < LmcsPivot[++i] ) { break; };
        case  1: if( min < LmcsPivot[++i] ) { break; };
        case  0: if( min < LmcsPivot[++i] ) { break; };
        }
        --i;

        int16x8_t xidx0 = vdupq_n_s16( i );
        int16x8_t xidx1 = xidx0;
        int16x8_t xidx2 = xidx0;
        int16x8_t xidx3 = xidx0;

        int16x8_t xlmcs = vdupq_n_s16( LmcsPivot[ i ] );

        int16x8_t diff0 = vsubq_s16( xsrc0, xlmcs );
        int16x8_t diff1 = vsubq_s16( xsrc1, xlmcs );
        int16x8_t diff2 = vsubq_s16( xsrc2, xlmcs );
        int16x8_t diff3 = vsubq_s16( xsrc3, xlmcs );

        for( i++; i <= effMaxBin; i++ )
        {
          xlmcs = vdupq_n_s16( LmcsPivot[ i ] );

          int16x8_t currd = vsubq_s16( xsrc0, xlmcs );
          diff0           = (int16x8_t) vminq_u16( (uint16x8_t) diff0, (uint16x8_t) currd );
          int16x8_t chnd0 = (int16x8_t) vceqq_s16( currd, diff0 );

          currd           = vsubq_s16( xsrc1, xlmcs );
          diff1           = (int16x8_t) vminq_u16( (uint16x8_t) diff1, (uint16x8_t) currd );
          int16x8_t chnd1 = (int16x8_t) vceqq_s16( currd, diff1 );

          currd           = vsubq_s16( xsrc2, xlmcs );
          diff2           = (int16x8_t) vminq_u16( (uint16x8_t) diff2, (uint16x8_t) currd );
          int16x8_t chnd2 = (int16x8_t) vceqq_s16( currd, diff2 );

          currd           = vsubq_s16( xsrc3, xlmcs );
          diff3           = (int16x8_t) vminq_u16( (uint16x8_t) diff3, (uint16x8_t) currd );
          int16x8_t chnd3 = (int16x8_t) vceqq_s16( currd, diff3 );

          xidx0 = vsubq_s16( xidx0, chnd0 );
          xidx1 = vsubq_s16( xidx1, chnd1 );
          xidx2 = vsubq_s16( xidx2, chnd2 );
          xidx3 = vsubq_s16( xidx3, chnd3 );

          chnd0 = vorrq_s16( chnd0, chnd1 );
          chnd2 = vorrq_s16( chnd2, chnd3 );
          chnd0 = vorrq_s16( chnd0, chnd2 );

          if( !vmaxvq_u16( (uint16x8_t) chnd0 ) )
            break;
        }

        const uint8x16_t xidx0_8 = vqtbl1q_u8( (uint8x16_t) xidx0, idx4idx );
        const uint8x16_t xidx1_8 = vqtbl1q_u8( (uint8x16_t) xidx1, idx4idx );
        const uint8x16_t xidx2_8 = vqtbl1q_u8( (uint8x16_t) xidx2, idx4idx );
        const uint8x16_t xidx3_8 = vqtbl1q_u8( (uint8x16_t) xidx3, idx4idx );

        int16x8_t xinp = (int16x8_t)vzip1q_s8( vqtbl1q_s8( mInputPivotLo, xidx0_8 ), vqtbl1q_s8( mInputPivotHi, xidx0_8 ) );
        int16x8_t xscl = (int16x8_t)vzip1q_s8( vqtbl1q_s8( mScaleCoeffLo, xidx0_8 ), vqtbl1q_s8( mScaleCoeffHi, xidx0_8 ) );

        xtmp2 = vcombine_s16( vrshrn_n_s32( vmull_s16( vget_low_s16( diff0 ), vget_low_s16( xscl ) ), 11 ),
                              vrshrn_n_s32( vmull_s16( vget_high_s16( diff0 ), vget_high_s16( xscl ) ), 11 ) );

        xtmp2 = vaddq_s16( xinp, xtmp2 );

        xtmp2 = vminq_s16( xtmp2, mMax );
        xtmp2 = vmaxq_s16( xtmp2, mMin );

        vst1q_s16( &ptr[ x ], xtmp2 );

        xinp = (int16x8_t)vzip1q_s8( vqtbl1q_s8( mInputPivotLo, xidx1_8 ), vqtbl1q_s8( mInputPivotHi, xidx1_8 ) );
        xscl = (int16x8_t)vzip1q_s8( vqtbl1q_s8( mScaleCoeffLo, xidx1_8 ), vqtbl1q_s8( mScaleCoeffHi, xidx1_8 ) );

        xtmp2 = vcombine_s16( vrshrn_n_s32( vmull_s16( vget_low_s16( diff1 ), vget_low_s16( xscl ) ), 11 ),
                              vrshrn_n_s32( vmull_s16( vget_high_s16( diff1 ), vget_high_s16( xscl ) ), 11 ) );

        xtmp2 = vaddq_s16( xinp, xtmp2 );

        xtmp2 = vminq_s16( xtmp2, mMax );
        xtmp2 = vmaxq_s16( xtmp2, mMin );

        vst1q_s16( &ptr[ x + ptrStride ], xtmp2 );

        xinp = (int16x8_t)vzip1q_s8( vqtbl1q_s8( mInputPivotLo, xidx2_8 ), vqtbl1q_s8( mInputPivotHi, xidx2_8 ) );
        xscl = (int16x8_t)vzip1q_s8( vqtbl1q_s8( mScaleCoeffLo, xidx2_8 ), vqtbl1q_s8( mScaleCoeffHi, xidx2_8 ) );

        xtmp2 = vcombine_s16( vrshrn_n_s32( vmull_s16( vget_low_s16( diff2 ), vget_low_s16( xscl ) ), 11 ),
                              vrshrn_n_s32( vmull_s16( vget_high_s16( diff2 ), vget_high_s16( xscl ) ), 11 ) );

        xtmp2 = vaddq_s16( xinp, xtmp2 );

        xtmp2 = vminq_s16( xtmp2, mMax );
        xtmp2 = vmaxq_s16( xtmp2, mMin );

        vst1q_s16( &ptr[ x + 2 * ptrStride ], xtmp2 );

        xinp = (int16x8_t)vzip1q_s8( vqtbl1q_s8( mInputPivotLo, xidx3_8 ), vqtbl1q_s8( mInputPivotHi, xidx3_8 ) );
        xscl = (int16x8_t)vzip1q_s8( vqtbl1q_s8( mScaleCoeffLo, xidx3_8 ), vqtbl1q_s8( mScaleCoeffHi, xidx3_8 ) );

        xtmp2 = vcombine_s16( vrshrn_n_s32( vmull_s16( vget_low_s16( diff3 ), vget_low_s16( xscl ) ), 11 ),
                              vrshrn_n_s32( vmull_s16( vget_high_s16( diff3 ), vget_high_s16( xscl ) ), 11 ) );

        xtmp2 = vaddq_s16( xinp, xtmp2 );

        xtmp2 = vminq_s16( xtmp2, mMax );
        xtmp2 = vmaxq_s16( xtmp2, mMin );

        vst1q_s16( &ptr[ x + 3 * ptrStride ], xtmp2 );
      }

      ptr += ptrStride << 2;
    }
  }
}

#endif // __ARM_ARCH >= 8

}   // namespace vvdec

#  endif   // TARGET_SIMD_ARM
#endif     // ENABLE_SIMD_OPT_BUFFER
//! \}
