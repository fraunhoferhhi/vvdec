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
 * \file InterpolationFilter_neon.cpp
 * \brief Neon implementation of InterpolationFilter for AArch64.
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "CommonLib/CommonDef.h"
#include "CommonLib/InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCIF

#include "../InterpolationFilter_neon.h"
#include "mem_neon.h"
#include "sum_neon.h"

namespace vvdec
{

static void simdInterpolateN2_2D_neon( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *ch, TFilterCoeff const *cv )
{
  const int shift1st  = IF_FILTER_PREC_BILINEAR - ( IF_INTERNAL_PREC_BILINEAR - clpRng.bd );
  const int offset1st = 1 << ( shift1st - 1 );

  const int shift2nd  = 4;
  const int offset2nd = 1 << ( shift2nd - 1 );

  int16x8_t mmOffset1 = vdupq_n_s16( offset1st );
  int16x8_t mmOffset2 = vdupq_n_s16( offset2nd );
  int16x8_t mmCoeffH  = vdupq_n_s16( ch[ 1 ] );
  int16x8_t mmCoeffV  = vdupq_n_s16( cv[ 1 ] );

  int16x8_t mmLastH[ 16 ];

  int16x8_t mmLast4H;

  // workaround for over-sensitive compilers
  mmLastH[ 0 ] = vdupq_n_s16( 0 );

  int16x8_t shift1inv = vdupq_n_s16( -shift1st );
  int16x8_t shift2inv = vdupq_n_s16( -shift2nd );

  for( int row = -1; row < height; row++ )
  {
    int16x8_t mmPix  = vld1q_s16( src );
    int16x8_t mmPix1 = vld1q_s16( src + 1 );

    int16x8_t mmFiltered = vmlaq_n_s16( mmOffset1, mmPix, 16 );

    mmFiltered = vmlaq_s16( mmFiltered, vsubq_s16( mmPix1, mmPix ), mmCoeffH );
    mmFiltered = vshlq_s16( mmFiltered, shift1inv );

    if( row >= 0 )
    {
      int16x8_t mmFiltered2 = vmlaq_n_s16( mmOffset2, mmLast4H, 16 );
      mmFiltered2           = vmlaq_s16( mmFiltered2, vsubq_s16( mmFiltered, mmLast4H ), mmCoeffV );
      mmFiltered2           = vshlq_s16( mmFiltered2, shift2inv );

      vst1q_lane_s64( (int64_t*) dst, (int64x2_t) mmFiltered2, 0 );
    }

    mmLast4H = mmFiltered;

    for( int x = 4; x < width; x += 8 )
    {
      int16x8_t mmPix  = vld1q_s16( src + x );
      int16x8_t mmPix1 = vld1q_s16( src + x + 1 );

      int16x8_t mmFiltered = vmlaq_n_s16( mmOffset1, mmPix, 16 );
      mmFiltered           = vmlaq_s16( mmFiltered, vsubq_s16( mmPix1, mmPix ), mmCoeffH );
      mmFiltered           = vshlq_s16( mmFiltered, shift1inv );

      int       idx   = x >> 3;
      int16x8_t mLast = mmLastH[ idx ];
      mmLastH[ idx ]  = mmFiltered;

      if( row >= 0 )
      {
        int16x8_t mmFiltered2 = vmlaq_n_s16( mmOffset2, mLast, 16 );
        mmFiltered2           = vmlaq_s16( mmFiltered2, vsubq_s16( mmFiltered, mLast ), mmCoeffV );
        mmFiltered2           = vshlq_s16( mmFiltered2, shift2inv );

        vst1q_s16( ( dst + x ), mmFiltered2 );
      }
    }
    if( row >= 0 )
      dst += dstStride;

    src += srcStride;
  }
}

static inline int16x4_t filter_horiz_4x1_N8_neon( Pel const* src, int16x8_t ch, int32x4_t voffset1,
                                                  int32x4_t invshift1st )
{
  int16x8_t vsrca0 = vld1q_s16( src + 0 );
  int16x8_t vsrca1 = vld1q_s16( src + 1 );
  int16x8_t vsrca2 = vld1q_s16( src + 2 );
  int16x8_t vsrca3 = vld1q_s16( src + 3 );

  int32x4_t a0 = vmull_s16( vget_low_s16( vsrca0 ), vget_low_s16( ch ) );
  int32x4_t a1 = vmull_s16( vget_low_s16( vsrca1 ), vget_low_s16( ch ) );
  int32x4_t a2 = vmull_s16( vget_low_s16( vsrca2 ), vget_low_s16( ch ) );
  int32x4_t a3 = vmull_s16( vget_low_s16( vsrca3 ), vget_low_s16( ch ) );

  a0 = vmlal_s16( a0, vget_high_s16( vsrca0 ), vget_high_s16( ch ) );
  a1 = vmlal_s16( a1, vget_high_s16( vsrca1 ), vget_high_s16( ch ) );
  a2 = vmlal_s16( a2, vget_high_s16( vsrca2 ), vget_high_s16( ch ) );
  a3 = vmlal_s16( a3, vget_high_s16( vsrca3 ), vget_high_s16( ch ) );

  int32x4_t vsuma = horizontal_add_4d_s32x4( a0, a1, a2, a3 );
  vsuma = vaddq_s32( vsuma, voffset1 );
  vsuma = vshlq_s32( vsuma, invshift1st );
  return vqmovn_s32( vsuma );
}

static inline int16x8_t filter_horiz_8x1_N8_neon( Pel const* src, int16x8_t ch, int32x4_t voffset1,
                                                  int32x4_t invshift1st )
{
  int16x4_t lo = filter_horiz_4x1_N8_neon( src + 0, ch, voffset1, invshift1st );
  int16x4_t hi = filter_horiz_4x1_N8_neon( src + 4, ch, voffset1, invshift1st );
  return vcombine_s16( lo, hi );
}

static inline int16x8x2_t filter_horiz_16x1_N8_neon( Pel const* src, int16x8_t ch, int32x4_t voffset1,
                                                     int32x4_t invshift1st )
{
  int16x8x2_t result;
  result.val[0] = filter_horiz_8x1_N8_neon( src + 0, ch, voffset1, invshift1st );
  result.val[1] = filter_horiz_8x1_N8_neon( src + 8, ch, voffset1, invshift1st );
  return result; // explicit return since MSVC for arm64 does not support direct return with typecast
}

template<bool isLast>
void simdFilter4x4_N6_neon( const ClpRng& clpRng, Pel const* src, const ptrdiff_t srcStride, Pel* dst,
                            const ptrdiff_t dstStride, int width, int height, TFilterCoeff const* coeffH,
                            TFilterCoeff const* coeffV )
{
  CHECKD( width != 4, "Width must be 4" );
  CHECKD( height != 4, "Height must be 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -3, -2 );

  const int headRoom = IF_INTERNAL_PREC - clpRng.bd;
  const int shift1st = IF_FILTER_PREC - headRoom;
  const int shift2nd = IF_FILTER_PREC + headRoom;

  const int offset1st = -IF_INTERNAL_OFFS * ( 1 << shift1st );
  int offset2nd;
  if( isLast )
  {
    offset2nd = ( 1 << ( shift2nd - 1 ) ) + ( IF_INTERNAL_OFFS << IF_FILTER_PREC );
  }
  else
  {
    offset2nd = 0;
  }
  const int32x4_t voffset1 = vdupq_n_s32( offset1st );
  const int32x4_t voffset2 = vdupq_n_s32( offset2nd );
  const int16x4_t vibdimax = vdup_n_s16( clpRng.max() );

  int16x8_t ch = vld1q_s16( coeffH );
  int16x8_t cv = vld1q_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x4_t vsrcv[9];
  vsrcv[0] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[3] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[4] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[5] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[6] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[7] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[8] = filter_horiz_4x1_N8_neon( src, ch, voffset1, invshift1st );

  int h = 0;
  do
  {
    int32x4_t vsum0 = filter_vert_4x1_N6_neon( vsrcv + h, cv, voffset2 );

    int16x4_t vsum01;
    if( isLast ) // clip
    {
      uint16x4_t usum01 = vqmovun_s32( vshlq_s32( vsum0, invshift2nd ) );
      vsum01 = vmin_s16( vibdimax, vreinterpret_s16_u16( usum01 ) );
    }
    else
    {
      vsum01 = vqshrn_n_s32( vsum0, IF_FILTER_PREC );
    }

    vst1_s16( dst, vsum01 );

    dst += dstStride;
  } while( ++h != 4 );
}

template<bool isLast>
void simdFilter8xH_N8_neon( const ClpRng& clpRng, Pel const* src, const ptrdiff_t srcStride, Pel* dst,
                            const ptrdiff_t dstStride, int width, int height, TFilterCoeff const* coeffH,
                            TFilterCoeff const* coeffV )
{
  CHECKD( width != 8, "Width must be 8" );
  CHECKD( height < 4, "Height must be >= 4" );
  CHECKD( height % 4 != 0, "Height must be a multiple of 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -3, -3 );

  const int headRoom = IF_INTERNAL_PREC - clpRng.bd;
  const int shift1st = IF_FILTER_PREC - headRoom;
  const int shift2nd = IF_FILTER_PREC + headRoom;

  const int offset1st = -IF_INTERNAL_OFFS * ( 1 << shift1st );
  int offset2nd;
  if( isLast )
  {
    offset2nd = ( 1 << ( shift2nd - 1 ) ) + ( IF_INTERNAL_OFFS << IF_FILTER_PREC );
  }
  else
  {
    offset2nd = 0;
  }
  const int32x4_t voffset1 = vdupq_n_s32( offset1st );
  const int32x4_t voffset2 = vdupq_n_s32( offset2nd );
  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );

  int16x8_t ch = vld1q_s16( coeffH );
  int16x8_t cv = vld1q_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x8_t vsrcv[8];
  vsrcv[0] = filter_horiz_8x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_8x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_8x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[3] = filter_horiz_8x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[4] = filter_horiz_8x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[5] = filter_horiz_8x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[6] = filter_horiz_8x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;

  do
  {
    vsrcv[7] = filter_horiz_8x1_N8_neon( src, ch, voffset1, invshift1st );
    src += srcStride;

    int32x4x2_t vsum = filter_vert_8x1_N8_neon( vsrcv, cv, voffset2 );

    int16x8_t vsum01;
    if( isLast ) // clip
    {
      int32x4_t vsum0 = vshlq_s32( vsum.val[0], invshift2nd );
      int32x4_t vsum1 = vshlq_s32( vsum.val[1], invshift2nd );

      uint16x8_t usum01 = vcombine_u16( vqmovun_s32( vsum0 ), vqmovun_s32( vsum1 ) );
      vsum01 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum01 ) );
    }
    else
    {
      vsum01 = vcombine_s16( vqshrn_n_s32( vsum.val[0], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[1], IF_FILTER_PREC ) );
    }

    vsrcv[0] = vsrcv[1];
    vsrcv[1] = vsrcv[2];
    vsrcv[2] = vsrcv[3];
    vsrcv[3] = vsrcv[4];
    vsrcv[4] = vsrcv[5];
    vsrcv[5] = vsrcv[6];
    vsrcv[6] = vsrcv[7];

    vst1q_s16( dst, vsum01 );

    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
void simdFilter16xH_N8_neon( const ClpRng& clpRng, Pel const* src, const ptrdiff_t srcStride, Pel* dst,
                             const ptrdiff_t dstStride, int width, int height, TFilterCoeff const* coeffH,
                             TFilterCoeff const* coeffV )
{
  CHECKD( width != 16, "Width must be 16" );
  CHECKD( height < 4, "Height must be >= 4" );
  CHECKD( height % 4 != 0, "Height must be a multiple of 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -3, -3 );

  const int headRoom = IF_INTERNAL_PREC - clpRng.bd;
  const int shift1st = IF_FILTER_PREC - headRoom;
  const int shift2nd = IF_FILTER_PREC + headRoom;

  const int offset1st = -IF_INTERNAL_OFFS * ( 1 << shift1st );
  int offset2nd;
  if( isLast )
  {
    offset2nd = ( 1 << ( shift2nd - 1 ) ) + ( IF_INTERNAL_OFFS << IF_FILTER_PREC );
  }
  else
  {
    offset2nd = 0;
  }
  const int32x4_t voffset1 = vdupq_n_s32( offset1st );
  const int32x4_t voffset2 = vdupq_n_s32( offset2nd );
  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );

  int16x8_t ch = vld1q_s16( coeffH );
  int16x8_t cv = vld1q_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x8x2_t vsrcv[8];
  vsrcv[0] = filter_horiz_16x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_16x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_16x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[3] = filter_horiz_16x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[4] = filter_horiz_16x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[5] = filter_horiz_16x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[6] = filter_horiz_16x1_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;

  do
  {
    vsrcv[7] = filter_horiz_16x1_N8_neon( src, ch, voffset1, invshift1st );
    src += srcStride;

    int32x4x4_t vsum = filter_vert_16x1_N8_neon( vsrcv, cv, voffset2 );

    int16x8_t vsum01, vsum23;
    if( isLast ) // clip
    {
      int32x4_t vsum0 = vshlq_s32( vsum.val[0], invshift2nd );
      int32x4_t vsum1 = vshlq_s32( vsum.val[1], invshift2nd );
      int32x4_t vsum2 = vshlq_s32( vsum.val[2], invshift2nd );
      int32x4_t vsum3 = vshlq_s32( vsum.val[3], invshift2nd );

      uint16x8_t usum01 = vcombine_u16( vqmovun_s32( vsum0 ), vqmovun_s32( vsum1 ) );
      uint16x8_t usum23 = vcombine_u16( vqmovun_s32( vsum2 ), vqmovun_s32( vsum3 ) );

      vsum01 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum01 ) );
      vsum23 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum23 ) );
    }
    else
    {
      vsum01 = vcombine_s16( vqshrn_n_s32( vsum.val[0], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[1], IF_FILTER_PREC ) );
      vsum23 = vcombine_s16( vqshrn_n_s32( vsum.val[2], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[3], IF_FILTER_PREC ) );
    }

    vsrcv[0] = vsrcv[1];
    vsrcv[1] = vsrcv[2];
    vsrcv[2] = vsrcv[3];
    vsrcv[3] = vsrcv[4];
    vsrcv[4] = vsrcv[5];
    vsrcv[5] = vsrcv[6];
    vsrcv[6] = vsrcv[7];

    vst1q_s16( dst + 0, vsum01 );
    vst1q_s16( dst + 8, vsum23 );

    dst += dstStride;
  } while( --height != 0 );
}

static inline int16x4_t filter_horiz_4x1_N4_neon( Pel const* src, int16x4_t ch, int32x4_t voffset1,
                                                  int32x4_t invshift1st )
{
  int16x4_t vsrca0 = vld1_s16( src + 0 );
  int16x4_t vsrca1 = vld1_s16( src + 1 );
  int16x4_t vsrca2 = vld1_s16( src + 2 );
  int16x4_t vsrca3 = vld1_s16( src + 3 );

  int32x4_t a0 = vmull_s16( vsrca0, ch );
  int32x4_t a1 = vmull_s16( vsrca1, ch );
  int32x4_t a2 = vmull_s16( vsrca2, ch );
  int32x4_t a3 = vmull_s16( vsrca3, ch );

  int32x4_t vsuma = horizontal_add_4d_s32x4( a0, a1, a2, a3 );
  vsuma = vaddq_s32( vsuma, voffset1 );
  vsuma = vshlq_s32( vsuma, invshift1st );
  return vqmovn_s32( vsuma );
}

static inline int16x8_t filter_horiz_8x1_N4_neon( Pel const* src, int16x4_t ch, int32x4_t voffset1,
                                                  int32x4_t invshift1st )
{
  int16x4_t lo = filter_horiz_4x1_N4_neon( src + 0, ch, voffset1, invshift1st );
  int16x4_t hi = filter_horiz_4x1_N4_neon( src + 4, ch, voffset1, invshift1st );
  return vcombine_s16( lo, hi );
}

static inline int16x8x2_t filter_horiz_16x1_N4_neon( Pel const* src, int16x4_t ch, int32x4_t voffset1,
                                                     int32x4_t invshift1st )
{
  int16x8x2_t result;
  result.val[0] = filter_horiz_8x1_N4_neon( src + 0, ch, voffset1, invshift1st );
  result.val[1] = filter_horiz_8x1_N4_neon( src + 8, ch, voffset1, invshift1st );
  return result;
}

static inline int32x4_t filter_vert_4x1_N4_neon( int16x4_t const vsrc[4], int16x4_t cv, int32x4_t voffset2 )
{
  int32x4_t vsum = vmlal_lane_s16( voffset2, vsrc[0], cv, 0 );
  vsum = vmlal_lane_s16( vsum, vsrc[1], cv, 1 );
  vsum = vmlal_lane_s16( vsum, vsrc[2], cv, 2 );
  vsum = vmlal_lane_s16( vsum, vsrc[3], cv, 3 );
  return vsum;
}

static inline int32x4x2_t filter_vert_8x1_N4_neon( int16x8_t const vsrc[4], int16x4_t cv, int32x4_t voffset2 )
{
  int32x4x2_t vsum;
  vsum.val[0] = vmlal_lane_s16( voffset2, vget_low_s16( vsrc[0] ), cv, 0 );
  vsum.val[1] = vmlal_lane_s16( voffset2, vget_high_s16( vsrc[0] ), cv, 0 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[1] ), cv, 1 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[1] ), cv, 1 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[2] ), cv, 2 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[2] ), cv, 2 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[3] ), cv, 3 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[3] ), cv, 3 );
  return vsum;
}

static inline int32x4x4_t filter_vert_16x1_N4_neon( int16x8x2_t const vsrc[4], int16x4_t cv, int32x4_t voffset2 )
{
  int32x4x4_t vsum;
  vsum.val[0] = vmlal_lane_s16( voffset2, vget_low_s16( vsrc[0].val[0] ), cv, 0 );
  vsum.val[1] = vmlal_lane_s16( voffset2, vget_high_s16( vsrc[0].val[0] ), cv, 0 );
  vsum.val[2] = vmlal_lane_s16( voffset2, vget_low_s16( vsrc[0].val[1] ), cv, 0 );
  vsum.val[3] = vmlal_lane_s16( voffset2, vget_high_s16( vsrc[0].val[1] ), cv, 0 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[1].val[0] ), cv, 1 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[1].val[0] ), cv, 1 );
  vsum.val[2] = vmlal_lane_s16( vsum.val[2], vget_low_s16( vsrc[1].val[1] ), cv, 1 );
  vsum.val[3] = vmlal_lane_s16( vsum.val[3], vget_high_s16( vsrc[1].val[1] ), cv, 1 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[2].val[0] ), cv, 2 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[2].val[0] ), cv, 2 );
  vsum.val[2] = vmlal_lane_s16( vsum.val[2], vget_low_s16( vsrc[2].val[1] ), cv, 2 );
  vsum.val[3] = vmlal_lane_s16( vsum.val[3], vget_high_s16( vsrc[2].val[1] ), cv, 2 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[3].val[0] ), cv, 3 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[3].val[0] ), cv, 3 );
  vsum.val[2] = vmlal_lane_s16( vsum.val[2], vget_low_s16( vsrc[3].val[1] ), cv, 3 );
  vsum.val[3] = vmlal_lane_s16( vsum.val[3], vget_high_s16( vsrc[3].val[1] ), cv, 3 );
  return vsum;
}

template<bool isLast>
void simdFilter4x4_N4_neon( const ClpRng& clpRng, Pel const* src, ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride,
                            int width, int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  CHECKD( width != 4, "Width must be 4" );
  CHECKD( height != 4, "Height must be 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -1, -1 );

  const int headRoom = IF_INTERNAL_PREC - clpRng.bd;
  const int shift1st = IF_FILTER_PREC - headRoom;
  const int shift2nd = IF_FILTER_PREC + headRoom;

  const int offset1st = -IF_INTERNAL_OFFS * ( 1 << shift1st );
  int offset2nd;
  if( isLast )
  {
    offset2nd = ( 1 << ( shift2nd - 1 ) ) + ( IF_INTERNAL_OFFS << IF_FILTER_PREC );
  }
  else
  {
    offset2nd = 0;
  }
  const int32x4_t voffset1 = vdupq_n_s32( offset1st );
  const int32x4_t voffset2 = vdupq_n_s32( offset2nd );
  const int16x4_t vibdimax = vdup_n_s16( clpRng.max() );

  int16x4_t ch = vld1_s16( coeffH );
  int16x4_t cv = vld1_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x4_t vsrcv[7];
  vsrcv[0] = filter_horiz_4x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_4x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_4x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[3] = filter_horiz_4x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[4] = filter_horiz_4x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[5] = filter_horiz_4x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[6] = filter_horiz_4x1_N4_neon( src, ch, voffset1, invshift1st );

  int h = 0;
  do
  {
    int32x4_t vsum0 = filter_vert_4x1_N4_neon( vsrcv + h, cv, voffset2 );

    int16x4_t vsum01;
    if( isLast ) // clip
    {
      uint16x4_t usum01 = vqmovun_s32( vshlq_s32( vsum0, invshift2nd ) );
      vsum01 = vmin_s16( vibdimax, vreinterpret_s16_u16( usum01 ) );
    }
    else
    {
      vsum01 = vqshrn_n_s32( vsum0, IF_FILTER_PREC );
    }

    vst1_s16( dst, vsum01 );

    dst += dstStride;
  } while( ++h != 4 );
}

template<bool isLast>
void simdFilter8xH_N4_neon( const ClpRng& clpRng, Pel const* src, ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride,
                            int width, int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  CHECKD( width != 8, "Width must be 8" );
  CHECKD( height < 2, "Height must be >= 2" );
  CHECKD( height % 4 != 0 && height != 2, "Height must be a multiple of 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -1, -1 );

  const int headRoom = IF_INTERNAL_PREC - clpRng.bd;
  const int shift1st = IF_FILTER_PREC - headRoom;
  const int shift2nd = IF_FILTER_PREC + headRoom;

  const int offset1st = -IF_INTERNAL_OFFS * ( 1 << shift1st );
  int offset2nd;
  if( isLast )
  {
    offset2nd = ( 1 << ( shift2nd - 1 ) ) + ( IF_INTERNAL_OFFS << IF_FILTER_PREC );
  }
  else
  {
    offset2nd = 0;
  }
  const int32x4_t voffset1 = vdupq_n_s32( offset1st );
  const int32x4_t voffset2 = vdupq_n_s32( offset2nd );
  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );

  int16x4_t ch = vld1_s16( coeffH );
  int16x4_t cv = vld1_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x8_t vsrcv[7];
  vsrcv[0] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;

  if( height >= 4 )
  {
    do
    {
      vsrcv[3] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );
      src += srcStride;
      vsrcv[4] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );
      src += srcStride;
      vsrcv[5] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );
      src += srcStride;
      vsrcv[6] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );
      src += srcStride;

      int h = 0;
      do
      {
        int32x4x2_t vsum = filter_vert_8x1_N4_neon( vsrcv + h, cv, voffset2 );

        int16x8_t vsum01;
        if( isLast ) // clip
        {
          int32x4_t vsum0 = vshlq_s32( vsum.val[0], invshift2nd );
          int32x4_t vsum1 = vshlq_s32( vsum.val[1], invshift2nd );

          uint16x8_t usum01 = vcombine_u16( vqmovun_s32( vsum0 ), vqmovun_s32( vsum1 ) );
          vsum01 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum01 ) );
        }
        else
        {
          vsum01 =
              vcombine_s16( vqshrn_n_s32( vsum.val[0], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[1], IF_FILTER_PREC ) );
        }

        vst1q_s16( dst, vsum01 );

        dst += dstStride;
      } while( ++h != 4 );

      vsrcv[0] = vsrcv[4];
      vsrcv[1] = vsrcv[5];
      vsrcv[2] = vsrcv[6];

      height -= 4;
    } while( height != 0 );
  }
  else // height == 2
  {
    vsrcv[3] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );
    src += srcStride;
    vsrcv[4] = filter_horiz_8x1_N4_neon( src, ch, voffset1, invshift1st );

    int h = 0;
    do
    {
      int32x4x2_t vsum = filter_vert_8x1_N4_neon( vsrcv + h, cv, voffset2 );

      int16x8_t vsum01;
      if( isLast ) // clip
      {
        int32x4_t vsum0 = vshlq_s32( vsum.val[0], invshift2nd );
        int32x4_t vsum1 = vshlq_s32( vsum.val[1], invshift2nd );

        uint16x8_t usum01 = vcombine_u16( vqmovun_s32( vsum0 ), vqmovun_s32( vsum1 ) );
        vsum01 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum01 ) );
      }
      else
      {
        vsum01 =
            vcombine_s16( vqshrn_n_s32( vsum.val[0], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[1], IF_FILTER_PREC ) );
      }

      vst1q_s16( dst, vsum01 );

      dst += dstStride;
    } while( ++h != 2 );
  }
}

template<bool isLast>
void simdFilter16xH_N4_neon( const ClpRng& clpRng, Pel const* src, ptrdiff_t srcStride, Pel* dst, ptrdiff_t dstStride,
                             int width, int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  CHECKD( width != 16, "Width must be 16" );
  CHECKD( height < 2, "Height must be >= 2" );
  CHECKD( height % 4 != 0 && height != 2, "Height must be a multiple of 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -1, -1 );

  const int headRoom = IF_INTERNAL_PREC - clpRng.bd;
  const int shift1st = IF_FILTER_PREC - headRoom;
  const int shift2nd = IF_FILTER_PREC + headRoom;

  const int offset1st = -IF_INTERNAL_OFFS * ( 1 << shift1st );
  int offset2nd;
  if( isLast )
  {
    offset2nd = ( 1 << ( shift2nd - 1 ) ) + ( IF_INTERNAL_OFFS << IF_FILTER_PREC );
  }
  else
  {
    offset2nd = 0;
  }
  const int32x4_t voffset1 = vdupq_n_s32( offset1st );
  const int32x4_t voffset2 = vdupq_n_s32( offset2nd );
  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );

  int16x4_t ch = vld1_s16( coeffH );
  int16x4_t cv = vld1_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x8x2_t vsrcv[7];
  vsrcv[0] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );
  src += srcStride;

  if( height >= 4 )
  {
    do
    {
      vsrcv[3] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );
      src += srcStride;
      vsrcv[4] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );
      src += srcStride;
      vsrcv[5] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );
      src += srcStride;
      vsrcv[6] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );
      src += srcStride;

      int h = 0;
      do
      {
        int32x4x4_t vsum = filter_vert_16x1_N4_neon( vsrcv + h, cv, voffset2 );

        int16x8_t vsum01, vsum23;
        if( isLast ) // clip
        {
          int32x4_t vsum0 = vshlq_s32( vsum.val[0], invshift2nd );
          int32x4_t vsum1 = vshlq_s32( vsum.val[1], invshift2nd );
          int32x4_t vsum2 = vshlq_s32( vsum.val[2], invshift2nd );
          int32x4_t vsum3 = vshlq_s32( vsum.val[3], invshift2nd );

          uint16x8_t usum01 = vcombine_u16( vqmovun_s32( vsum0 ), vqmovun_s32( vsum1 ) );
          uint16x8_t usum23 = vcombine_u16( vqmovun_s32( vsum2 ), vqmovun_s32( vsum3 ) );

          vsum01 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum01 ) );
          vsum23 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum23 ) );
        }
        else
        {
          vsum01 =
              vcombine_s16( vqshrn_n_s32( vsum.val[0], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[1], IF_FILTER_PREC ) );
          vsum23 =
              vcombine_s16( vqshrn_n_s32( vsum.val[2], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[3], IF_FILTER_PREC ) );
        }

        vst1q_s16( dst + 0, vsum01 );
        vst1q_s16( dst + 8, vsum23 );

        dst += dstStride;
      } while( ++h != 4 );

      vsrcv[0] = vsrcv[4];
      vsrcv[1] = vsrcv[5];
      vsrcv[2] = vsrcv[6];

      height -= 4;
    } while( height != 0 );
  }
  else // height == 2
  {
    vsrcv[3] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );
    src += srcStride;
    vsrcv[4] = filter_horiz_16x1_N4_neon( src, ch, voffset1, invshift1st );

    int h = 0;
    do
    {
      int32x4x4_t vsum = filter_vert_16x1_N4_neon( vsrcv + h, cv, voffset2 );

      int16x8_t vsum01, vsum23;
      if( isLast ) // clip
      {
        int32x4_t vsum0 = vshlq_s32( vsum.val[0], invshift2nd );
        int32x4_t vsum1 = vshlq_s32( vsum.val[1], invshift2nd );
        int32x4_t vsum2 = vshlq_s32( vsum.val[2], invshift2nd );
        int32x4_t vsum3 = vshlq_s32( vsum.val[3], invshift2nd );

        uint16x8_t usum01 = vcombine_u16( vqmovun_s32( vsum0 ), vqmovun_s32( vsum1 ) );
        uint16x8_t usum23 = vcombine_u16( vqmovun_s32( vsum2 ), vqmovun_s32( vsum3 ) );

        vsum01 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum01 ) );
        vsum23 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum23 ) );
      }
      else
      {
        vsum01 =
            vcombine_s16( vqshrn_n_s32( vsum.val[0], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[1], IF_FILTER_PREC ) );
        vsum23 =
            vcombine_s16( vqshrn_n_s32( vsum.val[2], IF_FILTER_PREC ), vqshrn_n_s32( vsum.val[3], IF_FILTER_PREC ) );
      }

      vst1q_s16( dst + 0, vsum01 );
      vst1q_s16( dst + 8, vsum23 );

      dst += dstStride;
    } while( ++h != 2 );
  }
}

template<bool isLast>
void simdInterpolateHor_N8_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng& clpRng,
                                 int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 4, "Width must be >= 4" );
  CHECKD( width % 4 != 0, "Width must be a multiple of 4!" );

  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );
  const int32x4_t invshift = vdupq_n_s32( -shift );

  const int16x8_t vcoeff = vld1q_s16( coeff );

  do
  {
    int col = 0;
    for( ; col + 8 <= width; col += 8 )
    {
      int16x8_t vsrc0 = vld1q_s16( &src[col + 0] );
      int16x8_t vsrc1 = vld1q_s16( &src[col + 1] );
      int16x8_t vsrc2 = vld1q_s16( &src[col + 2] );
      int16x8_t vsrc3 = vld1q_s16( &src[col + 3] );
      int16x8_t vsrc4 = vld1q_s16( &src[col + 4] );
      int16x8_t vsrc5 = vld1q_s16( &src[col + 5] );
      int16x8_t vsrc6 = vld1q_s16( &src[col + 6] );
      int16x8_t vsrc7 = vld1q_s16( &src[col + 7] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc0 ), vget_low_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc1 ), vget_low_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc2 ), vget_low_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc3 ), vget_low_s16( vcoeff ), 3 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc4 ), vget_high_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc5 ), vget_high_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc6 ), vget_high_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc7 ), vget_high_s16( vcoeff ), 3 );

      int32x4_t vsumb = vdupq_n_s32( offset );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc0 ), vget_low_s16( vcoeff ), 0 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc1 ), vget_low_s16( vcoeff ), 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc2 ), vget_low_s16( vcoeff ), 2 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc3 ), vget_low_s16( vcoeff ), 3 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc4 ), vget_high_s16( vcoeff ), 0 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc5 ), vget_high_s16( vcoeff ), 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc6 ), vget_high_s16( vcoeff ), 2 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc7 ), vget_high_s16( vcoeff ), 3 );

      vsuma = vshlq_s32( vsuma, invshift );
      vsumb = vshlq_s32( vsumb, invshift );

      int16x8_t vsum = pack_sum_s32_to_s16x8<isLast>( vsuma, vsumb, vibdimax );

      vst1q_s16( &dst[col], vsum );
    }
    if( col != width ) // Last four samples.
    {
      int16x4_t vsrc0 = vld1_s16( &src[col + 0] );
      int16x4_t vsrc1 = vld1_s16( &src[col + 1] );
      int16x4_t vsrc2 = vld1_s16( &src[col + 2] );
      int16x4_t vsrc3 = vld1_s16( &src[col + 3] );
      int16x4_t vsrc4 = vld1_s16( &src[col + 4] );
      int16x4_t vsrc5 = vld1_s16( &src[col + 5] );
      int16x4_t vsrc6 = vld1_s16( &src[col + 6] );
      int16x4_t vsrc7 = vld1_s16( &src[col + 7] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vsrc0, vget_low_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vsrc1, vget_low_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc2, vget_low_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vsrc3, vget_low_s16( vcoeff ), 3 );
      vsuma = vmlal_lane_s16( vsuma, vsrc4, vget_high_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vsrc5, vget_high_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc6, vget_high_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vsrc7, vget_high_s16( vcoeff ), 3 );

      vsuma = vshlq_s32( vsuma, invshift );

      int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vget_low_s16( vibdimax ) );

      vst1_s16( &dst[col], vsum );
    }

    src += srcStride;
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
void simdInterpolateHor_N6_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng& clpRng,
                                 int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 4, "Width must be >= 4" );
  CHECKD( width % 4 != 0, "Width must be a multiple of 4!" );

  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );
  const int32x4_t invshift = vdupq_n_s32( -shift );

  const int16x8_t vcoeff = vld1q_s16( coeff );

  do
  {
    int col = 0;
    for( ; col + 8 <= width; col += 8 )
    {
      // For 6-tap, src[0] and src[7] are multiplied by 0 so ignore them.
      int16x8_t vsrc1 = vld1q_s16( &src[col + 1] );
      int16x8_t vsrc2 = vld1q_s16( &src[col + 2] );
      int16x8_t vsrc3 = vld1q_s16( &src[col + 3] );
      int16x8_t vsrc4 = vld1q_s16( &src[col + 4] );
      int16x8_t vsrc5 = vld1q_s16( &src[col + 5] );
      int16x8_t vsrc6 = vld1q_s16( &src[col + 6] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc1 ), vget_low_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc2 ), vget_low_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc3 ), vget_low_s16( vcoeff ), 3 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc4 ), vget_high_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc5 ), vget_high_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc6 ), vget_high_s16( vcoeff ), 2 );

      int32x4_t vsumb = vdupq_n_s32( offset );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc1 ), vget_low_s16( vcoeff ), 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc2 ), vget_low_s16( vcoeff ), 2 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc3 ), vget_low_s16( vcoeff ), 3 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc4 ), vget_high_s16( vcoeff ), 0 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc5 ), vget_high_s16( vcoeff ), 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc6 ), vget_high_s16( vcoeff ), 2 );

      vsuma = vshlq_s32( vsuma, invshift );
      vsumb = vshlq_s32( vsumb, invshift );

      int16x8_t vsum = pack_sum_s32_to_s16x8<isLast>( vsuma, vsumb, vibdimax );

      vst1q_s16( &dst[col], vsum );
    }
    if( col != width ) // Last four samples.
    {
      // For 6-tap, src[0] and src[7] are multiplied by 0 so ignore them.
      int16x4_t vsrc1 = vld1_s16( &src[col + 1] );
      int16x4_t vsrc2 = vld1_s16( &src[col + 2] );
      int16x4_t vsrc3 = vld1_s16( &src[col + 3] );
      int16x4_t vsrc4 = vld1_s16( &src[col + 4] );
      int16x4_t vsrc5 = vld1_s16( &src[col + 5] );
      int16x4_t vsrc6 = vld1_s16( &src[col + 6] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vsrc1, vget_low_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc2, vget_low_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vsrc3, vget_low_s16( vcoeff ), 3 );
      vsuma = vmlal_lane_s16( vsuma, vsrc4, vget_high_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vsrc5, vget_high_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc6, vget_high_s16( vcoeff ), 2 );

      vsuma = vshlq_s32( vsuma, invshift );

      int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vget_low_s16( vibdimax ) );

      vst1_s16( &dst[col], vsum );
    }

    src += srcStride;
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
void simdInterpolateHor_N4_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng& clpRng,
                                 int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 4, "Width must be >= 4" );
  CHECKD( width % 4 != 0, "Width must be a multiple of 4!" );

  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );
  const int32x4_t invshift = vdupq_n_s32( -shift );

  const int16x4_t vcoeff = vld1_s16( coeff );

  do
  {
    int col = 0;
    for( ; col + 8 <= width; col += 8 )
    {
      int16x8_t vsrc0 = vld1q_s16( &src[col + 0] );
      int16x8_t vsrc1 = vld1q_s16( &src[col + 1] );
      int16x8_t vsrc2 = vld1q_s16( &src[col + 2] );
      int16x8_t vsrc3 = vld1q_s16( &src[col + 3] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc0 ), vcoeff, 0 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc1 ), vcoeff, 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc2 ), vcoeff, 2 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc3 ), vcoeff, 3 );

      int32x4_t vsumb = vdupq_n_s32( offset );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc0 ), vcoeff, 0 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc1 ), vcoeff, 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc2 ), vcoeff, 2 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc3 ), vcoeff, 3 );

      vsuma = vshlq_s32( vsuma, invshift );
      vsumb = vshlq_s32( vsumb, invshift );

      int16x8_t vsum = pack_sum_s32_to_s16x8<isLast>( vsuma, vsumb, vibdimax );

      vst1q_s16( &dst[col], vsum );
    }
    if( col != width ) // Last four samples.
    {
      int16x4_t vsrc0 = vld1_s16( &src[col + 0] );
      int16x4_t vsrc1 = vld1_s16( &src[col + 1] );
      int16x4_t vsrc2 = vld1_s16( &src[col + 2] );
      int16x4_t vsrc3 = vld1_s16( &src[col + 3] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vsrc0, vcoeff, 0 );
      vsuma = vmlal_lane_s16( vsuma, vsrc1, vcoeff, 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc2, vcoeff, 2 );
      vsuma = vmlal_lane_s16( vsuma, vsrc3, vcoeff, 3 );

      vsuma = vshlq_s32( vsuma, invshift );

      int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vget_low_s16( vibdimax ) );

      vst1_s16( &dst[col], vsum );
    }

    src += srcStride;
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
void simdInterpolateHorM2_N4_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst,
                                   const ptrdiff_t dstStride, int width, int height, int shift, int offset,
                                   const ClpRng& clpRng, int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width != 2, "Width must be two!" );

  const int16x4_t vibdimax = vdup_n_s16( clpRng.max() );
  const int32x4_t invshift = vdupq_n_s32( -shift );

  const int16x4_t vcoeff = vld1_s16( coeff );

  int row = 0;
  for( ; row + 2 <= height; row += 2 )
  {
    int16x4_t vsrc0 = load_s16x2x2( &src[0], srcStride );
    int16x4_t vsrc1 = load_s16x2x2( &src[1], srcStride );
    int16x4_t vsrc2 = load_s16x2x2( &src[2], srcStride );
    int16x4_t vsrc3 = load_s16x2x2( &src[3], srcStride );

    int32x4_t vsuma = vdupq_n_s32( offset );
    vsuma = vmlal_lane_s16( vsuma, vsrc0, vcoeff, 0 );
    vsuma = vmlal_lane_s16( vsuma, vsrc1, vcoeff, 1 );
    vsuma = vmlal_lane_s16( vsuma, vsrc2, vcoeff, 2 );
    vsuma = vmlal_lane_s16( vsuma, vsrc3, vcoeff, 3 );

    vsuma = vshlq_s32( vsuma, invshift );

    int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vibdimax );

    store_s16x2x2( &dst[0], vsum, dstStride );

    src += 2 * srcStride;
    dst += 2 * dstStride;
  }
  if( row != height )
  {
    int16x4_t vsrc0 = load_s16x2( &src[0] );
    int16x4_t vsrc1 = load_s16x2( &src[1] );
    int16x4_t vsrc2 = load_s16x2( &src[2] );
    int16x4_t vsrc3 = load_s16x2( &src[3] );

    int32x4_t vsuma = vdupq_n_s32( offset );
    vsuma = vmlal_lane_s16( vsuma, vsrc0, vcoeff, 0 );
    vsuma = vmlal_lane_s16( vsuma, vsrc1, vcoeff, 1 );
    vsuma = vmlal_lane_s16( vsuma, vsrc2, vcoeff, 2 );
    vsuma = vmlal_lane_s16( vsuma, vsrc3, vcoeff, 3 );

    vsuma = vshlq_s32( vsuma, invshift );

    int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vibdimax );

    store_s16x2( &dst[0], vsum );
  }
}

template<bool isLast>
void simdInterpolateVer_N8_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng& clpRng,
                                 int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 4, "Width must be >= 4" );
  CHECKD( width % 4 != 0, "Width must be a multiple of 4!" );

  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );
  const int32x4_t invshift = vdupq_n_s32( -shift );

  const int16x8_t vcoeff = vld1q_s16( coeff );

  do
  {
    int col = 0;
    for( ; col + 8 <= width; col += 8 )
    {
      int16x8_t vsrc0 = vld1q_s16( &src[col + 0 * srcStride] );
      int16x8_t vsrc1 = vld1q_s16( &src[col + 1 * srcStride] );
      int16x8_t vsrc2 = vld1q_s16( &src[col + 2 * srcStride] );
      int16x8_t vsrc3 = vld1q_s16( &src[col + 3 * srcStride] );
      int16x8_t vsrc4 = vld1q_s16( &src[col + 4 * srcStride] );
      int16x8_t vsrc5 = vld1q_s16( &src[col + 5 * srcStride] );
      int16x8_t vsrc6 = vld1q_s16( &src[col + 6 * srcStride] );
      int16x8_t vsrc7 = vld1q_s16( &src[col + 7 * srcStride] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc0 ), vget_low_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc1 ), vget_low_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc2 ), vget_low_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc3 ), vget_low_s16( vcoeff ), 3 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc4 ), vget_high_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc5 ), vget_high_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc6 ), vget_high_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc7 ), vget_high_s16( vcoeff ), 3 );

      int32x4_t vsumb = vdupq_n_s32( offset );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc0 ), vget_low_s16( vcoeff ), 0 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc1 ), vget_low_s16( vcoeff ), 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc2 ), vget_low_s16( vcoeff ), 2 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc3 ), vget_low_s16( vcoeff ), 3 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc4 ), vget_high_s16( vcoeff ), 0 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc5 ), vget_high_s16( vcoeff ), 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc6 ), vget_high_s16( vcoeff ), 2 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc7 ), vget_high_s16( vcoeff ), 3 );

      vsuma = vshlq_s32( vsuma, invshift );
      vsumb = vshlq_s32( vsumb, invshift );

      int16x8_t vsum = pack_sum_s32_to_s16x8<isLast>( vsuma, vsumb, vibdimax );

      vst1q_s16( &dst[col], vsum );
    }
    if( col != width ) // Last four samples.
    {
      int16x4_t vsrc0 = vld1_s16( &src[col + 0 * srcStride] );
      int16x4_t vsrc1 = vld1_s16( &src[col + 1 * srcStride] );
      int16x4_t vsrc2 = vld1_s16( &src[col + 2 * srcStride] );
      int16x4_t vsrc3 = vld1_s16( &src[col + 3 * srcStride] );
      int16x4_t vsrc4 = vld1_s16( &src[col + 4 * srcStride] );
      int16x4_t vsrc5 = vld1_s16( &src[col + 5 * srcStride] );
      int16x4_t vsrc6 = vld1_s16( &src[col + 6 * srcStride] );
      int16x4_t vsrc7 = vld1_s16( &src[col + 7 * srcStride] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vsrc0, vget_low_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vsrc1, vget_low_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc2, vget_low_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vsrc3, vget_low_s16( vcoeff ), 3 );
      vsuma = vmlal_lane_s16( vsuma, vsrc4, vget_high_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vsrc5, vget_high_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc6, vget_high_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vsrc7, vget_high_s16( vcoeff ), 3 );

      vsuma = vshlq_s32( vsuma, invshift );

      int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vget_low_s16( vibdimax ) );

      vst1_s16( &dst[col], vsum );
    }

    src += srcStride;
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
void simdInterpolateVer_N6_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng& clpRng,
                                 int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 4, "Width must be >= 4" );
  CHECKD( width % 4 != 0, "Width must be a multiple of 4!" );

  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );
  const int32x4_t invshift = vdupq_n_s32( -shift );

  const int16x8_t vcoeff = vld1q_s16( coeff );

  do
  {
    int col = 0;
    for( ; col + 8 <= width; col += 8 )
    {
      // For 6-tap, src[0] and src[7] are multiplied by 0 so ignore them.
      int16x8_t vsrc1 = vld1q_s16( &src[col + 1 * srcStride] );
      int16x8_t vsrc2 = vld1q_s16( &src[col + 2 * srcStride] );
      int16x8_t vsrc3 = vld1q_s16( &src[col + 3 * srcStride] );
      int16x8_t vsrc4 = vld1q_s16( &src[col + 4 * srcStride] );
      int16x8_t vsrc5 = vld1q_s16( &src[col + 5 * srcStride] );
      int16x8_t vsrc6 = vld1q_s16( &src[col + 6 * srcStride] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc1 ), vget_low_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc2 ), vget_low_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc3 ), vget_low_s16( vcoeff ), 3 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc4 ), vget_high_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc5 ), vget_high_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc6 ), vget_high_s16( vcoeff ), 2 );

      int32x4_t vsumb = vdupq_n_s32( offset );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc1 ), vget_low_s16( vcoeff ), 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc2 ), vget_low_s16( vcoeff ), 2 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc3 ), vget_low_s16( vcoeff ), 3 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc4 ), vget_high_s16( vcoeff ), 0 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc5 ), vget_high_s16( vcoeff ), 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc6 ), vget_high_s16( vcoeff ), 2 );

      vsuma = vshlq_s32( vsuma, invshift );
      vsumb = vshlq_s32( vsumb, invshift );

      int16x8_t vsum = pack_sum_s32_to_s16x8<isLast>( vsuma, vsumb, vibdimax );

      vst1q_s16( &dst[col], vsum );
    }
    if( col != width ) // Last four samples.
    {
      // For 6-tap, src[0] and src[7] are multiplied by 0 so ignore them.
      int16x4_t vsrc1 = vld1_s16( &src[col + 1 * srcStride] );
      int16x4_t vsrc2 = vld1_s16( &src[col + 2 * srcStride] );
      int16x4_t vsrc3 = vld1_s16( &src[col + 3 * srcStride] );
      int16x4_t vsrc4 = vld1_s16( &src[col + 4 * srcStride] );
      int16x4_t vsrc5 = vld1_s16( &src[col + 5 * srcStride] );
      int16x4_t vsrc6 = vld1_s16( &src[col + 6 * srcStride] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vsrc1, vget_low_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc2, vget_low_s16( vcoeff ), 2 );
      vsuma = vmlal_lane_s16( vsuma, vsrc3, vget_low_s16( vcoeff ), 3 );
      vsuma = vmlal_lane_s16( vsuma, vsrc4, vget_high_s16( vcoeff ), 0 );
      vsuma = vmlal_lane_s16( vsuma, vsrc5, vget_high_s16( vcoeff ), 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc6, vget_high_s16( vcoeff ), 2 );

      vsuma = vshlq_s32( vsuma, invshift );

      int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vget_low_s16( vibdimax ) );

      vst1_s16( &dst[col], vsum );
    }

    src += srcStride;
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
void simdInterpolateVer_N4_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst, const ptrdiff_t dstStride,
                                 int width, int height, int shift, int offset, const ClpRng& clpRng,
                                 int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 4, "Width must be >= 4" );
  CHECKD( width % 4 != 0, "Width must be a multiple of 4!" );

  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );
  const int32x4_t invshift = vdupq_n_s32( -shift );

  const int16x4_t vcoeff = vld1_s16( coeff );

  do
  {
    int col = 0;
    for( ; col + 8 <= width; col += 8 )
    {
      int16x8_t vsrc0 = vld1q_s16( &src[col + 0 * srcStride] );
      int16x8_t vsrc1 = vld1q_s16( &src[col + 1 * srcStride] );
      int16x8_t vsrc2 = vld1q_s16( &src[col + 2 * srcStride] );
      int16x8_t vsrc3 = vld1q_s16( &src[col + 3 * srcStride] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc0 ), vcoeff, 0 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc1 ), vcoeff, 1 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc2 ), vcoeff, 2 );
      vsuma = vmlal_lane_s16( vsuma, vget_low_s16( vsrc3 ), vcoeff, 3 );

      int32x4_t vsumb = vdupq_n_s32( offset );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc0 ), vcoeff, 0 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc1 ), vcoeff, 1 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc2 ), vcoeff, 2 );
      vsumb = vmlal_lane_s16( vsumb, vget_high_s16( vsrc3 ), vcoeff, 3 );

      vsuma = vshlq_s32( vsuma, invshift );
      vsumb = vshlq_s32( vsumb, invshift );

      int16x8_t vsum = pack_sum_s32_to_s16x8<isLast>( vsuma, vsumb, vibdimax );

      vst1q_s16( &dst[col], vsum );
    }
    if( col != width ) // Last four samples.
    {
      int16x4_t vsrc0 = vld1_s16( &src[col + 0 * srcStride] );
      int16x4_t vsrc1 = vld1_s16( &src[col + 1 * srcStride] );
      int16x4_t vsrc2 = vld1_s16( &src[col + 2 * srcStride] );
      int16x4_t vsrc3 = vld1_s16( &src[col + 3 * srcStride] );

      int32x4_t vsuma = vdupq_n_s32( offset );
      vsuma = vmlal_lane_s16( vsuma, vsrc0, vcoeff, 0 );
      vsuma = vmlal_lane_s16( vsuma, vsrc1, vcoeff, 1 );
      vsuma = vmlal_lane_s16( vsuma, vsrc2, vcoeff, 2 );
      vsuma = vmlal_lane_s16( vsuma, vsrc3, vcoeff, 3 );

      vsuma = vshlq_s32( vsuma, invshift );

      int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vget_low_s16( vibdimax ) );

      vst1_s16( &dst[col], vsum );
    }

    src += srcStride;
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
void simdInterpolateVerM2_N4_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst,
                                   const ptrdiff_t dstStride, int width, int height, int shift, int offset,
                                   const ClpRng& clpRng, int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width != 2, "Width must be two!" );

  const int16x4_t vibdimax = vdup_n_s16( clpRng.max() );
  const int32x4_t invshift = vdupq_n_s32( -shift );

  const int16x4_t vcoeff = vld1_s16( coeff );

  int16x4_t vsrc0 = load_s16x2x2( &src[0 * srcStride], srcStride );
  int16x4_t vsrc1 = load_s16x2x2( &src[1 * srcStride], srcStride );

  int row = 0;
  for( ; row + 2 <= height; row += 2 )
  {
    int16x4_t vsrc2 = load_s16x2x2( &src[2 * srcStride], srcStride );
    int16x4_t vsrc3 = load_s16x2x2( &src[3 * srcStride], srcStride );

    int32x4_t vsuma = vdupq_n_s32( offset );
    vsuma = vmlal_lane_s16( vsuma, vsrc0, vcoeff, 0 );
    vsuma = vmlal_lane_s16( vsuma, vsrc1, vcoeff, 1 );
    vsuma = vmlal_lane_s16( vsuma, vsrc2, vcoeff, 2 );
    vsuma = vmlal_lane_s16( vsuma, vsrc3, vcoeff, 3 );

    vsuma = vshlq_s32( vsuma, invshift );

    int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vibdimax );

    store_s16x2x2( &dst[0], vsum, dstStride );

    vsrc0 = vsrc2;
    vsrc1 = vsrc3;

    src += 2 * srcStride;
    dst += 2 * dstStride;
  }
  if( row != height )
  {
    int16x4_t vsrc2 = load_s16x2( &src[2 * srcStride] );
    int16x4_t vsrc3 = load_s16x2( &src[3 * srcStride] );

    int32x4_t vsuma = vdupq_n_s32( offset );
    vsuma = vmlal_lane_s16( vsuma, vsrc0, vcoeff, 0 );
    vsuma = vmlal_lane_s16( vsuma, vsrc1, vcoeff, 1 );
    vsuma = vmlal_lane_s16( vsuma, vsrc2, vcoeff, 2 );
    vsuma = vmlal_lane_s16( vsuma, vsrc3, vcoeff, 3 );

    vsuma = vshlq_s32( vsuma, invshift );

    int16x4_t vsum = pack_sum_s32_to_s16x4<isLast>( vsuma, vibdimax );

    store_s16x2( &dst[0], vsum );
  }
}

template<bool isVertical>
void simdInterpolate_N2_neon( const int16_t* src, const ptrdiff_t srcStride, int16_t* dst, const ptrdiff_t dstStride,
                              int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const* coeff )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 4, "Width must be >= 4" );
  CHECKD( width % 8 != 4, "Width must be a multiple of 4 but not 8!" );
  CHECKD( clpRng.bd > 10, "VVdeC does not support bitdepths larger than 10!" );
  CHECKD( std::max( coeff[0], coeff[1] ) > 16, "Bilinear coeff should not be greater than 16" );
  CHECKD( offset != 1 << ( shift - 1 ), "Offset must be equal to 1 << ( shift - 1 )" );

  const int16x8_t invshift = vdupq_n_s16( -shift );
  const int16x4_t vcoeff = load_s16x2( coeff );

  const ptrdiff_t cStride = isVertical ? srcStride : 1;

  do
  {
    int col = 0;
    for( ; col + 8 <= width; col += 8 )
    {
      int16x8_t vsrc0 = vld1q_s16( &src[col + 0 * cStride] );
      int16x8_t vsrc1 = vld1q_s16( &src[col + 1 * cStride] );

      int16x8_t vsuma = vmulq_lane_s16( vsrc0, vcoeff, 0 );
      vsuma = vmlaq_lane_s16( vsuma, vsrc1, vcoeff, 1 );

      // Use rounding shift since offset == 1 << ( shift - 1 ).
      vsuma = vrshlq_s16( vsuma, invshift );

      vst1q_s16( &dst[col], vsuma );
    }

    // Last four samples.
    int16x4_t vsrc0 = vld1_s16( &src[col + 0 * cStride] );
    int16x4_t vsrc1 = vld1_s16( &src[col + 1 * cStride] );

    int16x4_t vsuma = vmul_lane_s16( vsrc0, vcoeff, 0 );
    vsuma = vmla_lane_s16( vsuma, vsrc1, vcoeff, 1 );

    // Use rounding shift since offset == 1 << ( shift - 1 ).
    vsuma = vrshl_s16( vsuma, vget_low_s16( invshift ) );

    vst1_s16( &dst[col], vsuma );

    src += srcStride;
    dst += dstStride;
  } while( --height != 0 );
}

template<int N, bool isVertical, bool isFirst, bool isLast>
void simdFilter_neon( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst,
                      const ptrdiff_t dstStride, int width, int height, TFilterCoeff const* coeff )
{
  static_assert( N == 2 || N == 4 || N == 8, "Supported taps: 2/4/8" );
  CHECKD( height < 1, "Height must be >= 1!" );
  CHECKD( width < 1, "Width must be >= 1!" );
  CHECKD( clpRng.bd > 10, "VVdeC does not support bitdepths larger than 10!" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  const int16_t* c = coeff;

  const ptrdiff_t cStride = isVertical ? srcStride : 1;
  src -= ( N / 2 - 1 ) * cStride;

  int offset;
  int headRoom = IF_INTERNAL_PREC - clpRng.bd;
  int shift = IF_FILTER_PREC;

  bool is_N6 = false;
  if( N == 8 )
  {
    is_N6 = coeff[0] == 0 && coeff[7] == 0;
  }

  if( N != 2 )
  {
    if( isLast )
    {
      shift += isFirst ? 0 : headRoom;
      offset = 1 << ( shift - 1 );
      offset += isFirst ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
    }
    else
    {
      shift -= isFirst ? headRoom : 0;
      offset = isFirst ? -IF_INTERNAL_OFFS * ( 1 << shift ) : 0;
    }
  }
  else // N == 2
  {
    if( isFirst )
    {
      shift = IF_FILTER_PREC_BILINEAR - ( IF_INTERNAL_PREC_BILINEAR - clpRng.bd );
      offset = 1 << ( shift - 1 );
    }
    else
    {
      shift = 4;
      offset = 1 << ( shift - 1 );
    }
  }

  if( N == 8 )
  {
    CHECKD( width % 4 != 0 && width != 1, "N8 width must be 1 or multiple of 4! width=" << width );

    if( width % 4 == 0 )
    {
      if( is_N6 ) // 6-tap
      {
        if( !isVertical )
        {
          simdInterpolateHor_N6_neon<isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else
        {
          simdInterpolateVer_N6_neon<isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
      }
      else // 8-tap
      {
        if( !isVertical )
        {
          simdInterpolateHor_N8_neon<isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else
        {
          simdInterpolateVer_N8_neon<isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
      }

      return;
    }
  }

  if( N == 4 )
  {
    CHECKD( width % 4 != 0 && width != 2 && width != 1, "N4 width must be 1 or 2 or multiple of 4! width=" << width );

    if( width % 4 == 0 )
    {
      if( !isVertical )
      {
        simdInterpolateHor_N4_neon<isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
      {
        simdInterpolateVer_N4_neon<isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }

      return;
    }
    else if( width == 2 )
    {
      if( !isVertical )
      {
        simdInterpolateHorM2_N4_neon<isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
      {
        simdInterpolateVerM2_N4_neon<isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }

      return;
    }
  }

  if( N == 2 )
  {
    CHECKD( isLast, "isLast is not supported for 2-tap" );
    CHECKD( width % 8 != 4, "N2 width must be multiple of 4 but not 8! width=" << width );

    simdInterpolate_N2_neon<isVertical>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );

    return;
  }

  // Fallback to scalar code for width == 1.
  CHECKD( width != 1, "Width must be 1!" );

  if( !is_N6 ) // 4-tap or 8-tap
  {
    do
    {
      int sum = src[0 * cStride] * c[0];
      sum    += src[1 * cStride] * c[1];
      sum    += src[2 * cStride] * c[2];
      sum    += src[3 * cStride] * c[3];
      if( N == 8 )
      {
        sum += src[4 * cStride] * c[4];
        sum += src[5 * cStride] * c[5];
        sum += src[6 * cStride] * c[6];
        sum += src[7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[0] = val;

      src += srcStride;
      dst += dstStride;
    } while( --height != 0 );
  }
  else // 6-tap
  {
    do
    {
      // For 6-tap, src[0] and src[7] are multiplied by 0 so ignore them.
      int sum = src[1 * cStride] * c[1];
      sum    += src[2 * cStride] * c[2];
      sum    += src[3 * cStride] * c[3];
      sum    += src[4 * cStride] * c[4];
      sum    += src[5 * cStride] * c[5];
      sum    += src[6 * cStride] * c[6];

      Pel val = ( sum + offset ) >> shift;
      if( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[0] = val;

      src += srcStride;
      dst += dstStride;
    } while( --height != 0 );
  }
}

template<>
void InterpolationFilter::_initInterpolationFilterARM<NEON>()
{
  m_filter4x4[0][0] = simdFilter4x4_N6_neon<false>;
  m_filter4x4[0][1] = simdFilter4x4_N6_neon<true>;
  m_filter4x4[1][0] = simdFilter4x4_N4_neon<false>;
  m_filter4x4[1][1] = simdFilter4x4_N4_neon<true>;

  m_filter8xH[0][0] = simdFilter8xH_N8_neon<false>;
  m_filter8xH[0][1] = simdFilter8xH_N8_neon<true>;
  m_filter8xH[1][0] = simdFilter8xH_N4_neon<false>;
  m_filter8xH[1][1] = simdFilter8xH_N4_neon<true>;

  m_filter16xH[0][0] = simdFilter16xH_N8_neon<false>;
  m_filter16xH[0][1] = simdFilter16xH_N8_neon<true>;
  m_filter16xH[1][0] = simdFilter16xH_N4_neon<false>;
  m_filter16xH[1][1] = simdFilter16xH_N4_neon<true>;

  m_filterN2_2D = simdInterpolateN2_2D_neon;

  m_filterHor[0][0][0] = simdFilter_neon<8, false, false, false>;
  m_filterHor[0][0][1] = simdFilter_neon<8, false, false, true>;
  m_filterHor[0][1][0] = simdFilter_neon<8, false, true, false>;
  m_filterHor[0][1][1] = simdFilter_neon<8, false, true, true>;

  m_filterHor[1][0][0] = simdFilter_neon<4, false, false, false>;
  m_filterHor[1][0][1] = simdFilter_neon<4, false, false, true>;
  m_filterHor[1][1][0] = simdFilter_neon<4, false, true, false>;
  m_filterHor[1][1][1] = simdFilter_neon<4, false, true, true>;

  m_filterHor[2][0][0] = simdFilter_neon<2, false, false, false>;
  m_filterHor[2][0][1] = simdFilter_neon<2, false, false, true>;
  m_filterHor[2][1][0] = simdFilter_neon<2, false, true, false>;
  m_filterHor[2][1][1] = simdFilter_neon<2, false, true, true>;

  m_filterVer[0][0][0] = simdFilter_neon<8, true, false, false>;
  m_filterVer[0][0][1] = simdFilter_neon<8, true, false, true>;
  m_filterVer[0][1][0] = simdFilter_neon<8, true, true, false>;
  m_filterVer[0][1][1] = simdFilter_neon<8, true, true, true>;

  m_filterVer[1][0][0] = simdFilter_neon<4, true, false, false>;
  m_filterVer[1][0][1] = simdFilter_neon<4, true, false, true>;
  m_filterVer[1][1][0] = simdFilter_neon<4, true, true, false>;
  m_filterVer[1][1][1] = simdFilter_neon<4, true, true, true>;

  m_filterVer[2][0][0] = simdFilter_neon<2, true, false, false>;
  m_filterVer[2][0][1] = simdFilter_neon<2, true, false, true>;
  m_filterVer[2][1][0] = simdFilter_neon<2, true, true, false>;
  m_filterVer[2][1][1] = simdFilter_neon<2, true, true, true>;
}

} // namespace vvdec
#endif
//! \}
