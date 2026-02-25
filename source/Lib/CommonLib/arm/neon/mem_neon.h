/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVDeC Authors.
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

/** \file     mem_neon.h
    \brief    Helper functions for loading and storing vectors
*/

#pragma once

#include "CommonDef.h"
#include <string.h>

#if defined( TARGET_SIMD_ARM )

#include <arm_neon.h>

namespace vvdec
{
// Load helpers
static inline int8x16x2_t load_s8x16_x2( const int8_t* x )
{
  int8x16x2_t ret;
  ret.val[0] = vld1q_s8( x );
  ret.val[1] = vld1q_s8( x + 16 );
  return ret;
}

static inline int16x8x2_t load_s16x8_x2( const int16_t* x )
{
  int16x8x2_t ret;
  ret.val[0] = vld1q_s16( x );
  ret.val[1] = vld1q_s16( x + 8 );
  return ret;
}

static inline int16x4_t load_s16x2( const int16_t* src )
{
  uint32_t tmp;
  memcpy( &tmp, src, sizeof( uint32_t ) );
  return vreinterpret_s16_u32( vset_lane_u32( tmp, vdup_n_u32( 0 ), 0 ) );
}

static inline int16x4_t load_s16x2x2( const int16_t* src, ptrdiff_t stride )
{
  uint32x2_t ret = vdup_n_u32( 0 );
  uint32_t tmp0, tmp1;
  memcpy( &tmp0, src + 0 * stride, sizeof( uint32_t ) );
  memcpy( &tmp1, src + 1 * stride, sizeof( uint32_t ) );
  ret = vset_lane_u32( tmp0, ret, 0 );
  ret = vset_lane_u32( tmp1, ret, 1 );
  return vreinterpret_s16_u32( ret );
}

// Store helpers
static inline void store_s16x2( int16_t* dst, int16x4_t src )
{
  uint32_t tmp = vget_lane_u32( vreinterpret_u32_s16( src ), 0 );
  memcpy( dst, &tmp, sizeof( uint32_t ) );
}

static inline void store_s16x2x2( int16_t* dst, int16x4_t src, ptrdiff_t stride )
{
  uint32_t tmp0 = vget_lane_u32( vreinterpret_u32_s16( src ), 0 );
  uint32_t tmp1 = vget_lane_u32( vreinterpret_u32_s16( src ), 1 );
  memcpy( dst + 0 * stride, &tmp0, sizeof( uint32_t ) );
  memcpy( dst + 1 * stride, &tmp1, sizeof( uint32_t ) );
}

template<int Lane>
static inline void store_unaligned_u32_4x1( void* dst, uint32x4_t src )
{
  static_assert( Lane >= 0 && Lane < 4, "Lane must be in [0,3]" );
  uint32_t a = vgetq_lane_u32( src, Lane );
  memcpy( dst, &a, sizeof( uint32_t ) );
}

} // namespace vvdec

#endif
