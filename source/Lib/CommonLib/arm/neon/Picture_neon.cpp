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
 * \file Picture_neon.cpp
 * \brief Neon picture border padding.
 */

#include "CommonLib/arm/CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_PICTURE

namespace vvdec
{

static void paddPicBorderLeftRight_neon( Pel* pi, ptrdiff_t stride, int width, int xmargin, int height )
{
  for( int i = 1; i < height - 1; i++ )
  {
    const int16x8_t xleft  = vdupq_n_s16( pi[0] );
    const int16x8_t xright = vdupq_n_s16( pi[width - 1] );

    int temp = xmargin;
    int x    = 0;
    while( ( temp >> 3 ) > 0 )
    {
      vst1q_s16( pi - xmargin + x, xleft );
      vst1q_s16( pi + width   + x, xright );
      x    += 8;
      temp -= 8;
    }
    while( ( temp >> 2 ) > 0 )
    {
      vst1_s16( pi - xmargin + x, vget_low_s16( xleft ) );
      vst1_s16( pi + width   + x, vget_low_s16( xright ) );
      x    += 4;
      temp -= 4;
    }
    while( ( temp >> 1 ) > 0 )
    {
      *( int32_t* )( pi - xmargin + x ) = vgetq_lane_s32( vreinterpretq_s32_s16( xleft ),  0 );
      *( int32_t* )( pi + width   + x ) = vgetq_lane_s32( vreinterpretq_s32_s16( xright ), 0 );
      x    += 2;
      temp -= 2;
    }
    pi += stride;
  }
}

static void paddPicBorderBot_neon( Pel* pi, ptrdiff_t stride, int width, int xmargin, int ymargin )
{
  paddPicBorderLeftRight_neon( pi, stride, width, xmargin, 3 );

  pi -= xmargin;

  for( int i = 1; i <= ymargin; i++ )
  {
    int j    = 0;
    int temp = width + ( xmargin << 1 );
    while( ( temp >> 3 ) > 0 )
    {
      vst1q_s16( pi + j + i * stride, vld1q_s16( pi + j ) );
      j    += 8;
      temp -= 8;
    }
    while( ( temp >> 2 ) > 0 )
    {
      vst1_s16( pi + j + i * stride, vld1_s16( pi + j ) );
      j    += 4;
      temp -= 4;
    }
    while( ( temp >> 1 ) > 0 )
    {
      *( int32_t* )( pi + j + i * stride ) = *( const int32_t* )( pi + j );
      j    += 2;
      temp -= 2;
    }
  }
}

static void paddPicBorderTop_neon( Pel* pi, ptrdiff_t stride, int width, int xmargin, int ymargin )
{
  paddPicBorderLeftRight_neon( pi, stride, width, xmargin, 3 );

  pi -= xmargin;

  for( int i = 1; i <= ymargin; i++ )
  {
    int j    = 0;
    int temp = width + ( xmargin << 1 );
    while( ( temp >> 3 ) > 0 )
    {
      vst1q_s16( pi + j - i * stride, vld1q_s16( pi + j ) );
      j    += 8;
      temp -= 8;
    }
    while( ( temp >> 2 ) > 0 )
    {
      vst1_s16( pi + j - i * stride, vld1_s16( pi + j ) );
      j    += 4;
      temp -= 4;
    }
    while( ( temp >> 1 ) > 0 )
    {
      *( int32_t* )( pi + j - i * stride ) = *( const int32_t* )( pi + j );
      j    += 2;
      temp -= 2;
    }
  }
}

template<>
void Picture::_initPictureARM<NEON>()
{
  paddPicBorderBot       = paddPicBorderBot_neon;
  paddPicBorderTop       = paddPicBorderTop_neon;
  paddPicBorderLeftRight = paddPicBorderLeftRight_neon;
}

} // namespace vvdec

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_PICTURE
//! \}
