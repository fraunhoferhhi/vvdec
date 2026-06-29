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

/** \file     Picture_neon.cpp
    \brief    Neon implementation of Picture border padding.
*/

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"

namespace vvdec
{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_PICTURE
template<int Step>
static inline void paddPicBorderRow_neon( Pel* pi, const int width, int xmargin )
{
  static_assert( Step == 8 || Step == 16, "Step must be either 8 or 16" );

  CHECKD( xmargin < 16, "xmargin must be at least 16" );
  CHECKD( Step == 8 && ( xmargin & 15 ) != 8, "xmargin must have one 8-sample tail" );
  CHECKD( Step == 16 && ( xmargin & 15 ) != 0, "xmargin must be multiple of 16" );

  Pel* leftDst = pi - xmargin;
  Pel* rightDst = pi + width;

  const int16x8_t left = vdupq_n_s16( pi[0] );
  const int16x8_t right = vdupq_n_s16( pi[width - 1] );

  if( Step == 8 )
  {
    // One 8-sample outside loop.
    vst1q_s16( leftDst, left );
    vst1q_s16( rightDst, right );

    leftDst += 8;
    rightDst += 8;
    xmargin -= 8;
  }

  do
  {
    vst1q_s16( leftDst + 0, left );
    vst1q_s16( leftDst + 8, left );
    vst1q_s16( rightDst + 0, right );
    vst1q_s16( rightDst + 8, right );

    leftDst += 16;
    rightDst += 16;
    xmargin -= 16;
  } while( xmargin != 0 );
}

void paddPicBorderLeftRight_neon( Pel* pi, ptrdiff_t stride, int width, int xmargin, int height )
{
  CHECKD( height < 3, "height must be at least 3" );
  height -= 2;

  if( ( xmargin & 15 ) == 0 )
  {
    do
    {
      paddPicBorderRow_neon<16>( pi, width, xmargin );
      pi += stride;
    } while( --height != 0 );
  }
  else
  {
    do
    {
      paddPicBorderRow_neon<8>( pi, width, xmargin );
      pi += stride;
    } while( --height != 0 );
  }
}

template<>
void Picture::_initPictureARM<NEON>()
{
  paddPicBorderLeftRight = paddPicBorderLeftRight_neon;
}

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_PICTURE

} // namespace vvdec
