/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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
#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/InterpolationFilter.h"

#if ENABLE_SIMD_OPT_BUFFER
#  ifdef TARGET_SIMD_ARM

namespace vvdec
{

template<ARM_VEXT vext>
void applyLut_SIMD( Pel* ptr, ptrdiff_t ptrStride, int width, int height, const Pel* lut )
{
  if( ( width & 31 ) == 0 )
  {
    int16x8x4_t xtmp1;
    int16x8x4_t xtmp2;
    int16x8x4_t xtmp3;
    int16x8x4_t xtmp4;

    for( int y = 0; y < height; y += 4 )
    {
      for( int x = 0; x < width; x += 32 )
      {
        GCC_WARNING_DISABLE_maybe_uninitialized

        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 0 ] ], xtmp1.val[0], 0 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 ] ], xtmp1.val[1], 0 );
        xtmp1.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 ] ], xtmp1.val[2], 0 );
        xtmp1.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 ] ], xtmp1.val[3], 0 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 4 ] ], xtmp1.val[0], 1 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 5 ] ], xtmp1.val[1], 1 );
        xtmp1.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 6 ] ], xtmp1.val[2], 1 );
        xtmp1.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 7 ] ], xtmp1.val[3], 1 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 8 ] ], xtmp1.val[0], 2 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 9 ] ], xtmp1.val[1], 2 );
        xtmp1.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 10 ] ], xtmp1.val[2], 2 );
        xtmp1.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 11 ] ], xtmp1.val[3], 2 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 12 ] ], xtmp1.val[0], 3 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 13 ] ], xtmp1.val[1], 3 );
        xtmp1.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 14 ] ], xtmp1.val[2], 3 );
        xtmp1.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 15 ] ], xtmp1.val[3], 3 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 16 ] ], xtmp1.val[0], 4 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 17 ] ], xtmp1.val[1], 4 );
        xtmp1.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 18 ] ], xtmp1.val[2], 4 );
        xtmp1.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 19 ] ], xtmp1.val[3], 4 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 20 ] ], xtmp1.val[0], 5 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 21 ] ], xtmp1.val[1], 5 );
        xtmp1.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 22 ] ], xtmp1.val[2], 5 );
        xtmp1.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 23 ] ], xtmp1.val[3], 5 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 24 ] ], xtmp1.val[0], 6 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 25 ] ], xtmp1.val[1], 6 );
        xtmp1.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 26 ] ], xtmp1.val[2], 6 );
        xtmp1.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 27 ] ], xtmp1.val[3], 6 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 28 ] ], xtmp1.val[0], 7 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 29 ] ], xtmp1.val[1], 7 );
        xtmp1.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 30 ] ], xtmp1.val[2], 7 );
        xtmp1.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 31 ] ], xtmp1.val[3], 7 );

        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 0 ] ], xtmp2.val[3], 0 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 1 ] ], xtmp2.val[3], 0 );
        xtmp2.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 2 ] ], xtmp2.val[3], 0 );
        xtmp2.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 3 ] ], xtmp2.val[3], 0 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 4 ] ], xtmp2.val[0], 1 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 5 ] ], xtmp2.val[1], 1 );
        xtmp2.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 6 ] ], xtmp2.val[2], 1 );
        xtmp2.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 7 ] ], xtmp2.val[3], 1 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 8 ] ], xtmp2.val[0], 2 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 9 ] ], xtmp2.val[1], 2 );
        xtmp2.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 10 ] ], xtmp2.val[2], 2 );
        xtmp2.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 11 ] ], xtmp2.val[3], 2 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 12 ] ], xtmp2.val[0], 3 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 13 ] ], xtmp2.val[1], 3 );
        xtmp2.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 14 ] ], xtmp2.val[2], 3 );
        xtmp2.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 15 ] ], xtmp2.val[3], 3 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 16 ] ], xtmp2.val[0], 4 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 17 ] ], xtmp2.val[1], 4 );
        xtmp2.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 18 ] ], xtmp2.val[2], 4 );
        xtmp2.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 19 ] ], xtmp2.val[3], 4 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 20 ] ], xtmp2.val[0], 5 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 21 ] ], xtmp2.val[1], 5 );
        xtmp2.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 22 ] ], xtmp2.val[2], 5 );
        xtmp2.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 23 ] ], xtmp2.val[3], 5 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 24 ] ], xtmp2.val[0], 6 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 25 ] ], xtmp2.val[1], 6 );
        xtmp2.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 26 ] ], xtmp2.val[2], 6 );
        xtmp2.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 27 ] ], xtmp2.val[3], 6 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 28 ] ], xtmp2.val[0], 7 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 29 ] ], xtmp2.val[1], 7 );
        xtmp2.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 30 ] ], xtmp2.val[2], 7 );
        xtmp2.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 31 ] ], xtmp2.val[3], 7 );
 
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 0 ] ], xtmp3.val[0], 0 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 1 ] ], xtmp3.val[1], 0 );
        xtmp3.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 2 ] ], xtmp3.val[2], 0 );
        xtmp3.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 3 ] ], xtmp3.val[3], 0 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 4 ] ], xtmp3.val[0], 1 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 5 ] ], xtmp3.val[1], 1 );
        xtmp3.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 6 ] ], xtmp3.val[2], 1 );
        xtmp3.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 7 ] ], xtmp3.val[3], 1 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 8 ] ], xtmp3.val[0], 2 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 9 ] ], xtmp3.val[1], 2 );
        xtmp3.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 10 ] ], xtmp3.val[2], 2 );
        xtmp3.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 11 ] ], xtmp3.val[3], 2 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 12 ] ], xtmp3.val[0], 3 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 13 ] ], xtmp3.val[1], 3 );
        xtmp3.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 14 ] ], xtmp3.val[2], 3 );
        xtmp3.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 15 ] ], xtmp3.val[3], 3 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 16 ] ], xtmp3.val[0], 4 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 17 ] ], xtmp3.val[1], 4 );
        xtmp3.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 18 ] ], xtmp3.val[2], 4 );
        xtmp3.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 19 ] ], xtmp3.val[3], 4 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 20 ] ], xtmp3.val[0], 5 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 21 ] ], xtmp3.val[1], 5 );
        xtmp3.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 22 ] ], xtmp3.val[2], 5 );
        xtmp3.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 23 ] ], xtmp3.val[3], 5 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 24 ] ], xtmp3.val[0], 6 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 25 ] ], xtmp3.val[1], 6 );
        xtmp3.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 26 ] ], xtmp3.val[2], 6 );
        xtmp3.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 27 ] ], xtmp3.val[3], 6 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 28 ] ], xtmp3.val[0], 7 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 29 ] ], xtmp3.val[1], 7 );
        xtmp3.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 30 ] ], xtmp3.val[2], 7 );
        xtmp3.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 31 ] ], xtmp3.val[3], 7 );

        // interleaved assign -> there is only interleaved storing/loading
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 0 ] ], xtmp4.val[0], 0 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 1 ] ], xtmp4.val[1], 0 );
        xtmp4.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 2 ] ], xtmp4.val[2], 0 );
        xtmp4.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 3 ] ], xtmp4.val[3], 0 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 4 ] ], xtmp4.val[0], 1 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 5 ] ], xtmp4.val[1], 1 );
        xtmp4.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 6 ] ], xtmp4.val[2], 1 );
        xtmp4.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 7 ] ], xtmp4.val[3], 1 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 8 ] ], xtmp4.val[0], 2 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 9 ] ], xtmp4.val[1], 2 );
        xtmp4.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 10 ] ], xtmp4.val[2], 2 );
        xtmp4.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 11 ] ], xtmp4.val[3], 2 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 12 ] ], xtmp4.val[0], 3 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 13 ] ], xtmp4.val[1], 3 );
        xtmp4.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 14 ] ], xtmp4.val[2], 3 );
        xtmp4.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 15 ] ], xtmp4.val[3], 3 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 16 ] ], xtmp4.val[0], 4 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 17 ] ], xtmp4.val[1], 4 );
        xtmp4.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 18 ] ], xtmp4.val[2], 4 );
        xtmp4.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 19 ] ], xtmp4.val[3], 4 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 20 ] ], xtmp4.val[0], 5 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 21 ] ], xtmp4.val[1], 5 );
        xtmp4.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 22 ] ], xtmp4.val[2], 5 );
        xtmp4.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 23 ] ], xtmp4.val[3], 5 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 24 ] ], xtmp4.val[0], 6 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 25 ] ], xtmp4.val[1], 6 );
        xtmp4.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 26 ] ], xtmp4.val[2], 6 );
        xtmp4.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 27 ] ], xtmp4.val[3], 6 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 28 ] ], xtmp4.val[0], 7 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 29 ] ], xtmp4.val[1], 7 );
        xtmp4.val[ 2 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 30 ] ], xtmp4.val[2], 7 );
        xtmp4.val[ 3 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 31 ] ], xtmp4.val[3], 7 );

        GCC_WARNING_RESET

        // deinterleaved storing
        vst4q_s16( &ptr[ x ], xtmp1 );
        vst4q_s16( &ptr[ x + 1 * ptrStride ], xtmp2 );
        vst4q_s16( &ptr[ x + 2 * ptrStride ], xtmp3 );
        vst4q_s16( &ptr[ x + 3 * ptrStride ], xtmp4 );
      }
      ptr += ( ptrStride << 2 );
    }
  }
  else if( ( width & 15 ) == 0 )
  {
    int16x8x2_t xtmp1;
    int16x8x2_t xtmp2;
    int16x8x2_t xtmp3;
    int16x8x2_t xtmp4;

    for( int y = 0; y < height; y += 4 )
    {
      for( int x = 0; x < width; x += 16 )
      {
        // vld2q_s16( &ptr[ x ] );

        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 0 ] ], xtmp1.val[ 0 ], 0 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 ] ], xtmp1.val[ 1 ], 0 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 ] ], xtmp1.val[ 0 ], 1 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 ] ], xtmp1.val[ 1 ], 1 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 4 ] ], xtmp1.val[ 0 ], 2 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 5 ] ], xtmp1.val[ 1 ], 2 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 6 ] ], xtmp1.val[ 0 ], 3 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 7 ] ], xtmp1.val[ 1 ], 3 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 8 ] ], xtmp1.val[ 0 ], 4 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 9 ] ], xtmp1.val[ 1 ], 4 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 10 ] ], xtmp1.val[ 0 ], 5 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 11 ] ], xtmp1.val[ 1 ], 5 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 12 ] ], xtmp1.val[ 0 ], 6 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 13 ] ], xtmp1.val[ 1 ], 6 );
        xtmp1.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 14 ] ], xtmp1.val[ 0 ], 7 );
        xtmp1.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 15 ] ], xtmp1.val[ 1 ], 7 );

        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 0 ] ], xtmp2.val[ 0 ], 0 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 1 ] ], xtmp2.val[ 1 ], 0 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 2 ] ], xtmp2.val[ 0 ], 1 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 3 ] ], xtmp2.val[ 1 ], 1 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 4 ] ], xtmp2.val[ 0 ], 2 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 5 ] ], xtmp2.val[ 1 ], 2 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 6 ] ], xtmp2.val[ 0 ], 3 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 7 ] ], xtmp2.val[ 1 ], 3 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 8 ] ], xtmp2.val[ 0 ], 4 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 9 ] ], xtmp2.val[ 1 ], 4 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 10 ] ], xtmp2.val[ 0 ], 5 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 11 ] ], xtmp2.val[ 1 ], 5 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 12 ] ], xtmp2.val[ 0 ], 6 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 13 ] ], xtmp2.val[ 1 ], 6 );
        xtmp2.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 14 ] ], xtmp2.val[ 0 ], 7 );
        xtmp2.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 15 ] ], xtmp2.val[ 1 ], 7 );

        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 0 ] ], xtmp3.val[ 0 ], 0 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 1 ] ], xtmp3.val[ 1 ], 0 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 2 ] ], xtmp3.val[ 0 ], 1 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 3 ] ], xtmp3.val[ 1 ], 1 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 4 ] ], xtmp3.val[ 0 ], 2 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 5 ] ], xtmp3.val[ 1 ], 2 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 6 ] ], xtmp3.val[ 0 ], 3 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 7 ] ], xtmp3.val[ 1 ], 3 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 8 ] ], xtmp3.val[ 0 ], 4 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 9 ] ], xtmp3.val[ 1 ], 4 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 10 ] ], xtmp3.val[ 0 ], 5 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 11 ] ], xtmp3.val[ 1 ], 5 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 12 ] ], xtmp3.val[ 0 ], 6 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 13 ] ], xtmp3.val[ 1 ], 6 );
        xtmp3.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 14 ] ], xtmp3.val[ 0 ], 7 );
        xtmp3.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 15 ] ], xtmp3.val[ 1 ], 7 );

        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 0 ] ], xtmp4.val[ 0 ], 0 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 1 ] ], xtmp4.val[ 1 ], 0 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 2 ] ], xtmp4.val[ 0 ], 1 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 3 ] ], xtmp4.val[ 1 ], 1 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 4 ] ], xtmp4.val[ 0 ], 2 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 5 ] ], xtmp4.val[ 1 ], 2 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 6 ] ], xtmp4.val[ 0 ], 3 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 7 ] ], xtmp4.val[ 1 ], 3 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 8 ] ], xtmp4.val[ 0 ], 4 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 9 ] ], xtmp4.val[ 1 ], 4 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 10 ] ], xtmp4.val[ 0 ], 5 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 11 ] ], xtmp4.val[ 1 ], 5 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 12 ] ], xtmp4.val[ 0 ], 6 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 13 ] ], xtmp4.val[ 1 ], 6 );
        xtmp4.val[ 0 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 14 ] ], xtmp4.val[ 0 ], 7 );
        xtmp4.val[ 1 ] = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 15 ] ], xtmp4.val[ 1 ], 7 );

        vst2q_s16( &ptr[ x ], xtmp1 );
        vst2q_s16( &ptr[ x + 1 * ptrStride ], xtmp2 );
        vst2q_s16( &ptr[ x + 2 * ptrStride ], xtmp3 );
        vst2q_s16( &ptr[ x + 3 * ptrStride ], xtmp4 );
      }
      ptr += ( ptrStride << 2 );
    }
  }
  else if( ( width & 7 ) == 0 )
  {

// MSVC requires neon variables to be initialised when their elements are set with vsetq_lane (it outputs a warning). However, this is not the case for datatypes like int16x8x4_t.
#if defined(_MSC_VER) && !defined(__clang__) && !defined(__INTEL_COMPILER)
    int16x8_t xtmp1 = vdupq_n_s16(0);
    int16x8_t xtmp2 = vdupq_n_s16(0);
    int16x8_t xtmp3 = vdupq_n_s16(0);
    int16x8_t xtmp4 = vdupq_n_s16(0);
#else
    int16x8_t xtmp1;
    int16x8_t xtmp2;
    int16x8_t xtmp3;
    int16x8_t xtmp4;
#endif

    for( int y = 0; y < height; y += 4 )
    {
      for( int x = 0; x < width; x += 8 )
      {
        GCC_WARNING_DISABLE_maybe_uninitialized

        xtmp1 = vsetq_lane_s16( lut[ ptr[ x + 0 ] ], xtmp1, 0 );
        xtmp1 = vsetq_lane_s16( lut[ ptr[ x + 1 ] ], xtmp1, 1 );
        xtmp1 = vsetq_lane_s16( lut[ ptr[ x + 2 ] ], xtmp1, 2 );
        xtmp1 = vsetq_lane_s16( lut[ ptr[ x + 3 ] ], xtmp1, 3 );
        xtmp1 = vsetq_lane_s16( lut[ ptr[ x + 4 ] ], xtmp1, 4 );
        xtmp1 = vsetq_lane_s16( lut[ ptr[ x + 5 ] ], xtmp1, 5 );
        xtmp1 = vsetq_lane_s16( lut[ ptr[ x + 6 ] ], xtmp1, 6 );
        xtmp1 = vsetq_lane_s16( lut[ ptr[ x + 7 ] ], xtmp1, 7 );

        xtmp2 = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 0 ] ], xtmp2, 0 );
        xtmp2 = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 1 ] ], xtmp2, 1 );
        xtmp2 = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 2 ] ], xtmp2, 2 );
        xtmp2 = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 3 ] ], xtmp2, 3 );
        xtmp2 = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 4 ] ], xtmp2, 4 );
        xtmp2 = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 5 ] ], xtmp2, 5 );
        xtmp2 = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 6 ] ], xtmp2, 6 );
        xtmp2 = vsetq_lane_s16( lut[ ptr[ x + 1 * ptrStride + 7 ] ], xtmp2, 7 );

        xtmp3 = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 0 ] ], xtmp3, 0 );
        xtmp3 = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 1 ] ], xtmp3, 1 );
        xtmp3 = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 2 ] ], xtmp3, 2 );
        xtmp3 = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 3 ] ], xtmp3, 3 );
        xtmp3 = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 4 ] ], xtmp3, 4 );
        xtmp3 = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 5 ] ], xtmp3, 5 );
        xtmp3 = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 6 ] ], xtmp3, 6 );
        xtmp3 = vsetq_lane_s16( lut[ ptr[ x + 2 * ptrStride + 7 ] ], xtmp3, 7 );

        xtmp4 = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 0 ] ], xtmp4, 0 );
        xtmp4 = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 1 ] ], xtmp4, 1 );
        xtmp4 = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 2 ] ], xtmp4, 2 );
        xtmp4 = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 3 ] ], xtmp4, 3 );
        xtmp4 = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 4 ] ], xtmp4, 4 );
        xtmp4 = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 5 ] ], xtmp4, 5 );
        xtmp4 = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 6 ] ], xtmp4, 6 );
        xtmp4 = vsetq_lane_s16( lut[ ptr[ x + 3 * ptrStride + 7 ] ], xtmp4, 7 );

        GCC_WARNING_RESET

        vst1q_s16( &ptr[ x ], xtmp1 );
        vst1q_s16( &ptr[ x + 1 * ptrStride ], xtmp2 );
        vst1q_s16( &ptr[ x + 2 * ptrStride ], xtmp3 );
        vst1q_s16( &ptr[ x + 3 * ptrStride ], xtmp4 );
      }

      ptr += ( ptrStride << 2 );
    }
  }

  return;
}

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

template<ARM_VEXT vext>
void rspFwdCore_SIMD( Pel*       ptr,
                      ptrdiff_t  ptrStride,
                      int        width,
                      int        height,
                      const int  bd,
                      const Pel  OrgCW,
                      const Pel* LmcsPivot,
                      const Pel* ScaleCoeff,
                      const Pel* InputPivot )
{
  int shift = getLog2( OrgCW );

  if( ( width & 7 ) == 0 )
  {
    int16x8_t xtmp1;

    int8x16x2_t mLmcsPivot  = vld2q_s8( (const signed char*) &LmcsPivot[ 0 ] );
    int8x16x2_t mInputPivot = vld2q_s8( (const signed char*) &InputPivot[ 0 ] );
    int8x16x2_t mScaleCoeff = vld2q_s8( (const signed char*) &ScaleCoeff[ 0 ] );

    const int16x8_t mMin = vdupq_n_s16( 0 );
    const int16x8_t mMax = vdupq_n_s16( ( 1 << bd ) - 1 );

#if defined(_MSC_VER) && !defined(__clang__) && !defined(__INTEL_COMPILER)
    const uint8_t idx4idx_array[16] = { 0, 2, 4, 6, 8, 10, 12, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    const uint8x16_t idx4idx = vld1q_u8(idx4idx_array);
#else
    const uint8x16_t idx4idx = { 0, 2, 4, 6, 8, 10, 12, 14, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
#endif

    while( height-- )
    {
      for( int x = 0; x < width; x += 8 )
      {
        const int16x8_t xsrc = vld1q_s16( &ptr[ x ] );
        const uint8x16_t xidx = vqtbl1q_u8( (uint8x16_t) vshlq_s16( xsrc, vdupq_n_s16( -shift ) ), idx4idx );

        const int16x8_t xlmc = (int16x8_t) vzip1q_s8( vqtbl1q_s8( mLmcsPivot.val[ 0 ], xidx ), vqtbl1q_s8( mLmcsPivot.val[ 1 ], xidx ) );
        const int16x8_t xinp = (int16x8_t) vzip1q_s8( vqtbl1q_s8( mInputPivot.val[ 0 ], xidx ), vqtbl1q_s8( mInputPivot.val[ 1 ], xidx ) );
        const int16x8_t xscl = (int16x8_t) vzip1q_s8( vqtbl1q_s8( mScaleCoeff.val[ 0 ], xidx ), vqtbl1q_s8( mScaleCoeff.val[ 1 ], xidx ) );

        xtmp1 = vcombine_s16( vrshrn_n_s32( vmull_s16( vget_low_s16( vqsubq_s16( xsrc, xinp ) ), vget_low_s16( xscl ) ), 11 ),
                              vrshrn_n_s32( vmull_s16( vget_high_s16( vqsubq_s16( xsrc, xinp ) ), vget_high_s16( xscl ) ), 11 ) );

        xtmp1 = vaddq_s16( xlmc, xtmp1 );

        xtmp1 = vminq_s16( xtmp1, mMax );
        xtmp1 = vmaxq_s16( xtmp1, mMin );

        vst1q_s16( &ptr[ x ], xtmp1 );
      }
      ptr += ptrStride;
    }
  }
  else
  {
    int idxY;

    //    const auto rsp_sgnl_op  = [=, &dst]( int ADDR ){ idxY = ( dst[ADDR] >> shift ); dst[ADDR] = static_cast<Pel>( ClipBD<int>( LmcsPivot[idxY] + ( (
    //    ScaleCoeff[idxY] * ( dst[ADDR] - InputPivot[idxY] ) + ( 1 << 10 ) ) >> 11 ), bd ) ); }; const auto rsp_sgnl_inc = [=, &dst]            { dst +=
    //    stride; };

    //    size_aware_pel_op( rsp_sgnl_op, rsp_sgnl_inc, width, height );

#  define RSP_FWD_OP( ADDR )                                                                                                                              \
    {                                                                                                                                                     \
      idxY = ( ptr[ ADDR ] >> shift );                                                                                                                    \
      ptr[ ADDR ] =                                                                                                                                       \
        static_cast<Pel>( ClipBD<int>( LmcsPivot[ idxY ] + ( ( ScaleCoeff[ idxY ] * ( ptr[ ADDR ] - InputPivot[ idxY ] ) + ( 1 << 10 ) ) >> 11 ), bd ) ); \
    }
#  define RSP_FWD_INC ptr += ptrStride;

    SIZE_AWARE_PER_EL_OP( RSP_FWD_OP, RSP_FWD_INC )

#  undef RSP_FWD_OP
#  undef RSP_FWD_INC
  }
}

#    endif   // __ARM_ARCH >= 8

template<ARM_VEXT vext>
void PelBufferOps::_initPelBufOpsARM()
{
  //  addAvg16 = addAvg_SSE<vext, 16>;
  //  addAvg8  = addAvg_SSE<vext,  8>;
  //  addAvg4  = addAvg_SSE<vext,  4>;
  //
  //  reco8 = reco_SSE<vext, 8>;
  //  reco4 = reco_SSE<vext, 4>;
  //
  //  linTf8 = linTf_SSE_entry<vext, 8>;
  //  linTf4 = linTf_SSE_entry<vext, 4>;
  // #if ENABLE_SIMD_OPT_GBI
  //
  //  wghtAvg4 = addWghtAvg_SSE<vext, 4>;
  //  wghtAvg8 = addWghtAvg_SSE<vext, 8>;
  // #endif
  //
  //  copyBuffer = copyBuffer_SSE<vext>;
  //
  //  transpose4x4 = transposePel_SSE<vext, 4>;
  //  transpose8x8 = transposePel_SSE<vext, 8>;

  applyLut = applyLut_SIMD<vext>;
#    if __ARM_ARCH >= 8
  rspFwd   = rspFwdCore_SIMD<vext>;
  //  rspBcw   = rspBcwCore_SIMD<vext>;     // disabled, because applyLut is faster
#    endif   // __ARM_ARCH >= 8

  // #if INTPTR_MAX == INT64_MAX || INTPTR_MAX == INT32_MAX
  //   fillN_CU = fillN_CU_SIMD<vext>;
  // #endif
  //
  //   sampleRateConv = sampleRateConvSIMD<vext>;
}

template void PelBufferOps::_initPelBufOpsARM<SIMDARM>();

}   // namespace vvdec

#  endif   // TARGET_SIMD_ARM
#endif     // ENABLE_SIMD_OPT_BUFFER
//! \}
