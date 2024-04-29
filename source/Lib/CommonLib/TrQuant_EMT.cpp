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

/** \file     TrQuant_EMT.cpp
    \brief    transform and quantization class
*/

#include "TrQuant_EMT.h"

#include "Rom.h"

#include <memory.h>

namespace vvdec
{

template<int uiTrSize>
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT );

template<>
inline void _fastInverseMM<2>( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
{
  const int rnd_factor  = 1 << (shift - 1);
  const int reducedLine = line - iSkipLine;
  const int cutoff      = 2 - iSkipLine2;

  memset( dst, 0, reducedLine * 2 * sizeof( TCoeff ) );

  for( int k = 0; k < cutoff; k++ )
  {
    const TCoeff* srcPtr = &src[k * line];
    for( int i = 0; i < reducedLine; i++ )
    {
            TCoeff*       dstPtr = &dst[i << 1];
      const TMatrixCoeff*  itPtr =  &iT[k << 1];
      const TCoeff        srcVal = *srcPtr;
      for( int j = 0; j < 2; j++ )
      {
        *dstPtr++ += srcVal * *itPtr++;
      }
      srcPtr++;
    }
  }

  if( clip )
  {
    for( int i = 0; i < reducedLine; i++ )
    {
      TCoeff* dstPtr = &dst[i << 1];
      for( int j = 0; j < 2; j++, dstPtr++ )
      {
        *dstPtr = Clip3( outputMinimum, outputMaximum, ( int ) ( *dstPtr + rnd_factor ) >> shift );
      }
    }
  }

  if( iSkipLine )
  {
    memset( dst + ( reducedLine << 1 ), 0, ( iSkipLine << 1 ) * sizeof( TCoeff ) );
  }
}

template<int uiTrSize>
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
{
  const int  rnd_factor  = 1 << (shift - 1);
  const int  reducedLine = line - iSkipLine;
  const int  cutoff      = uiTrSize - iSkipLine2;

  memset( dst, 0, line * uiTrSize * sizeof( TCoeff ) );

  g_tCoeffOps.fastInvCore[getLog2( uiTrSize ) - 2]( iT, src, dst, line, reducedLine, cutoff );

  if( clip )
  {
    if( uiTrSize == 4 )
      g_tCoeffOps.roundClip4( dst, uiTrSize, reducedLine, uiTrSize, outputMinimum, outputMaximum, rnd_factor, shift );
    else
      g_tCoeffOps.roundClip8( dst, uiTrSize, reducedLine, uiTrSize, outputMinimum, outputMaximum, rnd_factor, shift );
  }
}


// ********************************** DCT-II **********************************

//Fast DCT-II transforms
void fastInverseDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  int j;
  int E, O;
  int add = 1 << (shift - 1);
  TCoeff* dstOrg = dst;

  const TMatrixCoeff *iT = g_trCoreDCT2P2[0];

  const int  reducedLine = line - iSkipLine;

  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    E = iT[0] * (src[0] + src[line]);
    O = iT[2] * (src[0] - src[line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = E;
    dst[1] = O;

    src++;
    dst += 2;
  }

  if( clip )
  {
    for( int i = 0; i < reducedLine; i++ )
    {
      TCoeff* dstPtr = &dstOrg[i << 1];
      for( int j = 0; j < 2; j++, dstPtr++ )
      {
        *dstPtr = Clip3( outputMinimum, outputMaximum, ( int ) ( *dstPtr + add ) >> shift );
      }
    }
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 1) * sizeof(TCoeff));
  }
}

/** 4x4 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B4( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
#if 0
  const TMatrixCoeff *iT = g_trCoreDCT2P4[0];

  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j;
  int E[2], O[2];
  int add = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = g_trCoreDCT2P4[0];

  TCoeff* orgDst = dst;

  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    O[0] = iT[1 * 4 + 0] * src[line] + iT[3 * 4 + 0] * src[3 * line];
    O[1] = iT[1 * 4 + 1] * src[line] + iT[3 * 4 + 1] * src[3 * line];
    E[0] = iT[0 * 4 + 0] * src[   0] + iT[2 * 4 + 0] * src[2 * line];
    E[1] = iT[0 * 4 + 1] * src[   0] + iT[2 * 4 + 1] * src[2 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = E[0] + O[0];
    dst[1] = E[1] + O[1];
    dst[2] = E[1] - O[1];
    dst[3] = E[0] - O[0];

    src++;
    dst += 4;
  }

  if( clip )
    g_tCoeffOps.roundClip4( orgDst, 4, reducedLine, 4, outputMinimum, outputMaximum, add, shift );

  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 2 ) * sizeof( TCoeff ) );
  }
#endif
}

/** 8x8 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  _fastInverseMM<8>( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDCT2P8[0] );
#else
  int j, k;
  int E[4], O[4];
  int EE[2], EO[2];
  int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT2P8[0];

  TCoeff *orgDst = dst;

  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for( k = 0; k < 4; k++ )
    {
      O[k] = iT[1 * 8 + k] * src[line] + iT[3 * 8 + k] * src[3 * line] + iT[5 * 8 + k] * src[5 * line] + iT[7 * 8 + k] * src[7 * line];
    }

    EO[0] = iT[2 * 8 + 0] * src[2 * line] + iT[6 * 8 + 0] * src[6 * line];
    EO[1] = iT[2 * 8 + 1] * src[2 * line] + iT[6 * 8 + 1] * src[6 * line];
    EE[0] = iT[0 * 8 + 0] * src[0       ] + iT[4 * 8 + 0] * src[4 * line];
    EE[1] = iT[0 * 8 + 1] * src[0       ] + iT[4 * 8 + 1] * src[4 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];

    for( k = 0; k < 4; k++ )
    {
      dst[k    ] = E[    k] + O[    k];
      dst[k + 4] = E[3 - k] - O[3 - k];
    }
    src++;
    dst += 8;
  }

  if( clip )
    g_tCoeffOps.roundClip8( orgDst, 8, reducedLine, 8, outputMinimum, outputMaximum, add, shift );

  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 3 ) * sizeof( TCoeff ) );
  }
#endif
}

/** 16x16 inverse transform implemented using partial butterfly structure (1D)
*  \param src            input data (transform coefficients)
*  \param dst            output data (residual)
*  \param shift          specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B16( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  _fastInverseMM<16>( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDCT2P16[0] );
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM<32>( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDCT2P32[0] );
}

void fastInverseDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM<64>( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDCT2P64[0] );
}

void fastInverseDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDST7P4[0] );
}

void fastInverseDST7_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDST7P8[0]);
}

void fastInverseDST7_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDST7P16[0] );
}

void fastInverseDST7_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDST7P32[0] );
}


// ********************************** DCT-VIII **********************************

void fastInverseDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDCT8P4[0] );
}

void fastInverseDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDCT8P8[0] );
}

void fastInverseDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDCT8P16[0] );
}

void fastInverseDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, bool clip, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, clip, outputMinimum, outputMaximum, g_trCoreDCT8P32[0] );
}

}

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP 1

#include "Unit.h"
#include "Buffer.h"

namespace vvdec
{

void cpyResiClipCore( const TCoeff* src, Pel* dst, ptrdiff_t stride, unsigned width, unsigned height, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift )
{
#define CPYRESI_OP( ADDR ) dst[ADDR] = Clip3( outputMin, outputMax, ( src[ADDR] + round ) >> shift )
#define CPYRESI_INC dst += stride; src += width;

  SIZE_AWARE_PER_EL_OP( CPYRESI_OP, CPYRESI_INC );

#undef CPYRESI_INC
#undef CPYRESI_OP
}

void clipCore( TCoeff *dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin, const TCoeff outputMax, const TCoeff round, const TCoeff shift )
{
#define CLIP_OP( ADDR ) dst[ADDR] = Clip3( outputMin, outputMax, ( dst[ADDR] + round ) >> shift )
#define CLIP_INC        dst      += stride

  SIZE_AWARE_PER_EL_OP( CLIP_OP, CLIP_INC );

#undef CLIP_INC
#undef CLIP_OP
}

template<int trSize>
void fastInvCore_( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned lines, unsigned reducedLines, unsigned rows )
{
  for( int k = 0; k < rows; k++ )
  {
    const TCoeff* srcPtr = &src[k * lines];
    for( int i = 0; i < reducedLines; i++ )
    {
            TCoeff*       dstPtr = &dst[i * trSize];
      const TMatrixCoeff*  itPtr =  &it[k * trSize];
      for( int j = 0; j < trSize; j++ )
      {
        *dstPtr++ += *srcPtr * *itPtr++;
      }
      srcPtr++;
    }
  }
}

TCoeffOps::TCoeffOps()
{
  cpyResiClip[0] = cpyResiClipCore; //  1
  cpyResiClip[1] = cpyResiClipCore; //  2
  cpyResiClip[2] = cpyResiClipCore; //  4
  cpyResiClip[3] = cpyResiClipCore; //  8
  cpyResiClip[4] = cpyResiClipCore; // 16
  cpyResiClip[5] = cpyResiClipCore; // 32
  cpyResiClip[6] = cpyResiClipCore; // 64
  roundClip4     = clipCore;
  roundClip8     = clipCore;
  fastInvCore[0] = fastInvCore_< 4>;
  fastInvCore[1] = fastInvCore_< 8>;
  fastInvCore[2] = fastInvCore_<16>;
  fastInvCore[3] = fastInvCore_<32>;
  fastInvCore[4] = fastInvCore_<64>;
}

TCoeffOps g_tCoeffOps;

}
