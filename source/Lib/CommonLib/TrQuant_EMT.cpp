/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software, 
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2018-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/** \file     TrQuant_EMT.cpp
    \brief    transform and quantization class
*/

#include "TrQuant_EMT.h"

#include "Rom.h"

#include <stdlib.h>
#include <math.h>
#include <limits>
#include <memory.h>

namespace vvdec
{

#if ENABLE_SIMD_TCOEFF_OPS
template<int uiTrSize>
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT );

template<>
inline void _fastInverseMM<2>( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
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

  for( int i = 0; i < reducedLine; i++ )
  {
    TCoeff* dstPtr = &dst[i << 1];
    for( int j = 0; j < 2; j++, dstPtr++ )
    {
      *dstPtr = Clip3( outputMinimum, outputMaximum, ( int ) ( *dstPtr + rnd_factor ) >> shift );
    }
  }

  if( iSkipLine )
  {
    memset( dst + ( reducedLine << 1 ), 0, ( iSkipLine << 1 ) * sizeof( TCoeff ) );
  }
}

template<>
inline void _fastInverseMM<4>( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
{
  const int rnd_factor = 1 << ( shift - 1 );
  const int reducedLine = line - iSkipLine;
  const int cutoff = 4 - iSkipLine2;

  memset( dst, 0, line * 4 * sizeof( TCoeff ) );

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.fastInvCore4( iT, src, dst, 4, line, reducedLine, cutoff );
  g_tCoeffOps.roundClip4  ( dst, 4, reducedLine, 4, outputMinimum, outputMaximum, rnd_factor, shift );
#else
  for( int k = 0; k < cutoff; k++ )
  {
    const TCoeff* srcPtr = &src[k * line];
    for( int i = 0; i < reducedLine; i++ )
    {
            TCoeff*       dstPtr = &dst[i << 2];
      const TMatrixCoeff*  itPtr =  &iT[k << 2];
      for( int j = 0; j < 4; j++ )
      {
        *dstPtr++ += *srcPtr * *itPtr++;
      }
      srcPtr++;
    }
  }

  for( int i = 0; i < reducedLine; i++ )
  {
    TCoeff* dstPtr = &dst[i << 2];
    for( int j = 0; j < 4; j++, dstPtr++ )
    {
      *dstPtr = Clip3( outputMinimum, outputMaximum, ( int ) ( *dstPtr + rnd_factor ) >> shift );
    }
  }
#endif
}

#endif

template<int uiTrSize>
inline void _fastInverseMM( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum, const TMatrixCoeff* iT )
{
  const int  rnd_factor  = 1 << (shift - 1);
  const int  reducedLine = line - iSkipLine;
  const int  cutoff      = uiTrSize - iSkipLine2;

  memset( dst, 0, line * uiTrSize * sizeof( TCoeff ) );

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.fastInvCore8( iT, src, dst, uiTrSize, line, reducedLine, cutoff );
#else
  for( int k = 0; k < cutoff; k++ )
  {
    const TCoeff* srcPtr = &src[k * line];
    for( int i = 0; i < reducedLine; i++ )
    {
            TCoeff*       dstPtr = &dst[i * uiTrSize];
      const TMatrixCoeff*  itPtr =  &iT[k * uiTrSize];
      for( int j = 0; j < uiTrSize; j++ )
      {
        *dstPtr++ += *srcPtr * *itPtr++;
      }
      srcPtr++;
    }
  }
#endif

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.roundClip8( dst, uiTrSize, reducedLine, uiTrSize, outputMinimum, outputMaximum, rnd_factor, shift );
#else
  for( int i = 0; i < reducedLine; i++ )
  {
    TCoeff* dstPtr = &dst[i * uiTrSize];
    for( int j = 0; j < uiTrSize; j++, dstPtr++ )
    {
      *dstPtr = Clip3( outputMinimum, outputMaximum, ( int ) ( *dstPtr + rnd_factor ) >> shift );
    }
  }
#endif
}


// ********************************** DCT-II **********************************

//Fast DCT-II transforms
void fastInverseDCT2_B2(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  int j;
  int E, O;
  int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT2P2[0];

  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    E = iT[0] * (src[0] + src[line]);
    O = iT[2] * (src[0] - src[line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    dst[0] = Clip3(outputMinimum, outputMaximum, (E + add) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (O + add) >> shift);

    src++;
    dst += 2;
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
void fastInverseDCT2_B4( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
#if 0
  const TMatrixCoeff *iT = g_trCoreDCT2P4[0];

  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j;
  int E[2], O[2];
  int add = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = g_trCoreDCT2P4[0];

#if ENABLE_SIMD_TCOEFF_OPS
  TCoeff* orgDst = dst;
#endif

  const int  reducedLine = line - iSkipLine;
  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    O[0] = iT[1 * 4 + 0] * src[line] + iT[3 * 4 + 0] * src[3 * line];
    O[1] = iT[1 * 4 + 1] * src[line] + iT[3 * 4 + 1] * src[3 * line];
    E[0] = iT[0 * 4 + 0] * src[   0] + iT[2 * 4 + 0] * src[2 * line];
    E[1] = iT[0 * 4 + 1] * src[   0] + iT[2 * 4 + 1] * src[2 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
#if ENABLE_SIMD_TCOEFF_OPS
    dst[0] = E[0] + O[0];
    dst[1] = E[1] + O[1];
    dst[2] = E[1] - O[1];
    dst[3] = E[0] - O[0];
#else
    dst[0] = Clip3( outputMinimum, outputMaximum, ( E[0] + O[0] + add ) >> shift );
    dst[1] = Clip3( outputMinimum, outputMaximum, ( E[1] + O[1] + add ) >> shift );
    dst[2] = Clip3( outputMinimum, outputMaximum, ( E[1] - O[1] + add ) >> shift );
    dst[3] = Clip3( outputMinimum, outputMaximum, ( E[0] - O[0] + add ) >> shift );
#endif

    src++;
    dst += 4;
  }

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.roundClip4( orgDst, 4, reducedLine, 4, outputMinimum, outputMaximum, add, shift );
#endif

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
void fastInverseDCT2_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 0
  const TMatrixCoeff *iT = g_trCoreDCT2P8[0];

  _fastInverseMM<8>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j, k;
  int E[4], O[4];
  int EE[2], EO[2];
  int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT2P8[0];

#if ENABLE_SIMD_TCOEFF_OPS
  TCoeff *orgDst = dst;
#endif

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
#if ENABLE_SIMD_TCOEFF_OPS
      dst[k    ] = E[    k] + O[    k];
      dst[k + 4] = E[3 - k] - O[3 - k];
#else
      dst[k    ] = Clip3( outputMinimum, outputMaximum, ( E[    k] + O[    k] + add ) >> shift );
      dst[k + 4] = Clip3( outputMinimum, outputMaximum, ( E[3 - k] - O[3 - k] + add ) >> shift );
#endif
    }
    src++;
    dst += 8;
  }

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.roundClip8( orgDst, 8, reducedLine, 8, outputMinimum, outputMaximum, add, shift );
#endif

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
void fastInverseDCT2_B16( const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
#if 1
  const TMatrixCoeff *iT = g_trCoreDCT2P16[0];

  _fastInverseMM<16>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j, k;
  int E  [8], O  [8];
  int EE [4], EO [4];
  int EEE[2], EEO[2];
  int add = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = g_trCoreDCT2P16[0];

#if ENABLE_SIMD_TCOEFF_OPS
  TCoeff *orgDst = dst;
#endif

  const int  reducedLine = line - iSkipLine;

  for( j = 0; j < reducedLine; j++ )
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for( k = 0; k < 8; k++ )
    {
      O[k] = iT[1 * 16 + k] * src[    line] + iT[ 3 * 16 + k] * src[ 3 * line] + iT[ 5 * 16 + k] * src[ 5 * line] + iT[ 7 * 16 + k] * src[ 7 * line] +
        iT[9 * 16 + k] * src[9 * line] + iT[11 * 16 + k] * src[11 * line] + iT[13 * 16 + k] * src[13 * line] + iT[15 * 16 + k] * src[15 * line];
    }
    for( k = 0; k < 4; k++ )
    {
      EO[k] = iT[2 * 16 + k] * src[2 * line] + iT[6 * 16 + k] * src[6 * line] + iT[10 * 16 + k] * src[10 * line] + iT[14 * 16 + k] * src[14 * line];
    }
    EEO[0] = iT[4 * 16    ] * src[4 * line] + iT[12 * 16    ] * src[12 * line];
    EEE[0] = iT[0         ] * src[0       ] + iT[ 8 * 16    ] * src[ 8 * line];
    EEO[1] = iT[4 * 16 + 1] * src[4 * line] + iT[12 * 16 + 1] * src[12 * line];
    EEE[1] = iT[0 * 16 + 1] * src[0       ] + iT[ 8 * 16 + 1] * src[ 8 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for( k = 0; k < 2; k++ )
    {
      EE[k    ] = EEE[    k] + EEO[    k];
      EE[k + 2] = EEE[1 - k] - EEO[1 - k];
    }
    for( k = 0; k < 4; k++ )
    {
      E[k    ] = EE[    k] + EO[    k];
      E[k + 4] = EE[3 - k] - EO[3 - k];
    }
    for( k = 0; k < 8; k++ )
    {
#if ENABLE_SIMD_TCOEFF_OPS
      dst[k    ] = E[    k] + O[    k];
      dst[k + 8] = E[7 - k] - O[7 - k];
#else
      dst[k    ] = Clip3( outputMinimum, outputMaximum, ( E[    k] + O[    k] + add ) >> shift );
      dst[k + 8] = Clip3( outputMinimum, outputMaximum, ( E[7 - k] - O[7 - k] + add ) >> shift );
#endif
    }
    src++;
    dst += 16;
  }

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.roundClip8( orgDst, 16, reducedLine, 16, outputMinimum, outputMaximum, add, shift );
#endif

  if( iSkipLine )
  {
    memset( dst, 0, ( iSkipLine << 4 ) * sizeof( TCoeff ) );
  }
#endif
}

/** 32x32 inverse transform implemented using partial butterfly structure (1D)
*  \param src   input data (transform coefficients)
*  \param dst   output data (residual)
*  \param shift specifies right shift after 1D transform
*  \param line
*  \param outputMinimum  minimum for clipping
*  \param outputMaximum  maximum for clipping
*/
void fastInverseDCT2_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  const TMatrixCoeff *iT = g_trCoreDCT2P32[0];

  _fastInverseMM<32>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int j, k;
  int E[16], O[16];
  int EE[8], EO[8];
  int EEE[4], EEO[4];
  int EEEE[2], EEEO[2];
  int add = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT2P32[0];

#if ENABLE_SIMD_TCOEFF_OPS
  TCoeff *orgDst = dst;
#endif

  const int  reducedLine = line - iSkipLine;
  for (j = 0; j<reducedLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k = 0;k<16;k++)
    {
      O[k] = iT[1 * 32 + k] * src[line] + iT[3 * 32 + k] * src[3 * line] + iT[5 * 32 + k] * src[5 * line] + iT[7 * 32 + k] * src[7 * line] +
        iT[9 * 32 + k] * src[9 * line] + iT[11 * 32 + k] * src[11 * line] + iT[13 * 32 + k] * src[13 * line] + iT[15 * 32 + k] * src[15 * line] +
        iT[17 * 32 + k] * src[17 * line] + iT[19 * 32 + k] * src[19 * line] + iT[21 * 32 + k] * src[21 * line] + iT[23 * 32 + k] * src[23 * line] +
        iT[25 * 32 + k] * src[25 * line] + iT[27 * 32 + k] * src[27 * line] + iT[29 * 32 + k] * src[29 * line] + iT[31 * 32 + k] * src[31 * line];
    }
    for (k = 0;k<8;k++)
    {
      EO[k] = iT[2 * 32 + k] * src[2 * line] + iT[6 * 32 + k] * src[6 * line] + iT[10 * 32 + k] * src[10 * line] + iT[14 * 32 + k] * src[14 * line] +
        iT[18 * 32 + k] * src[18 * line] + iT[22 * 32 + k] * src[22 * line] + iT[26 * 32 + k] * src[26 * line] + iT[30 * 32 + k] * src[30 * line];
    }
    for (k = 0;k<4;k++)
    {
      EEO[k] = iT[4 * 32 + k] * src[4 * line] + iT[12 * 32 + k] * src[12 * line] + iT[20 * 32 + k] * src[20 * line] + iT[28 * 32 + k] * src[28 * line];
    }
    EEEO[0] = iT[8 * 32 + 0] * src[8 * line] + iT[24 * 32 + 0] * src[24 * line];
    EEEO[1] = iT[8 * 32 + 1] * src[8 * line] + iT[24 * 32 + 1] * src[24 * line];
    EEEE[0] = iT[0 * 32 + 0] * src[0] + iT[16 * 32 + 0] * src[16 * line];
    EEEE[1] = iT[0 * 32 + 1] * src[0] + iT[16 * 32 + 1] * src[16 * line];

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];
    for (k = 0;k<4;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 4] = EEE[3 - k] - EEO[3 - k];
    }
    for (k = 0;k<8;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 8] = EE[7 - k] - EO[7 - k];
    }
    for (k = 0;k<16;k++)
    {
#if ENABLE_SIMD_TCOEFF_OPS
      dst[k     ] = E[k     ] + O[k     ];
      dst[k + 16] = E[15 - k] - O[15 - k];
#else
      dst[k] = Clip3(outputMinimum, outputMaximum, (E[k] + O[k] + add) >> shift);
      dst[k + 16] = Clip3(outputMinimum, outputMaximum, (E[15 - k] - O[15 - k] + add) >> shift);
#endif
    }
    src++;
    dst += 32;
  }

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.roundClip8( orgDst, 32, reducedLine, 32, outputMinimum, outputMaximum, add, shift );
#endif

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 5) * sizeof(TCoeff));
  }
#endif
}

void fastInverseDCT2_B64(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  const TMatrixCoeff *iT = g_trCoreDCT2P64[0];

  _fastInverseMM<64>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, iT );
#else
  int rnd_factor = 1 << (shift - 1);
  const int uiTrSize = 64;
  const TMatrixCoeff *iT = g_trCoreDCT2P64[0];

#if ENABLE_SIMD_TCOEFF_OPS
  TCoeff *orgDst = dst;
#endif

  int    j, k;
  TCoeff E[32], O[32];
  TCoeff EE[16], EO[16];
  TCoeff EEE[8], EEO[8];
  TCoeff EEEE[4], EEEO[4];
  TCoeff EEEEE[2], EEEEO[2];
  bool zo = iSkipLine2 >= 32;
  for (j = 0; j<line - iSkipLine; j++)
  {
    /* Utilizing symmetry properties to the maximum to minimize the number of multiplications */
    for (k = 0;k<32;k++)
    {
      O[k] = iT[1 * 64 + k] * src[line] + iT[3 * 64 + k] * src[3 * line] + iT[5 * 64 + k] * src[5 * line] + iT[7 * 64 + k] * src[7 * line] +
        iT[9 * 64 + k] * src[9 * line] + iT[11 * 64 + k] * src[11 * line] + iT[13 * 64 + k] * src[13 * line] + iT[15 * 64 + k] * src[15 * line] +
        iT[17 * 64 + k] * src[17 * line] + iT[19 * 64 + k] * src[19 * line] + iT[21 * 64 + k] * src[21 * line] + iT[23 * 64 + k] * src[23 * line] +
        iT[25 * 64 + k] * src[25 * line] + iT[27 * 64 + k] * src[27 * line] + iT[29 * 64 + k] * src[29 * line] + iT[31 * 64 + k] * src[31 * line] +
        (zo ? 0 : (
        iT[33 * 64 + k] * src[33 * line] + iT[35 * 64 + k] * src[35 * line] + iT[37 * 64 + k] * src[37 * line] + iT[39 * 64 + k] * src[39 * line] +
        iT[41 * 64 + k] * src[41 * line] + iT[43 * 64 + k] * src[43 * line] + iT[45 * 64 + k] * src[45 * line] + iT[47 * 64 + k] * src[47 * line] +
        iT[49 * 64 + k] * src[49 * line] + iT[51 * 64 + k] * src[51 * line] + iT[53 * 64 + k] * src[53 * line] + iT[55 * 64 + k] * src[55 * line] +
        iT[57 * 64 + k] * src[57 * line] + iT[59 * 64 + k] * src[59 * line] + iT[61 * 64 + k] * src[61 * line] + iT[63 * 64 + k] * src[63 * line]));
    }
    for (k = 0;k<16;k++)
    {
      EO[k] = iT[2 * 64 + k] * src[2 * line] + iT[6 * 64 + k] * src[6 * line] + iT[10 * 64 + k] * src[10 * line] + iT[14 * 64 + k] * src[14 * line] +
        iT[18 * 64 + k] * src[18 * line] + iT[22 * 64 + k] * src[22 * line] + iT[26 * 64 + k] * src[26 * line] + iT[30 * 64 + k] * src[30 * line] +
        (zo ? 0 : (
        iT[34 * 64 + k] * src[34 * line] + iT[38 * 64 + k] * src[38 * line] + iT[42 * 64 + k] * src[42 * line] + iT[46 * 64 + k] * src[46 * line] +
        iT[50 * 64 + k] * src[50 * line] + iT[54 * 64 + k] * src[54 * line] + iT[58 * 64 + k] * src[58 * line] + iT[62 * 64 + k] * src[62 * line]));
    }
    for (k = 0;k<8;k++)
    {
      EEO[k] = iT[4 * 64 + k] * src[4 * line] + iT[12 * 64 + k] * src[12 * line] + iT[20 * 64 + k] * src[20 * line] + iT[28 * 64 + k] * src[28 * line] +
        (zo ? 0 : (
        iT[36 * 64 + k] * src[36 * line] + iT[44 * 64 + k] * src[44 * line] + iT[52 * 64 + k] * src[52 * line] + iT[60 * 64 + k] * src[60 * line]));
    }
    for (k = 0;k<4;k++)
    {
      EEEO[k] = iT[8 * 64 + k] * src[8 * line] + iT[24 * 64 + k] * src[24 * line] + (zo ? 0 : (iT[40 * 64 + k] * src[40 * line] + iT[56 * 64 + k] * src[56 * line]));
    }
    EEEEO[0] = iT[16 * 64 + 0] * src[16 * line] + (zo ? 0 : iT[48 * 64 + 0] * src[48 * line]);
    EEEEO[1] = iT[16 * 64 + 1] * src[16 * line] + (zo ? 0 : iT[48 * 64 + 1] * src[48 * line]);
    EEEEE[0] = iT[0 * 64 + 0] * src[0] + (zo ? 0 : iT[32 * 64 + 0] * src[32 * line]);
    EEEEE[1] = iT[0 * 64 + 1] * src[0] + (zo ? 0 : iT[32 * 64 + 1] * src[32 * line]);

    /* Combining even and odd terms at each hierarchy levels to calculate the final spatial domain vector */
    for (k = 0;k<2;k++)
    {
      EEEE[k] = EEEEE[k] + EEEEO[k];
      EEEE[k + 2] = EEEEE[1 - k] - EEEEO[1 - k];
    }
    for (k = 0;k<4;k++)
    {
      EEE[k] = EEEE[k] + EEEO[k];
      EEE[k + 4] = EEEE[3 - k] - EEEO[3 - k];
    }
    for (k = 0;k<8;k++)
    {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 8] = EEE[7 - k] - EEO[7 - k];
    }
    for (k = 0;k<16;k++)
    {
      E[k] = EE[k] + EO[k];
      E[k + 16] = EE[15 - k] - EO[15 - k];
    }
    for (k = 0;k<32;k++)
    {
#if ENABLE_SIMD_TCOEFF_OPS
      dst[k]      = E[k] + O[k];
      dst[k + 32] = E[31 - k] - O[31 - k];
#else
      dst[k]      = Clip3( outputMinimum, outputMaximum, ( E[k] + O[k] + rnd_factor ) >> shift );
      dst[k + 32] = Clip3( outputMinimum, outputMaximum, ( E[31 - k] - O[31 - k] + rnd_factor ) >> shift );
#endif
    }
    src++;
    dst += uiTrSize;
  }

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.roundClip8( orgDst, 32, line - iSkipLine, 32, outputMinimum, outputMaximum, rnd_factor, shift );

#endif
  memset(dst, 0, uiTrSize*iSkipLine * sizeof(TCoeff));
#endif
}

void fastInverseDST7_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDST7P4[0] );
#else
  int i;
  TCoeff c[4];
  TCoeff rnd_factor = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P4[0];

#if ENABLE_SIMD_TCOEFF_OPS
  TCoeff *orgDst = dst;
#endif

  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0 * line] + src[2 * line];
    c[1] = src[2 * line] + src[3 * line];
    c[2] = src[0 * line] - src[3 * line];
    c[3] = iT[2] * src[1 * line];

#if ENABLE_SIMD_TCOEFF_OPS
    dst[0] = iT[0] * c[0] + iT[1] * c[1] + c[3];
    dst[1] = iT[1] * c[2] - iT[0] * c[1] + c[3];
    dst[2] = iT[2] * ( src[0 * line] - src[2 * line] + src[3 * line] );
    dst[3] = iT[1] * c[0] + iT[0] * c[2] - c[3] ;
#else
    dst[0] = Clip3(outputMinimum, outputMaximum, (iT[0] * c[0] + iT[1] * c[1] + c[3] + rnd_factor) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (iT[1] * c[2] - iT[0] * c[1] + c[3] + rnd_factor) >> shift);
    dst[2] = Clip3(outputMinimum, outputMaximum, (iT[2] * (src[0 * line] - src[2 * line] + src[3 * line]) + rnd_factor) >> shift);
    dst[3] = Clip3(outputMinimum, outputMaximum, (iT[1] * c[0] + iT[0] * c[2] - c[3] + rnd_factor) >> shift);
#endif

    dst += 4;
    src++;
  }

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.roundClip4( orgDst, 4, reducedLine, 4, outputMinimum, outputMaximum, rnd_factor, shift );
#endif

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 2) * sizeof(TCoeff));
  }
#endif
}

void fastInverseDST7_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDST7P8[0]);
}

void fastInverseDST7_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDST7P16[0] );
#else
  int j, k;
  TCoeff a[5], b[5], c[5], d[5], t;

  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P16[0];

  const int  reducedLine = line - iSkipLine;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 5; k++)
    {
      a[k] = src[       k * line] + src[(10 - k) * line];
      b[k] = src[(11 + k) * line] + src[(10 - k) * line];
      c[k] = src[       k * line] - src[(11 + k) * line];
      d[k] = src[       k * line] + src[(11 + k) * line] - src[(10 - k)*line];
    }

    t = iT[10] * src[5 * line];

    dst[ 2] = Clip3(outputMinimum, outputMaximum, (int)( iT[ 2]*d[0] + iT[ 8]*d[1] + iT[14]*d[2] + iT[11]*d[3] + iT[ 5]*d[4] + add ) >> shift);
    dst[ 5] = Clip3(outputMinimum, outputMaximum, (int)( iT[ 5]*d[0] + iT[14]*d[1] + iT[ 2]*d[2] - iT[ 8]*d[3] - iT[11]*d[4] + add ) >> shift);
    dst[ 8] = Clip3(outputMinimum, outputMaximum, (int)( iT[ 8]*d[0] + iT[ 5]*d[1] - iT[11]*d[2] - iT[ 2]*d[3] + iT[14]*d[4] + add ) >> shift);
    dst[11] = Clip3(outputMinimum, outputMaximum, (int)( iT[11]*d[0] - iT[ 2]*d[1] - iT[ 5]*d[2] + iT[14]*d[3] - iT[ 8]*d[4] + add ) >> shift);
    dst[14] = Clip3(outputMinimum, outputMaximum, (int)( iT[14]*d[0] - iT[11]*d[1] + iT[ 8]*d[2] - iT[ 5]*d[3] + iT[ 2]*d[4] + add ) >> shift);

    dst[10] = Clip3(outputMinimum, outputMaximum, (int)( iT[10]*(src[ 0*line]-src[ 2*line]+src[ 3*line]-src[5*line]
                                                                +src[ 6*line]-src[ 8*line]+src[ 9*line]-src[11*line]
                                                                +src[12*line]-src[14*line]+src[15*line]) + add ) >> shift);

    dst[ 0] = Clip3(outputMinimum, outputMaximum, (int)( iT[0]*a[0] + iT[9]*b[0] + iT[2]*a[1] + iT[7]*b[1] + iT[4]*a[2] + iT[5]*b[2] + iT[6]*a[3] + iT[3]*b[3] + iT[8]*a[4] + iT[1]*b[4] + t + add ) >> shift);
    dst[ 1] = Clip3(outputMinimum, outputMaximum, (int)( iT[1]*c[0] - iT[8]*b[0] + iT[5]*c[1] - iT[4]*b[1] + iT[9]*c[2] - iT[0]*b[2] + iT[2]*a[3] + iT[7]*c[3] + iT[6]*a[4] + iT[3]*c[4] + t + add ) >> shift);
    dst[ 3] = Clip3(outputMinimum, outputMaximum, (int)( iT[3]*a[0] + iT[6]*b[0] + iT[0]*c[1] + iT[9]*a[1] + iT[1]*a[2] + iT[8]*c[2] + iT[4]*c[3] - iT[5]*b[3] - iT[2]*a[4] - iT[7]*b[4] - t + add ) >> shift);
    dst[ 4] = Clip3(outputMinimum, outputMaximum, (int)( iT[4]*c[0] - iT[5]*b[0] + iT[6]*c[1] + iT[3]*a[1] + iT[7]*a[2] + iT[2]*b[2] - iT[1]*c[3] + iT[8]*b[3] - iT[9]*c[4] - iT[0]*a[4] - t + add ) >> shift);
    dst[ 6] = Clip3(outputMinimum, outputMaximum, (int)( iT[6]*a[0] + iT[3]*b[0] + iT[9]*c[1] + iT[0]*a[1] - iT[1]*a[2] - iT[8]*b[2] - iT[4]*c[3] - iT[5]*a[3] - iT[2]*c[4] + iT[7]*b[4] + t + add ) >> shift);
    dst[ 7] = Clip3(outputMinimum, outputMaximum, (int)( iT[7]*c[0] - iT[2]*b[0] + iT[8]*a[1] + iT[1]*b[1] - iT[6]*c[2] + iT[3]*b[2] - iT[9]*a[3] - iT[0]*b[3] + iT[5]*c[4] - iT[4]*b[4] + t + add ) >> shift);
    dst[ 9] = Clip3(outputMinimum, outputMaximum, (int)( iT[9]*a[0] + iT[0]*b[0] + iT[2]*c[1] - iT[7]*b[1] - iT[5]*c[2] - iT[4]*a[2] + iT[3]*a[3] + iT[6]*b[3] + iT[8]*c[4] - iT[1]*b[4] - t + add ) >> shift);
    dst[12] = Clip3(outputMinimum, outputMaximum, (int)( iT[1]*c[0] + iT[8]*a[0] - iT[5]*a[1] - iT[4]*b[1] - iT[0]*c[2] + iT[9]*b[2] + iT[7]*c[3] - iT[2]*b[3] - iT[6]*c[4] - iT[3]*a[4] + t + add ) >> shift);
    dst[13] = Clip3(outputMinimum, outputMaximum, (int)( iT[7]*c[0] + iT[2]*a[0] - iT[8]*c[1] + iT[1]*b[1] + iT[3]*c[2] - iT[6]*b[2] + iT[0]*a[3] + iT[9]*b[3] - iT[5]*a[4] - iT[4]*b[4] + t + add ) >> shift);
    dst[15] = Clip3(outputMinimum, outputMaximum, (int)( iT[4]*c[0] + iT[5]*a[0] - iT[3]*c[1] - iT[6]*a[1] + iT[2]*c[2] + iT[7]*a[2] - iT[1]*c[3] - iT[8]*a[3] + iT[0]*c[4] + iT[9]*a[4] - t + add ) >> shift);

    src++;
    dst += 16;
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine * 16) * sizeof(TCoeff));
  }
#endif
}

void fastInverseDST7_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDST7P32[0] );
#else
  int j, k;
  TCoeff a[10][6];
  TCoeff t[2];
  TCoeff b[6];
  TCoeff c[2];

  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;
  const TMatrixCoeff *iT = g_trCoreDST7P32[0];
  const int  reducedLine = line - iSkipLine;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 6; k++)
    {
      a[0][k] = src[      k  * line] + src[(12 - k) * line];
      a[1][k] = src[      k  * line] - src[(13 + k) * line];
      a[2][k] = src[      k  * line] + src[(25 - k) * line];
      a[3][k] = src[      k  * line] - src[(26 + k) * line];
      a[4][k] = src[( 7 + k) * line] + src[(18 - k) * line];
      a[5][k] = src[( 7 + k) * line] - src[(20 + k) * line];
      a[6][k] = src[( 7 + k) * line] + src[(31 - k) * line];
      a[7][k] = src[(13 + k) * line] + src[(25 - k) * line];
      a[8][k] = src[(13 + k) * line] - src[(26 + k) * line];
      a[9][k] = src[(20 + k) * line] + src[(31 - k) * line];

      b[k] = src[k * line] - src[(12-k) * line] + src[(13+k) * line] - src[(25-k) * line] + src[(26+k) * line];
    }
    for (k = 0; k < 2; k++)
    {
      c[k] = src[k * line] - src[(4-k) * line] + src[(5+k) * line] - src[(9-k) * line] + src[(10+k) * line] - src[(14-k) * line] + src[(15+k)*line] - src[(19-k)*line] + src[(20+k)*line] - src[(24-k)*line] + src[(25+k)*line] - src[(29-k)*line] + src[(30+k)*line];
    }

    t[0] = iT[12] * src[6*line] + iT[25] * src[19*line];
    t[1] = iT[25] * src[6*line] - iT[12] * src[19*line];

    dst[ 0] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[1][0] - iT[11] * a[8][0] + iT[13] * a[7][0] + iT[24] * a[4][5] - iT[1] * a[8][5] + iT[10] * a[1][5] + iT[14] * a[4][0] + iT[23] * a[7][5] + iT[2] * a[1][1] - iT[9] * a[8][1] + iT[15] * a[7][1] + iT[22] * a[4][4] - iT[3] * a[8][4] + iT[8] * a[1][4] + iT[16] * a[4][1] + iT[21] * a[7][4] + iT[4] * a[1][2] - iT[7] * a[8][2] + iT[17] * a[7][2] + iT[20] * a[4][3] - iT[5] * a[8][3] + iT[6] * a[1][3] + iT[18] * a[4][2] + iT[19] * a[7][3] + t[0] + add) >> shift);
    dst[ 1] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[4][2] - iT[11] * a[6][2] + iT[13] * a[0][3] + iT[24] * a[5][2] + iT[1] * a[2][0] + iT[10] * a[7][0] + iT[14] * a[5][5] - iT[23] * a[9][5] + iT[2] * a[7][2] + iT[9] * a[2][2] - iT[15] * a[9][3] + iT[22] * a[5][3] - iT[3] * a[6][0] - iT[8] * a[4][0] + iT[16] * a[5][0] + iT[21] * a[0][5] - iT[4] * a[4][1] - iT[7] * a[6][1] + iT[17] * a[0][4] + iT[20] * a[5][1] + iT[5] * a[2][1] + iT[6] * a[7][1] + iT[18] * a[5][4] - iT[19] * a[9][4] + t[1] + add) >> shift);
    dst[ 2] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[2][4] - iT[11] * a[3][4] + iT[13] * a[0][4] + iT[24] * a[1][4] + iT[1] * a[4][3] + iT[10] * a[7][2] + iT[14] * a[1][2] - iT[23] * a[8][2] + iT[2] * a[3][0] - iT[9] * a[6][5] - iT[15] * a[8][0] + iT[22] * a[9][5] - iT[3] * a[6][4] + iT[8] * a[3][1] + iT[16] * a[9][4] - iT[21] * a[8][1] + iT[4] * a[7][3] + iT[7] * a[4][2] - iT[17] * a[8][3] + iT[20] * a[1][3] - iT[5] * a[3][5] - iT[6] * a[2][5] + iT[18] * a[1][5] + iT[19] * a[0][5] + t[1] + add) >> shift);
    dst[ 3] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[5][4] + iT[11] * a[0][1] - iT[13] * a[4][4] - iT[24] * a[6][4] - iT[1] * a[1][3] - iT[10] * a[0][3] + iT[14] * a[2][3] + iT[23] * a[3][3] - iT[2] * a[0][4] - iT[9] * a[1][4] + iT[15] * a[3][4] + iT[22] * a[2][4] + iT[3] * a[0][0] + iT[8] * a[5][5] - iT[16] * a[6][5] - iT[21] * a[4][5] + iT[4] * a[5][0] - iT[7] * a[9][0] + iT[17] * a[7][5] + iT[20] * a[2][5] - iT[5] * a[8][2] + iT[6] * a[9][3] - iT[18] * a[6][3] + iT[19] * a[3][2] + t[0] + add) >> shift);
    dst[ 5] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[1][5] + iT[11] * a[8][5] - iT[13] * a[7][5] - iT[24] * a[4][0] + iT[1] * a[5][1] + iT[10] * a[0][4] - iT[14] * a[4][1] - iT[23] * a[6][1] - iT[2] * a[8][3] + iT[9] * a[9][2] - iT[15] * a[6][2] + iT[22] * a[3][3] - iT[3] * a[0][2] - iT[8] * a[1][2] + iT[16] * a[3][2] + iT[21] * a[2][2] - iT[4] * a[9][4] + iT[7] * a[5][4] + iT[17] * a[2][1] + iT[20] * a[7][1] + iT[5] * a[1][0] - iT[6] * a[8][0] + iT[18] * a[7][0] + iT[19] * a[4][5] - t[0] + add) >> shift);
    dst[ 6] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[7][5] - iT[11] * a[2][5] + iT[13] * a[9][0] - iT[24] * a[5][0] + iT[1] * a[3][4] - iT[10] * a[6][1] - iT[14] * a[8][4] + iT[23] * a[9][1] + iT[2] * a[4][2] + iT[9] * a[7][3] + iT[15] * a[1][3] - iT[22] * a[8][3] - iT[3] * a[2][2] - iT[8] * a[3][2] + iT[16] * a[0][2] + iT[21] * a[1][2] - iT[4] * a[6][4] - iT[7] * a[4][4] + iT[17] * a[5][4] + iT[20] * a[0][1] + iT[5] * a[7][0] + iT[6] * a[2][0] - iT[18] * a[9][5] + iT[19] * a[5][5] - t[1] + add) >> shift);
    dst[ 7] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[6][3] - iT[11] * a[4][3] + iT[13] * a[5][3] + iT[24] * a[0][2] + iT[1] * a[7][1] + iT[10] * a[4][4] - iT[14] * a[8][1] + iT[23] * a[1][1] - iT[2] * a[7][5] - iT[9] * a[4][0] + iT[15] * a[8][5] - iT[22] * a[1][5] + iT[3] * a[7][3] + iT[8] * a[2][3] - iT[16] * a[9][2] + iT[21] * a[5][2] - iT[4] * a[6][5] + iT[7] * a[3][0] + iT[17] * a[9][5] - iT[20] * a[8][0] + iT[5] * a[6][1] - iT[6] * a[3][4] - iT[18] * a[9][1] + iT[19] * a[8][4] - t[1] + add) >> shift);
    dst[ 8] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[1][1] - iT[11] * a[0][1] + iT[13] * a[2][1] + iT[24] * a[3][1] + iT[1] * a[1][3] - iT[10] * a[8][3] + iT[14] * a[7][3] + iT[23] * a[4][2] - iT[2] * a[9][1] + iT[9] * a[8][4] - iT[15] * a[3][4] + iT[22] * a[6][1] + iT[3] * a[5][5] + iT[8] * a[0][0] - iT[16] * a[4][5] - iT[21] * a[6][5] + iT[4] * a[0][5] + iT[7] * a[1][5] - iT[17] * a[3][5] - iT[20] * a[2][5] + iT[5] * a[5][3] - iT[6] * a[9][3] + iT[18] * a[7][2] + iT[19] * a[2][2] - t[0] + add) >> shift);
    dst[10] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[8][3] - iT[11] * a[1][3] - iT[13] * a[4][2] - iT[24] * a[7][3] - iT[1] * a[8][0] + iT[10] * a[1][0] + iT[14] * a[4][5] + iT[23] * a[7][0] + iT[2] * a[5][3] + iT[9] * a[0][2] - iT[15] * a[4][3] - iT[22] * a[6][3] - iT[3] * a[5][0] - iT[8] * a[0][5] + iT[16] * a[4][0] + iT[21] * a[6][0] + iT[4] * a[1][4] + iT[7] * a[0][4] - iT[17] * a[2][4] - iT[20] * a[3][4] - iT[5] * a[1][1] - iT[6] * a[0][1] + iT[18] * a[2][1] + iT[19] * a[3][1] + t[0] + add) >> shift);
    dst[11] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[7][0] + iT[11] * a[2][0] - iT[13] * a[9][5] + iT[24] * a[5][5] + iT[1] * a[2][5] + iT[10] * a[7][5] + iT[14] * a[5][0] - iT[23] * a[9][0] - iT[2] * a[2][1] - iT[9] * a[3][1] + iT[15] * a[0][1] + iT[22] * a[1][1] - iT[3] * a[7][4] - iT[8] * a[4][1] + iT[16] * a[8][4] - iT[21] * a[1][4] + iT[4] * a[3][2] - iT[7] * a[6][3] - iT[17] * a[8][2] + iT[20] * a[9][3] + iT[5] * a[4][2] + iT[6] * a[6][2] - iT[18] * a[0][3] - iT[19] * a[5][2] + t[1] + add) >> shift);
    dst[13] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[9][5] - iT[11] * a[8][0] + iT[13] * a[3][0] - iT[24] * a[6][5] - iT[1] * a[8][5] + iT[10] * a[9][0] - iT[14] * a[6][0] + iT[23] * a[3][5] + iT[2] * a[5][4] - iT[9] * a[9][4] + iT[15] * a[7][1] + iT[22] * a[2][1] - iT[3] * a[1][4] + iT[8] * a[8][4] - iT[16] * a[7][4] - iT[21] * a[4][1] - iT[4] * a[0][2] - iT[7] * a[5][3] + iT[17] * a[6][3] + iT[20] * a[4][3] + iT[5] * a[0][3] + iT[6] * a[1][3] - iT[18] * a[3][3] - iT[19] * a[2][3] + t[0] + add) >> shift);
    dst[15] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[9][1] + iT[11] * a[5][1] + iT[13] * a[2][4] + iT[24] * a[7][4] + iT[1] * a[9][3] - iT[10] * a[5][3] - iT[14] * a[2][2] - iT[23] * a[7][2] - iT[2] * a[9][5] + iT[9] * a[5][5] + iT[15] * a[2][0] + iT[22] * a[7][0] + iT[3] * a[9][4] - iT[8] * a[8][1] + iT[16] * a[3][1] - iT[21] * a[6][4] - iT[4] * a[9][2] + iT[7] * a[8][3] - iT[17] * a[3][3] + iT[20] * a[6][2] + iT[5] * a[9][0] - iT[6] * a[8][5] + iT[18] * a[3][5] - iT[19] * a[6][0] - t[0] + add) >> shift);
    dst[16] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[4][4] + iT[11] * a[7][1] + iT[13] * a[1][1] - iT[24] * a[8][1] + iT[1] * a[6][2] - iT[10] * a[3][3] - iT[14] * a[9][2] + iT[23] * a[8][3] - iT[2] * a[6][1] - iT[9] * a[4][1] + iT[15] * a[5][1] + iT[22] * a[0][4] - iT[3] * a[4][5] - iT[8] * a[6][5] + iT[16] * a[0][0] + iT[21] * a[5][5] - iT[4] * a[6][0] + iT[7] * a[3][5] + iT[17] * a[9][0] - iT[20] * a[8][5] + iT[5] * a[6][3] + iT[6] * a[4][3] - iT[18] * a[5][3] - iT[19] * a[0][2] - t[1] + add) >> shift);
    dst[17] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[7][2] - iT[11] * a[4][3] + iT[13] * a[8][2] - iT[24] * a[1][2] + iT[1] * a[7][1] + iT[10] * a[2][1] - iT[14] * a[9][4] + iT[23] * a[5][4] - iT[2] * a[3][5] + iT[9] * a[6][0] + iT[15] * a[8][5] - iT[22] * a[9][0] - iT[3] * a[2][3] - iT[8] * a[7][3] - iT[16] * a[5][2] + iT[21] * a[9][2] + iT[4] * a[4][5] + iT[7] * a[7][0] + iT[17] * a[1][0] - iT[20] * a[8][0] - iT[5] * a[2][4] - iT[6] * a[3][4] + iT[18] * a[0][4] + iT[19] * a[1][4] - t[1] + add) >> shift);
    dst[18] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[9][0] + iT[11] * a[8][5] - iT[13] * a[3][5] + iT[24] * a[6][0] + iT[1] * a[5][1] - iT[10] * a[9][1] + iT[14] * a[7][4] + iT[23] * a[2][4] + iT[2] * a[0][3] + iT[9] * a[5][2] - iT[15] * a[6][2] - iT[22] * a[4][2] + iT[3] * a[1][2] + iT[8] * a[0][2] - iT[16] * a[2][2] - iT[21] * a[3][2] - iT[4] * a[8][1] + iT[7] * a[1][1] + iT[17] * a[4][4] + iT[20] * a[7][1] + iT[5] * a[9][5] - iT[6] * a[8][0] + iT[18] * a[3][0] - iT[19] * a[6][5] - t[0] + add) >> shift);
    dst[20] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[8][2] - iT[11] * a[9][3] + iT[13] * a[6][3] - iT[24] * a[3][2] + iT[1] * a[0][1] + iT[10] * a[5][4] - iT[14] * a[6][4] - iT[23] * a[4][4] + iT[2] * a[1][5] + iT[9] * a[0][5] - iT[15] * a[2][5] - iT[22] * a[3][5] - iT[3] * a[9][2] + iT[8] * a[5][2] + iT[16] * a[2][3] + iT[21] * a[7][3] + iT[4] * a[5][5] - iT[7] * a[9][5] + iT[17] * a[7][0] + iT[20] * a[2][0] + iT[5] * a[0][4] + iT[6] * a[5][1] - iT[18] * a[6][1] - iT[19] * a[4][1] + t[0] + add) >> shift);
    dst[21] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[2][1] - iT[11] * a[7][1] - iT[13] * a[5][4] + iT[24] * a[9][4] - iT[1] * a[6][2] - iT[10] * a[4][2] + iT[14] * a[5][2] + iT[23] * a[0][3] - iT[2] * a[2][4] - iT[9] * a[7][4] - iT[15] * a[5][1] + iT[22] * a[9][1] - iT[3] * a[6][5] - iT[8] * a[4][5] + iT[16] * a[5][5] + iT[21] * a[0][0] - iT[4] * a[4][0] - iT[7] * a[7][5] - iT[17] * a[1][5] + iT[20] * a[8][5] - iT[5] * a[7][2] - iT[6] * a[4][3] + iT[18] * a[8][2] - iT[19] * a[1][2] + t[1] + add) >> shift);
    dst[22] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[6][1] - iT[11] * a[3][4] - iT[13] * a[9][1] + iT[24] * a[8][4] + iT[1] * a[4][3] + iT[10] * a[6][3] - iT[14] * a[0][2] - iT[23] * a[5][3] + iT[2] * a[7][0] + iT[9] * a[4][5] - iT[15] * a[8][0] + iT[22] * a[1][0] - iT[3] * a[3][1] + iT[8] * a[6][4] + iT[16] * a[8][1] - iT[21] * a[9][4] - iT[4] * a[2][3] - iT[7] * a[3][3] + iT[17] * a[0][3] + iT[20] * a[1][3] - iT[5] * a[7][5] - iT[6] * a[2][5] + iT[18] * a[9][0] - iT[19] * a[5][0] + t[1] + add) >> shift);
    dst[23] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[0][3] - iT[11] * a[1][3] + iT[13] * a[3][3] + iT[24] * a[2][3] - iT[1] * a[8][0] + iT[10] * a[9][5] - iT[14] * a[6][5] + iT[23] * a[3][0] + iT[2] * a[8][2] - iT[9] * a[1][2] - iT[15] * a[4][3] - iT[22] * a[7][2] + iT[3] * a[0][5] + iT[8] * a[5][0] - iT[16] * a[6][0] - iT[21] * a[4][0] + iT[4] * a[8][4] - iT[7] * a[9][1] + iT[17] * a[6][1] - iT[20] * a[3][4] - iT[5] * a[5][4] - iT[6] * a[0][1] + iT[18] * a[4][4] + iT[19] * a[6][4] + t[0] + add) >> shift);
    dst[26] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[3][0] - iT[11] * a[2][0] + iT[13] * a[1][0] + iT[24] * a[0][0] - iT[1] * a[2][5] - iT[10] * a[3][5] + iT[14] * a[0][5] + iT[23] * a[1][5] + iT[2] * a[4][4] + iT[9] * a[6][4] - iT[15] * a[0][1] - iT[22] * a[5][4] - iT[3] * a[4][1] - iT[8] * a[7][4] - iT[16] * a[1][4] + iT[21] * a[8][4] + iT[4] * a[2][2] + iT[7] * a[7][2] + iT[17] * a[5][3] - iT[20] * a[9][3] + iT[5] * a[3][3] - iT[6] * a[6][2] - iT[18] * a[8][3] + iT[19] * a[9][2] - t[1] + add) >> shift);
    dst[27] = Clip3(outputMinimum, outputMaximum, (int)(-iT[0] * a[3][3] + iT[11] * a[6][2] + iT[13] * a[8][3] - iT[24] * a[9][2] - iT[1] * a[2][0] - iT[10] * a[3][0] + iT[14] * a[0][0] + iT[23] * a[1][0] - iT[2] * a[6][3] + iT[9] * a[3][2] + iT[15] * a[9][3] - iT[22] * a[8][2] - iT[3] * a[4][0] - iT[8] * a[6][0] + iT[16] * a[0][5] + iT[21] * a[5][0] - iT[4] * a[7][4] - iT[7] * a[2][4] + iT[17] * a[9][1] - iT[20] * a[5][1] - iT[5] * a[4][4] - iT[6] * a[7][1] - iT[18] * a[1][1] + iT[19] * a[8][1] - t[1] + add) >> shift);
    dst[28] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[0][4] + iT[11] * a[5][1] - iT[13] * a[6][1] - iT[24] * a[4][1] + iT[1] * a[9][3] - iT[10] * a[8][2] + iT[14] * a[3][2] - iT[23] * a[6][3] - iT[2] * a[1][0] - iT[9] * a[0][0] + iT[15] * a[2][0] + iT[22] * a[3][0] + iT[3] * a[8][1] - iT[8] * a[9][4] + iT[16] * a[6][4] - iT[21] * a[3][1] - iT[4] * a[5][2] - iT[7] * a[0][3] + iT[17] * a[4][2] + iT[20] * a[6][2] + iT[5] * a[1][5] - iT[6] * a[8][5] + iT[18] * a[7][5] + iT[19] * a[4][0] - t[0] + add) >> shift);
    dst[30] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[5][3] - iT[11] * a[9][3] + iT[13] * a[7][2] + iT[24] * a[2][2] + iT[1] * a[0][1] + iT[10] * a[1][1] - iT[14] * a[3][1] - iT[23] * a[2][1] + iT[2] * a[9][0] - iT[9] * a[5][0] - iT[15] * a[2][5] - iT[22] * a[7][5] - iT[3] * a[5][2] + iT[8] * a[9][2] - iT[16] * a[7][3] - iT[21] * a[2][3] - iT[4] * a[0][0] - iT[7] * a[1][0] + iT[17] * a[3][0] + iT[20] * a[2][0] - iT[5] * a[9][1] + iT[6] * a[5][1] + iT[18] * a[2][4] + iT[19] * a[7][4] + t[0] + add) >> shift);
    dst[31] = Clip3(outputMinimum, outputMaximum, (int)( iT[0] * a[3][5] + iT[11] * a[2][5] - iT[13] * a[1][5] - iT[24] * a[0][5] - iT[1] * a[3][4] - iT[10] * a[2][4] + iT[14] * a[1][4] + iT[23] * a[0][4] + iT[2] * a[3][3] + iT[9] * a[2][3] - iT[15] * a[1][3] - iT[22] * a[0][3] - iT[3] * a[3][2] - iT[8] * a[2][2] + iT[16] * a[1][2] + iT[21] * a[0][2] + iT[4] * a[3][1] + iT[7] * a[2][1] - iT[17] * a[1][1] - iT[20] * a[0][1] - iT[5] * a[3][0] - iT[6] * a[2][0] + iT[18] * a[1][0] + iT[19] * a[0][0] + t[1] + add) >> shift);

    dst[ 4] = Clip3(outputMinimum, outputMaximum, (int)(iT[ 4] * b[0] + iT[14] * b[1] + iT[24] * b[2] + iT[29] * b[3] + iT[19] * b[4] + iT[ 9] * b[5] + add) >> shift);
    dst[ 9] = Clip3(outputMinimum, outputMaximum, (int)(iT[ 9] * b[0] + iT[29] * b[1] + iT[14] * b[2] - iT[ 4] * b[3] - iT[24] * b[4] - iT[19] * b[5] + add) >> shift);
    dst[14] = Clip3(outputMinimum, outputMaximum, (int)(iT[14] * b[0] + iT[19] * b[1] - iT[ 9] * b[2] - iT[24] * b[3] + iT[ 4] * b[4] + iT[29] * b[5] + add) >> shift);
    dst[19] = Clip3(outputMinimum, outputMaximum, (int)(iT[19] * b[0] + iT[ 4] * b[1] - iT[29] * b[2] + iT[ 9] * b[3] + iT[14] * b[4] - iT[24] * b[5] + add) >> shift);
    dst[24] = Clip3(outputMinimum, outputMaximum, (int)(iT[24] * b[0] - iT[ 9] * b[1] - iT[ 4] * b[2] + iT[19] * b[3] - iT[29] * b[4] + iT[14] * b[5] + add) >> shift);
    dst[29] = Clip3(outputMinimum, outputMaximum, (int)(iT[29] * b[0] - iT[24] * b[1] + iT[19] * b[2] - iT[14] * b[3] + iT[ 9] * b[4] - iT[ 4] * b[5] + add) >> shift);

    dst[12] = Clip3(outputMinimum, outputMaximum, (int)(iT[12]*c[0] + iT[25]*c[1] + add) >> shift);
    dst[25] = Clip3(outputMinimum, outputMaximum, (int)(iT[25]*c[0] - iT[12]*c[1] + add) >> shift);

    src++;
    dst += 32;
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine * 32) * sizeof(TCoeff));
  }
#endif
}


// ********************************** DCT-VIII **********************************

void fastInverseDCT8_B4(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  _fastInverseMM<4>( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDCT8P4[0] );
#else
  int i;
  int rnd_factor = 1 << (shift - 1);

  const TMatrixCoeff *iT = g_trCoreDCT8P4[0];

#if ENABLE_SIMD_TCOEFF_OPS
  TCoeff* orgDst = dst;
#endif

  int c[4];
  const int  reducedLine = line - iSkipLine;
  for (i = 0; i<reducedLine; i++)
  {
    // Intermediate Variables
    c[0] = src[0 * line] + src[3 * line];
    c[1] = src[2 * line] + src[0 * line];
    c[2] = src[3 * line] - src[2 * line];
    c[3] = iT[1] * src[1 * line];

#if ENABLE_SIMD_TCOEFF_OPS
    dst[0] = iT[3] * c[0] + iT[2] * c[1] + c[3];
    dst[1] = iT[1] * ( src[0 * line] - src[2 * line] - src[3 * line] );
    dst[2] = iT[3] * c[2] + iT[2] * c[0] - c[3];
    dst[3] = iT[3] * c[1] - iT[2] * c[2] - c[3];
#else
    dst[0] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[0] + iT[2] * c[1] + c[3] + rnd_factor) >> shift);
    dst[1] = Clip3(outputMinimum, outputMaximum, (iT[1] * (src[0 * line] - src[2 * line] - src[3 * line]) + rnd_factor) >> shift);
    dst[2] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[2] + iT[2] * c[0] - c[3] + rnd_factor) >> shift);
    dst[3] = Clip3(outputMinimum, outputMaximum, (iT[3] * c[1] - iT[2] * c[2] - c[3] + rnd_factor) >> shift);
#endif

    dst += 4;
    src++;
  }

#if ENABLE_SIMD_TCOEFF_OPS
  g_tCoeffOps.roundClip4( orgDst, 4, reducedLine, 4, outputMinimum, outputMaximum, rnd_factor, shift );
#endif

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine << 2) * sizeof(TCoeff));
  }
#endif
}

void fastInverseDCT8_B8(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
  _fastInverseMM< 8 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDCT8P8[0] );
}

void fastInverseDCT8_B16(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  _fastInverseMM< 16 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDCT8P16[0] );
#else
  int j, k;
  TCoeff a[5], b[5], c[5], d[5], t;

  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P16[0];

  const int reducedLine = line - iSkipLine;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 5; k++)
    {
      a[k] = src[(15 - k ) * line] + src[( 4 - k) * line];
      b[k] = src[( 6 + k ) * line] + src[( 4 - k) * line];
      c[k] = src[(15 - k ) * line] - src[( 6 + k) * line];
      d[k] = src[(15 - k ) * line] + src[( 6 + k) * line] - src[(4 - k) * line];
    }

    t = iT[10] * src[5*line];

    dst[ 1] = Clip3(outputMinimum, outputMaximum, (int)( - iT[ 2]*d[0] - iT[ 5]*d[1] - iT[ 8]*d[2] - iT[11]*d[3] - iT[14]*d[4] + add) >> shift);
    dst[ 4] = Clip3(outputMinimum, outputMaximum, (int)(   iT[ 8]*d[0] + iT[14]*d[1] + iT[ 5]*d[2] - iT[ 2]*d[3] - iT[11]*d[4] + add) >> shift);
    dst[ 7] = Clip3(outputMinimum, outputMaximum, (int)( - iT[14]*d[0] - iT[ 2]*d[1] + iT[11]*d[2] + iT[ 5]*d[3] - iT[ 8]*d[4] + add) >> shift);
    dst[10] = Clip3(outputMinimum, outputMaximum, (int)(   iT[11]*d[0] - iT[ 8]*d[1] - iT[ 2]*d[2] + iT[14]*d[3] - iT[ 5]*d[4] + add) >> shift);
    dst[13] = Clip3(outputMinimum, outputMaximum, (int)( - iT[ 5]*d[0] + iT[11]*d[1] - iT[14]*d[2] + iT[ 8]*d[3] - iT[ 2]*d[4] + add) >> shift);

    dst[ 5] = Clip3(outputMinimum, outputMaximum, (int)( - iT[10] * (src[15 * line] + src[14 * line] - src[12 * line] - src[11 * line] + src[9 * line] + src[8 * line] - src[6 * line] - src[5 * line] + src[3 * line] + src[2 * line] - src[0 * line]) + add) >> shift);

    dst[ 0] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0]*a[0] + iT[9]*b[0] + iT[1]*a[1] + iT[8]*b[1] + iT[2]*a[2] + iT[7]*b[2] + iT[3]*a[3] + iT[6]*b[3] + iT[4]*a[4] + iT[5]*b[4] + t + add ) >> shift );
    dst[ 2] = Clip3(outputMinimum, outputMaximum, (int)(   iT[4]*c[0] - iT[5]*b[0] + iT[9]*c[1] - iT[0]*b[1] + iT[6]*c[2] + iT[3]*a[2] + iT[1]*c[3] + iT[8]*a[3] + iT[7]*a[4] + iT[2]*b[4] - t + add ) >> shift );
    dst[ 3] = Clip3(outputMinimum, outputMaximum, (int)( - iT[6]*a[0] - iT[3]*b[0] - iT[2]*c[1] - iT[7]*a[1] - iT[9]*c[2] - iT[0]*a[2] - iT[4]*c[3] + iT[5]*b[3] + iT[1]*a[4] + iT[8]*b[4] - t + add ) >> shift );
    dst[ 6] = Clip3(outputMinimum, outputMaximum, (int)(   iT[8]*a[0] + iT[1]*c[0] + iT[6]*c[1] - iT[3]*b[1] - iT[5]*a[2] - iT[4]*b[2] - iT[7]*c[3] - iT[2]*a[3] - iT[0]*c[4] + iT[9]*b[4] + t + add ) >> shift );
    dst[ 8] = Clip3(outputMinimum, outputMaximum, (int)(   iT[4]*c[0] + iT[5]*a[0] - iT[0]*c[1] + iT[9]*b[1] - iT[3]*c[2] - iT[6]*a[2] + iT[1]*c[3] - iT[8]*b[3] + iT[2]*c[4] + iT[7]*a[4] - t + add ) >> shift );
    dst[ 9] = Clip3(outputMinimum, outputMaximum, (int)( - iT[7]*c[0] - iT[2]*a[0] + iT[4]*a[1] + iT[5]*b[1] + iT[8]*c[2] - iT[1]*b[2] - iT[9]*a[3] - iT[0]*b[3] - iT[3]*c[4] + iT[6]*b[4] - t + add ) >> shift );
    dst[11] = Clip3(outputMinimum, outputMaximum, (int)( - iT[9]*a[0] - iT[0]*b[0] + iT[8]*c[1] + iT[1]*a[1] - iT[2]*c[2] + iT[7]*b[2] - iT[6]*a[3] - iT[3]*b[3] + iT[5]*c[4] + iT[4]*a[4] + t + add ) >> shift );
    dst[12] = Clip3(outputMinimum, outputMaximum, (int)(   iT[7]*c[0] - iT[2]*b[0] - iT[5]*c[1] - iT[4]*a[1] + iT[8]*a[2] + iT[1]*b[2] - iT[0]*a[3] - iT[9]*b[3] - iT[6]*c[4] + iT[3]*b[4] + t + add ) >> shift );
    dst[14] = Clip3(outputMinimum, outputMaximum, (int)(   iT[3]*a[0] + iT[6]*b[0] - iT[7]*a[1] - iT[2]*b[1] + iT[0]*c[2] + iT[9]*a[2] - iT[4]*c[3] - iT[5]*a[3] + iT[8]*c[4] + iT[1]*a[4] - t + add ) >> shift );
    dst[15] = Clip3(outputMinimum, outputMaximum, (int)( - iT[1]*c[0] + iT[8]*b[0] + iT[3]*c[1] - iT[6]*b[1] - iT[5]*c[2] + iT[4]*b[2] + iT[7]*c[3] - iT[2]*b[3] - iT[9]*c[4] + iT[0]*b[4] - t + add ) >> shift );

    src++;
    dst += 16;
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine * 16) * sizeof(TCoeff));
  }
#endif
}

void fastInverseDCT8_B32(const TCoeff *src, TCoeff *dst, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum)
{
#if 1
  _fastInverseMM< 32 >( src, dst, shift, line, iSkipLine, iSkipLine2, outputMinimum, outputMaximum, g_trCoreDCT8P32[0] );
#else
  int j, k;
  TCoeff a[10][6];
  TCoeff t[2];
  TCoeff b[6];
  TCoeff c[2];
  TCoeff add = (shift > 0) ? (1 << (shift - 1)) : 0;

  const TMatrixCoeff *iT = g_trCoreDST7P32[0];

  const int  reducedLine = line - iSkipLine;

  for (j = 0; j < reducedLine; j++)
  {
    for (k = 0; k < 6; k++)
    {
      a[0][k] = src[(31 - k)*line] - src[(20 + k)*line];
      a[1][k] = src[(31 - k)*line] + src[(18 - k)*line];
      a[2][k] = src[(31 - k)*line] + src[( 7 + k)*line];
      a[3][k] = src[(31 - k)*line] - src[( 5 - k)*line];
      a[4][k] = src[(25 - k)*line] + src[(13 + k)*line];
      a[5][k] = src[(25 - k)*line] + src[(12 - k)*line];
      a[6][k] = src[(25 - k)*line] - src[      k *line];
      a[7][k] = src[(18 - k)*line] - src[( 7 + k)*line];
      a[8][k] = src[(18 - k)*line] + src[( 5 - k)*line];
      a[9][k] = src[(12 - k)*line] + src[      k *line];

      b[k] = src[(31 - k)*line] + src[(20 + k)*line] - src[(18 - k)*line] - src[(7 + k)*line] + src[(5 - k)*line];
    }

    for (k = 0; k < 2; k++)
    {
      c[k] = src[(31 - k)*line] + src[(28 + k)*line] - src[(26 - k)*line] - src[(23 + k)*line] + src[(21 - k)*line] + src[(18 + k)*line] - src[(16 - k)*line] - src[(13 + k)*line] + src[(11 - k)*line] + src[(8 + k)*line] - src[(6 - k)*line] - src[(3 + k)*line] + src[(1 - k)*line];
    }

    t[0] = iT[12] * src[19 * line] + iT[25] * src[ 6 * line];
    t[1] = iT[12] * src[ 6 * line] - iT[25] * src[19 * line];

    dst[ 0] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[3][0] + iT[11] * a[6][5] + iT[13] * a[8][0] + iT[24] * a[9][5] + iT[1] * a[3][1] + iT[10] * a[6][4] + iT[14] * a[8][1] + iT[23] * a[9][4] + iT[2] * a[3][2] + iT[9] * a[6][3] + iT[15] * a[8][2] + iT[22] * a[9][3] + iT[3] * a[3][3] + iT[8] * a[6][2] + iT[16] * a[8][3] + iT[21] * a[9][2] + iT[4] * a[3][4] + iT[7] * a[6][1] + iT[17] * a[8][4] + iT[20] * a[9][1] + iT[5] * a[3][5] + iT[6] * a[6][0] + iT[18] * a[8][5] + iT[19] * a[9][0] + t[0] + add) >> shift);
    dst[ 1] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[5][2] - iT[11] * a[0][3] - iT[13] * a[4][2] - iT[24] * a[6][2] - iT[1] * a[9][1] - iT[10] * a[8][4] - iT[14] * a[3][4] - iT[23] * a[6][1] - iT[2] * a[0][0] + iT[9] * a[5][5] - iT[15] * a[6][5] - iT[22] * a[4][5] + iT[3] * a[5][3] - iT[8] * a[0][2] - iT[16] * a[4][3] - iT[21] * a[6][3] - iT[4] * a[9][0] - iT[7] * a[8][5] - iT[17] * a[3][5] - iT[20] * a[6][0] - iT[5] * a[0][1] + iT[6] * a[5][4] - iT[18] * a[6][4] - iT[19] * a[4][4] + t[1] + add) >> shift);
    dst[ 3] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[9][4] + iT[11] * a[5][4] - iT[13] * a[2][1] + iT[24] * a[7][1] + iT[1] * a[0][3] + iT[10] * a[1][3] - iT[14] * a[3][3] - iT[23] * a[2][3] - iT[2] * a[8][5] - iT[9] * a[9][0] - iT[15] * a[6][0] - iT[22] * a[3][5] + iT[3] * a[1][4] + iT[8] * a[0][4] - iT[16] * a[2][4] - iT[21] * a[3][4] + iT[4] * a[5][3] + iT[7] * a[9][3] + iT[17] * a[7][2] - iT[20] * a[2][2] - iT[5] * a[8][0] - iT[6] * a[1][0] + iT[18] * a[4][5] + iT[19] * a[7][0] - t[1] + add) >> shift);
    dst[ 4] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[3][2] - iT[11] * a[2][2] + iT[13] * a[1][2] + iT[24] * a[0][2] + iT[1] * a[6][0] + iT[10] * a[3][5] + iT[14] * a[9][0] + iT[23] * a[8][5] - iT[2] * a[2][3] - iT[9] * a[3][3] + iT[15] * a[0][3] + iT[22] * a[1][3] - iT[3] * a[7][0] + iT[8] * a[2][0] - iT[16] * a[9][5] - iT[21] * a[5][5] + iT[4] * a[4][4] + iT[7] * a[6][4] + iT[17] * a[0][1] - iT[20] * a[5][4] - iT[5] * a[7][4] - iT[6] * a[4][1] + iT[18] * a[8][4] + iT[19] * a[1][4] - t[0] + add) >> shift);
    dst[ 5] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[3][5] + iT[11] * a[6][0] + iT[13] * a[8][5] + iT[24] * a[9][0] - iT[1] * a[6][5] - iT[10] * a[3][0] - iT[14] * a[9][5] - iT[23] * a[8][0] + iT[2] * a[7][4] - iT[9] * a[2][4] + iT[15] * a[9][1] + iT[22] * a[5][1] + iT[3] * a[7][1] + iT[8] * a[4][4] - iT[16] * a[8][1] - iT[21] * a[1][1] - iT[4] * a[6][2] - iT[7] * a[4][2] + iT[17] * a[5][2] - iT[20] * a[0][3] + iT[5] * a[3][2] + iT[6] * a[2][2] - iT[18] * a[1][2] - iT[19] * a[0][2] - t[0] + add) >> shift);
    dst[ 8] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[9][3] + iT[11] * a[8][2] + iT[13] * a[3][2] + iT[24] * a[6][3] + iT[1] * a[1][5] + iT[10] * a[0][5] - iT[14] * a[2][5] - iT[23] * a[3][5] - iT[2] * a[1][3] - iT[9] * a[8][3] + iT[15] * a[7][3] + iT[22] * a[4][2] - iT[3] * a[9][5] - iT[8] * a[5][5] + iT[16] * a[2][0] - iT[21] * a[7][0] - iT[4] * a[1][1] - iT[7] * a[0][1] + iT[17] * a[2][1] + iT[20] * a[3][1] + iT[5] * a[5][1] + iT[6] * a[9][1] + iT[18] * a[7][4] - iT[19] * a[2][4] + t[1] + add) >> shift);
    dst[ 9] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[2][1] + iT[11] * a[3][1] - iT[13] * a[0][1] - iT[24] * a[1][1] - iT[1] * a[7][3] + iT[10] * a[2][3] - iT[14] * a[9][2] - iT[23] * a[5][2] - iT[2] * a[4][0] - iT[9] * a[7][5] + iT[15] * a[1][5] + iT[22] * a[8][5] - iT[3] * a[3][4] - iT[8] * a[2][4] + iT[16] * a[1][4] + iT[21] * a[0][4] - iT[4] * a[6][3] - iT[7] * a[3][2] - iT[17] * a[9][3] - iT[20] * a[8][2] - iT[5] * a[4][5] - iT[6] * a[6][5] - iT[18] * a[0][0] + iT[19] * a[5][5] + t[0] + add) >> shift);
    dst[10] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[6][1] - iT[11] * a[4][1] + iT[13] * a[5][1] - iT[24] * a[0][4] + iT[1] * a[2][2] - iT[10] * a[7][2] - iT[14] * a[5][3] - iT[23] * a[9][3] + iT[2] * a[6][4] + iT[9] * a[4][4] - iT[15] * a[5][4] + iT[22] * a[0][1] - iT[3] * a[2][5] + iT[8] * a[7][5] + iT[16] * a[5][0] + iT[21] * a[9][0] - iT[4] * a[7][0] - iT[7] * a[4][5] + iT[17] * a[8][0] + iT[20] * a[1][0] + iT[5] * a[4][2] + iT[6] * a[7][3] - iT[18] * a[1][3] - iT[19] * a[8][3] + t[0] + add) >> shift);
    dst[11] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[1][3] - iT[11] * a[0][3] + iT[13] * a[2][3] + iT[24] * a[3][3] - iT[1] * a[9][1] - iT[10] * a[5][1] + iT[14] * a[2][4] - iT[23] * a[7][4] - iT[2] * a[8][0] - iT[9] * a[9][5] - iT[15] * a[6][5] - iT[22] * a[3][0] + iT[3] * a[0][2] - iT[8] * a[5][3] + iT[16] * a[6][3] + iT[21] * a[4][3] + iT[4] * a[5][0] - iT[7] * a[0][5] - iT[17] * a[4][0] - iT[20] * a[6][0] + iT[5] * a[9][4] + iT[6] * a[5][4] - iT[18] * a[2][1] + iT[19] * a[7][1] + t[1] + add) >> shift);
    dst[13] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[0][0] + iT[11] * a[1][0] - iT[13] * a[3][0] - iT[24] * a[2][0] + iT[1] * a[5][4] - iT[10] * a[0][1] - iT[14] * a[4][4] - iT[23] * a[6][4] - iT[2] * a[9][3] - iT[9] * a[5][3] + iT[15] * a[2][2] - iT[22] * a[7][2] + iT[3] * a[8][3] + iT[8] * a[9][2] + iT[16] * a[6][2] + iT[21] * a[3][3] - iT[4] * a[1][4] - iT[7] * a[8][4] + iT[17] * a[7][4] + iT[20] * a[4][1] + iT[5] * a[0][5] + iT[6] * a[1][5] - iT[18] * a[3][5] - iT[19] * a[2][5] - t[1] + add) >> shift);
    dst[14] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[4][2] + iT[11] * a[7][3] - iT[13] * a[1][3] - iT[24] * a[8][3] + iT[1] * a[4][1] + iT[10] * a[6][1] + iT[14] * a[0][4] - iT[23] * a[5][1] - iT[2] * a[3][0] - iT[9] * a[2][0] + iT[15] * a[1][0] + iT[22] * a[0][0] - iT[3] * a[6][3] - iT[8] * a[4][3] + iT[16] * a[5][3] - iT[21] * a[0][2] - iT[4] * a[7][5] - iT[7] * a[4][0] + iT[17] * a[8][5] + iT[20] * a[1][5] + iT[5] * a[6][4] + iT[6] * a[3][1] + iT[18] * a[9][4] + iT[19] * a[8][1] - t[0] + add) >> shift);
    dst[15] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[7][4] + iT[11] * a[4][1] - iT[13] * a[8][4] - iT[24] * a[1][4] - iT[1] * a[2][2] - iT[10] * a[3][2] + iT[14] * a[0][2] + iT[23] * a[1][2] - iT[2] * a[2][1] + iT[9] * a[7][1] + iT[15] * a[5][4] + iT[22] * a[9][4] + iT[3] * a[7][5] - iT[8] * a[2][5] + iT[16] * a[9][0] + iT[21] * a[5][0] + iT[4] * a[2][0] + iT[7] * a[3][0] - iT[17] * a[0][0] - iT[20] * a[1][0] + iT[5] * a[2][3] - iT[6] * a[7][3] - iT[18] * a[5][2] - iT[19] * a[9][2] - t[0] + add) >> shift);
    dst[16] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[0][1] + iT[11] * a[5][4] - iT[13] * a[6][4] - iT[24] * a[4][4] + iT[1] * a[0][3] - iT[10] * a[5][2] + iT[14] * a[6][2] + iT[23] * a[4][2] - iT[2] * a[0][5] + iT[9] * a[5][0] - iT[15] * a[6][0] - iT[22] * a[4][0] - iT[3] * a[0][4] - iT[8] * a[1][4] + iT[16] * a[3][4] + iT[21] * a[2][4] + iT[4] * a[0][2] + iT[7] * a[1][2] - iT[17] * a[3][2] - iT[20] * a[2][2] - iT[5] * a[0][0] - iT[6] * a[1][0] + iT[18] * a[3][0] + iT[19] * a[2][0] - t[1] + add) >> shift);
    dst[18] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[0][5] + iT[11] * a[1][5] - iT[13] * a[3][5] - iT[24] * a[2][5] - iT[1] * a[1][0] - iT[10] * a[0][0] + iT[14] * a[2][0] + iT[23] * a[3][0] - iT[2] * a[5][1] + iT[9] * a[0][4] + iT[15] * a[4][1] + iT[22] * a[6][1] - iT[3] * a[8][1] - iT[8] * a[1][1] + iT[16] * a[4][4] + iT[21] * a[7][1] - iT[4] * a[9][2] - iT[7] * a[5][2] + iT[17] * a[2][3] - iT[20] * a[7][3] - iT[5] * a[9][3] - iT[6] * a[8][2] - iT[18] * a[3][2] - iT[19] * a[6][3] + t[1] + add) >> shift);
    dst[20] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[4][0] - iT[11] * a[6][0] - iT[13] * a[0][5] + iT[24] * a[5][0] + iT[1] * a[6][5] + iT[10] * a[4][5] - iT[14] * a[5][5] + iT[23] * a[0][0] - iT[2] * a[6][1] - iT[9] * a[3][4] - iT[15] * a[9][1] - iT[22] * a[8][4] + iT[3] * a[4][4] + iT[8] * a[7][1] - iT[16] * a[1][1] - iT[21] * a[8][1] - iT[4] * a[3][3] - iT[7] * a[2][3] + iT[17] * a[1][3] + iT[20] * a[0][3] + iT[5] * a[7][2] - iT[6] * a[2][2] + iT[18] * a[9][3] + iT[19] * a[5][3] + t[0] + add) >> shift);
    dst[21] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[1][2] + iT[11] * a[8][2] - iT[13] * a[7][2] - iT[24] * a[4][3] + iT[1] * a[1][5] + iT[10] * a[8][5] - iT[14] * a[7][5] - iT[23] * a[4][0] + iT[2] * a[5][2] + iT[9] * a[9][2] + iT[15] * a[7][3] - iT[22] * a[2][3] + iT[3] * a[5][5] + iT[8] * a[9][5] + iT[16] * a[7][0] - iT[21] * a[2][0] + iT[4] * a[8][1] + iT[7] * a[9][4] + iT[17] * a[6][4] + iT[20] * a[3][1] + iT[5] * a[8][4] + iT[6] * a[9][1] + iT[18] * a[6][1] + iT[19] * a[3][4] + t[1] + add) >> shift);
    dst[23] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[8][4] + iT[11] * a[9][1] + iT[13] * a[6][1] + iT[24] * a[3][4] - iT[1] * a[8][2] - iT[10] * a[1][2] + iT[14] * a[4][3] + iT[23] * a[7][2] - iT[2] * a[0][1] - iT[9] * a[1][1] + iT[15] * a[3][1] + iT[22] * a[2][1] + iT[3] * a[5][0] + iT[8] * a[9][0] + iT[16] * a[7][5] - iT[21] * a[2][5] - iT[4] * a[9][5] - iT[7] * a[8][0] - iT[17] * a[3][0] - iT[20] * a[6][5] + iT[5] * a[5][2] - iT[6] * a[0][3] - iT[18] * a[4][2] - iT[19] * a[6][2] - t[1] + add) >> shift);
    dst[24] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[2][3] + iT[11] * a[7][3] + iT[13] * a[5][2] + iT[24] * a[9][2] + iT[1] * a[4][1] + iT[10] * a[7][4] - iT[14] * a[1][4] - iT[23] * a[8][4] - iT[2] * a[4][5] - iT[9] * a[7][0] + iT[15] * a[1][0] + iT[22] * a[8][0] + iT[3] * a[4][3] + iT[8] * a[6][3] + iT[16] * a[0][2] - iT[21] * a[5][3] - iT[4] * a[2][5] - iT[7] * a[3][5] + iT[17] * a[0][5] + iT[20] * a[1][5] + iT[5] * a[2][1] + iT[6] * a[3][1] - iT[18] * a[0][1] - iT[19] * a[1][1] - t[0] + add) >> shift);
    dst[25] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[4][5] - iT[11] * a[6][5] - iT[13] * a[0][0] + iT[24] * a[5][5] - iT[1] * a[3][1] - iT[10] * a[2][1] + iT[14] * a[1][1] + iT[23] * a[0][1] + iT[2] * a[7][2] + iT[9] * a[4][3] - iT[15] * a[8][2] - iT[22] * a[1][2] + iT[3] * a[6][2] + iT[8] * a[3][3] + iT[16] * a[9][2] + iT[21] * a[8][3] + iT[4] * a[2][4] - iT[7] * a[7][4] - iT[17] * a[5][1] - iT[20] * a[9][1] - iT[5] * a[4][0] - iT[6] * a[6][0] - iT[18] * a[0][5] + iT[19] * a[5][0] - t[0] + add) >> shift);
    dst[26] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[8][0] + iT[11] * a[1][0] - iT[13] * a[4][5] - iT[24] * a[7][0] + iT[1] * a[5][4] + iT[10] * a[9][4] + iT[14] * a[7][1] - iT[23] * a[2][1] - iT[2] * a[1][2] - iT[9] * a[0][2] + iT[15] * a[2][2] + iT[22] * a[3][2] - iT[3] * a[9][2] - iT[8] * a[8][3] - iT[16] * a[3][3] - iT[21] * a[6][2] + iT[4] * a[0][4] - iT[7] * a[5][1] + iT[17] * a[6][1] + iT[20] * a[4][1] + iT[5] * a[8][5] + iT[6] * a[1][5] - iT[18] * a[4][0] - iT[19] * a[7][5] - t[1] + add) >> shift);
    dst[28] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[5][1] - iT[11] * a[9][1] - iT[13] * a[7][4] + iT[24] * a[2][4] + iT[1] * a[8][2] + iT[10] * a[9][3] + iT[14] * a[6][3] + iT[23] * a[3][2] - iT[2] * a[9][4] - iT[9] * a[8][1] - iT[15] * a[3][1] - iT[22] * a[6][4] + iT[3] * a[9][0] + iT[8] * a[5][0] - iT[16] * a[2][5] + iT[21] * a[7][5] - iT[4] * a[5][5] + iT[7] * a[0][0] + iT[17] * a[4][5] + iT[20] * a[6][5] + iT[5] * a[1][3] + iT[6] * a[0][3] - iT[18] * a[2][3] - iT[19] * a[3][3] + t[1] + add) >> shift);
    dst[29] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[6][4] + iT[11] * a[3][1] + iT[13] * a[9][4] + iT[24] * a[8][1] - iT[1] * a[7][3] - iT[10] * a[4][2] + iT[14] * a[8][3] + iT[23] * a[1][3] - iT[2] * a[3][5] - iT[9] * a[2][5] + iT[15] * a[1][5] + iT[22] * a[0][5] + iT[3] * a[2][4] + iT[8] * a[3][4] - iT[16] * a[0][4] - iT[21] * a[1][4] + iT[4] * a[4][3] + iT[7] * a[7][2] - iT[17] * a[1][2] - iT[20] * a[8][2] - iT[5] * a[3][0] - iT[6] * a[6][5] - iT[18] * a[8][0] - iT[19] * a[9][5] + t[0] + add) >> shift);
    dst[30] = Clip3(outputMinimum, outputMaximum, (int)( - iT[0] * a[7][2] + iT[11] * a[2][2] - iT[13] * a[9][3] - iT[24] * a[5][3] - iT[1] * a[6][0] - iT[10] * a[4][0] + iT[14] * a[5][0] - iT[23] * a[0][5] - iT[2] * a[4][2] - iT[9] * a[6][2] - iT[15] * a[0][3] + iT[22] * a[5][2] + iT[3] * a[2][0] - iT[8] * a[7][0] - iT[16] * a[5][5] - iT[21] * a[9][5] + iT[4] * a[7][1] - iT[7] * a[2][1] + iT[17] * a[9][4] + iT[20] * a[5][4] + iT[5] * a[6][1] + iT[6] * a[4][1] - iT[18] * a[5][1] + iT[19] * a[0][4] + t[0] + add) >> shift);
    dst[31] = Clip3(outputMinimum, outputMaximum, (int)(   iT[0] * a[8][5] + iT[11] * a[1][5] - iT[13] * a[4][0] - iT[24] * a[7][5] - iT[1] * a[1][0] - iT[10] * a[8][0] + iT[14] * a[7][0] + iT[23] * a[4][5] - iT[2] * a[8][4] - iT[9] * a[1][4] + iT[15] * a[4][1] + iT[22] * a[7][4] + iT[3] * a[1][1] + iT[8] * a[8][1] - iT[16] * a[7][1] - iT[21] * a[4][4] + iT[4] * a[8][3] + iT[7] * a[1][3] - iT[17] * a[4][2] - iT[20] * a[7][3] - iT[5] * a[1][2] - iT[6] * a[8][2] + iT[18] * a[7][2] + iT[19] * a[4][3] + t[1] + add) >> shift);

    dst[ 2] = Clip3(outputMinimum, outputMaximum, (int)(   iT[ 4] * b[0] + iT[ 9] * b[1] + iT[14] * b[2] + iT[19] * b[3] + iT[24] * b[4] + iT[29] * b[5] + add) >> shift);
    dst[ 7] = Clip3(outputMinimum, outputMaximum, (int)( - iT[14] * b[0] - iT[29] * b[1] - iT[19] * b[2] - iT[ 4] * b[3] + iT[ 9] * b[4] + iT[24] * b[5] + add) >> shift);
    dst[12] = Clip3(outputMinimum, outputMaximum, (int)(   iT[24] * b[0] + iT[14] * b[1] - iT[ 9] * b[2] - iT[29] * b[3] - iT[ 4] * b[4] + iT[19] * b[5] + add) >> shift);
    dst[17] = Clip3(outputMinimum, outputMaximum, (int)( - iT[29] * b[0] + iT[ 4] * b[1] + iT[24] * b[2] - iT[ 9] * b[3] - iT[19] * b[4] + iT[14] * b[5] + add) >> shift);
    dst[22] = Clip3(outputMinimum, outputMaximum, (int)(   iT[19] * b[0] - iT[24] * b[1] + iT[ 4] * b[2] + iT[14] * b[3] - iT[29] * b[4] + iT[ 9] * b[5] + add) >> shift);
    dst[27] = Clip3(outputMinimum, outputMaximum, (int)( - iT[ 9] * b[0] + iT[19] * b[1] - iT[29] * b[2] + iT[24] * b[3] - iT[14] * b[4] + iT[ 4] * b[5] + add) >> shift);

    dst[ 6] = Clip3(outputMinimum, outputMaximum, (int)(   iT[12] * c[0] + iT[25] * c[1] + add) >> shift);
    dst[19] = Clip3(outputMinimum, outputMaximum, (int)( - iT[25] * c[0] + iT[12] * c[1] + add) >> shift);

    src++;
    dst += 32;
  }

  if (iSkipLine)
  {
    memset(dst, 0, (iSkipLine * 32) * sizeof(TCoeff));
  }
#endif
}

#if ENABLE_SIMD_TCOEFF_OPS

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP 1

}

#include "Unit.h"
#include "Buffer.h"

namespace vvdec
{

void cpyResiCore( const TCoeff* src, Pel* dst, ptrdiff_t stride, unsigned width, unsigned height )
{
#define CPYRESI_OP( ADDR ) dst[ADDR] = Pel( src[ADDR] );
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

void fastInvCore( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned trSize, unsigned lines, unsigned reducedLines, unsigned rows )
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
  cpyResi4     = cpyResiCore;
  cpyResi8     = cpyResiCore;
  roundClip4   = clipCore;
  roundClip8   = clipCore;
  fastInvCore4 = fastInvCore;
  fastInvCore8 = fastInvCore;
}

TCoeffOps g_tCoeffOps;

#endif

}