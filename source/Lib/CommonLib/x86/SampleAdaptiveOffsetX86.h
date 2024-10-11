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

/** \file     SampleAdaptiveOffsetX86.h
    \brief    SAO filter class
*/
#include "CommonDefX86.h"
#include "../SampleAdaptiveOffset.h"

namespace vvdec
{

#ifdef TARGET_SIMD_X86

#  define SAO_NUM_OFFSETS 4                             /* number of SAO offset values */
#  define SAO_EO_NUM_CATEGORIES ( SAO_NUM_OFFSETS + 1 ) /* number of different eo categories */

#  if USE_AVX2 && !defined( _mm256_set_m128i )
#    define VVCLIB_OWN_mm256_set_m128i
#    define _mm256_set_m128i( v0, v1 ) _mm256_inserti128_si256( _mm256_castsi128_si256( v1 ), ( v0 ), 1 )
#  endif

static bool isProcessDisabled( int xPos, int yPos, int numVerVirBndry, int numHorVirBndry, int verVirBndryPos[], int horVirBndryPos[] )
{
  for( int i = 0; i < numVerVirBndry; i++ )
  {
    if( ( xPos == verVirBndryPos[i] ) || ( xPos == verVirBndryPos[i] - 1 ) )
    {
      return true;
    }
  }
  for( int i = 0; i < numHorVirBndry; i++ )
  {
    if( ( yPos == horVirBndryPos[i] ) || ( yPos == horVirBndryPos[i] - 1 ) )
    {
      return true;
    }
  }

  return false;
}

static bool isHorProcessDisabled( int yPos, int numHorVirBndry, int horVirBndryPos[] )
{
  for( int i = 0; i < numHorVirBndry; i++ )
  {
    if( ( yPos == horVirBndryPos[i] ) || ( yPos == horVirBndryPos[i] - 1 ) )
    {
      return true;
    }
  }
  return false;
}
static bool isVerProcessDisabled( int xPos, int numVerVirBndry, int verVirBndryPos[] )
{
  for( int i = 0; i < numVerVirBndry; i++ )
  {
    if( ( xPos == verVirBndryPos[i] ) || ( xPos == verVirBndryPos[i] - 1 ) )
    {
      return true;
    }
  }
  return false;
}

template<X86_VEXT vext>
static void offsetBlock_SIMD_SAO_TYPE_BO( const int  channelBitDepth,
                                          int*       offset,
                                          int        startIdx,
                                          const Pel* srcBlk,
                                          Pel*       resBlk,
                                          ptrdiff_t  srcStride,
                                          ptrdiff_t  resStride,
                                          int        width,
                                          int        height )
{
  const Pel* srcLine = srcBlk;
  Pel*       resLine = resBlk;

  const int shiftBits        = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
  int8_t    p_eo_offsets[16] = { 0 };
  for( int i = 0; i < 4; i++ )
  {
    p_eo_offsets[i] = offset[( startIdx + i ) % MAX_NUM_SAO_CLASSES];
  }
  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; )
    {
#  ifdef USE_AVX2
      // AVX2
      if( width - x >= 16 && vext >= AVX2 )
      {
        __m256i vbaseoffset = _mm256_set1_epi16( startIdx - MAX_NUM_SAO_CLASSES );
        __m256i vminus      = _mm256_set1_epi8( -1 );
        __m256i vzero       = _mm256_set1_epi8( 0 );

        __m256i vfour      = _mm256_set1_epi16( 4 );
        __m256i vibdimax   = _mm256_set1_epi16( ( 1 << channelBitDepth ) - 1 );
        __m256i voffsettbl = _mm256_broadcastsi128_si256( _mm_loadu_si128( (__m128i*)p_eo_offsets ) );

        __m256i vsrc  = _mm256_loadu_si256( (__m256i*)&srcLine[x] );
        __m256i bands = _mm256_srai_epi16( vsrc, shiftBits );
        bands         = _mm256_sub_epi16 ( bands, vbaseoffset );
        bands         = _mm256_and_si256 ( bands, _mm256_set1_epi16( MAX_NUM_SAO_CLASSES - 1 ) ); // modulo 32 = modulo NUM_SAO_BO_CLASSES_LOG2
        __m256i mask1 = _mm256_cmpgt_epi16( bands, vminus );
        __m256i mask2 = _mm256_cmpgt_epi16( vfour, bands );

        __m256i veoffsets = _mm256_shuffle_epi8( voffsettbl, bands );
        veoffsets         = _mm256_slli_epi16( veoffsets, 8 );
        veoffsets         = _mm256_srai_epi16( veoffsets, 8 );

        veoffsets = _mm256_and_si256( veoffsets, mask1 );
        veoffsets = _mm256_and_si256( veoffsets, mask2 );

        vsrc = _mm256_add_epi16( vsrc, veoffsets );
        vsrc = _mm256_min_epi16( _mm256_max_epi16( vsrc, vzero ), vibdimax );
        _mm256_storeu_si256( (__m256i*)&resLine[x], vsrc );

        x += 16;
      }
      else
#  endif
      {
        __m128i vbaseoffset = _mm_set1_epi16( startIdx - MAX_NUM_SAO_CLASSES );
        __m128i vminus      = _mm_set1_epi8( -1 );
        __m128i vzero       = _mm_set1_epi8( 0 );

        __m128i vfour      = _mm_set1_epi16( 4 );
        __m128i vibdimax   = _mm_set1_epi16( ( 1 << channelBitDepth ) - 1 );
        __m128i voffsettbl = _mm_loadu_si128( (__m128i*)p_eo_offsets );

        __m128i vsrc  = _mm_loadu_si128( (__m128i*)&srcLine[x] );
        __m128i bands = _mm_srai_epi16( vsrc, shiftBits );
        bands         = _mm_sub_epi16 ( bands, vbaseoffset );
        bands         = _mm_and_si128 ( bands, _mm_set1_epi16( MAX_NUM_SAO_CLASSES - 1 ) ); // modulo 32 = modulo NUM_SAO_BO_CLASSES_LOG2
        __m128i mask1 = _mm_cmpgt_epi16( bands, vminus );
        __m128i mask2 = _mm_cmplt_epi16( bands, vfour );

        __m128i veoffsets = _mm_shuffle_epi8( voffsettbl, bands );
        veoffsets         = _mm_slli_epi16( veoffsets, 8 );
        veoffsets         = _mm_srai_epi16( veoffsets, 8 );

        veoffsets = _mm_and_si128( veoffsets, mask1 );
        veoffsets = _mm_and_si128( veoffsets, mask2 );

        vsrc = _mm_add_epi16( vsrc, veoffsets );
        vsrc = _mm_min_epi16( _mm_max_epi16( vsrc, vzero ), vibdimax );
        _mm_store_si128( (__m128i*)&resLine[x], vsrc );

        x += 8;
      }
    }
    srcLine += srcStride;
    resLine += resStride;
  }
}

template<X86_VEXT vext>
static void offsetBlock_SIMD_SAO_TYPE_EO_0( const int            channelBitDepth,
                                            const ClpRng&        clpRng,
                                            int*                 offset,
                                            const Pel*           srcBlk,
                                            Pel*                 resBlk,
                                            ptrdiff_t            srcStride,
                                            ptrdiff_t            resStride,
                                            int                  width,
                                            int                  height,
                                            bool                 isLeftAvail,
                                            bool                 isRightAvail,
                                            bool                 isAboveAvail,
                                            bool                 isBelowAvail,
                                            bool                 isAboveLeftAvail,
                                            bool                 isAboveRightAvail,
                                            bool                 isBelowLeftAvail,
                                            bool                 isBelowRightAvail,
                                            std::vector<int8_t>* m_signLineBuf1,
                                            std::vector<int8_t>* m_signLineBuf2,
                                            bool                 isCtuCrossedByVirtualBoundaries,
                                            int                  horVirBndryPos[],
                                            int                  verVirBndryPos[],
                                            int                  numHorVirBndry,
                                            int                  numVerVirBndry,
                                            uint16_t             bndmask[MAX_CU_SIZE] )
{
  const Pel* srcLine = srcBlk;
  Pel*       resLine = resBlk;

  int    x, y, startX, endX, edgeType;
  int8_t signLeft, signRight;

  if( isLeftAvail && isRightAvail )
  {
    int8_t p_eo_offsets[16] = { 0 };
    for( int i = 0; i < SAO_EO_NUM_CATEGORIES; i++ )
    {
      p_eo_offsets[i] = offset[i];
    }
#if defined( USE_AVX2 )
    // AVX2
    if( ( width & 15 ) == 0 && vext >= AVX2 )
    {

      __m256i vsrca, vsrcal, vsrcar, virBmask;

      __m256i vbaseoffset = _mm256_set1_epi16( 2 );
      __m256i vplusone    = _mm256_set1_epi16( 1 );
      __m256i vzero       = _mm256_set1_epi8( 0 );
      __m256i vibdimax    = _mm256_set1_epi16( ( 1 << channelBitDepth ) - 1 );
      __m256i voffsettbl  = _mm256_broadcastsi128_si256( _mm_loadu_si128( (__m128i*)p_eo_offsets ) );

      if( isCtuCrossedByVirtualBoundaries )
      {
        for( y = 0; y < height; y++ )
        {
          for( x = 0; x < width; x += 16 )
          {
            vsrca    = _mm256_loadu_si256( (__m256i*)&srcLine[x] );
            vsrcal   = _mm256_loadu_si256( (__m256i*)&srcLine[x - 1] );
            vsrcar   = _mm256_loadu_si256( (__m256i*)&srcLine[x + 1] );
            virBmask = _mm256_loadu_si256( (__m256i*)&bndmask[x] );

            vsrcal            = _mm256_subs_epi16( vsrca, vsrcal );
            vsrcar            = _mm256_subs_epi16( vsrca, vsrcar );
            __m256i vsignl    = _mm256_sign_epi16( vplusone, vsrcal );
            __m256i vsignr    = _mm256_sign_epi16( vplusone, vsrcar );
            __m256i vsign     = _mm256_adds_epi16( _mm256_adds_epi16( vsignl, vsignr ), vbaseoffset );
            __m256i veoffsets = _mm256_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm256_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm256_srai_epi16( veoffsets, 8 );

            vsrcal = _mm256_add_epi16( vsrca, veoffsets );
            vsrcal = _mm256_min_epi16( _mm256_max_epi16( vsrcal, vzero ), vibdimax );

            vsrcar = _mm256_blendv_epi8( vsrcal, vsrca, virBmask );

            _mm256_storeu_si256( (__m256i*)&resLine[x], vsrcar );
          }
          srcLine += srcStride;
          resLine += resStride;
        }
      }
      else
      {
        for( y = 0; y < height; y++ )
        {
          for( x = 0; x < width; x += 16 )
          {
            vsrca             = _mm256_loadu_si256( (__m256i*)&srcLine[x] );
            vsrcal            = _mm256_loadu_si256( (__m256i*)&srcLine[x - 1] );
            vsrcar            = _mm256_loadu_si256( (__m256i*)&srcLine[x + 1] );
            vsrcal            = _mm256_subs_epi16( vsrca, vsrcal );
            vsrcar            = _mm256_subs_epi16( vsrca, vsrcar );
            __m256i vsignl    = _mm256_sign_epi16( vplusone, vsrcal );
            __m256i vsignr    = _mm256_sign_epi16( vplusone, vsrcar );
            __m256i vsign     = _mm256_adds_epi16( _mm256_adds_epi16( vsignl, vsignr ), vbaseoffset );
            __m256i veoffsets = _mm256_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm256_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm256_srai_epi16( veoffsets, 8 );

            vsrca = _mm256_add_epi16( vsrca, veoffsets );
            vsrca = _mm256_min_epi16( _mm256_max_epi16( vsrca, vzero ), vibdimax );

            _mm256_storeu_si256( (__m256i*)&resLine[x], vsrca );
          }
          srcLine += srcStride;
          resLine += resStride;
        }
      }
    }
    else
#  endif
    {
      __m128i vsrca, vsrcal, vsrcar, virBmask;
      __m128i vbaseoffset = _mm_set1_epi16( 2 );
      __m128i vplusone    = _mm_set1_epi16( 1 );
      __m128i vzero       = _mm_set1_epi8( 0 );
      __m128i vibdimax    = _mm_set1_epi16( ( 1 << channelBitDepth ) - 1 );
      __m128i voffsettbl  = _mm_loadu_si128( (__m128i*)p_eo_offsets );
      if( isCtuCrossedByVirtualBoundaries )
      {
        for( y = 0; y < height; y++ )
        {
          for( x = 0; x < width; x += 8 )
          {
            vsrca             = _mm_loadu_si128( (__m128i*)&srcLine[x] );
            vsrcal            = _mm_loadu_si128( (__m128i*)&srcLine[x - 1] );
            vsrcar            = _mm_loadu_si128( (__m128i*)&srcLine[x + 1] );
            virBmask          = _mm_loadu_si128( (__m128i*)&bndmask[x] );
            vsrcal            = _mm_subs_epi16( vsrca, vsrcal );
            vsrcar            = _mm_subs_epi16( vsrca, vsrcar );
            __m128i vsignl    = _mm_sign_epi16( vplusone, vsrcal );
            __m128i vsignr    = _mm_sign_epi16( vplusone, vsrcar );
            __m128i vsign     = _mm_adds_epi16( _mm_adds_epi16( vsignl, vsignr ), vbaseoffset );
            __m128i veoffsets = _mm_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm_srai_epi16( veoffsets, 8 );

            vsrcal = _mm_add_epi16( vsrca, veoffsets );
            vsrcal = _mm_min_epi16( _mm_max_epi16( vsrcal, vzero ), vibdimax );

            vsrcar = _mm_blendv_epi8( vsrcal, vsrca, virBmask );

            _mm_store_si128( (__m128i*)&resLine[x], vsrcar );
          }
          srcLine += srcStride;
          resLine += resStride;
        }
      }
      else
      {
        for( y = 0; y < height; y++ )
        {
          for( x = 0; x < width; x += 8 )
          {
            vsrca             = _mm_loadu_si128( (__m128i*)&srcLine[x] );
            vsrcal            = _mm_loadu_si128( (__m128i*)&srcLine[x - 1] );
            vsrcar            = _mm_loadu_si128( (__m128i*)&srcLine[x + 1] );
            vsrcal            = _mm_subs_epi16( vsrca, vsrcal );
            vsrcar            = _mm_subs_epi16( vsrca, vsrcar );
            __m128i vsignl    = _mm_sign_epi16( vplusone, vsrcal );
            __m128i vsignr    = _mm_sign_epi16( vplusone, vsrcar );
            __m128i vsign     = _mm_adds_epi16( _mm_adds_epi16( vsignl, vsignr ), vbaseoffset );
            __m128i veoffsets = _mm_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm_srai_epi16( veoffsets, 8 );

            vsrca = _mm_add_epi16( vsrca, veoffsets );
            vsrca = _mm_min_epi16( _mm_max_epi16( vsrca, vzero ), vibdimax );
            _mm_store_si128( (__m128i*)&resLine[x], vsrca );
          }
          srcLine += srcStride;
          resLine += resStride;
        }
      }
    }
  }
  else
  {
    offset += 2;
    startX = isLeftAvail ? 0 : 1;
    endX   = isRightAvail ? width : ( width - 1 );
    for( y = 0; y < height; y++ )
    {
      signLeft = (int8_t)sgn( srcLine[startX] - srcLine[startX - 1] );
      for( x = startX; x < endX; x++ )
      {
        signRight = (int8_t)sgn( srcLine[x] - srcLine[x + 1] );
        if( isCtuCrossedByVirtualBoundaries && isVerProcessDisabled( x, numVerVirBndry, verVirBndryPos ) )
        {
          signLeft = -signRight;
          continue;
        }
        edgeType = signRight + signLeft;
        signLeft = -signRight;

        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
      }

      srcLine += srcStride;
      resLine += resStride;
    }
  }
}

template<X86_VEXT vext>
static void offsetBlock_SIMD_SAO_TYPE_EO_90( const int            channelBitDepth,
                                             const ClpRng&        clpRng,
                                             int*                 offset,
                                             const Pel*           srcBlk,
                                             Pel*                 resBlk,
                                             ptrdiff_t            srcStride,
                                             ptrdiff_t            resStride,
                                             int                  width,
                                             int                  height,
                                             bool                 isLeftAvail,
                                             bool                 isRightAvail,
                                             bool                 isAboveAvail,
                                             bool                 isBelowAvail,
                                             bool                 isAboveLeftAvail,
                                             bool                 isAboveRightAvail,
                                             bool                 isBelowLeftAvail,
                                             bool                 isBelowRightAvail,
                                             std::vector<int8_t>* m_signLineBuf1,
                                             std::vector<int8_t>* m_signLineBuf2,
                                             bool                 isCtuCrossedByVirtualBoundaries,
                                             int                  horVirBndryPos[],
                                             int                  verVirBndryPos[],
                                             int                  numHorVirBndry,
                                             int                  numVerVirBndry,
                                             uint16_t             bndmask[MAX_CU_SIZE] )
{
  const Pel* srcLine = srcBlk;
  Pel*       resLine = resBlk;

  int x, y, startY, endY;

  int8_t p_eo_offsets[16] = { 0 };
  for( int i = 0; i < SAO_EO_NUM_CATEGORIES; i++ )
  {
    p_eo_offsets[i] = offset[i];
  }
  const Pel* srcLineAbove = srcLine - srcStride;
  const Pel* srcLineBelow = srcLine + srcStride;
  startY                  = 0;
  if( !isAboveAvail )
  {
    startY       = 1;
    srcLineAbove = srcLine;
    srcLine += srcStride;
    resLine += resStride;
    srcLineBelow = srcLine + srcStride;
  }
  endY = height;
  if( !isBelowAvail )
  {
    endY = height - 1;
  }
#  if defined( USE_AVX2 )
  // AVX2
  if( ( width & 15 ) == 0 && vext >= AVX2 )
  {
    __m256i vsrca, vsrcat, vsrcab;

    __m256i    vbaseoffset  = _mm256_set1_epi16( 2 );
    __m256i    vplusone     = _mm256_set1_epi16( 1 );
    __m256i    vzero        = _mm256_set1_epi8( 0 );
    __m256i    vibdimax     = _mm256_set1_epi16( ( 1 << channelBitDepth ) - 1 );
    __m256i    voffsettbl   = _mm256_broadcastsi128_si256( _mm_loadu_si128( (__m128i*)p_eo_offsets ) );
    const Pel* srcLineBelow = srcLine + srcStride;

    if( isCtuCrossedByVirtualBoundaries )
    {
      for( y = startY; y < endY; y++ )
      {
        if( !isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
        {
          for( x = 0; x < width; x += 16 )
          {
            vsrca             = _mm256_loadu_si256( (__m256i*)&srcLine[x] );
            vsrcat            = _mm256_loadu_si256( (__m256i*)&srcLineAbove[x] );
            vsrcab            = _mm256_loadu_si256( (__m256i*)&srcLineBelow[x] );
            vsrcat            = _mm256_subs_epi16( vsrca, vsrcat );
            vsrcab            = _mm256_subs_epi16( vsrca, vsrcab );
            __m256i vsignt    = _mm256_sign_epi16( vplusone, vsrcat );
            __m256i vsignb    = _mm256_sign_epi16( vplusone, vsrcab );
            __m256i vsign     = _mm256_adds_epi16( _mm256_adds_epi16( vsignt, vsignb ), vbaseoffset );
            __m256i veoffsets = _mm256_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm256_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm256_srai_epi16( veoffsets, 8 );

            vsrca = _mm256_add_epi16( vsrca, veoffsets );
            vsrca = _mm256_min_epi16( _mm256_max_epi16( vsrca, vzero ), vibdimax );

            _mm256_storeu_si256( (__m256i*)&resLine[x], vsrca );
          }
        }
        srcLine += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
    else
    {
      for( y = startY; y < endY; y++ )
      {
        for( x = 0; x < width; x += 16 )
        {
          vsrca             = _mm256_loadu_si256( (__m256i*)&srcLine[x] );
          vsrcat            = _mm256_loadu_si256( (__m256i*)&srcLineAbove[x] );
          vsrcab            = _mm256_loadu_si256( (__m256i*)&srcLineBelow[x] );
          vsrcat            = _mm256_subs_epi16( vsrca, vsrcat );
          vsrcab            = _mm256_subs_epi16( vsrca, vsrcab );
          __m256i vsignt    = _mm256_sign_epi16( vplusone, vsrcat );
          __m256i vsignb    = _mm256_sign_epi16( vplusone, vsrcab );
          __m256i vsign     = _mm256_adds_epi16( _mm256_adds_epi16( vsignt, vsignb ), vbaseoffset );
          __m256i veoffsets = _mm256_shuffle_epi8( voffsettbl, vsign );
          veoffsets         = _mm256_slli_epi16( veoffsets, 8 );
          veoffsets         = _mm256_srai_epi16( veoffsets, 8 );

          vsrca = _mm256_add_epi16( vsrca, veoffsets );
          vsrca = _mm256_min_epi16( _mm256_max_epi16( vsrca, vzero ), vibdimax );

          _mm256_storeu_si256( (__m256i*)&resLine[x], vsrca );
        }
        srcLine += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
  }
  else
#  endif
  {
    __m128i vsrca, vsrcat, vsrcab;
    __m128i vbaseoffset = _mm_set1_epi16( 2 );
    __m128i vplusone    = _mm_set1_epi16( 1 );
    __m128i vzero       = _mm_set1_epi8( 0 );
    __m128i vibdimax    = _mm_set1_epi16( ( 1 << channelBitDepth ) - 1 );
    __m128i voffsettbl  = _mm_loadu_si128( (__m128i*)p_eo_offsets );

    if( isCtuCrossedByVirtualBoundaries )
    {
      for( y = startY; y < endY; y++ )
      {
        if( !isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
        {
          for( x = 0; x < width; x += 8 )
          {
            vsrca             = _mm_loadu_si128( (__m128i*)&srcLine[x] );
            vsrcat            = _mm_loadu_si128( (__m128i*)&srcLineAbove[x] );
            vsrcab            = _mm_loadu_si128( (__m128i*)&srcLineBelow[x] );
            vsrcat            = _mm_subs_epi16( vsrca, vsrcat );
            vsrcab            = _mm_subs_epi16( vsrca, vsrcab );
            __m128i vsignt    = _mm_sign_epi16( vplusone, vsrcat );
            __m128i vsignb    = _mm_sign_epi16( vplusone, vsrcab );
            __m128i vsign     = _mm_adds_epi16( _mm_adds_epi16( vsignt, vsignb ), vbaseoffset );
            __m128i veoffsets = _mm_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm_srai_epi16( veoffsets, 8 );

            vsrca = _mm_add_epi16( vsrca, veoffsets );
            vsrca = _mm_min_epi16( _mm_max_epi16( vsrca, vzero ), vibdimax );

            _mm_store_si128( (__m128i*)&resLine[x], vsrca );
          }
        }
        srcLine += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
    else
    {
      for( y = startY; y < endY; y++ )
      {
        for( x = 0; x < width; x += 8 )
        {
          vsrca             = _mm_loadu_si128( (__m128i*)&srcLine[x] );
          vsrcat            = _mm_loadu_si128( (__m128i*)&srcLineAbove[x] );
          vsrcab            = _mm_loadu_si128( (__m128i*)&srcLineBelow[x] );
          vsrcat            = _mm_subs_epi16( vsrca, vsrcat );
          vsrcab            = _mm_subs_epi16( vsrca, vsrcab );
          __m128i vsignt    = _mm_sign_epi16( vplusone, vsrcat );
          __m128i vsignb    = _mm_sign_epi16( vplusone, vsrcab );
          __m128i vsign     = _mm_adds_epi16( _mm_adds_epi16( vsignt, vsignb ), vbaseoffset );
          __m128i veoffsets = _mm_shuffle_epi8( voffsettbl, vsign );
          veoffsets         = _mm_slli_epi16( veoffsets, 8 );
          veoffsets         = _mm_srai_epi16( veoffsets, 8 );

          vsrca = _mm_add_epi16( vsrca, veoffsets );
          vsrca = _mm_min_epi16( _mm_max_epi16( vsrca, vzero ), vibdimax );
          _mm_store_si128( (__m128i*)&resLine[x], vsrca );
        }
        srcLine += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
  }
}

template<X86_VEXT vext>
static void offsetBlock_SIMD_SAO_TYPE_EO_135( const int            channelBitDepth,
                                              const ClpRng&        clpRng,
                                              int*                 offset,
                                              const Pel*           srcBlk,
                                              Pel*                 resBlk,
                                              ptrdiff_t            srcStride,
                                              ptrdiff_t            resStride,
                                              int                  width,
                                              int                  height,
                                              bool                 isLeftAvail,
                                              bool                 isRightAvail,
                                              bool                 isAboveAvail,
                                              bool                 isBelowAvail,
                                              bool                 isAboveLeftAvail,
                                              bool                 isAboveRightAvail,
                                              bool                 isBelowLeftAvail,
                                              bool                 isBelowRightAvail,
                                              std::vector<int8_t>* m_signLineBuf1,
                                              std::vector<int8_t>* m_signLineBuf2,
                                              bool                 isCtuCrossedByVirtualBoundaries,
                                              int                  horVirBndryPos[],
                                              int                  verVirBndryPos[],
                                              int                  numHorVirBndry,
                                              int                  numVerVirBndry,
                                              uint16_t             bndmask[MAX_CU_SIZE] )
{
  const Pel* srcLine = srcBlk;
  Pel*       resLine = resBlk;

  int    x, y, startX, startY, endX, endY, edgeType;
  int    firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signDown;

  if( isLeftAvail && isRightAvail && isAboveLeftAvail && isBelowRightAvail )
  {
    int8_t p_eo_offsets[16] = { 0 };
    for( int i = 0; i < SAO_EO_NUM_CATEGORIES; i++ )
    {
      p_eo_offsets[i] = offset[i];
    }
    const Pel* srcLineAbove = srcLine - srcStride;
    const Pel* srcLineBelow = srcLine + srcStride;
    startY                  = 0;
    if( !isAboveAvail )
    {
      startY       = 1;
      srcLineAbove = srcLine;
      srcLine += srcStride;
      resLine += resStride;
      srcLineBelow = srcLine + srcStride;
    }
    endY = height;
    if( !isBelowAvail )
    {
      endY = height - 1;
    }
#  if defined( USE_AVX2 )
    // AVX2
    if( ( width & 15 ) == 0 && vext >= AVX2 )
    {
      __m256i vsrca, vsrcat, vsrcab, virBmask;

      __m256i    vbaseoffset  = _mm256_set1_epi16( 2 );
      __m256i    vplusone     = _mm256_set1_epi16( 1 );
      __m256i    vzero        = _mm256_set1_epi8( 0 );
      __m256i    vibdimax     = _mm256_set1_epi16( ( 1 << channelBitDepth ) - 1 );
      __m256i    voffsettbl   = _mm256_broadcastsi128_si256( _mm_loadu_si128( (__m128i*)p_eo_offsets ) );
      const Pel* srcLineBelow = srcLine + srcStride;

      for( y = startY; y < endY; y++ )
      {
        if( !isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
        {
          for( x = 0; x < width; x += 16 )
          {
            vsrca             = _mm256_loadu_si256( (__m256i*)&srcLine[x] );
            vsrcat            = _mm256_loadu_si256( (__m256i*)&srcLineAbove[x - 1] );
            vsrcab            = _mm256_loadu_si256( (__m256i*)&srcLineBelow[x + 1] );
            virBmask          = _mm256_loadu_si256( (__m256i*)&bndmask[x] );
            vsrcat            = _mm256_subs_epi16( vsrca, vsrcat );
            vsrcab            = _mm256_subs_epi16( vsrca, vsrcab );
            __m256i vsignt    = _mm256_sign_epi16( vplusone, vsrcat );
            __m256i vsignb    = _mm256_sign_epi16( vplusone, vsrcab );
            __m256i vsign     = _mm256_adds_epi16( _mm256_adds_epi16( vsignt, vsignb ), vbaseoffset );
            __m256i veoffsets = _mm256_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm256_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm256_srai_epi16( veoffsets, 8 );

            vsrcat = _mm256_add_epi16( vsrca, veoffsets );
            vsrcat = _mm256_min_epi16( _mm256_max_epi16( vsrcat, vzero ), vibdimax );

            vsrcab = _mm256_blendv_epi8( vsrcat, vsrca, virBmask );

            _mm256_storeu_si256( (__m256i*)&resLine[x], vsrcab );
          }
        }
        srcLine += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
    else
#  endif
    {
      __m128i vsrca, vsrcat, vsrcab, virBmask;
      __m128i vbaseoffset = _mm_set1_epi16( 2 );
      __m128i vplusone    = _mm_set1_epi16( 1 );
      __m128i vzero       = _mm_set1_epi8( 0 );
      __m128i vibdimax    = _mm_set1_epi16( ( 1 << channelBitDepth ) - 1 );
      __m128i voffsettbl  = _mm_loadu_si128( (__m128i*)p_eo_offsets );

      for( y = startY; y < endY; y++ )
      {
        if( !isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
        {
          for( x = 0; x < width; x += 8 )
          {
            vsrca             = _mm_loadu_si128( (__m128i*)&srcLine[x] );
            vsrcat            = _mm_loadu_si128( (__m128i*)&srcLineAbove[x - 1] );
            vsrcab            = _mm_loadu_si128( (__m128i*)&srcLineBelow[x + 1] );
            virBmask          = _mm_loadu_si128( (__m128i*)&bndmask[x] );
            vsrcat            = _mm_subs_epi16( vsrca, vsrcat );
            vsrcab            = _mm_subs_epi16( vsrca, vsrcab );
            __m128i vsignt    = _mm_sign_epi16( vplusone, vsrcat );
            __m128i vsignb    = _mm_sign_epi16( vplusone, vsrcab );
            __m128i vsign     = _mm_adds_epi16( _mm_adds_epi16( vsignt, vsignb ), vbaseoffset );
            __m128i veoffsets = _mm_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm_srai_epi16( veoffsets, 8 );

            vsrcat = _mm_add_epi16( vsrca, veoffsets );
            vsrcat = _mm_min_epi16( _mm_max_epi16( vsrcat, vzero ), vibdimax );

            vsrcab = _mm_blendv_epi8( vsrcat, vsrca, virBmask );

            _mm_store_si128( (__m128i*)&resLine[x], vsrcab );
          }
        }
        srcLine += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
  }
  else
  {
    offset += 2;
    int8_t *signUpLine, *signDownLine, *signTmpLine;

    signUpLine   = &m_signLineBuf1->front();
    signDownLine = &m_signLineBuf2->front();

    startX = isLeftAvail ? 0 : 1;
    endX   = isRightAvail ? width : ( width - 1 );

    // prepare 2nd line's upper sign
    const Pel* srcLineBelow = srcLine + srcStride;
    for( x = startX; x < endX + 1; x++ )
    {
      signUpLine[x] = (int8_t)sgn( srcLineBelow[x] - srcLine[x - 1] );
    }
    if( isCtuCrossedByVirtualBoundaries )
    {
      // 1st line
      const Pel* srcLineAbove = srcLine - srcStride;
      firstLineStartX         = isAboveLeftAvail ? 0 : 1;
      firstLineEndX           = isAboveAvail ? endX : 1;
      if( !isHorProcessDisabled( 0, numHorVirBndry, horVirBndryPos ) )
      {
        for( x = firstLineStartX; x < firstLineEndX; x++ )
        {
          if( isVerProcessDisabled( x, numVerVirBndry, verVirBndryPos ) )
          {
            continue;
          }
          edgeType = sgn( srcLine[x] - srcLineAbove[x - 1] ) - signUpLine[x + 1];

          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
        }
      }
      srcLine += srcStride;
      resLine += resStride;
      // middle lines
      for( y = 1; y < height - 1; y++ )
      {
        if( isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
        {
          srcLineBelow = srcLine + srcStride;
          for( x = startX; x < endX; x++ )
          {
            signDown            = (int8_t)sgn( srcLine[x] - srcLineBelow[x + 1] );
            signDownLine[x + 1] = -signDown;
          }
          signDownLine[startX] = (int8_t)sgn( srcLineBelow[startX] - srcLine[startX - 1] );
          signTmpLine          = signUpLine;
          signUpLine           = signDownLine;
          signDownLine         = signTmpLine;
          srcLine += srcStride;
          resLine += resStride;
        }
        else
        {
          srcLineBelow = srcLine + srcStride;
          for( x = startX; x < endX; x++ )
          {
            signDown = (int8_t)sgn( srcLine[x] - srcLineBelow[x + 1] );
            if( isVerProcessDisabled( x, numVerVirBndry, verVirBndryPos ) )
            {
              signDownLine[x + 1] = -signDown;
              continue;
            }
            edgeType   = signDown + signUpLine[x];
            resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );

            signDownLine[x + 1] = -signDown;
          }
          signDownLine[startX] = (int8_t)sgn( srcLineBelow[startX] - srcLine[startX - 1] );
          signTmpLine          = signUpLine;
          signUpLine           = signDownLine;
          signDownLine         = signTmpLine;
          srcLine += srcStride;
          resLine += resStride;
        }
      }
      // last line
      srcLineBelow   = srcLine + srcStride;
      lastLineStartX = isBelowAvail ? startX : ( width - 1 );
      lastLineEndX   = isBelowRightAvail ? width : ( width - 1 );
      if( !isHorProcessDisabled( height - 1, numHorVirBndry, horVirBndryPos ) )
      {
        for( x = lastLineStartX; x < lastLineEndX; x++ )
        {
          if( isVerProcessDisabled( x, numVerVirBndry, verVirBndryPos ) )
          {
            continue;
          }
          edgeType   = sgn( srcLine[x] - srcLineBelow[x + 1] ) + signUpLine[x];
          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
        }
      }
    }
    else
    {
      // 1st line
      const Pel* srcLineAbove = srcLine - srcStride;
      firstLineStartX         = isAboveLeftAvail ? 0 : 1;
      firstLineEndX           = isAboveAvail ? endX : 1;
      for( x = firstLineStartX; x < firstLineEndX; x++ )
      {
        edgeType   = sgn( srcLine[x] - srcLineAbove[x - 1] ) - signUpLine[x + 1];
        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
      }
      srcLine += srcStride;
      resLine += resStride;
      // middle lines
      for( y = 1; y < height - 1; y++ )
      {
        srcLineBelow = srcLine + srcStride;
        for( x = startX; x < endX; x++ )
        {
          signDown            = (int8_t)sgn( srcLine[x] - srcLineBelow[x + 1] );
          edgeType            = signDown + signUpLine[x];
          resLine[x]          = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
          signDownLine[x + 1] = -signDown;
        }
        signDownLine[startX] = (int8_t)sgn( srcLineBelow[startX] - srcLine[startX - 1] );
        signTmpLine          = signUpLine;
        signUpLine           = signDownLine;
        signDownLine         = signTmpLine;
        srcLine += srcStride;
        resLine += resStride;
      }
      // last line
      srcLineBelow   = srcLine + srcStride;
      lastLineStartX = isBelowAvail ? startX : ( width - 1 );
      lastLineEndX   = isBelowRightAvail ? width : ( width - 1 );
      for( x = lastLineStartX; x < lastLineEndX; x++ )
      {
        edgeType   = sgn( srcLine[x] - srcLineBelow[x + 1] ) + signUpLine[x];
        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
      }
    }
  }
}

template<X86_VEXT vext>
static void offsetBlock_SIMD_SAO_TYPE_EO_45( const int            channelBitDepth,
                                             const ClpRng&        clpRng,
                                             int*                 offset,
                                             const Pel*           srcBlk,
                                             Pel*                 resBlk,
                                             ptrdiff_t            srcStride,
                                             ptrdiff_t            resStride,
                                             int                  width,
                                             int                  height,
                                             bool                 isLeftAvail,
                                             bool                 isRightAvail,
                                             bool                 isAboveAvail,
                                             bool                 isBelowAvail,
                                             bool                 isAboveLeftAvail,
                                             bool                 isAboveRightAvail,
                                             bool                 isBelowLeftAvail,
                                             bool                 isBelowRightAvail,
                                             std::vector<int8_t>* m_signLineBuf1,
                                             std::vector<int8_t>* m_signLineBuf2,
                                             bool                 isCtuCrossedByVirtualBoundaries,
                                             int                  horVirBndryPos[],
                                             int                  verVirBndryPos[],
                                             int                  numHorVirBndry,
                                             int                  numVerVirBndry,
                                             uint16_t             bndmask[MAX_CU_SIZE] )
{
  const Pel* srcLine = srcBlk;
  Pel*       resLine = resBlk;

  int    x, y, startX, startY, endX, endY, edgeType;
  int    firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signDown;

  if( isLeftAvail && isRightAvail && isAboveLeftAvail && isBelowRightAvail )
  {
    int8_t p_eo_offsets[16] = { 0 };
    for( int i = 0; i < SAO_EO_NUM_CATEGORIES; i++ )
    {
      p_eo_offsets[i] = offset[i];
    }
    const Pel* srcLineAbove = srcLine - srcStride;
    const Pel* srcLineBelow = srcLine + srcStride;
    startY                  = 0;
    if( !isAboveAvail )
    {
      startY       = 1;
      srcLineAbove = srcLine;
      srcLine += srcStride;
      resLine += resStride;
      srcLineBelow = srcLine + srcStride;
    }
    endY = height;
    if( !isBelowAvail )
    {
      endY = height - 1;
    }
#  if defined( USE_AVX2 )
    // AVX2
    if( ( width & 15 ) == 0 && vext >= AVX2 )
    {
      __m256i    virBmask;
      __m256i    vsrca, vsrcat, vsrcab;
      __m256i    vbaseoffset  = _mm256_set1_epi16( 2 );
      __m256i    vplusone     = _mm256_set1_epi16( 1 );
      __m256i    vzero        = _mm256_set1_epi8( 0 );
      __m256i    vibdimax     = _mm256_set1_epi16( ( 1 << channelBitDepth ) - 1 );
      __m256i    voffsettbl   = _mm256_broadcastsi128_si256( _mm_loadu_si128( (__m128i*)p_eo_offsets ) );
      const Pel* srcLineBelow = srcLine + srcStride;

      for( y = startY; y < endY; y++ )
      {
        //         printf("y %d endY %d x%d y %d \n"y,endY,isAboveAvail,)
        if( !isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
        {
          for( x = 0; x < width; x += 16 )
          {
            virBmask = _mm256_loadu_si256( (__m256i*)&bndmask[x] );
            vsrca    = _mm256_loadu_si256( (__m256i*)&srcLine[x] );
            vsrcat   = _mm256_loadu_si256( (__m256i*)&srcLineAbove[x + 1] );
            vsrcab   = _mm256_loadu_si256( (__m256i*)&srcLineBelow[x - 1] );

            vsrcat            = _mm256_subs_epi16( vsrca, vsrcat );
            vsrcab            = _mm256_subs_epi16( vsrca, vsrcab );
            __m256i vsignt    = _mm256_sign_epi16( vplusone, vsrcat );
            __m256i vsignb    = _mm256_sign_epi16( vplusone, vsrcab );
            __m256i vsign     = _mm256_adds_epi16( _mm256_adds_epi16( vsignt, vsignb ), vbaseoffset );
            __m256i veoffsets = _mm256_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm256_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm256_srai_epi16( veoffsets, 8 );

            vsrcat = _mm256_add_epi16( vsrca, veoffsets );
            vsrcat = _mm256_min_epi16( _mm256_max_epi16( vsrcat, vzero ), vibdimax );

            vsrcab = _mm256_blendv_epi8( vsrcat, vsrca, virBmask );

            _mm256_storeu_si256( (__m256i*)&resLine[x], vsrcab );
          }
        }
        srcLine += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
    else
#  endif
    {
      __m128i vsrca, vsrcat, vsrcab, virBmask;
      __m128i vbaseoffset = _mm_set1_epi16( 2 );
      __m128i vplusone    = _mm_set1_epi16( 1 );
      __m128i vzero       = _mm_set1_epi8( 0 );
      __m128i vibdimax    = _mm_set1_epi16( ( 1 << channelBitDepth ) - 1 );
      __m128i voffsettbl  = _mm_loadu_si128( (__m128i*)p_eo_offsets );

      for( y = startY; y < endY; y++ )
      {
        if( !isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
        {
          for( x = 0; x < width; x += 8 )
          {
            vsrca    = _mm_loadu_si128( (__m128i*)&srcLine[x] );
            vsrcat   = _mm_loadu_si128( (__m128i*)&srcLineAbove[x + 1] );
            virBmask = _mm_loadu_si128( (__m128i*)&bndmask[x] );

            vsrcab            = _mm_loadu_si128( (__m128i*)&srcLineBelow[x - 1] );
            vsrcat            = _mm_subs_epi16( vsrca, vsrcat );
            vsrcab            = _mm_subs_epi16( vsrca, vsrcab );
            __m128i vsignt    = _mm_sign_epi16( vplusone, vsrcat );
            __m128i vsignb    = _mm_sign_epi16( vplusone, vsrcab );
            __m128i vsign     = _mm_adds_epi16( _mm_adds_epi16( vsignt, vsignb ), vbaseoffset );
            __m128i veoffsets = _mm_shuffle_epi8( voffsettbl, vsign );
            veoffsets         = _mm_slli_epi16( veoffsets, 8 );
            veoffsets         = _mm_srai_epi16( veoffsets, 8 );

            vsrcat = _mm_add_epi16( vsrca, veoffsets );
            vsrcat = _mm_min_epi16( _mm_max_epi16( vsrcat, vzero ), vibdimax );

            vsrcab = _mm_blendv_epi8( vsrcat, vsrca, virBmask );

            _mm_store_si128( (__m128i*)&resLine[x], vsrcab );
          }
        }
        srcLine += srcStride;
        srcLineBelow += srcStride;
        srcLineAbove += srcStride;
        resLine += resStride;
      }
    }
  }
  else
  {
    offset += 2;
    int8_t* signUpLine = &m_signLineBuf1->at( 1 );
    startX             = isLeftAvail ? 0 : 1;
    endX               = isRightAvail ? width : ( width - 1 );
    // prepare 2nd line upper sign
    const Pel* srcLineBelow = srcLine + srcStride;
    for( x = startX - 1; x < endX; x++ )
    {
      signUpLine[x] = (int8_t)sgn( srcLineBelow[x] - srcLine[x + 1] );
    }

    if( isCtuCrossedByVirtualBoundaries )
    {
      // first line
      const Pel* srcLineAbove = srcLine - srcStride;
      firstLineStartX         = isAboveAvail ? startX : ( width - 1 );
      firstLineEndX           = isAboveRightAvail ? width : ( width - 1 );
      if( !isHorProcessDisabled( 0, numHorVirBndry, horVirBndryPos ) )
      {
        for( x = firstLineStartX; x < firstLineEndX; x++ )
        {
          if( isVerProcessDisabled( x, numVerVirBndry, verVirBndryPos ) )
          {
            continue;
          }
          edgeType   = sgn( srcLine[x] - srcLineAbove[x + 1] ) - signUpLine[x - 1];
          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
        }
      }
      srcLine += srcStride;
      resLine += resStride;
      // middle lines
      for( y = 1; y < height - 1; y++ )
      {
        if( isHorProcessDisabled( y, numHorVirBndry, horVirBndryPos ) )
        {
          srcLineBelow = srcLine + ( srcStride );
          for( x = startX; x < endX; x++ )
          {
            signDown          = (int8_t)sgn( srcLine[x] - srcLineBelow[x - 1] );
            signUpLine[x - 1] = -signDown;
          }
          signUpLine[endX - 1] = (int8_t)sgn( srcLineBelow[endX - 1] - srcLine[endX] );
          srcLine += srcStride;
          resLine += ( resStride );
        }
        else
        {
          srcLineBelow = srcLine + srcStride;
          for( x = startX; x < endX; x++ )
          {
            signDown = (int8_t)sgn( srcLine[x] - srcLineBelow[x - 1] );
            if( isVerProcessDisabled( x, numVerVirBndry, verVirBndryPos ) )
            {
              signUpLine[x - 1] = -signDown;
              continue;
            }
            edgeType          = signDown + signUpLine[x];
            resLine[x]        = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
            signUpLine[x - 1] = -signDown;
          }
          signUpLine[endX - 1] = (int8_t)sgn( srcLineBelow[endX - 1] - srcLine[endX] );
          srcLine += srcStride;
          resLine += resStride;
        }
      }
      // last line
      srcLineBelow   = srcLine + srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      if( !isHorProcessDisabled( height - 1, numHorVirBndry, horVirBndryPos ) )
      {
        for( x = lastLineStartX; x < lastLineEndX; x++ )
        {
          if( isVerProcessDisabled( x, numVerVirBndry, verVirBndryPos ) )
          {
            continue;
          }
          edgeType   = sgn( srcLine[x] - srcLineBelow[x - 1] ) + signUpLine[x];
          resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
        }
      }
    }
    else
    {
      // first line
      const Pel* srcLineAbove = srcLine - srcStride;
      firstLineStartX         = isAboveAvail ? startX : ( width - 1 );
      firstLineEndX           = isAboveRightAvail ? width : ( width - 1 );
      for( x = firstLineStartX; x < firstLineEndX; x++ )
      {
        edgeType   = sgn( srcLine[x] - srcLineAbove[x + 1] ) - signUpLine[x - 1];
        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
      }
      srcLine += srcStride;
      resLine += resStride;
      // middle lines
      for( y = 1; y < height - 1; y++ )
      {
        srcLineBelow = srcLine + srcStride;
        for( x = startX; x < endX; x++ )
        {
          signDown          = (int8_t)sgn( srcLine[x] - srcLineBelow[x - 1] );
          edgeType          = signDown + signUpLine[x];
          resLine[x]        = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
          signUpLine[x - 1] = -signDown;
        }
        signUpLine[endX - 1] = (int8_t)sgn( srcLineBelow[endX - 1] - srcLine[endX] );
        srcLine += srcStride;
        resLine += resStride;
      }
      // last line
      srcLineBelow   = srcLine + srcStride;
      lastLineStartX = isBelowLeftAvail ? 0 : 1;
      lastLineEndX   = isBelowAvail ? endX : 1;
      for( x = lastLineStartX; x < lastLineEndX; x++ )
      {
        edgeType   = sgn( srcLine[x] - srcLineBelow[x - 1] ) + signUpLine[x];
        resLine[x] = ClipPel<int>( srcLine[x] + offset[edgeType], clpRng );
      }
    }
  }
}

template<X86_VEXT vext>
void offsetBlock_SIMD( const int            channelBitDepth,
                       const ClpRng&        clpRng,
                       int                  typeIdx,
                       int*                 offset,
                       int                  startIdx,
                       const Pel*           srcBlk,
                       Pel*                 resBlk,
                       ptrdiff_t            srcStride,
                       ptrdiff_t            resStride,
                       int                  width,
                       int                  height,
                       bool                 isLeftAvail,
                       bool                 isRightAvail,
                       bool                 isAboveAvail,
                       bool                 isBelowAvail,
                       bool                 isAboveLeftAvail,
                       bool                 isAboveRightAvail,
                       bool                 isBelowLeftAvail,
                       bool                 isBelowRightAvail,
                       std::vector<int8_t>* m_signLineBuf1,
                       std::vector<int8_t>* m_signLineBuf2,
                       bool                 isCtuCrossedByVirtualBoundaries,
                       int                  horVirBndryPos[],
                       int                  verVirBndryPos[],
                       int                  numHorVirBndry,
                       int                  numVerVirBndry )
{
  if( typeIdx == SAO_TYPE_BO )
  {
    offsetBlock_SIMD_SAO_TYPE_BO<vext>( channelBitDepth, offset, startIdx, srcBlk, resBlk, srcStride, resStride, width, height );

#  if USE_AVX2
    _mm256_zeroupper();
#  endif
    return;
  }

  uint16_t bndmask[MAX_CU_SIZE];
  memset( &bndmask, 0, MAX_CU_SIZE * sizeof( uint16_t ) );
  if( isCtuCrossedByVirtualBoundaries && numVerVirBndry > 0 )
  {
    for( int i = 0; i < numVerVirBndry; i++ )
    {
      if( verVirBndryPos[i] >= 0 && verVirBndryPos[i] < width )
      {
        bndmask[verVirBndryPos[i]] = 0xffff;
      }

      if( verVirBndryPos[i] - 1 >= 0 && verVirBndryPos[i] - 1 < width )
      {
        bndmask[verVirBndryPos[i] - 1] = 0xffff;
      }
    }
  }

  switch( typeIdx )
  {
  case SAO_TYPE_EO_0:
    offsetBlock_SIMD_SAO_TYPE_EO_0<vext>( channelBitDepth,
                                          clpRng,
                                          offset,
                                          srcBlk,
                                          resBlk,
                                          srcStride,
                                          resStride,
                                          width,
                                          height,
                                          isLeftAvail,
                                          isRightAvail,
                                          isAboveAvail,
                                          isBelowAvail,
                                          isAboveLeftAvail,
                                          isAboveRightAvail,
                                          isBelowLeftAvail,
                                          isBelowRightAvail,
                                          m_signLineBuf1,
                                          m_signLineBuf2,
                                          isCtuCrossedByVirtualBoundaries,
                                          horVirBndryPos,
                                          verVirBndryPos,
                                          numHorVirBndry,
                                          numVerVirBndry,
                                          bndmask );
    break;

  case SAO_TYPE_EO_90:
    offsetBlock_SIMD_SAO_TYPE_EO_90<vext>( channelBitDepth,
                                           clpRng,
                                           offset,
                                           srcBlk,
                                           resBlk,
                                           srcStride,
                                           resStride,
                                           width,
                                           height,
                                           isLeftAvail,
                                           isRightAvail,
                                           isAboveAvail,
                                           isBelowAvail,
                                           isAboveLeftAvail,
                                           isAboveRightAvail,
                                           isBelowLeftAvail,
                                           isBelowRightAvail,
                                           m_signLineBuf1,
                                           m_signLineBuf2,
                                           isCtuCrossedByVirtualBoundaries,
                                           horVirBndryPos,
                                           verVirBndryPos,
                                           numHorVirBndry,
                                           numVerVirBndry,
                                           bndmask );
    break;

  case SAO_TYPE_EO_135:
    offsetBlock_SIMD_SAO_TYPE_EO_135<vext>( channelBitDepth,
                                            clpRng,
                                            offset,
                                            srcBlk,
                                            resBlk,
                                            srcStride,
                                            resStride,
                                            width,
                                            height,
                                            isLeftAvail,
                                            isRightAvail,
                                            isAboveAvail,
                                            isBelowAvail,
                                            isAboveLeftAvail,
                                            isAboveRightAvail,
                                            isBelowLeftAvail,
                                            isBelowRightAvail,
                                            m_signLineBuf1,
                                            m_signLineBuf2,
                                            isCtuCrossedByVirtualBoundaries,
                                            horVirBndryPos,
                                            verVirBndryPos,
                                            numHorVirBndry,
                                            numVerVirBndry,
                                            bndmask );
    break;

  case SAO_TYPE_EO_45:
    offsetBlock_SIMD_SAO_TYPE_EO_45<vext>( channelBitDepth,
                                           clpRng,
                                           offset,
                                           srcBlk,
                                           resBlk,
                                           srcStride,
                                           resStride,
                                           width,
                                           height,
                                           isLeftAvail,
                                           isRightAvail,
                                           isAboveAvail,
                                           isBelowAvail,
                                           isAboveLeftAvail,
                                           isAboveRightAvail,
                                           isBelowLeftAvail,
                                           isBelowRightAvail,
                                           m_signLineBuf1,
                                           m_signLineBuf2,
                                           isCtuCrossedByVirtualBoundaries,
                                           horVirBndryPos,
                                           verVirBndryPos,
                                           numHorVirBndry,
                                           numVerVirBndry,
                                           bndmask );
    break;

  default:
    THROW_FATAL( "Not a supported SAO types\n" );
  }

#  if USE_AVX2
  _mm256_zeroupper();
#  endif
}

template<X86_VEXT vext>
void SampleAdaptiveOffset::_initSampleAdaptiveOffsetX86()
{
  offsetBlock = offsetBlock_SIMD<vext>;
}

template void SampleAdaptiveOffset::_initSampleAdaptiveOffsetX86<SIMDX86>();
#endif   //#ifdef TARGET_SIMD_X86

}
