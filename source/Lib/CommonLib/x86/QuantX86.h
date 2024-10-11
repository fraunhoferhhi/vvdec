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

/** \file     QuantX86.h
    \brief    SIMD for Quant/Dequant
*/

#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "CommonLib/Quant.h"

namespace vvdec
{

#if ENABLE_SIMD_OPT_QUANT
#ifdef TARGET_SIMD_X86

#if USE_AVX2 && !defined( _mm256_set_m128i )
#define VVCLIB_OWN_mm256_set_m128i
#define _mm256_set_m128i( v0, v1 ) _mm256_inserti128_si256( _mm256_castsi128_si256( v1 ), ( v0 ), 1 )

#endif

static constexpr unsigned short levmask[16] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0,0,0,0,0,0};
#if USE_AVX2
static constexpr unsigned short xlevmask[32] = {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif


template< X86_VEXT vext>
static void DeQuantCoreSIMD(const int maxX,const int restX,const int maxY,const int scale,const TCoeffSig*const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum)
{
  const int inputMinimum = -(inputMaximum+1);
  const TCoeff transformMinimum = -(transformMaximum+1);
  const int width = restX+maxX+1;
  __m128i vlevmask;
  if (maxX<7)
    vlevmask = _mm_loadu_si128( ( __m128i const * )&levmask[7-maxX] );
  else
    vlevmask = _mm_set_epi64x(0xffffffffffffffff,0xffffffffffffffff);

#if USE_AVX2
  __m256i xvlevmask;
  if (maxX<15)
    xvlevmask = _mm256_loadu_si256( ( __m256i const * )&xlevmask[15-maxX] );
  else
    xvlevmask = _mm256_set_epi64x(0xffffffffffffffff,0xffffffffffffffff,0xffffffffffffffff,0xffffffffffffffff);

#endif

  if (rightShift>0)
  {
    const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);

    __m128i v_max =  _mm_set1_epi16 ((short)inputMaximum);
    __m128i v_min =  _mm_set1_epi16 ((short)inputMinimum);
    __m128i v_Tmax =  _mm_set1_epi32 ((short)transformMaximum);
    __m128i v_Tmin =  _mm_set1_epi32 ((short)transformMinimum);
    __m128i v_scale = _mm_set1_epi16 ((short)scale);
    __m128i v_add = _mm_set1_epi32 (iAdd);
    __m128i v_rshift = _mm_set1_epi64x (rightShift);

    if (maxX<4)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = maxX  > 1 ? _mm_loadu_si64( ( const __m128i* ) &piQCoef[y * piQCfStride] )
                        : maxX == 1 ? _mm_setr_epi16( piQCoef[y * piQCfStride], piQCoef[y * piQCfStride + 1], 0, 0, 0, 0, 0, 0 )
                                    : _mm_setr_epi16( piQCoef[y * piQCfStride], 0, 0, 0, 0, 0, 0, 0 );

        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level =  _mm_add_epi32(v_level,v_add);
        v_level = _mm_sra_epi32(v_level,v_rshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        _mm_storeu_si128(( __m128i * )(piCoef+y*width ), v_level );
      }
    }
#if USE_AVX2
    else if (maxX<8)
    {
      __m256i xv_add = _mm256_set1_epi32 (iAdd);
      __m256i xv_Tmax =  _mm256_set1_epi32 ((short)transformMaximum);
      __m256i xv_Tmin =  _mm256_set1_epi32 ((short)transformMinimum);

      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = _mm_loadu_si128( ( __m128i const * )&piQCoef[y * piQCfStride]  );
        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        __m256i xv_level = _mm256_set_m128i (_mm_unpackhi_epi16(v_low,v_high), _mm_unpacklo_epi16(v_low,v_high));

        xv_level =  _mm256_add_epi32(xv_level,xv_add);
        xv_level = _mm256_sra_epi32(xv_level,v_rshift);

        xv_level = _mm256_max_epi32 (xv_level, xv_Tmin);
        xv_level = _mm256_min_epi32 (xv_level, xv_Tmax);

        _mm256_storeu_si256(( __m256i * )(piCoef+y*width ), xv_level );
      }
    }
    else
    {
      __m256i xv_max =  _mm256_set1_epi16 ((short)inputMaximum);
      __m256i xv_min =  _mm256_set1_epi16 ((short)inputMinimum);
      __m256i xv_scale = _mm256_set1_epi16 ((short)scale);
      __m256i xv_add = _mm256_set1_epi32 (iAdd);
      __m256i xv_Tmax =  _mm256_set1_epi32 ((short)transformMaximum);
      __m256i xv_Tmin =  _mm256_set1_epi32 ((short)transformMinimum);

      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=16)
        {
          __m256i xv_level = _mm256_loadu_si256( ( __m256i const * )&piQCoef[x+ y * piQCfStride]  );
          xv_level = _mm256_and_si256(xv_level,xvlevmask);
          xv_level = _mm256_max_epi16 (xv_level, xv_min);
          xv_level = _mm256_min_epi16 (xv_level, xv_max);
          __m256i xv_low = _mm256_mullo_epi16(xv_level,xv_scale);
          __m256i xv_high = _mm256_mulhi_epi16(xv_level,xv_scale);

          xv_low = _mm256_permute4x64_epi64(xv_low,0xD8);
          xv_high = _mm256_permute4x64_epi64(xv_high,0xD8);


          xv_level = _mm256_unpacklo_epi16(xv_low,xv_high);
          xv_level =  _mm256_add_epi32(xv_level,xv_add);
          xv_level = _mm256_sra_epi32(xv_level,v_rshift);

          xv_level = _mm256_max_epi32 (xv_level, xv_Tmin);
          xv_level = _mm256_min_epi32 (xv_level, xv_Tmax);

          _mm256_storeu_si256(( __m256i * )(piCoef+x+y*width ), xv_level );
          xv_level = _mm256_unpackhi_epi16(xv_low,xv_high);
          xv_level =  _mm256_add_epi32(xv_level,xv_add);
          xv_level = _mm256_sra_epi32(xv_level,v_rshift);

          xv_level = _mm256_max_epi32 (xv_level, xv_Tmin);
          xv_level = _mm256_min_epi32 (xv_level, xv_Tmax);

          _mm256_storeu_si256(( __m256i * )(piCoef+8+x+y*width ), xv_level );
        }
      }

    }

#else
/*
    else if (maxX<8)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = _mm_loadu_si128( ( __m128i const * )&piQCoef[y * piQCfStride]  );
        v_level = _mm_and_si128(v_level,vlevmask);
        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level =  _mm_add_epi32(v_level,v_add);
        v_level = _mm_sra_epi32(v_level,v_rshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        _mm_storeu_si128(( __m128i * )(piCoef+y*width ), v_level );

        v_level = _mm_unpackhi_epi16(v_low,v_high);
        v_level =  _mm_add_epi32(v_level,v_add);
        v_level = _mm_sra_epi32(v_level,v_rshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        _mm_storeu_si128(( __m128i * )(piCoef+4+y*width ), v_level );
      }
    }
*/
    else
    {
      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=8)
        {
          __m128i v_level = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+ y * piQCfStride]  );
          v_level = _mm_and_si128(v_level,vlevmask);
          v_level = _mm_max_epi16 (v_level, v_min);
          v_level = _mm_min_epi16 (v_level, v_max);
          __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
          __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

          v_level = _mm_unpacklo_epi16(v_low,v_high);
          v_level =  _mm_add_epi32(v_level,v_add);
          v_level = _mm_sra_epi32(v_level,v_rshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+x+y*width ), v_level );

          if( maxX + 1 - x <= 4 ) continue;

          v_level = _mm_unpackhi_epi16(v_low,v_high);
          v_level =  _mm_add_epi32(v_level,v_add);
          v_level = _mm_sra_epi32(v_level,v_rshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+4+x+y*width ), v_level );
        }
      }
    }
#endif
  }
  else  // rightshift <0
  {

    __m128i v_max =  _mm_set1_epi16 ((short)inputMaximum);
    __m128i v_min =  _mm_set1_epi16 ((short)inputMinimum);
    __m128i v_Tmax =  _mm_set1_epi32 ((short)transformMaximum);
    __m128i v_Tmin =  _mm_set1_epi32 ((short)transformMinimum);
    __m128i v_scale = _mm_set1_epi16 ((short)scale);
    __m128i v_lshift = _mm_set1_epi64x (-rightShift);

    if (maxX<4)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = maxX  > 1 ? _mm_loadu_si64( ( const __m128i* ) &piQCoef[y * piQCfStride] )
                        : maxX == 1 ? _mm_setr_epi16( piQCoef[y * piQCfStride], piQCoef[y * piQCfStride + 1], 0, 0, 0, 0, 0, 0 )
                                    : _mm_setr_epi16( piQCoef[y * piQCfStride], 0, 0, 0, 0, 0, 0, 0 );

        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level = _mm_sll_epi32(v_level,v_lshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        _mm_storeu_si128(( __m128i * )(piCoef+y*width ), v_level );

      }
    }
#if USE_AVX2
    else if (maxX<8)
    {
      __m256i xv_Tmax =  _mm256_set1_epi32 ((short)transformMaximum);
      __m256i xv_Tmin =  _mm256_set1_epi32 ((short)transformMinimum);

      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = _mm_loadu_si128( ( __m128i const * )&piQCoef[y * piQCfStride]  );
        v_level = _mm_and_si128(v_level,vlevmask);
        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        __m256i xv_level = _mm256_set_m128i (_mm_unpackhi_epi16(v_low,v_high), _mm_unpacklo_epi16(v_low,v_high));

        xv_level = _mm256_sll_epi32(xv_level,v_lshift);

        xv_level = _mm256_max_epi32 (xv_level, xv_Tmin);
        xv_level = _mm256_min_epi32 (xv_level, xv_Tmax);

        _mm256_storeu_si256(( __m256i * )(piCoef+y*width ), xv_level );
      }
    }

    else
    {
      __m256i xv_max =  _mm256_set1_epi16 ((short)inputMaximum);
      __m256i xv_min =  _mm256_set1_epi16 ((short)inputMinimum);
      __m256i xv_scale = _mm256_set1_epi16 ((short)scale);
      __m256i xv_Tmax =  _mm256_set1_epi32 ((short)transformMaximum);
      __m256i xv_Tmin =  _mm256_set1_epi32 ((short)transformMinimum);

      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=16)
        {
          __m256i xv_level = _mm256_loadu_si256( ( __m256i const * )&piQCoef[x+ y * piQCfStride]  );
          xv_level = _mm256_and_si256(xv_level,xvlevmask);
          xv_level = _mm256_max_epi16 (xv_level, xv_min);
          xv_level = _mm256_min_epi16 (xv_level, xv_max);
          __m256i xv_low = _mm256_mullo_epi16(xv_level,xv_scale);
          __m256i xv_high = _mm256_mulhi_epi16(xv_level,xv_scale);

          xv_low = _mm256_permute4x64_epi64(xv_low,0xD8);
          xv_high = _mm256_permute4x64_epi64(xv_high,0xD8);


          xv_level = _mm256_unpacklo_epi16(xv_low,xv_high);
          xv_level = _mm256_sll_epi32(xv_level,v_lshift);

          xv_level = _mm256_max_epi32 (xv_level, xv_Tmin);
          xv_level = _mm256_min_epi32 (xv_level, xv_Tmax);

          _mm256_storeu_si256(( __m256i * )(piCoef+x+y*width ), xv_level );
          xv_level = _mm256_unpackhi_epi16(xv_low,xv_high);
          xv_level = _mm256_sll_epi32(xv_level,v_lshift);

          xv_level = _mm256_max_epi32 (xv_level, xv_Tmin);
          xv_level = _mm256_min_epi32 (xv_level, xv_Tmax);

          _mm256_storeu_si256(( __m256i * )(piCoef+8+x+y*width ), xv_level );
        }
      }
    }

#else
/*
    else if (maxX<8)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = _mm_loadu_si128( ( __m128i const * )&piQCoef[y * piQCfStride]  );
        v_level = _mm_and_si128(v_level,vlevmask);
        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level = _mm_sll_epi32(v_level,v_lshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        _mm_storeu_si128(( __m128i * )(piCoef+y*width ), v_level );

        v_level = _mm_unpackhi_epi16(v_low,v_high);
        v_level = _mm_sll_epi32(v_level,v_lshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        _mm_storeu_si128(( __m128i * )(piCoef+4+y*width ), v_level );
      }
    }
*/
    else
    {
      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=8)
        {
          __m128i v_level = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+ y * piQCfStride]  );
          v_level = _mm_and_si128(v_level,vlevmask);
          v_level = _mm_max_epi16 (v_level, v_min);
          v_level = _mm_min_epi16 (v_level, v_max);
          __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
          __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

          v_level = _mm_unpacklo_epi16(v_low,v_high);
          v_level = _mm_sll_epi32(v_level,v_lshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+x+y*width ), v_level );

          if( maxX + 1 - x <= 4 ) continue;

          v_level = _mm_unpackhi_epi16(v_low,v_high);
          v_level = _mm_sll_epi32(v_level,v_lshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+4+x+y*width ), v_level );
        }
      }
    }
#endif
  }
}


template< X86_VEXT vext>
static void DeQuantCorePCMSIMD(const int maxX,const int restX,const int maxY,const int scale,TCoeff   *const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum)
{
  const int inputMinimum = -(inputMaximum+1);
  const TCoeff transformMinimum = -(transformMaximum);
  const int width = restX+maxX+1;
  __m128i vlevmask;
  if (maxX<7)
    vlevmask = _mm_loadu_si128( ( __m128i const * )&levmask[7-maxX] );
  else
    vlevmask = _mm_set_epi64x(0xffffffffffffffff,0xffffffffffffffff);

  if (rightShift>0)
  {
    const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);

    __m128i v_max =  _mm_set1_epi16 ((short)inputMaximum);
    __m128i v_min =  _mm_set1_epi16 ((short)inputMinimum);
    __m128i v_Tmax =  _mm_set1_epi32 ((short)transformMaximum);
    __m128i v_Tmin =  _mm_set1_epi32 ((short)transformMinimum);
    __m128i v_scale = _mm_set1_epi16 ((short)scale);
    __m128i v_add = _mm_set1_epi32 (iAdd);
    __m128i v_rshift = _mm_set1_epi64x (rightShift);

    if (maxX<4)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = _mm_loadu_si128( ( __m128i const * )&piQCoef[y * piQCfStride]  );
        v_level = _mm_packs_epi32 (v_level,v_level);
        v_level = _mm_and_si128(v_level,vlevmask);
        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level =  _mm_add_epi32(v_level,v_add);
        v_level = _mm_sra_epi32(v_level,v_rshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        _mm_storeu_si128(( __m128i * )(piCoef+y*width ), v_level );
      }
    }
    else
    {
      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=8)
        {

          __m128i v_levell = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+ y * piQCfStride]  );
          __m128i v_levelh = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+4 + y * piQCfStride]  );
          __m128i v_level = _mm_packs_epi32 (v_levell,v_levelh);
          v_level = _mm_and_si128(v_level,vlevmask);
          v_level = _mm_max_epi16 (v_level, v_min);
          v_level = _mm_min_epi16 (v_level, v_max);
          __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
          __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

          v_level = _mm_unpacklo_epi16(v_low,v_high);
          v_level =  _mm_add_epi32(v_level,v_add);
          v_level = _mm_sra_epi32(v_level,v_rshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+x+y*width ), v_level );

          v_level = _mm_unpackhi_epi16(v_low,v_high);
          v_level =  _mm_add_epi32(v_level,v_add);
          v_level = _mm_sra_epi32(v_level,v_rshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+4+x+y*width ), v_level );
        }
      }
    }
  }
  else  // rightshift <0
  {

    __m128i v_max =  _mm_set1_epi16 ((short)inputMaximum);
    __m128i v_min =  _mm_set1_epi16 ((short)inputMinimum);
    __m128i v_Tmax =  _mm_set1_epi32 ((short)transformMaximum);
    __m128i v_Tmin =  _mm_set1_epi32 ((short)transformMinimum);
    __m128i v_scale = _mm_set1_epi16 ((short)scale);
    __m128i v_lshift = _mm_set1_epi64x (-rightShift);

    if (maxX<4)
    {
      for( int y = 0; y <= maxY; y++)
      {
        __m128i v_level = _mm_loadu_si128( ( __m128i const * )&piQCoef[y * piQCfStride]  );
        v_level = _mm_packs_epi32 (v_level,v_level);
        v_level = _mm_and_si128(v_level,vlevmask);

        v_level = _mm_max_epi16 (v_level, v_min);
        v_level = _mm_min_epi16 (v_level, v_max);
        __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
        __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

        v_level = _mm_unpacklo_epi16(v_low,v_high);
        v_level = _mm_sll_epi32(v_level,v_lshift);

        v_level = _mm_max_epi32 (v_level, v_Tmin);
        v_level = _mm_min_epi32 (v_level, v_Tmax);
        _mm_storeu_si128(( __m128i * )(piCoef+y*width ), v_level );
      }
    }
    else
    {
      for( int y = 0; y <= maxY; y++)
      {
        for( int x = 0; x <= maxX; x+=8)
        {
          __m128i v_levell = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+ y * piQCfStride]  );
          __m128i v_levelh = _mm_loadu_si128( ( __m128i const * )&piQCoef[x+4 + y * piQCfStride]  );
          __m128i v_level = _mm_packs_epi32 (v_levell,v_levelh);
          v_level = _mm_and_si128(v_level,vlevmask);
          v_level = _mm_max_epi16 (v_level, v_min);
          v_level = _mm_min_epi16 (v_level, v_max);
          __m128i v_low = _mm_mullo_epi16(v_level,v_scale);
          __m128i v_high = _mm_mulhi_epi16(v_level,v_scale);

          v_level = _mm_unpacklo_epi16(v_low,v_high);
          v_level = _mm_sll_epi32(v_level,v_lshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+x+y*width ), v_level );

          v_level = _mm_unpackhi_epi16(v_low,v_high);
          v_level = _mm_sll_epi32(v_level,v_lshift);

          v_level = _mm_max_epi32 (v_level, v_Tmin);
          v_level = _mm_min_epi32 (v_level, v_Tmax);
          _mm_storeu_si128(( __m128i * )(piCoef+4+x+y*width ), v_level );
        }
      }
    }
  }
}




template<X86_VEXT vext>
void Quant::_initQuantX86()
{
  DeQuant = DeQuantCoreSIMD<vext>;
  DeQuantPCM = DeQuantCorePCMSIMD<vext>;
}
template void Quant::_initQuantX86<SIMDX86>();

#endif // TARGET_SIMD_X86
#endif

}
