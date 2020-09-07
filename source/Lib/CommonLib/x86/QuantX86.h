/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVdec

(c) Copyright (2018-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVdec (“Fraunhofer Versatile Video Decoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Decoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Decoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Decoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Decoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Decoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Decoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Decoding Library. If You use the Fraunhofer Versatile Video Decoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Decoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Decoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Decoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Decoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
------------------------------------------------------------------------------------------- */

/** \file     QuantX86.h
    \brief    SIMD for Quant/Dequant
*/

//! \ingroup CommonLib
//! \{


#include "CommonLib/CommonDef.h"
#include "CommonDefX86.h"
#include "CommonLib/Quant.h"


#if ENABLE_SIMD_OPT_QUANT
#ifdef TARGET_SIMD_X86

#define _mm_loadu_si64(p) _mm_loadl_epi64((__m128i const*)(p))
#define _mm_storeu_si64(p, a) (_mm_storel_epi64((__m128i*)(p), (a)))
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
  const TCoeff transformMinimum = -(transformMaximum);
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
        __m128i v_level = _mm_loadu_si64( ( __m128i const * )&piQCoef[y * piQCfStride]  );
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
        __m128i v_level = _mm_loadu_si64( ( __m128i const * )&piQCoef[y * piQCfStride]  );
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
//! \}
