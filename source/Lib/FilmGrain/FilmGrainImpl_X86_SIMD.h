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

#include "FilmGrainImplX86.h"

#include <algorithm>

#include <CommonDef.h>

#ifdef TARGET_SIMD_X86
#  include <CommonDefX86.h>

namespace vvdec
{
using namespace x86_simd;

template<>
void FilmGrainImplX86<CURR_X86_VEXT>::make_grain_pattern( const void* I,
                                                          int         c,
                                                          int         x,
                                                          int         subx,
                                                          uint8_t     oc1,
                                                          uint8_t     oc2,
                                                          uint8_t     ox,
                                                          uint8_t     ox_up,
                                                          uint8_t     oy,
                                                          uint8_t     oy_up,
                                                          int         s,
                                                          int         s_up,
                                                          int16_t     grain[3][32],
                                                          uint8_t     scale[3][32] ) const
{
  const uint8_t*  I8  = (const uint8_t*) I;
  const uint16_t* I16 = (const uint16_t*) I;
  if( allZero[c] == 1 )
  {
    if( c == 0 )
    {
      __m128i vP = _mm_lddqu_si128( (__m128i*) &pattern[0][0][oy][ox] );
      if( s == -1 )
      {
        vP = _mm_sub_epi8( _mm_set1_epi8( 0 ), vP );
      }
#ifdef USE_AVX2
      __m256i vmask = _mm256_set1_epi32(0xff);
      __m128i tmp0;
      __m128i tmp1;
      __m256i vintensity;
      if (bs)
      {
        vintensity = _mm256_lddqu_si256((__m256i*)&I16[x]);  //load 16 16 bit values
        vintensity = _mm256_sra_epi16 (vintensity, _mm_set_epi32 (0,0,0,bs));
        tmp0=_mm256_extracti128_si256 (vintensity,0);
        tmp1=_mm256_extracti128_si256 (vintensity,1);
      }
      else
      {
        __m128i vintensity128 = _mm_lddqu_si128((__m128i*)&I8[x]);  //load 16 8 bit value
        tmp0=_mm_cvtepi8_epi16 (vintensity128);
        tmp1=_mm_cvtepi8_epi16 (_mm_bsrli_si128(vintensity128,8));
        tmp0 = _mm_and_si128 (tmp0,_mm_set1_epi16(0xff));  // only 8 bit
        tmp1 = _mm_and_si128 (tmp1,_mm_set1_epi16(0xff));  // only 8 bit
        vintensity = _mm256_castsi128_si256 (vintensity128);
      }
      __m256i vindex0=_mm256_cvtepi16_epi32 (tmp0);
      __m256i vindex1=_mm256_cvtepi16_epi32 (tmp1);

      __m256i avP = _mm256_cvtepi8_epi16( vP );
      if( oc1 )
      {
        __m256i avoc1 = _mm256_set1_epi16( oc1 );
        __m256i avoc2 = _mm256_set1_epi16( oc2 );
        // p*oc1
        avP = _mm256_mullo_epi16( avP, avoc1 );   // max 16 Bit
        // pattern * s_up
        __m128i vP2 = _mm_lddqu_si128( (__m128i*) &pattern[0][0][oy_up][ox_up] );
        if( s_up == -1 )
        {
          vP2 = _mm_sub_epi8( _mm_set1_epi8( 0 ), vP2 );
        }
        __m256i avP2 = _mm256_cvtepi8_epi16( vP2 );
        // * oc2
        avP2 = _mm256_mullo_epi16( avP2, avoc2 );
        // add
        avP = _mm256_add_epi16( avP, avP2 );
        // round to 16 bit
        __m256i avadd   = _mm256_set1_epi16( 1 << ( 5 - 1 ) );
        __m128i avshift = _mm_set_epi16( 0, 0, 0, 0, 0, 0, 0, 5 );
        avP             = _mm256_add_epi16( avP, avadd );
        avP             = _mm256_sra_epi16( avP, avshift );
      }
      _mm256_storeu_si256( (__m256i*) &grain[c][16], avP );

      __m256i vscale0 = _mm256_i32gather_epi32 ((int *)&sLUT[0][0], vindex0, 1);  // load 8 32 bit values
      __m256i vscale1 = _mm256_i32gather_epi32 ((int *)&sLUT[0][0], vindex1, 1);  // load 8 32 bit values

      vscale0 = _mm256_and_si256 (vscale0,vmask);
      vscale1 = _mm256_and_si256 (vscale1,vmask);

      vintensity = _mm256_packus_epi32 (vscale0, vscale1);
      vscale0 = _mm256_permute4x64_epi64 (vintensity, 0x8);
      vscale1 = _mm256_permute4x64_epi64 (vintensity, 0xd);
      vscale0 = _mm256_packus_epi16 (vscale0, vscale1);
      _mm_storeu_si128(( __m128i * )&scale[0][16],_mm256_castsi256_si128(vscale0));
# else
      __m128i vPlo = _mm_cvtepi8_epi16( vP );
      __m128i vPhi = _mm_cvtepi8_epi16( _mm_bsrli_si128( vP, 8 ) );
      if( oc1 )
      {
        __m128i voc1 = _mm_set1_epi16( oc1 );
        __m128i voc2 = _mm_set1_epi16( oc2 );
        // p*oc1
        vPlo = _mm_mullo_epi16( vPlo, voc1 );   // max 16 Bit
        vPhi = _mm_mullo_epi16( vPhi, voc1 );
        // pattern * s_up
        __m128i vP2 = _mm_lddqu_si128( (__m128i*) &pattern[0][0][oy_up][ox_up] );
        if( s_up == -1 )
        {
          vP2 = _mm_sub_epi8( _mm_set1_epi8( 0 ), vP2 );
        }
        __m128i vP2lo = _mm_cvtepi8_epi16( vP2 );
        __m128i vP2hi = _mm_cvtepi8_epi16( _mm_bsrli_si128( vP2, 8 ) );
        // * oc2
        vP2lo = _mm_mullo_epi16( vP2lo, voc2 );
        vP2hi = _mm_mullo_epi16( vP2hi, voc2 );
        // add
        vPlo = _mm_add_epi16( vPlo, vP2lo );
        vPhi = _mm_add_epi16( vPhi, vP2hi );
        // round to 16 bit
        __m128i vadd   = _mm_set1_epi16( 1 << ( 5 - 1 ) );
        __m128i vshift = _mm_set_epi16( 0, 0, 0, 0, 0, 0, 0, 5 );
        vPlo           = _mm_add_epi16( vPlo, vadd );
        vPhi           = _mm_add_epi16( vPhi, vadd );
        vPlo           = _mm_sra_epi16( vPlo, vshift );
        vPhi           = _mm_sra_epi16( vPhi, vshift );
      }
      _mm_storeu_si128( (__m128i*) &grain[c][16], vPlo );
      _mm_storeu_si128( (__m128i*) &grain[c][16 + 8], vPhi );
      // Scale sign already integrated above because of overlap
      //scale[0][16+i] = sLUT[0][intensity];
      uint8_t intensity;
      uint8_t *pscale=&scale[0][16];
      const uint8_t *pLUT=sLUT[0];
      if (bs)
      {
        const uint16_t *pI16 = I16+x;
        for (int i=0; i<16; i++)
        {
          intensity =  *pI16++ >> bs ;
          *pscale++ = pLUT[intensity];
        }
      }
      else
      {
        const uint8_t *pI8 = I8+x;
        for (int i=0; i<16; i++)
        {
          intensity =  *pI8++ ;
          *pscale++ = pLUT[intensity];
        }
      }
#endif
    }   // Y
    else
    {   // U/V
      __m128i vP;
#ifdef USE_AVX2
      __m256i vindex;
      __m128i vintensity;
      if (bs)
      {
        vintensity = _mm_lddqu_si128((__m128i*)&I16[x>>1]);  //load 8 16 bit values
        vintensity = _mm_sra_epi16 (vintensity, _mm_set_epi32 (0,0,0,bs));
      }
      else
      {
        vintensity = _mm_loadu_si64(&I8[x>>1]);  //load 8 8 bit values
        vintensity=_mm_cvtepi8_epi16 (vintensity);
        vintensity = _mm_and_si128 (vintensity,_mm_set1_epi16(0xff));  // only 8 bit
      }
      vindex=_mm256_cvtepi16_epi32 (vintensity);
#endif
      vP = _mm_loadl_epi64( (__m128i*) &pattern[1][0][oy][ox] );

      if( s == -1 )
      {
        vP = _mm_sub_epi8( _mm_set1_epi8( 0 ), vP );
      }
      __m128i vPlo = _mm_cvtepi8_epi16( vP );
      if( oc1 )
      {
        __m128i voc1 = _mm_set1_epi16( oc1 );
        __m128i voc2 = _mm_set1_epi16( oc2 );
        // p*oc1
        vPlo = _mm_mullo_epi16( vPlo, voc1 );   // max 16 Bit
        // pattern * s_up
        __m128i vP2 = _mm_loadl_epi64( (__m128i*) &pattern[c ? 1 : 0][0][oy_up][ox_up] );
        if( s_up == -1 )
        {
          vP2 = _mm_sub_epi8( _mm_set1_epi8( 0 ), vP2 );
        }
        __m128i vP2lo = _mm_cvtepi8_epi16( vP2 );
        // * oc2
        vP2lo = _mm_mullo_epi16( vP2lo, voc2 );
        // add
        vPlo = _mm_add_epi16( vPlo, vP2lo );
        // round to 16 bit
        __m128i vadd   = _mm_set1_epi16( 1 << ( 5 - 1 ) );
        __m128i vshift = _mm_set_epi16( 0, 0, 0, 0, 0, 0, 0, 5 );
        vPlo           = _mm_add_epi16( vPlo, vadd );
        vPlo           = _mm_sra_epi16( vPlo, vshift );
      }
      _mm_storeu_si128( (__m128i*) &grain[c][8], vPlo );
#ifdef USE_AVX2
      __m256i vmask = _mm256_set1_epi32(0xff);
      __m256i vscale = _mm256_i32gather_epi32 ((int *)&sLUT[c][0], vindex, 1);  // load 8 32 bit values
      vscale = _mm256_and_si256 (vscale,vmask);

      vmask = _mm256_packus_epi32 (vscale, vscale);
      vscale = _mm256_permute4x64_epi64 (vmask, 0x8);
      vscale = _mm256_packus_epi16 (vscale, vscale);
      _mm_storeu_si64(( __m128i * )&scale[c][8],_mm256_castsi256_si128(vscale));
#else
      uint8_t*       pscale = &scale[c][8];
      const uint8_t* pLUT   = sLUT[c];
      if (bs)
      {
        const uint16_t* pI16 = &I16[x >> 1];
        for( int i = 0; i < 8; i++ )
        {
          uint8_t intensity = *pI16++ >> bs;
          *pscale++         = pLUT[intensity];
        }
      }
      else
      {
        const uint8_t* pI8 = &I8[x >> 1];
        for( int i = 0; i < 8; i++ )
        {
          uint8_t intensity = *pI8++;
          *pscale++         = pLUT[intensity];
        }
      }
#endif
    }
  }
#ifdef USE_AVX2
  else if( c>0 && allZero[c] == 0 )
  {
    __m128i vP;
    __m128i vintensity;
    __m256i vindex;
    __m256i vmask = _mm256_set1_epi32(0xff);
    if (bs)
    {
      vintensity = _mm_lddqu_si128((__m128i*)&I16[x>>1]);  //load 8 16 bit values
      vintensity = _mm_sra_epi16 (vintensity, _mm_set_epi32 (0,0,0,bs));
    }
    else
    {
      vintensity = _mm_loadu_si64(&I8[x>>1]);  //load 8 8 bit values
      vintensity=_mm_cvtepi8_epi16 (vintensity);
    }
    vindex=_mm256_cvtepi16_epi32 (vintensity);
    vindex = _mm256_and_si256 (vindex,vmask);  // only 8 bit

    __m256i vadd = _mm256_set_epi32(7,6,5,4,3,2,1,0);
    __m256i vpi = _mm256_i32gather_epi32 ((int *)&pLUT[c][0], vindex, 1);  // load 8 32 bit values
    vpi = _mm256_and_si256 (vpi,vmask);  // only 8 bit
    vpi = _mm256_slli_epi32 (vpi, 8);  // 12-4
    vpi = _mm256_add_epi32 (vpi, vadd);
    __m256i avP = _mm256_i32gather_epi32 ((int *)&pattern[1][0][oy][ox], vpi, 1);  // load 8 32 bit values
    avP = _mm256_and_si256 (avP,vmask);  // only 8 bit
    // convert to packed 8 bit
    __m256i vtmp = _mm256_packus_epi32 (avP, avP);
    avP = _mm256_permute4x64_epi64 (vtmp, 0x8);
    avP = _mm256_packus_epi16 (avP, avP);
    vP = _mm256_castsi256_si128(avP);
    if( s == -1 )
    {
      vP = _mm_sub_epi8( _mm_set1_epi8( 0 ), vP );
    }
    __m128i vPlo = _mm_cvtepi8_epi16( vP );
    if( oc1 )
    {
      __m128i voc1 = _mm_set1_epi16( oc1 );
      __m128i voc2 = _mm_set1_epi16( oc2 );
      // p*oc1
      vPlo = _mm_mullo_epi16( vPlo, voc1 );   // max 16 Bit
      // pattern * s_up
      __m256i avP2 = _mm256_i32gather_epi32 ((int *)&pattern[1][0][oy_up][ox_up], vpi, 1);  // load 8 32 bit values
      avP2 = _mm256_and_si256 (avP2,vmask);  // only 8 bit
      // convert to packed 8 bit
      vtmp = _mm256_packus_epi32 (avP2, avP2);
      avP2 = _mm256_permute4x64_epi64 (vtmp, 0x8);
      avP2 = _mm256_packus_epi16 (avP2, avP2);
      __m128i vP2= _mm256_castsi256_si128(avP2);
      if( s_up == -1 )
      {
        vP2 = _mm_sub_epi8( _mm_set1_epi8( 0 ), vP2 );
      }
      __m128i vP2lo = _mm_cvtepi8_epi16( vP2 );
      vP2lo = _mm_mullo_epi16( vP2lo, voc2 );
      vPlo = _mm_add_epi16( vPlo, vP2lo );
      // round to 16 bit
      __m128i vadd   = _mm_set1_epi16( 1 << ( 5 - 1 ) );
      __m128i vshift = _mm_set_epi16( 0, 0, 0, 0, 0, 0, 0, 5 );
      vPlo           = _mm_add_epi16( vPlo, vadd );
      vPlo           = _mm_sra_epi16( vPlo, vshift );
    }
    _mm_storeu_si128( (__m128i*) &grain[c][8], vPlo );
    __m256i vscale = _mm256_i32gather_epi32 ((int *)&sLUT[c][0], vindex, 1);  // load 8 32 bit values
    vscale = _mm256_and_si256 (vscale,vmask);
    vmask = _mm256_packus_epi32 (vscale, vscale);
    vscale = _mm256_permute4x64_epi64 (vmask, 0x8);
    vscale = _mm256_packus_epi16 (vscale, vscale);
    _mm_storeu_si64(( __m128i * )&scale[c][8],_mm256_castsi256_si128(vscale));
  }
#endif
  else
  {
    for( int i = 0; i < 16 / subx; i++ )
    {
      uint8_t intensity = bs ? I16[x / subx + i] >> bs : I8[x / subx + i];
      uint8_t pi        = pLUT[c][intensity] >> 4;                  // pattern index (integer part)
      int     P         = pattern[c ? 1 : 0][pi][oy][ox + i] * s;   // Pattern sample (from current pattern index)
                                                                    // We could consider just XORing the sign bit
#if PATTERN_INTERPOLATION
      uint8_t pf = pLUT[c][intensity] & 15;           // pattern index fractional part (interpolate with next) -- could restrict to less bits (e.g. 2)
      int     Pn =
        pattern[c ? 1 : 0][pi + 1][oy][ox + i] * s;   // Next-pattern sample (from pattern index+1)
                                                      // But there are equivalent hw tricks, e.g. storing values as sign + amplitude instead of two's complement
#endif

      if( oc1 )   // overlap
      {
        P = round( P * oc1 + pattern[c ? 1 : 0][pi][oy_up][ox_up + i] * oc2 * s_up, 5 );
#if PATTERN_INTERPOLATION
        Pn = round( Pn * oc1 + pattern[c ? 1 : 0][pi + 1][oy_up][ox_up + i] * oc2 * s_up, 5 );
#endif
      }
#if PATTERN_INTERPOLATION
      // Pattern interpolation: P is current, Pn is next, pf is interpolation coefficient
      grain[c][16 / subx + i] = round( P * ( 16 - pf ) + Pn * pf, 4 );
#else
      grain[c][16 / subx + i] = P;
#endif
      // Scale sign already integrated above because of overlap
      scale[c][16 / subx + i] = sLUT[c][intensity];
    }
  }
}

template<>
void FilmGrainImplX86<CURR_X86_VEXT>::scale_and_output( void* I, int c, int x, int subx, int width, int16_t grain[3][32], uint8_t scale[3][32] ) const
{
  uint8_t*  I8  = (uint8_t*) I;
  uint16_t* I16 = (uint16_t*) I;

  const uint8_t I_min = c ? C_min : Y_min;
  const uint8_t I_max = c ? C_max : Y_max;

  int flush = 0;
  do
  {
    if( x > 0 )
    {
      if( !flush )
      {
        // Horizontal deblock (across previous block)
        __m128i vgrain;
        __m128i vfac = _mm_set_epi16( 0, 0, 0, 1, 1, 3, 1, 1 );
        if( c == 0 )
        {
          vgrain = _mm_loadl_epi64( (__m128i*) &grain[0][16 - 2] );   // r1 r0 l0 l1
        }
        else
        {
          vgrain = _mm_loadl_epi64( (__m128i*) &grain[c][8 - 2] );   // r1 r0 l0 l1
        }
        __m128i vgrainh = _mm_mullo_epi16( vgrain, vfac );           // r1 3*r0  l0 l1
        vgrainh         = _mm_srli_si128( vgrainh, 2 );              //     r1 3+r0 l0
        vfac            = _mm_srli_si128( vfac, 2 );
        __m128i vgrainl = _mm_mullo_epi16( vgrain, vfac );           // r1 r0 3*lo l1
        vgrainl         = _mm_slli_si128( vgrainl, 10 );
        vgrainl         = _mm_srli_si128( vgrainl, 10 );             //    r0 3*lo l1
        vgrainl         = _mm_hadd_epi16( vgrainl, vgrainl );        // r0 3*lo+l1
        vgrainl         = _mm_hadd_epi16( vgrainl, vgrainl );        // r0+3*lo+l1
        vgrainh         = _mm_hadd_epi16( vgrainh, vgrainh );
        vgrainh         = _mm_hadd_epi16( vgrainh, vgrainh );
        vgrainh         = _mm_srli_si128( vgrainh, 2 );
        vgrain          = _mm_or_si128( vgrainl, vgrainh );
        vgrain          = _mm_add_epi16( vgrain, _mm_set_epi16( 0, 0, 0, 0, 0, 0, 2, 2 ) );
        vgrain          = _mm_srai_epi16( vgrain, 2 );
        if( c == 0 )
        {
          _mm_storeu_si32( (__m128i*) &grain[0][16 - 1], vgrain );
        }
        else
        {
          _mm_storeu_si32( (__m128i*) &grain[c][8 - 1], vgrain );
        }
      }
      if( bs )
      {
#  ifdef USE_AVX2
        __m128i vshift = _mm_set_epi16( 0, 0, 0, 0, 0, 0, 0, scale_shift );
        if( c == 0 )
        {
          __m256i vadd    = _mm256_set1_epi32( 1 << ( scale_shift - 1 ) );
          __m256i vgrain  = _mm256_lddqu_si256( (__m256i*) &grain[0][0] );   // load 16 * 16 bit
          __m256i vscale  = _mm256_cvtepi8_epi16( _mm_lddqu_si128( (__m128i*) &scale[0][0] ) );
          __m256i tmplo   = _mm256_mullo_epi16( vscale, vgrain );
          __m256i tmphi   = _mm256_mulhi_epi16( vscale, vgrain );
          __m256i tmpgvlo = _mm256_unpacklo_epi16( tmplo, tmphi );   // 32 bit
          __m256i tmpgvhi = _mm256_unpackhi_epi16( tmplo, tmphi );
          // deinterleave
          __m256i gvlo = _mm256_permute2x128_si256( tmpgvlo, tmpgvhi, 0x20 );
          __m256i gvhi = _mm256_permute2x128_si256( tmpgvlo, tmpgvhi, 0x31 );
          // round
          gvlo           = _mm256_add_epi32( gvlo, vadd );
          gvhi           = _mm256_add_epi32( gvhi, vadd );
          gvlo           = _mm256_sra_epi32( gvlo, vshift );
          gvhi           = _mm256_sra_epi32( gvhi, vshift );
          __m256i vI16lo = _mm256_cvtepi16_epi32( _mm_lddqu_si128( (__m128i*) &I16[( x - 16 )] ) );
          __m256i vI16hi = _mm256_cvtepi16_epi32( _mm_lddqu_si128( (__m128i*) &I16[( x - 16 ) + 8] ) );
          vI16lo         = _mm256_add_epi32( gvlo, vI16lo );
          vI16hi         = _mm256_add_epi32( gvhi, vI16hi );
          vI16lo         = _mm256_max_epi32( _mm256_set1_epi32( I_min ), vI16lo );
          vI16hi         = _mm256_max_epi32( _mm256_set1_epi32( I_min ), vI16hi );
          vI16lo         = _mm256_min_epi32( _mm256_set1_epi32( I_max << bs ), vI16lo );
          vI16hi         = _mm256_min_epi32( _mm256_set1_epi32( I_max << bs ), vI16hi );
          vI16lo         = _mm256_packs_epi32( vI16lo, vI16hi );
          vI16lo         = _mm256_permute4x64_epi64( vI16lo, 0xd8 );
          _mm256_storeu_si256( (__m256i*) &I16[( x - 16 )], vI16lo );
        }
        else
        {
          __m128i vadd   = _mm_set1_epi32( 1 << ( scale_shift - 1 ) );
          __m128i vscale = _mm_lddqu_si128( (__m128i*) &scale[c] );
          __m128i vgrain = _mm_lddqu_si128( (__m128i*) &grain[c] );
          vscale         = _mm_cvtepi8_epi16( vscale );   // 16 bit
          __m128i tmplo  = _mm_mullo_epi16( vscale, vgrain );
          __m128i tmphi  = _mm_mulhi_epi16( vscale, vgrain );
          __m128i gvlo   = _mm_unpacklo_epi16( tmplo, tmphi );   // 32 bit
          __m128i gvhi   = _mm_unpackhi_epi16( tmplo, tmphi );
          gvlo           = _mm_add_epi32( gvlo, vadd );
          gvhi           = _mm_add_epi32( gvhi, vadd );
          gvlo           = _mm_sra_epi32( gvlo, vshift );
          gvhi           = _mm_sra_epi32( gvhi, vshift );
          __m128i vI16lo = _mm_lddqu_si128( (__m128i*) &I16[( x - 16 ) / subx] );
          __m128i vI16hi = _mm_lddqu_si128( (__m128i*) &I16[( x - 16 ) / subx + 4] );
          vI16lo         = _mm_cvtepi16_epi32( vI16lo );   // 32 bit
          vI16hi         = _mm_cvtepi16_epi32( vI16hi );
          vI16lo         = _mm_add_epi32( gvlo, vI16lo );
          vI16hi         = _mm_add_epi32( gvhi, vI16hi );
          vI16lo         = _mm_max_epi32( _mm_set1_epi32( I_min ), vI16lo );
          vI16hi         = _mm_max_epi32( _mm_set1_epi32( I_min ), vI16hi );
          vI16lo         = _mm_min_epi32( _mm_set1_epi32( I_max << bs ), vI16lo );
          vI16hi         = _mm_min_epi32( _mm_set1_epi32( I_max << bs ), vI16hi );
          vI16lo         = _mm_packs_epi32( vI16lo, vI16hi );
          _mm_storeu_si128( (__m128i*) &I16[( x - 16 ) / subx], vI16lo );
        }
#  else    // !USE_AVX2
        __m128i vadd   = _mm_set1_epi32( 1 << ( scale_shift - 1 ) );
        __m128i vshift = _mm_set_epi16( 0, 0, 0, 0, 0, 0, 0, scale_shift );
        __m128i vscale = _mm_lddqu_si128( (__m128i*) &scale[c] );
        __m128i vgrain = _mm_lddqu_si128( (__m128i*) &grain[c] );
        vscale         = _mm_cvtepi8_epi16( vscale );   // 16 bit
        __m128i tmplo  = _mm_mullo_epi16( vscale, vgrain );
        __m128i tmphi  = _mm_mulhi_epi16( vscale, vgrain );
        __m128i gvlo   = _mm_unpacklo_epi16( tmplo, tmphi );   // 32 bit
        __m128i gvhi   = _mm_unpackhi_epi16( tmplo, tmphi );
        gvlo           = _mm_add_epi32( gvlo, vadd );
        gvhi           = _mm_add_epi32( gvhi, vadd );
        gvlo           = _mm_sra_epi32( gvlo, vshift );
        gvhi           = _mm_sra_epi32( gvhi, vshift );
        __m128i vI16lo = _mm_lddqu_si128( (__m128i*) &I16[( x - 16 ) / subx] );
        __m128i vI16hi = _mm_lddqu_si128( (__m128i*) &I16[( x - 16 ) / subx + 4] );

        vI16lo = _mm_cvtepi16_epi32( vI16lo );   // 32 bit
        vI16hi = _mm_cvtepi16_epi32( vI16hi );
        vI16lo = _mm_add_epi32( gvlo, vI16lo );
        vI16hi = _mm_add_epi32( gvhi, vI16hi );
        vI16lo = _mm_max_epi32( _mm_set1_epi32( I_min ), vI16lo );
        vI16hi = _mm_max_epi32( _mm_set1_epi32( I_min ), vI16hi );
        vI16lo = _mm_min_epi32( _mm_set1_epi32( I_max << bs ), vI16lo );
        vI16hi = _mm_min_epi32( _mm_set1_epi32( I_max << bs ), vI16hi );
        vI16lo = _mm_packs_epi32( vI16lo, vI16hi );
        _mm_storeu_si128( (__m128i*) &I16[( x - 16 ) / subx], vI16lo );
        if( c == 0 )
        {
          __m128i vscale = _mm_lddqu_si128( (__m128i*) &scale[c][8] );
          __m128i vgrain = _mm_lddqu_si128( (__m128i*) &grain[c][8] );
          vscale         = _mm_cvtepi8_epi16( vscale );   // 16 bit
          __m128i tmplo  = _mm_mullo_epi16( vscale, vgrain );
          __m128i tmphi  = _mm_mulhi_epi16( vscale, vgrain );
          __m128i gvlo   = _mm_unpacklo_epi16( tmplo, tmphi );   // 32 bit
          __m128i gvhi   = _mm_unpackhi_epi16( tmplo, tmphi );
          // round
          gvlo           = _mm_add_epi32( gvlo, vadd );
          gvhi           = _mm_add_epi32( gvhi, vadd );
          gvlo           = _mm_sra_epi32( gvlo, vshift );
          gvhi           = _mm_sra_epi32( gvhi, vshift );
          __m128i vI16lo = _mm_lddqu_si128( (__m128i*) &I16[( x - 16 ) / subx + 8] );
          __m128i vI16hi = _mm_lddqu_si128( (__m128i*) &I16[( x - 16 ) / subx + 12] );
          vI16lo         = _mm_cvtepi16_epi32( vI16lo );   // 32 bit
          vI16hi         = _mm_cvtepi16_epi32( vI16hi );
          vI16lo         = _mm_add_epi32( gvlo, vI16lo );
          vI16hi         = _mm_add_epi32( gvhi, vI16hi );
          vI16lo         = _mm_max_epi32( _mm_set1_epi32( I_min ), vI16lo );
          vI16hi         = _mm_max_epi32( _mm_set1_epi32( I_min ), vI16hi );
          vI16lo         = _mm_min_epi32( _mm_set1_epi32( I_max << bs ), vI16lo );
          vI16hi         = _mm_min_epi32( _mm_set1_epi32( I_max << bs ), vI16hi );
          vI16lo         = _mm_packs_epi32( vI16lo, vI16hi );
          _mm_storeu_si128( (__m128i*) &I16[( x - 16 ) / subx + 8], vI16lo );
        }
#endif   // !USE_AVX2
      }    // bs
      else
      {
        for( int i = 0; i < 16 / subx; i++ )
        {
          // Output previous block (or flush current)
          int32_t g = round( scale[c][i] * (int16_t) grain[c][i], scale_shift );
          if( bs )
          {
            I16[( x - 16 ) / subx + i] = std::max<int32_t>( I_min << bs, std::min<int32_t>( I_max << bs, I16[( x - 16 ) / subx + i] + g ) );
          }
          else
          {
            I8[( x - 16 ) / subx + i] = std::max<int32_t>( I_min, std::min<int32_t>( I_max, I8[( x - 16 ) / subx + i] + g ) );
          }
        }
      }
    }
    // Shift pipeline
    if( !flush )
    {
      if( c == 0 )
      {
#ifdef USE_AVX2
        __m256i vgrain = _mm256_lddqu_si256( (__m256i*) &grain[0][16] );
        _mm256_storeu_si256( (__m256i*) &grain[0][0], vgrain );
#else
        __m128i vgrain0 = _mm_lddqu_si128( (__m128i*) &grain[0][16] );
        __m128i vgrain1 = _mm_lddqu_si128( (__m128i*) &grain[0][24] );
        _mm_storeu_si128( (__m128i*) &grain[0][0], vgrain0 );
        _mm_storeu_si128( (__m128i*) &grain[0][8], vgrain1 );
#endif
        __m128i vscale = _mm_lddqu_si128( (__m128i*) &scale[0][16] );
        _mm_storeu_si128( (__m128i*) &scale[0][0], vscale );
      }
      else
      {
        __m128i vgrain = _mm_lddqu_si128( (__m128i*) &grain[c][8] );
        __m128i vscale = _mm_loadl_epi64( (__m128i*) &scale[c][8] );
        _mm_storeu_si128( (__m128i*) &grain[c][0], vgrain );
        _mm_storel_epi64( (__m128i*) &scale[c][0], vscale );
      }
    }
    if( x + 16 >= width )
    {
      flush++;
      x += 16;
    }
  } while( flush == 1 );
}

}   // namespace vvdec

#endif   // TARGET_SIMD_X86
