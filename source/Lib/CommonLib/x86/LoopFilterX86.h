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

/** \file     LoopFilterX86.h
    \brief    deblocking filter simd code
*/

#include "CommonDefX86.h"
#include "../Rom.h"
#include "../LoopFilter.h"

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_DBLF

namespace vvdec
{

template<X86_VEXT vext>
static inline void xPelLumaCore( __m128i& m0, __m128i& m1, __m128i& m2, __m128i& m3, __m128i& m4, __m128i& m5, __m128i& m6, __m128i m7, const int tc )
{
  const __m128i m1234 = _mm_add_epi16( _mm_add_epi16( m1, m2 ), _mm_add_epi16( m3, m4 ) );
  const __m128i m3456 = _mm_add_epi16( _mm_add_epi16( m5, m6 ), _mm_add_epi16( m3, m4 ) );

  const __m128i r1    = _mm_add_epi16( _mm_add_epi16( m1234, _mm_set1_epi16( 4 ) ), _mm_slli_epi16( _mm_add_epi16( m0, m1 ), 1 ) );
  const __m128i r2    = _mm_add_epi16( m1234, _mm_set1_epi16( 2 ) );
  const __m128i r3    = _mm_add_epi16( _mm_add_epi16( _mm_slli_epi16( m1234, 1 ), m5 ), _mm_sub_epi16( _mm_set1_epi16( 4 ), m1 ) );
  const __m128i r4    = _mm_add_epi16( _mm_add_epi16( _mm_slli_epi16( m3456, 1 ), m2 ), _mm_sub_epi16( _mm_set1_epi16( 4 ), m6 ) );
  const __m128i r5    = _mm_add_epi16( m3456, _mm_set1_epi16( 2 ) );
  const __m128i r6    = _mm_add_epi16( _mm_add_epi16( m3456, _mm_set1_epi16( 4 ) ), _mm_slli_epi16( _mm_add_epi16( m6, m7 ), 1 ) );

  const char tc3[3]          = { 3, 2, 1 };

  const __m128i vtc0  = _mm_set1_epi16   ( tc3[0] * tc );
  const __m128i vtc1  = _mm_set1_epi16   ( tc3[1] * tc );
  const __m128i vtc2  = _mm_set1_epi16   ( tc3[2] * tc );
  const __m128i vzero = _mm_setzero_si128();

  __m128i vmax  = vtc1;
  __m128i vmin  = _mm_sub_epi16    ( vzero, vmax );

  __m128i org   = _mm_unpacklo_epi64( m5,    m2 );
  __m128i vec   = _mm_unpacklo_epi64( r5,    r2 );
  vec           = _mm_srli_epi16   ( vec,   2 );
  vec           = _mm_sub_epi16    ( vec,   org );
  vec           = _mm_min_epi16    ( vmax,  _mm_max_epi16( vmin, vec ) );
  vec           = _mm_add_epi16    ( vec,   org );
  m5            = vec;
  m2            = _mm_unpackhi_epi64( vec,  vec );

  vmax          = _mm_blend_epi16  ( vtc0, vtc2, 0xf0 );
  vmin          = _mm_sub_epi16    ( vzero, vmax );
  org           = _mm_unpacklo_epi64( m3,    m1 );
  vec           = _mm_unpacklo_epi64( r3,    r1 );
  vec           = _mm_srli_epi16   ( vec,   3 );
  vec           = _mm_sub_epi16    ( vec,   org );
  vec           = _mm_min_epi16    ( vmax,  _mm_max_epi16( vmin, vec ) );
  vec           = _mm_add_epi16    ( vec,   org );
  m3            = vec;
  m1            = _mm_unpackhi_epi64( vec,  vec );
  
  vmax          = _mm_blend_epi16  ( vtc2, vtc0, 0xf0 );
  vmin          = _mm_sub_epi16    ( vzero, vmax );
  org           = _mm_unpacklo_epi64( m6,    m4 );
  vec           = _mm_unpacklo_epi64( r6,    r4 );
  vec           = _mm_srli_epi16   ( vec,   3 );
  vec           = _mm_sub_epi16    ( vec,   org );
  vec           = _mm_min_epi16    ( vmax,  _mm_max_epi16( vmin, vec ) );
  vec           = _mm_add_epi16    ( vec,   org );
  m6            = vec;
  m4            = _mm_unpackhi_epi64( vec,  vec );
}

template<X86_VEXT vext>
static inline void xPelFilterLumaLoopHor( Pel* piSrc, const ptrdiff_t offset, const int tc )
{
  __m128i m0 = _mm_loadu_si64( ( const __m128i* ) &piSrc[-4 * offset] );
  __m128i m1 = _mm_loadu_si64( ( const __m128i* ) &piSrc[-3 * offset] );
  __m128i m2 = _mm_loadu_si64( ( const __m128i* ) &piSrc[-2 * offset] );
  __m128i m3 = _mm_loadu_si64( ( const __m128i* ) &piSrc[-1 * offset] );
  __m128i m4 = _mm_loadu_si64( ( const __m128i* ) &piSrc[ 0         ] );
  __m128i m5 = _mm_loadu_si64( ( const __m128i* ) &piSrc[ 1 * offset] );
  __m128i m6 = _mm_loadu_si64( ( const __m128i* ) &piSrc[ 2 * offset] );
  __m128i m7 = _mm_loadu_si64( ( const __m128i* ) &piSrc[ 3 * offset] );

  xPelLumaCore<vext>( m0, m1, m2, m3, m4, m5, m6, m7, tc );
  
  _mm_storeu_si64( ( __m128i* ) &piSrc[-3 * offset], m1 );
  _mm_storeu_si64( ( __m128i* ) &piSrc[-2 * offset], m2 );
  _mm_storeu_si64( ( __m128i* ) &piSrc[-1 * offset], m3 );
  _mm_storeu_si64( ( __m128i* ) &piSrc[ 0         ], m4 );
  _mm_storeu_si64( ( __m128i* ) &piSrc[ 1 * offset], m5 );
  _mm_storeu_si64( ( __m128i* ) &piSrc[ 2 * offset], m6 );
}

template<X86_VEXT vext>
static inline void xPelFilterLumaLoopVer( Pel* piSrc, const ptrdiff_t step, const int tc )
{
  //// Transpose the 8x4 matrix:  a0 a4 (ax in N^(1,4)) into 4x8: [m0 m1 m2 m3 m4 m5 m6 m7] (mx in N^(4,1))
  ////                            a1 a5
  ////                            a2 a6
  ////                            a3 a7

  __m128i va01      = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 0 * step] );
  __m128i va23      = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 1 * step] );
  __m128i va45      = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 2 * step] );
  __m128i va67      = _mm_loadu_si128( ( const __m128i* ) &piSrc[-4 + 3 * step] );

  __m128i va01a23lo = _mm_unpacklo_epi16( va01, va23 );
  __m128i va01a23hi = _mm_unpackhi_epi16( va01, va23 );
  __m128i va45a67lo = _mm_unpacklo_epi16( va45, va67 );
  __m128i va45a67hi = _mm_unpackhi_epi16( va45, va67 );

  va01 = _mm_unpacklo_epi32( va01a23lo, va45a67lo );
  va23 = _mm_unpackhi_epi32( va01a23lo, va45a67lo );
  va45 = _mm_unpacklo_epi32( va01a23hi, va45a67hi );
  va67 = _mm_unpackhi_epi32( va01a23hi, va45a67hi );

  __m128i m0 = va01;
  __m128i m1 = _mm_unpackhi_epi64( va01, va01 );
  __m128i m2 = va23;
  __m128i m3 = _mm_unpackhi_epi64( va23, va23 );
  __m128i m4 = va45;
  __m128i m5 = _mm_unpackhi_epi64( va45, va45 );
  __m128i m6 = va67;
  __m128i m7 = _mm_unpackhi_epi64( va67, va67 );

  // do the loop filter of the the 4x8 matrix
  xPelLumaCore<vext>( m0, m1, m2, m3, m4, m5, m6, m7, tc );

  // Transpose back
  va01 = _mm_unpacklo_epi64( m0, m4 );
  va23 = _mm_unpacklo_epi64( m1, m5 );
  va45 = _mm_unpacklo_epi64( m2, m6 );
  va67 = _mm_unpacklo_epi64( m3, m7 );

  va01a23lo = _mm_unpacklo_epi16( va01, va23 );
  va01a23hi = _mm_unpackhi_epi16( va01, va23 );
  va45a67lo = _mm_unpacklo_epi16( va45, va67 );
  va45a67hi = _mm_unpackhi_epi16( va45, va67 );

  va01 = _mm_unpacklo_epi32( va01a23lo, va45a67lo );
  va23 = _mm_unpackhi_epi32( va01a23lo, va45a67lo );
  va45 = _mm_unpacklo_epi32( va01a23hi, va45a67hi );
  va67 = _mm_unpackhi_epi32( va01a23hi, va45a67hi );

  _mm_storeu_si128( ( __m128i* ) &piSrc[-4 + 0 * step], _mm_unpacklo_epi64( va01, va45 ) );
  _mm_storeu_si128( ( __m128i* ) &piSrc[-4 + 1 * step], _mm_unpackhi_epi64( va01, va45 ) );
  _mm_storeu_si128( ( __m128i* ) &piSrc[-4 + 2 * step], _mm_unpacklo_epi64( va23, va67 ) );
  _mm_storeu_si128( ( __m128i* ) &piSrc[-4 + 3 * step], _mm_unpackhi_epi64( va23, va67 ) );
}

template<X86_VEXT vext>
static inline void xPelFilterLumaWeakCore( __m128i &vm1, __m128i &vm2, __m128i &vm3, __m128i &vm4, __m128i &vm5, __m128i &vm6, const int tc, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng )
{
  __m128i vmin = _mm_set1_epi16( clpRng.min() );
  __m128i vmax = _mm_set1_epi16( clpRng.max() );

  __m128i vtmp0 = _mm_sub_epi16( vm4, vm3 );
  __m128i vtmp1 = _mm_sub_epi16( vm5, vm2 );
  __m128i vm3s  = vm3;
  __m128i vm4s  = vm4;

  vtmp0 = _mm_unpacklo_epi16( vtmp0, vtmp1 );
  vtmp1 = _mm_set1_epi32( 0xfffd0009 );

  vtmp0 = _mm_madd_epi16( vtmp0, vtmp1 );
  vtmp1 = _mm_add_epi32( vtmp0, _mm_set1_epi32( 8 ) );
  __m128i vdlt = _mm_srai_epi32( vtmp1, 4 );
  vdlt = _mm_packs_epi32( vdlt, _mm_setzero_si128() );

  short deltaV[4] = { 0, 0, 0, 0 };
  _mm_storeu_si64( ( __m128i* ) deltaV, vdlt );

  __m128i vmsk = _mm_cmpgt_epi16( _mm_set1_epi16( iThrCut ), _mm_abs_epi16( vdlt ) );
  __m128i vtc  = _mm_set1_epi16( -tc );
  vdlt = _mm_max_epi16( vdlt, vtc );
  vtc  = _mm_set1_epi16( tc );
  vdlt = _mm_min_epi16( vdlt, vtc );
  vtc  = _mm_set1_epi16( tc >> 1 );
  __m128i vtcn = _mm_set1_epi16( -( tc >> 1 ) );

  vtmp0 = _mm_add_epi16( vm3, vdlt );
  vtmp1 = _mm_min_epi16( _mm_max_epi16( vmin, vtmp0 ), vmax );
  vtmp0 = _mm_blendv_epi8( vm3, vtmp1, vmsk );
  vm3   = _mm_unpacklo_epi64( vtmp0, _mm_setzero_si128() );
  //_mm_storeu_si64( ( __m128i* ) &m3, vtmp0 );

  if( bFilterSecondP )
  {
    vtmp0 = _mm_srli_epi16( _mm_add_epi16( _mm_add_epi16( vm1, vm3s ), _mm_set1_epi16( 1 ) ), 1 );
    vtmp1 = _mm_srai_epi16( _mm_add_epi16( _mm_sub_epi16( vtmp0, vm2 ), vdlt ), 1 );
    vtmp0 = _mm_min_epi16( _mm_max_epi16( vtmp1, vtcn ), vtc );

    vtmp1 = _mm_add_epi16( vm2, vtmp0 );
    vtmp0 = _mm_min_epi16( _mm_max_epi16( vmin, vtmp1 ), vmax );
    vtmp1 = _mm_blendv_epi8( vm2, vtmp0, vmsk );
    vm2   = _mm_unpacklo_epi64( vtmp1, _mm_setzero_si128() );
    //_mm_storeu_si64( ( __m128i* ) &m2, vtmp1 );
  }

  vtmp0 = _mm_sub_epi16( vm4, vdlt );
  vtmp1 = _mm_min_epi16( _mm_max_epi16( vmin, vtmp0 ), vmax );
  vtmp0 = _mm_blendv_epi8( vm4, vtmp1, vmsk );
  vm4   = _mm_unpacklo_epi64( vtmp0, _mm_setzero_si128() );
  //_mm_storeu_si64( ( __m128i* ) &m4, vtmp0 );

  if( bFilterSecondQ )
  {
    vtmp0 = _mm_srli_epi16( _mm_add_epi16( _mm_add_epi16( vm6, vm4s ), _mm_set1_epi16( 1 ) ), 1 );
    vtmp1 = _mm_srai_epi16( _mm_sub_epi16( _mm_sub_epi16( vtmp0, vm5 ), vdlt ), 1 );
    vtmp0 = _mm_min_epi16( _mm_max_epi16( vtmp1, vtcn ), vtc );
        
    vtmp1 = _mm_add_epi16( vm5, vtmp0 );
    vtmp0 = _mm_min_epi16( _mm_max_epi16( vmin, vtmp1 ), vmax );
    vtmp1 = _mm_blendv_epi8( vm5, vtmp0, vmsk );
    vm5   = _mm_unpacklo_epi64( vtmp1, _mm_setzero_si128() );
    //_mm_storeu_si64( ( __m128i* ) &m5, vtmp1 );
  }
}

template<X86_VEXT vext>
NO_THREAD_SANITIZE static void xPelFilterLumaX86( Pel* piSrc, const ptrdiff_t step, const ptrdiff_t offset, const int tc, const bool sw, const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ, const ClpRng& clpRng )
{
  if( sw )
  {
    if( offset == 1 )
    {
      xPelFilterLumaLoopVer<vext>( piSrc, step,   tc );
    }
    else
    {
      xPelFilterLumaLoopHor<vext>( piSrc, offset, tc );
    }
  }
  else
  {
    if( offset == 1 )
    {
      //// Transpose the 8x4 matrix:  a0 a4 (ax in N^(1,4)) into 4x8: [m0 m1 m2 m3 m4 m5 m6 m7] (mx in N^(4,1))
      ////                            a1 a5
      ////                            a2 a6
      ////                            a3 a7

      __m128i va01 = _mm_unpacklo_epi64( _mm_loadu_si64( ( const __m128i* ) &piSrc[-3 + 0 * step] ), _mm_setr_epi16( piSrc[1 + 0 * step], piSrc[2 + 0 * step], 0, 0, 0, 0, 0, 0 ) );
      __m128i va23 = _mm_unpacklo_epi64( _mm_loadu_si64( ( const __m128i* ) &piSrc[-3 + 1 * step] ), _mm_setr_epi16( piSrc[1 + 1 * step], piSrc[2 + 1 * step], 0, 0, 0, 0, 0, 0 ) );
      __m128i va45 = _mm_unpacklo_epi64( _mm_loadu_si64( ( const __m128i* ) &piSrc[-3 + 2 * step] ), _mm_setr_epi16( piSrc[1 + 2 * step], piSrc[2 + 2 * step], 0, 0, 0, 0, 0, 0 ) );
      __m128i va67 = _mm_unpacklo_epi64( _mm_loadu_si64( ( const __m128i* ) &piSrc[-3 + 3 * step] ), _mm_setr_epi16( piSrc[1 + 3 * step], piSrc[2 + 3 * step], 0, 0, 0, 0, 0, 0 ) );

      va01 = _mm_bslli_si128( va01, 2 );
      va23 = _mm_bslli_si128( va23, 2 );
      va45 = _mm_bslli_si128( va45, 2 );
      va67 = _mm_bslli_si128( va67, 2 );

      __m128i va01a23lo = _mm_unpacklo_epi16( va01, va23 );
      __m128i va01a23hi = _mm_unpackhi_epi16( va01, va23 );
      __m128i va45a67lo = _mm_unpacklo_epi16( va45, va67 );
      __m128i va45a67hi = _mm_unpackhi_epi16( va45, va67 );

      va01 = _mm_unpacklo_epi32( va01a23lo, va45a67lo );
      va23 = _mm_unpackhi_epi32( va01a23lo, va45a67lo );
      va45 = _mm_unpacklo_epi32( va01a23hi, va45a67hi );
      va67 = _mm_unpackhi_epi32( va01a23hi, va45a67hi );

      __m128i vzr = _mm_setzero_si128();
      __m128i vm1 = _mm_unpackhi_epi64( va01, vzr );
      __m128i vm2 = _mm_unpacklo_epi64( va23, vzr );
      __m128i vm3 = _mm_unpackhi_epi64( va23, vzr );
      __m128i vm4 = _mm_unpacklo_epi64( va45, vzr );
      __m128i vm5 = _mm_unpackhi_epi64( va45, vzr );
      __m128i vm6 = _mm_unpacklo_epi64( va67, vzr );

      // do the loop filter of the the 4x8 matrix
      xPelFilterLumaWeakCore<vext>( vm1, vm2, vm3, vm4, vm5, vm6,
                                    tc, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );

      va01a23lo = _mm_unpacklo_epi16( vm2, vm3 );
      va01a23hi = _mm_unpackhi_epi16( vm2, vm3 );
      va45a67lo = _mm_unpacklo_epi16( vm4, vm5 );
      va45a67hi = _mm_unpackhi_epi16( vm4, vm5 );

      va01 = _mm_unpacklo_epi32( va01a23lo, va45a67lo );
      va23 = _mm_unpackhi_epi32( va01a23lo, va45a67lo );

      _mm_storeu_si64( ( __m128i* ) &piSrc[-2 + 0 * step], _mm_unpacklo_epi64( va01, vzr ) );
      _mm_storeu_si64( ( __m128i* ) &piSrc[-2 + 1 * step], _mm_unpackhi_epi64( va01, vzr ) );
      _mm_storeu_si64( ( __m128i* ) &piSrc[-2 + 2 * step], _mm_unpacklo_epi64( va23, vzr ) );
      _mm_storeu_si64( ( __m128i* ) &piSrc[-2 + 3 * step], _mm_unpackhi_epi64( va23, vzr ) );
    }
    else
    {
      __m128i vm1 = _mm_loadu_si64( ( const __m128i * ) &piSrc[-3 * offset] );
      __m128i vm2 = _mm_loadu_si64( ( const __m128i * ) &piSrc[-2 * offset] );
      __m128i vm3 = _mm_loadu_si64( ( const __m128i * ) &piSrc[-1 * offset] );
      __m128i vm4 = _mm_loadu_si64( ( const __m128i * ) &piSrc[ 0 * offset] );
      __m128i vm5 = _mm_loadu_si64( ( const __m128i * ) &piSrc[ 1 * offset] );
      __m128i vm6 = _mm_loadu_si64( ( const __m128i * ) &piSrc[ 2 * offset] );

      xPelFilterLumaWeakCore<vext>( vm1, vm2, vm3, vm4, vm5, vm6,
                                    tc, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );

      if( bFilterSecondP ) 
        _mm_storeu_si64( ( __m128i * ) &piSrc[-2 * offset], vm2 );
      _mm_storeu_si64  ( ( __m128i * ) &piSrc[-1 * offset], vm3 );
      _mm_storeu_si64  ( ( __m128i * ) &piSrc[ 0 * offset], vm4 );
      if( bFilterSecondQ )
        _mm_storeu_si64( ( __m128i * ) &piSrc[ 1 * offset], vm5 );
    }
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

static constexpr short  dbCoeffs7[8] = { 59, 50, 41, 32, 23, 14,  5,  0 };
static constexpr short  dbCoeffs5[8] = { 58, 45, 32, 19,  6,  0,  0,  0 };
static constexpr short  dbCoeffs3[8] = { 53, 32, 11,  0,  0,  0,  0,  0 };
static constexpr short  tc7[8]       = { 6, 5, 4, 3, 2, 1, 1, 0 };
static constexpr short  tc5[8]       = { 6, 5, 4, 3, 2, 0, 0, 0 };
static constexpr short  tc3[8]       = { 6, 4, 2, 0, 0, 0, 0, 0 };

template<X86_VEXT vext>
static inline void xFilteringPandQX86Hor( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc )
{
  CHECKD( step != 1, "Offset has to be '1' for vertical edge filtering!" );

  const short* dbCoeffsP  = numberPSide == 7 ? dbCoeffs7 : ( numberPSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const short* dbCoeffsQ  = numberQSide == 7 ? dbCoeffs7 : ( numberQSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const short* tcP        = numberPSide == 7 ? tc7 : ( numberPSide == 5 ) ? tc5 : tc3;
  const short* tcQ        = numberQSide == 7 ? tc7 : ( numberQSide == 5 ) ? tc5 : tc3;

  __m128i refP, refQ, refM;

  {
    Pel *srcP = src - offset;
    Pel *srcQ = src;

    __m128i srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcP[-  numberPSide       * offset] );
    __m128i srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcP[-( numberPSide - 1 ) * offset] );

    refP = _mm_add_epi16( srcp1, srcp2 );
    refP = _mm_add_epi16( refP, _mm_set1_epi16( 1 ) );
    refP = _mm_srli_epi16( refP, 1 );

    __m128i srcq1 = _mm_loadu_si64( ( const __m128i * ) &srcQ[  numberQSide       * offset] );
    __m128i srcq2 = _mm_loadu_si64( ( const __m128i * ) &srcQ[( numberQSide - 1 ) * offset] );

    refQ = _mm_add_epi16( srcq1, srcq2 );
    refQ = _mm_add_epi16( refQ, _mm_set1_epi16( 1 ) );
    refQ = _mm_srli_epi16( refQ, 1 );

    __m128i srcp0 = _mm_loadu_si64( ( const __m128i * ) srcP );
    __m128i srcq0 = _mm_loadu_si64( ( const __m128i * ) srcQ );

    if( numberPSide == numberQSide )
    {
      refM = _mm_add_epi16( srcp0, srcq0 );
      if( numberQSide == 7 ) refM = _mm_slli_epi16( refM, 1 );
      
      srcp0 = _mm_loadu_si64( ( const __m128i * ) &srcP[-1 * offset] );
      srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcP[-2 * offset] );
      srcq0 = _mm_loadu_si64( ( const __m128i * ) &srcQ[ 1 * offset] );
      srcq1 = _mm_loadu_si64( ( const __m128i * ) &srcQ[ 2 * offset] );

      refM = _mm_add_epi16( refM, _mm_add_epi16( srcp0, srcq0 ) );
      refM = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcq1 ) );
      if( numberQSide == 5 ) refM = _mm_slli_epi16( refM, 1 );
      refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp2, srcq2 ) );
      srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcP[-3 * offset] );
      srcq2 = _mm_loadu_si64( ( const __m128i * ) &srcQ[ 3 * offset] );
      refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp2, srcq2 ) );

      if( numberPSide == 7 )
      {
        srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcP[-4 * offset] );
        srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcP[-5 * offset] );
        srcq1 = _mm_loadu_si64( ( const __m128i * ) &srcQ[ 4 * offset] );
        srcq2 = _mm_loadu_si64( ( const __m128i * ) &srcQ[ 5 * offset] );

        refM = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcq1 ) );
        refM = _mm_add_epi16( refM, _mm_add_epi16( srcp2, srcq2 ) );
      }

      refM = _mm_add_epi16( refM, _mm_set1_epi16( 8 ) );
      refM = _mm_srli_epi16( refM, 4 );
    }
    else
    {
      Pel *srcPt = srcP;
      Pel *srcQt = srcQ;

      int newNumberQSide = numberQSide;
      int newNumberPSide = numberPSide;
      ptrdiff_t offsett = offset;

      if( numberQSide > numberPSide )
      {
        std::swap( srcPt, srcQt );
        newNumberQSide = numberPSide;
        newNumberPSide = numberQSide;
        offsett = -offset;
        srcp1 = srcp0;
        srcp0 = srcq0;
        srcq0 = srcp1;
      }

      if( newNumberPSide == 5 )
      {
        refM = _mm_add_epi16( srcp0, srcq0 );

        srcp0 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-1 * offsett] );
        srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-2 * offsett] );
        srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-3 * offsett] );
        srcq0 = _mm_loadu_si64( ( const __m128i * ) &srcQt[ 1 * offsett] );
        srcq1 = _mm_loadu_si64( ( const __m128i * ) &srcQt[ 2 * offsett] );
        srcq2 = _mm_loadu_si64( ( const __m128i * ) &srcQt[ 3 * offsett] );

        refM = _mm_add_epi16( refM, _mm_add_epi16( srcp0, srcq0 ) );
        refM = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcq1 ) );
        refM = _mm_add_epi16( refM, _mm_add_epi16( srcp2, srcq2 ) );

        refM = _mm_add_epi16( refM, _mm_set1_epi16( 4 ) );
        refM = _mm_srli_epi16( refM, 3 );
      }
      else if( newNumberQSide == 3 )
      {
        refM  = _mm_slli_epi16( _mm_add_epi16( srcq0, srcp0 ), 1 );
        refM  = _mm_add_epi16( refM, srcq0 );

        srcq1 = _mm_loadu_si64( ( const __m128i * ) &srcQt[1 * offsett] );
        srcq2 = _mm_loadu_si64( ( const __m128i * ) &srcQt[2 * offsett] );

        refM  = _mm_add_epi16( refM, _mm_slli_epi16( _mm_add_epi16( srcq1, srcq2 ), 1 ) );
        refM  = _mm_add_epi16( refM, srcq1 );

        srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-1 * offsett] );
        srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-2 * offsett] );

        refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcp2 ) );

        srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-3 * offsett] );
        srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-4 * offsett] );

        refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcp2 ) );

        srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-5 * offsett] );
        srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-6 * offsett] );

        refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcp2 ) );
              
        refM  = _mm_add_epi16( refM, _mm_set1_epi16( 8 ) );
        refM  = _mm_srli_epi16( refM, 4 );
      }
      else
      {
        refM  = _mm_add_epi16( srcp0, srcq0 );
      
        srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-1 * offsett] );
        srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-2 * offsett] );
        srcq1 = _mm_loadu_si64( ( const __m128i * ) &srcQt[ 1 * offsett] );
        srcq2 = _mm_loadu_si64( ( const __m128i * ) &srcQt[ 2 * offsett] );

        refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcq1 ) );
        refM  = _mm_slli_epi16( refM, 1 );
        refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp2, srcq2 ) );
        
        srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-3 * offsett] );
        srcp2 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-4 * offsett] );
        srcq1 = _mm_loadu_si64( ( const __m128i * ) &srcQt[ 3 * offsett] );
        srcq2 = _mm_loadu_si64( ( const __m128i * ) &srcQt[ 4 * offsett] );

        refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcq1 ) );
        refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp2, srcq2 ) );
              
        srcp1 = _mm_loadu_si64( ( const __m128i * ) &srcPt[-5 * offsett] );
        srcq1 = _mm_loadu_si64( ( const __m128i * ) &srcQt[ 5 * offsett] );

        refM  = _mm_add_epi16( refM, _mm_add_epi16( srcp1, srcq1 ) );
              
        refM  = _mm_add_epi16( refM, _mm_set1_epi16( 8 ) );
        refM  = _mm_srli_epi16( refM, 4 );
      }
    }
  }

  Pel* srcP = src - offset;
  Pel* srcQ = src;

  for( int pos = 0; pos < numberPSide; pos++ )
  {
    __m128i vref1 = refP;
    __m128i vref0 = refM;
    __m128i vsrc  = _mm_loadu_si64( ( const __m128i* ) &srcP[-offset * pos] );
    __m128i vmax  = _mm_set1_epi16( ( tc * tcP[pos] ) >> 1 );
    __m128i vmin  = _mm_sub_epi16( vsrc, vmax );
    vmax          = _mm_add_epi16( vsrc, vmax );
    vref0         = _mm_unpacklo_epi16( vref0, vref1 );
    __m128i vtmp  = _mm_set1_epi32( dbCoeffsP[pos] | ( ( 64 - dbCoeffsP[pos] ) << 16 ) );
    vtmp          = _mm_madd_epi16( vref0, vtmp );
    vtmp          = _mm_add_epi32( vtmp, _mm_set1_epi32( 32 ) );
    vtmp          = _mm_srli_epi32( vtmp, 6 );
    vtmp          = _mm_packs_epi32( vtmp, vtmp );
    vtmp          = _mm_min_epi16( _mm_max_epi16( vtmp, vmin ), vmax );
    _mm_storeu_si64( ( __m128i* ) &srcP[-offset * pos], vtmp );
  }

  for( int pos = 0; pos < numberQSide; pos++ )
  {
    __m128i vref1 = refQ;
    __m128i vref0 = refM;
    __m128i vsrc  = _mm_loadu_si64( ( const __m128i* ) &srcQ[offset * pos] );
    __m128i vmax  = _mm_set1_epi16( ( tc * tcQ[pos] ) >> 1 );
    __m128i vmin  = _mm_sub_epi16( vsrc, vmax );
    vmax          = _mm_add_epi16( vsrc, vmax );
    vref0         = _mm_unpacklo_epi16( vref0, vref1 );
    __m128i vtmp  = _mm_set1_epi32( dbCoeffsQ[pos] | ( ( 64 - dbCoeffsQ[pos] ) << 16 ) );
    vtmp          = _mm_madd_epi16( vref0, vtmp );
    vtmp          = _mm_add_epi32( vtmp, _mm_set1_epi32( 32 ) );
    vtmp          = _mm_srli_epi32( vtmp, 6 );
    vtmp          = _mm_packs_epi32( vtmp, vtmp );
    vtmp          = _mm_min_epi16( _mm_max_epi16( vtmp, vmin ), vmax );
    _mm_storeu_si64( ( __m128i* ) &srcQ[offset * pos], vtmp );
  }
}


template<X86_VEXT vext>
static inline void xFilteringPandQX86Ver( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc )
{
  CHECKD( offset != 1, "Offset has to be '1' for vertical edge filtering!" );

  _mm_prefetch( ( const char * ) ( src - numberPSide ), _MM_HINT_T0 );
  _mm_prefetch( ( const char * ) ( src + numberQSide ), _MM_HINT_T0 );

  const short* dbCoeffsP = numberPSide == 7 ? dbCoeffs7 : ( numberPSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const short* dbCoeffsQ = numberQSide == 7 ? dbCoeffs7 : ( numberQSide == 5 ) ? dbCoeffs5 : dbCoeffs3;
  const short* tcP       = numberPSide == 7 ? tc7 : ( numberPSide == 5 ) ? tc5: tc3;
  const short* tcQ       = numberQSide == 7 ? tc7 : ( numberQSide == 5 ) ? tc5: tc3;

        __m128i vtc    = _mm_set1_epi16  ( tc );
  const __m128i shInv  = _mm_setr_epi8   ( 14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1 );
  const __m128i tcQarr = _mm_srli_epi16( _mm_mullo_epi16( _mm_loadu_si128 ( ( const __m128i * ) tcQ ), vtc ), 1 );
  const __m128i tcParr = _mm_srli_epi16( _mm_mullo_epi16( _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i * ) tcP ), shInv ), vtc ), 1 );
  const __m128i dbQarr = _mm_loadu_si128 ( ( const __m128i * ) dbCoeffsQ );
  const __m128i dbParr = _mm_shuffle_epi8( _mm_loadu_si128( ( const __m128i * ) dbCoeffsP ), shInv );

  __m128i vtmp, vmin, vmax, vsrcq, vsrcp;

  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    Pel* srcP = src + step * i - offset;
    Pel* srcQ = src + step * i;

    _mm_prefetch( ( const char * ) ( srcP + step - numberPSide ), _MM_HINT_T0 );
    _mm_prefetch( ( const char * ) ( srcQ + step + numberQSide ), _MM_HINT_T0 );

    if( numberPSide == 3 )
      vsrcp = _mm_unpacklo_epi64( _mm_setzero_si128(), _mm_loadu_si64( ( const __m128i* ) ( srcP - 3 * offset ) ) );
    else
      vsrcp = _mm_loadu_si128( ( const __m128i* ) ( srcP - 7 * offset ) );

    if( numberQSide == 3 )
      vsrcq = _mm_loadu_si64( ( const __m128i* ) srcQ );
    else
      vsrcq = _mm_loadu_si128( ( const __m128i* ) srcQ );

    vtmp = _mm_hadd_epi16( vsrcp, vsrcq );
    vtmp = _mm_add_epi16 ( vtmp, _mm_set1_epi16( 1 ) );
    vtmp = _mm_srli_epi16( vtmp, 1 );
    int getP = 3 - ( numberPSide >> 1 ), getQ = 4 + ( numberQSide >> 1 );
    getP <<= 1; getQ <<= 1;
    vtmp = _mm_shuffle_epi8( vtmp, _mm_setr_epi8( getP, getP + 1, getQ, getQ + 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 ) );

    const Pel refP = _mm_extract_epi16( vtmp, 0 );
    const Pel refQ = _mm_extract_epi16( vtmp, 1 );

    if( numberPSide == 7 )      vtmp = _mm_srli_si128( vsrcp,  2 );
    else if( numberPSide == 5 ) vtmp = _mm_srli_si128( vsrcp,  6 );
    else if( numberPSide == 3 ) vtmp = _mm_srli_si128( vsrcp, 10 );
    __m128i
    vtmq = _mm_shuffle_epi8( vsrcq, _mm_sub_epi8( shInv, _mm_set1_epi8( ( 8 - numberQSide ) << 1 ) ) );
    Pel refMiddle = 0;

    if( numberPSide == numberQSide )
    {
      vtmp = _mm_add_epi16( vtmp, vtmq );
      vtmq = numberPSide == 5 ? _mm_setr_epi16( 1, 1, 2, 2, 2, 0, 0, 0 ) : _mm_setr_epi16( 1, 1, 1, 1, 1, 1, 2, 0 );
      vtmp = _mm_madd_epi16( vtmp, vtmq );
      vtmp = _mm_hadd_epi32( vtmp, vtmp );
      vtmp = _mm_hadd_epi32( vtmp, vtmp );

      refMiddle = ( _mm_extract_epi32( vtmp, 0 ) + 8 ) >> 4;
    }
    else
    {
      int newNumberQSide = numberQSide;
      int newNumberPSide = numberPSide;

      if( numberQSide > numberPSide )
      {
        newNumberQSide = numberPSide;
        newNumberPSide = numberQSide;
      }

      if( newNumberPSide == 7 && newNumberQSide == 5 )
      {
        vtmp = _mm_add_epi16( vsrcp, _mm_shuffle_epi8( vsrcq, shInv ) );
        vtmq = _mm_setr_epi16( 0, 0, 1, 1, 1, 1, 2, 2 );
        vtmp = _mm_madd_epi16( vtmp, vtmq );
        vtmp = _mm_hadd_epi32( vtmp, vtmp );
        vtmp = _mm_hadd_epi32( vtmp, vtmp );
        refMiddle = ( _mm_extract_epi32( vtmp, 0 ) + 8 ) >> 4;
      }
      else if( newNumberPSide == 7 && newNumberQSide == 3 )
      {
        if( numberQSide > numberPSide )
        {
          vmin = vtmp;
          vtmp = vtmq;
          vtmq = vmin;
        }

        vtmq = _mm_shuffle_epi8( vtmq, _mm_setr_epi8( 0, 1, 2, 3, 4, 5, 0, 1, 2, 3,  4,  5,  2,  3,  4,  5 ) );
        vtmp = _mm_shuffle_epi8( vtmp, _mm_setr_epi8( 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 12, 13 ) );
        vtmp = _mm_add_epi16( vtmp, vtmq );
        vtmq = _mm_set1_epi16( 1 );
        vtmp = _mm_madd_epi16( vtmp, vtmq );
        vtmp = _mm_hadd_epi32( vtmp, vtmp );
        vtmp = _mm_hadd_epi32( vtmp, vtmp );
        refMiddle = ( _mm_extract_epi32( vtmp, 0 ) + 8 ) >> 4;
      }
      else
      {
        vtmp = _mm_add_epi16( vsrcp, _mm_shuffle_epi8( vsrcq, shInv ) );
        vtmq = _mm_set1_epi16( 1 );
        vtmp = _mm_madd_epi16( vtmp, vtmq );
        vtmp = _mm_hadd_epi32( vtmp, vtmp );
        refMiddle = ( _mm_extract_epi32( vtmp, 3 ) + 4 ) >> 3;
      }
    }

#if USE_AVX2
    if( vext >= AVX2 )
    {
      __m256i ydbp, ytmp, ydst;

      // P-part
      vmax = tcParr;
      vmin = _mm_sub_epi16        ( vsrcp, vmax );
      vmax = _mm_add_epi16        ( vsrcp, vmax );
      vtmp = dbParr;
      ytmp = _mm256_inserti128_si256(
               _mm256_castsi128_si256(
                 _mm_unpacklo_epi16( dbParr, dbParr ) ),
               _mm_unpackhi_epi16( dbParr, dbParr ), 1 );
      ydbp = _mm256_abs_epi16     ( _mm256_sub_epi16( _mm256_set1_epi32( 64 ), ytmp ) );
      ytmp = _mm256_set1_epi32    ( refP | ( refMiddle * ( 1 << 16 ) ) );
      ydst = _mm256_madd_epi16    ( ydbp, ytmp );
      ydst = _mm256_add_epi32     ( ydst, _mm256_set1_epi32( 32 ) );
      ydst = _mm256_srli_epi32    ( ydst, 6 );
      vtmp = _mm256_cvtepi32_epi16x( ydst );

      vtmp = _mm_max_epi16( _mm_min_epi16( vtmp, vmax ), vmin );
      if( numberPSide == 7 )
      {
        vtmp = _mm_srli_si128( vtmp, 2 );
        _mm_storeu_si128( ( __m128i * ) ( srcP - 6 ), vtmp );
      }
      else if( numberPSide == 5 )
      {
        vtmp = _mm_srli_si128( vtmp, 6 );
        _mm_storeu_si128( ( __m128i * ) ( srcP - 4 ), vtmp );
      }
      else
      {
        vtmp = _mm_srli_si128( vtmp, 10 );
        _mm_storeu_si64( ( __m128i * ) ( srcP - 2 ), vtmp );
      }
      
      // Q-part
      vmax = tcQarr;
      vmin = _mm_sub_epi16        ( vsrcq, vmax );
      vmax = _mm_add_epi16        ( vsrcq, vmax );
      vtmp = dbQarr;
      ytmp = _mm256_inserti128_si256(
               _mm256_castsi128_si256(
                 _mm_unpacklo_epi16( dbQarr, dbQarr ) ),
               _mm_unpackhi_epi16( dbQarr, dbQarr ), 1 );
      ydbp = _mm256_abs_epi16     ( _mm256_sub_epi16( _mm256_set1_epi32( 64 ), ytmp ) );
      ytmp = _mm256_set1_epi32    ( refQ | ( refMiddle * ( 1 << 16 ) ) );
      ydst = _mm256_madd_epi16    ( ydbp, ytmp );
      ydst = _mm256_add_epi32     ( ydst, _mm256_set1_epi32( 32 ) );
      ydst = _mm256_srli_epi32    ( ydst, 6 );
      vtmp = _mm256_cvtepi32_epi16x( ydst );

      vtmp = _mm_max_epi16( _mm_min_epi16( vtmp, vmax ), vmin );
      if( numberQSide != 3 )
      {
        _mm_storeu_si128( ( __m128i * ) srcQ, vtmp );
      }
      else
      {
        _mm_storeu_si64( ( __m128i * ) srcQ, vtmp );
      }
    }
    else
#endif// en
    {
      __m128i ydb1, ydb2, ytm1, ytm2;

      // P-part
      vmax = tcParr;
      vmin = _mm_sub_epi16        ( vsrcp, vmax );
      vmax = _mm_add_epi16        ( vsrcp, vmax );
      ydb1 = _mm_unpacklo_epi16   ( dbParr, dbParr );
      ydb2 = _mm_unpackhi_epi16   ( dbParr, dbParr );
      ydb1 = _mm_abs_epi16        ( _mm_sub_epi16( _mm_set1_epi32( 64 ), ydb1 ) );
      ydb2 = _mm_abs_epi16        ( _mm_sub_epi16( _mm_set1_epi32( 64 ), ydb2 ) );
      ytm1 = _mm_unpacklo_epi16   ( _mm_set1_epi16( refP ), _mm_set1_epi16( refMiddle ) );
      ytm2 = ytm1;
      ytm1 = _mm_madd_epi16       ( ydb1, ytm1 );
      ytm2 = _mm_madd_epi16       ( ydb2, ytm2 );
      ytm1 = _mm_add_epi32        ( ytm1, _mm_set1_epi32( 32 ) );
      ytm2 = _mm_add_epi32        ( ytm2, _mm_set1_epi32( 32 ) );
      ytm1 = _mm_srli_epi32       ( ytm1, 6 );
      ytm2 = _mm_srli_epi32       ( ytm2, 6 );
      vtmp = _mm_packs_epi32      ( ytm1, ytm2 );

      vtmp = _mm_max_epi16( _mm_min_epi16( vtmp, vmax ), vmin );
      if( numberPSide == 7 )
      {
        vtmp = _mm_srli_si128( vtmp, 2 );
        _mm_storeu_si128( ( __m128i * ) ( srcP - 6 ), vtmp );
      }
      else if( numberPSide == 5 )
      {
        vtmp = _mm_srli_si128( vtmp, 6 );
        _mm_storeu_si128( ( __m128i * ) ( srcP - 4 ), vtmp );
      }
      else
      {
        vtmp = _mm_srli_si128( vtmp, 10 );
        _mm_storeu_si64( ( __m128i * ) ( srcP - 2 ), vtmp );
      }
      
      // Q-part
      vmax = tcQarr;
      vmin = _mm_sub_epi16        ( vsrcq, vmax );
      vmax = _mm_add_epi16        ( vsrcq, vmax );
      ydb1 = _mm_unpacklo_epi16   ( dbQarr, dbQarr );
      ydb2 = _mm_unpackhi_epi16   ( dbQarr, dbQarr );
      ydb1 = _mm_abs_epi16        ( _mm_sub_epi16( _mm_set1_epi32( 64 ), ydb1 ) );
      ydb2 = _mm_abs_epi16        ( _mm_sub_epi16( _mm_set1_epi32( 64 ), ydb2 ) );
      ytm1 = _mm_unpacklo_epi16   ( _mm_set1_epi16( refQ ), _mm_set1_epi16( refMiddle ) );
      ytm2 = ytm1;
      ytm1 = _mm_madd_epi16       ( ydb1, ytm1 );
      ytm2 = _mm_madd_epi16       ( ydb2, ytm2 );
      ytm1 = _mm_add_epi32        ( ytm1, _mm_set1_epi32( 32 ) );
      ytm2 = _mm_add_epi32        ( ytm2, _mm_set1_epi32( 32 ) );
      ytm1 = _mm_srli_epi32       ( ytm1, 6 );
      ytm2 = _mm_srli_epi32       ( ytm2, 6 );
      vtmp = _mm_packs_epi32      ( ytm1, ytm2 );

      vtmp = _mm_max_epi16( _mm_min_epi16( vtmp, vmax ), vmin );
      if( numberQSide != 3 )
      {
        _mm_storeu_si128( ( __m128i * ) srcQ, vtmp );
      }
      else
      {
        _mm_storeu_si64( ( __m128i * ) srcQ, vtmp );
      }
    }
  }
}

static inline void xBilinearFilter( Pel* srcP, Pel* srcQ, ptrdiff_t offset, int refMiddle, int refP, int refQ, int numberPSide, int numberQSide, const int* dbCoeffsP, const int* dbCoeffsQ, int tc )
{
  int src;
  const char tc7[7] = { 6, 5, 4, 3, 2, 1, 1 };
  const char tc3[3] = { 6, 4, 2 };
  const char *tcP = ( numberPSide == 3 ) ? tc3 : tc7;
  const char *tcQ = ( numberQSide == 3 ) ? tc3 : tc7;

  for( int pos = 0; pos < numberPSide; pos++ )
  {
    src = srcP[-offset * pos];
    int cvalue = ( tc * tcP[pos] ) >> 1;
    srcP[-offset * pos] = Clip3( src - cvalue, src + cvalue, ( ( refMiddle * dbCoeffsP[pos] + refP * ( 64 - dbCoeffsP[pos] ) + 32 ) >> 6 ) );
  }

  for( int pos = 0; pos < numberQSide; pos++ )
  {
    src = srcQ[offset * pos];
    int cvalue = ( tc * tcQ[pos] ) >> 1;
    srcQ[offset * pos] = Clip3( src - cvalue, src + cvalue, ( ( refMiddle * dbCoeffsQ[pos] + refQ * ( 64 - dbCoeffsQ[pos] ) + 32 ) >> 6 ) );
  }
}

template<X86_VEXT vext>
static void xFilteringPandQX86( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc )
{
  CHECKD( numberPSide <= 3 && numberQSide <= 3, "Short filtering in long filtering function" );
  if( step == 1 )
  {
    xFilteringPandQX86Hor<vext>( src, step, offset, numberPSide, numberQSide, tc );
  }
  else
  {
    xFilteringPandQX86Ver<vext>( src, step, offset, numberPSide, numberQSide, tc );
  }
#if USE_AVX2

  _mm256_zeroupper();
#endif
}

// The chroma deblocking of xPelFilterChroma8Core() for 8 lines at once: m[i] holds the sample i - 4 across the edge
// (p3 .. q3) of each line, one line per 16 bit lane, and each lane has its own tc and filter. The weak filter changes
// m[3] and m[4], the strong filter m[1] .. m[6] (m[3] .. m[6] at a CTU boundary). The sums of the strong filter are at
// most 8 * 1023 + 4 and the difference of the weak filter at most 5 * 1023 + 4 for samples of up to 10 bits, so 16 bit
// lanes suffice. A lane with tc 0 does not change.
static ALWAYS_INLINE void xPelFilterChromaLanes( const __m128i m[8], __m128i out[8], const __m128i vTc, const __m128i vStrong, const bool anyStrong, const bool isChromaHorCTBBoundary, const ClpRng& clpRng )
{
  const __m128i vFour = _mm_set1_epi16( 4 );
  const auto    add   = []( const __m128i a, const __m128i b ) { return _mm_add_epi16( a, b ); };
  const auto    x2    = []( const __m128i a ) { return _mm_add_epi16( a, a ); };

  for( int i = 0; i < 8; i++ )
  {
    out[i] = m[i];
  }

  // weak: delta = Clip3( -tc, tc, ( ( q0 - p0 ) * 4 + p1 - q1 + 4 ) >> 3 ), p0 + delta and q0 - delta clipped to the bit depth
  const __m128i vDiff  = _mm_add_epi16( _mm_slli_epi16( _mm_sub_epi16( m[4], m[3] ), 2 ), _mm_sub_epi16( m[2], m[5] ) );
  __m128i       vDelta = _mm_srai_epi16( _mm_add_epi16( vDiff, vFour ), 3 );
  vDelta = _mm_min_epi16( _mm_max_epi16( vDelta, _mm_sub_epi16( _mm_setzero_si128(), vTc ) ), vTc );
  const __m128i vMax = _mm_set1_epi16( clpRng.max() );
  out[3] = _mm_min_epi16( _mm_max_epi16( _mm_add_epi16( m[3], vDelta ), _mm_setzero_si128() ), vMax );
  out[4] = _mm_min_epi16( _mm_max_epi16( _mm_sub_epi16( m[4], vDelta ), _mm_setzero_si128() ), vMax );

  if( !anyStrong )
  {
    return;
  }

  // strong: the weighted sums of xPelFilterChroma(), rounded and divided by 8, clipped to center +- tc
  const auto filter = [&]( const __m128i vSum, const __m128i vCenter )
  {
    const __m128i vVal = _mm_srai_epi16( _mm_add_epi16( vSum, vFour ), 3 );
    return _mm_min_epi16( _mm_max_epi16( vVal, _mm_sub_epi16( vCenter, vTc ) ), _mm_add_epi16( vCenter, vTc ) );
  };
  const auto select = [&]( const __m128i vOther, const __m128i vStrongResult ) { return _mm_blendv_epi8( vOther, vStrongResult, vStrong ); };

  if( isChromaHorCTBBoundary )
  {
    // above a CTU boundary only one line, m[2], is used on the P side
    const __m128i vM345 = add( add( m[3], m[4] ), m[5] );
    out[3] = select( out[3], filter( add( add( add( x2( m[2] ), m[2] ), x2( m[3] ) ), add( add( m[4], m[5] ), m[6] ) ), m[3] ) );
    out[4] = select( out[4], filter( add( add( x2( m[2] ), add( vM345, m[4] ) ), add( m[6], m[7] ) ), m[4] ) );
    out[5] = select( out[5], filter( add( add( m[2], vM345 ), add( add( m[5], m[6] ), x2( m[7] ) ) ), m[5] ) );
    out[6] = select( out[6], filter( add( vM345, add( x2( m[6] ), add( x2( m[7] ), m[7] ) ) ), m[6] ) );
  }
  else
  {
    const __m128i vM34 = add( m[3], m[4] );
    out[1] = select( out[1], filter( add( add( add( x2( m[0] ), m[0] ), x2( m[1] ) ), add( m[2], vM34 ) ), m[1] ) );
    out[2] = select( out[2], filter( add( add( x2( m[0] ), m[1] ), add( add( x2( m[2] ), vM34 ), m[5] ) ), m[2] ) );
    out[3] = select( out[3], filter( add( add( add( m[0], m[1] ), add( m[2], m[3] ) ), add( add( vM34, m[5] ), m[6] ) ), m[3] ) );
    out[4] = select( out[4], filter( add( add( add( m[1], m[2] ), vM34 ), add( add( m[4], m[5] ), add( m[6], m[7] ) ) ), m[4] ) );
    out[5] = select( out[5], filter( add( add( m[2], vM34 ), add( add( x2( m[5] ), m[6] ), x2( m[7] ) ) ), m[5] ) );
    out[6] = select( out[6], filter( add( add( vM34, m[5] ), add( x2( m[6] ), add( x2( m[7] ), m[7] ) ) ), m[6] ) );
  }
}

template<X86_VEXT vext>
static void xPelFilterChroma8X86( Pel* piSrc, const ptrdiff_t step, const ptrdiff_t offset, const int16_t tc[8], const bool strong[8], const bool isChromaHorCTBBoundary, const ClpRng& clpRng )
{
  static_assert( sizeof( bool ) == 1, "the strong flags are loaded as bytes" );

  const __m128i vTc       = _mm_loadu_si128( (const __m128i*) tc );
  const __m128i vFiltered = _mm_cmpgt_epi16( vTc, _mm_setzero_si128() );
  // -1 for the lanes that use the strong filter, 0 for the others
  const __m128i vStrong   = _mm_and_si128( _mm_sub_epi16( _mm_setzero_si128(), _mm_cvtepu8_epi16( _mm_loadl_epi64( (const __m128i*) strong ) ) ), vFiltered );
  const bool    anyStrong = _mm_movemask_epi8( vStrong ) != 0;

  // Only the samples that xPelFilterChroma8Core() writes are written: p0 and q0 of the filtered lines, and the samples
  // further out of the strong lines. One bit per line.
  const int filteredLines = _mm_movemask_epi8( _mm_packs_epi16( vFiltered, _mm_setzero_si128() ) );
  const int strongLines   = _mm_movemask_epi8( _mm_packs_epi16( vStrong, _mm_setzero_si128() ) );
  const auto writtenLines = [&]( const int i ) { return i == 3 || i == 4 ? filteredLines : strongLines; };

  // the samples that are read (m[first] .. m[7]) and those that can change (m[firstOut] .. m[lastOut])
  const int first    = anyStrong && !isChromaHorCTBBoundary ? 0 : 2;
  const int last     = anyStrong ? 7 : 5;
  const int firstOut = anyStrong && !isChromaHorCTBBoundary ? 1 : 3;
  const int lastOut  = anyStrong ? 6 : 4;

  __m128i m[8], out[8];
  for( int i = 0; i < 8; i++ )
  {
    m[i] = _mm_setzero_si128();
  }

  if( step == 1 )
  {
    // a horizontal edge: the lines are neighbouring columns, so each row across the edge is one load
    for( int i = first; i <= last; i++ )
    {
      m[i] = _mm_loadu_si128( (const __m128i*) ( piSrc + ( i - 4 ) * offset ) );
    }
    xPelFilterChromaLanes( m, out, vTc, vStrong, anyStrong, isChromaHorCTBBoundary, clpRng );
    for( int i = firstOut; i <= lastOut; i++ )
    {
      Pel*      row   = piSrc + ( i - 4 ) * offset;
      const int lines = writtenLines( i );
      if( lines == 0xff )
      {
        _mm_storeu_si128( (__m128i*) row, out[i] );
      }
      else if( lines )
      {
        Pel samples[8];
        _mm_storeu_si128( (__m128i*) samples, out[i] );
        for( int l = 0; l < 8; l++ )
        {
          if( lines & ( 1 << l ) )
          {
            row[l] = samples[l];
          }
        }
      }
    }
    return;
  }

  // a vertical edge: the lines are rows of 8 samples p3 .. q3, transposed into one vector per sample position
  __m128i vRow[8];
  for( int l = 0; l < 8; l++ )
  {
    vRow[l] = _mm_loadu_si128( (const __m128i*) ( piSrc + l * step - 4 ) );
  }
  const __m128i a0 = _mm_unpacklo_epi16( vRow[0], vRow[1] ), a1 = _mm_unpackhi_epi16( vRow[0], vRow[1] );
  const __m128i a2 = _mm_unpacklo_epi16( vRow[2], vRow[3] ), a3 = _mm_unpackhi_epi16( vRow[2], vRow[3] );
  const __m128i a4 = _mm_unpacklo_epi16( vRow[4], vRow[5] ), a5 = _mm_unpackhi_epi16( vRow[4], vRow[5] );
  const __m128i a6 = _mm_unpacklo_epi16( vRow[6], vRow[7] ), a7 = _mm_unpackhi_epi16( vRow[6], vRow[7] );
  const __m128i b0 = _mm_unpacklo_epi32( a0, a2 ), b1 = _mm_unpackhi_epi32( a0, a2 );
  const __m128i b2 = _mm_unpacklo_epi32( a1, a3 ), b3 = _mm_unpackhi_epi32( a1, a3 );
  const __m128i b4 = _mm_unpacklo_epi32( a4, a6 ), b5 = _mm_unpackhi_epi32( a4, a6 );
  const __m128i b6 = _mm_unpacklo_epi32( a5, a7 ), b7 = _mm_unpackhi_epi32( a5, a7 );
  m[0] = _mm_unpacklo_epi64( b0, b4 );
  m[1] = _mm_unpackhi_epi64( b0, b4 );
  m[2] = _mm_unpacklo_epi64( b1, b5 );
  m[3] = _mm_unpackhi_epi64( b1, b5 );
  m[4] = _mm_unpacklo_epi64( b2, b6 );
  m[5] = _mm_unpackhi_epi64( b2, b6 );
  m[6] = _mm_unpacklo_epi64( b3, b7 );
  m[7] = _mm_unpackhi_epi64( b3, b7 );

  xPelFilterChromaLanes( m, out, vTc, vStrong, anyStrong, isChromaHorCTBBoundary, clpRng );

  // Back to the rows, two samples at a time: the pairs p2 p1, p0 q0 and q1 q2 are written by the same lines.
  for( int i = firstOut; i <= lastOut; i += 2 )
  {
    const int lines = writtenLines( i );
    if( !lines )
    {
      continue;
    }
    int32_t pairs[8];
    _mm_storeu_si128( (__m128i*) &pairs[0], _mm_unpacklo_epi16( out[i], out[i + 1] ) );
    _mm_storeu_si128( (__m128i*) &pairs[4], _mm_unpackhi_epi16( out[i], out[i + 1] ) );
    for( int l = 0; l < 8; l++ )
    {
      if( lines & ( 1 << l ) )
      {
        memcpy( piSrc + l * step + i - 4, &pairs[l], sizeof( int32_t ) );
      }
    }
  }
}

template <X86_VEXT vext>
void LoopFilter::_initLoopFilterX86()
{
  xPelFilterLuma    = xPelFilterLumaX86<vext>;
  xPelFilterChroma8 = xPelFilterChroma8X86<vext>;
  xFilteringPandQ   = xFilteringPandQX86<vext>;
}

template void LoopFilter::_initLoopFilterX86<SIMDX86>();

}

#endif
