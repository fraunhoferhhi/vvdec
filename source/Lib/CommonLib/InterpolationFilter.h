/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

/**
 * \file
 * \brief Declaration of InterpolationFilter class
 */

#pragma once

#include "CommonDef.h"

#include "Unit.h"
#include "Buffer.h"

namespace vvdec
{

#define IF_INTERNAL_PREC 14 ///< Number of bits for internal precision
#define IF_FILTER_PREC    6 ///< Log2 of sum of filter taps
#define IF_INTERNAL_OFFS (1<<(IF_INTERNAL_PREC-1)) ///< Offset used internally
#define IF_INTERNAL_PREC_BILINEAR 10 ///< Number of bits for internal precision
#define IF_FILTER_PREC_BILINEAR   4  ///< Bilinear filter coeff precision so that intermediate value will not exceed 16 bit for SIMD - bit exact
/**
 * \brief Interpolation filter class
 */
class InterpolationFilter
{
  static const TFilterCoeff m_lumaFilter4x4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA];
public:
  static const TFilterCoeff m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps
  static const TFilterCoeff m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA]; ///< Chroma filter taps
  static const TFilterCoeff m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps 2x
  static const TFilterCoeff m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA]; ///< Chroma filter taps 1.5x
  static const TFilterCoeff m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA]; ///< Chroma filter taps 2x
  static const TFilterCoeff m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps 1.5x
  static const TFilterCoeff m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA]; ///< Luma filter taps 2x
private:
  static const TFilterCoeff m_lumaAltHpelIFilter[NTAPS_LUMA]; ///< Luma filter taps
  static const TFilterCoeff m_bilinearFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR]; ///< bilinear filter taps
  static const TFilterCoeff m_bilinearFilterPrec4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR]; ///< bilinear filter taps
public:
  template<bool isFirst, bool isLast>
  static void filterCopy(const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, bool biMCForDMVR );
  template<int N, bool isVertical, bool isFirst, bool isLast>
  static void filter    (const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR);
  template<int N>
  void filterHor        (const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height,               bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR);
  template<int N>
  void filterVer        (const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, bool isFirst, bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR);

  template<bool isLast, int w>
  static void filterXxY_N2     (const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);
  template<bool isLast, int w>
  static void filterXxY_N4     (const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);
  template<bool isLast, int w>
  static void filterXxY_N8     (const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV);

  static void scalarFilterN2_2D(const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *ch,     TFilterCoeff const *cv);

  static void xWeightedGeoBlk( const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, const ClpRng& clipRng );
  void        weightedGeoBlk ( const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, const ClpRng& clipRng );
protected:
public:
  InterpolationFilter();
  ~InterpolationFilter() {}


  void( *m_filterN2_2D        )( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *ch, TFilterCoeff const *cv );
  void( *m_filterHor[3][2][2] )( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR );
  void( *m_filterVer[3][2][2] )( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR );
  void( *m_filterCopy[2][2] )  ( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, bool biMCForDMVR );
  void( *m_filter4x4  [3][2] ) ( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV );
  void( *m_filter8x8  [3][2] ) ( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV );
  void( *m_filter16x16[3][2] ) ( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV );
  void( *m_weightedGeoBlk )(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, const ClpRng& clpRng);
  void initInterpolationFilter( bool enable );
#ifdef TARGET_SIMD_X86
  void initInterpolationFilterX86();
  template <X86_VEXT vext>
  void _initInterpolationFilterX86();
#endif

  void filterN2_2D(const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int fracX, int fracY, const ChromaFormat fmt, const ClpRng& clpRng);
  void filter4x4  (const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0);
  void filter8x8  (const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf = false, int nFilterIdx = 0);
  void filter16x16(const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf = false, int nFilterIdx = 0);
  void filterHor  (const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int frac,               bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false );
  void filterVer  (const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx = 0, bool biMCForDMVR = false, bool useAltHpelIf = false );
  static TFilterCoeff const * getChromaFilterTable(const int deltaFract) { return m_chromaFilter[deltaFract]; };
};

}
