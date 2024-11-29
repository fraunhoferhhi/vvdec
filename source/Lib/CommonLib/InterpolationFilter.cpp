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

/**
 * \file
 * \brief Implementation of InterpolationFilter class
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "InterpolationFilter.h"

#include "Buffer.h"
#include "Rom.h"

#include "ChromaFormat.h"

namespace vvdec
{

// ====================================================================================================================
// Tables
// ====================================================================================================================
alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_lumaFilter4x4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  {  0, 1,  -5, 62,  8,  -3,  1,  0 },
  {  0, 2,  -8, 60, 13,  -4,  1,  0 },
  {  0, 3, -10, 58, 17,  -5,  1,  0 }, //1/4
  {  0, 3, -11, 52, 26,  -8,  2,  0 },
  {  0, 2,  -9, 47, 31, -10,  3,  0 },
  {  0, 3, -11, 45, 34, -10,  3,  0 },
  {  0, 3, -11, 40, 40, -11,  3,  0 }, //1/2
  {  0, 3, -10, 34, 45, -11,  3,  0 },
  {  0, 3, -10, 31, 47,  -9,  2,  0 },
  {  0, 2,  -8, 26, 52, -11,  3,  0 },
  {  0, 1,  -5, 17, 58, -10,  3,  0 }, //3/4
  {  0, 1,  -4, 13, 60,  -8,  2,  0 },
  {  0, 1,  -3,  8, 62,  -5,  1,  0 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 },
};

alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 4, -11, 52, 26,  -8,  3, -1 },
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 },
  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  { -1, 3,  -8, 26, 52, -11,  4, -1 },
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
  {  0, 1,  -4, 13, 60,  -8,  3, -1 },
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 },
};

alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_lumaAltHpelIFilter[NTAPS_LUMA] = {  0, 3, 9, 20, 20, 9, 3, 0 };

// 1.5x
alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  { -1, -5, 17, 42, 17, -5, -1,  0 },
  {  0, -5, 15, 41, 19, -5, -1,  0 },
  {  0, -5, 13, 40, 21, -4, -1,  0 },
  {  0, -5, 11, 39, 24, -4, -2,  1 },
  {  0, -5,  9, 38, 26, -3, -2,  1 },
  {  0, -5,  7, 38, 28, -2, -3,  1 },
  {  1, -5,  5, 36, 30, -1, -3,  1 },
  {  1, -4,  3, 35, 32,  0, -4,  1 },
  {  1, -4,  2, 33, 33,  2, -4,  1 },
  {  1, -4,  0, 32, 35,  3, -4,  1 },
  {  1, -3, -1, 30, 36,  5, -5,  1 },
  {  1, -3, -2, 28, 38,  7, -5,  0 },
  {  1, -2, -3, 26, 38,  9, -5,  0 },
  {  1, -2, -4, 24, 39, 11, -5,  0 },
  {  0, -1, -4, 21, 40, 13, -5,  0 },
  {  0, -1, -5, 19, 41, 15, -5,  0 }
};

// 2x
alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  { -4,  2, 20, 28, 20,  2, -4,  0 },
  { -4,  0, 19, 29, 21,  5, -4, -2 },
  { -4, -1, 18, 29, 22,  6, -4, -2 },
  { -4, -1, 16, 29, 23,  7, -4, -2 },
  { -4, -1, 16, 28, 24,  7, -4, -2 },
  { -4, -1, 14, 28, 25,  8, -4, -2 },
  { -3, -3, 14, 27, 26,  9, -3, -3 },
  { -3, -1, 12, 28, 25, 10, -4, -3 },
  { -3, -3, 11, 27, 27, 11, -3, -3 },
  { -3, -4, 10, 25, 28, 12, -1, -3 },
  { -3, -3,  9, 26, 27, 14, -3, -3 },
  { -2, -4,  8, 25, 28, 14, -1, -4 },
  { -2, -4,  7, 24, 28, 16, -1, -4 },
  { -2, -4,  7, 23, 29, 16, -1, -4 },
  { -2, -4,  6, 22, 29, 18, -1, -4 },
  { -2, -4,  5, 21, 29, 19,  0, -4 }
};

// 1.5x
alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  {  0, -6, 17, 42, 17, -5, -1,  0 },
  {  0, -5, 15, 41, 19, -5, -1,  0 },
  {  0, -5, 13, 40, 21, -4, -1,  0 },
  {  0, -5, 11, 39, 24, -4, -1,  0 },
  {  0, -5,  9, 38, 26, -3, -1,  0 },
  {  0, -5,  7, 38, 28, -2, -2,  0 },
  {  0, -4,  5, 36, 30, -1, -2,  0 },
  {  0, -3,  3, 35, 32,  0, -3,  0 },
  {  0, -3,  2, 33, 33,  2, -3,  0 },
  {  0, -3,  0, 32, 35,  3, -3,  0 },
  {  0, -2, -1, 30, 36,  5, -4,  0 },
  {  0, -2, -2, 28, 38,  7, -5,  0 },
  {  0, -1, -3, 26, 38,  9, -5,  0 },
  {  0, -1, -4, 24, 39, 11, -5,  0 },
  {  0, -1, -4, 21, 40, 13, -5,  0 },
  {  0, -1, -5, 19, 41, 15, -5,  0 }
};

// 2x
alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  {  0, -2, 20, 28, 20,  2, -4,  0 },
  {  0, -4, 19, 29, 21,  5, -6,  0 },
  {  0, -5, 18, 29, 22,  6, -6,  0 },
  {  0, -5, 16, 29, 23,  7, -6,  0 },
  {  0, -5, 16, 28, 24,  7, -6,  0 },
  {  0, -5, 14, 28, 25,  8, -6,  0 },
  {  0, -6, 14, 27, 26,  9, -6,  0 },
  {  0, -4, 12, 28, 25, 10, -7,  0 },
  {  0, -6, 11, 27, 27, 11, -6,  0 },
  {  0, -7, 10, 25, 28, 12, -4,  0 },
  {  0, -6,  9, 26, 27, 14, -6,  0 },
  {  0, -6,  8, 25, 28, 14, -5,  0 },
  {  0, -6,  7, 24, 28, 16, -5,  0 },
  {  0, -6,  7, 23, 29, 16, -5,  0 },
  {  0, -6,  6, 22, 29, 18, -5,  0 },
  {  0, -6,  5, 21, 29, 19, -4,  0 }
};

alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -1, 63,  2,  0 },
  { -2, 62,  4,  0 },
  { -2, 60,  7, -1 },
  { -2, 58, 10, -2 },
  { -3, 57, 12, -2 },
  { -4, 56, 14, -2 },
  { -4, 55, 15, -2 },
  { -4, 54, 16, -2 },
  { -5, 53, 18, -2 },
  { -6, 52, 20, -2 },
  { -6, 49, 24, -3 },
  { -6, 46, 28, -4 },
  { -5, 44, 29, -4 },
  { -4, 42, 30, -4 },
  { -4, 39, 33, -4 },
  { -4, 36, 36, -4 },
  { -4, 33, 39, -4 },
  { -4, 30, 42, -4 },
  { -4, 29, 44, -5 },
  { -4, 28, 46, -6 },
  { -3, 24, 49, -6 },
  { -2, 20, 52, -6 },
  { -2, 18, 53, -5 },
  { -2, 16, 54, -4 },
  { -2, 15, 55, -4 },
  { -2, 14, 56, -4 },
  { -2, 12, 57, -3 },
  { -2, 10, 58, -2 },
  { -1,  7, 60, -2 },
  {  0,  4, 62, -2 },
  {  0,  2, 63, -1 },
};

//1.5x
alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
{
  { 12, 40, 12,  0 },
  { 11, 40, 13,  0 },
  { 10, 40, 15, -1 },
  {  9, 40, 16, -1 },
  {  8, 40, 17, -1 },
  {  8, 39, 18, -1 },
  {  7, 39, 19, -1 },
  {  6, 38, 21, -1 },
  {  5, 38, 22, -1 },
  {  4, 38, 23, -1 },
  {  4, 37, 24, -1 },
  {  3, 36, 25,  0 },
  {  3, 35, 26,  0 },
  {  2, 34, 28,  0 },
  {  2, 33, 29,  0 },
  {  1, 33, 30,  0 },
  {  1, 31, 31,  1 },
  {  0, 30, 33,  1 },
  {  0, 29, 33,  2 },
  {  0, 28, 34,  2 },
  {  0, 26, 35,  3 },
  {  0, 25, 36,  3 },
  { -1, 24, 37,  4 },
  { -1, 23, 38,  4 },
  { -1, 22, 38,  5 },
  { -1, 21, 38,  6 },
  { -1, 19, 39,  7 },
  { -1, 18, 39,  8 },
  { -1, 17, 40,  8 },
  { -1, 16, 40,  9 },
  { -1, 15, 40, 10 },
  {  0, 13, 40, 11 },
};

//2x
alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
{
  { 17, 30, 17,  0 },
  { 17, 30, 18, -1 },
  { 16, 30, 18,  0 },
  { 16, 30, 18,  0 },
  { 15, 30, 18,  1 },
  { 14, 30, 18,  2 },
  { 13, 29, 19,  3 },
  { 13, 29, 19,  3 },
  { 12, 29, 20,  3 },
  { 11, 28, 21,  4 },
  { 10, 28, 22,  4 },
  { 10, 27, 22,  5 },
  {  9, 27, 23,  5 },
  {  9, 26, 24,  5 },
  {  8, 26, 24,  6 },
  {  7, 26, 25,  6 },
  {  7, 25, 25,  7 },
  {  6, 25, 26,  7 },
  {  6, 24, 26,  8 },
  {  5, 24, 26,  9 },
  {  5, 23, 27,  9 },
  {  5, 22, 27, 10 },
  {  4, 22, 28, 10 },
  {  4, 21, 28, 11 },
  {  3, 20, 29, 12 },
  {  3, 19, 29, 13 },
  {  3, 19, 29, 13 },
  {  2, 18, 30, 14 },
  {  1, 18, 30, 15 },
  {  0, 18, 30, 16 },
  {  0, 18, 30, 16 },
  { -1, 18, 30, 17 }
};

alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_bilinearFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
  { 64,  0, },
  { 60,  4, },
  { 56,  8, },
  { 52, 12, },
  { 48, 16, },
  { 44, 20, },
  { 40, 24, },
  { 36, 28, },
  { 32, 32, },
  { 28, 36, },
  { 24, 40, },
  { 20, 44, },
  { 16, 48, },
  { 12, 52, },
  {  8, 56, },
  {  4, 60, },
};

alignas(uint64_t) const TFilterCoeff InterpolationFilter::m_bilinearFilterPrec4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
  { 16,  0, },
  { 15,  1, },
  { 14,  2, },
  { 13,  3, },
  { 12,  4, },
  { 11,  5, },
  { 10,  6, },
  {  9,  7, },
  {  8,  8, },
  {  7,  9, },
  {  6, 10, },
  {  5, 11, },
  {  4, 12, },
  {  3, 13, },
  {  2, 14, },
  {  1, 15, }
};
// ====================================================================================================================
// Private member functions
// ====================================================================================================================

InterpolationFilter::InterpolationFilter()
{
  m_filterHor[0][0][0] = filter<8, false, false, false>;
  m_filterHor[0][0][1] = filter<8, false, false, true>;
  m_filterHor[0][1][0] = filter<8, false, true, false>;
  m_filterHor[0][1][1] = filter<8, false, true, true>;

  m_filterHor[1][0][0] = filter<4, false, false, false>;
  m_filterHor[1][0][1] = filter<4, false, false, true>;
  m_filterHor[1][1][0] = filter<4, false, true, false>;
  m_filterHor[1][1][1] = filter<4, false, true, true>;

  m_filterHor[2][0][0] = filter<2, false, false, false>;
  m_filterHor[2][0][1] = filter<2, false, false, true>;
  m_filterHor[2][1][0] = filter<2, false, true, false>;
  m_filterHor[2][1][1] = filter<2, false, true, true>;

  m_filterVer[0][0][0] = filter<8, true, false, false>;
  m_filterVer[0][0][1] = filter<8, true, false, true>;
  m_filterVer[0][1][0] = filter<8, true, true, false>;
  m_filterVer[0][1][1] = filter<8, true, true, true>;

  m_filterVer[1][0][0] = filter<4, true, false, false>;
  m_filterVer[1][0][1] = filter<4, true, false, true>;
  m_filterVer[1][1][0] = filter<4, true, true, false>;
  m_filterVer[1][1][1] = filter<4, true, true, true>;

  m_filterVer[2][0][0] = filter<2, true, false, false>;
  m_filterVer[2][0][1] = filter<2, true, false, true>;
  m_filterVer[2][1][0] = filter<2, true, true, false>;
  m_filterVer[2][1][1] = filter<2, true, true, true>;

  m_filterCopy[0][0]   = filterCopy<false, false>;
  m_filterCopy[0][1]   = filterCopy<false, true>;
  m_filterCopy[1][0]   = filterCopy<true, false>;
  m_filterCopy[1][1]   = filterCopy<true, true>;

  m_filter4x4[0][0] = filterXxY_N8<false, 4>;
  m_filter4x4[0][1] = filterXxY_N8<true , 4>;
  m_filter4x4[1][0] = filterXxY_N4<false, 4>;
  m_filter4x4[1][1] = filterXxY_N4<true , 4>;

  m_filter8x8[0][0] = filterXxY_N8<false, 8>;
  m_filter8x8[0][1] = filterXxY_N8<true , 8>;
  m_filter8x8[1][0] = filterXxY_N4<false, 8>;
  m_filter8x8[1][1] = filterXxY_N4<true , 8>;

  m_filter16x16[0][0] = filterXxY_N8<false, 16>;
  m_filter16x16[0][1] = filterXxY_N8<true , 16>;
  m_filter16x16[1][0] = filterXxY_N4<false, 16>;
  m_filter16x16[1][1] = filterXxY_N4<true , 16>;

  m_filterN2_2D = scalarFilterN2_2D;

  m_weightedGeoBlk = xWeightedGeoBlk;
}


/**
 * \brief Apply unit FIR filter to a block of samples
 *
 * \param bitDepth   bitDepth of samples
 * \param src        Pointer to source samples
 * \param srcStride  Stride of source samples
 * \param dst        Pointer to destination samples
 * \param dstStride  Stride of destination samples
 * \param width      Width of block
 * \param height     Height of block
 * \param isFirst    Flag indicating whether it is the first filtering operation
 * \param isLast     Flag indicating whether it is the last filtering operation
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<bool isFirst, bool isLast>
void InterpolationFilter::filterCopy( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, bool biMCForDMVR )
{
  int row, col;

  if ( isFirst == isLast )
  {
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        dst[col] = src[col];
      }

      INCY( src, srcStride );
      INCY( dst, dstStride );
    }
  }
  else if ( isFirst )
  {
    const int shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));

    if( biMCForDMVR )
    {
#if 0 // only relevant for high bit depth
      if( ( clpRng.bd - IF_INTERNAL_PREC_BILINEAR ) > 0 )
      {
        int shift10BitOut = ( clpRng.bd - IF_INTERNAL_PREC_BILINEAR );
        int offset = ( 1 << ( shift10BitOut - 1 ) );
        for( row = 0; row < height; row++ )
        {
          for( col = 0; col < width; col++ )
          {
            dst[col] = ( src[col] + offset ) >> shift10BitOut;
          }
          INCY( src, srcStride );
          INCY( dst, dstStride );
        }
      }
      else
#endif
      {
        int shift10BitOut = ( IF_INTERNAL_PREC_BILINEAR - clpRng.bd );
        for( row = 0; row < height; row++ )
        {
          for( col = 0; col < width; col++ )
          {
            dst[col] = src[col] << shift10BitOut;
          }
          INCY( src, srcStride );
          INCY( dst, dstStride );
        }
      }
    }
    else
    {
      if( shift >= 0 )
      {
        for( row = 0; row < height; row++ )
        {
          for( col = 0; col < width; col++ )
          {
            Pel val  = src[col] << shift;
            dst[col] = val - ( Pel ) IF_INTERNAL_OFFS;
          }

          INCY( src, srcStride );
          INCY( dst, dstStride );
        }
      }
      else
      {
        int shift1 = -shift - 1;
        int shift2 = -shift;
        for( row = 0; row < height; row++ )
        {
          for( col = 0; col < width; col++ )
          {
            Pel val  = ( src[col] + ( 1 << shift1 ) ) >> shift2;
            dst[col] = val - ( Pel ) IF_INTERNAL_OFFS;
          }

          INCY( src, srcStride );
          INCY( dst, dstStride );
        }
      }
    }
  }
  else
  {
    const int shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));

    for( row = 0; row < height; row++ )
    {
      for( col = 0; col < width; col++ )
      {
        Pel val = src[ col ];
        val = rightShift_round( ( val + IF_INTERNAL_OFFS ), shift );

        dst[ col ] = ClipPel( val, clpRng );
      }

      INCY( src, srcStride );
      INCY( dst, dstStride );
    }
  }
}

/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<int N, bool isVertical, bool isFirst, bool isLast>
void InterpolationFilter::filter(const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeff)
{
  int row, col;

  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if ( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if ( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if ( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }

  ptrdiff_t cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  int offset;
  int headRoom = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
  int shift    = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  CHECK(shift < 0, "Negative shift");

  if( N == 2 )
  {
    if( isFirst )
    {
      shift = IF_FILTER_PREC_BILINEAR - ( IF_INTERNAL_PREC_BILINEAR - clpRng.bd );
      offset = 1 << ( shift - 1 );
    }
    else
    {
      shift = 4;
      offset = 1 << ( shift - 1 );
    }
  }
  else
  {
    if( isLast )
    {
      shift += ( isFirst ) ? 0 : headRoom;
      offset = 1 << ( shift - 1 );
      offset += ( isFirst ) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
    }
    else
    {
      shift -= ( isFirst ) ? headRoom : 0;
      offset = ( isFirst ) ? -IF_INTERNAL_OFFS * ( 1 << shift ) : 0;
    }
  }

  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      int sum;

      sum  = src[ col + 0 * cStride] * c[0];
      sum += src[ col + 1 * cStride] * c[1];
      if ( N >= 4 )
      {
        sum += src[ col + 2 * cStride] * c[2];
        sum += src[ col + 3 * cStride] * c[3];
      }
      if ( N >= 6 )
      {
        sum += src[ col + 4 * cStride] * c[4];
        sum += src[ col + 5 * cStride] * c[5];
      }
      if ( N == 8 )
      {
        sum += src[ col + 6 * cStride] * c[6];
        sum += src[ col + 7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if ( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[col] = val;
    }

    INCY( src, srcStride );
    INCY( dst, dstStride );
  }
}


void InterpolationFilter::filterN2_2D( const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int fracX, int fracY, const ChromaFormat fmt, const ClpRng& clpRng )
{
  m_filterN2_2D( clpRng, src, srcStride, dst, dstStride, width, height, m_bilinearFilterPrec4[fracX], m_bilinearFilterPrec4[fracY] );
}


void InterpolationFilter::scalarFilterN2_2D( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *ch, TFilterCoeff const *cv )
{
  Pel *tmp = ( Pel* ) alloca( width * ( height + 1 ) * sizeof( Pel ) );

  filter<2, false, true,  false>( clpRng, src, srcStride, tmp, width,     width, height + 1, ch );
  filter<2, true , false, false>( clpRng, tmp, width,     dst, dstStride, width, height,     cv );
}

void InterpolationFilter::filter4x4( const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng )
{
  const int vFilterSize = isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA;

  if( vFilterSize == 8 )
  {
    CHECKD( !isLuma( compID ), "8-tap filter is only allowed for luma!" );
    m_filter4x4[0][isLast]( clpRng, src, srcStride, dst, dstStride, 4, 4, m_lumaFilter4x4[fracX], m_lumaFilter4x4[fracY] );
  }
  else if( vFilterSize == 4 )
  {
    CHECKD( !isChroma( compID ), "4-tap filter is only allowed for luma!" );

    const int csx = getComponentScaleX( compID, fmt );
    const int csy = getComponentScaleY( compID, fmt );

    m_filter4x4[1][isLast]( clpRng, src, srcStride, dst, dstStride, 4, 4, m_chromaFilter[fracX << ( 1 - csx )], m_chromaFilter[fracY << ( 1 - csy )] );
  }
}

void InterpolationFilter::filter8x8( const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf )
{
  const int vFilterSize = isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA;

  if( vFilterSize == 8 )
  {
    CHECKD( !isLuma( compID ), "8-tap filter is only allowed for luma!" );
    const TFilterCoeff* vc = ( fracX == 8 && useAltHpelIf ) ? m_lumaAltHpelIFilter : m_lumaFilter[fracX];
    const TFilterCoeff* hc = ( fracY == 8 && useAltHpelIf ) ? m_lumaAltHpelIFilter : m_lumaFilter[fracY];

    m_filter8x8[0][isLast]( clpRng, src, srcStride, dst, dstStride, 8, height, vc, hc );
  }
  else if( vFilterSize == 4 )
  {
    CHECKD( !isChroma( compID ), "4-tap filter is only allowed for luma!" );

    const int csx = getComponentScaleX( compID, fmt );
    const int csy = getComponentScaleY( compID, fmt );

    m_filter8x8[1][isLast]( clpRng, src, srcStride, dst, dstStride, 8, height, m_chromaFilter[fracX << ( 1 - csx )], m_chromaFilter[fracY << ( 1 - csy )] );
  }
}

void InterpolationFilter::filter16x16( const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int fracX, int fracY, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, bool useAltHpelIf )
{
  const int vFilterSize = isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA;

  if( vFilterSize == 8 )
  {
    CHECKD( !isLuma( compID ), "8-tap filter is only allowed for luma!" );
    const TFilterCoeff* vc = ( fracX == 8 && useAltHpelIf ) ? m_lumaAltHpelIFilter : m_lumaFilter[fracX];
    const TFilterCoeff* hc = ( fracY == 8 && useAltHpelIf ) ? m_lumaAltHpelIFilter : m_lumaFilter[fracY];

    m_filter16x16[0][isLast]( clpRng, src, srcStride, dst, dstStride, 16, height, vc, hc );
  }
  else if( vFilterSize == 4 )
  {
    CHECKD( !isChroma( compID ), "4-tap filter is only allowed for luma!" );

    const int csx = getComponentScaleX( compID, fmt );
    const int csy = getComponentScaleY( compID, fmt );

    m_filter16x16[1][isLast]( clpRng, src, srcStride, dst, dstStride, 16, height, m_chromaFilter[fracX << ( 1 - csx )], m_chromaFilter[fracY << ( 1 - csy )] );
  }
}


template<bool isLast, int w>
void InterpolationFilter::filterXxY_N2( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* _dst, const ptrdiff_t dstStride, int width, int h, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  int row, col;

  Pel cH[2];
  cH[0] = coeffH[0]; cH[1] = coeffH[1];
  Pel cV[2];
  cV[0] = coeffV[0]; cV[1] = coeffV[1];

  int offset1st, offset2nd;
  int headRoom   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st   = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if( isLast )
  {
    shift1st  -= headRoom;
    shift2nd  += headRoom;
    offset1st  = -IF_INTERNAL_OFFS << shift1st;
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS << shift1st;
    offset2nd = 0;
  }

  int *tmp = ( int * ) alloca( w * h * sizeof( int ) );
  memset( tmp, 0, w * h * sizeof( int ) );

  int** dst = ( int ** ) alloca( h * sizeof( int * ) );

  for( int i = 0; i < h; i++ ) dst[i] = &tmp[i * w];

  for( row = 0; row < ( h + 1 ); row++ )
  {
    for( col = 0; col < w; col++ )
    {
      int sum;

      sum  = src[col    ] * cH[0];
      sum += src[col + 1] * cH[1];

      sum = ( sum + offset1st ) >> shift1st;

      if( row >= 0 && row < h ) dst[row    ][col] += sum * cV[0];

      if( row >= 1 )
      {
        int val = ( dst[row - 1][col] + sum * cV[1] + offset2nd ) >> shift2nd;
        if( isLast )
        {
          val = ClipPel( val, clpRng );
        }
        _dst[col] = val;
      }
    }

    INCY( src, srcStride );
    if( row >= 1 ) INCY( _dst, dstStride );;
  }
}


template<bool isLast, int w>
void InterpolationFilter::filterXxY_N4( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* _dst, const ptrdiff_t dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  int row, col;

  Pel cH[4];
  cH[0] = coeffH[0]; cH[1] = coeffH[1];
  cH[2] = coeffH[2]; cH[3] = coeffH[3];
  Pel cV[4];
  cV[0] = coeffV[0]; cV[1] = coeffV[1];
  cV[2] = coeffV[2]; cV[3] = coeffV[3];

  int offset1st, offset2nd;
  int headRoom   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st   = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if( isLast )
  {
    shift1st  -= headRoom;
    shift2nd  += headRoom;
    offset1st  = -IF_INTERNAL_OFFS *(1<< shift1st);
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS *(1<< shift1st);
    offset2nd = 0;
  }

  OFFSET( src, srcStride, -1, -1 );

  int *tmp = ( int * ) alloca( w * height * sizeof( int ) );
  memset( tmp, 0, w * height * sizeof( int ) );

  int** dst = ( int ** ) alloca( height * sizeof( int * ) );

  for( int i = 0; i < height; i++ ) dst[i] = &tmp[i * w];

  for( row = 0; row < ( height + 3 ); row++ )
  {
    for( col = 0; col < w; col++ )
    {
      int sum;

      sum  = src[col    ] * cH[0];
      sum += src[col + 1] * cH[1];
      sum += src[col + 2] * cH[2];
      sum += src[col + 3] * cH[3];

      sum = ( sum + offset1st ) >> shift1st;

      if( row >= 0 && row < ( height + 0 ) ) dst[row    ][col] += sum * cV[0];
      if( row >= 1 && row < ( height + 1 ) ) dst[row - 1][col] += sum * cV[1]; 
      if( row >= 2 && row < ( height + 2 ) ) dst[row - 2][col] += sum * cV[2];

      if( row >= 3 )
      {
        int val = ( dst[row - 3][col] + sum * cV[3] + offset2nd ) >> shift2nd;
        if( isLast )
        {
          val = ClipPel( val, clpRng );
        }
        _dst[col] = val;
      }
    }

    INCY( src, srcStride );
    if( row >= 3 ) INCY( _dst, dstStride );;
  }
}


template<bool isLast, int w>
void InterpolationFilter::filterXxY_N8( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* _dst, const ptrdiff_t dstStride, int width, int h, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  int row, col;

  Pel cH[8];
  cH[0] = coeffH[0]; cH[1] = coeffH[1];
  cH[2] = coeffH[2]; cH[3] = coeffH[3];
  cH[4] = coeffH[4]; cH[5] = coeffH[5];
  cH[6] = coeffH[6]; cH[7] = coeffH[7];
  Pel cV[8];
  cV[0] = coeffV[0]; cV[1] = coeffV[1];
  cV[2] = coeffV[2]; cV[3] = coeffV[3];
  cV[4] = coeffV[4]; cV[5] = coeffV[5];
  cV[6] = coeffV[6]; cV[7] = coeffV[7];

  int offset1st, offset2nd;
  int headRoom   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift1st   = IF_FILTER_PREC, shift2nd = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  if( isLast )
  {
    shift1st  -= headRoom;
    shift2nd  += headRoom;
    offset1st  = -IF_INTERNAL_OFFS *(1<< shift1st);
    offset2nd  = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift1st -= headRoom;
    offset1st = -IF_INTERNAL_OFFS *(1<< shift1st);
    offset2nd = 0;
  }

  OFFSET( src, srcStride, -3, -3 );

  int *tmp = ( int * ) alloca( w * h * sizeof( int ) );
  memset( tmp, 0, w * h * sizeof( int ) );

  int** dst = ( int ** ) alloca( h * sizeof( int * ) );

  for( int i = 0; i < h; i++ ) dst[i] = &tmp[i * w];

  for( row = 0; row < ( h + 7 ); row++ )
  {
    for( col = 0; col < w; col++ )
    {
      int sum;

      sum  = src[col    ] * cH[0];
      sum += src[col + 1] * cH[1];
      sum += src[col + 2] * cH[2];
      sum += src[col + 3] * cH[3];
      sum += src[col + 4] * cH[4];
      sum += src[col + 5] * cH[5];
      sum += src[col + 6] * cH[6];
      sum += src[col + 7] * cH[7];

      sum = ( sum + offset1st ) >> shift1st;

      if( row >= 0 && row < ( h + 0 ) ) dst[row    ][col] += sum * cV[0];
      if( row >= 1 && row < ( h + 1 ) ) dst[row - 1][col] += sum * cV[1]; 
      if( row >= 2 && row < ( h + 2 ) ) dst[row - 2][col] += sum * cV[2];
      if( row >= 3 && row < ( h + 3 ) ) dst[row - 3][col] += sum * cV[3];
      if( row >= 4 && row < ( h + 4 ) ) dst[row - 4][col] += sum * cV[4];
      if( row >= 5 && row < ( h + 5 ) ) dst[row - 5][col] += sum * cV[5];
      if( row >= 6 && row < ( h + 6 ) ) dst[row - 6][col] += sum * cV[6];

      if( row >= 7 )
      {
        int val = ( dst[row - 7][col] + sum * cV[7] + offset2nd ) >> shift2nd;
        if( isLast )
        {
          val = ClipPel( val, clpRng );
        }
        _dst[col] = val;
      }
    }

    INCY( src, srcStride );
    if( row >= 7 ) INCY( _dst, dstStride );;
  }
}

/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<int N>
void InterpolationFilter::filterHor(const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, bool isLast, TFilterCoeff const *coeff)
{
//#if ENABLE_SIMD_OPT_MCIF
  if( N == 8 )
  {
    m_filterHor[0][1][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else if( N == 4 )
  {
    m_filterHor[1][1][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else if( N == 2 )
  {
    m_filterHor[2][1][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else
  {
    THROW_FATAL( "Invalid tap number" );
  }
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<int N>
void InterpolationFilter::filterVer( const ClpRng& clpRng, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, bool isFirst, bool isLast, TFilterCoeff const *coeff )
{
//#if ENABLE_SIMD_OPT_MCIF
  if( N == 8 )
  {
    m_filterVer[0][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else if( N == 4 )
  {
    m_filterVer[1][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else if( N == 2 )
  {
    m_filterVer[2][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff );
  }
  else
  {
    THROW_FATAL( "Invalid tap number" );
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * \brief Filter a block of Luma/Chroma samples (horizontal)
 *
 * \param  compID     Chroma component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
void InterpolationFilter::filterHor( const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int frac, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx, bool useAltHpelIf )
{
  if( frac == 0 && nFilterIdx < 2 )
  {
    if( isLast )
      g_pelBufOP.copyBuffer( ( const char* ) src, srcStride * sizeof( Pel ), ( char* ) dst, dstStride * sizeof( Pel ), width * sizeof( Pel ), height );
    else
      m_filterCopy[true][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, nFilterIdx == 1 );
  }
  else if( isLuma( compID ) )
  {
    CHECK( frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );

    if( nFilterIdx == 0 )
    {
      if( frac == 8 && useAltHpelIf )
      {
        filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaAltHpelIFilter );
      }
      else
      {
        if( ( width == 4 && height == 4 ) || ( width == 4 && height == ( 4 + NTAPS_LUMA - 1 ) ) )
        {
          filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter4x4[frac] );
        }
        else
        {
          filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac] );
        }
      }
    }
    else if( nFilterIdx == 1 )
    {
      filterHor<NTAPS_BILINEAR>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_bilinearFilterPrec4[frac] );
    }
    else if( nFilterIdx >= 2 && nFilterIdx < 7 )
    {
      const TFilterCoeff* filterCoeff[5] = { m_lumaFilter4x4[frac], m_lumaFilterRPR1[frac], m_lumaFilterRPR2[frac], m_affineLumaFilterRPR1[frac], m_affineLumaFilterRPR2[frac] };

      filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, filterCoeff[nFilterIdx - 2] );
    }
    else
    {
      THROW_FATAL( "Unknown luma filter index '" << nFilterIdx << "'!" );
    }
  }
  else
  {
    const uint32_t csx = getComponentScaleX( compID, fmt );

    CHECK( frac < 0 || csx >= 2 || ( frac << ( 1 - csx ) ) >= CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    
    if( nFilterIdx == 0 )
    {
      filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac << ( 1 - csx )] );
    }
    else if( nFilterIdx == 3 )
    {
      filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilterRPR1[frac << ( 1 - csx )] );
    }
    else if( nFilterIdx == 4 )
    {
      filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilterRPR2[frac << ( 1 - csx )] );
    }
    else
    {
      filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac << ( 1 - csx )] );
    }
  }
}


/**
 * \brief Filter a block of Luma/Chroma samples (vertical)
 *
 * \param  compID     Colour component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
void InterpolationFilter::filterVer( const ComponentID compID, const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx, bool useAltHpelIf )
{
  if( frac == 0 && nFilterIdx < 2 )
  {
    m_filterCopy[isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, nFilterIdx == 1 );
  }
  else if( isLuma( compID ) )
  {
    CHECK( frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );

    if( nFilterIdx == 0 )
    {
      if( frac == 8 && useAltHpelIf )
      {
        filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaAltHpelIFilter );
      }
      else
      {
        if( width == 4 && height == 4 )
        {
          filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter4x4[frac] );
        }
        else
        {
          filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter[frac] );
        }
      }
    }
    else if( nFilterIdx == 1 )
    {
      filterVer<NTAPS_BILINEAR>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_bilinearFilterPrec4[frac] );
    }
    else if( nFilterIdx >= 2 && nFilterIdx < 7 )
    {
      const TFilterCoeff* filterCoeff[5] = { m_lumaFilter4x4[frac], m_lumaFilterRPR1[frac], m_lumaFilterRPR2[frac], m_affineLumaFilterRPR1[frac], m_affineLumaFilterRPR2[frac] };

      filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, filterCoeff[nFilterIdx - 2] );
    }
    else
    {
      THROW_FATAL( "Unknown luma filter index '" << nFilterIdx << "'!" );
    }
  }
  else
  {
    const uint32_t csy = getComponentScaleY( compID, fmt );
    CHECK( frac < 0 || csy >= 2 || ( frac << ( 1 - csy ) ) >= CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );

    if( nFilterIdx == 0 )
    {
      filterVer<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac << ( 1 - csy )] );
    }
    else if( nFilterIdx == 3 )
    {
      filterVer<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilterRPR1[frac << ( 1 - csy )] );
    }
    else if( nFilterIdx == 4 )
    {
      filterVer<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilterRPR2[frac << ( 1 - csy )] );
    }
    else
    {
      filterVer<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac << ( 1 - csy )] );
    }
  }
}

void InterpolationFilter::weightedGeoBlk(const CodingUnit &cu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, const ClpRng& clipRng)
{
  m_weightedGeoBlk(cu, width, height, compIdx, splitDir, predDst, predSrc0, predSrc1, clipRng);
}

void InterpolationFilter::xWeightedGeoBlk(const CodingUnit &cu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, const ClpRng& clipRng)
{
  Pel*    dst = predDst.get(compIdx).buf;
  Pel*    src0 = predSrc0.get(compIdx).buf;
  Pel*    src1 = predSrc1.get(compIdx).buf;
  ptrdiff_t strideDst = predDst.get(compIdx).stride - width;
  ptrdiff_t strideSrc0 = predSrc0.get(compIdx).stride - width;
  ptrdiff_t strideSrc1 = predSrc1.get(compIdx).stride - width;

  const char    log2WeightBase = 3;
//  const ClpRng  clipRng = cu.slice->clpRngs().comp[compIdx];
  const int32_t clipbd = clipRng.bd;
  const int32_t shiftWeighted = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
  const int32_t offsetWeighted = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);
  const uint32_t scaleX = getComponentScaleX(compIdx, cu.chromaFormat);
  const uint32_t scaleY = getComponentScaleY(compIdx, cu.chromaFormat);

  int16_t angle = g_GeoParams[splitDir][0];
  int16_t wIdx = getLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = getLog2(cu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t stepX = 1 << scaleX;
  int16_t stepY = 0;
  int16_t* weight = nullptr;
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -(int)((GEO_WEIGHT_MASK_SIZE << scaleY) + cu.lwidth());
    weight = &g_globalGeoWeights[g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX = -1 *(1<< scaleX);
    stepY = (GEO_WEIGHT_MASK_SIZE << scaleY) + cu.lwidth();
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = (GEO_WEIGHT_MASK_SIZE << scaleY) - cu.lwidth();
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      *dst++  = ClipPel(rightShift((*weight*(*src0++) + ((8 - *weight) * (*src1++)) + offsetWeighted), shiftWeighted), clipRng);
      weight += stepX;
    }
    dst    += strideDst;
    src0   += strideSrc0;
    src1   += strideSrc1;
    weight += stepY;
  }
}

/**
 * \brief turn on SIMD fuc
 *
 * \param bEn   enabled of SIMD function for interpolation
 */
void InterpolationFilter::initInterpolationFilter( bool enable )
{
#if ENABLE_SIMD_OPT_MCIF
  if( enable )
  {
#  ifdef TARGET_SIMD_X86
    initInterpolationFilterX86();
#  endif
#  ifdef TARGET_SIMD_ARM
    initInterpolationFilterARM();
#  endif
  }
#endif
}
}   // namespace vvdec
