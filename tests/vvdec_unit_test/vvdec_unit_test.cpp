/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2018-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVdeC Authors.
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

#include <iostream>
#include <limits.h>

#include "CommonLib/CommonDef.h"
#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/InterpolationFilter.h"

using namespace vvdec;

#define NUM_CASES 100

template<typename T>
static inline bool compare_value( const std::string& context, const T ref, const T opt )
{
  if( opt != ref )
  {
    std::cerr << "failed: " << context << "\n"
              << "  mismatch:  ref=" << ref << "  opt=" << opt << "\n";
  }
  return opt == ref;
}

template<typename T>
static inline bool compare_values_1d( const std::string& context, const T* ref, const T* opt, unsigned length )
{
  for( unsigned idx = 0; idx < length; ++idx )
  {
    if( ref[idx] != opt[idx] )
    {
      std::cout << "failed: " << context << "\n"
                << "  mismatch:  ref[" << idx << "]=" << ref[idx] << "  opt[" << idx << "]=" << opt[idx] << "\n";
      return false;
    }
  }
  return true;
}

template<typename T, typename U = T>
static inline bool compare_values_2d( const std::string& context, const T* ref, const T* opt, unsigned rows,
                                      unsigned cols, unsigned stride = 0, U tolerance = U( 0 ) )
{
  stride = stride != 0 ? stride : cols;

  auto abs_diff = []( T value1, T value2 ) -> U { return static_cast<U>( std::abs( value1 - value2 ) ); };

  for( unsigned row = 0; row < rows; ++row )
  {
    for( unsigned col = 0; col < cols; ++col )
    {
      unsigned idx = row * stride + col;
      if( abs_diff( ref[idx], opt[idx] ) > tolerance )
      {
        std::cout << "failed: " << context << "\n"
                  << "  mismatch:  ref[" << row << "*" << stride << "+" << col << "]=" << ref[idx] << "  opt[" << row
                  << "*" << stride << "+" << col << "]=" << opt[idx] << "\n";
        return false;
      }
    }
  }
  return true;
}

template<typename T>
class MinMaxGenerator
{
public:
  explicit MinMaxGenerator( unsigned bits, bool is_signed = true )
  {
    if( is_signed )
    {
      T half = 1 << ( bits - 1 );
      m_min = -half;
      m_max = half - 1;
    }
    else
    {
      m_min = 0;
      m_max = ( 1 << bits ) - 1;
    }
  }

  T operator()() const
  {
    return ( rand() & 1 ) ? m_max : m_min;
  }

  std::string input_type() const
  {
    return "MinOrMax";
  }

private:
  T m_min;
  T m_max;
};

template<typename T>
class InputGenerator
{
public:
  explicit InputGenerator( unsigned bits, bool is_signed = true ) : m_bits( bits ), m_signed( is_signed )
  {
  }

  T operator()() const
  {
    if( !m_signed )
    {
      return static_cast<T>( rand() & ( ( 1 << m_bits ) - 1 ) );
    }
    else
    {
      return ( rand() & ( ( 1 << m_bits ) - 1 ) ) - ( 1 << m_bits >> 1 );
    }
  }

  std::string input_type() const
  {
    return "Rand";
  }

private:
  unsigned m_bits;
  bool m_signed;
};

class DimensionGenerator
{
public:
  unsigned get( unsigned min, unsigned max, unsigned mod = 1 ) const
  {
    CHECK( max < min, "max should be >= min" );
    CHECK( mod == 0, "mod must be >= 1" );                   // Avoids div by 0.
    CHECK( min == 0 && max == UINT_MAX, "range too large" ); // Avoids (max-min+1)==0.
    unsigned ret = rand() % ( max - min + 1 ) + min;
    ret -= ret % mod;
    return ret;
  }

  template<typename T>
  T getOneOf( const std::vector<T>& values ) const
  {
    CHECK( values.empty(), "getOneOf: values vector must not be empty" );
    return values[rand() % values.size()];
  }
};

#if ENABLE_SIMD_OPT_ALF
template<AlfFilterType filtType, typename G>
static bool check_one_filterBlk( AdaptiveLoopFilter* ref, AdaptiveLoopFilter* opt, ptrdiff_t srcStride,
                                 ptrdiff_t dstStride, unsigned int w, unsigned int h, int bitDepth, G input_generator )
{
  CHECK( srcStride < w, "OrgStride must be greater than or equal to width" );
  CHECK( dstStride < w, "BufStride must be greater than or equal to width" );

  std::ostringstream sstm;
  sstm << "filterBlk srcStride=" << srcStride << " dstStride=" << dstStride << " w=" << w << " h=" << h
       << " bd=" << bitDepth;

  ClpRng clpRng{ bitDepth };

  // Padding to src memory so that filterBlk can safely index [-3,+3] rows.
  constexpr int pad = 3;
  std::vector<Pel> src( ( h + 2 * pad ) * srcStride );
  std::vector<Pel> dst_ref( h * dstStride );
  std::vector<Pel> dst_opt( h * dstStride );
  std::generate( src.begin(), src.end(), input_generator );

  const Area blk{ 0, 0, w, h };

  Size sz{ w, h };
  AreaBuf<Pel> areaBufDst_ref{ dst_ref.data(), dstStride, sz };
  AreaBuf<Pel> areaBufDst_opt{ dst_opt.data(), dstStride, sz };
  AreaBuf<const Pel> areaBufSrc{ src.data() + pad * srcStride, srcStride, sz };

  DimensionGenerator rng;

  if( filtType == ALF_FILTER_7 )
  {
    ComponentID compId = COMPONENT_Y;
    ChromaFormat chromaFormat = CHROMA_400;

    // Give all three planes same buffer, as only one of them is active in filterBlk.
    PelUnitBuf dstUnitBuf_ref{
        chromaFormat,
        areaBufDst_ref, // COMPONENT_Y
        areaBufDst_ref, // COMPONENT_Cb
        areaBufDst_ref  // COMPONENT_Cr
    };
    PelUnitBuf dstUnitBuf_opt{ chromaFormat, areaBufDst_opt, areaBufDst_opt, areaBufDst_opt };
    CPelUnitBuf srcUnitBuf{ chromaFormat, areaBufSrc, areaBufSrc, areaBufSrc };

    const size_t numBlocks = AdaptiveLoopFilter::m_CLASSIFICATION_ARR_SIZE * ( ( h + 3 ) / 4 );
    std::vector<AlfClassifier> classifier;
    for( unsigned i = 0; i < numBlocks; ++i )
    {
      const uint8_t classIdx = rng.get( 0, MAX_NUM_ALF_CLASSES - 1 );
      const uint8_t transposeIdx = rng.get( 0, MAX_NUM_ALF_TRANSPOSE_ID - 1 );
      classifier.emplace_back( classIdx, transposeIdx );
    }

    const int vbCTUHeight = rng.getOneOf<int>( { 32, 64, 128 } );
    const int vbPos = vbCTUHeight - ALF_VB_POS_ABOVE_CTUROW_LUMA;

    // Build full coefficient and clip arrays for all classes and transpose variants.
    constexpr size_t LumaSz = MAX_NUM_ALF_TRANSPOSE_ID * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF;
    std::vector<short> coeffLuma( LumaSz );
    std::vector<short> clipLuma( LumaSz );

    for( unsigned t = 0; t < MAX_NUM_ALF_TRANSPOSE_ID; ++t )
    {
      for( unsigned c = 0; c < MAX_NUM_ALF_CLASSES; ++c )
      {
        int offset = ( t * MAX_NUM_ALF_CLASSES + c ) * MAX_NUM_ALF_LUMA_COEFF;
        auto coeff_idx = rng.get( 0, ALF_FIXED_FILTER_NUM - 1 );
        for( unsigned i = 0; i < MAX_NUM_ALF_LUMA_COEFF; ++i )
        {
          coeffLuma[offset + i] = static_cast<short>( AdaptiveLoopFilter::m_fixedFilterSetCoeff[coeff_idx][i] );
          auto clip_idx = rng.get( 0, AdaptiveLoopFilter::MaxAlfNumClippingValues - 1 );
          clipLuma[offset + i] = AdaptiveLoopFilter::m_alfClippVls[bitDepth - 8][clip_idx];
        }
      }
    }

    ref->m_filter7x7Blk( classifier.data(), dstUnitBuf_ref, srcUnitBuf, blk, compId, coeffLuma.data(), clipLuma.data(),
                         clpRng, vbCTUHeight, vbPos );
    opt->m_filter7x7Blk( classifier.data(), dstUnitBuf_opt, srcUnitBuf, blk, compId, coeffLuma.data(), clipLuma.data(),
                         clpRng, vbCTUHeight, vbPos );
  }
  else
  {
    ComponentID compId = COMPONENT_Cb;
    ChromaFormat chromaFormat = CHROMA_444;

    // Give all three planes same buffer, as only one of them is active in filterBlk.
    PelUnitBuf dstUnitBuf_ref{
        chromaFormat,
        areaBufDst_ref, // COMPONENT_Y
        areaBufDst_ref, // COMPONENT_Cb
        areaBufDst_ref  // COMPONENT_Cr
    };
    PelUnitBuf dstUnitBuf_opt{ chromaFormat, areaBufDst_opt, areaBufDst_opt, areaBufDst_opt };
    CPelUnitBuf srcUnitBuf{ chromaFormat, areaBufSrc, areaBufSrc, areaBufSrc };

    constexpr size_t numSampleChromaCoeff = 25;
    // Values taken from real codec runs.
    short chromaCoeffs[numSampleChromaCoeff][MAX_NUM_ALF_CHROMA_COEFF + 1] = {
        { -11, 2, 20, 3, -9, 23 },   { -10, 11, 14, 15, 10, 18 }, { -5, 0, 11, 5, -2, -4 },
        { -3, 8, 9, 9, 0, -23 },     { -10, 2, 13, 0, 13, 24 },   { -8, -2, 10, 4, -8, 18 },
        { 2, 2, -14, 3, 5, -14 },    { 2, 2, -7, 2, 3, -5 },      { 3, 2, -11, 5, 6, -14 },
        { -3, 1, 12, -2, 5, -4 },    { -2, -6, 33, -9, -8, 30 },  { -1, -5, 10, -5, 16, 8 },
        { -5, 11, 7, 6, -9, 20 },    { -11, 3, 12, 7, -8, 15 },   { -7, 2, 23, -3, -4, 37 },
        { -5, 6, 11, 14, -13, 17 },  { -8, 28, 21, 30, -10, 19 }, { -11, 2, 20, 3, -9, 23 },
        { -10, 11, 14, 15, 10, 18 }, { -6, 6, 16, 8, 15, 14 },    { -5, 13, 8, 11, -5, 7 },
        { -8, 34, 13, 11, -7, 11 },  { 15, -4, 4, -4, 10, -3 },   { -5, 16, 10, -2, -5, 8 },
        { 3, -5, -3, -3, 3, -1 } };

    short* chromaCoeff = chromaCoeffs[rng.get( 0, numSampleChromaCoeff - 1 )];

    std::vector<short> chrmClip( MAX_NUM_ALF_CHROMA_COEFF + 1, 0 );
    for( unsigned i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; ++i )
    {
      auto clip_idx = rng.get( 0, AdaptiveLoopFilter::MaxAlfNumClippingValues - 1 );
      chrmClip[i] = AdaptiveLoopFilter::m_alfClippVls[bitDepth - 8][clip_idx];
    }

    const int vbCTUHeight = rng.getOneOf<int>( { 16, 32 } );
    const int vbPos = vbCTUHeight - ALF_VB_POS_ABOVE_CTUROW_CHMA;

    // The classifier is unused for 5x5 chroma filtering, so pass nullptr.
    ref->m_filter5x5Blk( nullptr, dstUnitBuf_ref, srcUnitBuf, blk, compId, chromaCoeff, chrmClip.data(), clpRng,
                         vbCTUHeight, vbPos );
    opt->m_filter5x5Blk( nullptr, dstUnitBuf_opt, srcUnitBuf, blk, compId, chromaCoeff, chrmClip.data(), clpRng,
                         vbCTUHeight, vbPos );
  }

  return compare_values_2d( sstm.str(), dst_ref.data(), dst_opt.data(), h, w, (unsigned)dstStride );
}

template<AlfFilterType filtType>
static bool check_filterBlk( AdaptiveLoopFilter* ref, AdaptiveLoopFilter* opt, unsigned num_cases, int w, int h )
{
  printf( "Testing AdaptiveLoopFilter::filterBlk filterType=%s w=%d h=%d\n",
          filtType == ALF_FILTER_7 ? "ALF_FILTER_7" : "ALF_FILTER_5", w, h );

  DimensionGenerator rng;

  for( unsigned int bitDepth : { 8, 10 } )
  {
    InputGenerator<TCoeff> g{ bitDepth, /*is_signed=*/false };
    for( unsigned i = 0; i < num_cases; ++i )
    {
      unsigned srcStride = rng.get( w, MAX_CU_SIZE );
      unsigned dstStride = rng.get( w, MAX_CU_SIZE );

      if( !check_one_filterBlk<filtType>( ref, opt, srcStride, dstStride, w, h, bitDepth, g ) )
      {
        return false;
      }
    }
  }
  return true;
}

static bool test_AdaptiveLoopFilter()
{
  AdaptiveLoopFilter ref{ /*enableOpt=*/false };
  AdaptiveLoopFilter opt{ /*enableOpt=*/true };

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  for( unsigned w : { 8, 16, 32 } )
  {
    for( unsigned h : { 4, 8, 16, 24, 32 } )
    {
      passed = check_filterBlk<ALF_FILTER_7>( &ref, &opt, num_cases, w, h ) && passed;
    }
  }

  for( unsigned w = 4; w <= 64; w += 4 )
  {
    for( unsigned h : { 4, 8, 16, 24, 32, 40, 48, 56, 64 } )
    {
      passed = check_filterBlk<ALF_FILTER_5>( &ref, &opt, num_cases, w, h ) && passed;
    }
  }

  return passed;
}
#endif // ENABLE_SIMD_OPT_ALF

#if ENABLE_SIMD_OPT_MCIF
template<bool isLast, unsigned width>
static bool check_filterWxH_N8( InterpolationFilter* ref, InterpolationFilter* opt, unsigned height,
                                unsigned num_cases )
{
  static_assert( width == 4 || width == 8 || width == 16, "Width must be either 4, 8, or 16" );

  static constexpr unsigned bd = 10; // default bit-depth
  ClpRng clpRng{ ( int )bd };
  DimensionGenerator dim;
  InputGenerator<Pel> inp_gen{ bd, /*is_signed=*/false };

  // Max buffer size for src is ( height + 7 ) * srcStride.
  std::vector<Pel> src( ( MAX_CU_SIZE + 7 ) * ( MAX_CU_SIZE + 7 ) );
  std::vector<Pel> dst_ref( MAX_CU_SIZE * MAX_CU_SIZE );
  std::vector<Pel> dst_opt( MAX_CU_SIZE * MAX_CU_SIZE );

  bool passed = true;

  std::ostringstream sstm_test;
  sstm_test << "InterpolationFilter::filter" << width << "x" << height << "[0][" << isLast << "]";
  std::cout << "Testing " << sstm_test.str() << std::endl;

  for( unsigned n = 0; n < num_cases; n++ )
  {
    unsigned srcStride = dim.get( width, MAX_CU_SIZE ) + 7; // srcStride >= width + 7
    unsigned dstStride = dim.get( width, MAX_CU_SIZE );

    const TFilterCoeff *pCoeffH, *pCoeffV;
    if( width == 4 )
    {
      unsigned hCoeff_idx = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
      unsigned vCoeff_idx = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
      pCoeffH = InterpolationFilter::m_lumaFilter4x4[hCoeff_idx];
      pCoeffV = InterpolationFilter::m_lumaFilter4x4[vCoeff_idx];
    }
    else // Include lumaAltHpelIFilter for other widths.
    {
      unsigned hCoeff_idx = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS );
      unsigned vCoeff_idx = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS );
      pCoeffH = hCoeff_idx == LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS
                    ? InterpolationFilter::m_lumaAltHpelIFilter
                    : InterpolationFilter::m_lumaFilter[hCoeff_idx];
      pCoeffV = vCoeff_idx == LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS
                    ? InterpolationFilter::m_lumaAltHpelIFilter
                    : InterpolationFilter::m_lumaFilter[vCoeff_idx];
    }

    // Fill input buffers with unsigned data.
    std::generate( src.begin(), src.end(), inp_gen );

    // Clear output blocks.
    std::fill( dst_ref.begin(), dst_ref.end(), 0 );
    std::fill( dst_opt.begin(), dst_opt.end(), 0 );

    ptrdiff_t src_offset = 3 * ( 1 + srcStride );

    if( width == 4 )
    {
      ref->m_filter4x4[0][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_ref.data(),
                                   ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
      opt->m_filter4x4[0][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_opt.data(),
                                   ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
    }
    else if( width == 8 )
    {
      ref->m_filter8xH[0][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_ref.data(),
                                   ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
      opt->m_filter8xH[0][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_opt.data(),
                                   ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
    }
    else // width == 16
    {
      ref->m_filter16xH[0][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_ref.data(),
                                    ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
      opt->m_filter16xH[0][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_opt.data(),
                                    ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
    }

    std::ostringstream sstm_subtest;
    sstm_subtest << sstm_test.str() << " srcStride=" << srcStride << " dstStride=" << dstStride;

    passed =
        compare_values_2d( sstm_subtest.str(), dst_ref.data(), dst_opt.data(), height, width, (unsigned)dstStride ) && passed;
  }

  return passed;
}

static bool test_InterpolationFilter()
{
  InterpolationFilter ref;
  InterpolationFilter opt;

  ref.initInterpolationFilter( /*enable=*/false );
  opt.initInterpolationFilter( /*enable=*/true );

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  // The width = 4 case is only called with height = 4.
  passed = check_filterWxH_N8<false, 4>( &ref, &opt, 4, num_cases ) && passed;
  passed = check_filterWxH_N8<true, 4>( &ref, &opt, 4, num_cases ) && passed;
  for( unsigned height : { 4, 8, 16, 32, 64, 128 } )
  {
    passed = check_filterWxH_N8<false, 8>( &ref, &opt, height, num_cases ) && passed;
    passed = check_filterWxH_N8<true, 8>( &ref, &opt, height, num_cases ) && passed;
    passed = check_filterWxH_N8<false, 16>( &ref, &opt, height, num_cases ) && passed;
    passed = check_filterWxH_N8<true, 16>( &ref, &opt, height, num_cases ) && passed;
  }

  return passed;
}
#endif // ENABLE_SIMD_OPT_MCIF

#ifdef ENABLE_SIMD_OPT_BUFFER
static bool check_addAvg( PelBufferOps* ref, PelBufferOps* opt, unsigned num_cases )
{
  static constexpr unsigned bd = 10;
  ClpRng clpRng{ bd };
  DimensionGenerator dim;
  InputGenerator<Pel> inp_gen{ 16, false };
  const unsigned shiftNum = std::max<int>( 2, IF_INTERNAL_PREC - bd ) + 1;
  const int offset = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;

  static constexpr size_t buf_size = MAX_CU_SIZE * MAX_CU_SIZE;

  // Use xMalloc to create aligned buffers.
  Pel* src0 = ( Pel* )xMalloc( Pel, buf_size );
  Pel* src1 = ( Pel* )xMalloc( Pel, buf_size );
  Pel* dest_ref = ( Pel* )xMalloc( Pel, buf_size );
  Pel* dest_opt = ( Pel* )xMalloc( Pel, buf_size );

  bool passed = true;

  // Test addAvg with strides.
  for( int height : { 4, 8, 16, 24, 32, 64 } )
  {
    for( int width : { 4, 8, 12, 16, 20, 24, 32, 40, 48, 64 } )
    {
      std::ostringstream sstm_test;
      sstm_test << "PelBufferOps::addAvg(strided)"
                << " w=" << width << " h=" << height;
      std::cout << "Testing " << sstm_test.str() << std::endl;

      for( unsigned n = 0; n < num_cases; n++ )
      {
        // Set random strides >= width.
        const int src0Stride = dim.get( width, MAX_CU_SIZE );
        const int src1Stride = dim.get( width, MAX_CU_SIZE );
        const int destStride = dim.get( width, MAX_CU_SIZE );

        // Fill input buffers with unsigned 16-bit data from generator.
        std::generate( src0, src0 + buf_size, inp_gen );
        std::generate( src1, src1 + buf_size, inp_gen );

        // Clear output blocks.
        memset( dest_ref, 0, buf_size * sizeof( Pel ) );
        memset( dest_opt, 0, buf_size * sizeof( Pel ) );

        if( ( width & 15 ) == 0 )
        {
          ref->addAvg16( src0, src0Stride, src1, src1Stride, dest_ref, destStride, width, height, shiftNum, offset,
                         clpRng );
          opt->addAvg16( src0, src0Stride, src1, src1Stride, dest_opt, destStride, width, height, shiftNum, offset,
                         clpRng );
        }
        else if( ( width & 7 ) == 0 )
        {
          ref->addAvg8( src0, src0Stride, src1, src1Stride, dest_ref, destStride, width, height, shiftNum, offset,
                        clpRng );
          opt->addAvg8( src0, src0Stride, src1, src1Stride, dest_opt, destStride, width, height, shiftNum, offset,
                        clpRng );
        }
        else if( ( width & 3 ) == 0 )
        {
          ref->addAvg4( src0, src0Stride, src1, src1Stride, dest_ref, destStride, width, height, shiftNum, offset,
                        clpRng );
          opt->addAvg4( src0, src0Stride, src1, src1Stride, dest_opt, destStride, width, height, shiftNum, offset,
                        clpRng );
        }
        else // Shouldn't come here.
        {
          THROW_FATAL( "Unsupported size" );
        }

        std::ostringstream sstm_subtest;
        sstm_subtest << sstm_test.str() << " src0Stride=" << src0Stride << " src1Stride=" << src1Stride
                     << " destStride=" << destStride;

        passed = compare_values_2d( sstm_subtest.str(), dest_ref, dest_opt, height, width, destStride ) && passed;
      }
    }
  }

  xFree( src0 );
  xFree( src1 );
  xFree( dest_ref );
  xFree( dest_opt );

  return passed;
}

static bool test_PelBufferOps()
{
  PelBufferOps ref;
  PelBufferOps opt;

#if defined( TARGET_SIMD_X86 )
  opt.initPelBufOpsX86();
#endif
#if defined( TARGET_SIMD_ARM )
  opt.initPelBufOpsARM();
#endif

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  passed = check_addAvg( &ref, &opt, num_cases ) && passed;

  return passed;
}
#endif // ENABLE_SIMD_OPT_BUFFER

struct UnitTestEntry
{
  std::string name;
  bool ( *fn )();
};

static const UnitTestEntry test_suites[] = {
#if ENABLE_SIMD_OPT_ALF
    { "ALF", test_AdaptiveLoopFilter },
#endif
#if ENABLE_SIMD_OPT_MCIF
    { "InterpolationFilter", test_InterpolationFilter },
#endif
#if ENABLE_SIMD_OPT_BUFFER
    { "PelBufferOps", test_PelBufferOps },
#endif
};

int main( int argc, char* argv[] )
{
  srand( ( unsigned )time( NULL ) );

  bool passed = true;

  for( const auto& entry : test_suites )
  {
    std::cout << "Running test suite: " << entry.name << "\n";
    passed = entry.fn() && passed;
  }

  if( !passed )
  {
    printf( "\nerror: some tests failed!\n\n" );
    exit( EXIT_FAILURE );
  }

  printf( "\nsuccess: all tests passed!\n\n" );
}
