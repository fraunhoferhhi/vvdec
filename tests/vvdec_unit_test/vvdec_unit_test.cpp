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

#include <iostream>
#include <limits.h>

#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/InterpolationFilter.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/TrQuant_EMT.h"

using namespace vvdec;

#define NUM_CASES 100

template<typename T>
static inline bool compare_value( const std::string& context, const T ref, const T opt )
{
  if( opt != ref )
  {
    std::cerr << "failed: " << context << "\n"
              << "  mismatch:  ref=" << +ref << "  opt=" << +opt << "\n";
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
                << "  mismatch:  ref[" << idx << "]=" << +ref[idx] << "  opt[" << idx << "]=" << +opt[idx] << "\n";
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
                  << "  mismatch:  ref[" << row << "*" << stride << "+" << col << "]=" << +ref[idx] << "  opt[" << row
                  << "*" << stride << "+" << col << "]=" << +opt[idx] << "\n";
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

template<typename T>
class TrafoGenerator
{
public:
  explicit TrafoGenerator( unsigned bits ) : m_bits( bits )
  {
  }

  T operator()() const
  {
    return ( rand() & ( ( 1 << m_bits ) - 1 ) ) - ( 1 << m_bits >> 1 );
  }

private:
  unsigned m_bits;
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

  bool getBool() const
  {
    return get( 0, 1 ) != 0;
  }

  template<typename T>
  T getOneOf( const std::vector<T>& values ) const
  {
    CHECK( values.empty(), "getOneOf: values vector must not be empty" );
    return values[rand() % values.size()];
  }
};

#if ENABLE_SIMD_TCOEFF_OPS
template<typename G, typename T>
static bool check_one_fastInvCore( TCoeffOps* ref, TCoeffOps* opt, unsigned idx, unsigned trSize, unsigned lines,
                                   unsigned reducedLines, unsigned rows, G input_generator, T trafo_generator )
{
  CHECK( lines == 0, "Lines must be non-zero." );
  CHECK( reducedLines > lines, "ReducedLines must be less than or equal to lines." );
  CHECK( rows == 0, "Rows must be non-zero." );
  CHECK( rows > trSize, "Rows must not be larger than transformation size." );

  std::ostringstream sstm;
  sstm << "fastInvCore trSize=" << trSize << " lines=" << lines << " reducedLines=" << reducedLines << " rows=" << rows;

  TMatrixCoeff* it = ( TMatrixCoeff* )xMalloc( TMatrixCoeff, trSize * trSize );
  TCoeff* src = ( TCoeff* )xMalloc( TCoeff, trSize * lines );
  TCoeff* dst_ref = ( TCoeff* )xMalloc( TCoeff, trSize * lines );
  TCoeff* dst_opt = ( TCoeff* )xMalloc( TCoeff, trSize * lines );

  // First `rows` of coefficients are non-zero, remainder are zero.
  std::generate_n( it, rows * trSize, trafo_generator );
  std::fill_n( it + rows * trSize, (trSize - rows) * trSize, 0 );

  std::generate_n( src, trSize * lines, input_generator );
  memset( dst_ref, 0, trSize * lines * sizeof( TCoeff ) );
  memset( dst_opt, 0, trSize * lines * sizeof( TCoeff ) );

  ref->fastInvCore[idx]( it, src, dst_ref, lines, reducedLines, rows );
  opt->fastInvCore[idx]( it, src, dst_opt, lines, reducedLines, rows );

  const bool passed = compare_values_2d( sstm.str(), dst_ref, dst_opt, trSize, lines );

  xFree( it );
  xFree( src );
  xFree( dst_ref );
  xFree( dst_opt );

  return passed;
}

static bool check_fastInvCore( TCoeffOps* ref, TCoeffOps* opt, unsigned num_cases, unsigned idx, unsigned trSize )
{
  printf( "Testing TCoeffOps::fastInvCore trSize=%d\n", trSize );

  InputGenerator<TCoeff> g{ 16 };
  TrafoGenerator<TMatrixCoeff> t{ 8 };
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    unsigned lines = 1 << rng.get( 1, 6 );
    unsigned reducedLines = lines == 2 ? lines : std::min( 32u, rng.get( 4, lines, 4 ) );
    // In real decoding rows may be either a multiple of four, or exactly one.
    unsigned rows = std::max(1u, rng.get( 0, trSize, 4 ));
    if( !check_one_fastInvCore( ref, opt, idx, trSize, lines, reducedLines, rows, g, t ) )
    {
      return false;
    }
  }

  return true;
}

static bool test_TCoeffOps()
{
  TCoeffOps ref;
  TCoeffOps opt;

#if defined( TARGET_SIMD_X86 )
  opt.initTCoeffOpsX86();
#endif
#if defined( TARGET_SIMD_ARM )
  opt.initTCoeffOpsARM();
#endif

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  passed = check_fastInvCore( &ref, &opt, num_cases, 0, 4 ) && passed;
  passed = check_fastInvCore( &ref, &opt, num_cases, 1, 8 ) && passed;
  passed = check_fastInvCore( &ref, &opt, num_cases, 2, 16 ) && passed;
  passed = check_fastInvCore( &ref, &opt, num_cases, 3, 32 ) && passed;
  passed = check_fastInvCore( &ref, &opt, num_cases, 4, 64 ) && passed;

  return passed;
}
#endif // ENABLE_SIMD_TCOEFF_OPS

#if ENABLE_SIMD_OPT_ALF
template<typename G>
static bool check_one_deriveClassificationBlk( AdaptiveLoopFilter* ref, AdaptiveLoopFilter* opt, ptrdiff_t srcStride,
                                               unsigned int w, unsigned int h, int x, int y, G input_generator )
{
  CHECK( srcStride < w, "OrgStride must be greater than or equal to width" );

  std::ostringstream sstm;
  sstm << "deriveClassificationBlk srcStride=" << srcStride << " w=" << w << " h=" << h;

  DimensionGenerator rng;

  const Area blk{ x, y, w, h };

  // Padding to src memory so that deriveClassificationBlk can safely index [-3,+3] rows.
  constexpr int pad = 3;

  std::vector<Pel> src( ( y + h + 2 * pad ) * srcStride );
  std::generate( src.begin(), src.end(), input_generator );

  Size sz{ w, h };
  CPelBuf areaBufSrc{ src.data() + pad * srcStride + pad, srcStride, sz };

  std::array<AlfClassifier, AdaptiveLoopFilter::m_CLASSIFICATION_ARR_SIZE> classifier_ref;
  std::array<AlfClassifier, AdaptiveLoopFilter::m_CLASSIFICATION_ARR_SIZE> classifier_opt;

  const int vbCTUHeight = rng.getOneOf<int>( { 32, 64, 128 } );
  const int vbPos = vbCTUHeight - ALF_VB_POS_ABOVE_CTUROW_LUMA;

  const int shift = rng.getOneOf<int>( { 8, 10 } ) + 4;
  ref->m_deriveClassificationBlk( classifier_ref.data(), areaBufSrc, blk, shift, vbCTUHeight, vbPos );
  opt->m_deriveClassificationBlk( classifier_opt.data(), areaBufSrc, blk, shift, vbCTUHeight, vbPos );

  bool passed = true;
  // Optimized version writes extra data in the struct, so compare only the used part.
  for( int i = 0; i < blk.height / 4; ++i )
  {
    for( int j = 0; j < blk.width / 4; ++j )
    {
      int index = i * ( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE / 4 ) + j;
      const AlfClassifier& refCl = classifier_ref[index];
      const AlfClassifier& optCl = classifier_opt[index];

      passed = compare_value( sstm.str(), refCl.classIdx, optCl.classIdx ) && passed;
      passed = compare_value( sstm.str(), refCl.transposeIdx, optCl.transposeIdx ) && passed;
    }
  }

  return passed;
}

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

static bool check_deriveClassificationBlk( AdaptiveLoopFilter* ref, AdaptiveLoopFilter* opt, unsigned num_cases, int w,
                                           int h )
{
  printf( "Testing AdaptiveLoopFilter::check_deriveClassificationBlk w=%d h=%d\n", w, h );

  DimensionGenerator rng;
  InputGenerator<TCoeff> g{ 10, /*is_signed=*/false };
  for( unsigned i = 0; i < num_cases; ++i )
  {
    unsigned x = rng.get( 0, MAX_CU_SIZE, 8 );
    unsigned y = rng.get( 0, MAX_CU_SIZE, 8 );

    constexpr int padL = 3;
    constexpr int padR = 15; // Allow extra right-side samples for 8-lane SIMD loads.

    // Ensure each row is wide enough for the block position (x), block width (w), and left/right padding.
    unsigned srcStride = std::max<unsigned>( rng.get( w, MAX_CU_SIZE ), x + w + padL + padR );

    if( !check_one_deriveClassificationBlk( ref, opt, srcStride, w, h, x, y, g ) )
    {
      return false;
    }
  }

  return true;
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
      unsigned srcStride = rng.get( w + 4, MAX_CU_SIZE );  // +4 to prevent overreads in x86 simd implemenation
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
      passed = check_deriveClassificationBlk( &ref, &opt, num_cases, w, h ) && passed;
    }
  }

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

#if ENABLE_SIMD_DBLF
template<typename G>
static bool check_one_xPelFilterLuma( LoopFilter* ref, LoopFilter* opt, ptrdiff_t stride, bool isVertical,
                                      bool isStrong, int bitDepth, int tc, bool bFilterSecondP, bool bFilterSecondQ,
                                      G input_generator, std::ostringstream& sstm_test )
{
  static constexpr unsigned buf_size = MAX_CU_SIZE * MAX_CU_SIZE;
  std::vector<Pel> src_ref( buf_size );
  std::generate( src_ref.begin(), src_ref.end(), input_generator );
  std::vector<Pel> src_opt = src_ref;

  const ClpRng clpRng{ bitDepth };

  // From xEdgeFilterLuma: weak-filter delta cutoff is derived as 10 * tc.
  const int iThrCut = tc * 10;

  // Adjust src pointers to allow indexing up to (-4 * offset) within xPelFilterLuma.
  Pel* ptr_ref = src_ref.data() + 4 * stride;
  Pel* ptr_opt = src_opt.data() + 4 * stride;

  const ptrdiff_t step   = isVertical ? stride : 1;
  const ptrdiff_t offset = isVertical ? 1 : stride;

  ref->xPelFilterLuma( ptr_ref, step, offset, tc, isStrong, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );
  opt->xPelFilterLuma( ptr_opt, step, offset, tc, isStrong, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );

  std::ostringstream sstm_subtest;
  sstm_subtest << sstm_test.str() << ( isVertical ? " step=" : " offset=" ) << stride << " tc=" << tc
               << " secondP=" << bFilterSecondP << " secondQ=" << bFilterSecondQ;
  return compare_values_1d( sstm_subtest.str(), src_ref.data(), src_opt.data(), buf_size );
}

static bool check_xPelFilterLuma( LoopFilter* ref, LoopFilter* opt, unsigned num_cases, bool isVertical, bool isStrong,
                                  int bitDepth )
{
  DimensionGenerator rng;
  InputGenerator<Pel> inp_gen{ ( unsigned )bitDepth, /*is_signed=*/false };
  bool passed = true;

  std::ostringstream sstm_test;
  sstm_test << "LoopFilter::pelFilterLuma" << ( isStrong ? "Strong" : "Weak" ) << ( isVertical ? "Ver" : "Hor" )
            << " bd=" << bitDepth;
  std::cout << "Testing " << sstm_test.str() << std::endl;

  for( unsigned n = 0; n < num_cases; n++ )
  {
    const ptrdiff_t stride = ( ptrdiff_t )rng.get( 8, MAX_CU_SIZE );

    // sm_tcTable[MAX_QP + DEFAULT_INTRA_TC_OFFSET] is 395; xEdgeFilterLuma scales it by bit depth.
    static constexpr int maxTc10Bit = 395;
    const int maxTc = bitDepth < 10 ? ( ( maxTc10Bit + ( 1 << ( 9 - bitDepth ) ) ) >> ( 10 - bitDepth ) )
                                    : ( maxTc10Bit << ( bitDepth - 10 ) );
    const int iTc = ( int )rng.get( 0, maxTc );

    const bool bFilterSecondP = rng.getBool();
    const bool bFilterSecondQ = rng.getBool();

    passed = check_one_xPelFilterLuma( ref, opt, stride, isVertical, isStrong, bitDepth, iTc, bFilterSecondP,
                                       bFilterSecondQ, inp_gen, sstm_test ) && passed;
  }

  return passed;
}

static bool test_LoopFilter()
{
  LoopFilter ref{ /*enableOpt=*/false };
  LoopFilter opt{ /*enableOpt=*/true };

  unsigned num_cases = NUM_CASES;
  bool passed = true;

  for( int bitDepth : { 8, 10 } )
  {
    for( bool isVertical : { false, true } )
    {
      for( bool isStrong : { false, true } )
      {
        passed = check_xPelFilterLuma( &ref, &opt, num_cases, isVertical, isStrong, bitDepth ) && passed;
      }
    }
  }

  return passed;
}
#endif // ENABLE_SIMD_DBLF

#if ENABLE_SIMD_OPT_MCIF
template<int N, bool isVertical, bool isFirst, bool isLast>
static bool check_filter( InterpolationFilter* ref, InterpolationFilter* opt, unsigned width, unsigned height )
{
  static_assert( N == 2 || N == 4 || N == 6 || N == 8, "Supported taps: 2/4/6/8" );

  const unsigned padN = N == 6 ? 8 : N;

  std::string str_VerHor = isVertical ? "Ver" : "Hor";

  static constexpr unsigned bd = 10; // default bit-depth
  ClpRng clpRng{ ( int )bd };
  DimensionGenerator dim;
  InputGenerator<Pel> inp_gen{ bd, /*is_signed=*/false };

  // Max buffer size is ( height + 8 ) * srcStride.
  static constexpr unsigned BUF_SIZE = ( MAX_CU_SIZE + 8 ) * ( MAX_CU_SIZE + 8 );
  std::vector<Pel> src( BUF_SIZE );
  std::vector<Pel> dst_ref( BUF_SIZE );
  std::vector<Pel> dst_opt( BUF_SIZE );

  std::ostringstream sstm_test;
  sstm_test << "InterpolationFilter::filter" << str_VerHor << "_N" << N << "[" << isFirst << "][" << isLast << "]"
            << " width=" << width << " height=" << height;
  std::cout << "Testing " << sstm_test.str() << std::endl;

  const unsigned minSrcStride = isVertical ? width : width + padN - 1;
  const unsigned srcStride = dim.get( minSrcStride, MAX_CU_SIZE + 8 );
  const unsigned dstStride = dim.get( width, MAX_CU_SIZE + 8 );

  // Fill input buffers with unsigned data.
  std::generate( src.begin(), src.end(), inp_gen );

  const ptrdiff_t cStride = isVertical ? srcStride : 1;
  const ptrdiff_t src_offset = ( padN / 2 - 1 ) * cStride;

  unsigned frac, tapIdx;
  const TFilterCoeff* pCoeff;

  if( N == 8 )
  {
    tapIdx = 0;
    frac = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
    pCoeff = InterpolationFilter::m_lumaFilter[frac];
  }
  else if( N == 6 )
  {
    tapIdx = 0; // 6-tap coeffs are simply passed onto the 8-tap filters.
    frac = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS );
    pCoeff = frac == LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS ? InterpolationFilter::m_lumaAltHpelIFilter
                                                                    : InterpolationFilter::m_lumaFilter4x4[frac];
  }
  else if( N == 4 )
  {
    tapIdx = 1;
    frac = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
    pCoeff = InterpolationFilter::m_chromaFilter[frac << 1];
  }
  else // N == 2
  {
    tapIdx = 2;
    frac = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
    pCoeff = InterpolationFilter::m_bilinearFilterPrec4[frac];
  }

  if( isVertical )
  {
    ref->m_filterVer[tapIdx][isFirst][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_ref.data(),
                                               ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeff );
    opt->m_filterVer[tapIdx][isFirst][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_opt.data(),
                                               ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeff );
  }
  else // Horizontal
  {
    ref->m_filterHor[tapIdx][isFirst][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_ref.data(),
                                               ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeff );
    opt->m_filterHor[tapIdx][isFirst][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_opt.data(),
                                               ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeff );
  }

  std::ostringstream sstm_subtest;
  sstm_subtest << sstm_test.str() << " srcStride=" << srcStride << " dstStride=" << dstStride << " frac=" << frac;

  return compare_values_2d( sstm_subtest.str(), dst_ref.data(), dst_opt.data(), height, dstStride );
}

template<bool isLast, unsigned width>
static bool check_filterWxH_N8( InterpolationFilter* ref, InterpolationFilter* opt, unsigned height,
                                unsigned num_cases )
{
  static_assert( width == 4 || width == 8 || width == 16, "Width must be either 4, 8, or 16" );

  static constexpr unsigned bd = 10; // Default bit-depth.
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

template<bool isLast, unsigned width>
static bool check_filterWxH_N4( InterpolationFilter* ref, InterpolationFilter* opt, unsigned height,
                                unsigned num_cases )
{
  static_assert( width == 4 || width == 8 || width == 16, "Width must be either 4, 8, or 16" );

  static constexpr unsigned bd = 10; // default bit-depth
  ClpRng clpRng{ ( int )bd };
  DimensionGenerator dim;
  InputGenerator<Pel> inp_gen{ bd, /*is_signed=*/false };

  std::vector<Pel> src( MAX_CU_SIZE * MAX_CU_SIZE );
  std::vector<Pel> dst_ref( MAX_CU_SIZE * MAX_CU_SIZE );
  std::vector<Pel> dst_opt( MAX_CU_SIZE * MAX_CU_SIZE );

  bool passed = true;

  std::ostringstream sstm_test;
  sstm_test << "InterpolationFilter::filter" << width << "x" << height << "[1][" << isLast << "]";
  std::cout << "Testing " << sstm_test.str() << std::endl;

  for( unsigned n = 0; n < num_cases; n++ )
  {
    unsigned srcStride = dim.get( width + 3, MAX_CU_SIZE ); // srcStride >= width + 3
    unsigned dstStride = dim.get( width, MAX_CU_SIZE );

    unsigned hCoeff_idx = dim.get( 0, CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
    unsigned vCoeff_idx = dim.get( 0, CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
    const TFilterCoeff* pCoeffH = InterpolationFilter::m_chromaFilter[hCoeff_idx];
    const TFilterCoeff* pCoeffV = InterpolationFilter::m_chromaFilter[vCoeff_idx];

    // Fill input buffers with unsigned data.
    std::generate( src.begin(), src.end(), inp_gen );

    // Clear output blocks.
    std::fill( dst_ref.begin(), dst_ref.end(), 0 );
    std::fill( dst_opt.begin(), dst_opt.end(), 0 );

    ptrdiff_t src_offset = 1 + srcStride;

    if( width == 4 )
    {
      ref->m_filter4x4[1][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_ref.data(),
                                   ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
      opt->m_filter4x4[1][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_opt.data(),
                                   ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
    }
    else if( width == 8 )
    {
      ref->m_filter8xH[1][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_ref.data(),
                                   ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
      opt->m_filter8xH[1][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_opt.data(),
                                   ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
    }
    else // width == 16
    {
      ref->m_filter16xH[1][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_ref.data(),
                                    ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
      opt->m_filter16xH[1][isLast]( clpRng, src.data() + src_offset, ( ptrdiff_t )srcStride, dst_opt.data(),
                                    ( ptrdiff_t )dstStride, ( int )width, ( int )height, pCoeffH, pCoeffV );
    }

    std::ostringstream sstm_subtest;
    sstm_subtest << sstm_test.str() << " srcStride=" << srcStride << " dstStride=" << dstStride;

    passed =
        compare_values_2d( sstm_subtest.str(), dst_ref.data(), dst_opt.data(), height, width, dstStride ) && passed;
  }

  return passed;
}

template<bool isFirst, bool isLast>
static bool check_filterCopy( InterpolationFilter* ref, InterpolationFilter* opt, unsigned num_cases, bool biMCForDMVR )
{
  DimensionGenerator dim;

  // Scale buffer size to allow for width=(MAX_CU_SIZE + 4) if biMCForDMVR.
  static constexpr size_t buf_size = MAX_CU_SIZE * (MAX_CU_SIZE + 4);

  std::vector<Pel> src( buf_size );
  std::vector<Pel> dst_ref( buf_size );
  std::vector<Pel> dst_opt( buf_size );

  bool passed = true;

  // Test unsigned 8-bit and 10-bit.
  for( unsigned bd : { 8, 10 } )
  {
    ClpRng clpRng{ ( int )bd };
    InputGenerator<Pel> inp_gen{ bd, /*is_signed=*/false };

    std::ostringstream sstm_test;
    sstm_test << "InterpolationFilter::filterCopy[" << isFirst << "][" << isLast << "]"
              << " biMCForDMVR=" << std::boolalpha << biMCForDMVR << " bitDepth=" << bd;
    std::cout << "Testing " << sstm_test.str() << std::endl;

    for( unsigned n = 0; n < num_cases; n++ )
    {
      // In biMCForDMVR cases, width must be a multiple of eight, plus four.
      const unsigned height = dim.get( 1, MAX_CU_SIZE );
      const unsigned width  = biMCForDMVR ? dim.get( 0, MAX_CU_SIZE, 8 ) + 4 : dim.get( 1, MAX_CU_SIZE );

      // Extend stride maximum to ensure stride >= width if biMCForDMVR.
      const unsigned srcStride = dim.get( width, MAX_CU_SIZE + 4 );
      const unsigned dstStride = dim.get( width, MAX_CU_SIZE + 4 );

      // Fill input buffers with unsigned data.
      std::generate( src.begin(), src.end(), inp_gen );

      // Clear output blocks.
      std::fill( dst_ref.begin(), dst_ref.end(), 0 );
      std::fill( dst_opt.begin(), dst_opt.end(), 0 );

      ref->m_filterCopy[isFirst][isLast]( clpRng, src.data(), ( ptrdiff_t )srcStride, dst_ref.data(),
                                          ( ptrdiff_t )dstStride, ( int )width, ( int )height, biMCForDMVR );
      opt->m_filterCopy[isFirst][isLast]( clpRng, src.data(), ( ptrdiff_t )srcStride, dst_opt.data(),
                                          ( ptrdiff_t )dstStride, ( int )width, ( int )height, biMCForDMVR );

      std::ostringstream sstm_subtest;
      sstm_subtest << sstm_test.str() << " srcStride=" << srcStride << " dstStride=" << dstStride << " w=" << width
                   << " h=" << height;

      passed =
          compare_values_2d( sstm_subtest.str(), dst_ref.data(), dst_opt.data(), height, width, dstStride ) && passed;
    }
  }

  return passed;
}

static bool check_filterN2_2D( InterpolationFilter* ref, InterpolationFilter* opt, unsigned width, unsigned height,
                               unsigned num_cases )
{
  static constexpr unsigned bd = 10;
  ClpRng clpRng{ ( int )bd };
  DimensionGenerator dim;
  InputGenerator<Pel> inp_gen{ bd, /*is_signed=*/false };
  const int w = width + 4;  // Input width is the CU width + 4.
  const int h = height + 4; // Input height is the CU height + 4.

  const int bufferSize = ( MAX_CU_SIZE + 5 ) * ( MAX_CU_SIZE + 5 );
  std::vector<Pel> src( bufferSize );
  std::vector<Pel> dst_ref( bufferSize );
  std::vector<Pel> dst_opt( bufferSize );

  bool passed = true;

  std::ostringstream sstm_test;
  sstm_test << "InterpolationFilter::filterN2_2D " << width << "x" << height;
  std::cout << "Testing " << sstm_test.str() << std::endl;

  for( unsigned n = 0; n < num_cases; n++ )
  {
    unsigned srcStride = dim.get( w, MAX_CU_SIZE + 4 );
    unsigned dstStride = dim.get( w, MAX_CU_SIZE + 4 );

    unsigned fracX = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
    unsigned fracY = dim.get( 0, LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS - 1 );
    const TFilterCoeff* ch = InterpolationFilter::m_bilinearFilterPrec4[fracX];
    const TFilterCoeff* cv = InterpolationFilter::m_bilinearFilterPrec4[fracY];

    std::generate( src.begin(), src.end(), inp_gen );
    std::fill( dst_ref.begin(), dst_ref.end(), 0 );
    std::fill( dst_opt.begin(), dst_opt.end(), 0 );

    ref->m_filterN2_2D( clpRng, src.data(), srcStride, dst_ref.data(), dstStride, w, h, ch, cv );
    opt->m_filterN2_2D( clpRng, src.data(), srcStride, dst_opt.data(), dstStride, w, h, ch, cv );

    std::ostringstream sstm_subtest;
    sstm_subtest << sstm_test.str() << " width=" << w << " height=" << h << " srcStride=" << srcStride
                 << " dstStride=" << dstStride << " fracX=" << fracX << " fracY=" << fracY;

    passed = compare_values_2d( sstm_subtest.str(), dst_ref.data(), dst_opt.data(), h, w, dstStride ) && passed;
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

  // filterHor and filterVer
  for( unsigned height : { 1, 4, 8, 16, 32, 64, 128 } )
  {
    for( unsigned width : { 1, 4, 8, 16, 32, 64, 128 } )
    {
      // Luma 6-tap
      passed = check_filter<6, false, true, false>( &ref, &opt, width, height + 5 ) && passed; // height + N - 1
      passed = check_filter<6, false, true, true>( &ref, &opt, width, height ) && passed;
      passed = check_filter<6, true, false, false>( &ref, &opt, width, height ) && passed;
      passed = check_filter<6, true, false, true>( &ref, &opt, width, height ) && passed;
      passed = check_filter<6, true, true, false>( &ref, &opt, width, height ) && passed;
      passed = check_filter<6, true, true, true>( &ref, &opt, width, height ) && passed;

      // Luma 8-tap
      passed = check_filter<8, false, true, false>( &ref, &opt, width, height + 7 ) && passed; // height + N - 1
      passed = check_filter<8, false, true, true>( &ref, &opt, width, height ) && passed;
      passed = check_filter<8, true, false, false>( &ref, &opt, width, height ) && passed;
      passed = check_filter<8, true, false, true>( &ref, &opt, width, height ) && passed;
      passed = check_filter<8, true, true, false>( &ref, &opt, width, height ) && passed;
      passed = check_filter<8, true, true, true>( &ref, &opt, width, height ) && passed;
    }
  }
  for( unsigned height : { 1, 2, 4, 8, 16, 32, 64, 128 } )
  {
    for( unsigned width : { 1, 2, 4, 8, 12, 16, 20, 24, 32, 64, 128 } )
    {
      // Chroma 4-tap
      passed = check_filter<4, false, true, false>( &ref, &opt, width, height + 3 ) && passed; // height + N - 1
      passed = check_filter<4, false, true, true>( &ref, &opt, width, height ) && passed;
      passed = check_filter<4, true, false, false>( &ref, &opt, width, height ) && passed;
      passed = check_filter<4, true, false, true>( &ref, &opt, width, height ) && passed;
      passed = check_filter<4, true, true, false>( &ref, &opt, width, height ) && passed;
      passed = check_filter<4, true, true, true>( &ref, &opt, width, height ) && passed;
    }
  }
  for( unsigned height : { 8, 16, 32, 64, 128 } )
  {
    for( unsigned width : { 8, 16, 32, 64, 128 } )
    {
      // Bilinear 2-tap: height and width are padded by four.
      passed = check_filter<2, false, true, false>( &ref, &opt, width + 4, height + 4 ) && passed;
      passed = check_filter<2, true, true, false>( &ref, &opt, width + 4, height + 4 ) && passed;
    }
  }

  // filterWxH_N8
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

  // filterWxH_N4
  // The width = 4 case is only called with height = 4.
  passed = check_filterWxH_N4<false, 4>( &ref, &opt, 4, num_cases ) && passed;
  passed = check_filterWxH_N4<true, 4>( &ref, &opt, 4, num_cases ) && passed;
  for( unsigned height : { 2, 4, 8, 16, 32 } )
  {
    passed = check_filterWxH_N4<false, 8>( &ref, &opt, height, num_cases ) && passed;
    passed = check_filterWxH_N4<true, 8>( &ref, &opt, height, num_cases ) && passed;
    passed = check_filterWxH_N4<false, 16>( &ref, &opt, height, num_cases ) && passed;
    passed = check_filterWxH_N4<true, 16>( &ref, &opt, height, num_cases ) && passed;
  }

  passed = check_filterCopy<false, false>( &ref, &opt, num_cases, false ) && passed;
  passed = check_filterCopy<false, true>( &ref, &opt, num_cases, false ) && passed;
  passed = check_filterCopy<true, false>( &ref, &opt, num_cases, false ) && passed;
  passed = check_filterCopy<true, true>( &ref, &opt, num_cases, false ) && passed;
  // The biMCForDMVR case for filterCopy only uses isFirst=true, isLast=false configuration.
  passed = check_filterCopy<true, false>( &ref, &opt, num_cases, true ) && passed;

  // filterN2_2D
  for( unsigned height : { 8, 16, 32, 64, 128 } )
  {
    for( unsigned width : { 8, 16, 32, 64, 128 } )
    {
      passed = check_filterN2_2D( &ref, &opt, width, height, num_cases ) && passed;
    }
  }

  return passed;
}
#endif // ENABLE_SIMD_OPT_MCIF

#if ENABLE_SIMD_OPT_BUFFER
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

static bool check_applyLut( PelBufferOps* ref, PelBufferOps* opt, unsigned num_cases )
{
  DimensionGenerator rng;
  InputGenerator<TCoeff> g{ 14, /*is_signed=*/false };
  bool passed = true;

  static constexpr size_t buf_size = MAX_CU_SIZE * MAX_CU_SIZE;
  Pel* src_ref = ( Pel* )xMalloc( Pel, buf_size );
  Pel* src_opt = ( Pel* )xMalloc( Pel, buf_size );
  Pel* lut = ( Pel* )xMalloc( Pel, UINT16_MAX );

  for( unsigned w : { 8, 16, 32, 64, 128 } )
  {
    for( unsigned h : { 8, 16, 32, 64, 80, 96, 112, 128 } )
    {
      printf( "Testing PelBufferOps::applyLut=%d h=%d\n", w, h );
      for( unsigned i = 0; i < num_cases; ++i )
      {
        unsigned stride = rng.get( w, MAX_CU_SIZE, w );

        std::ostringstream sstm;
        sstm << "applyLut stride=" << stride << " w=" << w << " h=" << h;

        std::generate( src_ref, src_ref + buf_size, g );
        std::memcpy( src_opt, src_ref, sizeof( Pel ) * buf_size );
        std::generate( lut, lut + UINT16_MAX, g );

        ref->applyLut( src_ref, stride, w, h, lut );
        opt->applyLut( src_opt, stride, w, h, lut );

        passed = compare_values_2d( sstm.str(), src_ref, src_opt, h, w, stride ) && passed;
      }
    }
  }

  xFree( src_ref );
  xFree( src_opt );
  xFree( lut );
  return passed;
}

static bool check_one_rspFwdCore( PelBufferOps* ref, PelBufferOps* opt, unsigned num_cases, unsigned width,
                                  unsigned height, unsigned bd )
{
  DimensionGenerator dim;
  InputGenerator<Pel> inp_gen{ bd, /*is_signed=*/false };

  const unsigned reshapeLUTSize = 1 << bd;
  const Pel initCW = reshapeLUTSize / PIC_CODE_CW_BINS; // bd=8:16, bd=10:64

  static constexpr size_t size = MAX_CU_SIZE * MAX_CU_SIZE;

  std::vector<Pel> src_ref( size );
  std::vector<Pel> src_opt( size );

  std::vector<Pel> LmcsPivot( PIC_CODE_CW_BINS + 1 );
  std::vector<Pel> ScaleCoeff( PIC_CODE_CW_BINS );
  std::vector<Pel> InputPivot( PIC_CODE_CW_BINS + 1 );

  bool passed = true;

  std::ostringstream sstm_test;
  sstm_test << "rspFwdCore w=" << width << " h=" << height << " bd=" << bd;
  std::cout << "Testing " << sstm_test.str() << std::endl;

  for( unsigned n = 0; n < num_cases; n++ )
  {
    std::generate( src_ref.begin(), src_ref.end(), inp_gen );
    src_opt = src_ref;

    std::generate( LmcsPivot.begin(), LmcsPivot.end(), inp_gen );
    std::generate( ScaleCoeff.begin(), ScaleCoeff.end(), inp_gen );
    std::generate( InputPivot.begin(), InputPivot.end(), inp_gen );

    // Set random strides >= width.
    const unsigned stride = dim.get( width, MAX_CU_SIZE );

    opt->rspFwd( src_ref.data(), ( ptrdiff_t )stride, ( int )width, ( int )height, ( int )bd, initCW, LmcsPivot.data(),
                 ScaleCoeff.data(), InputPivot.data() );
    ref->rspFwd( src_opt.data(), ( ptrdiff_t )stride, ( int )width, ( int )height, ( int )bd, initCW, LmcsPivot.data(),
                 ScaleCoeff.data(), InputPivot.data() );

    std::ostringstream sstm_subtest;
    sstm_subtest << sstm_test.str() << " stride=" << stride;

    passed = compare_values_2d( sstm_subtest.str(), src_ref.data(), src_opt.data(), height, width, stride ) && passed;
  }

  return passed;
}

bool check_rspFwdCore( PelBufferOps* ref, PelBufferOps* opt, unsigned num_cases )
{
  bool passed = true;

  for( unsigned height : { 4, 8, 16, 32, 64, 128 } )
  {
    for( unsigned width : { 4, 8, 16, 32, 64, 128 } )
    {
      for( unsigned bd : { 8, 10 } )
      {
        passed = check_one_rspFwdCore( ref, opt, num_cases, width, height, bd ) && passed;
      }
    }
  }

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
  passed = check_rspFwdCore( &ref, &opt, num_cases ) && passed;
  passed = check_applyLut( &ref, &opt, num_cases ) && passed;

  return passed;
}
#endif // ENABLE_SIMD_OPT_BUFFER

#if ENABLE_SIMD_OPT_INTER
template<typename G>
static bool check_one_gradFilter( InterPrediction* ref, InterPrediction* opt, int width, int height, int srcStride,
                                  int gradStride, G input_generator, bool padding )
{
  std::ostringstream sstm;
  sstm << "gradFilter width=" << width << " height=" << height << " padding=" << padding;

  int srcSize;
  int gradSize;

  if( padding )
  {
    srcSize = height * srcStride;
    gradSize = height * gradStride;
  }
  else
  {
    srcSize = ( height + 2 ) * srcStride;
    gradSize = height * gradStride;
  }

  std::vector<Pel> src_1( srcSize );
  std::vector<Pel> src_2( srcSize );

  std::vector<Pel> gradX_ref( gradSize );
  std::vector<Pel> gradY_ref( gradSize );
  std::vector<Pel> gradX_opt( gradSize );
  std::vector<Pel> gradY_opt( gradSize );

  std::generate( src_1.begin(), src_1.end(), input_generator );
  src_2 = src_1;

  Pel* srcPtr_1 = padding ? src_1.data() : src_1.data() + srcStride + 1;
  Pel* srcPtr_2 = padding ? src_2.data() : src_2.data() + srcStride + 1;

  const int bitDepth = 10; // Unused in gradFilter.

  if( padding )
  {
    ref->BioGradFilter( srcPtr_1, srcStride, width, height, gradStride, gradX_ref.data(), gradY_ref.data(), bitDepth );
    opt->BioGradFilter( srcPtr_2, srcStride, width, height, gradStride, gradX_opt.data(), gradY_opt.data(), bitDepth );
  }
  else
  {
    ref->profGradFilter( srcPtr_1, srcStride, width, height, gradStride, gradX_ref.data(), gradY_ref.data(), bitDepth );
    opt->profGradFilter( srcPtr_2, srcStride, width, height, gradStride, gradX_opt.data(), gradY_opt.data(), bitDepth );
  }

  bool res_gradX =
      compare_values_1d( "Incorrect gradX buffer in " + sstm.str(), gradX_ref.data(), gradX_opt.data(), gradSize );
  bool res_gradY =
      compare_values_1d( "Incorrect gradY buffer in " + sstm.str(), gradY_ref.data(), gradY_opt.data(), gradSize );
  bool res_src = compare_values_1d( "Incorrect src buffer in " + sstm.str(), src_1.data(), src_2.data(), srcSize );

  return res_gradX && res_gradY && res_src;
}

template<typename G>
static bool check_one_biOptFlow( InterPrediction* ref, InterPrediction* opt, int width, int height, ptrdiff_t dstStride,
                                 int shiftNum, int offset, int limit, const ClpRng& clpRng, unsigned bitDepth,
                                 G input_generator )
{
  CHECK( width % 8, "Width must be a multiple of eight" );
  CHECK( height % 8, "Height must be a multiple of eight" );

  std::ostringstream sstm;
  sstm << "BiOptFlow width=" << width << " height=" << height << " shift=" << shiftNum << " offset=" << offset
       << " limit=" << limit;

  int stridePredMC = width + BIO_ALIGN_SIZE;

  // Copied from m_bdofBlock definition in InterPrediction.h.
  constexpr int BDOF_BLOCK_SIZE = ( MAX_BDOF_APPLICATION_REGION + ( 2 * BIO_ALIGN_SIZE + BIO_ALIGN_SIZE ) + 16 ) *
                                  ( MAX_BDOF_APPLICATION_REGION + ( 2 * BIO_EXTEND_SIZE + 2 ) + 2 );

  std::vector<Pel> srcY0( BDOF_BLOCK_SIZE );
  std::vector<Pel> srcY1( BDOF_BLOCK_SIZE );
  std::vector<Pel> gradX0( BIO_TEMP_BUFFER_SIZE );
  std::vector<Pel> gradX1( BIO_TEMP_BUFFER_SIZE );
  std::vector<Pel> gradY0( BIO_TEMP_BUFFER_SIZE );
  std::vector<Pel> gradY1( BIO_TEMP_BUFFER_SIZE );
  std::vector<Pel> dstYref( dstStride * height );
  std::vector<Pel> dstYopt( dstStride * height );

  std::generate( srcY0.begin(), srcY0.end(), input_generator );
  std::generate( srcY1.begin(), srcY1.end(), input_generator );
  std::generate( gradX0.begin(), gradX0.end(), input_generator );
  std::generate( gradX1.begin(), gradX1.end(), input_generator );
  std::generate( gradY0.begin(), gradY0.end(), input_generator );
  std::generate( gradY1.begin(), gradY1.end(), input_generator );

  const Pel* src0 = srcY0.data() + stridePredMC;
  const Pel* src1 = srcY1.data() + stridePredMC;

  ref->BiOptFlow( src0, src1, gradX0.data(), gradX1.data(), gradY0.data(), gradX1.data(), width, height, dstYref.data(),
                  dstStride, shiftNum, offset, limit, clpRng, bitDepth );

  opt->BiOptFlow( src0, src1, gradX0.data(), gradX1.data(), gradY0.data(), gradX1.data(), width, height, dstYopt.data(),
                  dstStride, shiftNum, offset, limit, clpRng, bitDepth );

  return compare_values_2d( sstm.str(), dstYref.data(), dstYopt.data(), height, ( unsigned )dstStride );
}

static bool check_biOptFlow( InterPrediction* ref, InterPrediction* opt, unsigned num_cases, int width, int height )
{
  printf( "Testing InterPrediction::BiOptFlow w=%d h=%d\n", width, height );
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    // Width is either 8 or 16.
    // DstStride is a multiple of eight in the range width to 128 inclusive.
    unsigned dstStride = rng.get( width, 128, 8 );

    for( int bitDepth : { 8, 10 } )
    {
      InputGenerator<Pel> g{ ( unsigned )bitDepth };
      const int shiftNum = IF_INTERNAL_PREC + 1 - bitDepth;
      const int offset = ( 1 << ( shiftNum - 1 ) ) + 2 * IF_INTERNAL_OFFS;
      const int limit = ( 1 << 4 ) - 1;
      ClpRng clpRng{ bitDepth };

      if( !check_one_biOptFlow( ref, opt, width, height, dstStride, shiftNum, offset, limit, clpRng, bitDepth, g ) )
      {
        return false;
      }
    }
  }

  return true;
}

struct GradFilterParameter
{
  int width, height, srcStride, gradStride;
  bool padding;
};

static constexpr GradFilterParameter gradFilter_parameters[] = {
    { 10, 18, 16, 16, true },
    { 18, 10, 24, 24, true },
    { 18, 18, 24, 24, true },
    { 4, 4, 6, 4, false },
};

static bool check_gradFilter( InterPrediction* ref, InterPrediction* opt, unsigned num_cases )
{
  InputGenerator<Pel> g{ 14 }; // Signed 14 bit.

  bool passed = true;

  for( const auto& test : gradFilter_parameters )
  {
    printf( "Testing InterPred::gradFilter w=%d h=%d padding=%d\n", test.width, test.height, test.padding );
    for( unsigned i = 0; i < num_cases; ++i )
    {
      passed =
          check_one_gradFilter( ref, opt, test.width, test.height, test.srcStride, test.gradStride, g, test.padding ) &&
          passed;
    }
  }

  return passed;
}

static bool check_one_prefetchPad( const InterPrediction* ref, const InterPrediction* opt, int width, int height,
                                   ptrdiff_t srcStride, ptrdiff_t dstStride, int prefetchPadIdx, int padSize )
{
  std::ostringstream sstm;
  sstm << "xPrefetchPad width=" << width << " height=" << height << " padSize=" << padSize;

  InputGenerator<Pel> g{ 14 };
  std::vector<Pel> src( srcStride * height );

  int dstHeight = height + 2 * padSize;
  std::vector<Pel> dst_ref( dstStride * dstHeight );
  std::vector<Pel> dst_opt( dstStride * dstHeight );

  std::generate( src.begin(), src.end(), g );

  // Calculate pointer to actual image area in destination.
  Pel* dst_ref_img = dst_ref.data() + padSize * dstStride + padSize;
  Pel* dst_opt_img = dst_opt.data() + padSize * dstStride + padSize;

  ref->prefetchPad[prefetchPadIdx]( src.data(), srcStride, dst_ref_img, dstStride, width, height );
  opt->prefetchPad[prefetchPadIdx]( src.data(), srcStride, dst_opt_img, dstStride, width, height );

  const int checkWidth = width + 2 * padSize;
  return compare_values_2d( sstm.str(), dst_ref.data(), dst_opt.data(), dstHeight, checkWidth, ( unsigned )dstStride );
}

static bool check_prefetchPad( const InterPrediction* ref, const InterPrediction* opt, unsigned num_cases, int width,
                               int height, int prefetchPadIdx, int padSize )
{
  printf( "Testing InterPrediction::xPrefetchPad w=%d h=%d padSize=%d\n", width, height, padSize );
  DimensionGenerator rng;

  for( unsigned i = 0; i < num_cases; ++i )
  {
    ptrdiff_t srcStride = rng.get( width + 8, 128 );
    ptrdiff_t dstStride = rng.get( width + 2 * padSize + 8, 128 );

    if( !check_one_prefetchPad( ref, opt, width, height, srcStride, dstStride, prefetchPadIdx, padSize ) )
    {
      return false;
    }
  }

  return true;
}

static bool test_InterPrediction()
{
  RdCost pcRdCost;
  unsigned num_cases = NUM_CASES;
  bool passed = true;

  // The value of these do not affect gradFilter,
  // it is only needed for initialization.
  ChromaFormat chromaFormatIDC = CHROMA_444;
  const int ctuSize = 1;

  InterPrediction ref;
  InterPrediction opt;
  ref.init( &pcRdCost, chromaFormatIDC, ctuSize, /*enableOpt=*/false );
  opt.init( &pcRdCost, chromaFormatIDC, ctuSize, /*enableOpt=*/true );

  passed = check_biOptFlow( &ref, &opt, num_cases, 8, 16 ) && passed;
  passed = check_biOptFlow( &ref, &opt, num_cases, 16, 8 ) && passed;
  passed = check_biOptFlow( &ref, &opt, num_cases, 16, 16 ) && passed;

  passed = check_gradFilter( &ref, &opt, num_cases ) && passed;

  // Test prefetchPad[2] where padSize=1.
  for( unsigned width : { 7, 11 } )
  {
    for( unsigned height : { 7, 11 } )
    {
      passed = check_prefetchPad( &ref, &opt, num_cases, width, height, 2, 1 ) && passed;
    }
  }

  // Test prefetchPad[0] where padSize=2.
  for( unsigned width : { 15, 23 } )
  {
    for( unsigned height : { 15, 23 } )
    {
      passed = check_prefetchPad( &ref, &opt, num_cases, width, height, 0, 2 ) && passed;
    }
  }

  return passed;
}
#endif // ENABLE_SIMD_OPT_INTER

#if ENABLE_SIMD_OPT_DIST

static bool check_SAD( RdCost* ref, RdCost* opt, unsigned num_cases, int width, int height )
{
  std::ostringstream sstm;
  sstm << "RdCost::xGetSAD" << width << " w=" << width << " h=" << height;
  printf( "Testing %s\n", sstm.str().c_str() );

  DimensionGenerator rng;
  InputGenerator<Pel> inp_gen{ 10, /*is_signed=*/false };

  bool passed = true;
  for( unsigned i = 0; i < num_cases; i++ )
  {
    int stride = rng.get( width, 1024 );

    // subShift is always 1 in real decodings (set in InterPrediction::xProcessDMVR).
    constexpr int subShift = 1;

    std::vector<Pel> orgBuf( stride * height );
    std::vector<Pel> curBuf( stride * height );

    std::generate( orgBuf.begin(), orgBuf.end(), inp_gen );
    std::generate( curBuf.begin(), curBuf.end(), inp_gen );

    DistParam dtRef, dtOpt;
    ref->setDistParam( dtRef, orgBuf.data(), curBuf.data(), stride, stride, /*bitDepth=*/10, width, height, subShift );
    opt->setDistParam( dtOpt, orgBuf.data(), curBuf.data(), stride, stride, /*bitDepth=*/10, width, height, subShift );

    Distortion sum_ref = dtRef.distFunc( dtRef );
    Distortion sum_opt = dtOpt.distFunc( dtOpt );

    passed = compare_value( sstm.str(), sum_ref, sum_opt ) && passed;
  }
  return passed;
}

static bool check_SADX5( RdCost* ref, RdCost* opt, unsigned num_cases, int width, int height, bool isCalCentrePos )
{
  std::ostringstream sstm;
  sstm << "RdCost::xGetSAD" << width << "X5"
       << " w=" << width << " h=" << height << " isCalCentrePos=" << std::boolalpha << isCalCentrePos;
  printf( "Testing %s\n", sstm.str().c_str() );

  DimensionGenerator rng;
  InputGenerator<Pel> inp_gen{ 10, /*is_signed=*/false };

  constexpr int kMargin = 4; // per-row horizontal margin (X5 kernel moves the ptr +/-4).

  bool passed = true;
  for( unsigned i = 0; i < num_cases; i++ )
  {
    // subShift is always 1 in real decodings (set in InterPrediction::xProcessDMVR).
    constexpr int subShift = 1;
    const int minStride = width + 2 * kMargin;
    const int stride = rng.get( minStride, 1024 );

    std::vector<Pel> orgBuf( stride * height );
    std::vector<Pel> curBuf( stride * height );

    std::generate( orgBuf.begin(), orgBuf.end(), inp_gen );
    std::generate( curBuf.begin(), curBuf.end(), inp_gen );

    const Pel* orgPtr = orgBuf.data() + kMargin;
    const Pel* curPtr = curBuf.data() + kMargin;

    DistParam dtRef;
    DistParam dtOpt;
    ref->setDistParam( dtRef, orgPtr, curPtr, stride, stride, /*bitDepth=*/10, width, height, subShift );
    opt->setDistParam( dtOpt, orgPtr, curPtr, stride, stride, /*bitDepth=*/10, width, height, subShift );

    std::array<Distortion, 5> costRef;
    std::array<Distortion, 5> costOpt;

    dtRef.distFuncX5( dtRef, costRef.data(), isCalCentrePos );
    dtOpt.distFuncX5( dtOpt, costOpt.data(), isCalCentrePos );

    for( int k = 0; k < 5; k++ )
    {
      if( !isCalCentrePos && k == 2 )
      {
        continue;
      }
      passed = compare_value( sstm.str(), costRef[k], costOpt[k] ) && passed;
    }
  }
  return passed;
}

static bool test_RdCost()
{
  RdCost ref{ /*enableOpt=*/false };
  RdCost opt{ /*enableOpt=*/true };

  unsigned num_cases = NUM_CASES;
  bool passed = true;
  std::array<int, 2> dims = { 8, 16 };

  for( int h : dims )
  {
    for( int w : dims )
    {
      passed = check_SAD( &ref, &opt, num_cases, w, h ) && passed;

      passed = check_SADX5( &ref, &opt, num_cases, w, h, /*isCalCentrePos=*/true ) && passed;
      passed = check_SADX5( &ref, &opt, num_cases, w, h, /*isCalCentrePos=*/false ) && passed;
    }
  }
  return passed;
}
#endif // ENABLE_SIMD_OPT_DIST

#if ENABLE_SIMD_OPT_SAO
struct SaoAvailCase
{
  bool left;
  bool right;
  bool above;
  bool below;
  const char* name;
};

struct SaoVirtualBoundaryCase
{
  unsigned numVertical;
  unsigned numHorizontal;
  enum Placement
  {
    Middle,
    NearEdge,
    BlockStart,
    BlockEnd,
  } placement;
  const char* name;
};

static void fillVirtualBoundaryPositions( int positions[2], int& numPositions, unsigned size, unsigned numRequested,
                                          SaoVirtualBoundaryCase::Placement placement )
{
  CHECK( numRequested > 2, "SAO unit test supports up to two virtual boundaries" );

  numPositions = ( int )numRequested;

  if( numRequested == 0 )
  {
    return;
  }

  if( numRequested == 1 )
  {
    switch( placement )
    {
    case SaoVirtualBoundaryCase::NearEdge:
      positions[0] = 1;
      break;
    case SaoVirtualBoundaryCase::BlockStart:
      positions[0] = 0;
      break;
    case SaoVirtualBoundaryCase::BlockEnd:
      positions[0] = ( int )size;
      break;
    default:
      positions[0] = ( int )size / 2;
      break;
    }
  }
  else // numRequested == 2
  {
    switch( placement )
    {
    case SaoVirtualBoundaryCase::NearEdge:
      positions[0] = 1;
      positions[1] = ( int )size - 1;
      break;
    case SaoVirtualBoundaryCase::BlockStart:
      positions[0] = 0;
      positions[1] = 1;
      break;
    case SaoVirtualBoundaryCase::BlockEnd:
      positions[0] = ( int )size - 1;
      positions[1] = ( int )size;
      break;
    default:
      positions[0] = ( int )size / 3;
      positions[1] = ( int )( 2 * size ) / 3;
      break;
    }
  }
}

static bool check_one_offsetBlock( SampleAdaptiveOffset* ref, SampleAdaptiveOffset* opt, int bitDepth, int typeIdx,
                                   unsigned width, unsigned height, const SaoAvailCase& avail,
                                   const SaoVirtualBoundaryCase& vbCase, unsigned caseIdx,
                                   std::ostringstream& sstm_test )
{
  DimensionGenerator rng;
  InputGenerator<Pel> inputGenerator{ ( unsigned )bitDepth, /*is_signed=*/false };
  const ClpRng clpRng{ bitDepth };

  CHECK( width % 8 != 0, "SAO offsetBlock expects 8-aligned widths" );

  // Source padding for EO neighbor reads around the tested block.
  static constexpr unsigned padLeft = 1;
  static constexpr unsigned padRight = 1;
  static constexpr unsigned padTop = 1;
  static constexpr unsigned padBottom = 1;
  const unsigned srcStride = padLeft + width + padRight;
  const unsigned srcRows = padTop + height + padBottom;
  const unsigned dstStride = width;
  const unsigned dstRows = height;

  // Use aligned buffers.
  Pel* src = ( Pel* )xMalloc( Pel, srcRows * srcStride );
  Pel* dstRef = ( Pel* )xMalloc( Pel, dstRows * dstStride );
  Pel* dstOpt = ( Pel* )xMalloc( Pel, dstRows * dstStride );

  std::generate_n( src, srcRows * srcStride, inputGenerator );

  // Point srcBlk inside the padded buffer so EO modes can read neighboring samples.
  const Pel* srcBlk = src + padTop * srcStride + padLeft;

  // Pre-fill dst with source block values to match the real resBlk precondition.
  // EO modes leave boundary/disabled samples unchanged, and those samples are still checked.
  for( unsigned y = 0; y < height; y++ )
  {
    std::copy_n( srcBlk + y * srcStride, width, dstRef + y * dstStride );
    std::copy_n( srcBlk + y * srcStride, width, dstOpt + y * dstStride );
  }

  int offset[MAX_NUM_SAO_CLASSES] = { 0 };
  int startIdx = 0;
  const int maxOffset = SampleAdaptiveOffset::getMaxOffsetQVal( bitDepth );
  if( typeIdx == SAO_TYPE_BO )
  {
    startIdx = rng.get( 0, MAX_NUM_SAO_CLASSES - 1 );
    for( int i = 0; i < 4; i++ )
    {
      // BO uses offsets for 4 consecutive bands starting at startIdx, with wraparound.
      offset[( startIdx + i ) % MAX_NUM_SAO_CLASSES] = rng.get( 0, 2 * maxOffset ) - maxOffset;
    }
  }
  else
  {
    for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
    {
      offset[i] = rng.get( 0, 2 * maxOffset ) - maxOffset;
    }
    offset[SAO_CLASS_EO_PLAIN] = 0;
  }

  std::vector<int8_t> signLineBuf1Ref( MAX_CU_SIZE + 2 );
  std::vector<int8_t> signLineBuf2Ref( MAX_CU_SIZE + 2 );
  std::vector<int8_t> signLineBuf1Opt( MAX_CU_SIZE + 2 );
  std::vector<int8_t> signLineBuf2Opt( MAX_CU_SIZE + 2 );

  int horVirBndryPos[2] = { 0, 0 };
  int verVirBndryPos[2] = { 0, 0 };
  int numHorVirBndry = 0;
  int numVerVirBndry = 0;
  fillVirtualBoundaryPositions( horVirBndryPos, numHorVirBndry, height, vbCase.numHorizontal, vbCase.placement );
  fillVirtualBoundaryPositions( verVirBndryPos, numVerVirBndry, width, vbCase.numVertical, vbCase.placement );
  const bool isCtuCrossedByVirtualBoundaries = numHorVirBndry > 0 || numVerVirBndry > 0;

  const bool aboveLeft = avail.above && avail.left;
  const bool aboveRight = avail.above && avail.right;
  const bool belowLeft = avail.below && avail.left;
  const bool belowRight = avail.below && avail.right;

  ref->offsetBlock( bitDepth, clpRng, typeIdx, offset, startIdx, srcBlk, dstRef, srcStride, dstStride, width, height,
                    avail.left, avail.right, avail.above, avail.below, aboveLeft, aboveRight, belowLeft, belowRight,
                    &signLineBuf1Ref, &signLineBuf2Ref, isCtuCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos,
                    numHorVirBndry, numVerVirBndry );
  opt->offsetBlock( bitDepth, clpRng, typeIdx, offset, startIdx, srcBlk, dstOpt, srcStride, dstStride, width, height,
                    avail.left, avail.right, avail.above, avail.below, aboveLeft, aboveRight, belowLeft, belowRight,
                    &signLineBuf1Opt, &signLineBuf2Opt, isCtuCrossedByVirtualBoundaries, horVirBndryPos, verVirBndryPos,
                    numHorVirBndry, numVerVirBndry );

  std::ostringstream sstm_subtest;
  sstm_subtest << sstm_test.str() << " avail=" << avail.name << " vb=" << vbCase.name << " case=" << caseIdx;

  const bool passed = compare_values_2d( sstm_subtest.str(), dstRef, dstOpt, height, width, dstStride );

  xFree( src );
  xFree( dstRef );
  xFree( dstOpt );

  return passed;
}

static bool check_offsetBlock( SampleAdaptiveOffset* ref, SampleAdaptiveOffset* opt, unsigned numCases, int bitDepth,
                               int typeIdx, unsigned width, unsigned height )
{
  static constexpr const char* typeNames[] = { "EO_0", "EO_90", "EO_135", "EO_45", "BO" };

  std::ostringstream sstm_test;
  sstm_test << "SampleAdaptiveOffset::offsetBlock bd=" << bitDepth << " typeIdx=" << typeNames[typeIdx]
            << " w=" << width << " h=" << height;
  std::cout << "Testing " << sstm_test.str() << std::endl;

  static constexpr SaoAvailCase availCases[] = {
      { false, false, false, false, "l0_r0_a0_b0"  },
      { false, false, false, true,  "l0_r0_a0_b1"  },
      { false, false, true,  false, "l0_r0_a1_b0"  },
      { false, false, true,  true,  "l0_r0_a1_b1"  },
      { false, true,  false, false, "l0_r1_a0_b0"  },
      { false, true,  false, true,  "l0_r1_a0_b1"  },
      { false, true,  true,  false, "l0_r1_a1_b0"  },
      { false, true,  true,  true,  "l0_r1_a1_b1"  },
      { true,  false, false, false, "l1_r0_a0_b0"  },
      { true,  false, false, true,  "l1_r0_a0_b1"  },
      { true,  false, true,  false, "l1_r0_a1_b0"  },
      { true,  false, true,  true,  "l1_r0_a1_b1"  },
      { true,  true,  false, false, "l1_r1_a0_b0"  },
      { true,  true,  false, true,  "l1_r1_a0_b1"  },
      { true,  true,  true,  false, "l1_r1_a1_b0"  },
      { true,  true,  true,  true,  "allAvailCase" },
  };
  static constexpr SaoVirtualBoundaryCase vbCases[] = {
      { 0, 0, SaoVirtualBoundaryCase::Middle,     "none"            },
      { 1, 0, SaoVirtualBoundaryCase::Middle,     "vertical"        },
      { 0, 1, SaoVirtualBoundaryCase::Middle,     "horizontal"      },
      { 1, 1, SaoVirtualBoundaryCase::Middle,     "both"            },
      { 1, 1, SaoVirtualBoundaryCase::NearEdge,   "bothNearEdge"    },
      { 1, 1, SaoVirtualBoundaryCase::BlockStart, "bothBlockStart"  },
      { 1, 1, SaoVirtualBoundaryCase::BlockEnd,   "bothBlockEnd"    },
      { 2, 0, SaoVirtualBoundaryCase::Middle,     "twoVertical"     },
      { 0, 2, SaoVirtualBoundaryCase::Middle,     "twoHorizontal"   },
      { 2, 2, SaoVirtualBoundaryCase::Middle,     "twoEach"         },
      { 2, 2, SaoVirtualBoundaryCase::NearEdge,   "twoEachNearEdge" },
  };

  bool passed = true;
  constexpr unsigned allAvailCaseIdx = sizeof( availCases ) / sizeof( availCases[0] ) - 1;
  const unsigned numAvailCases = typeIdx == SAO_TYPE_BO ? 1 : sizeof( availCases ) / sizeof( availCases[0] );
  constexpr unsigned numVbCases = sizeof( vbCases ) / sizeof( vbCases[0] );
  const unsigned numTestCases = std::max( numCases, numAvailCases * numVbCases );

  for( unsigned caseIdx = 0; caseIdx < numTestCases; caseIdx++ )
  {
    const SaoAvailCase& avail =
        typeIdx == SAO_TYPE_BO ? availCases[allAvailCaseIdx] : availCases[caseIdx % numAvailCases];
    const SaoVirtualBoundaryCase& vbCase = vbCases[( caseIdx / numAvailCases ) % numVbCases];
    passed = check_one_offsetBlock( ref, opt, bitDepth, typeIdx, width, height, avail, vbCase, caseIdx, sstm_test ) &&
             passed;
  }
  return passed;
}

static bool test_SampleAdaptiveOffset()
{
  SampleAdaptiveOffset ref{ /*enableOpt=*/false };
  SampleAdaptiveOffset opt{ /*enableOpt=*/true };

  unsigned numCases = NUM_CASES;
  bool passed = true;

  for( int bitDepth : { 8, 10 } )
  {
    for( int typeIdx : { SAO_TYPE_EO_0, SAO_TYPE_EO_90, SAO_TYPE_EO_135, SAO_TYPE_EO_45, SAO_TYPE_BO } )
    {
      for( unsigned h : { 8, 16, 32, 48, 64, 128 } )
      {
        for( unsigned w : { 8, 16, 24, 32, 40, 64, 72, 120, 128 } )
        {
          passed = check_offsetBlock( &ref, &opt, numCases, bitDepth, typeIdx, w, h ) && passed;
        }
      }
    }
  }

  return passed;
}
#endif // ENABLE_SIMD_OPT_SAO

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
#if ENABLE_SIMD_OPT_INTER
    { "InterPrediction", test_InterPrediction },
#endif
#if ENABLE_SIMD_DBLF
    { "LoopFilter", test_LoopFilter },
#endif
#if ENABLE_SIMD_OPT_BUFFER
    { "PelBufferOps", test_PelBufferOps },
#endif
#if ENABLE_SIMD_OPT_DIST
    { "RdCost", test_RdCost },
#endif
#if ENABLE_SIMD_OPT_SAO
    { "SAO", test_SampleAdaptiveOffset },
#endif
#if ENABLE_SIMD_TCOEFF_OPS
    { "TCoeffOps", test_TCoeffOps },
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
