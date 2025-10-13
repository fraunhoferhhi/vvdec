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
        compare_values_2d( sstm_subtest.str(), dst_ref.data(), dst_opt.data(), height, width, dstStride ) && passed;
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

struct UnitTestEntry
{
  std::string name;
  bool ( *fn )();
};

static const UnitTestEntry test_suites[] = {
#if ENABLE_SIMD_OPT_MCIF
    { "InterpolationFilter", test_InterpolationFilter },
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
