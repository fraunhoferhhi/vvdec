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

using namespace vvdec;

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

static bool test_empty()
{
  bool passed = true;

  return passed;
}

struct UnitTestEntry
{
  std::string name;
  bool ( *fn )();
};

static const UnitTestEntry test_suites[] = {
    { "Empty", test_empty },
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
