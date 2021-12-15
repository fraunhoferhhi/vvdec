#pragma once

#include <immintrin.h>
#include <cstdint>

namespace vvdec
{

#ifndef HAVE_INITIN_mm_storeu_si16
static inline void _mm_storeu_si16( void* p, __m128i a )
{
  ( void ) ( *( short* ) ( p ) = ( short ) _mm_cvtsi128_si32( a ) );
}
#endif

#ifndef HAVE_INITIN_mm_storeu_si32
static inline void _mm_storeu_si32( void* p, __m128i a )
{
  *(int32_t*)p = _mm_cvtsi128_si32( a );
}
#endif

#ifndef HAVE_INITIN_mm_storeu_si64
static inline void _mm_storeu_si64( void* p, __m128i a )
{
  _mm_storel_epi64( (__m128i*)p, a);
}
#endif

#ifndef HAVE_INITIN_mm_loadu_si32
static inline __m128i _mm_loadu_si32( const void* p )
{
  return _mm_cvtsi32_si128( *(int32_t*)p );
}
#endif

#ifndef HAVE_INITIN_mm_loadu_si64
static inline __m128i _mm_loadu_si64( const void* p )
{
  return _mm_loadl_epi64( (const __m128i*)p );
}
#endif

}