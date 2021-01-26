#pragma once

#include <immintrin.h>
#include <cstdint>

#ifndef HAVE_INITIN_mm_storeu_si32
static inline void _mm_storeu_si32( void* p, __m128i a )
{
  *(int32_t*)p = _mm_cvtsi128_si32( a );
}
#endif

#ifndef HAVE_INITIN_mm_storeu_si64
static inline void _mm_storeu_si64( void* p, __m128i a )
{
  *(int64_t*)p = _mm_cvtsi128_si64( a );
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
  return _mm_cvtsi64_si128( *(int64_t*)p );
}
#endif
