#pragma once

#ifdef TARGET_SIMD_X86

#include <simde/x86/sse2.h>
#include <cstdint>

namespace vvdec
{

#ifdef MISSING_INTRIN_mm_storeu_si16
static inline void _mm_storeu_si16( void* p, __m128i a )
{
  *(short*) ( p ) = (short) _mm_cvtsi128_si32( a );
}
#endif

#ifdef MISSING_INTRIN_mm_storeu_si32
static inline void _mm_storeu_si32( void* p, __m128i a )
{
  *(int32_t*)p = _mm_cvtsi128_si32( a );
}
#endif

#ifdef MISSING_INTRIN_mm_storeu_si64
static inline void _mm_storeu_si64( void* p, __m128i a )
{
  _mm_storel_epi64( (__m128i*)p, a);
}
#endif

#ifdef MISSING_INTRIN_mm_loadu_si32
static inline __m128i _mm_loadu_si32( const void* p )
{
  return _mm_cvtsi32_si128( *(int32_t*)p );
}
#endif

#ifdef MISSING_INTRIN_mm_loadu_si64
static inline __m128i _mm_loadu_si64( const void* p )
{
  return _mm_loadl_epi64( (const __m128i*)p );
}
#endif

// this should only be true for non-x86 architectures
#ifdef MISSING_INTRIN_mm256_zeroupper
#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( __i386 ) || defined( _M_IX86 )
#error MISSING_INTRIN_mm256_zeroupper should not be defined on x86
#endif

static inline void _mm256_zeroupper() {}  // NOOP
#endif

}   // namespace vvdec

#endif   // TARGET_SIMD_X86
