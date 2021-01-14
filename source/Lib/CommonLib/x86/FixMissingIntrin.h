#pragma once

#include <immintrin.h>
#include <cstdint>

#ifndef HAVE_INITIN_mm_storeu_si32
#define _mm_storeu_si32(p, a) (void)(*(int*)(p) = _mm_cvtsi128_si32((a)))
#endif

#ifndef HAVE_INITIN_mm_storeu_si64
#define _mm_storeu_si64(p, a) (_mm_storel_epi64((__m128i*)(p), (a)))
#endif

#ifndef HAVE_INITIN_mm_loadu_si32
#define _mm_loadu_si32(p) _mm_cvtsi32_si128( *(int32_t*)p )
#endif

#ifndef HAVE_INITIN_mm_loadu_si64
#define _mm_loadu_si64(p) _mm_cvtsi64_si128( *(int64_t*)p )
#endif
