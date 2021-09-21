include_guard(GLOBAL)

include(CheckCSourceCompiles)

macro( check_intrinsic symbol_name code )
  check_c_source_compiles("
    #include <immintrin.h>
    #include <stdint.h>
    int main()
    {
      ${code}
      return 0;
    }"
    HAVE_INITIN${symbol_name} )

  if( HAVE_INITIN${symbol_name} )
    add_compile_definitions( HAVE_INITIN${symbol_name} )
   endif()
endmacro()

check_intrinsic( _mm_storeu_si16 "int16_t a = 0; __m128i x = _mm_setzero_si128(); _mm_storeu_si16( &a, x );" )
check_intrinsic( _mm_storeu_si32 "int32_t a = 0; __m128i x = _mm_setzero_si128(); _mm_storeu_si32( &a, x );" )
check_intrinsic( _mm_storeu_si64 "int64_t a = 0; __m128i x = _mm_setzero_si128(); _mm_storeu_si64( &a, x );" )
check_intrinsic( _mm_loadu_si32  "int32_t a = 0; __m128i x = _mm_setzero_si128(); x = _mm_loadu_si32( &a );" )
check_intrinsic( _mm_loadu_si64  "int64_t a = 0; __m128i x = _mm_setzero_si128(); x = _mm_loadu_si64( &a );" )
