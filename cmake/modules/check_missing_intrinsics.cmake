include(CheckCSourceCompiles)

check_c_source_compiles( "#include <immintrin.h>\n int main() { int32_t a = 0; __m128i x = _mm_setzero_si128(); _mm_storeu_si32( &a, x ); return 0; }" HAVE_INITIN_mm_storeu_si32 )
check_c_source_compiles( "#include <immintrin.h>\n int main() { int64_t a = 0; __m128i x = _mm_setzero_si128(); _mm_storeu_si64( &a, x ); return 0; }" HAVE_INITIN_mm_storeu_si64 )
check_c_source_compiles( "#include <immintrin.h>\n int main() { int32_t a = 0; __m128i x =                      _mm_loadu_si32( &a );     return 0; }" HAVE_INITIN_mm_loadu_si32  )
check_c_source_compiles( "#include <immintrin.h>\n int main() { int64_t a = 0; __m128i x =                      _mm_loadu_si64( &a );     return 0; }" HAVE_INITIN_mm_loadu_si64  )

macro( add_compile_definition_if_var VAR )
  if( ${VAR} )
    add_compile_definitions( ${VAR} )
   endif()
endmacro()

add_compile_definition_if_var( HAVE_INITIN_mm_storeu_si32 )
add_compile_definition_if_var( HAVE_INITIN_mm_storeu_si64 )
add_compile_definition_if_var( HAVE_INITIN_mm_loadu_si32  )
add_compile_definition_if_var( HAVE_INITIN_mm_loadu_si64  )
