include(CheckSymbolExists)

check_symbol_exists( _mm_storeu_si32 "immintrin.h" HAVE_INITIN_mm_storeu_si32 )
check_symbol_exists( _mm_storeu_si64 "immintrin.h" HAVE_INITIN_mm_storeu_si64 )
check_symbol_exists( _mm_loadu_si32  "immintrin.h" HAVE_INITIN_mm_loadu_si32  )
check_symbol_exists( _mm_loadu_si64  "immintrin.h" HAVE_INITIN_mm_loadu_si64  )

macro( add_compile_definition_if_var VAR )
  if( ${VAR} )
    add_compile_definitions( ${VAR} )
   endif()
endmacro()

add_compile_definition_if_var( HAVE_INITIN_mm_storeu_si32 )
add_compile_definition_if_var( HAVE_INITIN_mm_storeu_si64 )
add_compile_definition_if_var( HAVE_INITIN_mm_loadu_si32  )
add_compile_definition_if_var( HAVE_INITIN_mm_loadu_si64  )
