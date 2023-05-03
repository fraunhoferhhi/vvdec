# library
set( LIB_NAME vvdec )

# create upper case name
string( TOUPPER ${LIB_NAME} LIB_NAME_UC )

# create file version.h
if(PROJECT_VERSION_RC)
  configure_file( version-rc.h.in "${CMAKE_BINARY_DIR}/${LIB_NAME}/version.h" )
else()
  configure_file( version.h.in "${CMAKE_BINARY_DIR}/${LIB_NAME}/version.h" )
endif()

# get source files
file( GLOB BASE_SRC_FILES "*.cpp"  "../CommonLib/*.cpp"  "../Utilities/*.cpp" "../DecoderLib/*.cpp")

# get include files
file( GLOB BASE_INC_FILES "*.h" "../CommonLib/*.h"  "../Utilities/*.h" "../DecoderLib/*.h" )


if( NOT DEFINED VVDEC_ENABLE_X86_SIMD )
  message( FATAL_ERROR "VVDEC_ENABLE_X86_SIMD is not defined. Please include TopLevel CMakeLists.txt file of vvdec to set all dependencies!" )
endif()

if( VVDEC_ENABLE_X86_SIMD )
  # get x86 source files
  file( GLOB X86_SRC_FILES "../CommonLib/x86/*.cpp" )

  # get x86 include files
  file( GLOB X86_INC_FILES "../CommonLib/x86/*.h" )

  # get avx source files
  file( GLOB AVX_SRC_FILES "../CommonLib/x86/avx/*.cpp" )

  if( NOT VVDEC_TARGET_WASM )
    # get avx2 source files
    file( GLOB AVX2_SRC_FILES "../CommonLib/x86/avx2/*.cpp" )
  endif()

  # get sse4.1 source files
  file( GLOB SSE41_SRC_FILES "../CommonLib/x86/sse41/*.cpp" )

  # get sse4.2 source files
  file( GLOB SSE42_SRC_FILES "../CommonLib/x86/sse42/*.cpp" )
endif()

# get libmd5 source files
file( GLOB MD5_SRC_FILES "../libmd5/*.cpp" )

# get libmd5 include files
file( GLOB MD5_INC_FILES "../libmd5/*.h" )

# get public/extern include files
file( GLOB PUBLIC_INC_FILES  "../../../include/${LIB_NAME}/*.h" )

# get all source files
set( SRC_FILES ${BASE_SRC_FILES} ${X86_SRC_FILES} ${MD5_SRC_FILES} )

# get all include files
file( GLOB PRIVATE_INC_FILES ${BASE_INC_FILES} ${X86_INC_FILES} ${MD5_INC_FILES}  )

set( INC_FILES ${PRIVATE_INC_FILES} ${PUBLIC_INC_FILES}  )

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )

  # example: place header files in different folders
  source_group( "Natvis Files" FILES ${NATVIS_FILES} )
endif()

source_group( "Header Files"          FILES ${PUBLIC_INC_FILES} )
source_group( "Header Files\\private" FILES ${PRIVATE_INC_FILES} )

# set PRIVATE defines for all targets in this directory
add_compile_definitions( ${LIB_NAME_UC}_SOURCE )

# set PRIVATE include directories for all targets in this directory
include_directories( $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include> $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}> )
include_directories( . .. ../DecoderLib ../CommonLib ../CommonLib/x86 ../libmd5 )
include_directories( SYSTEM ../../../thirdparty )

# don't export all symbols from shared libraries by default (gcc: -fvisibility=hidden), only those marked as VVDEC_DECL
#  behavior similar to __declspec(dllexport) on windows
set( CMAKE_C_VISIBILITY_PRESET hidden )
set( CMAKE_CXX_VISIBILITY_PRESET hidden )

set( CMAKE_VISIBILITY_INLINES_HIDDEN TRUE )

if( VVDEC_ENABLE_X86_SIMD )
  # set needed compile definitions
  set_property( SOURCE ${SSE41_SRC_FILES} APPEND PROPERTY COMPILE_DEFINITIONS USE_SSE41 )
  set_property( SOURCE ${SSE42_SRC_FILES} APPEND PROPERTY COMPILE_DEFINITIONS USE_SSE42 )
  set_property( SOURCE ${AVX_SRC_FILES}   APPEND PROPERTY COMPILE_DEFINITIONS USE_AVX )
  set_property( SOURCE ${AVX2_SRC_FILES}  APPEND PROPERTY COMPILE_DEFINITIONS USE_AVX2 )
  # set needed compile flags
  if( MSVC )
    set_property( SOURCE ${AVX_SRC_FILES}   APPEND PROPERTY COMPILE_FLAGS "/arch:AVX" )
    set_property( SOURCE ${AVX2_SRC_FILES}  APPEND PROPERTY COMPILE_FLAGS "/arch:AVX2" )
  elseif( UNIX OR MINGW )
    include( vvdecCompilerSupport )

    if( NOT VVDEC_TARGET_WASM )
      set_if_compiler_supports_flag( FLAG_mxsave -mxsave )
      set_property( SOURCE ${X86_SRC_FILES} APPEND PROPERTY COMPILE_FLAGS ${FLAG_mxsave} )
    endif()

    set_if_compiler_supports_flag( FLAG_msse41 -msse4.1 )
    set_if_compiler_supports_flag( FLAG_msse42 -msse4.2 )
    set_if_compiler_supports_flag( FLAG_mavx   -mavx    )
    set_if_compiler_supports_flag( FLAG_mavx2  -mavx2   )
    set_property( SOURCE ${SSE41_SRC_FILES} APPEND PROPERTY COMPILE_FLAGS "${FLAG_msse41}" )
    set_property( SOURCE ${SSE42_SRC_FILES} APPEND PROPERTY COMPILE_FLAGS "${FLAG_msse42}" )
    set_property( SOURCE ${AVX_SRC_FILES}   APPEND PROPERTY COMPILE_FLAGS "${FLAG_mavx}"   )
    set_property( SOURCE ${AVX2_SRC_FILES}  APPEND PROPERTY COMPILE_FLAGS "${FLAG_mavx2}"  )
  endif()

  add_library( ${LIB_NAME}_x86_simd OBJECT ${SSE41_SRC_FILES} ${SSE42_SRC_FILES} ${AVX_SRC_FILES} ${AVX2_SRC_FILES} )
  # disble LTO for the files compiled with special architecture flags
  set_target_properties( ${LIB_NAME}_x86_simd PROPERTIES
                                              INTERPROCEDURAL_OPTIMIZATION                OFF
                                              INTERPROCEDURAL_OPTIMIZATION_RELEASE        OFF
                                              INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO OFF
                                              INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL     OFF )

  set_target_properties( ${LIB_NAME}_x86_simd PROPERTIES FOLDER lib )
endif()

if( VVDEC_ENABLE_TRACING )
  add_compile_definitions( ENABLE_TRACING=1 )
endif()

# set resource file for MSVC compilers
if( MSVC )
  set( RESOURCE_FILE ${LIB_NAME}.rc )
endif()

add_library( ${LIB_NAME} ${SRC_FILES} $<$<TARGET_EXISTS:${LIB_NAME}_x86_simd>:$<TARGET_OBJECTS:${LIB_NAME}_x86_simd>> ${INC_FILES} ${NATVIS_FILES} ${RESOURCE_FILE} )

target_compile_definitions( ${LIB_NAME} PUBLIC $<$<STREQUAL:$<TARGET_PROPERTY:${LIB_NAME},TYPE>,SHARED_LIBRARY>:${LIB_NAME_UC}_DYN_LINK> )

target_include_directories( ${LIB_NAME} SYSTEM INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include> $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}> )

target_link_libraries( ${LIB_NAME} ${INTEL_ITT_LINK_TARGET} Threads::Threads )

set_target_properties( ${LIB_NAME} PROPERTIES
                                   SOVERSION ${PROJECT_VERSION_MAJOR}
                                   VERSION   ${PROJECT_VERSION} )

# set the folder where to place the projects
set_target_properties( ${LIB_NAME} PROPERTIES FOLDER lib )

