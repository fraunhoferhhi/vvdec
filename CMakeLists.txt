# Top level CMakeLists.txt
#
# minimum required cmake version
cmake_minimum_required( VERSION 3.12.0 FATAL_ERROR )

# Use latest policy
cmake_policy( VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} )

# project name
project( vvdec VERSION 3.0.0 )
# set alternative version numbering for release candidates
#set( PROJECT_VERSION_RC rc3 )
if( PROJECT_VERSION_RC )
    set( PROJECT_VERSION "${PROJECT_VERSION}-${PROJECT_VERSION_RC}" )
endif()

list( APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules" )
message( STATUS "CMAKE_MODULE_PATH: updating module path to: ${CMAKE_MODULE_PATH}" )

# # dump all cmake variables
# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

include( vvdecCompilerSupport )

detect_target_architecture( VVDEC_TARGET_ARCH )
if( VVDEC_TARGET_ARCH STREQUAL "ARM" )
  set( VVDEC_ARM_SIMD_DEFAULT TRUE )
endif()

# we enable x86 intrinsics for all target architectures, because they are implemented through simd-everywhere on non-x86
set( VVDEC_ENABLE_X86_SIMD   TRUE                      CACHE BOOL "enable x86 intrinsics" )
set( VVDEC_ENABLE_ARM_SIMD   ${VVDEC_ARM_SIMD_DEFAULT} CACHE BOOL "enable ARM intrinsics" )

set( VVDEC_ENABLE_TRACING    FALSE                     CACHE BOOL "Compile in tracing functionality" )

# enable sse4.1 build for all source files for gcc and clang
if( VVDEC_ENABLE_X86_SIMD )
  if( UNIX OR MINGW )
    # when building for non-x86, but emulating simd using simd-everywhere (e.g. on ARM),
    # the x86-compiler flags are not understood by the compiler
    set_if_compiler_supports_flag( FLAG_msse41 -msse4.1 )
    add_compile_options( ${FLAG_msse41} )
  endif()

  if( NOT MSVC AND NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Emscripten" )
    check_missing_intrinsics()
  endif()

  message( STATUS "x86 SIMD intrinsics enabled (using SIMDE for non-x86 targets)" )
  add_compile_definitions( TARGET_SIMD_X86 )

  # This potentially brings performance improvements for simd-everywhere, but I haven't
  # seen any, so it's disabled for now (GH)
  #set_if_compiler_supports_flag( FLAG_openmp_simd -fopenmp-simd )
  #add_compile_options( ${FLAG_openmp_simd} )

  if( VVDEC_TARGET_ARCH STREQUAL "ARM" )
    set_if_compiler_supports_flag( FLAG_mfpu_neon -mfpu=neon  )
    set_if_compiler_supports_flag( FLAG_mfloat_abi_hard -mfloat-abi=hard  )
    add_compile_options( ${FLAG_mfpu_neon} ${FLAG_mfloat_abi_hard} )
  endif()
endif()

if( VVDEC_ENABLE_ARM_SIMD )
  message( STATUS "ARM SIMD intrinsics enabled" )
  add_compile_definitions( TARGET_SIMD_ARM )

  set_if_compiler_supports_flag( FLAG_mfpu_neon -mfpu=neon  )
  set_if_compiler_supports_flag( FLAG_mfloat_abi_hard -mfloat-abi=hard  )
  add_compile_options( ${FLAG_mfpu_neon} ${FLAG_mfloat_abi_hard} )
endif()

if( VVDEC_TARGET_ARCH STREQUAL "WASM" )
  add_compile_options( -pthread )
  add_link_options(
    --bind
    -sWASM_BIGINT
    -sINITIAL_MEMORY=1300MB # this should be enough for FullHD decoding
    -sINVOKE_RUN=0          # don't call main() automatically
    -sUSE_PTHREADS
    #-sPROXY_TO_PTHREAD
    #-sPTHREAD_POOL_SIZE=20
    -sMINIFY_HTML=0
    -sMODULARIZE
    -sEXPORT_NAME=CreateVVdeC
    -sEXPORTED_RUNTIME_METHODS=[ccall,cwrap,getValue,setValue,_malloc,callMain,FS]
    -sEXPORTED_FUNCTIONS=@${CMAKE_CURRENT_SOURCE_DIR}/source/Lib/vvdec/wasm_exported_functions.json
#    --preload-file=${CMAKE_CURRENT_SOURCE_DIR}/ext/bitstreams/AMVR_B_HHI_3/AMVR_B_HHI_3.bit@in.bit
  )

  if( VVDEC_ENABLE_X86_SIMD )
    add_compile_options( -msimd128 )            # needed for all SIMD code, but breaks running in nodejs (firefox & chrome do work)

    add_compile_definitions( TARGET_SIMD_WASM ) # this is currently only used by a single helper function vvdec_get_RGBA_frame()
                                                # optimized specifically in WASM-SIMD. This has no effect on normal codec operations.
  endif()
endif()

# set common warning flags
add_compile_options( "$<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall;-Wno-unused-function;-Wno-enum-compare-switch;-Wno-unknown-attributes;-pedantic;-Wno-error=pedantic;-Wno-c++2a-extensions>" )
add_compile_options( "$<$<CXX_COMPILER_ID:GNU>:-Wall;-Wno-unused-function;-Wno-sign-compare;-fdiagnostics-show-option;-Wno-ignored-attributes;-pedantic;-Wno-error=pedantic>" )
add_compile_options( "$<$<CXX_COMPILER_ID:MSVC>:/W4;/wd4100;/wd4127;/wd4244;/wd4245;/wd4389;/wd4456;/wd4457;/wd4458;/wd4459;/wd4505;/wd4701;/wd4702;/wd4703>" )


if( NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR )
  # set exception handling
  if( MSVC )
    add_compile_options( "/EHsc" )
  endif()

  # vvdec embedded by superproject, always include source/Lib/vvdec  as first component
  list( PREPEND ${PROJECT_NAME}_ADD_SUBDIRECTORIES "source/Lib/vvdec" )
  list( REMOVE_DUPLICATES ${PROJECT_NAME}_ADD_SUBDIRECTORIES )
  message( STATUS "${CMAKE_CURRENT_SOURCE_DIR}: ${PROJECT_NAME} embedded, subdirectories to be added: ${${PROJECT_NAME}_ADD_SUBDIRECTORIES}" )
  # add subdirectories the superproject asked for
  foreach( subdir IN LISTS ${PROJECT_NAME}_ADD_SUBDIRECTORIES )
    add_subdirectory( ${subdir} )
  endforeach()

  return()
endif()

# enable or disable bitstream download
set( VVDEC_ENABLE_BITSTREAM_DOWNLOAD        OFF CACHE BOOL   "Enable or disable bitstream download" )

# enable local bitstream download
set( VVDEC_ENABLE_LOCAL_BITSTREAM_DOWNLOAD  OFF CACHE BOOL   "Enable or disable local bitstream download" )

# also install the test executable vvdecapp instead of the libraries and headers only
set( VVDEC_INSTALL_VVDECAPP                 OFF CACHE BOOL   "Enable or disable installing the test vvdecapp" )

set( VVDEC_LIBRARY_ONLY                     OFF CACHE BOOL   "Build libvvdec only (no vvdecapp)" )

set( BUILD_SHARED_LIBS                      OFF CACHE BOOL   "Build libvvdec as a shared library" )

# enable postfix
set( VVDEC_ENABLE_BUILD_TYPE_POSTFIX        OFF CACHE BOOL   "Enable or disable build type postfix for apps and libs" )

set( VVDEC_ENABLE_LINK_TIME_OPT             ON  CACHE BOOL   "Enable link time optimization for release and profile builds" )

set( VVDEC_ENABLE_WERROR                    ON  CACHE BOOL   "Treat warnings as errors (-Werror or /WX)" )

if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
    CMAKE_CXX_COMPILER_ID STREQUAL "Clang" )

  # enable address sanitizer
  set( VVDEC_USE_ADDRESS_SANITIZER          OFF CACHE BOOL   "Enable or disable address sanitizer" )
  set( VVDEC_USE_THREAD_SANITIZER           OFF CACHE BOOL   "Enable or disable thread sanitizer" )

  # add -march=native
  set( VVDEC_OPT_TARGET_ARCH                ""  CACHE STRING "Enable or disable building with architecture specific optimization" )
endif()

if( VVDEC_ENABLE_BUILD_TYPE_POSTFIX )
  if( BUILD_SHARED_LIBS )
    # set postfixes for shared libraries
    set( CMAKE_RELEASE_POSTFIX        "-s"   CACHE STRING "Set release library postfix" )
    set( CMAKE_DEBUG_POSTFIX          "-ds"  CACHE STRING "Set debug library postfix" )
    set( CMAKE_RELWITHDEBINFO_POSTFIX "-rds" CACHE STRING "Set relwithdebinfo library postfix" )
    set( CMAKE_MINSIZEREL_POSTFIX     "-mrs" CACHE STRING "Set minsizerel library postfix" )
  else()
    # set postfixes for static libraries
    set( CMAKE_RELEASE_POSTFIX        ""     CACHE STRING "Set release library postfix" )
    set( CMAKE_DEBUG_POSTFIX          "-d"   CACHE STRING "Set debug library postfix" )
    set( CMAKE_RELWITHDEBINFO_POSTFIX "-rd"  CACHE STRING "Set relwithdebinfo library postfix" )
    set( CMAKE_MINSIZEREL_POSTFIX     "-mr"  CACHE STRING "Set minsizerel library postfix" )
  endif()
endif()


# set VVDEC_OUTPUT_DIR_POSTFIX
if( BUILD_SHARED_LIBS )
  set( VVDEC_OUTPUT_DIR_POSTFIX shared )
else()
  set( VVDEC_OUTPUT_DIR_POSTFIX static )
endif()

set( VVDEC_TOPLEVEL_OUTPUT_DIRS              ON  CACHE BOOL   "Put build artifacts into ${CMAKE_SOURCE_DIR}/{bin,lib}/" )

if( VVDEC_TOPLEVEL_OUTPUT_DIRS )
  # Using CMake's default library name convention which is the same for all configurations.
  set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_SOURCE_DIR}/lib/debug-${VVDEC_OUTPUT_DIR_POSTFIX}" )
  set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_SOURCE_DIR}/lib/release-${VVDEC_OUTPUT_DIR_POSTFIX}" )
  set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/lib/relwithdebinfo-${VVDEC_OUTPUT_DIR_POSTFIX}" )
  set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL     "${CMAKE_SOURCE_DIR}/lib/minsizerel-${VVDEC_OUTPUT_DIR_POSTFIX}" )

  set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG}" )
  set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE}" )
  set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO}" )
  set( CMAKE_LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL     "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL}" )

  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG          "${CMAKE_SOURCE_DIR}/bin/debug-${VVDEC_OUTPUT_DIR_POSTFIX}" )
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE        "${CMAKE_SOURCE_DIR}/bin/release-${VVDEC_OUTPUT_DIR_POSTFIX}" )
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${CMAKE_SOURCE_DIR}/bin/relwithdebinfo-${VVDEC_OUTPUT_DIR_POSTFIX}" )
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL     "${CMAKE_SOURCE_DIR}/bin/minsizerel-${VVDEC_OUTPUT_DIR_POSTFIX}" )
else()
  # Put build artifacts below build-directory. (Don't need to add {debug,release,relwithdebinfo,minsizerel}-{static,shared}/)
  set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" )
  set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib" )
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin" )
endif()

# enable or disable Intel Vtune ITT Tracing
#if( CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
#    set( VVDEC_ENABLE_ITT ON CACHE BOOL "Enable Intel Runtime Support for Profiling" )
#else()
    set( VVDEC_ENABLE_ITT OFF CACHE BOOL "Enable Intel Runtime Support for Profiling" )
#endif()

# set default CMAKE_BUILD_TYPE to Release if not set
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE )
endif()

# set c++14
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_EXTENSIONS OFF )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# compile everything position independent (even static libraries)
set( CMAKE_POSITION_INDEPENDENT_CODE TRUE )

# set verbose compile options
#set( CMAKE_VERBOSE_MAKEFILE ON )

# use folders in IDEs for projects (e.g. lib sample app test)
set_property( GLOBAL PROPERTY USE_FOLDERS ON )

# Enable multithreading
find_package( Threads REQUIRED )

# set _WIN32_WINNT
if( WIN32 )
  # set _WIN32_WINT version global
  add_definitions( -D_WIN32_WINNT=0x0600 )
endif()

# enable parallel build for Visual Studio
if( MSVC )
  # add compile options
  add_compile_options( "/MP" )
  add_compile_options( "/EHsc" )
endif()

# set address sanitizer compiler arguments
if( VVDEC_USE_ADDRESS_SANITIZER )
  # add compile options
  add_compile_options( "-fsanitize=address" )
  add_compile_options( "-fsanitize=undefined" )
  add_compile_options( "-fsanitize=leak" )
  add_compile_options( "-fno-omit-frame-pointer" )
  add_link_options( "-fsanitize=address" )
  add_link_options( "-fsanitize=undefined" )
  add_link_options( "-fsanitize=leak" )
  add_link_options( "-fno-omit-frame-pointer" )
endif()

# set thread sanitizer compiler arguments
if( VVDEC_USE_THREAD_SANITIZER )
  # add compile options
  add_compile_options( "-fsanitize=thread" )
  add_link_options( "-fsanitize=thread" )
endif()

if( VVDEC_ENABLE_WERROR )
    add_compile_options( "$<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Werror>" )
    add_compile_options( "$<$<CXX_COMPILER_ID:MSVC>:/WX>" )
endif()

if( VVDEC_ENABLE_X86_SIMD )
  if( ( UNIX OR MINGW ) AND VVDEC_OPT_TARGET_ARCH )
    set( CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=${VVDEC_OPT_TARGET_ARCH} -mtune=${VVDEC_OPT_TARGET_ARCH}" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=${VVDEC_OPT_TARGET_ARCH} -mtune=${VVDEC_OPT_TARGET_ARCH}" )
  endif()
endif()

if( VVDEC_ENABLE_LINK_TIME_OPT )
  set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE        ON )
  set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON )
  set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL     ON )

  if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
    # enable parallel link-time optimization for GCC
    add_link_options( $<$<OR:$<CONFIG:Release>,$<CONFIG:RelWithDebInfo>,$<CONFIG:MinSizeRel>>:-flto=auto> )
  endif()
endif()

# use ccache
find_program( CCACHE_FOUND ccache )
if( CCACHE_FOUND )
  message( STATUS "ccache found. using it." )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_LINK ccache )
endif()

if( VVDEC_ENABLE_ITT )
  # search for Intel VTune
  if( MSVC )
    set( VVDEC_ITT_PATH "c:/Program Files (x86)/IntelSWTools/VTune Amplifier/" CACHE STRING "Path to the installation directory of Intel VTunes" )
  elseif( APPLE )
    message( WARNING "Not yet supported on Mac OS X" )
  elseif( UNIX OR MINGW )
    if( EXISTS "/opt/intel/vtune_profiler" )
      set( VVDEC_ITT_PATH "/opt/intel/vtune_profiler" CACHE STRING "Path to the installation directory of Intel VTunes" )
    else()
      set( VVDEC_ITT_PATH "/opt/intel/vtune_amplifier" CACHE STRING "Path to the installation directory of Intel VTunes" )
    endif()
  endif()

  if( EXISTS ${VVDEC_ITT_PATH}/lib64 AND EXISTS ${VVDEC_ITT_PATH}/include )
    set( INTEL_ITT_LINK_TARGET INTEL_ITT )
    add_library( ${INTEL_ITT_LINK_TARGET} STATIC IMPORTED GLOBAL )
    if( MSVC )
      set_target_properties( ${INTEL_ITT_LINK_TARGET} PROPERTIES IMPORTED_LOCATION        ${VVDEC_ITT_PATH}/lib64/libittnotify.lib )
    elseif( APPLE )
      # not supported
    elseif( UNIX OR MINGW )
      set_target_properties( ${INTEL_ITT_LINK_TARGET} PROPERTIES IMPORTED_LOCATION        ${VVDEC_ITT_PATH}/lib64/libittnotify.a )
      set_target_properties( ${INTEL_ITT_LINK_TARGET} PROPERTIES INTERFACE_LINK_LIBRARIES dl )
    endif()

    # set include directory. relative paths do not work.
    set_target_properties( ${INTEL_ITT_LINK_TARGET} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${VVDEC_ITT_PATH}/include )
    set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTRACE_ENABLE_ITT" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTRACE_ENABLE_ITT" )
  else()
    message( WARNING "VVDEC_ITT_PATH ${VVDEC_ITT_PATH} not found, ignoring option VVDEC_ENABLE_ITT" )
  endif()
endif()

if( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
  set( CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Standard install prefix" FORCE )
endif()

# use GNU install dirs
include( GNUInstallDirs )

# handle rpath correctly
if( BUILD_SHARED_LIBS AND NOT WIN32 )
  set( CMAKE_SKIP_INSTALL_RPATH OFF CACHE BOOL "skip rpath" )
  if( APPLE )
    set( RPATH_BASE @loader_path )
  elseif( UNIX )
    set( RPATH_BASE $ORIGIN )
  endif()

  file( RELATIVE_PATH RPATH_REL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR} ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR} )

  set( VVDEC_INSTALL_RPATH ${RPATH_BASE} ${RPATH_BASE}/${RPATH_REL_DIR} )
  message( STATUS "VVDEC_INSTALL_RPATH=${VVDEC_INSTALL_RPATH}" )
endif()

add_subdirectory( "source/Lib/vvdec" )
add_subdirectory( "source/App/vvdecapp" )

# create a list of all test bitstreams in variable BITSTREAM_FILES
# also defines BITSTREAM_URL_BASE
include( define_bitstream_files )

# set directory global
set( BITSTREAM_INSTALL_DIR_BASE "${CMAKE_SOURCE_DIR}/ext/bitstreams" )
if( VVDEC_ENABLE_BITSTREAM_DOWNLOAD )
  # enable FetchContent
  include( FetchContent )

  # download and uncompress
  set( COUNTER 1 )
  foreach( BITSTREAM_ZIP IN LISTS BITSTREAM_FILES BITSTREAM_FAULTY_FILES )
    string( REGEX REPLACE ".*/(.*)\\..*" "\\1" SUB_DIR "${BITSTREAM_URL_BASE}/${BITSTREAM_ZIP}" )
    set( BITSTREAM_INSTALL_DIR "${BITSTREAM_INSTALL_DIR_BASE}/${SUB_DIR}" )
    string( REGEX REPLACE "(.*).zip" "\\1" BITSTREAM_SHORT "${BITSTREAM_ZIP}" )
    if( NOT EXISTS "${BITSTREAM_INSTALL_DIR}/${BITSTREAM_SHORT}.bit" )
      message( STATUS "Downloading ${BITSTREAM_URL_BASE}/${BITSTREAM_ZIP}" )
      FetchContent_Populate( download_${COUNTER} QUIET SOURCE_DIR ${BITSTREAM_INSTALL_DIR} URL "${BITSTREAM_URL_BASE}/${BITSTREAM_ZIP}" )
    endif()
    math( EXPR COUNTER "${COUNTER}+1" )
  endforeach()
endif()

# enable testing with ctest
enable_testing()

if( VVDEC_LIBRARY_ONLY )
  message( WARNING "The test suite will not be able to run, when building the vvdec library only (VVDEC_LIBRARY_ONLY=ON)" )
endif()

function( read_bitstream_yuv_md5 BITSTREAM_FILE OUT_MD5_HASH )
  STRING( REGEX REPLACE "\\.bit$" ".yuv.md5" MD5_FILE ${BITSTREAM_FILE} )
  file( READ ${MD5_FILE} MD5_HASH LIMIT 32 )
  string( STRIP "${MD5_HASH}" MD5_HASH )
  string( TOLOWER "${MD5_HASH}" MD5_HASH )
  set( ${OUT_MD5_HASH} ${MD5_HASH} PARENT_SCOPE )
endfunction()

if( UNIX AND NOT VVDEC_USE_ADDRESS_SANITIZER AND
    ( CMAKE_BUILD_TYPE STREQUAL "Debug" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" ) )
    # run Debug and RelWithDebInfo test suite in GDB and collect backtrace on crashes
    set( VVDEC_TESTS_DEBUGGER_COMMAND gdb --batch --return-child-result --nx
                                          -iex "set print thread-events off"
                                          -iex "set index-cache on"
                                          -ex "run"
                                          -ex "info locals"
                                          -ex "backtrace"
                                          -ex "quit"
                                          --args
                                         CACHE STRING "provide a command wrapper (e.g. debugger) to run the tests." )
else()
    set( VVDEC_TESTS_DEBUGGER_COMMAND "" CACHE STRING "provide a command wrapper (e.g. debugger) to run the tests." )
endif()

# add tests for bitstreams
foreach( BITSTREAM IN LISTS BITSTREAM_FILES )
  string( REGEX REPLACE "(.*).zip" "\\1" BITSTREAM_SHORT "${BITSTREAM}" )
  set( BITSTREAM_FILE "${BITSTREAM_INSTALL_DIR_BASE}/${BITSTREAM_SHORT}/${BITSTREAM_SHORT}.bit" )

  if( EXISTS ${BITSTREAM_FILE} )
    read_bitstream_yuv_md5( ${BITSTREAM_FILE} RX_MD5_HASH )
    add_test( NAME Test:${BITSTREAM_SHORT}.bit COMMAND ${VVDEC_TESTS_DEBUGGER_COMMAND} $<TARGET_FILE:vvdecapp> -b ${BITSTREAM_FILE} -md5 ${RX_MD5_HASH} )
  else()
    add_test( NAME MISSING:${BITSTREAM_SHORT}.bit COMMAND ${VVDEC_TESTS_DEBUGGER_COMMAND} $<TARGET_FILE:vvdecapp> -b ${BITSTREAM_FILE} -dph )

    set( MISSING_BITSTREAM_FILES TRUE )
  endif()
endforeach()

foreach( BITSTREAM IN LISTS BITSTREAM_FAULTY_FILES )
  string( REGEX REPLACE "(.*).zip" "\\1" BITSTREAM_SHORT "${BITSTREAM}" )
  set( BITSTREAM_FILE "${BITSTREAM_INSTALL_DIR_BASE}/${BITSTREAM_SHORT}/${BITSTREAM_SHORT}.bit" )

  if( EXISTS ${BITSTREAM_FILE} )
    read_bitstream_yuv_md5( ${BITSTREAM_FILE} RX_MD5_HASH )
    add_test( NAME Faulty:${BITSTREAM_SHORT}.bit COMMAND ${VVDEC_TESTS_DEBUGGER_COMMAND} $<TARGET_FILE:vvdecapp> -b ${BITSTREAM_FILE} -dph -md5 ${RX_MD5_HASH} )
  else()
    add_test( NAME MISSING:${BITSTREAM_SHORT}.bit COMMAND ${VVDEC_TESTS_DEBUGGER_COMMAND} $<TARGET_FILE:vvdecapp> -b ${BITSTREAM_FILE} -dph )

    set( MISSING_BITSTREAM_FILES TRUE )
  endif()
endforeach()

get_directory_property( ALL_TESTS TESTS )
set_tests_properties( ${ALL_TESTS} PROPERTIES
                      TIMEOUT 120
                      FAIL_REGULAR_EXPRESSION "(WARNING:|runtime error)" )

add_custom_target( test-ok  USES_TERMINAL COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIG> -R "\"^(Test|MISSING)\"" )
add_custom_target( test-all USES_TERMINAL COMMAND ${CMAKE_CTEST_COMMAND} -C $<CONFIG> -R "\"^(Test|MISSING|Faulty)\"" )

if( MISSING_BITSTREAM_FILES )
  message( STATUS "Some bitstream files are missing." )
  message( STATUS "  If you want to run tests, reconfigure with -DVVDEC_ENABLE_BITSTREAM_DOWNLOAD=ON" )
  message( STATUS "  (or using top level Makefile: make test enable-bitstream-download=1)" )
endif()

# include installer
include( cmake/modules/vvdecInstall.cmake )
