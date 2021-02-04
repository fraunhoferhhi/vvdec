# Top level CMakeLists.txt
#
# minimum required cmake version
cmake_minimum_required( VERSION 3.12.0 FATAL_ERROR )

# set cmake policy
if( NOT CMAKE_VERSION VERSION_LESS 3.12.0 )
  # Use latest policy
  cmake_policy( VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} )
endif()

# project name
project( vvdec VERSION 0.2.1.0 )

if( NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR )
  # enable sse4.1 build for all source files for gcc and clang
  if( UNIX OR MINGW )
    add_compile_options( "-msse4.1" )
  endif()

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

# enable install target
set( VVDEC_ENABLE_INSTALL                   ON  CACHE BOOL   "Enable or disable install target" )

# enable postfix
set( VVDEC_ENABLE_BUILD_TYPE_POSTFIX        OFF CACHE BOOL   "Enable or disable build type postfix for apps and libs" )

set( VVDEC_ENABLE_LINK_TIME_OPT             ON  CACHE BOOL   "Enable link time optimization for release and profile builds" )

if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
    CMAKE_CXX_COMPILER_ID STREQUAL "Clang" )

  # enable address sanitizer
  set( VVDEC_USE_ADDRESS_SANITIZER          OFF CACHE BOOL   "Enable or disable address sanitizer" )

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

# enable or disable Intel Vtune ITT Tracing
#if( CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
#    set( VVDEC_ENABLE_ITT ON CACHE BOOL "Enable Intel Runtime Support for Profiling" )
#else()
#    set( VVDEC_ENABLE_ITT OFF CACHE BOOL "Enable Intel Runtime Support for Profiling" )
#endif()

# set default CMAKE_BUILD_TYPE to Release if not set
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE )
endif()

#set( SET_ENABLE_TRACING OFF CACHE BOOL "Set ENABLE_TRACING as a compiler flag" )
#set( ENABLE_TRACING OFF CACHE BOOL "If SET_ENABLE_TRACING is on, it will be set to this value" )

# set c++14
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# compile everything position independent (even static libraries)
set( CMAKE_POSITION_INDEPENDENT_CODE TRUE )

# set verbose compile options
#set( CMAKE_VERBOSE_MAKEFILE ON )

# use folders in IDEs for projects (e.g. lib sample app test)
set_property( GLOBAL PROPERTY USE_FOLDERS ON )

set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules" )
message( STATUS "CMAKE_MODULE_PATH: updating module path to: ${CMAKE_MODULE_PATH}" )

# Enable multithreading
find_package( Threads REQUIRED )

# enable sse4.1 build for all source files for gcc and clang
if( UNIX OR MINGW )
  add_compile_options( "-msse4.1" )
endif()

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
  add_link_options( "-fsanitize=address" )
endif()

if( ( UNIX OR MINGW ) AND NOT VVDEC_OPT_TARGET_ARCH STREQUAL "" )
  set( CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=${VVDEC_OPT_TARGET_ARCH} -mtune=${VVDEC_OPT_TARGET_ARCH}" )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=${VVDEC_OPT_TARGET_ARCH} -mtune=${VVDEC_OPT_TARGET_ARCH}" )
endif()

if( VVDEC_ENABLE_LINK_TIME_OPT )
    set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON )
    set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON )
    set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL ON )
endif()

# use ccache
find_program( CCACHE_FOUND ccache )
if( CCACHE_FOUND )
  message( STATUS "ccache found. using it." )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_LINK ccache )
endif()

if( VVDEC_ENABLE_ITT )
  if( MSVC )
    set( ITT_PATH "c:/Program Files (x86)/IntelSWTools/VTune Amplifier/" CACHE STRING "Path to the installation directory of Intel VTunes" )
  elseif( APPLE )
    message( WARNING "Not yet supported on Mac OS X" )
  elseif( UNIX OR MINGW )
    if( EXISTS "/opt/intel/vtune_profiler" )
      set( ITT_PATH "/opt/intel/vtune_profiler" CACHE STRING "Path to the installation directory of Intel VTunes" )
    else()
      set( ITT_PATH "/opt/intel/vtune_amplifier" CACHE STRING "Path to the installation directory of Intel VTunes" )
    endif()
  endif()

  if( EXISTS ${ITT_PATH} )
    set( LIB_NAME INTEL_ITT )
    add_library( ${LIB_NAME} STATIC IMPORTED GLOBAL )
    if( MSVC )
      set_target_properties( ${LIB_NAME} PROPERTIES IMPORTED_LOCATION        ${ITT_PATH}/lib64/libittnotify.lib )
    elseif( APPLE )
      # not supported
    elseif( UNIX OR MINGW )
      set_target_properties( ${LIB_NAME} PROPERTIES IMPORTED_LOCATION        ${ITT_PATH}/lib64/libittnotify.a )
      set_target_properties( ${LIB_NAME} PROPERTIES INTERFACE_LINK_LIBRARIES dl )
    endif()

    # set include directory. relative paths do not work.
    set_target_properties( ${LIB_NAME} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${ITT_PATH}/include )
    set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DTRACE_ENABLE_ITT" )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTRACE_ENABLE_ITT" )
  else()
    message( WARNING "ITT_PATH ${ITT_PATH} not found, ignoring option VVCLib_ENABLE_ITT" )
  endif()
endif()

#if( SET_ENABLE_TRACING )
#  if( ENABLE_TRACING )
#    add_compile_definitions( ENABLE_TRACING=1 )
#  else()
#    add_compile_definitions( ENABLE_TRACING=0 )
#  endif()
#endif()

# handle rpath correctly
if( VVDEC_ENABLE_INSTALL )
  if( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
    set( CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Standard install prefix" FORCE )
  endif()

  # use GNU install dirs
  include( GNUInstallDirs )

  if( BUILD_SHARED_LIBS AND NOT WIN32 )
    set( CMAKE_SKIP_INSTALL_RPATH OFF CACHE BOOL "skip rpath" )
    if( APPLE )
      set( RPATH_BASE @loader_path )
    elseif( UNIX )
      set( RPATH_BASE $ORIGIN )
    endif()

    file( RELATIVE_PATH RPATH_REL_DIR ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR} ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR} )

    set( CMAKE_INSTALL_RPATH ${RPATH_BASE} ${RPATH_BASE}/${RPATH_REL_DIR} )
    message( STATUS "CMAKE_INSTALL_RPATH=${CMAKE_INSTALL_RPATH}" )
  endif()
endif()

add_subdirectory( "source/Lib/vvdec" )
add_subdirectory( "source/App/vvdecapp" )
add_subdirectory( "source/App/libvvcdec" )

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
  foreach( BITSTREAM_ZIP IN LISTS BITSTREAM_FILES )
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

# add tests for bitstreams
foreach( BITSTREAM IN LISTS BITSTREAM_FILES )
  string( REGEX REPLACE "(.*).zip" "\\1" BITSTREAM_SHORT "${BITSTREAM}" )
  set( BITSTREAM_FILE "${BITSTREAM_INSTALL_DIR_BASE}/${BITSTREAM_SHORT}/${BITSTREAM_SHORT}.bit" )

  if( EXISTS ${BITSTREAM_FILE} )
    add_test( NAME Test:${BITSTREAM_SHORT}.bit COMMAND vvdecapp -b ${BITSTREAM_FILE} -dph )
    set_tests_properties( Test:${BITSTREAM_SHORT}.bit PROPERTIES TIMEOUT 120 )
  else()
    add_test( NAME MISSING:${BITSTREAM_SHORT}.bit COMMAND vvdecapp -b ${BITSTREAM_FILE} -dph )
    set_tests_properties( MISSING:${BITSTREAM_SHORT}.bit PROPERTIES TIMEOUT 120 )

    set( MISSING_BITSTREAM_FILES TRUE )
  endif()
endforeach()

if( MISSING_BITSTREAM_FILES )
  message( STATUS "Some bitstream files are missing." )
  message( STATUS "  If you want to run tests, reconfigure with -DVVDEC_ENABLE_BITSTREAM_DOWNLOAD=ON" )
  message( STATUS "  (or using top level Makefile: make test enable-bitstream-download=1)" )
endif()

if( VVDEC_ENABLE_INSTALL )
  # include installer
  include( cmake/modules/vvdecInstall.cmake )
endif()
