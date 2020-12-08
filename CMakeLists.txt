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
project( vvdec VERSION 0.2.0.0 )

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
set( VVDEC_ENABLE_BITSTREAM_DOWNLOAD OFF CACHE BOOL "Enable or disable bitstream download" )

# enable local bitstream download
set( VVDEC_ENABLE_LOCAL_BITSTREAM_DOWNLOAD OFF CACHE BOOL "Enable or disable local bitstream download" )

# enable install target
set( VVDEC_ENABLE_INSTALL ON CACHE BOOL "Enable or disable install target" )

# enable postfix
set( VVDEC_ENABLE_BUILD_TYPE_POSTFIX OFF CACHE BOOL "Enable or disable build type postfix for apps and libs" )

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

#if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
#    CMAKE_CXX_COMPILER_ID STREQUAL "Clang" )
#  set( USE_ADDRESS_SANITIZER OFF CACHE BOOL "Compiles and links with -fsanitize=address" )
#endif()

#set( ENABLE_LINK_TIME_OPT     ON       CACHE BOOL   "Enable link time optimization for release and profile builds" )
#set( OPT_TARGET_ARCH         "native"  CACHE STRING "Create code and optimize for this architecture (default native)" )
#set( DISABLE_BOOST_DEPENDENCY OFF      CACHE BOOL   "Disable Boost dependency => disables the test suite" )

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

## set address sanitizer compiler arguments
#if( USE_ADDRESS_SANITIZER )
#  # add compile options
#  add_compile_options( "-fsanitize=address" )
#  add_link_options( "-fsanitize=address" )
#endif()

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

#if( ENABLE_LINK_TIME_OPT )
#    set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON )
#    set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON )
#    set( CMAKE_INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL ON )
#endif()

#if( (UNIX OR MINGW) AND NOT OPT_TARGET_ARCH STREQUAL "" )
#    set( CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -march=${OPT_TARGET_ARCH} -mtune=${OPT_TARGET_ARCH}" )
#    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=${OPT_TARGET_ARCH} -mtune=${OPT_TARGET_ARCH}" )
#endif()

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

# set directory global
set( BITSTREAM_INSTALL_DIR_BASE "${CMAKE_SOURCE_DIR}/ext/bitstreams" )

if( VVDEC_ENABLE_BITSTREAM_DOWNLOAD )
  # enable FetchContent
  include( FetchContent )
  
  if( VVDEC_ENABLE_LOCAL_BITSTREAM_DOWNLOAD )
    # get bitstreams
    set( BITSTREAM_URL_BASE "https://visvn.fe.hhi.de/download/VVC/under_test/VTM-11.0" )
  else()
    # get bitstreams
    set( BITSTREAM_URL_BASE "https://www.itu.int/wftp3/av-arch/jvet-site/bitstream_exchange/VVC/under_test/VTM-11.0" )
  endif()
  
  # create a list of all test bitstreams
  list( APPEND BITSTREAM_FILES
    "10b400_A_Bytedance_2.zip" "10b400_B_Bytedance_2.zip"
    "8b400_A_Bytedance_2.zip" "8b400_B_Bytedance_2.zip"
    "8b420_A_Bytedance_2.zip" "8b420_B_Bytedance_2.zip"
    "AFF_A_HUAWEI_2.zip" "AFF_B_HUAWEI_2.zip"
    "ALF_A_Huawei_3.zip" "ALF_B_Huawei_3.zip"
    "ALF_C_KDDI_2.zip"
    "ALF_D_Qualcomm_2.zip"
    "AMVR_A_HHI_3.zip" "AMVR_B_HHI_3.zip"
    "APSALF_A_Qualcomm_2.zip"
    "APSLMCS_A_Dolby_3.zip" "APSLMCS_B_Dolby_3.zip" "APSLMCS_C_Dolby_2.zip"
    "APSMULT_A_MediaTek_3.zip" "APSMULT_B_MediaTek_3.zip"
    "AUD_A_Broadcom_3.zip"
    "BCW_A_MediaTek_3.zip"
    "BDOF_A_MediaTek_3.zip"
    "BDPCM_A_Orange_2.zip"
    "BOUNDARY_A_Huawei_3.zip"
    #"BUMP_A_LGE_1.zip" "BUMP_B_LGE_1.zip" "BUMP_C_LGE_1.zip" # missing in 11.0
    "CCALF_A_Sharp_3.zip" "CCALF_B_Sharp_3.zip" "CCALF_C_Sharp_3.zip" "CCALF_D_Sharp_3.zip"
    "CCLM_A_KDDI_1.zip"
    "CIIP_A_MediaTek_3.zip"
    "CodingToolsSets_A_Tencent_2.zip" "CodingToolsSets_B_Tencent_2.zip" "CodingToolsSets_C_Tencent_2.zip" "CodingToolsSets_D_Tencent_2.zip"
    #"CodingToolsSets_E_Tencent_1.zip" # fails because of sub-pictures
    "CROP_A_Panasonic_3.zip" "CROP_B_Panasonic_4.zip"
    "CST_A_MediaTek_3.zip"
    "CTU_A_MediaTek_3.zip" "CTU_B_MediaTek_3.zip" "CTU_C_MediaTek_3.zip"
    "DEBLOCKING_A_Sharp_3.zip" "DEBLOCKING_B_Sharp_2.zip"
    "DEBLOCKING_C_Huawei_3.zip"
    "DEBLOCKING_E_Ericsson_2.zip" "DEBLOCKING_F_Ericsson_1.zip"
    "DMVR_A_Huawei_3.zip"
    "DMVR_B_KDDI_3.zip"
    "DPB_A_Sharplabs_2.zip" "DPB_B_Sharplabs_2.zip"
    "DQ_A_HHI_3.zip" "DQ_B_HHI_3.zip"
    "ENTMAINTIER_A_Sony_3.zip" "ENTMAINTIER_B_Sony_3.zip" "ENTMAINTIER_C_Sony_3.zip" "ENTMAINTIER_D_Sony_3.zip"
    "ENTROPY_A_Chipsnmedia_2.zip"
    "ENTROPY_A_Qualcomm_2.zip"
    "ENTROPY_B_Sharp_2.zip"
    "GPM_A_Alibaba_3.zip"
    "HLG_A_NHK_2.zip" "HLG_B_NHK_2.zip"
    "IBC_A_Tencent_2.zip" "IBC_B_Tencent_2.zip" "IBC_C_Tencent_2.zip" "IBC_D_Tencent_2.zip"
    "IP_B_Nokia_1.zip"
    "ISP_A_HHI_3.zip" "ISP_B_HHI_3.zip"
    "JCCR_C_HHI_3.zip" "JCCR_D_HHI_3.zip"
    "JCCR_A_Nokia_2.zip" "JCCR_B_Nokia_2.zip" "JCCR_E_Nokia_1.zip" "JCCR_F_Nokia_1.zip"
    "LFNST_A_LGE_3.zip" "LFNST_B_LGE_3.zip"
    "LFNST_C_HHI_3.zip" "LFNST_D_HHI_3.zip"
    "LMCS_A_Dolby_3.zip"
    #"LMCS_B_Dolby_2.zip" # fails because of sub-pictures
    "LOSSLESS_A_HHI_3.zip" "LOSSLESS_B_HHI_3.zip"
    "LTRP_A_ERICSSON_2.zip"
    "MERGE_A_Qualcomm_2.zip" "MERGE_B_Qualcomm_2.zip" "MERGE_C_Qualcomm_2.zip" "MERGE_D_Qualcomm_2.zip" "MERGE_E_Qualcomm_2.zip"
    "MERGE_F_Qualcomm_2.zip" "MERGE_G_Qualcomm_2.zip" "MERGE_H_Qualcomm_2.zip" "MERGE_I_Qualcomm_2.zip" "MERGE_J_Qualcomm_2.zip"
    "MIP_A_HHI_3.zip" "MIP_B_HHI_3.zip"
    "MMVD_A_SAMSUNG_3.zip"
    "MPM_A_LGE_3.zip"
    "MRLP_A_HHI_2.zip" "MRLP_B_HHI_2.zip"
    "MTS_A_LGE_3.zip" "MTS_B_LGE_3.zip"
    "MTS_LFNST_A_LGE_3.zip" "MTS_LFNST_B_LGE_3.zip"
    "MVCOMP_A_Sharp_2.zip"
    "PDPC_A_Qualcomm_3.zip" "PDPC_B_Qualcomm_3.zip" "PDPC_C_Qualcomm_2.zip" 
    "PHSH_B_Sharp_1.zip"
    "POC_A_Nokia_1.zip"
    "POUT_A_Sharplabs_2.zip"
    "PPS_A_Bytedance_1.zip" "PPS_B_Bytedance_1.zip" "PPS_C_Bytedance_1.zip"
    "PQ_A_Dolby_1.zip"
    "PROF_A_Interdigital_3.zip" "PROF_B_Interdigital_3.zip"
    "PSEXT_A_Nokia_2.zip" "PSEXT_B_Nokia_2.zip"
    "QTBTT_A_MediaTek_3.zip"
    "QUANT_A_Huawei_2.zip" "QUANT_B_Huawei_2.zip" "QUANT_C_Huawei_2.zip" "QUANT_D_Huawei_2.zip"
    "RAP_A_HHI_1.zip" "RAP_B_HHI_1.zip" "RAP_C_HHI_1.zip" "RAP_D_HHI_1.zip"
    "RPL_A_ERICSSON_2.zip"
    "RPR_A_Alibaba_4.zip" "RPR_B_Alibaba_3.zip" "RPR_C_Alibaba_3.zip"
    "SAO_A_SAMSUNG_3.zip" "SAO_B_SAMSUNG_3.zip" "SAO_C_SAMSUNG_3.zip"
    "SBT_A_HUAWEI_2.zip"
    "SbTMVP_A_Bytedance_3.zip" "SbTMVP_B_Bytedance_3.zip"
    "SCALING_A_InterDigital_1.zip" "SCALING_B_InterDigital_1.zip" "SCALING_C_InterDigital_1.zip"
    "SDH_A_Dolby_2.zip"
    "SLICES_A_HUAWEI_2.zip"
    "SMVD_A_HUAWEI_2.zip"
    "SPS_A_Bytedance_1.zip" "SPS_B_Bytedance_1.zip" "SPS_C_Bytedance_1.zip"
    #"SUBPIC_A_HUAWEI_3.zip" "SUBPIC_B_HUAWEI_3.zip" # subpictures currently not supported
    "TEMPSCAL_A_Panasonic_4.zip"
    #"TEMPSCAL_B_Panasonic_4.zip" # seems broken, doesn't decode with VTM-11
    "TEMPSCAL_C_Panasonic_3.zip"
    "TILE_A_Nokia_2.zip" "TILE_B_Nokia_2.zip" "TILE_C_Nokia_2.zip" "TILE_D_Nokia_2.zip" "TILE_E_Nokia_2.zip" "TILE_F_Nokia_2.zip" "TILE_G_Nokia_2.zip"
    "TMVP_A_Chipsnmedia_3.zip" "TMVP_B_Chipsnmedia_3.zip" "TMVP_C_Chipsnmedia_3.zip" "TMVP_D_Chipsnmedia_3.zip"
    "TRANS_A_Chipsnmedia_2.zip" "TRANS_B_Chipsnmedia_2.zip" "TRANS_C_Chipsnmedia_2.zip" "TRANS_D_Chipsnmedia_2.zip"
    "TREE_A_HHI_3.zip" "TREE_B_HHI_3.zip" "TREE_C_HHI_3.zip"
    #"VIRTUAL_A_MediaTek_3.zip" "VIRTUAL_B_MediaTek_3.zip" # TODO
    "WP_A_InterDigital_3.zip" "WP_B_InterDigital_3.zip"
    "WPP_A_Sharp_3.zip" "WPP_B_Sharp_2.zip"
    "WRAP_A_InterDigital_4.zip" "WRAP_B_InterDigital_4.zip" "WRAP_C_InterDigital_4.zip" "WRAP_D_InterDigital_4.zip"
  )
  
  # download and uncompress
  set( COUNTER 1 )
  foreach( BITSTREAM_FILE IN LISTS BITSTREAM_FILES )
    string( REGEX REPLACE ".*/(.*)\\..*" "\\1" SUB_DIR "${BITSTREAM_URL_BASE}/${BITSTREAM_FILE}" )
    set( BITSTREAM_INSTALL_DIR "${BITSTREAM_INSTALL_DIR_BASE}/${SUB_DIR}" )
    if( NOT EXISTS "${BITSTREAM_INSTALL_DIR}" )
      FetchContent_Populate( ${COUNTER} SOURCE_DIR ${BITSTREAM_INSTALL_DIR} URL "${BITSTREAM_URL_BASE}/${BITSTREAM_FILE}" )  
    endif()
    math( EXPR COUNTER "${COUNTER}+1" )
  endforeach()
endif()

# enable testing with ctest
enable_testing()

# find all bitstreams and add test
file( GLOB_RECURSE BITSTREAMS LIST_DIRECTORIES false ${BITSTREAM_INSTALL_DIR_BASE}/*.bit )
foreach( BITSTREAM IN LISTS BITSTREAMS )
  string( REGEX REPLACE ".*/(.*)" "\\1" BITSTREAM_SHORT "${BITSTREAM}" )
  if( NOT BITSTREAM_SHORT MATCHES "^\\..+" )
    add_test( NAME Test_${BITSTREAM_SHORT} COMMAND vvdecapp -b ${BITSTREAM} -dph )
    set_tests_properties( Test_${BITSTREAM_SHORT} PROPERTIES TIMEOUT 60 )
  endif() 
endforeach()

if( VVDEC_ENABLE_INSTALL )
  # include installer
  include( cmake/modules/vvdecInstall.cmake )
endif()
