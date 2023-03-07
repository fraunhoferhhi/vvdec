# executable
set( EXE_NAME vvdecapp )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# set resource file for MSVC compilers
if( MSVC )
  set( RESOURCE_FILE ${EXE_NAME}.rc )
endif()

if( VVDEC_ENABLE_TRACING )
  add_compile_definitions( ENABLE_TRACING )
endif()

# add executable
add_executable( ${EXE_NAME} ${SRC_FILES} ${INC_FILES} ${RESOURCE_FILE} )
set_target_properties( ${EXE_NAME} PROPERTIES RELEASE_POSTFIX        "${CMAKE_RELEASE_POSTFIX}" )
set_target_properties( ${EXE_NAME} PROPERTIES DEBUG_POSTFIX          "${CMAKE_DEBUG_POSTFIX}" )
set_target_properties( ${EXE_NAME} PROPERTIES RELWITHDEBINFO_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}" )
set_target_properties( ${EXE_NAME} PROPERTIES MINSIZEREL_POSTFIX     "${CMAKE_MINSIZEREL_POSTFIX}" )
if( VVDEC_INSTALL_RPATH )
  set_target_properties( ${EXE_NAME} PROPERTIES INSTALL_RPATH        "${VVDEC_INSTALL_RPATH}" )
endif()

target_compile_options( ${EXE_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall -Werror>
                                            $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror>
                                            $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /wd4244 /wd4456 >)
target_link_libraries( ${EXE_NAME} Threads::Threads vvdec )
target_include_directories( ${EXE_NAME} PRIVATE  ../../Lib/libmd5 )

if( ${CMAKE_SYSTEM_NAME} STREQUAL "Emscripten" )
  set_target_properties( ${EXE_NAME} PROPERTIES LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../../Lib/vvdec/wasm_exported_functions.json;" )
endif()

# example: place header files in different folders
source_group( "Header Files"   FILES ${INC_FILES} )
source_group( "Resource Files" FILES ${RESOURCE_FILE} )

# set the folder where to place the projects
set_target_properties( ${EXE_NAME} PROPERTIES
    FOLDER app
    XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER de.fraunhofer.hhi.vvdec )
