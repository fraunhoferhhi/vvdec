# executable
set( EXE_NAME vvdec_unit_test )

# get source files
file( GLOB SRC_FILES CONFIGURE_DEPENDS "*.cpp" )

# get include files
file( GLOB INC_FILES CONFIGURE_DEPENDS "*.h" )

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
if( VVDEC_LIBRARY_ONLY )
  set_target_properties( ${EXE_NAME} PROPERTIES EXCLUDE_FROM_ALL TRUE )
endif()

target_include_directories( ${EXE_NAME}
  PRIVATE
    "${VVDEC_LIB_DIR}"
    "${VVDEC_LIB_DIR}/CommonLib"
    "${VVDEC_LIB_DIR}/CommonLib/x86"
)
target_include_directories( ${EXE_NAME}
   SYSTEM PRIVATE
    "../../thirdparty/"
)

target_link_libraries( ${EXE_NAME} Threads::Threads vvdec )

target_compile_options( ${EXE_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall>
                                            $<$<CXX_COMPILER_ID:GNU>:-Wall -fdiagnostics-show-option -Wno-ignored-attributes -Wno-sign-compare>
                                            $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /wd4244 /wd4251 /wd4996 /wd4189>)

# example: place header files in different folders
source_group( "Header Files"   FILES ${INC_FILES} )
source_group( "Resource Files" FILES ${RESOURCE_FILE} )

# set the folder where to place the projects
set_target_properties( ${EXE_NAME} PROPERTIES FOLDER tests )

add_test( NAME vvdec_unit_test COMMAND ${VVDEC_TESTS_DEBUGGER_COMMAND} $<TARGET_FILE:${EXE_NAME}> )
set_tests_properties( vvdec_unit_test PROPERTIES TIMEOUT 120 )
