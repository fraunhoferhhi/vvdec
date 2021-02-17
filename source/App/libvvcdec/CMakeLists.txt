# executable
set( DLL_NAME vvcdec )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# add executable
add_library(${DLL_NAME} SHARED  ${SRC_FILES} ${INC_FILES})
set_target_properties( ${DLL_NAME} PROPERTIES RELEASE_POSTFIX        "${CMAKE_RELEASE_POSTFIX}" )
set_target_properties( ${DLL_NAME} PROPERTIES DEBUG_POSTFIX          "${CMAKE_DEBUG_POSTFIX}" )
set_target_properties( ${DLL_NAME} PROPERTIES RELWITHDEBINFO_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}" )
set_target_properties( ${DLL_NAME} PROPERTIES MINSIZEREL_POSTFIX     "${CMAKE_MINSIZEREL_POSTFIX}" )

target_compile_options( ${DLL_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall -Werror>
                                            $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror>
                                            $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /wd4244 /wd4456 >)
target_link_libraries( ${DLL_NAME} Threads::Threads vvdec )

# example: place header files in different folders
source_group( "Header Files"   FILES ${INC_FILES} )

# set the folder where to place the projects
set_target_properties( ${DLL_NAME}  PROPERTIES FOLDER app )
