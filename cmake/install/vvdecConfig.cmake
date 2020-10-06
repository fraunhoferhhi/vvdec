# vvdecConfig.cmake - package configuration file

# get current directory
get_filename_component( SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )

# detect state
if( DEFINED vvdec_USE_SHARED_LIBS )
  if( vvdec_USE_SHARED_LIBS )
    set( USE_SHARED TRUE )
  else()
    set( USE_SHARED FALSE )
  endif()
else()
  if( BUILD_SHARED_LIBS )
    set( USE_SHARED TRUE )
  else()
    set( USE_SHARED FALSE )
  endif()  
endif()

if( USE_SHARED )
  if( EXISTS ${SELF_DIR}/vvdecTargets-shared.cmake )
    include( ${SELF_DIR}/vvdecTargets-shared.cmake )
  else()  
    include( ${SELF_DIR}/vvdecTargets-static.cmake )
  endif()
else()
  if( EXISTS ${SELF_DIR}/vvdecTargets-static.cmake )
    include( ${SELF_DIR}/vvdecTargets-static.cmake )
  else()  
    include( ${SELF_DIR}/vvdecTargets-shared.cmake )
  endif()
endif()
