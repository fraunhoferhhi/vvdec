# VVDecConfig.cmake - package configuration file

# get current directory
get_filename_component( SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )

if( DEFINED VVDec_USE_SHARED_LIBS )
  if( VVDec_USE_SHARED_LIBS )
    include( ${SELF_DIR}/shared/VVDecTargets.cmake )
  else()
    include( ${SELF_DIR}/static/VVDecTargets.cmake )
  endif()
else()
  if( BUILD_SHARED_LIBS )
    include( ${SELF_DIR}/shared/VVDecTargets.cmake )
  else()
    include( ${SELF_DIR}/static/VVDecTargets.cmake )
  endif()  
endif()
