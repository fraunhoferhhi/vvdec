# vvdecInstall

set( RUNTIME_DEST ${CMAKE_INSTALL_BINDIR} )
set( LIBRARY_DEST ${CMAKE_INSTALL_LIBDIR} )
set( ARCHIVE_DEST ${CMAKE_INSTALL_LIBDIR} )

# install targets
macro( install_targets targets_ config_ )
  string( TOLOWER ${config_} config_lc_ )
  install( TARGETS             ${targets_}
           EXPORT              vvdecTargets-${config_lc_}
           CONFIGURATIONS      ${config_}
           RUNTIME DESTINATION ${RUNTIME_DEST}
           BUNDLE DESTINATION  ${RUNTIME_DEST}
           LIBRARY DESTINATION ${LIBRARY_DEST}
           ARCHIVE DESTINATION ${ARCHIVE_DEST} )

  if( XCODE AND BUILD_SHARED_LIBS )
    # WORKAROUND: reapply code signature, which gets broken by cmake install step when modifying the RPATH
    foreach( tgt_ IN ITEMS ${targets_} )
      set( is_app_ "$<STREQUAL:$<TARGET_PROPERTY:${tgt_},TYPE>,EXECUTABLE>" )
      install( CODE "execute_process( COMMAND codesign --force --sign - --timestamp=none --generate-entitlement-der
                                              \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/$<IF:${is_app_},${RUNTIME_DEST},${LIBRARY_DEST}>/$<TARGET_FILE_NAME:${tgt_}>\" )" )
    endforeach()
  endif()
endmacro( install_targets )

# install pdb file for static and shared libraries
macro( install_lib_pdb lib_ )
  if( MSVC )
    install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,SHARED_LIBRARY>>:$<TARGET_PDB_FILE:${lib_}>>
             CONFIGURATIONS Debug DESTINATION ${RUNTIME_DEST} OPTIONAL )
    install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,SHARED_LIBRARY>>:$<TARGET_PDB_FILE:${lib_}>>
             CONFIGURATIONS RelWithDebInfo DESTINATION ${RUNTIME_DEST} OPTIONAL )
    install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,STATIC_LIBRARY>>:$<TARGET_FILE_DIR:${lib_}>/$<TARGET_PROPERTY:${lib_},NAME>.pdb>
             CONFIGURATIONS Debug DESTINATION ${ARCHIVE_DEST} OPTIONAL )
    install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,STATIC_LIBRARY>>:$<TARGET_FILE_DIR:${lib_}>/$<TARGET_PROPERTY:${lib_},NAME>.pdb>
             CONFIGURATIONS RelWithDebInfo DESTINATION ${ARCHIVE_DEST} OPTIONAL )
    #install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,STATIC_LIBRARY>>:$<TARGET_FILE_DIR:${lib_}>/${lib_}.pdb>
    #         CONFIGURATIONS Debug DESTINATION ${ARCHIVE_DEST} OPTIONAL )
    #install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,STATIC_LIBRARY>>:$<TARGET_FILE_DIR:${lib_}>/${lib_}.pdb>
    #         CONFIGURATIONS RelWithDebInfo DESTINATION ${ARCHIVE_DEST} OPTIONAL )
  endif()
endmacro( install_lib_pdb )

# install pdb file for executables
macro( install_exe_pdb exe_ )
  if( MSVC )
    install( FILES $<$<PLATFORM_ID:Windows>:$<TARGET_PDB_FILE:${exe_}>>  DESTINATION ${RUNTIME_DEST} CONFIGURATIONS Debug          OPTIONAL )
    install( FILES $<$<PLATFORM_ID:Windows>:$<TARGET_PDB_FILE:${exe_}>>  DESTINATION ${RUNTIME_DEST} CONFIGURATIONS RelWithDebInfo OPTIONAL )
  endif()
endmacro( install_exe_pdb )

# set interface include directories
target_include_directories( vvdec  SYSTEM INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> )

# install headers
install( FILES     ${CMAKE_BINARY_DIR}/vvdec/version.h  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/vvdec )
install( DIRECTORY include/vvdec                        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} )

if( VVDEC_INSTALL_VVDECAPP AND VVDEC_LIBRARY_ONLY )
  message( FATAL_ERROR "VVDEC_INSTALL_VVDECAPP conflicts with VVDEC_LIBRARY_ONLY" )
endif()

set( INSTALL_TARGETS vvdec )

if( NOT VVDEC_LIBRARY_ONLY )
  if( VVDEC_INSTALL_VVDECAPP
      OR (${CMAKE_SYSTEM_NAME} STREQUAL "Emscripten") ) # for Emscripten/WASM builds vvdecapp is always installed, since that is used like a library
    list( APPEND INSTALL_TARGETS vvdecapp )
  else()
    install( CODE "message( NOTICE \"The vvdecapp binary is not installed by default any more. To also install vvdecapp set '-DVVDEC_INSTALL_VVDECAPP=ON' (with make: 'install-vvdecapp=1')\" )" )
  endif()
endif()


# install targets
install_targets( "${INSTALL_TARGETS}" Release )
install_targets( "${INSTALL_TARGETS}" Debug )
install_targets( "${INSTALL_TARGETS}" RelWithDebInfo )
install_targets( "${INSTALL_TARGETS}" MinSizeRel )

# install pdb files
install_lib_pdb( vvdec )
if( VVDEC_INSTALL_VVDECAPP )
  install_exe_pdb( vvdecapp )
endif()

# install emscripten generated files
if( ${CMAKE_SYSTEM_NAME} STREQUAL "Emscripten" )
  install( PROGRAMS $<TARGET_FILE_DIR:vvdecapp>/vvdecapp.wasm DESTINATION ${RUNTIME_DEST} )
  install( PROGRAMS $<TARGET_FILE_DIR:vvdecapp>/vvdecapp.worker.js DESTINATION ${RUNTIME_DEST} )
endif()

# configure version file
configure_file( cmake/install/vvdecConfigVersion.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/vvdecConfigVersion.cmake @ONLY )

# install cmake releated files
install( FILES cmake/install/vvdecConfig.cmake                       DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvdec )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/vvdecConfigVersion.cmake  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvdec )

# set config postfix
if( BUILD_SHARED_LIBS )
  set( CONFIG_POSTFIX shared )
else()
  set( CONFIG_POSTFIX static )
endif()

# create target cmake files
install( EXPORT vvdecTargets-release        NAMESPACE vvdec:: FILE vvdecTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS Release        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvdec )
install( EXPORT vvdecTargets-debug          NAMESPACE vvdec:: FILE vvdecTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS Debug          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvdec )
install( EXPORT vvdecTargets-relwithdebinfo NAMESPACE vvdec:: FILE vvdecTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS RelWithDebInfo DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvdec )
install( EXPORT vvdecTargets-minsizerel     NAMESPACE vvdec:: FILE vvdecTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS MinSizeRel     DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvdec )


function( resolve_target_interface_libs TGT OUT_VAR )
  get_target_property( interface_libs ${TGT} INTERFACE_LINK_LIBRARIES )

  foreach( lib ${interface_libs} )
    if( TARGET ${lib} )
      # if it is a target and not a -llibrary, we need to further resolve it
      resolve_target_interface_libs( ${lib} lib )
    endif()

    list( APPEND ret ${lib} )
  endforeach()

  set( ${OUT_VAR} ${ret} PARENT_SCOPE )
endfunction()

# create pkg-config file
set( VVDEC_PKG_EXTRA_LIBS ${CMAKE_CXX_IMPLICIT_LINK_LIBRARIES} )
if( VVDEC_PKG_EXTRA_LIBS )
  foreach( LIB ${VVDEC_PKG_EXTRA_LIBS} )
    if((IS_ABSOLUTE ${LIB} AND EXISTS ${LIB}) OR (${LIB} MATCHES "^-"))
      list( APPEND EXTRALIBS ${LIB} )
    else()
      list( APPEND EXTRALIBS "-l${LIB}" )
    endif()
  endforeach()

  if( EXTRALIBS )
    set(VVDEC_PKG_EXTRA_LIBS ${EXTRALIBS})
  endif()

  list( REMOVE_ITEM VVDEC_PKG_EXTRA_LIBS "-lc" )
endif()

resolve_target_interface_libs( vvdec VVDEC_PKG_INTERFACE_LIBS )
if( VVDEC_PKG_INTERFACE_LIBS )
  list( APPEND VVDEC_PKG_EXTRA_LIBS ${VVDEC_PKG_INTERFACE_LIBS} )
endif()

list( JOIN VVDEC_PKG_EXTRA_LIBS " " VVDEC_PKG_EXTRA_LIBS  )
configure_file( pkgconfig/libvvdec.pc.in ${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/libvvdec.pc @ONLY )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/libvvdec.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig )

