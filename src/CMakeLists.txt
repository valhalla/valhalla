function(valhalla_module)
  set(oneValueArgs NAME)
  set(multiValueArgs SOURCES HEADERS DEPENDS INCLUDE_DIRECTORIES)
  cmake_parse_arguments(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(library valhalla-${MODULE_NAME})
  add_library(${library} STATIC ${MODULE_SOURCES} ${MODULE_HEADERS})
  add_library(valhalla::${MODULE_NAME} ALIAS ${library})
  target_compile_definitions(${library} PUBLIC RAPIDJSON_HAS_STDSTRING)
  target_include_directories(${library} ${MODULE_INCLUDE_DIRECTORIES})
  target_link_libraries(${library} ${MODULE_DEPENDS})

  install(FILES ${MODULE_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${MODULE_NAME}
    COMPONENT development)

  if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
    set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif()
endfunction()


add_subdirectory(${CMAKE_SOURCE_DIR}/proto ${CMAKE_BINARY_DIR}/valhalla/proto)
add_subdirectory(baldr)
add_subdirectory(loki)
add_subdirectory(meili)
add_subdirectory(midgard)
add_subdirectory(mjolnir)
add_subdirectory(odin)
add_subdirectory(sif)
add_subdirectory(skadi)
add_subdirectory(thor)
add_subdirectory(tyr)

message(STATUS "Generating  ${CMAKE_BINARY_DIR}/valhalla/valhalla.h")
configure_file(${CMAKE_SOURCE_DIR}/valhalla/valhalla.h.in ${CMAKE_BINARY_DIR}/valhalla/valhalla.h @ONLY)

message(STATUS "Generating  ${CMAKE_BINARY_DIR}/valhalla/config.h")
configure_file(${CMAKE_SOURCE_DIR}/valhalla/config.h.cmake ${CMAKE_BINARY_DIR}/valhalla/config.h @ONLY)

message(STATUS "Configuring valhalla library target")
set(valhalla_hdrs
  ${CMAKE_BINARY_DIR}/valhalla/valhalla.h
  ${CMAKE_SOURCE_DIR}/valhalla/exception.h
  ${CMAKE_SOURCE_DIR}/valhalla/worker.h)

add_library(valhalla worker.cc ${valhalla_hdrs})

if(MSVC)
  target_compile_definitions(valhalla PRIVATE "VC_EXTRALEAN;WIN32_LEAN_AND_MEAN;NOMINMAX;NOGDI")
endif()

target_include_directories(valhalla
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
    ${CMAKE_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/valhalla)

target_link_libraries(valhalla
  valhalla::mjolnir
  valhalla::baldr
  valhalla::tyr
  valhalla::midgard
  valhalla::meili
  valhalla::sif
  valhalla::skadi
  valhalla::protobuf)

set_target_properties(valhalla PROPERTIES
  VERSION "${VERSION}"
  SOVERSION "${VALHALLA_VERSION_MAJOR}")

set(LIBVALHALLA_SO_LINK ${CMAKE_SHARED_LIBRARY_PREFIX}valhalla${CMAKE_SHARED_LIBRARY_SUFFIX})

install(TARGETS valhalla
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shared NAMELINK_SKIP
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development)

if(BUILD_SHARED_LIBS)
  install(FILES ${CMAKE_SOURCE_DIR}/COPYING ${CMAKE_SOURCE_DIR}/ChangeLog
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/libvalhalla0"
    COMPONENT shared)
else()
  add_custom_command(TARGET valhalla POST_BUILD
    COMMAND ${CMAKE_AR} rc $<TARGET_FILE:valhalla>
      $<TARGET_FILE:valhalla::baldr>
      $<TARGET_FILE:valhalla::loki>
      $<TARGET_FILE:valhalla::meili>
      $<TARGET_FILE:valhalla::midgard>
      $<TARGET_FILE:valhalla::mjolnir>
      $<TARGET_FILE:valhalla::odin>
      $<TARGET_FILE:valhalla::sif>
      $<TARGET_FILE:valhalla::skadi>
      $<TARGET_FILE:valhalla::thor>
      $<TARGET_FILE:valhalla::tyr>
      $<TARGET_FILE:valhalla::mjolnir>)

  add_custom_target(${LIBVALHALLA_SO_LINK} ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink "${LIBVALHALLA_SO_LINK}.${VALHALLA_VERSION_MAJOR}" ${LIBVALHALLA_SO_LINK})
endif()

install(FILES ${CMAKE_SOURCE_DIR}/COPYING ${CMAKE_SOURCE_DIR}/ChangeLog ${CMAKE_SOURCE_DIR}/README.md
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/libvalhalla-dev"
  COMPONENT development)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${LIBVALHALLA_SO_LINK}
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  COMPONENT development)

install(FILES ${valhalla_hdrs}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/valhalla"
  COMPONENT development)


## pkg-config file
set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix ${CMAKE_INSTALL_PREFIX})
set(libdir ${CMAKE_INSTALL_LIBDIR})
set(includedir ${CMAKE_INSTALL_INCLUDEDIR}/valhalla)
configure_file(${CMAKE_SOURCE_DIR}/libvalhalla.pc.in libvalhalla.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libvalhalla.pc
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig
  COMPONENT development)
