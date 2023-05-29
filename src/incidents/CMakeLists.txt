file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/incidents/*.h)

set(sources worker.cc incident_action.cc)

# treat date library as system
# treat date library as system
set(system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include)
if(APPLE)
  list(APPEND system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include/date)
endif()

valhalla_module(NAME incidents
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${CMAKE_BINARY_DIR}/src/incidents
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
  DEPENDS
    valhalla::sif
    ${valhalla_protobuf_targets}
    Boost::boost
    libprime_server)
