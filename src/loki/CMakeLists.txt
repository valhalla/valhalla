file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/loki/*.h)

set(sources
  search.cc
  worker.cc
  height_action.cc
  locate_action.cc
  reach.cc
  route_action.cc
  matrix_action.cc
  isochrone_action.cc
  status_action.cc
  trace_route_action.cc
  transit_available_action.cc
  node_search.cc
  polygon_search.cc)

if (UNIX AND NOT APPLE AND ENABLE_SINGLE_FILES_WERROR)
  # Enables stricter compiler checks on a file-by-file basis
  # which allows us to migrate piecemeal
  set_source_files_properties(
    search.cc
    worker.cc
    locate_action.cc
    height_action.cc
    polygon_search.cc
  PROPERTIES COMPILE_FLAGS "-Wall -Werror")
endif ()

valhalla_module(NAME loki
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      ${VALHALLA_SOURCE_DIR}/third_party/date/include
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
      ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::skadi
    valhalla::sif
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost
    libprime_server)
