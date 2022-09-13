file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/thor/*.h)

set(sources
  alternates.cc
  astar_bss.cc
  bidirectional_astar.cc
  centroid.cc
  costmatrix.cc
  dijkstras.cc
  expansion_action.cc
  isochrone_action.cc
  isochrone.cc
  map_matcher.cc
  matrix_action.cc
  multimodal.cc
  optimized_route_action.cc
  optimizer.cc
  route_action.cc
  route_matcher.cc
  status_action.cc
  timedistancematrix.cc
  timedistancebssmatrix.cc
  trace_attributes_action.cc
  trace_route_action.cc
  triplegbuilder.cc
  triplegbuilder_utils.h
  unidirectional_astar.cc
  worker.cc)

if (UNIX AND NOT APPLE AND ENABLE_SINGLE_FILES_WERROR)
  # Enables stricter compiler checks on a file-by-file basis
  # which allows us to migrate piecemeal
  set_source_files_properties(
    alternates.cc
    attributes_controller.cc
    triplegbuilder.cc
    worker.cc
  PROPERTIES COMPILE_FLAGS "-Wall -Werror")
endif ()

valhalla_module(NAME thor
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
    valhalla::proto
    valhalla::sif
    valhalla::meili
    ${valhalla_protobuf_targets}
    Boost::boost
    libprime_server
    robin_hood::robin_hood)
