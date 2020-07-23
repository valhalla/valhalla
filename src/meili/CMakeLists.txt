file(GLOB headers ${valhalla_SOURCE_DIR}/valhalla/meili/*.h)

set(sources
  viterbi_search.cc
  topk_search.cc
  routing.cc
  candidate_search.cc
  transition_cost_model.cc
  map_matcher.cc
  map_matcher_factory.cc
  match_route.cc)

valhalla_module(NAME meili
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${valhalla_SOURCE_DIR}
      ${valhalla_SOURCE_DIR}/valhalla
      ${valhalla_SOURCE_DIR}/third_party/date/include
      $<$<BOOL:${MSVC}>:${valhalla_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${valhalla_SOURCE_DIR}/third_party/rapidjson/include
      ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::sif
    ${valhalla_protobuf_targets}
    Boost::boost)
