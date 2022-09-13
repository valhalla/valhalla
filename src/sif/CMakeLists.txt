file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/sif/*.h)

set(sources
  autocost.cc
  bicyclecost.cc
  hierarchylimits.cc
  motorcyclecost.cc
  motorscootercost.cc
  nocost.cc
  pedestriancost.cc
  transitcost.cc
  truckcost.cc
  dynamiccost.cc
  recost.cc)

if (UNIX AND ENABLE_SINGLE_FILES_WERROR)
  # Enables stricter compiler checks on a file-by-file basis
  # which allows us to migrate piecemeal
  set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "-Werror -Wall")
endif()

valhalla_module(NAME sif
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
  DEPENDS
    valhalla::baldr
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost)
