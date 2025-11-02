file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/heimdall/*.h)

set(sources
  util.cc
  worker.cc)

set(system_includes
  ${date_include_dir}
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>)

valhalla_module(NAME heimdall
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
    PRIVATE
      ${rapidjson_include_dir}
      ${vtzero_include_dir}
      ${VALHALLA_SOURCE_DIR}/third_party/protozero/include
  DEPENDS
    valhalla::meili
    valhalla::baldr
    Boost::boost)

