set(module baldr)
set(library valhalla-${module})

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/valhalla/date_time_zonespec.h
  COMMAND ${XXD} -i -s `head -1 date_time/zonespec.csv | wc -c` date_time/zonespec.csv ${CMAKE_BINARY_DIR}/valhalla/date_time_zonespec.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling date_time/zonespec.csv to date_time_zonespec.h"
  DEPENDS ${CMAKE_SOURCE_DIR}/date_time/zonespec.csv)

set(headers
  ${CMAKE_BINARY_DIR}/valhalla/date_time_zonespec.h

  ${CMAKE_SOURCE_DIR}/valhalla/baldr/accessrestriction.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/admin.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/admininfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/complexrestriction.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/connectivity_map.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/curler.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/datetime.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/directededge.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/double_bucket_queue.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/edge_elevation.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/edgeinfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/filesystem_utils.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphconstants.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphid.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphreader.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphtile.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphtileheader.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/json.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/nodeinfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/location.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/pathlocation.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/rapidjson_utils.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/sign.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/signinfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/tilehierarchy.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/turn.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetname.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetnames.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetnames_factory.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetname_us.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetnames_us.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/trafficassociation.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transitdeparture.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transitroute.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transitschedule.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transitstop.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transittransfer.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/laneconnectivity.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter_us.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter_us_co.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter_us_tx.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter_factory.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/reutil.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/edgetracker.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/merge.h)

set(sources
  accessrestriction.cc
  admin.cc
  connectivity_map.cc
  datetime.cc
  directededge.cc
  edge_elevation.cc
  edgeinfo.cc
  graphid.cc
  graphreader.cc
  graphtile.cc
  graphtileheader.cc
  edgetracker.cc
  merge.cc
  nodeinfo.cc
  location.cc
  pathlocation.cc
  tilehierarchy.cc
  turn.cc
  streetname.cc
  streetnames.cc
  streetnames_factory.cc
  streetname_us.cc
  streetnames_us.cc
  transitdeparture.cc
  transitroute.cc
  transitschedule.cc
  transitstop.cc
  transittransfer.cc
  laneconnectivity.cc
  verbal_text_formatter.cc
  verbal_text_formatter_us.cc
  verbal_text_formatter_us_co.cc
  verbal_text_formatter_us_tx.cc
  verbal_text_formatter_factory.cc)

add_library(${library} STATIC ${sources} ${headers})
add_library(valhalla::${module} ALIAS ${library})
target_compile_definitions(${library} PUBLIC RAPIDJSON_HAS_STDSTRING)
target_include_directories(${library}
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
  PRIVATE
    ${CMAKE_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/valhalla)
target_link_libraries(${library}
  valhalla::midgard
  Boost::date_time
  Boost::filesystem
  Boost::iostreams
  Boost::system
  CURL::CURL)

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${module} COMPONENT development)

if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
  set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
