add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/date_time_zonespec.h
  COMMAND ${CMAKE_COMMAND} -P ${VALHALLA_SOURCE_DIR}/cmake/Binary2Header.cmake
      ${VALHALLA_SOURCE_DIR}/date_time/zonespec.csv
      ${CMAKE_CURRENT_BINARY_DIR}/date_time_zonespec.h
      --variable-name date_time_zonespec_csv --skip 1 --raw
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
  COMMENT "Compiling date_time/zonespec.csv to date_time_zonespec.h"
  DEPENDS ${VALHALLA_SOURCE_DIR}/date_time/zonespec.csv)

file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/baldr/*.h)

valhalla_module(NAME baldr
  SOURCES
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_zonespec.h

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
    transittransfer.cc
    laneconnectivity.cc
    verbal_text_formatter.cc
    verbal_text_formatter_us.cc
    verbal_text_formatter_us_co.cc
    verbal_text_formatter_us_tx.cc
    verbal_text_formatter_factory.cc

  HEADERS
    ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
    PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    valhalla::midgard
    Boost::date_time
    Boost::filesystem
    Boost::iostreams
    Boost::system
    CURL::CURL)
