## Process the IANA Time Zone Database files
set(tz_db_files africa antarctica asia australasia backward etcetera europe
  northamerica southamerica leapseconds)

# Process the leap seconds file (it's in .gitignore of the tz repo)
add_custom_command(OUTPUT ${VALHALLA_SOURCE_DIR}/third_party/tz/leapseconds
COMMAND make leapseconds
WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}/third_party/tz
COMMENT "Compiling third_party/tz/leapseconds with awk"
DEPENDS ${VALHALLA_SOURCE_DIR}/third_party/tz/leapseconds.awk)

foreach(tz_db_file ${tz_db_files})
  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/date_time_${tz_db_file}.h
    COMMAND ${CMAKE_COMMAND} -DMSVC=${MSVC} -P ${VALHALLA_SOURCE_DIR}/cmake/ValhallaBin2Header.cmake
        ${VALHALLA_SOURCE_DIR}/third_party/tz/${tz_db_file}
        ${CMAKE_CURRENT_BINARY_DIR}/date_time_${tz_db_file}.h
        --variable-name date_time_${tz_db_file} --skip 1 --raw
    WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
    COMMENT "Compiling third_party/tz/${tz_db_file} to date_time_${tz_db_file}.h"
    DEPENDS ${VALHALLA_SOURCE_DIR}/third_party/tz/${tz_db_file} ${VALHALLA_SOURCE_DIR}/third_party/tz/leapseconds)
endforeach()

# Process the windowsZones.xml file
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/date_time_windows_zones.h
  COMMAND ${CMAKE_COMMAND} -DMSVC=${MSVC} -P ${VALHALLA_SOURCE_DIR}/cmake/ValhallaBin2Header.cmake
  ${VALHALLA_SOURCE_DIR}/date_time/windowsZones.xml
  ${CMAKE_CURRENT_BINARY_DIR}/date_time_windows_zones.h
  --variable-name date_time_windows_zones_xml --skip 1 --raw
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
  COMMENT "Compiling date_time/windowsZones.xml to date_time_windows_zones.h"
  DEPENDS ${VALHALLA_SOURCE_DIR}/date_time/windowsZones.xml)

file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/baldr/*.h)

set(includes
    ${VALHALLA_SOURCE_DIR}
     ${VALHALLA_SOURCE_DIR}/valhalla
     ${CMAKE_CURRENT_BINARY_DIR}/src/baldr
     )

set(system_includes 
  ${date_include_dir}
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>
  ${rapidjson_include_dir})

set(sources
    accessrestriction.cc
    admin.cc
    attributes_controller.cc
    compression_utils.cc
    connectivity_map.cc
    curler.cc
    datetime.cc
    directededge.cc
    edgeinfo.cc
    graphid.cc
    graphreader.cc
    graphtile.cc
    graphtileheader.cc
    incident_singleton.h
    edgetracker.cc
    nodeinfo.cc
    location.cc
    merge.cc
    pathlocation.cc
    predictedspeeds.cc
    tilehierarchy.cc
    timedomain.cc
    turn.cc
    shortcut_recovery.h
    streetname.cc
    streetnames.cc
    streetnames_factory.cc
    streetname_us.cc
    streetnames_us.cc
    transitdeparture.cc
    transitroute.cc
    transitschedule.cc
    transittransfer.cc
    tz_alt.cpp
    laneconnectivity.cc
    verbal_text_formatter.cc
    verbal_text_formatter_us.cc
    verbal_text_formatter_us_co.cc
    verbal_text_formatter_us_tx.cc
    verbal_text_formatter_factory.cc)

list(APPEND sources
    #basic timezone stuff
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_africa.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_antarctica.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_asia.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_australasia.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_backward.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_etcetera.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_europe.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_leapseconds.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_northamerica.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_southamerica.h
    ${CMAKE_CURRENT_BINARY_DIR}/date_time_windows_zones.h)

list(APPEND sources_with_warnings
    tz_alt.cpp)

#ios we have more work to make use of system tzdb
if(APPLE)
  list(APPEND sources ${VALHALLA_SOURCE_DIR}/third_party/date/src/ios.mm)
endif()

valhalla_module(NAME baldr
  SOURCES
    ${sources}
  SOURCES_WITH_WARNINGS
    ${sources_with_warnings}

  HEADERS
    ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${includes}
    PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}

  DEPENDS
    valhalla::midgard
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost
    ${curl_targets}
    PkgConfig::ZLIB)
