add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/googletest ${CMAKE_BINARY_DIR}/googletest)

set_target_properties(gtest PROPERTIES FOLDER "Dependencies")
set_target_properties(gtest_main PROPERTIES FOLDER "Dependencies")
set_target_properties(gmock PROPERTIES FOLDER "Dependencies")
set_target_properties(gmock_main PROPERTIES FOLDER "Dependencies")

# common things that the tests need
set(TEST_SRCS test.h test.cc)
if(ENABLE_DATA_TOOLS)
  list(APPEND TEST_SRCS gurka/gurka.h gurka/gurka.cc)
endif()
add_library(valhalla_test
  ${TEST_SRCS}
  ${VALHALLA_SOURCE_DIR}/third_party/microtar/src/microtar.h
  ${VALHALLA_SOURCE_DIR}/third_party/microtar/src/microtar.c
  )
target_include_directories(valhalla_test PUBLIC
  ${VALHALLA_SOURCE_DIR}/test
  ${VALHALLA_SOURCE_DIR}/test/gurka)
target_include_directories(valhalla_test SYSTEM PUBLIC
  ${robinhoodhashing_include_dir}
  ${VALHALLA_SOURCE_DIR}/third_party/just_gtfs/include
  ${VALHALLA_SOURCE_DIR}/third_party/protozero/include
  ${VALHALLA_SOURCE_DIR}/third_party/libosmium/include
  ${VALHALLA_SOURCE_DIR}/third_party/microtar/src)
target_link_libraries(valhalla_test valhalla gtest gtest_main gmock pthread PkgConfig::LZ4)

if(ENABLE_DATA_TOOLS)
  target_link_libraries(valhalla_test PkgConfig::GEOS)
endif()


## Lists tests
set(tests aabb2 access_restriction actor admin attributes_controller configuration datetime directededge
  distanceapproximator double_bucket_queue edgecollapser edgestatus ellipse encode
  enhancedtrippath factory graphid graphtile graphtileheader gridded_data grid_range_query grid_traversal instructions
  json laneconnectivity linesegment2 location logging maneuversbuilder map_matcher_factory mapmatch_config
  narrative_dictionary nodeinfo nodetransition obb2 openlr optimizer parse_request point2 pointll pointtileindex
  polyline2 predictedspeeds queue routing sample sequence sign signs statsd streetname streetnames streetnames_factory
  streetnames_us streetname_us tilehierarchy tiles transitdeparture transitroute transitschedule
  transitstop turn turnlanes util_midgard util_skadi vector2 verbal_text_formatter verbal_text_formatter_us
  verbal_text_formatter_us_co verbal_text_formatter_us_tx viterbi_search compression filesystem traffictile
  incident_loading worker_nullptr_tiles curl_tilegetter)

if(ENABLE_DATA_TOOLS)
  list(APPEND tests astar astar_bikeshare complexrestriction countryaccess edgeinfobuilder graphbuilder graphparser
    graphtilebuilder graphreader isochrone predictive_traffic idtable mapmatch matrix matrix_bss minbb multipoint_routes
    names node_search reach recover_shortcut refs search servicedays shape_attributes signinfo summary urban tar_index
    thor_worker timedep_paths timeparsing trivial_paths uniquenames util_mjolnir utrecht lua alternates)
  if(ENABLE_HTTP)
    list(APPEND tests http_tiles)
    # TODO: fix https://github.com/valhalla/valhalla/issues/3740
    # list(APPEND tests http_tiles elevation_builder)
  endif()
endif()

if(ENABLE_SERVICES)
  list(APPEND tests loki_service skadi_service)
endif()

## TODO: fix apple tests!
if(NOT APPLE)
  list(APPEND tests narrativebuilder util_odin)
endif()

# tests files that have warning,
# any other file not int this list will be processed with the
# "-Werror" flag
# Avoid adding a test to this list
set(tests_with_warnings
  filesystem
  graphbuilder
  graphtilebuilder
  gridded_data
  maneuversbuilder
  narrativebuilder
  parse_request
  sample
  sequence
  statsd
  traffictile
  util_odin
  timedep_paths
  worker
  )

## Add executable targets
foreach(test ${tests})
  add_executable(${test} EXCLUDE_FROM_ALL ${test}.cc )
  if (UNIX AND ENABLE_SINGLE_FILES_WERROR)
    if (${test} IN_LIST tests_with_warnings)
      set_source_files_properties(${test}.cc PROPERTIES COMPILE_FLAGS "-Wall")
    else()
      set_source_files_properties(${test}.cc PROPERTIES COMPILE_FLAGS "-Wall -Werror")
    endif()
  endif()


  set_target_properties(${test} PROPERTIES FOLDER "Tests")
  target_compile_definitions(${test} PRIVATE
      VALHALLA_SOURCE_DIR="${VALHALLA_SOURCE_DIR}/"
      VALHALLA_BUILD_DIR="${VALHALLA_BUILD_DIR}/")
  create_source_groups("Source Files" ${test}.cc)
  target_link_libraries(${test} valhalla_test)
  if (LUAJIT_FOUND AND APPLE AND CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
    # Using LuaJIT on macOS on Intel processors requires a couple of extra linker flags
    # Any tool that needs interact with Lua will need these flags on macOS
    target_link_options(${test} PUBLIC -pagezero_size 10000 -image_base 100000000)
  endif()
endforeach()

set(cost_tests autocost bicyclecost motorcyclecost motorscootercost pedestriancost transitcost truckcost)
foreach(test ${cost_tests})
  add_executable(${test} EXCLUDE_FROM_ALL ${VALHALLA_SOURCE_DIR}/src/sif/${test}.cc)
  set_target_properties(${test} PROPERTIES
    FOLDER "Tests"
    COMPILE_DEFINITIONS INLINE_TEST)
  create_source_groups("Source Files" ${VALHALLA_SOURCE_DIR}/src/sif/${test}.cc)
  target_link_libraries(${test} valhalla_test)
endforeach()

set(tyr_tests route_serializer_osrm)
foreach(test ${tyr_tests})
  add_executable(${test} EXCLUDE_FROM_ALL ${VALHALLA_SOURCE_DIR}/src/tyr/${test}.cc)
  set_target_properties(${test} PROPERTIES
    FOLDER "Tests"
    COMPILE_DEFINITIONS INLINE_TEST)
  create_source_groups("Source Files" ${VALHALLA_SOURCE_DIR}/src/tyr/${test}.cc)
  target_link_libraries(${test} valhalla_test)
endforeach()

## Test-specific data, properties and dependencies
target_compile_definitions(logging PRIVATE LOGGING_LEVEL_ALL)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/tz.sqlite
  DEPENDS ${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_timezones
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/
  COMMAND ${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_timezones > ${CMAKE_BINARY_DIR}/test/data/tz.sqlite.tmp
  COMMAND ${CMAKE_COMMAND} -E rename ${CMAKE_BINARY_DIR}/test/data/tz.sqlite.tmp ${CMAKE_BINARY_DIR}/test/data/tz.sqlite
  COMMENT "Building tz.sqlite..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_custom_target(build_timezones DEPENDS ${CMAKE_BINARY_DIR}/test/data/tz.sqlite)
set_target_properties(build_timezones PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/utrecht_tiles/traffic.tar
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/utrecht_tiles/
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/utrecht_tiles","timezone":"test/data/tz.sqlite","admin":"${VALHALLA_SOURCE_DIR}/test/data/netherlands_admin.sqlite","include_construction":true,"hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""},"data_processing":{"grid_divisions_within_tile":32}}}'
      -s initialize -e parseways
      ${VALHALLA_SOURCE_DIR}/test/data/utrecht_netherlands.osm.pbf
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/utrecht_tiles","timezone":"test/data/tz.sqlite","admin":"${VALHALLA_SOURCE_DIR}/test/data/netherlands_admin.sqlite","include_construction":true,"hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      -s parserelations -e parserelations
      ${VALHALLA_SOURCE_DIR}/test/data/utrecht_netherlands.osm.pbf
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/utrecht_tiles","timezone":"test/data/tz.sqlite","admin":"${VALHALLA_SOURCE_DIR}/test/data/netherlands_admin.sqlite","include_construction":true,"hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      -s parsenodes -e parsenodes
      ${VALHALLA_SOURCE_DIR}/test/data/utrecht_netherlands.osm.pbf
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/utrecht_tiles","timezone":"test/data/tz.sqlite","admin":"${VALHALLA_SOURCE_DIR}/test/data/netherlands_admin.sqlite","include_construction":true,"hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      -s build -e cleanup
      ${VALHALLA_SOURCE_DIR}/test/data/utrecht_netherlands.osm.pbf
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_add_predicted_traffic
      --inline-config '{"mjolnir":{"tile_dir":"test/data/utrecht_tiles","concurrency":1,"logging":{"type":""}}}'
      -t ${VALHALLA_SOURCE_DIR}/test/data/traffic_tiles/
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_extract
      --inline-config '{"mjolnir":{"tile_dir":"test/data/utrecht_tiles","tile_extract":"test/data/utrecht_tiles/tiles.tar","traffic_extract":"test/data/utrecht_tiles/traffic.tar","concurrency":1,"logging":{"type":""}}}'
      --with-traffic --overwrite
  COMMENT "Building Utrecht Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles valhalla_add_predicted_traffic build_timezones ${VALHALLA_SOURCE_DIR}/test/data/utrecht_netherlands.osm.pbf ${CMAKE_BINARY_DIR}/valhalla_build_extract)
add_custom_target(utrecht_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/utrecht_tiles/traffic.tar)
set_target_properties(utrecht_tiles PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/whitelion_tiles/2/000/814/309.gph
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/whitelion_tiles","hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/whitelion_bristol_uk.osm.pbf
  COMMENT "Building Whitelion Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles ${VALHALLA_SOURCE_DIR}/test/data/whitelion_bristol_uk.osm.pbf)
add_custom_target(whitelion_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/whitelion_tiles/2/000/814/309.gph)
set_target_properties(whitelion_tiles PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/whitelion_tiles_reverse/2/000/814/309.gph
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/whitelion_tiles_reverse","hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/whitelion_bristol_uk_reversed_oneway.osm.pbf
  COMMENT "Building reversed Whitelion Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles ${VALHALLA_SOURCE_DIR}/test/data/whitelion_bristol_uk_reversed_oneway.osm.pbf)
add_custom_target(reversed_whitelion_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/whitelion_tiles_reverse/2/000/814/309.gph)
set_target_properties(reversed_whitelion_tiles PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/bayfront_singapore_tiles/1/033/043.gph
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/bayfront_singapore_tiles","hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/bayfront_singapore.osm.pbf
      COMMENT "Building Singapore, Bayfront tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles ${VALHALLA_SOURCE_DIR}/test/data/bayfront_singapore.osm.pbf)
add_custom_target(bayfront_singapore_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/bayfront_singapore_tiles/1/033/043.gph)
set_target_properties(bayfront_singapore_tiles PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/roma_tiles/1/047/352.gph
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/roma_tiles/
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/roma_tiles","timezone":"test/data/tz.sqlite","hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/via_montebello_roma_italy.osm.pbf
  COMMENT "Building Roma Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles valhalla_add_predicted_traffic build_timezones ${VALHALLA_SOURCE_DIR}/test/data/via_montebello_roma_italy.osm.pbf)
add_custom_target(roma_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/roma_tiles/1/047/352.gph)
set_target_properties(roma_tiles PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/paris_bss_tiles/0/003/105.gph
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/paris_bss_tiles/
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/paris_bss_tiles","timezone":"test/data/tz.sqlite","hierarchy": true,"import_bike_share_stations": true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/paris_bss.osm.pbf
  COMMENT "Building Paris BSS Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles ${VALHALLA_SOURCE_DIR}/test/data/paris_bss.osm.pbf)
add_custom_target(paris_bss_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/paris_bss_tiles/0/003/105.gph)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/melborne_tiles/0/001/251.gph
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/melborne_tiles/
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/melborne_tiles","timezone":"test/data/tz.sqlite","hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/melborne.osm.pbf
  COMMENT "Building Melborne Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles valhalla_add_predicted_traffic build_timezones ${VALHALLA_SOURCE_DIR}/test/data/melborne.osm.pbf)
add_custom_target(melborne_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/melborne_tiles/0/001/251.gph)
set_target_properties(melborne_tiles PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/ny_ar_tiles/2/000/752/104.gph
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/ny_ar_tiles/
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/ny_ar_tiles","timezone":"test/data/tz.sqlite","hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/ny-access-restriction.osm.pbf
  COMMENT "Building New York Access Restriction Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles build_timezones ${VALHALLA_SOURCE_DIR}/test/data/ny-access-restriction.osm.pbf)
add_custom_target(ny_ar_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/ny_ar_tiles/2/000/752/104.gph)
set_target_properties(ny_ar_tiles PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/pa_ar_tiles/2/000/749/212.gph
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/pa_ar_tiles/
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/pa_ar_tiles","timezone":"test/data/tz.sqlite","hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/pa-access-restriction.osm.pbf
  COMMENT "Building PA Access Restriction Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles build_timezones ${VALHALLA_SOURCE_DIR}/test/data/pa-access-restriction.osm.pbf)
add_custom_target(pa_ar_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/pa_ar_tiles/2/000/749/212.gph)
set_target_properties(pa_ar_tiles PROPERTIES FOLDER "Tests")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/nh_ar_tiles/2/000/765/074.gph
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/nh_ar_tiles/
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles
      --inline-config '{"mjolnir":{"id_table_size":1000,"tile_dir":"test/data/nh_ar_tiles","timezone":"test/data/tz.sqlite","hierarchy":true,"shortcuts":true,"concurrency":1,"logging":{"type":""}}}'
      ${VALHALLA_SOURCE_DIR}/test/data/nh-access-restriction.osm.pbf
  COMMENT "Building New Hampshire Access Restriction Tiles..."
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  DEPENDS valhalla_build_tiles build_timezones ${VALHALLA_SOURCE_DIR}/test/data/nh-access-restriction.osm.pbf)
add_custom_target(nh_ar_tiles DEPENDS ${CMAKE_BINARY_DIR}/test/data/nh_ar_tiles/2/000/765/074.gph)
set_target_properties(nh_ar_tiles PROPERTIES FOLDER "Tests")

file(GLOB locales "${VALHALLA_SOURCE_DIR}/locales/*.json")
add_custom_command(OUTPUT .locales.timestamp
  COMMAND /bin/bash -c "for loc in $(jq --raw-output .posix_locale *.json); do \
       $<$<NOT:$<BOOL:${APPLE}>>:  localedef -i $\{loc\%.\*\} -f $\{loc\#\#\*.\} ./$\{loc\}   ; > \
       $<$<BOOL:${APPLE}>:         localedef -i /usr/share/locale/$\{loc\} ./$\{loc\} || true ; > \
     done \
     && touch ${CMAKE_CURRENT_BINARY_DIR}/.locales.timestamp"
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}/locales
  DEPENDS ${locales}
  COMMENT "Compile locale definition files..."
  VERBATIM)
add_custom_target(localedef DEPENDS .locales.timestamp)
set_target_properties(localedef PROPERTIES FOLDER "Tests")

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/test/data)
add_custom_command(OUTPUT test/data/sample/N00
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/sample/N00
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/sample/N40
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/samplegz/N40
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/samplelz4/N40
  COMMAND ${CMAKE_COMMAND} -E make_directory test/data/service
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMENT "Creating test directories")
add_custom_target(test_directories DEPENDS test/data/sample/N00)
set_target_properties(test_directories PROPERTIES FOLDER "Tests")

set (source "${CMAKE_BINARY_DIR}/test/data/utrecht_tiles")
set (destination "${CMAKE_BINARY_DIR}/test/data/tile_src")
# TODO: fix https://github.com/valhalla/valhalla/issues/3740
#set (elevationbuild_test "${CMAKE_BINARY_DIR}/test/data/elevationbuild/tile_dir")
#add_custom_command(
#        TARGET utrecht_tiles POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E make_directory ${destination}
#        COMMAND ${CMAKE_COMMAND} -E make_directory ${elevationbuild_test}
#        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/test/data/elevation_dst"
#        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/test/data/elevation_src"
#        COMMAND ${CMAKE_COMMAND} -E copy_directory ${source} ${destination}
#        COMMAND ${CMAKE_COMMAND} -E copy_directory ${source} ${elevationbuild_test}
#        DEPENDS ${destination}
#        COMMENT "tiles copied from ${source} to ${destination}"
#)

## Test run targets
foreach(test ${tests} ${cost_tests} ${tyr_tests})
  add_custom_command(OUTPUT ${test}.log
    COMMAND
      LOCPATH=${VALHALLA_SOURCE_DIR}/locales
      /bin/bash -c "${CMAKE_CURRENT_BINARY_DIR}/${test} >& ${CMAKE_CURRENT_BINARY_DIR}/${test}.log \
      && echo [PASS] ${test} \
      || ( exit=$? ; \
           echo [FAIL] ${test} ; \
           cat ${CMAKE_CURRENT_BINARY_DIR}/${test}.log ; \
           exit $exit )"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS ${test}
    VERBATIM)
  add_custom_target(run-${test} DEPENDS ${test}.log)
endforeach()

## Run test dependencies
## TODO: fix apple tests!
if(NOT APPLE)
  add_dependencies(run-util_odin localedef)
  add_dependencies(run-narrativebuilder localedef)
endif()

add_dependencies(run-sample test_directories)
if(ENABLE_DATA_TOOLS)
  add_dependencies(run-mapmatch utrecht_tiles)
  add_dependencies(run-isochrone utrecht_tiles)
  add_dependencies(run-matrix utrecht_tiles)
  add_dependencies(run-matrix_bss paris_bss_tiles)
  add_dependencies(run-timedep_paths utrecht_tiles)
  add_dependencies(run-trivial_paths utrecht_tiles)
  add_dependencies(predictive_traffic utrecht_tiles)
  add_dependencies(run-multipoint_routes utrecht_tiles)
  add_dependencies(run-reach utrecht_tiles)
  add_dependencies(run-shape_attributes utrecht_tiles)
  add_dependencies(run-summary utrecht_tiles)
  add_dependencies(run-urban utrecht_tiles)
  add_dependencies(run-thor_worker utrecht_tiles)
  add_dependencies(run-recover_shortcut utrecht_tiles)
  add_dependencies(run-minbb utrecht_tiles)
  add_dependencies(run-astar_bikeshare paris_bss_tiles)
  add_dependencies(run-astar whitelion_tiles roma_tiles reversed_whitelion_tiles bayfront_singapore_tiles ny_ar_tiles pa_ar_tiles nh_ar_tiles melborne_tiles utrecht_tiles)
  add_dependencies(run-alternates utrecht_tiles)
  add_dependencies(run-tar_index utrecht_tiles)
  add_dependencies(run-graphbuilder build_timezones)
  if(ENABLE_HTTP)
    add_dependencies(run-http_tiles utrecht_tiles)
  endif()
endif()

if(ENABLE_SERVICES)
  add_dependencies(run-skadi_service test_directories)
endif()

if(ENABLE_PYTHON_BINDINGS AND ENABLE_DATA_TOOLS)
  find_package(Python COMPONENTS Interpreter)
  add_custom_command(OUTPUT python_valhalla.log
    COMMAND
      LOCPATH=${VALHALLA_SOURCE_DIR}/locales
      PYTHONPATH=${CMAKE_BINARY_DIR}/src/bindings/python
      ASAN_OPTIONS=verify_asan_link_order=0:detect_leaks=0
      /bin/bash -cx "${Python_EXECUTABLE} -m unittest discover -s ${VALHALLA_SOURCE_DIR}/test/bindings/python -v > ${CMAKE_BINARY_DIR}/test/python_valhalla.log 2>&1 || cat ${CMAKE_BINARY_DIR}/test/python_valhalla.log"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS
      ${VALHALLA_SOURCE_DIR}/test/bindings/python/test_utrecht.py
      ${VALHALLA_SOURCE_DIR}/test/bindings/python/valhalla.json
      utrecht_tiles
      python_valhalla
    VERBATIM)
  add_custom_target(run-python_valhalla DEPENDS python_valhalla.log)
  set_target_properties(run-python_valhalla PROPERTIES FOLDER "Python Bindings")
  set(python_tests python_valhalla)
endif()

## High-level targets
string(REGEX REPLACE "([^;]+)" "run-\\1" test_targets "${tests};${cost_tests};${tyr_tests};${python_tests}")

add_custom_target(check DEPENDS ${test_targets})
set_target_properties(check PROPERTIES FOLDER "Tests")
add_custom_target(tests DEPENDS ${tests} ${cost_tests} ${tyr_tests})
set_target_properties(tests PROPERTIES FOLDER "Tests")
add_subdirectory(gurka)
add_subdirectory(scripts)
