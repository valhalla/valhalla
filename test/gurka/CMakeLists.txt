if(ENABLE_DATA_TOOLS)
  file(GLOB_RECURSE TEST_FILES "${CMAKE_CURRENT_LIST_DIR}/test_*.cc")

  add_custom_target(gurka)
  add_custom_target(run-gurka)

  foreach(FULLFILENAME IN ITEMS ${TEST_FILES})
    file(RELATIVE_PATH FILENAME "${CMAKE_CURRENT_LIST_DIR}" ${FULLFILENAME})
    string(REGEX REPLACE "test_(.*).cc" "gurka_\\1" TESTNAME ${FILENAME})
    add_executable(${TESTNAME} EXCLUDE_FROM_ALL ${FILENAME})
    set_target_properties(${TESTNAME} PROPERTIES
      COMPILE_DEFINITIONS VALHALLA_SOURCE_DIR="${VALHALLA_SOURCE_DIR}/")
    target_link_libraries(${TESTNAME} valhalla gtest_main gmock)
    target_include_directories(${TESTNAME} SYSTEM PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/protozero/include
      ${VALHALLA_SOURCE_DIR}/third_party/libosmium/include)

    add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/test/data/tz.sqlite
          COMMAND ${CMAKE_COMMAND} -E make_directory test/data/
          COMMAND ${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_timezones > ${CMAKE_BINARY_DIR}/test/data/tz.sqlite
          COMMENT "Building tz.sqlite..."
          WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )

    add_custom_command(OUTPUT ${TESTNAME}.log
      COMMAND
        LOCPATH=${VALHALLA_SOURCE_DIR}/locales
        /bin/bash -c "${CMAKE_CURRENT_BINARY_DIR}/${TESTNAME} >& ${CMAKE_CURRENT_BINARY_DIR}/${TESTNAME}.log \
        && echo [PASS] ${TESTNAME} \
        || ( exit=$? ; \
             echo [FAIL] ${TESTNAME} ; \
             cat ${CMAKE_CURRENT_BINARY_DIR}/${TESTNAME}.log ; \
             exit $exit )"
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      DEPENDS ${TESTNAME}
      VERBATIM)
    add_custom_target(run-${TESTNAME} DEPENDS ${TESTNAME}.log)
    add_dependencies(gurka ${TESTNAME})
    add_dependencies(run-gurka run-${TESTNAME})
  endforeach()

  add_dependencies(tests gurka)
  add_dependencies(check run-gurka)

endif()