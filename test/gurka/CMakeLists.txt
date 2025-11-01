if(ENABLE_DATA_TOOLS)
  file(GLOB_RECURSE TEST_FILES "${CMAKE_CURRENT_LIST_DIR}/test_*.cc")

  add_custom_target(gurka)
  set_target_properties(gurka PROPERTIES FOLDER "Tests")
  add_custom_target(run-gurka)
  set_target_properties(run-gurka PROPERTIES FOLDER "Tests")

  ## Add executable targets
  foreach(FULLFILENAME IN ITEMS ${TEST_FILES})
    file(RELATIVE_PATH FILENAME "${CMAKE_CURRENT_LIST_DIR}" ${FULLFILENAME})
    if (UNIX AND ENABLE_SINGLE_FILES_WERROR)
        set_source_files_properties(${FILENAME} PROPERTIES COMPILE_FLAGS "-Wall -Werror")
    endif()

    string(REGEX REPLACE "test_(.*).cc" "gurka_\\1" TESTNAME ${FILENAME})
    add_executable(${TESTNAME} EXCLUDE_FROM_ALL ${FILENAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER "Tests")
    target_compile_definitions(${TESTNAME} PRIVATE
      VALHALLA_SOURCE_DIR="${VALHALLA_SOURCE_DIR}/"
      VALHALLA_BUILD_DIR="${VALHALLA_BUILD_DIR}/")
    create_source_groups("Source Files" ${FILENAME})
    target_link_libraries(${TESTNAME} valhalla_test)
    if (LuaJIT_FOUND AND APPLE AND CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
      # Using LuaJIT on macOS on Intel processors requires a couple of extra linker flags
      target_link_options(${TESTNAME} PUBLIC -pagezero_size 10000 -image_base 100000000)
    endif()
    add_dependencies(${TESTNAME} build_timezones)

    ## Test run targets
    add_custom_command(OUTPUT ${TESTNAME}.log
      COMMAND ${CMAKE_COMMAND} -E env LOCPATH=${VALHALLA_SOURCE_DIR}/locales ${VALHALLA_SOURCE_DIR}/scripts/run_single_test.sh ${CMAKE_CURRENT_BINARY_DIR}/${TESTNAME}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      DEPENDS ${TESTNAME}
      VERBATIM)
    add_custom_target(run-${TESTNAME} DEPENDS ${TESTNAME}.log)
    set_target_properties(run-${TESTNAME} PROPERTIES FOLDER "Tests")
    add_dependencies(gurka ${TESTNAME})
    add_dependencies(run-gurka run-${TESTNAME})
  endforeach()

  add_dependencies(tests gurka)
  add_dependencies(check run-gurka)
endif()
