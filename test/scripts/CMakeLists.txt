# be aware: the python modules that need testing, need to be symlinked to the test/scripts folder with a .py extension
find_package(Python COMPONENTS Interpreter)
if (Python_FOUND)
  add_custom_command(OUTPUT scripts.log
      COMMAND
        LOCPATH=${VALHALLA_SOURCE_DIR}/locales
        ASAN_OPTIONS=verify_asan_link_order=0:detect_leaks=0
        /bin/bash -cx "${Python_EXECUTABLE} -m unittest discover -s ${VALHALLA_SOURCE_DIR}/test/scripts -v > ${CMAKE_BINARY_DIR}/test/scripts.log 2>&1"
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      DEPENDS
        ${VALHALLA_SOURCE_DIR}/test/scripts/test_valhalla_build_config.py
      VERBATIM)
  add_custom_target(run-scripts DEPENDS scripts.log)
  set_target_properties(run-scripts PROPERTIES FOLDER "Scripts")
  add_dependencies(check run-scripts)
else()
  message(WARNING "Python executable not found. Skipping run-scripts test target...")
endif()
