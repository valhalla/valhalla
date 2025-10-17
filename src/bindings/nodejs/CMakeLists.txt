cmake_minimum_required(VERSION 3.15)

include(FetchContent)

# Find Node.js
find_program(NODE_EXECUTABLE NAMES node nodejs REQUIRED)
if(NOT NODE_EXECUTABLE)
  message(FATAL_ERROR "Node.js not found. Please install Node.js to build Node.js bindings.")
endif()

# Get Node.js version
execute_process(
  COMMAND ${NODE_EXECUTABLE} -p "process.versions.node"
  OUTPUT_VARIABLE NODE_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Node.js version: ${NODE_VERSION}")

# Fetch node-addon-api from GitHub
FetchContent_Declare(
  node-addon-api
  GIT_REPOSITORY https://github.com/nodejs/node-addon-api.git
  GIT_TAG v8.2.1  # Use a stable version
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(node-addon-api)

message(STATUS "node-addon-api fetched from GitHub")

# Get node include directory
execute_process(
  COMMAND ${NODE_EXECUTABLE} -e "console.log(require('path').join(process.execPath, '..', '..', 'include', 'node'))"
  OUTPUT_VARIABLE NODE_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)

# If the above doesn't work, try alternative methods
if(NOT EXISTS "${NODE_INCLUDE_DIR}")
  execute_process(
    COMMAND ${NODE_EXECUTABLE} -p "process.execPath"
    OUTPUT_VARIABLE NODE_EXEC_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  string(REPLACE "\"" "" NODE_EXEC_PATH "${NODE_EXEC_PATH}")
  get_filename_component(NODE_BIN_DIR "${NODE_EXEC_PATH}" DIRECTORY)
  get_filename_component(NODE_ROOT_DIR "${NODE_BIN_DIR}" DIRECTORY)
  set(NODE_INCLUDE_DIR "${NODE_ROOT_DIR}/include/node")
endif()

if(EXISTS "${NODE_INCLUDE_DIR}")
  message(STATUS "Node.js include directory: ${NODE_INCLUDE_DIR}")
else()
  message(WARNING "Could not find Node.js include directory at ${NODE_INCLUDE_DIR}")
endif()

# Create the addon library
add_library(valhalla_node SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/valhalla.cc)

# Set target properties
set_target_properties(valhalla_node PROPERTIES
  PREFIX ""
  SUFFIX ".node"
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON
)

# Add include directories
target_include_directories(valhalla_node PRIVATE
  ${node-addon-api_SOURCE_DIR}
  ${VALHALLA_SOURCE_DIR}
  ${VALHALLA_SOURCE_DIR}/valhalla
)

if(NODE_INCLUDE_DIR AND EXISTS "${NODE_INCLUDE_DIR}")
  target_include_directories(valhalla_node PRIVATE ${NODE_INCLUDE_DIR})
endif()

# Link against Valhalla
target_link_libraries(valhalla_node PRIVATE valhalla)

# Define NAPI settings
target_compile_definitions(valhalla_node PRIVATE
  NAPI_VERSION=8
  NODE_ADDON_API_DISABLE_DEPRECATED
  NAPI_DISABLE_CPP_EXCEPTIONS
)

# Platform-specific settings
if(APPLE)
  target_link_options(valhalla_node PRIVATE
    -undefined dynamic_lookup
  )
elseif(WIN32)
  # Windows-specific settings
  if(NODE_INCLUDE_DIR AND EXISTS "${NODE_INCLUDE_DIR}")
    # Find node.lib
    get_filename_component(NODE_ROOT_DIR "${NODE_INCLUDE_DIR}" DIRECTORY)
    find_library(NODE_LIB
      NAMES node.lib libnode.lib
      PATHS "${NODE_ROOT_DIR}" "${NODE_ROOT_DIR}/lib"
      NO_DEFAULT_PATH
    )
    if(NODE_LIB)
      target_link_libraries(valhalla_node PRIVATE ${NODE_LIB})
    endif()
  endif()

  target_compile_definitions(valhalla_node PRIVATE
    _HAS_EXCEPTIONS=0
    WIN32_LEAN_AND_MEAN
  )
endif()

# Set output directory
set_target_properties(valhalla_node PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib
)

# Installation
install(TARGETS valhalla_node
  LIBRARY DESTINATION lib/nodejs
  RUNTIME DESTINATION lib/nodejs
)

message(STATUS "Node.js bindings configured successfully")
