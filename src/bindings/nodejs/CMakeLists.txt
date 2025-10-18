cmake_minimum_required(VERSION 3.15)

include(FetchContent)

# Fetch node-addon-api from GitHub
FetchContent_Declare(
  node-addon-api
  GIT_REPOSITORY https://github.com/nodejs/node-addon-api.git
  GIT_TAG v8.5.0
  GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(node-addon-api)

message(STATUS "node-addon-api fetched from GitHub")

# Fetch Node.js headers - using a stable LTS version
# N-API is ABI-stable, so this will work with any compatible Node.js version
set(NODE_VERSION "22.18.0")
FetchContent_Declare(
  node-headers
  URL https://nodejs.org/dist/v${NODE_VERSION}/node-v${NODE_VERSION}-headers.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(node-headers)

set(NODE_INCLUDE_DIR "${node-headers_SOURCE_DIR}/include/node")

message(STATUS "Using Node.js headers version ${NODE_VERSION}")
message(STATUS "Node.js include directory: ${NODE_INCLUDE_DIR}")

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
  ${NODE_INCLUDE_DIR}
  ${VALHALLA_SOURCE_DIR}
  ${VALHALLA_SOURCE_DIR}/valhalla
)

# Link against Valhalla
target_link_libraries(valhalla_node PRIVATE valhalla $<$<BOOL:${ENABLE_COVERAGE}>:gcov>)

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
