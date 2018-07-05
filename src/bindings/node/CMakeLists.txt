# node-cmake requires CMake 3.1 features; for the valhalla project we only
# require CMake 2.8.11 so that we can build e.g. on Trusty by default.
cmake_minimum_required(VERSION 3.1)

message(STATUS "Building node_valhalla")

set(BINDING_DIR "${PROJECT_SOURCE_DIR}/lib/binding")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/node_modules/node-cmake")

include(NodeJS)
nodejs_init()

message(STATUS "Configuring node_valhalla bindings for NodeJs ${NODEJS_VERSION}")

add_nodejs_module(node_valhalla node.cc)
set_target_properties(node_valhalla PROPERTIES CXX_STANDARD 11)
target_link_libraries(node_valhalla valhalla)

# node_valhalla artifacts in ${BINDING_DIR} to depend targets on
set(ARTIFACTS "")

# TODO: copy useful valhalla tools into node module for easy distribution
#set(OSRM_BINARIES osrm-extract osrm-contract osrm-routed osrm-datastore osrm-components osrm-partition osrm-customize)
#foreach(binary ${OSRM_BINARIES})
#  add_custom_command(OUTPUT ${BINDING_DIR}/${binary}
#                     COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${binary}> ${BINDING_DIR}
#                     DEPENDS ${binary} ${BINDING_DIR})
#  list(APPEND ARTIFACTS "${BINDING_DIR}/${binary}")
#endforeach(binary)

add_custom_command(OUTPUT ${BINDING_DIR}/node_valhalla.node
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:node_valhalla> ${BINDING_DIR}
                   DEPENDS node_valhalla ${BINDING_DIR})
list(APPEND ARTIFACTS "${BINDING_DIR}/node_valhalla.node")


message(STATUS "node_valhalla artifacts will be copied to: ${BINDING_DIR}")
add_custom_target(copy_node_artifacts ALL DEPENDS ${ARTIFACTS})
