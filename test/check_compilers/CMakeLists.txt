cmake_minimum_required(VERSION 3.16)
project(hello CXX)

# include all cc files
file(GLOB SRC_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")

# make the header a fake interface library target for convenience
add_library(common_headers INTERFACE)
target_include_directories(common_headers INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(common_headers INTERFACE cxx_std_17)

foreach(src ${SRC_FILES})
  get_filename_component(name "${src}" NAME_WE)

  add_executable(${name} "${src}")

  # exposes the common header path (and any flags)
  target_link_libraries(${name} PRIVATE common_headers)
endforeach()
