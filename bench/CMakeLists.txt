add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/benchmark
  ${CMAKE_BINARY_DIR}/benchmark)

# Custom target for all microbenchmarks in the project
add_custom_target(benchmarks)

# Macro for defining Google Benchmark microbenchmark targets
macro (add_valhalla_benchmark target_file)
  set(target_name benchmark_${target_file})
  add_executable(${target_name} ${target_file}.cc)
  set_target_properties(${target_name}
    PROPERTIES
    COMPILE_DEFINITIONS
    VALHALLA_SOURCE_DIR="${VALHALLA_SOURCE_DIR}/")
  target_link_libraries(${target_name} valhalla benchmark::benchmark)
  add_dependencies(benchmarks ${target_name})
endmacro()

add_subdirectory(meili)
