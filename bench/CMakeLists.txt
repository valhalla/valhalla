set(BENCHMARK_ENABLE_TESTING OFF)
add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/benchmark
  ${CMAKE_BINARY_DIR}/benchmark)
set_target_properties(benchmark PROPERTIES FOLDER "Dependencies")
set_target_properties(benchmark_main PROPERTIES FOLDER "Dependencies")

# Custom targets building and running all microbenchmarks in the project
add_custom_target(benchmarks)
set_target_properties(benchmarks PROPERTIES FOLDER "Benchmarks")
add_custom_target(run-benchmarks)
set_target_properties(run-benchmarks PROPERTIES FOLDER "Benchmarks")

# Benchmarks generally require utrecht test tiles to be present, so add this dependency by default.
add_dependencies(benchmarks utrecht_tiles)

# Macro for defining Google Benchmark microbenchmark targets
macro (add_valhalla_benchmark target_file)
  set(target_name benchmark-${target_file})
  add_executable(${target_name}
    ${target_file}.cc)
  set_target_properties(${target_name} PROPERTIES FOLDER "Benchmarks")
  set_target_properties(${target_name}
    PROPERTIES
    COMPILE_DEFINITIONS
    VALHALLA_SOURCE_DIR="${VALHALLA_SOURCE_DIR}/")
  target_link_libraries(${target_name} valhalla_test benchmark::benchmark)
  add_dependencies(benchmarks ${target_name})
  add_dependencies(${target_name} utrecht_tiles)
  # Add a custom target running the benchmark
  add_custom_target(run-${target_name}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${target_name}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running ${target_name} in ${CMAKE_CURRENT_BINARY_DIR}"
    DEPENDS ${target_name})
  set_target_properties(run-${target_name} PROPERTIES FOLDER "Benchmarks")
  add_dependencies(run-${target_name} utrecht_tiles)
  add_dependencies(run-benchmarks run-${target_name})
endmacro()

add_subdirectory(meili)
add_subdirectory(thor)
