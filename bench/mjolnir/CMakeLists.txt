add_valhalla_benchmark(buildtiles)

add_custom_target(generate-config
        COMMAND python3 ${PROJECT_SOURCE_DIR}/scripts/valhalla_build_config
          --mjolnir-tile-dir ${CMAKE_CURRENT_BINARY_DIR}/tiles
          > ${CMAKE_CURRENT_BINARY_DIR}/config.json
        DEPENDS ${PROJECT_SOURCE_DIR}/scripts/valhalla_build_config)

add_dependencies(benchmark-buildtiles generate-config)
