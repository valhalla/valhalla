#pragma once
#include "config.h"

#include <midgard/logging.h>
#include <midgard/util.h>

#include <chrono>
#include <string>

namespace valhalla {
namespace mjolnir {

#define SCOPED_TIMER()                                                                               \
  auto _scoped_timer_start_##__COUNTER__ = std::chrono::high_resolution_clock::now();                \
  auto _scoped_timer_finally_##__COUNTER__ =                                                         \
      valhalla::midgard::make_finally([_scoped_timer_start_##__COUNTER__, func_name = __func__]() {  \
        auto _scoped_timer_end = std::chrono::high_resolution_clock::now();                          \
        auto _scoped_timer_duration = std::chrono::duration_cast<std::chrono::seconds>(              \
                                          _scoped_timer_end - _scoped_timer_start_##__COUNTER__)     \
                                          .count();                                                  \
        std::string file_path = __FILE__;                                                            \
        std::string valhalla_dir_str = VALHALLA_STRINGIZE(VALHALLA_SOURCE_DIR);                      \
        size_t len = valhalla_dir_str.length();                                                      \
        std::string relative_file_path = file_path.substr(len + 1);                                  \
        valhalla::midgard::logging::Log(std::string(relative_file_path) +                            \
                                            "::" + std::string(func_name) + " took " +               \
                                            std::to_string(_scoped_timer_duration) + "s",            \
                                        " [TIMING] ");                                               \
      })

} // namespace mjolnir
} // namespace valhalla
