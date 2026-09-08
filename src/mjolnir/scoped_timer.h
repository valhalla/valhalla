#pragma once
#include "config.h"
#include "midgard/logging.h"
#include "midgard/util.h"
#include "mjolnir/util.h"

#include <algorithm>
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
        /* Emit to statsd: build.timing.<file_stem>.<func> */                                        \
        std::string _st_stem = relative_file_path;                                                   \
        /* strip directory and extension to get e.g. "elevationbuilder" */                           \
        auto _st_slash = _st_stem.rfind('/');                                                        \
        if (_st_slash != std::string::npos)                                                          \
          _st_stem = _st_stem.substr(_st_slash + 1);                                                 \
        auto _st_dot = _st_stem.rfind('.');                                                          \
        if (_st_dot != std::string::npos)                                                            \
          _st_stem = _st_stem.substr(0, _st_dot);                                                    \
        valhalla::mjolnir::build_stats::get()                                                        \
            .record_timing(std::string("mjolnir.timing.") + _st_stem + "." + std::string(func_name), \
                           static_cast<uint64_t>(_scoped_timer_duration));                           \
      })

} // namespace mjolnir
} // namespace valhalla
