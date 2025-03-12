#pragma once
#include <chrono>
#include <string>

#include "config.h"
#include <midgard/logging.h>
#include <midgard/util.h>

namespace valhalla {
namespace mjolnir {

#define SCOPED_TIMER()                                                                               \
  auto _scoped_timer_start_##__COUNTER__ = std::chrono::high_resolution_clock::now();                \
  auto _scoped_timer_finally_##__COUNTER__ =                                                         \
      valhalla::midgard::make_finally([_scoped_timer_start_##__COUNTER__]() {                        \
        auto _scoped_timer_end = std::chrono::high_resolution_clock::now();                          \
        auto _scoped_timer_duration = std::chrono::duration_cast<std::chrono::milliseconds>(         \
                                          _scoped_timer_end - _scoped_timer_start_##__COUNTER__)     \
                                          .count();                                                  \
        valhalla::midgard::logging::Log(std::string(VALHALLA_RELATIVE_FILE) +                        \
                                            "::" + std::string(__func__) + " took " +                \
                                            std::to_string(_scoped_timer_duration / 1000) + "s",     \
                                        " [TIMING] ");                                               \
      })

} // namespace mjolnir
} // namespace valhalla
