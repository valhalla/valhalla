#include "mjolnir/directededgeextbuilder.h"
#include "midgard/logging.h"

#include <algorithm>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

constexpr uint32_t kMinimumEdgeLength = 1;

// Constructor with parameters
DirectedEdgeExtBuilder::DirectedEdgeExtBuilder(const uint32_t morning_speed,
                                               const uint32_t general_speed,
                                               const uint32_t evening_speed)
    : DirectedEdgeExt() {

  set_morning_speed(morning_speed); // KPH
  set_general_speed(general_speed); // KPH
  set_evening_speed(evening_speed); // KPH
}

} // namespace mjolnir
} // namespace valhalla
