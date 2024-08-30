#pragma once

#include <valhalla/baldr/timedomain.h>

namespace valhalla {
namespace baldr {

/** A combination of `TimeDomain` condition and a speed limit in a single 8b word */
union ConditionalSpeedLimit {
  TimeDomain td_;
  struct {
    uint64_t padding_ : 54; // padding over the TimeDomain meaningful bits
    uint64_t speed_ : 8;    // speed limit in KPH
    uint64_t spare_ : 2;
  };
};
static_assert(sizeof(ConditionalSpeedLimit) == 8, "invalid ConditionalSpeedLimit struct size");

} // namespace baldr
} // namespace valhalla
