#ifndef VALHALLA_SIF_HIERARCHYLIMITS_H_
#define VALHALLA_SIF_HIERARCHYLIMITS_H_

#include <cstdint>
#include <limits>
#include <valhalla/proto/options.pb.h>

// Default hierarchy transitions. Note that this corresponds to a 3 level
// strategy: highway, arterial, local. Any changes to this will require
// updates to the defaults.
namespace {

constexpr uint32_t kUnlimitedTransitions = std::numeric_limits<uint32_t>::max();
constexpr float kMaxDistance = std::numeric_limits<float>::max();

// Default default maximum upward transitions (per level). These are optimized
// for bidirectional to allow enough expansion on local and arterial to account
// for routes where more direct paths are available near the origin and
// destination.
constexpr uint32_t kDefaultMaxUpTransitions[] = {0, 400, 100, 0, 0, 0, 0, 0};

// Default distances within which expansion is always allowed (per level). It's optimized
// for unidirectional search and can be modified by the path algorithm in case of
// bidirectional search.
constexpr float kDefaultExpansionWithinDist[] = {kMaxDistance, 100000.0f, 5000.0f, 0.0f,
                                                 0.0f,         0.0f,      0.0f,    0.0f};

// Default number of transitions allowed. Requests with higher values will be clamped to this default
// if no other value is specified in the config.
constexpr unsigned int kDefaultMaxAllowedTransitions = 400;
} // namespace

namespace valhalla {
namespace sif {

bool StopExpanding(const valhalla::HierarchyLimits& hierarchy_limits, const float dist);
bool StopExpanding(const valhalla::HierarchyLimits& hierarchy_limits);
void RelaxHierarchyLimits(valhalla::HierarchyLimits& hierarchy_limits,
                          const float factor,
                          const float expansion_within_factor);

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_HIERARCHYLIMITS_H_
