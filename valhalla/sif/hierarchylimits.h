#ifndef VALHALLA_SIF_HIERARCHYLIMITS_H_
#define VALHALLA_SIF_HIERARCHYLIMITS_H_

#include <valhalla/proto/options.pb.h>

#include <cstdint>
#include <limits>

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
constexpr float kDefaultExpansionWithinDistBidir[] = {kMaxDistance, 20000.0f, 5000.0f, 0.0f,
                                                      0.0f,         0.0f,     0.0f,    0.0f};
} // namespace

namespace valhalla {
namespace sif {

/**
 * Determine if expansion of a hierarchy level should be stopped once
 * the number of upward transitions has been exceeded. Allows expansion
 * within the specified distance from the destination regardless of
 * count.
 * @param hierarchy_limits Hierarchy limits.
 * @param dist             Distance (meters) from the destination.
 * @return                 Returns true if expansion at this hierarchy level should stop.
 */
inline bool StopExpanding(const valhalla::HierarchyLimits& hierarchy_limits, const float dist) {
  return (hierarchy_limits.up_transition_count() > hierarchy_limits.max_up_transitions() &&
          dist > hierarchy_limits.expand_within_dist());
}

/**
 * Determine if expansion of a hierarchy level should be stopped once
 * the number of upward transitions has been exceeded. This is used in
 * the bidirectional method where distance from the destination does not
 * matter.
 * @param hierarchy_limits Hierarchy limits.
 * @return                 Returns true if expansion at this hierarchy level should stop.
 */
inline bool StopExpanding(const valhalla::HierarchyLimits& hierarchy_limits) {
  return hierarchy_limits.up_transition_count() > hierarchy_limits.max_up_transitions();
}

/**
 * Relax hierarchy limits to try to find a route when initial attempt fails.
 * Do not relax limits if they are unlimited (bicycle and pedestrian for
 * example).
 */
inline void RelaxHierarchyLimits(valhalla::HierarchyLimits& hierarchy_limits,
                                 const float factor,
                                 const float expansion_within_factor) {

  if (hierarchy_limits.max_up_transitions() != kUnlimitedTransitions) {
    hierarchy_limits.set_max_up_transitions(hierarchy_limits.max_up_transitions() * factor);
    hierarchy_limits.set_expand_within_dist(hierarchy_limits.expand_within_dist() *
                                            expansion_within_factor);
  }
}
} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_HIERARCHYLIMITS_H_
