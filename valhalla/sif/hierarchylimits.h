#ifndef VALHALLA_SIF_HIERARCHYLIMITS_H_
#define VALHALLA_SIF_HIERARCHYLIMITS_H_

#include <boost/property_tree/ptree.hpp>
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

// Default distances within which expansion is always allowed
// (per level). Used only for A*.
constexpr float kDefaultExpansionWithinDist[] = {kMaxDistance, 100000.0f, 5000.0f, 0.0f,
                                                 0.0f,         0.0f,      0.0f,    0.0f};
} // namespace

namespace valhalla {
namespace sif {

/**
 * Hierarchy limits controls expansion and transitions between hierarchy
 * levels. It allows limiting expansion on hierarchy levels once
 * some number of "upward" transitions have been made (e.g. from the local
 * level to the arterial level). Expansion on a level is allowed within some
 * distance from the destination location. This also allows control of where
 * upward and downward transitions are allowed based on distance from the
 * destination.
 */
struct HierarchyLimits {
  uint32_t up_transition_count; // # of upward transitions from this level
  uint32_t max_up_transitions;  // Maximum number of upward transitions before
                                // expansion is stopped on a level.
  float expansion_within_dist;  // Distance (m) to destination within which
                                // expansion of a hierarchy level is
                                // always allowed. Used for A*.

  /**
   * Set hierarchy limits for the specified level using a property tree.
   * @param  pt     Property tree
   * @param  level  Hierarchy level
   */
  HierarchyLimits(const uint32_t level) : up_transition_count(0) {
    // Set maximum number of upward transitions
    max_up_transitions = kDefaultMaxUpTransitions[level];

    // Set distance within which expansion is always allowed for this level
    expansion_within_dist = kDefaultExpansionWithinDist[level];
  }

  /**
   * Determine if expansion of a hierarchy level should be stopped once
   * the number of upward transitions has been exceeded. Allows expansion
   * within the specified distance from the destination regardless of
   * count.
   * @param  dist  Distance (meters) from the destination.
   * @return  Returns true if expansion at this hierarchy level should stop.
   */
  bool StopExpanding(const float dist) const {
    return (dist > expansion_within_dist && up_transition_count > max_up_transitions);
  }

  /**
   * Determine if expansion of a hierarchy level should be stopped once
   * the number of upward transitions has been exceeded. This is used in
   * the bidirectional method where distance from the destination does not
   * matter.
   * @return  Returns true if expansion at this hierarchy level should stop.
   */
  bool StopExpanding() const {
    return up_transition_count > max_up_transitions;
  }

  /**
   * Relax hierarchy limits to try to find a route when initial attempt fails.
   * Do not relax limits if they are unlimited (bicycle and pedestrian for
   * example).
   */
  void Relax(const float factor, const float expansion_within_factor) {
    if (max_up_transitions != kUnlimitedTransitions) {
      max_up_transitions *= factor;
      expansion_within_dist *= expansion_within_factor;
    }
  }
};

} // namespace sif
} // namespace valhalla

#endif // VALHALLA_SIF_HIERARCHYLIMITS_H_
