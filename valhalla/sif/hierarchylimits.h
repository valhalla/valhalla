#ifndef VALHALLA_SIF_HIERARCHYLIMITS_H_
#define VALHALLA_SIF_HIERARCHYLIMITS_H_

#include <limits>
#include <boost/property_tree/ptree.hpp>

// Default hierarchy transitions. Note that this corresponds to a 3 level
// strategy: highway, arterial, local. Any changes to this will require
// updates to the defaults.
namespace {
constexpr float kMaxDistance = std::numeric_limits<float>::max();

// Default default maximum upward transitions (per level)
constexpr uint32_t kDefaultMaxUpTransitions[] = {
    0, 250, 50, 0, 0, 0, 0, 0 };

// Default distances within which expansion is always allowed
// (per level)
constexpr float kDefaultExpansionWithinDist[] = {
    kMaxDistance, 100000.0f, 5000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

// Default distances outside which upward transitions are allowed
// (per level)
constexpr float kDefaultUpwardUntilDist[] = {
    0.0f, 10000.0f, 5000.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

// Default distances within which downward transitions are allowed
// (per level)
constexpr float kDefaultDownwardWithinDist[] = {
    10000.0f, 5000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
}

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
  uint32_t up_transition_count;  // # of upward transitions from this level
  uint32_t max_up_transitions;   // Maximum number of upward transitions before
                                 // expansion is stopped on a level.
  float expansion_within_dist;   // Distance (m) to destination within which
                                 // expansion of a hierarchy level is
                                 // always allowed.
  float upward_until_dist;       // Distance (m) to destination outside which
                                 // upward hierarchy transitions are allowed.
  float downward_within_dist;    // Distance (m) to destination within which
                                 // downward transitions are allowed.

  /**
   * Set hierarchy limits for the specified level using a property tree.
   * @param  pt     Property tree
   * @param  level  Hierarchy level
   */
  HierarchyLimits(const boost::property_tree::ptree& pt, const uint32_t level)
      : up_transition_count(0) {

    // Construct string to identify the level of the hierarchy
    std::string hl = "hierarchy_limits." + std::to_string(level);

    // Set maximum number of upward transitions
    max_up_transitions = pt.get<uint32_t>(hl + ".max_up_transitions",
                    kDefaultMaxUpTransitions[level]);

    // Set distance within which expansion is always allowed for this level
    expansion_within_dist = pt.get<float>(hl + ".expansion_within_dist",
                    kDefaultExpansionWithinDist[level]);

    // Set distance outside which upward transitions are allowed
    upward_until_dist = pt.get<float>(hl +".upward_until_dist",
                    kDefaultUpwardUntilDist[level]);

    // Set distance within which downward transitions are allowed
    downward_within_dist = pt.get<float>(hl + ".downward_within_dist",
                    kDefaultDownwardWithinDist[level]);
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
    return (dist > expansion_within_dist &&
            up_transition_count > max_up_transitions);
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
   * Determines if an upward transition should be allowed. Allows upward
   * transitions outside the upward_until_dist distance.
   * @param  dist  Distance (meters) from the destination.
   */
  bool AllowUpwardTransition(const float dist) const {
    return dist > upward_until_dist;
  }

  /**
   * Determines if an downward transition should be allowed. Downward
   * transitions are allowed only within downward_within_dist from the
   * destination.
   * @param  dist  Distance (meters) from the destination.
   */
  bool AllowDownwardTransition(const float dist) const {
    return dist < downward_within_dist;
  }

  void Relax(const float factor, const float expansion_within_factor) {
    up_transition_count *= factor;
    max_up_transitions *= factor;
    upward_until_dist *= factor;
    downward_within_dist *= factor;
    expansion_within_dist *= expansion_within_factor;
  }

  void DisableHighwayTransitions() {
    up_transition_count = 0;
    max_up_transitions = 0;
    expansion_within_dist = kMaxDistance;
    upward_until_dist = kMaxDistance;
  }
};

}
}

#endif  // VALHALLA_SIF_HIERARCHYLIMITS_H_
