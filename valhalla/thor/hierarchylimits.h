#ifndef VALHALLA_THOR_HIERARCHYLIMITS_H_
#define VALHALLA_THOR_HIERARCHYLIMITS_H_

namespace valhalla {
namespace thor {

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
   * Determine if expansion of a hierarchy level should be stopped once
   * the number of upward transitions has been exceeded. Allows expansion
   * within the specified distance from the destination regardless of
   * count.
   * @param  dist  Distance (meters) from the destination.
   */
  bool StopExpanding(const float dist) {
    return (dist > expansion_within_dist &&
            up_transition_count > max_up_transitions);
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
};

}
}

#endif  // VALHALLA_THOR_HIERARCHYLIMITS_H_
