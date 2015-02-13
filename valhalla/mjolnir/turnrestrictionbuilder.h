#ifndef VALHALLA_MJOLNIR_TURNRESTRICTIONBUILDER_H_
#define VALHALLA_MJOLNIR_TURNRESTRICTIONBUILDER_H_

#include <valhalla/baldr/turnrestriction.h>

namespace valhalla {
namespace mjolnir {

/**
 * Generic sign builder. Encapsulates the sign type and the associated
 * text index. Includes the directed edge index - signs are accessed via
 * the directed edge to which they apply. Makes base class writable.
 */
class TurnRestrictionBuilder : public baldr::TurnRestriction {
 public:
  /**
   * Constructor with arguments.
   * @param  idx  Index of the directed edge the turn restriction is from.
   * @param  type Type of turn restriction.
   * @param  restriction_mask  bit mask where bits set to 1 indicate
   *            a restricted turn based on index of the outbound directed
   *            edge at the end node of this directed edge.
   */
  TurnRestrictionBuilder(const uint32_t idx, const baldr::RestrictionType type,
                  const uint32_t restriction_mask);

  /**
   * Set the directed edge index.
   * @param  idx  Directed edge index.
   */
  void set_edgeindex(const uint32_t idx);
};

}
}

#endif  // VALHALLA_MJOLNIR_TURNRESTRICTIONBUILDER_H_
