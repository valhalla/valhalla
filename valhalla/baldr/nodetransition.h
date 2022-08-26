#ifndef VALHALLA_BALDR_NODETRANSITION_H_
#define VALHALLA_BALDR_NODETRANSITION_H_

#include <cstdint>
#include <stdint.h>
#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace baldr {

/**
 * Records a transition between a node on the current tile and a node
 * at the same position on a different hierarchy level. Stores the GraphId
 * of the end node as well as a flag indicating whether the transition is
 * upwards (true) or downwards (false).
 */
class NodeTransition {
public:
  /**
   * Default constructor.
   */
  NodeTransition() : endnode_(kInvalidGraphId), up_(0), spare_(0) {
  }

  /**
   * Constructor given arguments.
   * @param  node  End node of the transition.
   * @param  up    true if the transition is up to a higher level, false
   *               if the transition is down to a lower level.
   */
  NodeTransition(const GraphId& node, const bool up) : endnode_(node.value), up_(up), spare_(0) {
  }

  /**
   * Get the id of the corresponding node on another hierarchy level
   * @return  The corresponding node id on another level
   */
  GraphId endnode() const {
    return GraphId(endnode_);
  }

  /**
   * Is the transition up to a higher level.
   * @return  Returns true if the transition is up to a higher level, false
   *          if the transition is down to a lower level.
   */
  bool up() const {
    return up_;
  }

protected:
  uint64_t endnode_ : 46;
  uint64_t up_ : 1;
  uint64_t spare_ : 17;
};
} // namespace baldr
} // namespace valhalla

#endif // VALHALLA_BALDR_NODETRANSITION_H_
