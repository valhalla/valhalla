#include "baldr/directededge.h"

namespace valhalla{
namespace baldr{

  DirectedEdge::DirectedEdge():speed_(0), length_(0) {
  }

  /**
   * Gets the length of the link in kilometers.
   * @return  Returns the length in kilometers.
   */
  float DirectedEdge::Length() const {
    return length_;
  }

  /**
   * Gets the end node of this directed edge.
   * @return  Returns the end node.
   */
  GraphId DirectedEdge::EndNode() const {
    return endnode_;
  }

  // TODO - methods for access

  /**
  * Gets the speed in KPH. TODO - cast to float instead?
  * @return  Returns the speed in KPH.
  */
  unsigned int DirectedEdge::Speed() const {
    return static_cast<unsigned int>(speed_);
  }


}
}
