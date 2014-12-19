#ifndef VALHALLA_THOR_TRIPPATHBUILDER_H_
#define VALHALLA_THOR_TRIPPATHBUILDER_H_

#include <vector>
#include <map>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include "thor/edgecost.h"

namespace valhalla {
namespace thor {

/**
 * Algorithm to create a trip path output from a list of directed edges.
 */
class TripPathBuilder {
 public:
  /**
   * Constructor.
   */
  TripPathBuilder();

  /**
   * Destructor
   */
  virtual ~TripPathBuilder();


  /**
   * Format the trip path output given the edges on the path.
   * For now just return length. TODO - modify to return trip path.
   */
  float Build(baldr::GraphReader& graphreader,
         const std::vector<baldr::GraphId>& pathedges);
};

}
}

#endif  // VALHALLA_THOR_TRIPPATHBUILDER_H_
