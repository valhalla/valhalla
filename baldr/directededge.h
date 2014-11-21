#ifndef __directededge_h_
#define __directededge_h_

#include "geosupport/geosupport.h"
#include "graphid.h"

/**
 * Directed edge within the graph.
 * @author  David W. Nesbitt
 */
class DirectedEdge {
 public:
  /**
   * Constructor
   */
  DirectedEdge() {
  }

  /**
   * Gets the length of the link in kilometers.
   * @return  Returns the length in kilometers.
   */
  float Length() const {
    return length_;
  }

  /**
   * Gets the end node of this directed edge.
   * @return  Returns the end node.
   */
  GraphId EndNode() const {
    return endnode_;
  }

  // TODO - methods for access

  /**
  * Gets the speed in KPH. TODO - cast to float instead?
  * @return  Returns the speed in KPH.
  */
  unsigned int Speed() const {
    return (unsigned int)speed_;
  }

 private:
  // Length of the link in miles
  float length_;

  // End node
  GraphId endnode_;

  // Legal access to the directed link ((also include reverse direction access).
  // TODO - come up with final set of values!
  struct Access {
    unsigned char car_          : 1;
    unsigned char taxi_         : 1;
    unsigned char truck_        : 1;
    unsigned char pedestrian_   : 1;
    unsigned char m_pedestrian_ : 1;
    unsigned char bicycle_      : 1;
    unsigned char emergency_    : 1;
    unsigned char horse_        : 1;
  };
  Access forwardaccess_;
  Access reverseaccess_;

  // Attributes. Can be used in edge costing methods to favor or avoid edges.
  struct Attributes {
    unsigned int ferry_          : 1;
    unsigned int railferry_      : 1;  // ???
    unsigned int toll_           : 1;
    unsigned int private_        : 1;
    unsigned int unpaved_        : 1;
    unsigned int tunnel_         : 1;
    unsigned int spare           : 30;
  };
  Attributes attributes_;

  // TODO - some representation of road class / highway tag importance

  // Speed in kilometers. Range 0-250 KPH. Applies to "normal vechicles".
  // Save values above 250 as special cases (closures, construction,
  // etc.)
  // TODO - do we need or does OSM support "truck speed"
  unsigned char speed_;

  // TODO - byte alignment / sizing

};

// TODO - do we derive a writeable class and treat the base class as read only?

#endif
