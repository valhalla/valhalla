#ifndef VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
#define VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

#include "geo/util.h"
#include "baldr/graphid.h"

namespace valhalla{
namespace mjolnir{

/**
 * Directed edge within the graph.
 * @author  David W. Nesbitt
 */
class DirectedEdgeBuilder : public baldr::DirectedEdge {
 public:
  /**
   * Constructor
   */
  DirectedEdgeBuilder();

  /**
   * Sets the length of the edge in kilometers.
   * @param  length  Length of the edge in kilometers.
   */
  void set_length(const float length);

  /**
   * Set the end node of this directed edge.
   * @param  endnode  End node of the directed link.
   */
  void set_endnode(const GraphId& endnode);

  // TODO - methods for access

  /**
   * Sets the speed in KPH.
   * @param  speed  Speed in KPH.
  */
  void set_speed(const float speed);

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
    unsigned char bicycle_      : 1;
    unsigned char emergency_    : 1;
    unsigned char horse_        : 1;  // ???
    unsigned char spare_        : 1;
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

#endif  // VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
