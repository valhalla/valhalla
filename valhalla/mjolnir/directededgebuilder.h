#ifndef VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
#define VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace mjolnir {

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
   * Set the end node of this directed edge.
   * @param  endnode  End node of the directed link.
   */
  void set_endnode(const baldr::GraphId& endnode);

  /**
   * Set the offset to the common edge data. The offset is from the start
   * of the common edge data within a tile.
   * @param  offset  Offset from the start of the edge data within a tile.
   */
  void set_edgedataoffset(const uint32_t offset);

  /**
   * Sets the length of the edge in meters.
   * @param  length  Length of the edge in meters.
   */
  void set_length(const uint32_t length);

  /**
   * Sets the car access of the edge in each direction.
   * @param  forward  Set the car access for the forward direction.
   * @param  reverse  Set the car access for the reverse direction.
   * @param  car      Is car access allowed?
   */
  void set_caraccess(const bool forward, const bool reverse, const bool car);

  /**
   * Sets the taxi access of the edge in each direction.
   * @param  forward  Set the taxi access for the forward direction.
   * @param  reverse  Set the taxi access for the reverse direction.
   * @param  taxi     Is taxi access allowed?
   */
  void set_taxiaccess(const bool forward, const bool reverse, const bool taxi);

  /**
   * Sets the truck access of the edge in each direction.
   * @param  forward  Set the truck access for the forward direction.
   * @param  reverse  Set the truck access for the reverse direction.
   * @param  truck    Is truck access allowed?
   */
  void set_truckaccess(const bool forward, const bool reverse, const bool truck);

  /**
   * Sets the pedestrian access of the edge in each direction.
   * @param  forward     Set the pedestrian access for the forward direction.
   * @param  reverse     Set the pedestrian access for the reverse direction.
   * @param  pedestrian  Is pedestrian access allowed?
   */
  void set_pedestrianaccess(const bool forward, const bool reverse, const bool pedestrian);

  /**
   * Sets the bicycle access of the edge in each direction.
   * @param  forward  Set the bicycle access for the forward direction.
   * @param  reverse  Set the bicycle access for the reverse direction.
   * @param  bicycle  Is bicycle access allowed?
   */
  void set_bicycleaccess(const bool forward, const bool reverse, const bool bicycle);

  /**
   * Sets the emergency access of the edge in each direction.
   * @param  forward    Set the emergency access for the forward direction.
   * @param  reverse    Set the emergency access for the reverse direction.
   * @param  emergency  Is emergency access allowed?
   */
  void set_emergencyaccess(const bool forward, const bool reverse, const bool emergency);

  /**
   * Sets the horse access of the edge in each direction.
   * @param  forward  Set the horse access for the forward direction.
   * @param  reverse  Set the horse access for the reverse direction.
   * @param  horse    Is horse access allowed?
   */
  void set_horseaccess(const bool forward, const bool reverse, const bool horse);

  /**
   * Sets the ferry flag.
   * @param  ferry    Is ferry?
   */
  void set_ferry(const bool ferry);

  /**
   * Sets the rail ferry flag.
   * @param  ferry    Is rail ferry?  Example: chunnel.
   */
  void set_railferry(const bool railferry);

  /**
   * Sets the toll flag.
   * @param  ferry    Is toll?
   */
  void set_toll(const bool toll);

  /**
   * Sets the destination only (private) flag.
   * @param  destonly     Is private (access to destination only)?
   */
  void set_dest_only(const bool destonly);

  /**
   * Sets the unpaved flag.
   * @param  unpaved  Is unpaved?
   */
  void set_unpaved(const bool unpaved);

  /**
   * Sets the tunnel flag.
   * @param  tunnel   Is tunnel?
   */
  void set_tunnel(const bool tunnel);

  /**
   * Sets the bridge flag.
   * @param  bridge   Is bridge?
   */
  void set_bridge(const bool bridge);

  /**
   * Sets the roundabout flag.
   * @param  roundabout   Is roundabout?
   */
  void set_roundabout(const bool roundabout);

  /**
   * Set the flag for whether this edge represents a transition up one level
   * in the hierarchy. Transition edges move between nodes in different levels
   * of the hierarchy but have no length or other attribution. An upward
   * transition is a transition from a minor road hierarchy (local) to more
   * major (arterial).
   * @param  trans_up  True if the edge is a transition from a lower level
   *          to a higher (false if not).
   */
  void set_trans_up(const bool trans_up);

   /**
    * Set the flag for whether this edge represents a transition down one level
    * in the hierarchy. Transition edges move between nodes in different levels
    * of the hierarchy but have no length or other attribution. A downward
    * transition is a transition from a major road hierarchy (highway) to more
    * minor (arterial).
    * @param   trans_down  True if the edge is a transition from an upper level
    *          to a lower (false if not).
    */
   void set_trans_down(const bool trans_down);

  /**
   * Set the flag for whether this edge represents a shortcut between 2 nodes.
   * Shortcuts bypass nodes that only connect to lower levels in the hierarchy
   * (other than the 1-2 higher level edges that superseded by the shortcut).
   * @param  shortcut  True if this edge is a shortcut edge, false if not.
   */
  void set_shortcut(const bool shortcut);

  /**
   * Set the flag for whether this edge is superseded by a shortcut edge.
   * Superseded edges can be skipped unless downward transitions are allowed.
   * @param  superseded True if this edge is part of a shortcut edge, false
  *          if not.
  */
  void set_superseded(const bool superseded);

  /**
   * Set the forward flag. Tells if this directed edge is stored forward
   * in edgeinfo (true) or reverse (false).
   * @param  forward  Forward flag.
   * */
  void set_forward(const bool forward);

  /**
   * Set the not_thru flag. If an edge leads to a "no thru" region where
   * there are no exits other than the incoming edge. This flag is populated
   * by processing the graph toidentify such edges.
   * @param  not_thru True if the edge leads into a no thru region.
   */
  void set_not_thru(const bool not_thru);

  /**
   * Set the index of the opposing directed edge at the end node of this
   * directed edge.
   * @param  opp_index  Opposing directed edge index at the end node.
   * */
  void set_opp_index(const uint32_t opp_index);

  /**
   * Sets the number of lanes
   * @param  lanecount  Number of lanes
  */
  void set_lanecount(const uint32_t lanecount);

  /**
   * Sets the bike network mask
   * @param  bikenetwork Bike network mask.
  */
  void set_bikenetwork(const uint32_t bikenetwork);

  /**
   * Sets the road class.
   * @param  roadclass  Road class.
   */
  void set_importance(const RoadClass roadclass);

  /**
   * Sets the link tag.
   * @param  link       Link.  Ramp or turn channel.
   */
  void set_link(const uint8_t link);

  /**
   * Sets the use.
   * @param  use        Use.  Something like "form of way."
   */
  void set_use(const Use use);

  /**
   * Sets the speed in KPH.
   * @param  speed  Speed in KPH.
  */
  void set_speed(const float speed);
};

}
}

#endif  // VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

