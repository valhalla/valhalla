#ifndef VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_
#define VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/mjolnir/osmway.h>

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
   * Constructor with arguments.
   * @param  way      OSM way info generated from parsing OSM tags with Lua.
   * @param  endnode  GraphId of the end node of this directed edge.
   * @param  length   Length in meters.
   * @param  speed    Speed in kph.
   * @param  use      Use of the edge.
   * @param  not_thru Is the edge not_thru?
   * @param  internal Is the edge an intersection internal edge?
   * @param  rc       Road class / importance
   * @param  localidx  Index of the edge (from the node) on the local level
   * @param  restrictions Mask of simple turn restrictions at the end node
   *                      of this directed edge.
   */
  DirectedEdgeBuilder(const OSMWay& way, const baldr::GraphId& endnode,
                      const bool forward, const uint32_t length,
                      const float speed, const baldr::Use use,
                      const bool not_thru,  const bool internal,
                      const baldr::RoadClass rc, const uint32_t localidx,
                      const uint32_t restrictions);

  /**
   * Set the end node of this directed edge.
   * @param  endnode  End node of the directed link.
   */
  void set_endnode(const baldr::GraphId& endnode);

  /**
   * Set the offset to the common edge info. The offset is from the start
   * of the common edge info within a tile.
   * @param  offset  Offset from the start of the edge info within a tile.
   */
  void set_edgeinfo_offset(const uint32_t offset);

  /**
   * Sets the length of the edge in meters.
   * @param  length  Length of the edge in meters.
   */
  void set_length(const uint32_t length);

  /**
   * Sets the intersection internal flag.
   * @param  internal  true if the edge is internal to an intersection. This
   *          is derived from OSM and used for doubly digitized intersections
   */
  void set_internal(const bool internal);

  /**
   * Set all forward access modes to true (used for transition edges)
   */
  void set_all_forward_access();

  /**
   * Sets the car access of the edge in each direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  car      Is car access allowed?
   */
  void set_caraccess(const bool forward, const bool car);

  /**
   * Sets the taxi access of the edge in each direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  taxi     Is taxi access allowed?
   */
  void set_taxiaccess(const bool forward, const bool taxi);

  /**
   * Sets the truck access of the edge in each direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  truck    Is truck access allowed?
   */
  void set_truckaccess(const bool forward, const bool truck);

  /**
   * Sets the pedestrian access of the edge in each direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  pedestrian  Is pedestrian access allowed?
   */
  void set_pedestrianaccess(const bool forward, const bool pedestrian);

  /**
   * Sets the bicycle access of the edge in each direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  bicycle  Is bicycle access allowed?
   */
  void set_bicycleaccess(const bool forward, const bool bicycle);

  /**
   * Sets the emergency access of the edge in each direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  emergency  Is emergency access allowed?
   */
  void set_emergencyaccess(const bool forward, const bool emergency);

  /**
   * Sets the horse access of the edge in each direction.
   * @param  forward  Set access for forward direction if true, otherwise
   *                  setting access for reverse direction.
   * @param  horse    Is horse access allowed?
   */
  void set_horseaccess(const bool forward, const bool horse);

  /**
   * Sets the ferry flag.
   * @param  ferry    Is ferry?
   */
  void set_ferry(const bool ferry);

  /**
   * Sets the rail ferry flag.
   * @param  railferry    Is rail ferry?  Example: chunnel.
   */
  void set_railferry(const bool railferry);

  /**
   * Sets the toll flag.
   * @param  toll    Is toll?
   */
  void set_toll(const bool toll);

  /**
   * Sets the exit sign flag.
   * @param  exit  True if this directed edge has exit signs, false if not.
   */
  void set_exitsign(const bool exit);

  /**
   * Sets the destination only (private) flag.
   * @param  destonly     Is private (access to destination only)?
   */
  void set_dest_only(const bool destonly);

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
   * Sets the surface.
   * @param  surface   Smoothness.
   */
  void set_surface(const Surface surface);

  /**
    * Sets the cycle lane.
    * @param  cyclelane   Type of cycle lane.
    */
  void set_cyclelane(const CycleLane cyclelane);

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
   * Sets the road classification.
   * @param  roadclass  Road class.
   */
  void set_classification(const RoadClass roadclass);

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

  /**
   * Set the index of the directed edge on the local level of the graph
   * hierarchy. This is used for turn restrictions so the edges can be
   * identified on the different levels.
   * @param idx The index of the edge on the local level.
   */
  void set_localedgeidx(const uint32_t idx);

  /**
   * Set simple turn restrictions from the end of this directed edge.
   * These are turn restrictions from one edge to another that apply to
   * all vehicles, at all times.
   * @param  mask A bit mask that indicates the local edge indexes
   *          of outbound directed edges that are restricted.
   */
  void set_restrictions(const uint32_t mask);
};

}
}

#endif  // VALHALLA_MJOLNIR_DIRECTEDEDGEBUILDER_H_

