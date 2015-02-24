#include "baldr/directededge.h"
#include <boost/functional/hash.hpp>

namespace valhalla {
namespace baldr {

// Default constructor
DirectedEdge::DirectedEdge()
    : dataoffsets_{},
      geoattributes_{},
      forwardaccess_{},
      reverseaccess_{},
      speed_(0),
      classification_{},
      attributes_{},
      transitions_{} {
}

// Gets the end node of this directed edge.
GraphId DirectedEdge::endnode() const {
  return endnode_;
}

// ------------------  Data offsets and flags for extended data -------------//

// Get the offset to the common edge information.
uint32_t DirectedEdge::edgeinfo_offset() const {
  return dataoffsets_.edgeinfo_offset;
}

// Does this directed edge have general access conditions?
bool DirectedEdge::access_conditions() const {
  return dataoffsets_.access_conditions;
}

// Does this edge start a simple, timed turn restriction (from one
// edge to another).
bool DirectedEdge::start_ttr() const {
  return dataoffsets_.start_ttr;
}

// Does this edge start a multi-edge turn restriction. These are restrictions
// from one edge to another via one or more edges. Can include times.
bool DirectedEdge::start_mer() const {
  return dataoffsets_.start_mer;
}

// Does this edge end a multi-edge turn restriction. These are restrictions
// from one edge to another via one or more edges. This is the end edge of
// such a restriction. Can include times.
bool DirectedEdge::end_mer() const {
  return dataoffsets_.end_mer;
}

// Does this directed edge have exit signs.
bool DirectedEdge::exitsign() const {
  return dataoffsets_.exitsign;
}

// ------------------------- Geographic attributes ------------------------- //

// Gets the length of the edge in meters.
uint32_t DirectedEdge::length() const {
  return geoattributes_.length;
}

// Gets the elevation factor (0-15).
uint32_t DirectedEdge::elevation() const {
  return geoattributes_.elevation;
}

// Get the road curvature factor. (0-15).
uint32_t DirectedEdge::curvature() const {
  return geoattributes_.curvature;
}

// -------------------------- Routing attributes --------------------------- //

// Is driving on the right hand side of the road along this edge?
bool DirectedEdge::drive_on_right() const {
  return attributes_.drive_on_right;
}

// Is this edge part of a ferry?
bool DirectedEdge::ferry() const {
  return attributes_.ferry;
}

// Is this edge part of a rail ferry?
bool DirectedEdge::railferry() const {
  return attributes_.railferry;
}

// Does this edge have a toll or is it part of a toll road?
bool DirectedEdge::toll() const {
  return attributes_.toll;
}

// Does this edge have a seasonal access (e.g., closed in the winter)?
bool DirectedEdge::seasonal() const {
  return attributes_.seasonal;
}

// Is this edge part of a private or no through road that allows access
// only if required to get to a destination?
bool DirectedEdge::destonly() const {
  return attributes_.dest_only;
}

// Is this edge part of a tunnel?
bool DirectedEdge::tunnel() const {
  return attributes_.tunnel;
}

// Is this edge part of a bridge?
bool DirectedEdge::bridge() const {
  return attributes_.bridge;
}

// Is this edge part of a roundabout?
bool DirectedEdge::roundabout() const {
  return attributes_.roundabout;
}

// Is this edge is unreachable by driving. This can happen if a driveable
// edge is surrounded by pedestrian only edges (e.g. in a city center) or
// is not properly connected to other edges.
bool DirectedEdge::unreachable() const {
  return attributes_.unreachable;
}

// A traffic signal occurs at the end of this edge.
bool DirectedEdge::traffic_signal() const {
  return attributes_.traffic_signal;
}

// Does this edge represent a transition up one level in the hierarchy.
bool DirectedEdge::trans_up() const {
  return attributes_.trans_up;
}

// Does this edge represent a transition down one level in the hierarchy.
bool DirectedEdge::trans_down() const {
  return attributes_.trans_down;
}

// Does this edge represent a shortcut between 2 nodes?
bool DirectedEdge::shortcut() const {
 return attributes_.shortcut;
}

// Is this edge superseded by a shortcut edge?
bool DirectedEdge::superseded() const {
  return attributes_.superseded;
}

// Is this directed edge stored forward in edgeinfo (true) or
// reverse (false).
bool DirectedEdge::forward() const {
  return attributes_.forward;
}

// Does this edge lead into a no-thru region
bool DirectedEdge::not_thru() const {
  return attributes_.not_thru;
}

// Get the index of the opposing directed edge at the end node of this
// directed edge.
uint32_t DirectedEdge::opp_index() const {
  return attributes_.opp_index;
}

// Get the cycle lane type along this edge.
CycleLane DirectedEdge::cyclelane() const {
  return static_cast<CycleLane>(attributes_.cycle_lane);
}

// Gets the bike network mask
uint32_t DirectedEdge::bikenetwork() const {
  return attributes_.bikenetwork;
}

// Gets the lane count
uint32_t DirectedEdge::lanecount() const {
  return attributes_.lanecount;
}

// Get the index of the directed edge on the local level of the graph
// hierarchy. This is used for turn restrictions so the edges can be
// identified on the different levels.
uint32_t DirectedEdge::localedgeidx() const {
  return attributes_.localedgeidx;
}

// Get the mask of simple turn restrictions from the end of this directed
// edge. These are turn restrictions from one edge to another that apply to
// all vehicles, at all times.
uint32_t DirectedEdge::restrictions() const {
  return attributes_.restrictions;
}

// Get the use type of this edge.
Use DirectedEdge::use() const {
  return static_cast<Use>(attributes_.use);
}

// Get the speed type (see graphconstants.h)
SpeedType DirectedEdge::speed_type() const {
  return static_cast<SpeedType>(attributes_.speed_type);
}

// Get the access modes in the forward direction (bit field).
uint8_t DirectedEdge::forwardaccess() const {
  return forwardaccess_.v;
}

// Get the access modes in the reverse direction (bit field).
uint8_t DirectedEdge::reverseaccess() const {
  return reverseaccess_.v;
}

// -------------------------------- Default speed -------------------------- //

// Gets the speed in KPH.
uint8_t DirectedEdge::speed() const {
  return speed_;
}

// ----------------------------- Classification ---------------------------- //
// Get the road classification.
RoadClass DirectedEdge::classification() const {
  return static_cast<RoadClass>(classification_.classification);
}

// Is the edge considered unpaved?
bool DirectedEdge::unpaved() const {
  return (surface() >= Surface::kCompacted);
}

// Get the smoothness of this edge.
Surface DirectedEdge::surface() const {
  return static_cast<Surface>(classification_.surface);
}

// Is this edge a link / ramp?
bool DirectedEdge::link() const {
  return classification_.link;
}

// Gets the intersection internal flag.
bool DirectedEdge::internal() const {
  return classification_.internal;
}

// Get the internal version. Used for data validity checks.
const uint64_t DirectedEdge::internal_version() {
  uint64_t seed = 0;

  DirectedEdge de;
  de.dataoffsets_ = {};
  de.geoattributes_ = {};
  de.forwardaccess_ = {};
  de.reverseaccess_ = {};
  de.speed_ = 0;
  de.attributes_ = {};
  de.classification_ = {};

  // DataOffsets
  de.dataoffsets_.edgeinfo_offset = ~de.dataoffsets_.edgeinfo_offset;
  boost::hash_combine(seed, ffs(de.dataoffsets_.edgeinfo_offset+1)-1);
  de.dataoffsets_.access_conditions = ~de.dataoffsets_.access_conditions;
  boost::hash_combine(seed, ffs(de.dataoffsets_.access_conditions+1)-1);
  de.dataoffsets_.start_ttr = ~de.dataoffsets_.start_ttr;
  boost::hash_combine(seed, ffs(de.dataoffsets_.start_ttr+1)-1);
  de.dataoffsets_.start_mer = ~de.dataoffsets_.start_mer;
  boost::hash_combine(seed, ffs(de.dataoffsets_.start_mer+1)-1);
  de.dataoffsets_.end_mer = ~de.dataoffsets_.end_mer;
  boost::hash_combine(seed, ffs(de.dataoffsets_.end_mer+1)-1);
  de.dataoffsets_.exitsign = ~de.dataoffsets_.exitsign;
  boost::hash_combine(seed, ffs(de.dataoffsets_.exitsign+1)-1);
  de.dataoffsets_.spare = ~de.dataoffsets_.spare;
  boost::hash_combine(seed, ffs(de.dataoffsets_.spare+1)-1);

  // GeoAttributes
  de.geoattributes_.length = ~de.geoattributes_.length;
  boost::hash_combine(seed,ffs(de.geoattributes_.length+1)-1);
  de.geoattributes_.elevation = ~de.geoattributes_.elevation;
  boost::hash_combine(seed,ffs(de.geoattributes_.elevation+1)-1);
  de.geoattributes_.curvature = ~de.geoattributes_.curvature;
  boost::hash_combine(seed,ffs(de.geoattributes_.curvature+1)-1);

  // Attributes
  de.attributes_.drive_on_right = ~de.attributes_.drive_on_right;
  boost::hash_combine(seed,ffs(de.attributes_.drive_on_right+1)-1);
  de.attributes_.ferry = ~de.attributes_.ferry;
  boost::hash_combine(seed,ffs(de.attributes_.ferry+1)-1);
  de.attributes_.railferry = ~de.attributes_.railferry;
  boost::hash_combine(seed,ffs(de.attributes_.railferry+1)-1);
  de.attributes_.toll = ~de.attributes_.toll;
  boost::hash_combine(seed,ffs(de.attributes_.toll+1)-1);
  de.attributes_.seasonal = ~de.attributes_.seasonal;
  boost::hash_combine(seed,ffs(de.attributes_.seasonal+1)-1);
  de.attributes_.dest_only = ~de.attributes_.dest_only;
  boost::hash_combine(seed,ffs(de.attributes_.dest_only+1)-1);
  de.attributes_.tunnel = ~de.attributes_.tunnel;
  boost::hash_combine(seed,ffs(de.attributes_.tunnel+1)-1);
  de.attributes_.bridge = ~de.attributes_.bridge;
  boost::hash_combine(seed,ffs(de.attributes_.bridge+1)-1);
  de.attributes_.roundabout = ~de.attributes_.roundabout;
  boost::hash_combine(seed,ffs(de.attributes_.roundabout+1)-1);
  de.attributes_.unreachable = ~de.attributes_.unreachable;
  boost::hash_combine(seed,ffs(de.attributes_.unreachable+1)-1);
  de.attributes_.traffic_signal = ~de.attributes_.traffic_signal;
  boost::hash_combine(seed,ffs(de.attributes_.traffic_signal+1)-1);
  de.attributes_.trans_up = ~de.attributes_.trans_up;
  boost::hash_combine(seed,ffs(de.attributes_.trans_up+1)-1);
  de.attributes_.trans_down = ~de.attributes_.trans_down;
  boost::hash_combine(seed,ffs(de.attributes_.trans_down+1)-1);
  de.attributes_.shortcut = ~de.attributes_.shortcut;
  boost::hash_combine(seed,ffs(de.attributes_.shortcut+1)-1);
  de.attributes_.superseded = ~de.attributes_.superseded;
  boost::hash_combine(seed,ffs(de.attributes_.superseded+1)-1);
  de.attributes_.forward = ~de.attributes_.forward;
  boost::hash_combine(seed,ffs(de.attributes_.forward+1)-1);
  de.attributes_.not_thru = ~de.attributes_.not_thru;
  boost::hash_combine(seed,ffs(de.attributes_.not_thru+1)-1);
  de.attributes_.opp_index = ~de.attributes_.opp_index;
  boost::hash_combine(seed,ffs(de.attributes_.opp_index+1)-1);
  de.attributes_.cycle_lane = ~de.attributes_.cycle_lane;
  boost::hash_combine(seed,ffs(de.attributes_.cycle_lane+1)-1);
  de.attributes_.bikenetwork = ~de.attributes_.bikenetwork;
  boost::hash_combine(seed,ffs(de.attributes_.bikenetwork+1)-1);
  de.attributes_.localedgeidx = ~de.attributes_.localedgeidx;
  boost::hash_combine(seed,ffs(de.attributes_.localedgeidx+1)-1);
  de.attributes_.lanecount = ~de.attributes_.lanecount;
  boost::hash_combine(seed,ffs(de.attributes_.lanecount+1)-1);
  de.attributes_.restrictions = ~de.attributes_.restrictions;
  boost::hash_combine(seed,ffs(de.attributes_.restrictions+1)-1);
  de.attributes_.use = ~de.attributes_.use;
  boost::hash_combine(seed,ffs(de.attributes_.use+1)-1);

  // Access
  de.forwardaccess_.fields.car  = ~de.forwardaccess_.fields.car;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.car+1)-1);
  de.forwardaccess_.fields.pedestrian  = ~de.forwardaccess_.fields.pedestrian;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.pedestrian+1)-1);
  de.forwardaccess_.fields.bicycle  = ~de.forwardaccess_.fields.bicycle;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.bicycle+1)-1);
  de.forwardaccess_.fields.truck  = ~de.forwardaccess_.fields.truck;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.truck+1)-1);
  de.forwardaccess_.fields.emergency  = ~de.forwardaccess_.fields.emergency;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.emergency+1)-1);
  de.forwardaccess_.fields.taxi  = ~de.forwardaccess_.fields.taxi;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.taxi+1)-1);
  de.forwardaccess_.fields.horse  = ~de.forwardaccess_.fields.horse;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.horse+1)-1);
  de.forwardaccess_.fields.hov  = ~de.forwardaccess_.fields.hov;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.hov+1)-1);

  // Speed
  boost::hash_combine(seed,de.speed_);

  // Classification
  de.classification_.classification = ~de.classification_.classification;
  boost::hash_combine(seed,ffs(de.classification_.classification+1)-1);
  de.classification_.surface = ~de.classification_.surface;
  boost::hash_combine(seed,ffs(de.classification_.surface+1)-1);
  de.classification_.link = ~de.classification_.link;
  boost::hash_combine(seed,ffs(de.classification_.link+1)-1);
  de.classification_.internal = ~de.classification_.internal;
  boost::hash_combine(seed,ffs(de.classification_.internal+1)-1);

  // IntersectionTransition (TODO)

  boost::hash_combine(seed,sizeof(DirectedEdge));

  return seed;
}

}
}
