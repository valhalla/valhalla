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
      attributes_{},
      classification_{}{
}

// Destructor.
DirectedEdge::~DirectedEdge() {
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

// Does this directed edge have exit signs.
bool DirectedEdge::exitsign() const {
  return dataoffsets_.exitsign;
}

// Gets the length of the edge in meters.
uint32_t DirectedEdge::length() const {
  return geoattributes_.length;
}

// Gets the elevation factor (0-15).
uint32_t DirectedEdge::elevation() const {
  return geoattributes_.elevation;
}

// Gets the lane count
uint32_t DirectedEdge::lanecount() const {
  return geoattributes_.lanecount;
}

// Get the access modes in the forward direction (bit field).
uint8_t DirectedEdge::forwardaccess() const {
  return forwardaccess_.v;
}

// Get the access modes in the reverse direction (bit field).
uint8_t DirectedEdge::reverseaccess() const {
  return reverseaccess_.v;
}

// Gets the speed in KPH.
uint8_t DirectedEdge::speed() const {
  return speed_;
}

// TODO - methods for individual access

// Get the road class / importance.
RoadClass DirectedEdge::importance() const {
  return static_cast<RoadClass>(classification_.importance);
}

// Is this edge a link / ramp?
bool DirectedEdge::link() const {
  return classification_.link;
}

// Get the use of this edge.
Use DirectedEdge::use() const {
  return static_cast<Use>(classification_.use);
}

bool DirectedEdge::ferry() const {
  return attributes_.ferry;
}

bool DirectedEdge::railferry() const {
  return attributes_.railferry;
}

bool DirectedEdge::toll() const {
  return attributes_.toll;
}

bool DirectedEdge::destonly() const {
  return attributes_.dest_only;
}

bool DirectedEdge::unpaved() const {
  return (surface() >= Surface::kCompacted);
}

bool DirectedEdge::tunnel() const {
  return attributes_.tunnel;
}

bool DirectedEdge::bridge() const {
  return attributes_.bridge;
}

bool DirectedEdge::roundabout() const {
  return attributes_.roundabout;
}

// Get the smoothness of this edge.
Surface DirectedEdge::surface() const {
  return static_cast<Surface>(attributes_.surface);
}

// Get the cycle lane of this edge.
CycleLane DirectedEdge::cyclelane() const {
  return static_cast<CycleLane>(attributes_.cycle_lane);
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

// Gets the bike network mask
uint32_t DirectedEdge::bikenetwork() const {
  return attributes_.bikenetwork;
}

// Gets the intersection internal flag.
bool DirectedEdge::internal() const {
  return attributes_.internal;
}

// Get the internal version
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
  de.dataoffsets_.spare = ~de.dataoffsets_.spare;
  boost::hash_combine(seed, ffs(de.dataoffsets_.spare+1)-1);
  de.dataoffsets_.exitsign = ~de.dataoffsets_.exitsign;
  boost::hash_combine(seed, ffs(de.dataoffsets_.exitsign+1)-1);

  // GeoAttributes
  de.geoattributes_.length = ~de.geoattributes_.length;
  boost::hash_combine(seed,ffs(de.geoattributes_.length+1)-1);
  de.geoattributes_.elevation = ~de.geoattributes_.elevation;
  boost::hash_combine(seed,ffs(de.geoattributes_.elevation+1)-1);
  de.geoattributes_.lanecount = ~de.geoattributes_.lanecount;
  boost::hash_combine(seed,ffs(de.geoattributes_.lanecount+1)-1);

  // Access
  de.forwardaccess_.fields.pedestrian  = ~de.forwardaccess_.fields.pedestrian;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.pedestrian+1)-1);
  de.forwardaccess_.fields.bicycle  = ~de.forwardaccess_.fields.bicycle;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.bicycle+1)-1);
  de.forwardaccess_.fields.car  = ~de.forwardaccess_.fields.car;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.car+1)-1);
  de.forwardaccess_.fields.emergency  = ~de.forwardaccess_.fields.emergency;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.emergency+1)-1);
  de.forwardaccess_.fields.horse  = ~de.forwardaccess_.fields.horse;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.horse+1)-1);
  de.forwardaccess_.fields.spare_  = ~de.forwardaccess_.fields.spare_;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.spare_+1)-1);
  de.forwardaccess_.fields.taxi  = ~de.forwardaccess_.fields.taxi;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.taxi+1)-1);
  de.forwardaccess_.fields.truck  = ~de.forwardaccess_.fields.truck;
  boost::hash_combine(seed,ffs(de.forwardaccess_.fields.truck+1)-1);

  // Speed
  boost::hash_combine(seed,de.speed_);

  // Classification
  de.classification_.importance = ~de.classification_.importance;
  boost::hash_combine(seed,ffs(de.classification_.importance+1)-1);
  de.classification_.link = ~de.classification_.link;
  boost::hash_combine(seed,ffs(de.classification_.link+1)-1);
  de.classification_.use = ~de.classification_.use;
  boost::hash_combine(seed,ffs(de.classification_.use+1)-1);

  // Attributes
  de.attributes_.bikenetwork = ~de.attributes_.bikenetwork;
  boost::hash_combine(seed,ffs(de.attributes_.bikenetwork+1)-1);
  de.attributes_.bridge = ~de.attributes_.bridge;
  boost::hash_combine(seed,ffs(de.attributes_.bridge+1)-1);
  de.attributes_.cycle_lane = ~de.attributes_.cycle_lane;
  boost::hash_combine(seed,ffs(de.attributes_.cycle_lane+1)-1);
  de.attributes_.dest_only = ~de.attributes_.dest_only;
  boost::hash_combine(seed,ffs(de.attributes_.dest_only+1)-1);
  de.attributes_.ferry = ~de.attributes_.ferry;
  boost::hash_combine(seed,ffs(de.attributes_.ferry+1)-1);
  de.attributes_.forward = ~de.attributes_.forward;
  boost::hash_combine(seed,ffs(de.attributes_.forward+1)-1);
  de.attributes_.not_thru = ~de.attributes_.not_thru;
  boost::hash_combine(seed,ffs(de.attributes_.not_thru+1)-1);
  de.attributes_.opp_index = ~de.attributes_.opp_index;
  boost::hash_combine(seed,ffs(de.attributes_.opp_index+1)-1);
  de.attributes_.railferry = ~de.attributes_.railferry;
  boost::hash_combine(seed,ffs(de.attributes_.railferry+1)-1);
  de.attributes_.roundabout = ~de.attributes_.roundabout;
  boost::hash_combine(seed,ffs(de.attributes_.roundabout+1)-1);
  de.attributes_.shortcut = ~de.attributes_.shortcut;
  boost::hash_combine(seed,ffs(de.attributes_.shortcut+1)-1);
  de.attributes_.superseded = ~de.attributes_.superseded;
  boost::hash_combine(seed,ffs(de.attributes_.superseded+1)-1);
  de.attributes_.surface = ~de.attributes_.surface;
  boost::hash_combine(seed,ffs(de.attributes_.surface+1)-1);
  de.attributes_.toll = ~de.attributes_.toll;
  boost::hash_combine(seed,ffs(de.attributes_.toll+1)-1);
  de.attributes_.trans_down = ~de.attributes_.trans_down;
  boost::hash_combine(seed,ffs(de.attributes_.trans_down+1)-1);
  de.attributes_.trans_up = ~de.attributes_.trans_up;
  boost::hash_combine(seed,ffs(de.attributes_.trans_up+1)-1);
  de.attributes_.tunnel = ~de.attributes_.tunnel;
  boost::hash_combine(seed,ffs(de.attributes_.tunnel+1)-1);
  de.attributes_.internal = ~de.attributes_.internal;
  boost::hash_combine(seed,ffs(de.attributes_.internal+1)-1);

  boost::hash_combine(seed,sizeof(DirectedEdge));

  return seed;

}

}
}
