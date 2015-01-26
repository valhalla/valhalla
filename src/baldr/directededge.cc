#include "baldr/directededge.h"
#include <boost/functional/hash.hpp>

namespace valhalla {
namespace baldr {

// Default constructor
DirectedEdge::DirectedEdge()
    : edgedataoffset_(0),
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

// Get the offset to the common edge data.
uint32_t DirectedEdge::edgedataoffset() const {
  return edgedataoffset_;
}

// Gets the length of the edge in meters.
uint32_t DirectedEdge::length() const {
  return geoattributes_.length;
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
  return (surface() <= Surface::kCompacted);
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

// Gets the lane count
uint32_t DirectedEdge::lanecount() const {
  return attributes_.lanecount;
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

// Get the hash_value
const uint64_t DirectedEdge::hash_value() {

  uint64_t seed = 0;

  boost::hash_combine(seed,edgedataoffset_);

  geoattributes_.length = ~geoattributes_.length;
  boost::hash_combine(seed,ffs(geoattributes_.length+1)-1);
  geoattributes_.elevation = ~geoattributes_.elevation;
  boost::hash_combine(seed,ffs(geoattributes_.elevation+1)-1);
  geoattributes_.spare = ~geoattributes_.spare;
  boost::hash_combine(seed,ffs(geoattributes_.spare+1)-1);

  forwardaccess_.fields.pedestrian  = ~forwardaccess_.fields.pedestrian;
  boost::hash_combine(seed,ffs(forwardaccess_.fields.pedestrian+1)-1);
  forwardaccess_.fields.bicycle  = ~forwardaccess_.fields.bicycle;
  boost::hash_combine(seed,ffs(forwardaccess_.fields.bicycle+1)-1);
  forwardaccess_.fields.car  = ~forwardaccess_.fields.car;
  boost::hash_combine(seed,ffs(forwardaccess_.fields.car+1)-1);
  forwardaccess_.fields.emergency  = ~forwardaccess_.fields.emergency;
  boost::hash_combine(seed,ffs(forwardaccess_.fields.emergency+1)-1);
  forwardaccess_.fields.horse  = ~forwardaccess_.fields.horse;
  boost::hash_combine(seed,ffs(forwardaccess_.fields.horse+1)-1);
  forwardaccess_.fields.spare_  = ~forwardaccess_.fields.spare_;
  boost::hash_combine(seed,ffs(forwardaccess_.fields.spare_+1)-1);
  forwardaccess_.fields.taxi  = ~forwardaccess_.fields.taxi;
  boost::hash_combine(seed,ffs(forwardaccess_.fields.taxi+1)-1);
  forwardaccess_.fields.truck  = ~forwardaccess_.fields.truck;
  boost::hash_combine(seed,ffs(forwardaccess_.fields.truck+1)-1);

  boost::hash_combine(seed,speed_);

  classification_.importance = ~classification_.importance;
  boost::hash_combine(seed,ffs(classification_.importance+1)-1);
  classification_.link = ~classification_.link;
  boost::hash_combine(seed,ffs(classification_.link+1)-1);
  classification_.use = ~classification_.use;
  boost::hash_combine(seed,ffs(classification_.use+1)-1);

  attributes_.bikenetwork = ~attributes_.bikenetwork;
  boost::hash_combine(seed,ffs(attributes_.bikenetwork+1)-1);
  attributes_.bridge = ~attributes_.bridge;
  boost::hash_combine(seed,ffs(attributes_.bridge+1)-1);
  attributes_.cycle_lane = ~attributes_.cycle_lane;
  boost::hash_combine(seed,ffs(attributes_.cycle_lane+1)-1);
  attributes_.dest_only = ~attributes_.dest_only;
  boost::hash_combine(seed,ffs(attributes_.dest_only+1)-1);
  attributes_.ferry = ~attributes_.ferry;
  boost::hash_combine(seed,ffs(attributes_.ferry+1)-1);
  attributes_.forward = ~attributes_.forward;
  boost::hash_combine(seed,ffs(attributes_.forward+1)-1);
  attributes_.lanecount = ~attributes_.lanecount;
  boost::hash_combine(seed,ffs(attributes_.lanecount+1)-1);
  attributes_.not_thru = ~attributes_.not_thru;
  boost::hash_combine(seed,ffs(attributes_.not_thru+1)-1);
  attributes_.opp_index = ~attributes_.opp_index;
  boost::hash_combine(seed,ffs(attributes_.opp_index+1)-1);
  attributes_.railferry = ~attributes_.railferry;
  boost::hash_combine(seed,ffs(attributes_.railferry+1)-1);
  attributes_.roundabout = ~attributes_.roundabout;
  boost::hash_combine(seed,ffs(attributes_.roundabout+1)-1);
  attributes_.shortcut = ~attributes_.shortcut;
  boost::hash_combine(seed,ffs(attributes_.shortcut+1)-1);
  attributes_.spare = ~attributes_.spare;
  boost::hash_combine(seed,ffs(attributes_.spare+1)-1);
  attributes_.superseded = ~attributes_.superseded;
  boost::hash_combine(seed,ffs(attributes_.superseded+1)-1);
  attributes_.surface = ~attributes_.surface;
  boost::hash_combine(seed,ffs(attributes_.surface+1)-1);
  attributes_.toll = ~attributes_.toll;
  boost::hash_combine(seed,ffs(attributes_.toll+1)-1);
  attributes_.trans_down = ~attributes_.trans_down;
  boost::hash_combine(seed,ffs(attributes_.trans_down+1)-1);
  attributes_.trans_up = ~attributes_.trans_up;
  boost::hash_combine(seed,ffs(attributes_.trans_up+1)-1);
  attributes_.tunnel = ~attributes_.tunnel;
  boost::hash_combine(seed,ffs(attributes_.tunnel+1)-1);

  boost::hash_combine(seed,sizeof(DirectedEdge));

  return seed;

}

}
}
