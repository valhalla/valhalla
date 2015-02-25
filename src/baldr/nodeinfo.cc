#include "baldr/nodeinfo.h"
#include <boost/functional/hash.hpp>

namespace valhalla {
namespace baldr {

// Default constructor
NodeInfo::NodeInfo()
    : latlng_{},
      attributes_{},
      intersection_(IntersectionType::kFalse),
      type_{},
      admin_{},
      stop_id_(0) {
}

// Get the latitude, longitude
const PointLL& NodeInfo::latlng() const {
  return static_cast<const PointLL&>(latlng_);
}

// Get the index in this tile of the first outbound directed edge
uint32_t NodeInfo::edge_index() const {
  return attributes_.edge_index_;
}

// Get the number of outbound edges from this node.
uint32_t NodeInfo::edge_count() const {
  return attributes_.edge_count_;
}

// Get the best road class of any outbound edges.
RoadClass NodeInfo::bestrc() const {
  return static_cast<RoadClass>(attributes_.bestrc_);
}

// Get the access modes (bit mask) allowed to pass through the node.
// See graphconstants.h
uint8_t NodeInfo::access() const {
  return access_.v;
}

// Get the intersection type.
IntersectionType NodeInfo::intersection() const {
  return intersection_;
}

// Get the index of the administrative information within this tile.
uint32_t NodeInfo::admin_index() const {
  return admin_.admin_index;
}

// Returns the timezone index.
uint32_t NodeInfo::timezone() const {
  return admin_.timezone;
}

// Is daylight saving time observed at the node's location.
bool NodeInfo::dst() const {
  return admin_.dst;
}

// Get the relative density at the node.
uint32_t NodeInfo::density() const {
  return type_.density;
}

// Gets the node type. See graphconstants.h for the list of types.
NodeType NodeInfo::type() const {
  return static_cast<NodeType>(type_.type);
}

// Get the number of driveable edges on the local level.
uint32_t NodeInfo::local_driveable() const {
  return type_.local_driveable;
}

// Is this a dead-end node that connects to only one edge?
bool NodeInfo::end() const {
  return type_.end;
}

// Is this a parent node (e.g. a parent transit stop).
bool NodeInfo::parent() const {
  return type_.parent;
}

// Is this a child node (e.g. a child transit stop).
bool NodeInfo::child() const {
  return type_.child;
}

// Is a mode change allowed at this node? The access data tells which
// modes are allowed at the node. Examples include transit stops, bike
// share locations, and parking locations.
bool NodeInfo::mode_change() const {
  return type_.mode_change;
}

// Is there a traffic signal at this node?
bool NodeInfo::traffic_signal() const {
  return type_.traffic_signal;
}

// Gets the transit stop Id. This is used for schedule lookups
// and possibly queries to a transit service.
uint32_t NodeInfo::stop_id() const {
  return stop_id_;
}

// Get the hash_value
const uint64_t NodeInfo::internal_version() {

  NodeInfo ni;

  ni.access_ = {};
  ni.admin_ = {};
  ni.attributes_ = {};
  ni.intersection_ = IntersectionType::kFalse;
  ni.stop_id_ = 0;
  ni.type_ = {};

  uint64_t seed = 0;
  boost::hash_combine(seed, ni.latlng_);

  //For bitfields, negate and find the index of the most significant bit to get the "size".  Finally, combine it to the seed.
  ni.attributes_.edge_index_ = ~ni.attributes_.edge_index_;
  boost::hash_combine(seed,ffs(ni.attributes_.edge_index_+1)-1);
  ni.attributes_.edge_count_ = ~ni.attributes_.edge_count_;
  boost::hash_combine(seed, ffs(ni.attributes_.edge_count_+1)-1);
  ni.attributes_.bestrc_ = ~ni.attributes_.bestrc_;
  boost::hash_combine(seed, ffs(ni.attributes_.bestrc_+1)-1);

  // Access
  ni.access_.fields.car  = ~ni.access_.fields.car;
  boost::hash_combine(seed,ffs(ni.access_.fields.car+1)-1);
  ni.access_.fields.pedestrian  = ~ni.access_.fields.pedestrian;
  boost::hash_combine(seed,ffs(ni.access_.fields.pedestrian+1)-1);
  ni.access_.fields.bicycle  = ~ni.access_.fields.bicycle;
  boost::hash_combine(seed,ffs(ni.access_.fields.bicycle+1)-1);
  ni.access_.fields.truck  = ~ni.access_.fields.truck;
  boost::hash_combine(seed,ffs(ni.access_.fields.truck+1)-1);
  ni.access_.fields.emergency  = ~ni.access_.fields.emergency;
  boost::hash_combine(seed,ffs(ni.access_.fields.emergency+1)-1);
  ni.access_.fields.taxi  = ~ni.access_.fields.taxi;
  boost::hash_combine(seed,ffs(ni.access_.fields.taxi+1)-1);
  ni.access_.fields.horse  = ~ni.access_.fields.horse;
  boost::hash_combine(seed,ffs(ni.access_.fields.horse+1)-1);
  ni.access_.fields.hov  = ~ni.access_.fields.hov;
  boost::hash_combine(seed,ffs(ni.access_.fields.hov+1)-1);

  boost::hash_combine(seed,ni.intersection_);

  ni.admin_.admin_index = ~ni.admin_.admin_index;
  boost::hash_combine(seed,ffs(ni.admin_.admin_index+1)-1);
  ni.admin_.timezone = ~ni.admin_.timezone;
  boost::hash_combine(seed, ffs(ni.admin_.timezone+1)-1);
  ni.admin_.dst = ~ni.admin_.dst;
  boost::hash_combine(seed, ffs(ni.admin_.dst+1)-1);
  ni.admin_.spare = ~ni.admin_.spare;
  boost::hash_combine(seed, ffs(ni.admin_.spare+1)-1);

  ni.type_.density = ~ni.type_.density;
  boost::hash_combine(seed,ffs(ni.type_.density+1)-1);
  ni.type_.type = ~ni.type_.type;
  boost::hash_combine(seed,ffs(ni.type_.type+1)-1);
  ni.type_.local_driveable = ~ni.type_.local_driveable;
  boost::hash_combine(seed,ffs(ni.type_.local_driveable+1)-1);
  ni.type_.end = ~ni.type_.end;
  boost::hash_combine(seed,ffs(ni.type_.end+1)-1);
  ni.type_.parent = ~ni.type_.parent;
  boost::hash_combine(seed,ffs(ni.type_.parent+1)-1);
  ni.type_.child = ~ni.type_.child;
  boost::hash_combine(seed,ffs(ni.type_.child+1)-1);
  ni.type_.mode_change = ~ni.type_.mode_change;
  boost::hash_combine(seed,ffs(ni.type_.mode_change+1)-1);
  ni.type_.traffic_signal = ~ni.type_.traffic_signal;
  boost::hash_combine(seed,ffs(ni.type_.traffic_signal+1)-1);
  ni.type_.spare = ~ni.type_.spare;
  boost::hash_combine(seed,ffs(ni.type_.spare+1)-1);

  boost::hash_combine(seed,ni.stop_id_);

  boost::hash_combine(seed,sizeof(NodeInfo));

  return seed;

}

}
}
