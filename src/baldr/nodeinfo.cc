#include "baldr/nodeinfo.h"
#include <boost/functional/hash.hpp>

namespace valhalla {
namespace baldr {

// Default constructor
NodeInfo::NodeInfo()
    : latlng_{},
      attributes_{},
      intersection_(0),
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

// TODO - intersection type?

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

// Get the relative density (TODO - define) at the node.
uint32_t NodeInfo::density() const {
  return type_.density;
}

// Gets the node type. See graphconstants.h for the list of types.
NodeType NodeInfo::type() const {
  return static_cast<NodeType>(type_.type);
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

// TODO - mode changes?

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

  uint64_t seed = 0;
  boost::hash_combine(seed, ni.latlng_);

  //For bitfields, negate and find the index of the most significant bit to get the "size".  Finally, combine it to the seed.
  ni.attributes_.edge_index_ = ~ni.attributes_.edge_index_;
  boost::hash_combine(seed,ffs(ni.attributes_.edge_index_+1)-1);

  ni.attributes_.edge_count_ = ~ni.attributes_.edge_count_;
  boost::hash_combine(seed, ffs(ni.attributes_.edge_count_+1)-1);

  ni.attributes_.bestrc_ = ~ni.attributes_.bestrc_;
  boost::hash_combine(seed, ffs(ni.attributes_.bestrc_+1)-1);

  boost::hash_combine(seed,sizeof(NodeInfo));

  return seed;

}

}
}
