#include "baldr/nodeinfo.h"
#include <boost/functional/hash.hpp>
#include <cmath>

namespace {
const uint32_t ContinuityLookup[] = {0, 7, 13, 18, 22, 25, 27};
}

namespace valhalla {
namespace baldr {

// Default constructor
NodeInfo::NodeInfo()
    : latlng_{},
      attributes_{},
      intersection_(IntersectionType::kRegular),
      type_{},
      admin_{},
      stop_{},
      headings_(0) {
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

// Get the driveability of the local directed edge given a local
// edge index.
Traversability NodeInfo::local_driveability(const uint32_t localidx) const {
  uint32_t s = localidx * 2;     // 2 bits per index
  return static_cast<Traversability>((type_.local_driveability & (3 << s)) >> s);
}

// Get the relative density at the node.
uint32_t NodeInfo::density() const {
  return type_.density;
}

// Gets the node type. See graphconstants.h for the list of types.
NodeType NodeInfo::type() const {
  return static_cast<NodeType>(type_.type);
}

// Checks if this node is a transit node.
bool NodeInfo::is_transit() const {
  NodeType nt = type();
  return (nt == NodeType::kRailStop || nt == NodeType::kBusStop ||
          nt == NodeType::kMultiUseTransitStop);
}

// Get the number of edges on the local level. We add 1 to allow up to
// up to kMaxLocalEdgeIndex + 1.
uint32_t NodeInfo::local_edge_count() const {
  return type_.local_edge_count + 1;
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
  return (is_transit()) ? stop_.stop_id : 0;
}

// Get the name consistency between a pair of local edges. This is limited
// to the first kMaxLocalEdgeIndex local edge indexes.
bool NodeInfo::name_consistency(const uint32_t from, const uint32_t to) const {
  if (from == to) {
    return true;
  } else if (from < to) {
    return (to > kMaxLocalEdgeIndex) ? false :
        (stop_.name_consistency & 1 << (ContinuityLookup[from] + (to-from-1)));
  } else {
    return (from > kMaxLocalEdgeIndex) ? false :
        (stop_.name_consistency & 1 << (ContinuityLookup[to] + (from-to-1)));
  }
}

// Get the heading of the local edge given its local index. Supports
// up to 8 local edges. Headings are expanded from 8 bits.
uint32_t NodeInfo::heading(const uint32_t localidx) const {
  // Make sure everything is 64 bit!
  uint64_t shift = localidx * 8;     // 8 bits per index
  return static_cast<uint32_t>(std::round(
      ((headings_ & (static_cast<uint64_t>(255) << shift)) >> shift)
          * kHeadingExpandFactor));
}

// Get the hash_value
const uint64_t NodeInfo::internal_version() {

  NodeInfo ni;

  ni.access_ = {};
  ni.admin_ = {};
  ni.attributes_ = {};
  ni.intersection_ = IntersectionType::kFalse;
  ni.stop_ = {};
  ni.type_ = {};

  size_t seed = 0;
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
  ni.access_.fields.bus  = ~ni.access_.fields.bus;
  boost::hash_combine(seed,ffs(ni.access_.fields.bus+1)-1);
  ni.access_.fields.hov  = ~ni.access_.fields.hov;
  boost::hash_combine(seed,ffs(ni.access_.fields.hov+1)-1);

  boost::hash_combine(seed,ni.intersection_);

  ni.admin_.admin_index = ~ni.admin_.admin_index;
  boost::hash_combine(seed,ffs(ni.admin_.admin_index+1)-1);
  ni.admin_.timezone = ~ni.admin_.timezone;
  boost::hash_combine(seed, ffs(ni.admin_.timezone+1)-1);
  ni.admin_.spare = ~ni.admin_.spare;
  boost::hash_combine(seed, ffs(ni.admin_.spare+1)-1);

  ni.type_.local_driveability = ~ni.type_.local_driveability;
  boost::hash_combine(seed,ffs(ni.type_.local_driveability+1)-1);
  ni.type_.density = ~ni.type_.density;
  boost::hash_combine(seed,ffs(ni.type_.density+1)-1);
  ni.type_.type = ~ni.type_.type;
  boost::hash_combine(seed,ffs(ni.type_.type+1)-1);
  ni.type_.local_edge_count = ~ni.type_.local_edge_count;
  boost::hash_combine(seed,ffs(ni.type_.local_edge_count+1)-1);
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

  boost::hash_combine(seed,ni.stop_.stop_id);

  boost::hash_combine(seed,ni.headings_);

  boost::hash_combine(seed,sizeof(NodeInfo));

  return seed;

}

}
}
