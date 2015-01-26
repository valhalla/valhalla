#include "baldr/nodeinfo.h"
#include <boost/functional/hash.hpp>

namespace valhalla {
namespace baldr {

// Default constructor
NodeInfo::NodeInfo() {
  latlng_.Set(0.0f, 0.0f);
  attributes_ = {0};
}

// Destructor.
NodeInfo::~NodeInfo() {
}

// Get the latitude, longitude
const PointLL& NodeInfo::latlng() const {
  return latlng_;
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

// Get the hash_value
const uint64_t NodeInfo::hash_value() {

  uint64_t seed = 0;
  boost::hash_combine(seed, latlng_);

  //For bitfields, negate and find the index of the most significant bit to get the "size".  Finally, combine it to the seed.
  attributes_.edge_index_ = ~attributes_.edge_index_;
  boost::hash_combine(seed,ffs(attributes_.edge_index_+1)-1);

  attributes_.edge_count_ = ~attributes_.edge_count_;
  boost::hash_combine(seed, ffs(attributes_.edge_count_+1)-1);

  attributes_.bestrc_ = ~attributes_.bestrc_;
  boost::hash_combine(seed, ffs(attributes_.bestrc_+1)-1);

  attributes_.spare_ = ~attributes_.spare_;
  boost::hash_combine(seed, ffs(attributes_.spare_+1)-1);

  boost::hash_combine(seed,sizeof(NodeInfo));

  return seed;

}

}
}
