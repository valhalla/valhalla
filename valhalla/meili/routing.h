// -*- mode: c++ -*-
#ifndef MMP_ROUTING_H_
#define MMP_ROUTING_H_
#include <cstdint>

#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>

namespace valhalla {
namespace meili {

constexpr uint16_t kInvalidDestination = std::numeric_limits<uint16_t>::max();

/**
 * Label used to mark edges within the map-matching routing algorithm.
 * Derived from EdgeLabel, this adds information required by map-matching
 * which uses a one to many routing algorithm.
 */
class Label : public sif::EdgeLabel {
public:
  Label() {
    // zero out the data but set the node Id and edge Id to invalid
    memset(this, 0, sizeof(Label));
    nodeid_ = baldr::GraphId();
    edgeid_ = baldr::GraphId();
    predecessor_ = baldr::kInvalidLabel;
  }

  /**
   * Construct a Label with all the required information.
   */
  Label(const baldr::GraphId& nodeid,
        uint16_t dest,
        const baldr::GraphId& edgeid,
        float source,
        float target,
        const sif::Cost& cost,
        float turn_cost,
        float sortcost,
        const uint32_t predecessor,
        const baldr::DirectedEdge* edge,
        const sif::TravelMode mode)
      : sif::EdgeLabel(predecessor, edgeid, edge, cost, sortcost, 0.0f, mode, 0), nodeid_(nodeid),
        dest_(dest), source_(source), target_(target), turn_cost_(turn_cost) {
    // Validate inputs
    if (!(0.f <= source && source <= target && target <= 1.f)) {
      throw std::invalid_argument("invalid source (" + std::to_string(source) + ") or target (" +
                                  std::to_string(target) + ")");
    }
    if (cost.cost < 0.f) {
      throw std::invalid_argument("invalid cost = " + std::to_string(cost.cost));
    }
    if (turn_cost < 0.f) {
      throw std::invalid_argument("invalid turn_cost = " + std::to_string(turn_cost));
    }
  }

  /**
   * Get the node Id.
   * @return  Returns the node Id.
   */
  baldr::GraphId nodeid() const {
    return nodeid_;
  }

  /**
   * Get the destination index.
   * @return Returns the destination index.
   */
  uint16_t dest() const {
    return dest_;
  }

  /**
   * Get the source distance.
   * @return  Returns the source distance (0-1).
   */
  float source() const {
    return source_;
  }

  /**
   * Get the target distance.
   * @return  Returns the target distance (0-1).
   */
  float target() const {
    return target_;
  }

  /**
   * Get the accumulated turn cost for this label.
   * @return  Returns the accumulated turn cost.
   */
  float turn_cost() const {
    return turn_cost_;
  }

  /**
   * Set all costs to 0. This is used when copying a prior Label to use as an
   * origin - we want to preserve Label values except costs must be set to 0.
   */
  void InitAsOrigin(const sif::TravelMode mode, const uint16_t dest, const baldr::GraphId& id) {
    source_ = 0.0f;
    target_ = 0.0f;
    turn_cost_ = 0.0f;
    sortcost_ = 0.0f;
    cost_ = {};
    predecessor_ = baldr::kInvalidLabel;
    mode_ = static_cast<uint32_t>(mode);
    dest_ = dest;
    nodeid_ = id;
  }

private:
  // Must be mutually exclusive, i.e. nodeid.Is_Valid() XOR dest != kInvalidDestination
  baldr::GraphId nodeid_;
  uint16_t dest_;

  // Assert: 0.f <= source <= target <= 1.f
  float source_;
  float target_;

  // Turn cost since origin (including the turn cost of this edge segment)
  float turn_cost_;
};

// Status information: label index and whether it is permanently labeled.
struct Status {
  Status() = delete;

  Status(uint32_t idx) : label_idx(idx), permanent(false) {
  }

  uint32_t label_idx : 31;
  uint32_t permanent : 1;
};

/**
 * LabelSet used during shortest path construction and recovery. Includes a
 * priority queue (sorted by sortdist) and maps that contain status (is the
 * element "permanently" labeled) of nodes and edges.
 */
class LabelSet {
public:
  LabelSet(const float max_cost, const float bucket_size = 1.0f);

  /**
   * Add an origin label using a destination index.
   */
  void put(const uint16_t dest, const sif::TravelMode mode, const Label* edgelabel) {
    // Do not add a duplicate label for the same destination index
    if (dest_status_.find(dest) == dest_status_.end()) {
      // If edgelabel is not null, append it to the label set otherwise append
      // a dummy. In both cases add the label to the priority queue, set its
      // predecessor to kInvalidLabel, and initialize costs to 0.
      const uint32_t idx = labels_.size();
      dest_status_.emplace(dest, idx);
      labels_.emplace_back(edgelabel ? *edgelabel : Label());
      labels_.back().InitAsOrigin(mode, dest, {});
      queue_->add(idx);
    }
  }

  /**
   * Add an origin label using a node id.
   */
  void put(const baldr::GraphId& nodeid, const sif::TravelMode mode, const Label* edgelabel) {
    // Do not add a duplicate origin label for the same node
    if (node_status_.find(nodeid) == node_status_.end()) {
      // If edgelabel is not null, append it to the label set otherwise append
      // a dummy. In both cases add the label to the priority queue and set its
      // predecessor to kInvalidLabel
      const uint32_t idx = labels_.size();
      node_status_.emplace(nodeid, idx);
      labels_.emplace_back(edgelabel ? *edgelabel : Label());
      labels_.back().InitAsOrigin(mode, kInvalidDestination, nodeid);
      queue_->add(idx);
    }
  }

  /**
   * Add a label with an edge and node Id.
   */
  void put(const baldr::GraphId& nodeid,
           const baldr::GraphId& edgeid,
           const float source,
           const float target,
           const sif::Cost& cost,
           const float turn_cost,
           const float sortcost,
           const uint32_t predecessor,
           const baldr::DirectedEdge* edge,
           const sif::TravelMode mode);

  /**
   * Add a label with an edge and a destination index.
   */
  void put(const uint16_t dest,
           const baldr::GraphId& edgeid,
           const float source,
           const float target,
           const sif::Cost& cost,
           const float turn_cost,
           const float sortcost,
           const uint32_t predecessor,
           const baldr::DirectedEdge* edge,
           const sif::TravelMode mode);

  /**
   * Get the next label from the priority queue. Marks the popped label
   * as permanent (best path found).
   * @return  Returns an index into the label set.
   */
  uint32_t pop();

  /**
   * Get a reference to a Label given its index.
   * @param label_idx  Label index.
   * @return  Returns a const reference to the label.
   */
  const Label& label(uint32_t label_idx) const {
    return labels_[label_idx];
  }

  /**
   * Clear the priority queue.
   */
  void clear_queue() {
    queue_->clear();
  }

  /**
   * Clear the status maps.
   */
  void clear_status() {
    node_status_.clear();
    dest_status_.clear();
  }

private:
  std::shared_ptr<baldr::DoubleBucketQueue> queue_;        // Priority queue
  std::unordered_map<baldr::GraphId, Status> node_status_; // Node status
  std::unordered_map<uint16_t, Status> dest_status_;       // Destination status
  std::vector<Label> labels_;                              // Label list.
};

using labelset_ptr_t = std::shared_ptr<LabelSet>;

/**
 * Find the shortest paths between an origin and a set of destinations.
 */
std::unordered_map<uint16_t, uint32_t>
find_shortest_path(baldr::GraphReader& reader,
                   const std::vector<baldr::PathLocation>& destinations,
                   uint16_t origin_idx,
                   labelset_ptr_t labelset,
                   const midgard::DistanceApproximator& approximator,
                   const float search_radius,
                   sif::cost_ptr_t costing,
                   const Label* edgelabel,
                   const float turn_cost_table[181],
                   const float max_dist,
                   const float max_time);

// Route path iterator. Methods to assist recovering route paths from Labels.
class RoutePathIterator : public std::iterator<std::forward_iterator_tag, const Label> {
public:
  // Construct a route path iterator.
  RoutePathIterator(const LabelSet* labelset, const uint32_t label_idx)
      : labelset_(labelset), label_idx_(label_idx) {
  }

  // Construct a tail iterator.
  RoutePathIterator(const LabelSet* labelset)
      : labelset_(labelset), label_idx_(baldr::kInvalidLabel) {
  }

  // Construct an invalid iterator.
  RoutePathIterator() : labelset_(nullptr), label_idx_(baldr::kInvalidLabel) {
  }

  // Postfix increment.
  RoutePathIterator operator++(int) {
    if (label_idx_ != baldr::kInvalidLabel) {
      auto clone = *this;
      label_idx_ = labelset_->label(label_idx_).predecessor();
      return clone;
    }
    return *this;
  }

  // Prefix increment.
  RoutePathIterator& operator++() {
    if (label_idx_ != baldr::kInvalidLabel) {
      label_idx_ = labelset_->label(label_idx_).predecessor();
    }
    return *this;
  }

  // Equality operator.
  bool operator==(const RoutePathIterator& other) const {
    return label_idx_ == other.label_idx_ && labelset_ == other.labelset_;
  }

  // Inequality operator.
  bool operator!=(const RoutePathIterator& other) const {
    return !(*this == other);
  }

  // Dereference
  reference operator*() const {
    return labelset_->label(label_idx_);
  }

  // Pointer dereference
  pointer operator->() const {
    return &(labelset_->label(label_idx_));
  }

private:
  const LabelSet* labelset_;
  uint32_t label_idx_;
};

} // namespace meili
} // namespace valhalla
#endif // MMP_ROUTING_H_
