// -*- mode: c++ -*-
#ifndef MMP_ROUTING_H_
#define MMP_ROUTING_H_
#include <cstdint>

#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <algorithm>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/dynamiccost.h>

namespace valhalla{
namespace meili {

constexpr uint16_t kInvalidDestination = std::numeric_limits<uint16_t>::max();

/**
 * Labels used to mark edges in the path. Includes predecessor and cost
 * information needed to construct and recover shortest paths.
 * TODO simplify by inheriting from sif::EdgeLabel
 */
struct Label
{
  Label() = delete;

  Label(const baldr::GraphId& the_nodeid, const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        const sif::Cost& the_cost, float the_turn_cost, float the_sortcost,
        uint32_t the_predecessor, const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : Label(the_nodeid, kInvalidDestination, the_edgeid,
              the_source, the_target, the_cost, the_turn_cost, the_sortcost,
              the_predecessor, the_edge, the_travelmode, the_edgelabel) {}

  /**
   * Construct a Label without a node Id.
   */
  Label(uint16_t the_dest, const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        const sif::Cost& the_cost, float the_turn_cost, float the_sortcost,
        uint32_t the_predecessor, const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : Label({}, the_dest, the_edgeid, the_source, the_target,
              the_cost, the_turn_cost, the_sortcost,
              the_predecessor, the_edge, the_travelmode, the_edgelabel) {}

  /**
   * Construct a Label with all the required information.
   */
  Label(const baldr::GraphId& the_nodeid, uint16_t the_dest,
        const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        const sif::Cost& the_cost, float the_turn_cost, float the_sortcost,
        uint32_t the_predecessor,
        const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : nodeid(the_nodeid), dest(the_dest), edgeid(the_edgeid),
        source(the_source), target(the_target),
        cost(the_cost), turn_cost(the_turn_cost), sortcost(the_sortcost),
        predecessor(the_predecessor),
        edgelabel(the_edgelabel) {
    if (!((nodeid.Is_Valid() && dest == kInvalidDestination) || (!nodeid.Is_Valid() && dest != kInvalidDestination))) {
      throw std::invalid_argument("nodeid and dest must be mutually exclusive, i.e. either nodeid is valid or dest is valid");
    }

    if (!(0.f <= source && source <= target && target <= 1.f)) {
      throw std::invalid_argument("invalid source ("
                                  + std::to_string(source)
                                  + ") or target ("
                                  + std::to_string(target) + ")");
    }

    if (cost.cost < 0.f) {
      throw std::invalid_argument("invalid cost = " + std::to_string(cost.cost));
    }

    if (turn_cost < 0.f) {
      throw std::invalid_argument("invalid turn_cost = " + std::to_string(turn_cost));
    }

    if (!edgelabel && the_edge) {
      // Populate the sif EdgeLabel. The cost within Cost is the distance.
      edgelabel = std::make_shared<const sif::EdgeLabel>(the_predecessor,
           the_edgeid, the_edge, the_cost, sortcost, 0.0f, the_travelmode, 0);
    }
  }

  // Must be mutually exclusive, i.e. nodeid.Is_Valid() XOR dest != kInvalidDestination
  baldr::GraphId nodeid;
  uint16_t dest;

  // Invalid graphid if dummy
  baldr::GraphId edgeid;

  // Assert: 0.f <= source <= target <= 1.f
  float source;
  float target;

  // Cost since origin (including the cost of this edge segment). cost is the
  // accumulate distance and secs is the elapsed time along the path
  sif::Cost cost;

  // Turn cost since origin (including the turn cost of this edge segment)
  float turn_cost;

  // For ranking labels: sortcost = accumulated cost since origin + heuristic cost to the goal
  float sortcost;

  // baldr::kInvalidLabel if dummy
  uint32_t predecessor;

  // An EdgeLabel is needed here for passing to sif's filters later
  std::shared_ptr<const sif::EdgeLabel> edgelabel;
};


// Status information: label index and whether it is permanently labeled.
struct Status{
  Status() = delete;

  Status(uint32_t idx)
      : label_idx(idx),
        permanent(false) {}

  uint32_t label_idx : 31;

  uint32_t permanent : 1;
};

/**
 * LabelSet used during shortest path construction and recovery. Includes a
 * priority queue (sorted by sortdist) and maps that contain status (is the
 * element "permanently" labeled) of nodes and edges.
 */
class LabelSet
{
 public:
  LabelSet(const float max_cost, const float bucket_size = 1.0f);

  bool put(const baldr::GraphId& nodeid, sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel) {
    return put(nodeid, {},                 // nodeid, (dummy) edgeid
               0.f, 0.f,                   // source, target
               sif::Cost(), 0.f, 0.f,      // cost, turn cost, sort cost
               baldr::kInvalidLabel,       // predecessor
               nullptr, travelmode, edgelabel);
  }


  bool put(const baldr::GraphId& nodeid,
           const baldr::GraphId& edgeid,
           float source, float target,
           const sif::Cost& cost, float turn_cost, float sortcost,
           uint32_t predecessor,
           const baldr::DirectedEdge* edge,
           sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel);

  bool put(uint16_t dest, sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel) {
    return put(dest, {},                  // dest, (dummy) edgeid
               0.f, 0.f,                  // source, target
               sif::Cost(), 0.f, 0.f,     // cost, turn cost, sort cost
               baldr::kInvalidLabel,      // predecessor
               nullptr, travelmode, edgelabel);
  }

  bool put(uint16_t dest,
           const baldr::GraphId& edgeid,
           float source, float target,
           const sif::Cost&  cost, float turn_cost, float sortcost,
           uint32_t predecessor,
           const baldr::DirectedEdge* edge,
           sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel);

  uint32_t pop();

  const Label& label(uint32_t label_idx) const
  { return labels_[label_idx]; }

  void clear_queue()
  { queue_->clear(); }

  void clear_status()
  {
    node_status_.clear();
    dest_status_.clear();
  }

 private:
  float max_cost_;
  std::shared_ptr<baldr::DoubleBucketQueue> queue_;
  std::unordered_map<baldr::GraphId, Status> node_status_;
  std::unordered_map<uint16_t, Status> dest_status_;
  std::vector<Label> labels_;
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
                   std::shared_ptr<const sif::EdgeLabel> edgelabel,
                   const float turn_cost_table[181]);

class RoutePathIterator:
      public std::iterator<std::forward_iterator_tag, const Label>
{
 public:
  RoutePathIterator(const LabelSet* labelset,
                    uint32_t label_idx)
      : labelset_(labelset),
        label_idx_(label_idx) {}

  // Construct a tail iterator
  RoutePathIterator(const LabelSet* labelset)
      : labelset_(labelset),
        label_idx_(baldr::kInvalidLabel) {}

  // Construct an invalid iterator
  RoutePathIterator()
      : labelset_(nullptr),
        label_idx_(baldr::kInvalidLabel) {}

  // Postfix increment
  RoutePathIterator operator++(int)
  {
    if (label_idx_ != baldr::kInvalidLabel) {
      auto clone = *this;
      label_idx_ = labelset_->label(label_idx_).predecessor;
      return clone;
    }
    return *this;
  }

  // Prefix increment
  RoutePathIterator& operator++()
  {
    if (label_idx_ != baldr::kInvalidLabel) {
      label_idx_ = labelset_->label(label_idx_).predecessor;
    }
    return *this;
  }

  bool operator==(const RoutePathIterator& other) const
  {
    return label_idx_ == other.label_idx_
        && labelset_ == other.labelset_;
  }

  bool operator!=(const RoutePathIterator& other) const
  { return !(*this == other); }

  // Derefrencnce
  reference operator*() const
  { return labelset_->label(label_idx_); }

  // Pointer dereference
  pointer operator->() const
  { return &(labelset_->label(label_idx_)); }

  bool is_valid() const
  { return labelset_ != nullptr; }

 private:
  const LabelSet* labelset_;
  uint32_t label_idx_;
};

}
}
#endif // MMP_ROUTING_H_
