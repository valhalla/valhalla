// -*- mode: c++ -*-
#ifndef MMP_ROUTING_H_
#define MMP_ROUTING_H_

#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <algorithm>

#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/costconstants.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/sif/dynamiccost.h>

#include <valhalla/meili/bucket_queue.h>


namespace valhalla{
namespace meili {

constexpr uint16_t kInvalidDestination = std::numeric_limits<uint16_t>::max();
constexpr uint32_t kInvalidLabelIndex = std::numeric_limits<uint32_t>::max();


// TODO simplify it by inheriting sif::EdgeLabel
struct Label
{
  Label() = delete;

  Label(const baldr::GraphId& the_nodeid,
        const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        float the_cost, float the_turn_cost, float the_sortcost,
        uint32_t the_predecessor,
        const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : Label(the_nodeid, kInvalidDestination, the_edgeid,
              the_source, the_target,
              the_cost, the_turn_cost, the_sortcost,
              the_predecessor,
              the_edge, the_travelmode, the_edgelabel) {}

  Label(uint16_t the_dest,
        const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        float the_cost, float the_turn_cost, float the_sortcost,
        uint32_t the_predecessor,
        const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : Label({}, the_dest, the_edgeid,
              the_source, the_target,
              the_cost, the_turn_cost, the_sortcost,
              the_predecessor,
              the_edge, the_travelmode, the_edgelabel) {}

  Label(const baldr::GraphId& the_nodeid,
        uint16_t the_dest,
        const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        float the_cost, float the_turn_cost, float the_sortcost,
        uint32_t the_predecessor,
        const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : nodeid(the_nodeid), dest(the_dest), edgeid(the_edgeid),
        source(the_source), target(the_target),
        cost(the_cost), turn_cost(the_turn_cost), sortcost(the_sortcost),
        predecessor(the_predecessor),
        edgelabel(the_edgelabel)
  {
    if (!((nodeid.Is_Valid() && dest == kInvalidDestination) || (!nodeid.Is_Valid() && dest != kInvalidDestination))) {
      throw std::invalid_argument("nodeid and dest must be mutually exclusive, i.e. either nodeid is valid or dest is valid");
    }

    if (!(0.f <= source && source <= target && target <= 1.f)) {
      throw std::invalid_argument("invalid source ("
                                  + std::to_string(source)
                                  + ") or target ("
                                  + std::to_string(target) + ")");
    }

    if (cost < 0.f) {
      throw std::invalid_argument("invalid cost = " + std::to_string(cost));
    }

    if (turn_cost < 0.f) {
      throw std::invalid_argument("invalid turn_cost = " + std::to_string(turn_cost));
    }

    if (!edgelabel && the_edge) {
      edgelabel = std::make_shared<const sif::EdgeLabel>(the_predecessor,
                                                         the_edgeid,
                                                         the_edge,
                                                         sif::Cost(the_cost, the_cost), // Cost
                                                         sortcost, // Sortcost
                                                         the_cost, // Distance
                                                         the_travelmode,
                                                         0);
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

  // Cost since origin (including the cost of this edge segment)
  float cost;

  // Turn cost since origin (including the turn cost of this edge
  // segment)
  float turn_cost;

  // For ranking labels: sortcost = accumulated cost since origin + heuristic cost to the goal
  float sortcost;

  // kInvalidLabelIndex if dummy
  uint32_t predecessor;

  // An EdgeLabel is needed here for passing to sif's filters later
  std::shared_ptr<const sif::EdgeLabel> edgelabel;
};


struct Status{
  Status() = delete;

  Status(uint32_t idx)
      : label_idx(idx),
        permanent(false) {}

  uint32_t label_idx : 31;

  uint32_t permanent : 1;
};


class LabelSet
{
 public:
  LabelSet(typename BucketQueue<uint32_t, kInvalidLabelIndex>::size_type count, float size = 1.f);

  bool put(const baldr::GraphId& nodeid, sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel);

  bool put(const baldr::GraphId& nodeid,
           const baldr::GraphId& edgeid,
           float source, float target,
           float cost, float turn_cost, float sortcost,
           uint32_t predecessor,
           const baldr::DirectedEdge* edge,
           sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel);

  bool put(uint16_t dest, sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel);

  bool put(uint16_t dest,
           const baldr::GraphId& edgeid,
           float source, float target,
           float cost, float turn_cost, float sortcost,
           uint32_t predecessor,
           const baldr::DirectedEdge* edge,
           sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel);

  uint32_t pop();

  bool empty() const
  { return queue_.empty(); }

  const Label& label(uint32_t label_idx) const
  { return labels_[label_idx]; }

  void clear_queue()
  { queue_.clear(); }

  void clear_status()
  {
    node_status_.clear();
    dest_status_.clear();
  }

 private:
  BucketQueue<uint32_t, kInvalidLabelIndex> queue_;
  std::unordered_map<baldr::GraphId, Status> node_status_;
  std::unordered_map<uint16_t, Status> dest_status_;
  std::vector<Label> labels_;
};


std::unordered_map<uint16_t, uint32_t>
find_shortest_path(baldr::GraphReader& reader,
                   const std::vector<baldr::PathLocation>& destinations,
                   uint16_t origin_idx,
                   LabelSet& labelset,
                   const midgard::DistanceApproximator& approximator,
                   float search_radius,
                   sif::cost_ptr_t costing = nullptr,
                   std::shared_ptr<const sif::EdgeLabel> edgelabel = nullptr,
                   const float turn_cost_table[181] = nullptr);


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
        label_idx_(kInvalidLabelIndex) {}

  // Construct an invalid iterator
  RoutePathIterator()
      : labelset_(nullptr),
        label_idx_(kInvalidLabelIndex) {}

  // Postfix increment
  RoutePathIterator operator++(int)
  {
    if (label_idx_ != kInvalidLabelIndex) {
      auto clone = *this;
      label_idx_ = labelset_->label(label_idx_).predecessor;
      return clone;
    }
    return *this;
  }

  // Prefix increment
  RoutePathIterator& operator++()
  {
    if (label_idx_ != kInvalidLabelIndex) {
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
