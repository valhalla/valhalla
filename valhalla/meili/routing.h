// -*- mode: c++ -*-
#ifndef MMP_ROUTING_H_
#define MMP_ROUTING_H_

#include <vector>
#include <unordered_set>
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


namespace valhalla{

namespace meili {


constexpr uint16_t kInvalidDestination = std::numeric_limits<uint16_t>::max();
constexpr uint32_t kInvalidLabelIndex = std::numeric_limits<uint32_t>::max();


template<typename key_t, key_t invalid_key>
class BucketQueue
{
 public:
  using size_type = typename std::vector<std::vector<key_t>>::size_type;

  BucketQueue(size_type count, float size = 1.f)
      : bucket_count_(count),
        bucket_size_(size),
        top_(0),
        costmap_(),
        buckets_()
  {
    if (bucket_size_ <= 0.f) {
      throw std::invalid_argument("expect bucket size to be positive");
    }
    buckets_.reserve(bucket_count_);
  }

  bool add(const key_t& key, float cost)
  {
    if (cost < 0.f) {
      throw std::invalid_argument("expect non-negative cost");
    }

    const auto idx = bucket_idx(cost);
    if (idx < bucket_count_) {
      const auto it = costmap_.emplace(key, cost);
      if (!it.second) {
        throw std::runtime_error("the key " + std::to_string(key) + " exists,"
                                 " you probably should call the decrease method");
      }

      if (buckets_.size() <= idx) {
        buckets_.resize(idx + 1);
      }
      buckets_[idx].push_back(key);

      // Update top cursor
      if (idx < top_) {
        top_ = idx;
      }

      return true;
    }

    return false;
  }

  void decrease(const key_t& key, float cost)
  {
    if (cost < 0.f) {
      throw std::invalid_argument("expect non-negative cost");
    }

    const auto it = costmap_.find(key);
    if (it == costmap_.end()) {
      throw std::runtime_error("the key " + std::to_string(key) + " to decrease doesn't exists,"
                               " you probably should call the add method");
    }

    if (cost < it->second) {
      const auto old_idx = bucket_idx(it->second);
      const auto idx = bucket_idx(cost);
      if (old_idx < idx) {
        throw std::logic_error("invalid cost: " + std::to_string(cost) + " (old value is " + std::to_string(it->second) + ")");
      }

      // Remove the key from the old bucket
      auto& old_bucket = buckets_[old_idx];
      const auto old_key = std::find(old_bucket.begin(), old_bucket.end(), key);
      if (old_key == old_bucket.end()) {
        throw std::logic_error("the key " + std::to_string(key) + " in the cost map was failed to add to the bukcet");
      }
      old_bucket.erase(old_key);

      // Add the key to the new bucket
      buckets_[idx].push_back(key);
      // Update the cost
      it->second = cost;

      // Update top cursor
      if (idx < top_) {
        top_ = idx;
      }
    } else {
      throw std::runtime_error("the cost " + std::to_string(cost)
                               + " is not less than the cost (" + std::to_string(it->second)
                               + ") associated with the key (" + std::to_string(key)
                               + ") you requested to decrease ");
    }
  }

  float cost(const key_t& key) const
  {
    const auto it = costmap_.find(key);
    return it == costmap_.end()? -1.f : it->second;
  }

  key_t pop()
  {
    if (empty()) {
      return invalid_key;
    }

    const auto key = buckets_[top_].back();
    buckets_[top_].pop_back();
    costmap_.erase(key);
    return key;
  }

  bool empty() const
  {
    while (top_ < buckets_.size() && buckets_[top_].empty()) {
      top_++;
    }
    return top_ >= buckets_.size();
  }

  size_type size() const
  { return costmap_.size(); }

  void clear()
  {
    buckets_.clear();
    costmap_.clear();
    top_ = 0;
  }

 private:

  size_type bucket_count_;

  float bucket_size_;

  mutable size_type top_;

  std::unordered_map<key_t, float> costmap_;

  std::vector<std::vector<key_t>> buckets_;

  size_type bucket_idx(float cost) const
  { return static_cast<size_type>(cost / bucket_size_); }
};


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
                                                         the_edge->restrictions(),
                                                         the_edge->opp_local_idx(),
                                                         the_travelmode);
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
