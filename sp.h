// -*- mode: c++ -*-

#include <unordered_set>
#include <stdexcept>
#include <algorithm>
#include <cassert>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>

using namespace valhalla::baldr;


constexpr uint16_t kInvalidDestination = std::numeric_limits<uint16_t>::max();
constexpr uint32_t kInvalidLabelIndex = std::numeric_limits<uint32_t>::max();


template<typename key_t, key_t invalid_key>
class BucketQueue
{
 public:
  BucketQueue(size_t count, float size = 1.f)
      : bucket_count_(count),
        bucket_size_(size),
        top_(0) {
    if (bucket_size_ <= 0.f) {
      throw std::invalid_argument("expect bucket size to be positive");
    }
    buckets_.reserve(bucket_count_);
  }

  bool add(const key_t& key, float cost) {
    if (cost < 0.f) {
      throw std::invalid_argument("expect non-negative cost");
    }

    auto idx = bucket_idx(cost);

    // TODO split them into add and update two functions
    auto it = costmap_.find(key);
    if (it == costmap_.end()) {
      if (idx < bucket_count_) {
        if (buckets_.size() <= idx) {
          buckets_.resize(idx+1);
        }
        buckets_[idx].push_back(key);
        costmap_[key] = cost;

        // Update top cursor
        if (idx < top_) {
          top_ = idx;
        }

        return true;
      }
    } else if (cost < it->second) {
      // Remove the old item
      auto old_idx = bucket_idx(it->second);
      if (idx > old_idx) {
        throw std::runtime_error("invalid cost: " + std::to_string(cost) + " (old value is " + std::to_string(it->second) + ")");
      }
      auto& keys = buckets_[old_idx];
      auto it = std::find(keys.begin(), keys.end(), key);
      assert(it != keys.end());
      keys.erase(it);

      // Add the new one
      buckets_[idx].push_back(key);
      costmap_[key] = cost;

      // Update top cursor
      if (idx < top_) {
        top_ = idx;
      }

      return true;
    }

    return false;
  }

  key_t pop() {
    if (empty()) {
      return invalid_key;
    }

    auto key = buckets_[top_].back();
    buckets_[top_].pop_back();
    costmap_.erase(key);
    return key;
  }

  bool empty() const {
    while (top_ < buckets_.size() && buckets_[top_].empty()) {
      top_++;
    }
    return top_ >= buckets_.size();
  }

  inline size_t size() const {
    return costmap_.size();
  }

  void clear() {
    buckets_.clear();
    costmap_.clear();
    top_ = 0;
  }

 private:
  size_t bucket_count_;
  float bucket_size_;
  mutable size_t top_;
  std::unordered_map<key_t, float> costmap_;
  std::vector<std::vector<key_t> > buckets_;

  inline size_t bucket_idx(float cost) const {
    return static_cast<size_t>(cost / bucket_size_);
  }
};


struct Label
{
  Label() = default;

  Label(const GraphId& the_nodeid, const GraphId& the_edgeid,
        float the_source, float the_target, float the_cost,
        uint32_t the_predecessor)
      : nodeid(the_nodeid), dest(kInvalidDestination), edgeid(the_edgeid),
        source(the_source), target(the_target),
        cost(the_cost), predecessor(the_predecessor) {
  }

  Label(uint16_t the_dest, const GraphId& the_edgeid,
        float the_source, float the_target, float the_cost,
        uint32_t the_predecessor)
      : dest(the_dest), edgeid(the_edgeid),
        source(the_source), target(the_target),
        cost(the_cost), predecessor(the_predecessor) {
  }

  GraphId nodeid;
  uint16_t dest;

  GraphId edgeid;

  // assert: 0.f <= source <= target <= 1.f
  float source;
  float target;

  // Cost since origin (including cost of this edge)
  float cost;

  uint32_t predecessor;
};


struct Status
{
  uint32_t label_idx : 31;
  uint32_t permanent : 1;
};


class LabelSet
{
 public:
  LabelSet(size_t count, float size = 1.f)
      : queue_(count, size) {
  }

  inline bool put(const GraphId& nodeid) {
    return put(nodeid, GraphId(), 0.f, 0.f, 0.f, kInvalidLabelIndex);
  }

  bool put(const GraphId& nodeid, const GraphId& edgeid,
           float source, float target,
           float cost, uint32_t predecessor) {
    if (!nodeid.Is_Valid()) {
      throw std::runtime_error("invalid nodeid");
    }
    const auto it = node_status_.find(nodeid);
    if (it == node_status_.end()) {
      uint32_t idx = labels_.size();
      bool inserted = queue_.add(idx, cost);
      if (inserted) {
        labels_.emplace_back(nodeid, edgeid, source, target, cost, predecessor);
        node_status_[nodeid] = {idx, false};
        return true;
      }
    } else {
      const auto& status = it->second;
      if (!status.permanent && cost < labels_[status.label_idx].cost) {
        labels_[status.label_idx] = {nodeid, edgeid, source, target, cost, predecessor};
        bool updated = queue_.add(status.label_idx, cost);
        assert(updated);
        return true;
      }
    }

    return false;
  }

  inline bool put(uint16_t dest) {
    return put(dest, GraphId(), 0.f, 0.f, 0.f, kInvalidLabelIndex);
  }

  bool put(uint16_t dest, const GraphId& edgeid,
           float source, float target,
           float cost, uint32_t predecessor) {
    if (dest == kInvalidDestination) {
      throw std::runtime_error("invalid destination");
    }

    const auto it = dest_status_.find(dest);
    if (it == dest_status_.end()) {
      uint32_t idx = labels_.size();
      bool inserted = queue_.add(idx, cost);
      if (inserted) {
        labels_.emplace_back(dest, edgeid, source, target, cost, predecessor);
        dest_status_[dest] = {idx, false};
        return true;
      }
    } else {
      const auto& status = it->second;
      if (!status.permanent && cost < labels_[status.label_idx].cost) {
        labels_[status.label_idx] = {dest, edgeid, source, target, cost, predecessor};
        bool updated = queue_.add(status.label_idx, cost);
        assert(updated);
        return true;
      }
    }

    return false;
  }

  inline uint32_t pop() {
    auto idx = queue_.pop();

    if (idx != kInvalidLabelIndex) {
      const auto& label = labels_[idx];
      if (label.nodeid.Is_Valid()) {
        assert(node_status_[label.nodeid].label_idx == idx);
        assert(!node_status_[label.nodeid].permanent);
        node_status_[label.nodeid].permanent = true;
      } else {
        assert(dest_status_[label.dest].label_idx == idx);
        assert(!dest_status_[label.dest].permanent);
        assert(label.dest != kInvalidDestination);
        dest_status_[label.dest].permanent = true;
      }
    }

    return idx;
  }

  inline bool empty() const {
    return queue_.empty();
  }

  inline const Label& label(uint32_t label_idx) const {
    return labels_[label_idx];
  }

  inline void clear_queue() {
    queue_.clear();
  }

  inline void clear_status() {
    node_status_.clear();
    dest_status_.clear();
  }

 private:
  BucketQueue<uint32_t, kInvalidLabelIndex> queue_;
  std::unordered_map<GraphId, Status> node_status_;
  std::unordered_map<size_t, Status> dest_status_;
  std::vector<Label> labels_;
};


void set_origin(GraphReader& reader,
                const std::vector<PathLocation>& destinations,
                uint16_t origin_idx,
                LabelSet& labelset)
{
  for (const auto& edge : destinations[origin_idx].edges()) {
    if (edge.dist == 0.f) {
      auto opp_edge = reader.GetOpposingEdge(edge.id);
      if (!opp_edge) continue;
      labelset.put(opp_edge->endnode());
    } else if (edge.dist == 1.f) {
      auto tile = reader.GetGraphTile(edge.id);
      if (!tile) continue;
      auto directededge = tile->directededge(edge.id);
      if (!directededge) continue;
      labelset.put(directededge->endnode());
    } else {
      labelset.put(origin_idx);
    }
  }
}

void set_destinations(GraphReader& reader,
                      const std::vector<PathLocation>& destinations,
                      std::unordered_map<GraphId, std::unordered_set<uint16_t>>& node_dests,
                      std::unordered_map<GraphId, std::unordered_set<uint16_t>>& edge_dests)
{
  for (uint16_t dest = 0; dest < destinations.size(); dest++) {
    for (const auto& edge : destinations[dest].edges()) {
      if (edge.dist == 0.f) {
        auto opp_edge = reader.GetOpposingEdge(edge.id);
        if (!opp_edge) continue;
        node_dests[opp_edge->endnode()].insert(dest);
      } else if (edge.dist == 1.f) {
        auto tile = reader.GetGraphTile(edge.id);
        if (!tile) continue;
        auto directededge = tile->directededge(edge.id);
        if (!directededge) continue;
        node_dests[directededge->endnode()].insert(dest);
      } else {
        edge_dests[edge.id].insert(dest);
      }
    }
  }
}


std::unordered_map<uint16_t, uint32_t>
find_shortest_path(GraphReader& reader,
                   const std::vector<PathLocation>& destinations,
                   uint16_t origin_idx,
                   LabelSet& labelset)
{
  // Destinations at nodes
  std::unordered_map<GraphId, std::unordered_set<uint16_t>> node_dests;

  // Destinations along edges
  std::unordered_map<GraphId, std::unordered_set<uint16_t>> edge_dests;

  // Load destinations
  set_destinations(reader, destinations, node_dests, edge_dests);

  // Load origin to the queue of the labelset
  set_origin(reader, destinations, origin_idx, labelset);

  std::unordered_map<uint16_t, uint32_t> results;

  while (!labelset.empty()) {
    auto label_idx = labelset.pop();
    // NOTE this refernce is possible to be invalid when you add
    // labels to the set later (which causes the label list
    // reallocated)
    const auto& label = labelset.label(label_idx);
    // So we cache the cost that will be used during expanding
    auto label_cost = label.cost;

    if (label.nodeid.Is_Valid()) {
      const auto& nodeid = label.nodeid;

      // If this node is a destination, path to destinations at this
      // node is found: remember them and remove this node from the
      // destination list
      auto it = node_dests.find(nodeid);
      if (it != node_dests.end()) {
        for (auto dest : it->second) {
          results[dest] = label_idx;
        }
        node_dests.erase(it);
      }

      // Congrats!
      if (node_dests.empty() && edge_dests.empty()) {
        break;
      }

      // Expand current node
      auto tile = reader.GetGraphTile(nodeid);
      if (!tile) continue;
      const auto nodeinfo = tile->node(nodeid);
      if (!nodeinfo) continue;
      GraphId other_edgeid(nodeid.tileid(), nodeid.level(), nodeinfo->edge_index());
      auto other_edge = tile->directededge(nodeinfo->edge_index());
      for (size_t i = 0; i < nodeinfo->edge_count(); i++, other_edge++, other_edgeid++) {
        // If destinations found along the edge, add segments to each
        // destination to the queue
        auto it = edge_dests.find(other_edgeid);
        if (it != edge_dests.end()) {
          for (const auto dest : it->second) {
            for (const auto& edge : destinations[dest].edges()) {
              if (edge.id == other_edgeid) {
                auto cost = label_cost + other_edge->length() * edge.dist;
                labelset.put(dest, other_edgeid, 0.f, edge.dist, cost, label_idx);
              }
            }
          }
        }

        float cost = label_cost + other_edge->length();
        labelset.put(other_edge->endnode(), other_edgeid, 0.f, 1.f, cost, label_idx);
      }
    } else {
      assert(label.dest != kInvalidDestination);
      const auto dest = label.dest;

      // Path to a destination along an edge is found: remember it and
      // remove the destination from the destination list
      results[dest] = label_idx;
      for (const auto& edge : destinations[dest].edges()) {
        auto it = edge_dests.find(edge.id);
        if (it != edge_dests.end()) {
          it->second.erase(dest);
          if (it->second.empty()) {
            edge_dests.erase(it);
          }
        }
      }

      // Congrats!
      if (edge_dests.empty() && node_dests.empty()) {
        break;
      }

      // Expand origin: add segments from origin to destinations ahead
      // at the same edge to the queue
      if (dest == origin_idx) {
        for (const auto& origin_edge : destinations[origin_idx].edges()) {

          auto tile = reader.GetGraphTile(origin_edge.id);
          if (!tile) continue;
          auto directededge = tile->directededge(origin_edge.id);
          if (!directededge) continue;

          // All destinations on this origin edge
          for (auto other_dest : edge_dests[origin_edge.id]) {
            // All edges of this destination
            for (const auto& other_edge : destinations[other_dest].edges()) {
              if (origin_edge.id == other_edge.id && origin_edge.dist <= other_edge.dist) {
                float cost = label_cost + directededge->length() * (other_edge.dist - origin_edge.dist);
                labelset.put(other_dest, origin_edge.id, origin_edge.dist, other_edge.dist, cost, label_idx);
              }
            }
          }

          float cost = label_cost + directededge->length() * (1.f - origin_edge.dist);
          labelset.put(directededge->endnode(), origin_edge.id, origin_edge.dist, 1.f, cost, label_idx);
        }
      }
    }
  }

  labelset.clear_queue();
  labelset.clear_status();

  return results;
}


class RoutePathIterator:
    public std::iterator<std::forward_iterator_tag,
                         const Label>
{
 public:
  RoutePathIterator(const LabelSet* labelset,
                    uint32_t label_idx)
      : labelset_(labelset),
        label_idx_(label_idx)
  {
  }

  // Construct a tail iterator
  RoutePathIterator(const LabelSet* labelset)
      : labelset_(labelset),
        label_idx_(kInvalidLabelIndex)
  {
  }

  // Construct an invalid iterator
  RoutePathIterator()
      : labelset_(nullptr),
        label_idx_(kInvalidLabelIndex)
  {
  }

  // Postfix increment
  RoutePathIterator& operator++(int)
  {
    if (label_idx_ != kInvalidLabelIndex) {
      label_idx_ = labelset_->label(label_idx_).predecessor;
    }
    return *this;
  }

  // Prefix increment
  RoutePathIterator operator++()
  {
    operator++(1);  // Call postfix increment
    return RoutePathIterator(labelset_, label_idx_);
  }

  bool operator==(const RoutePathIterator& other) const
  {
    return labelset_ == other.labelset_
        && label_idx_ == other.label_idx_;
  }

  bool operator!=(const RoutePathIterator& other) const
  {
    return !(*this == other);
  }

  // Derefrencnce
  std::iterator_traits<RoutePathIterator>::reference
  operator*() const
  {
    return labelset_->label(label_idx_);
  }

  // Pointer dereference
  std::iterator_traits<RoutePathIterator>::pointer
  operator->() const
  {
    return &(labelset_->label(label_idx_));
  }

  bool is_valid() const
  {
    return labelset_ != nullptr;
  }

 private:
  const LabelSet* labelset_;
  uint32_t label_idx_;
};
