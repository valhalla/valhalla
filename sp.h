// -*- mode: c++ -*-

#include <unordered_set>
#include <stdexcept>
#include <algorithm>
#include <cassert>

#include <valhalla/midgard/util.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/sif/dynamiccost.h>

#include "graph_helpers.h"

namespace mm {

using namespace valhalla;


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
        buckets_() {
    if (bucket_size_ <= 0.f) {
      throw std::invalid_argument("expect bucket size to be positive");
    }
    buckets_.reserve(bucket_count_);
  }

  bool add(const key_t& key, float cost) {
    if (cost < 0.f) {
      throw std::invalid_argument("expect non-negative cost");
    }

    const auto idx = bucket_idx(cost);

    // TODO split them into add and update two functions
    const auto it = costmap_.find(key);
    if (it == costmap_.end()) {
      if (idx < bucket_count_) {
        if (buckets_.size() <= idx) {
          buckets_.resize(idx + 1);
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
      const auto old_idx = bucket_idx(it->second);
      if (idx > old_idx) {
        throw std::runtime_error("invalid cost: " + std::to_string(cost) + " (old value is " + std::to_string(it->second) + ")");
      }
      auto& keys = buckets_[old_idx];
      const auto it = std::find(keys.begin(), keys.end(), key);
      assert(it != keys.end());
      keys.erase(it);

      // Add the new one
      assert (idx < buckets_.size());
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

    const auto key = buckets_[top_].back();
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

  size_type size() const {
    return costmap_.size();
  }

  void clear() {
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

  size_type bucket_idx(float cost) const {
    return static_cast<size_type>(cost / bucket_size_);
  }
};


struct Label
{
  Label() = delete;

  Label(const baldr::GraphId& the_nodeid,
        const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        float the_cost, float the_turn_cost,
        uint32_t the_predecessor,
        const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : Label(the_nodeid, kInvalidDestination, the_edgeid,
              the_source, the_target,
              the_cost, the_turn_cost,
              the_predecessor,
              the_edge, the_travelmode, the_edgelabel) {}

  Label(uint16_t the_dest,
        const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        float the_cost, float the_turn_cost,
        uint32_t the_predecessor,
        const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : Label({}, the_dest, the_edgeid,
              the_source, the_target,
              the_cost, the_turn_cost,
              the_predecessor,
              the_edge, the_travelmode, the_edgelabel)
  { assert(!nodeid.Is_Valid()); }

  Label(const baldr::GraphId& the_nodeid,
        uint16_t the_dest,
        const baldr::GraphId& the_edgeid,
        float the_source, float the_target,
        float the_cost, float the_turn_cost,
        uint32_t the_predecessor,
        const baldr::DirectedEdge* the_edge,
        sif::TravelMode the_travelmode,
        std::shared_ptr<const sif::EdgeLabel> the_edgelabel)
      : nodeid(the_nodeid), dest(the_dest), edgeid(the_edgeid),
        source(the_source), target(the_target),
        cost(the_cost), turn_cost(the_turn_cost),
        predecessor(the_predecessor),
        edgelabel(the_edgelabel)
  {
    if (!(0.f <= source && source <= target && target <= 1.f)) {
      throw std::runtime_error("invalid source ("
                               + std::to_string(source)
                               + ") or target ("
                               + std::to_string(target) + ")");
    }

    if (cost < 0.f) {
      throw std::runtime_error("invalid cost = " + std::to_string(cost));
    }

    if (turn_cost < 0.f) {
      throw std::runtime_error("invalid turn_cost = " + std::to_string(turn_cost));
    }

    if (!edgelabel && the_edge) {
      edgelabel = std::make_shared<const sif::EdgeLabel>(the_predecessor,
                                                         the_edgeid,
                                                         the_edge,
                                                         sif::Cost(the_cost, the_cost), // Cost
                                                         the_cost, // Sortcost
                                                         the_cost, // Distance
                                                         the_edge->restrictions(),
                                                         the_edge->opp_local_idx(),
                                                         the_travelmode);
    }
  }

  // assert: nodeid.Is_Valid() XOR dest != kInvalidDestination
  baldr::GraphId nodeid;
  uint16_t dest;

  baldr::GraphId edgeid;

  // assert: 0.f <= source <= target <= 1.f
  float source;
  float target;

  // Cost since origin (including the cost of this edge segment)
  float cost;

  // Turn cost since origin (including the turn cost of this edge
  // segment)
  float turn_cost;

  uint32_t predecessor;

  // An EdgeLabel is needed here for passing to sif's filters later
  std::shared_ptr<const sif::EdgeLabel> edgelabel;
};


struct Status
{
  uint32_t label_idx : 31;
  uint32_t permanent : 1;
};


class LabelSet
{
 public:
  LabelSet(typename BucketQueue<uint32_t, kInvalidLabelIndex>::size_type count,
           float size = 1.f)
      : queue_(count, size) {}

  bool put(const baldr::GraphId& nodeid, sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel)
  {
    return put(nodeid, {},         // nodeid, (invalid) edgeid
               0.f, 0.f,           // source, target
               0.f, 0.f,           // cost, turn cost
               kInvalidLabelIndex, // predecessor
               nullptr, travelmode, edgelabel);
  }

  bool put(const baldr::GraphId& nodeid,
           const baldr::GraphId& edgeid,
           float source, float target,
           float cost, float turn_cost,
           uint32_t predecessor,
           const baldr::DirectedEdge* edge,
           sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel)
  {
    if (!nodeid.Is_Valid()) {
      throw std::runtime_error("invalid nodeid");
    }
    const auto it = node_status_.find(nodeid);
    if (it == node_status_.end()) {
      uint32_t idx = labels_.size();
      bool inserted = queue_.add(idx, cost);
      if (inserted) {
        labels_.emplace_back(nodeid, edgeid,
                             source, target,
                             cost, turn_cost,
                             predecessor,
                             edge, travelmode, edgelabel);
        node_status_[nodeid] = {idx, false};
        return true;
      }
    } else {
      const auto& status = it->second;
      if (!status.permanent && cost < labels_[status.label_idx].cost) {
        // TODO check if it goes through constructor
        labels_[status.label_idx] = {nodeid, edgeid,
                                     source, target,
                                     cost, turn_cost,
                                     predecessor,
                                     edge, travelmode, edgelabel};
        bool updated = queue_.add(status.label_idx, cost);
        assert(updated);
        return true;
      }
    }

    return false;
  }

  bool put(uint16_t dest, sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel)
  {
    return put(dest, {},           // dest, (invalid) edgeid
               0.f, 0.f,           // source, target
               0.f, 0.f,           // cost, turn_cost
               kInvalidLabelIndex, // predecessor
               nullptr, travelmode, edgelabel);
  }

  bool put(uint16_t dest,
           const baldr::GraphId& edgeid,
           float source, float target,
           float cost, float turn_cost,
           uint32_t predecessor,
           const baldr::DirectedEdge* edge,
           sif::TravelMode travelmode,
           std::shared_ptr<const sif::EdgeLabel> edgelabel)
  {
    if (dest == kInvalidDestination) {
      throw std::runtime_error("invalid destination");
    }

    const auto it = dest_status_.find(dest);
    if (it == dest_status_.end()) {
      uint32_t idx = labels_.size();
      bool inserted = queue_.add(idx, cost);
      if (inserted) {
        labels_.emplace_back(dest, edgeid,
                             source, target,
                             cost, turn_cost,
                             predecessor,
                             edge, travelmode, edgelabel);
        dest_status_[dest] = {idx, false};
        return true;
      }
    } else {
      const auto& status = it->second;
      if (!status.permanent && cost < labels_[status.label_idx].cost) {
        // TODO check if it goes through constructor
        labels_[status.label_idx] = {dest, edgeid,
                                     source, target,
                                     cost, turn_cost,
                                     predecessor,
                                     edge, travelmode, edgelabel};
        bool updated = queue_.add(status.label_idx, cost);
        assert(updated);
        return true;
      }
    }

    return false;
  }

  uint32_t pop()
  {
    const auto idx = queue_.pop();

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


inline bool
IsEdgeAllowed(const baldr::DirectedEdge* edge,
              const baldr::GraphId& edgeid,
              const sif::cost_ptr_t costing,
              const std::shared_ptr<const sif::EdgeLabel> pred_edgelabel,
              const sif::EdgeFilter edgefilter,
              const baldr::GraphTile* tile)
{
  if (costing) {
    if (pred_edgelabel) {
      // Still on the same edge and the predecessor's show-up here
      // means it was allowed so we give it a pass directly

      // TODO let sif do this?
      return edgeid == pred_edgelabel->edgeid()
          || costing->Allowed(edge, *pred_edgelabel, tile, edgeid);
    } else {
      if (edgefilter) {
        return !edgefilter(edge);
      } else {
        return true;
      }
    }
  } else {
    return true;
  }
}


void set_origin(baldr::GraphReader& reader,
                const std::vector<baldr::PathLocation>& destinations,
                uint16_t origin_idx,
                LabelSet& labelset,
                const sif::TravelMode travelmode,
                sif::cost_ptr_t costing,
                std::shared_ptr<const sif::EdgeLabel> edgelabel)
{
  const baldr::GraphTile* tile = nullptr;

  for (const auto& edge : destinations[origin_idx].edges()) {
    assert(edge.id.Is_Valid());
    if (!edge.id.Is_Valid()) continue;

    if (edge.dist == 0.f) {
      const auto nodeid = helpers::edge_startnodeid(reader, edge.id, tile);
      if (!nodeid.Is_Valid()) continue;

#if 1  // TODO perhaps we shouldn't check start node
      const auto nodeinfo = helpers::edge_nodeinfo(reader, nodeid, tile);
      if (!nodeinfo) continue;
      if (costing && !costing->Allowed(nodeinfo)) continue;
#endif

      labelset.put(nodeid, travelmode, edgelabel);
    } else if (edge.dist == 1.f) {
      const auto nodeid = helpers::edge_endnodeid(reader, edge.id, tile);
      if (!nodeid.Is_Valid()) continue;

#if 1  // TODO perhaps we shouldn't check start node
      const auto nodeinfo = helpers::edge_nodeinfo(reader, nodeid, tile);
      if (!nodeinfo) continue;
      if (costing && !costing->Allowed(nodeinfo)) continue;
#endif

      labelset.put(nodeid, travelmode, edgelabel);
    } else {
      assert(0.f < edge.dist && edge.dist < 1.f);
      // Will decide whether to filter out this edge later
      labelset.put(origin_idx, travelmode, edgelabel);
    }
  }
}


void set_destinations(baldr::GraphReader& reader,
                      const std::vector<baldr::PathLocation>& destinations,
                      std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>>& node_dests,
                      std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>>& edge_dests)
{
  const baldr::GraphTile* tile = nullptr;

  for (uint16_t dest = 0; dest < destinations.size(); dest++) {
    for (const auto& edge : destinations[dest].edges()) {
      assert(edge.id.Is_Valid());
      if (!edge.id.Is_Valid()) continue;

      if (edge.dist == 0.f) {
        const auto nodeid = helpers::edge_startnodeid(reader, edge.id, tile);
        if (!nodeid.Is_Valid()) continue;
        node_dests[nodeid].insert(dest);

      } else if (edge.dist == 1.f) {
        const auto nodeid = helpers::edge_endnodeid(reader, edge.id, tile);
        if (!nodeid.Is_Valid()) continue;
        node_dests[nodeid].insert(dest);

      } else {
        edge_dests[edge.id].insert(dest);
      }
    }
  }
}


inline uint16_t
get_inbound_edgelabel_heading(baldr::GraphReader& graphreader,
                              const baldr::GraphTile* tile,
                              const sif::EdgeLabel& edgelabel,
                              const baldr::NodeInfo& nodeinfo)
{
  const auto idx = edgelabel.opp_local_idx();
  if (idx < 8) {
    return nodeinfo.heading(idx);
  } else {
    const auto directededge = helpers::edge_directededge(graphreader, edgelabel.edgeid(), tile);
    const auto edgeinfo = tile->edgeinfo(directededge->edgeinfo_offset());
    const auto& shape = edgeinfo->shape();
    if (shape.size() >= 2) {
      float heading;
      if (directededge->forward()) {
        heading = shape.back().Heading(shape.rbegin()[1]);
      } else {
        heading = shape.front().Heading(shape[1]);
      }
      return static_cast<uint16_t>(std::max(0.f, std::min(359.f, heading)));
    } else {
      return 0;
    }
  }
}


inline uint16_t
get_outbound_edge_heading(const baldr::GraphTile* tile,
                          const baldr::DirectedEdge* outbound_edge,
                          const baldr::NodeInfo& nodeinfo)
{
  const auto idx = outbound_edge->localedgeidx();
  if (idx < 8) {
    return nodeinfo.heading(idx);
  } else {
    const auto edgeinfo = tile->edgeinfo(outbound_edge->edgeinfo_offset());
    const auto& shape = edgeinfo->shape();
    if (shape.size() >= 2) {
      float heading;
      if (outbound_edge->forward()) {
        heading = shape.front().Heading(shape[1]);
      } else {
        heading = shape.back().Heading(shape.rbegin()[1]);
      }
      return static_cast<uint16_t>(std::max(0.f, std::min(359.f, heading)));
    } else {
      return 0;
    }
  }
}


inline uint8_t
get_turn_degree(uint16_t left, uint16_t right)
{
  const auto turn = std::abs(left - right);
  assert(0 <= turn && turn < 360);
  return turn > 180? 360 - turn : turn;
}


std::unordered_map<uint16_t, uint32_t>
find_shortest_path(baldr::GraphReader& reader,
                   const std::vector<baldr::PathLocation>& destinations,
                   uint16_t origin_idx,
                   LabelSet& labelset,
                   sif::cost_ptr_t costing = nullptr,
                   std::shared_ptr<const sif::EdgeLabel> edgelabel = nullptr,
                   const float turn_cost_table[181] = nullptr)
{
  // Destinations at nodes
  std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>> node_dests;

  // Destinations along edges
  std::unordered_map<baldr::GraphId, std::unordered_set<uint16_t>> edge_dests;

  // Load destinations
  set_destinations(reader, destinations, node_dests, edge_dests);

  const sif::TravelMode travelmode = costing? costing->travelmode() : static_cast<sif::TravelMode>(0);

  // Load origin to the queue of the labelset
  set_origin(reader, destinations, origin_idx, labelset, travelmode, costing, edgelabel);

  std::unordered_map<uint16_t, uint32_t> results;

  const auto edgefilter = costing? costing->GetFilter() : nullptr;

  const baldr::GraphTile* tile = nullptr;

  while (!labelset.empty()) {
    const auto label_idx = labelset.pop();
    // NOTE this refernce is possible to be invalid when you add
    // labels to the set later (which causes the label list
    // reallocated)
    const auto& label = labelset.label(label_idx);

    // So we cache the costs that will be used during expanding
    const auto label_cost = label.cost;
    const auto label_turn_cost = label.turn_cost;
    // and edgelabel pointer for checking edge accessibility later
    const auto pred_edgelabel = label.edgelabel;

    if (label.nodeid.Is_Valid()) {
      const auto nodeid = label.nodeid;

      // If this node is a destination, path to destinations at this
      // node is found: remember them and remove this node from the
      // destination list
      const auto it = node_dests.find(nodeid);
      if (it != node_dests.end()) {
        for (const auto dest : it->second) {
          results[dest] = label_idx;
        }
        node_dests.erase(it);
      }

      // Congrats!
      if (node_dests.empty() && edge_dests.empty()) {
        break;
      }

      const auto nodeinfo = helpers::edge_nodeinfo(reader, nodeid, tile);
      if (!nodeinfo || nodeinfo->edge_count() <= 0) continue;

      if (costing && !costing->Allowed(nodeinfo)) continue;

      const auto inbound_heading = (pred_edgelabel && turn_cost_table)?
                                   get_inbound_edgelabel_heading(reader, tile, *pred_edgelabel, *nodeinfo) : 0;
      assert(0 <= inbound_heading && inbound_heading < 360);

      // Expand current node
      baldr::GraphId other_edgeid(nodeid.tileid(), nodeid.level(), nodeinfo->edge_index());
      auto other_edge = tile->directededge(nodeinfo->edge_index());
      assert(other_edge);
      for (size_t i = 0; i < nodeinfo->edge_count(); i++, other_edge++, other_edgeid++) {
        // Disable shortcut TODO perhaps we should use
        // other_edge->is_shortcut()? but it failed to guarantee same
        // level
        if (nodeid.level() != other_edge->endnode().level()) continue;

        if (!IsEdgeAllowed(other_edge, other_edgeid, costing, pred_edgelabel, edgefilter, tile)) continue;

        // Turn cost
        float turn_cost = 0.f;
        if (pred_edgelabel && turn_cost_table) {
          const auto other_heading = get_outbound_edge_heading(tile, other_edge, *nodeinfo);
          assert(0 <= other_heading && other_heading < 360);
          const auto turn_degree = get_turn_degree(inbound_heading, other_heading);
          assert(0 <= turn_degree && turn_degree <= 180);
          turn_cost = label_turn_cost + turn_cost_table[turn_degree];
        }

        // If destinations found along the edge, add segments to each
        // destination to the queue
        const auto it = edge_dests.find(other_edgeid);
        if (it != edge_dests.end()) {
          for (const auto dest : it->second) {
            for (const auto& edge : destinations[dest].edges()) {
              if (edge.id == other_edgeid) {
                const float cost = label_cost + other_edge->length() * edge.dist;
                labelset.put(dest, other_edgeid,
                             0.f, edge.dist,
                             cost, turn_cost,
                             label_idx,
                             other_edge, travelmode, nullptr);
              }
            }
          }
        }

        const float cost = label_cost + other_edge->length();
        labelset.put(other_edge->endnode(), other_edgeid,
                     0.f, 1.f,
                     cost, turn_cost,
                     label_idx,
                     other_edge, travelmode, nullptr);
      }
    } else {
      assert(label.dest != kInvalidDestination);
      const auto dest = label.dest;

      // Path to a destination along an edge is found: remember it and
      // remove the destination from the destination list
      results[dest] = label_idx;
      for (const auto& edge : destinations[dest].edges()) {
        const auto it = edge_dests.find(edge.id);
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
          const auto directededge = helpers::edge_directededge(reader, origin_edge.id, tile);
          if (!directededge) continue;

          if (!IsEdgeAllowed(directededge, origin_edge.id, costing, pred_edgelabel, edgefilter, tile)) continue;

          // U-turn cost
          float turn_cost = 0.f;
          if (pred_edgelabel && turn_cost_table
              && pred_edgelabel->edgeid() != origin_edge.id
              && pred_edgelabel->opp_local_idx() == directededge->localedgeidx()) {
            turn_cost = label_turn_cost + turn_cost_table[0];
          }

          // All destinations on this origin edge
          for (const auto other_dest : edge_dests[origin_edge.id]) {
            // All edges of this destination
            for (const auto& other_edge : destinations[other_dest].edges()) {
              if (origin_edge.id == other_edge.id && origin_edge.dist <= other_edge.dist) {
                const float cost = label_cost + directededge->length() * (other_edge.dist - origin_edge.dist);
                labelset.put(other_dest, origin_edge.id,
                             origin_edge.dist, other_edge.dist,
                             cost, turn_cost,
                             label_idx,
                             directededge, travelmode, nullptr);
              }
            }
          }

          const float cost = label_cost + directededge->length() * (1.f - origin_edge.dist);
          labelset.put(directededge->endnode(), origin_edge.id,
                       origin_edge.dist, 1.f,
                       cost, turn_cost,
                       label_idx,
                       directededge, travelmode, nullptr);
        }
      }
    }
  }

  labelset.clear_queue();
  labelset.clear_status();

  return results;
}


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
