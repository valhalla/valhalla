#include <iostream> // TODO remove if not needed
#include <map>
#include <algorithm>
#include "thor/trafficalgorithm.h"
#include "baldr/datetime.h"
#include "midgard/logging.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

// TODO: make a class that extends std::exception, with messages and
// error codes and return the appropriate error codes

namespace valhalla {
namespace thor {

// Default constructor
TrafficAlgorithm::TrafficAlgorithm()
    : AStarPathAlgorithm() {
}

// Destructor
TrafficAlgorithm::~TrafficAlgorithm() {
  Clear();
}

// Calculate best path. This method is single mode, not time-dependent.
std::vector<PathInfo> TrafficAlgorithm::GetBestPath(PathLocation& origin,
             PathLocation& destination, GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>* mode_costing,
             const TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  costing_ = mode_costing[static_cast<uint32_t>(mode_)];

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  Init(origin.edges.front().projected, destination.edges.front().projected);
  float mindist = astarheuristic_.GetDistance(origin.edges.front().projected);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  uint32_t density = SetDestination(graphreader, destination);
  SetOrigin(graphreader, origin, destination);

  // Find shortest path
  uint32_t nc = 0;       // Count of iterations with no convergence
                         // towards destination
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " +
                     std::to_string(edgelabels_.size()));
      return { };
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    EdgeLabel pred = edgelabels_[predindex];
    if (destinations_.find(pred.edgeid()) != destinations_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return FormPath(predindex);
        }
      } else {
        return FormPath(predindex);
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > 500000) {
      LOG_ERROR("No convergence to destination after = " +
                           std::to_string(edgelabels_.size()));
      return {};
    }

    // Get the end node of the prior directed edge. Skip if tile not found
    // (can happen with regional data sets).
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Check access at the node
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing_->Allowed(nodeinfo)) {
      continue;
    }

    // Check if this tile has real-time speeds
    std::vector<uint8_t>& speeds = GetRealTimeSpeeds(node.tileid(), graphreader);

    // Expand from end node.
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count();
                i++, directededge++, ++edgeid) {
      // Disable upward transitions for traffic...for now only support traffic
      // for short routes since no shortcuts have traffic yet
      if (directededge->trans_up()) {
        continue;
      }

      // Skip if no access is allowed to this edge (based on costing method)
      if (!costing_->Allowed(directededge, pred, tile, edgeid)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      // TODO - want to add a traffic costing method in sif
      Cost edge_cost;
      Cost tc = costing_->TransitionCost(directededge, nodeinfo, pred);
      if (speeds.size() == 0 || speeds[edgeid.id()] == 0) {
        edge_cost = costing_->EdgeCost(directededge);
      } else {
        // Traffic exists for this edge
        float sec = directededge->length() * (kSecPerHour * 0.001f) /
                static_cast<float>(speeds[edgeid.id()]);
        edge_cost = { sec, sec };

        // For now reduce transition cost by half...thought is that traffic
        // will account for some of the transition cost
        tc.cost *= 0.5f;
        tc.secs *= 0.5f;
      }

      // Compute the cost to the end of this edge
      Cost newcost = pred.cost() + edge_cost + tc;

      // If this edge is a destination, subtract the partial/remainder cost
      // (cost from the dest. location to the end of the edge).
      auto p = destinations_.find(edgeid);
      if (p != destinations_.end()) {
        newcost -= p->second;
      }

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.set() == EdgeSet::kTemporary) {
        EdgeLabel& lab = edgelabels_[edgestatus.index()];
        if (newcost.cost <  lab.cost().cost) {
          float newsortcost = lab.sortcost() - (lab.cost().cost - newcost.cost);
          adjacencylist_->decrease(edgestatus.index(), newsortcost);
          lab.Update(predindex, newcost, newsortcost);
        }
        continue;
      }

      // If this is a destination edge the A* heuristic is 0. Otherwise the
      // sort cost (with A* heuristic) is found using the lat,lng at the
      // end node of the directed edge.
      float dist = 0.0f;
      float sortcost = newcost.cost;
      if (p == destinations_.end()) {
        const GraphTile* t2 = directededge->leaves_tile() ?
            graphreader.GetGraphTile(directededge->endnode()) : tile;
        if (t2 == nullptr) {
          continue;
        }
        sortcost += astarheuristic_.Get(
                    t2->node(directededge->endnode())->latlng(), dist);
      }

      // Add to the adjacency list and edge labels.
      uint32_t idx = edgelabels_.size();
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                          newcost, sortcost, dist, mode_, 0);
      edgestatus_->Set(edgeid, EdgeSet::kTemporary, idx);
      adjacencylist_->add(idx);
    }
  }
  return {};      // Should never get here
}

std::vector<uint8_t>& TrafficAlgorithm::GetRealTimeSpeeds(const uint32_t tileid,
                               GraphReader& graphreader) {
  // Check if this tile has real-time speeds
  auto rts = real_time_speeds_.find(tileid);
  if (rts == real_time_speeds_.end()) {
    // Try to load the speeds file
    std::ifstream rtsfile;
    std::string traffic_dir = graphreader.tile_dir() + "/traffic/";
    std::string fname = traffic_dir + std::to_string(tileid) + ".spd";
    rtsfile.open(fname, std::ios::binary | std::ios::in | std::ios::ate);
    if (rtsfile.is_open()) {
      uint32_t count = rtsfile.tellg();
      LOG_INFO("Load real time speeds: count = " + std::to_string(count));
      rtsfile.seekg(0, rtsfile.beg);
      std::vector<uint8_t> spds(count);
      rtsfile.read((char*)(&spds.front()), count);
      rtsfile.close();
      real_time_speeds_[tileid] = spds;
      return real_time_speeds_[tileid];
    } else {
      return empty_speeds_;
    }
  } else {
    return rts->second;
  }
}


}
}
