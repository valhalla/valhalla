#include <iostream> // TODO remove if not needed
#include <map>
#include <algorithm>
#include "thor/isochrone.h"
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;


// TODO: make a class that extends std::exception, with messages and
// error codes and return the appropriate error codes

namespace valhalla {
namespace thor {

constexpr uint32_t kBucketCount = 20000;
constexpr uint64_t kInitialEdgeLabelCount = 500000;

// Default constructor
Isochrone::Isochrone()
    : tile_creation_date_(0),
      shape_interval_(50.0f),
      mode_(TravelMode::kDrive),
      adjacencylist_(nullptr),
      edgestatus_(nullptr) {
}

// Destructor
Isochrone::~Isochrone() {
  Clear();
}

// Clear the temporary information generated during path construction.
void Isochrone::Clear() {
  // Clear the edge labels
  edgelabels_.clear();

  // Clear elements from the adjacency list
  adjacencylist_.reset();

  // Clear the edge status flags
  edgestatus_.reset();
}

// Compute an isochrone.
const std::shared_ptr<GriddedData<PointLL> > Isochrone::Compute(
             std::vector<PathLocation>& origin_locations,
             const uint32_t max_seconds,
             GraphReader& graphreader,
             const std::shared_ptr<DynamicCost>* mode_costing,
             const TravelMode mode) {
  // Set the mode and costing
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];

  // Initialize - create adjacency list, edgestatus support, and reserve
  // edgelabels
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new AdjacencyList(0.0f, range, bucketsize));
  edgestatus_.reset(new EdgeStatus());
  edgelabels_.reserve(kInitialEdgeLabelCount);

  // Construct the isotile. Use a grid size based on travel mode.
  // Convert time in seconds to a max distance in meters based on an
  // estimate of max speed for the travel mode.
  float grid_size, max_distance;
  if (mode_ == TravelMode::kPedestrian) {
    grid_size = 200.0f;
    max_distance = max_seconds * 5.0f * 0.44704f;
  } else if (mode_ == TravelMode::kBicycle) {
    grid_size = 200.0f;
    max_distance = max_seconds * 20.0f * 0.44704f;
  } else {
    // A driving mode
    grid_size = 400.0f;
    max_distance = max_seconds * 70.0f * 0.44704f;
  }

  // Form grid for isotiles. Convert grid size to degrees.
  grid_size /= kMetersPerDegreeLat;
  float lat = origin_locations[0].latlng_.lat();
  float delta_lat = max_distance / kMetersPerDegreeLat;
  float delta_lon = max_distance / DistanceApproximator::MetersPerLngDegree(lat);
  AABB2<PointLL> bounds(10000.0f, 10000.0f, -10000.0f, -10000.0f);
  for (const auto& loc : origin_locations) {
    PointLL center = loc.latlng_;
    AABB2<PointLL> bbox(PointLL(center.lng() - delta_lon, center.lat() - delta_lat),
                        PointLL(center.lng() + delta_lon, center.lat() + delta_lat));
    bounds.Expand(bbox);
  }
  isotile_.reset(new GriddedData<PointLL>(bounds, grid_size, max_seconds + 300));

  // Set the origin locations
  SetOriginLocations(graphreader, origin_locations, costing);

  // Compute the isotile
  uint32_t n = 0;
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->Remove(edgelabels_);
    if (predindex == kInvalidLabel) {
      return isotile_;
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    EdgeLabel pred = edgelabels_[predindex];

    // Mark the edge as permanently labeled.
    edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);

    // Get the end node of the prior directed edge. Skip if tile not found
    // (can happen with regional data sets).
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Get the nodeinfo and update the isotile
    const NodeInfo* nodeinfo = tile->node(node);
    UpdateIsoTile(pred, graphreader, nodeinfo->latlng());
    n++;

    // Return after the time interval has been met
    if (pred.cost().secs > max_seconds) {
      LOG_INFO("Exceed time interval: n = " + std::to_string(n));
      return isotile_;
    }

    // Check access at the node
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Expand from end node.
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, edgeid++) {
      // Do not transition to upper hierarchies
      if (directededge->trans_up()) {
        continue;
      }

      // Skip if no access is allowed to this edge (based on costing method)
      if (!costing->Allowed(directededge, pred, tile, edgeid)) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Compute the cost to the end of this edge
      Cost newcost = pred.cost() +
			     costing->EdgeCost(directededge, nodeinfo->density()) +
			     costing->TransitionCost(directededge, nodeinfo, pred);

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change)
      if (edgestatus.set() == EdgeSet::kTemporary) {
        CheckIfLowerCostPath(edgestatus.status.index, predindex, newcost);
        continue;
      }

      // Add to the adjacency list and edge labels.
      AddToAdjacencyList(edgeid, newcost.cost);
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, newcost.cost, 0.0f, directededge->restrictions(),
                    directededge->opp_local_idx(), mode_, 0);
    }
  }
  return isotile_;      // Should never get here
}

// Update the isotile
void Isochrone::UpdateIsoTile(const EdgeLabel& pred, GraphReader& graphreader,
                              const PointLL& ll) {
  // Get time at the end node of the predecessor
  float secs1 = pred.cost().secs;

  // Get the time at the end node of the predecessor
  float secs0;
  uint32_t predindex = pred.predecessor();
  if (predindex == kInvalidLabel) {
    //TODO - do we need partial shape from origin location to end of edge?
    secs0 = 0;
  } else {
    secs0 = edgelabels_[predindex].cost().secs;
  }

  // Get the directed edge. Mark grid cells along the shape if time is
  // less than what is already populated
  const GraphTile* tile = graphreader.GetGraphTile(pred.edgeid().Tile_Base());
  const DirectedEdge* edge = tile->directededge(pred.edgeid());
  float length = edge->length();
  if (length < shape_interval_) {
    isotile_->SetIfLessThan(ll, secs1);
  } else {
    float delta = (shape_interval_ * (secs1 - secs0)) / length;
    auto shape = tile->edgeinfo(edge->edgeinfo_offset())->shape();
    if (!edge->forward()) {
      std::reverse(shape.begin(), shape.end());
    }
    float secs = secs0;
    auto resampled = resample_spherical_polyline(shape, shape_interval_);
    for (auto itr = resampled.begin() + 1; itr < resampled.end() - 1; itr++) {
      secs += delta;
      isotile_->SetIfLessThan(*itr, secs);
    }
    isotile_->SetIfLessThan(ll, secs1);
  }
}

// Convenience method to add an edge to the adjacency list and temporarily
// label it.
void Isochrone::AddToAdjacencyList(const GraphId& edgeid,
                                   const float sortcost) {
  uint32_t idx = edgelabels_.size();
  adjacencylist_->Add(idx, sortcost);
  edgestatus_->Set(edgeid, EdgeSet::kTemporary, idx);
}

// Check if edge is temporarily labeled and this path has less cost. If
// less cost the predecessor is updated and the sort cost is decremented
// by the difference in real cost (A* heuristic doesn't change)
void Isochrone::CheckIfLowerCostPath(const uint32_t idx,
                                     const uint32_t predindex,
                                     const Cost& newcost) {
  float dc = edgelabels_[idx].cost().cost - newcost.cost;
  if (dc > 0) {
    float oldsortcost = edgelabels_[idx].sortcost();
    float newsortcost = oldsortcost - dc;
    edgelabels_[idx].Update(predindex, newcost, newsortcost);
    adjacencylist_->DecreaseCost(idx, newsortcost, oldsortcost);
  }
}

// Add edge(s) at each origin to the adjacency list
void Isochrone::SetOriginLocations(GraphReader& graphreader,
                 const std::vector<PathLocation>& origin_locations,
                 const std::shared_ptr<DynamicCost>& costing) {
  // Add edges for each location to the adjacency list
  for (const auto& origin : origin_locations) {
    // Set time at the origin lat, lon grid to 0
    isotile_->Set(origin.latlng_, 0);

    // Iterate through edges and add to adjacency list
    const NodeInfo* nodeinfo = nullptr;
    for (const auto& edge : (origin.edges)) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (edge.end_node()) {
        continue;
      }

      // Get the directed edge
      GraphId edgeid = edge.id;
      const GraphTile* tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Set the tile creation date
      tile_creation_date_ = tile->header()->date_created();

      // Get the tile at the end node. Skip if tile not found as we won't be
      // able to expand from this origin edge.
      const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
      if (endtile == nullptr) {
        continue;
      }

      // Get cost
      nodeinfo = endtile->node(directededge->endnode());
      Cost cost = costing->EdgeCost(directededge,
                      graphreader.GetEdgeDensity(edge.id)) * (1.0f - edge.dist);

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path.
      uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.dist));
      adjacencylist_->Add(edgelabels_.size(), cost.cost);
      EdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost,
              cost.cost, 0.0f, directededge->restrictions(),
              directededge->opp_local_idx(), mode_, d);
      edge_label.set_origin();

      // Set the origin flag
      edgelabels_.push_back(std::move(edge_label));
    }
  }
}

}
}
