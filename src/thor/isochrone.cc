#include "thor/isochrone.h"
#include "baldr/datetime.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include <algorithm>
#include <iostream> // TODO remove if not needed
#include <map>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

// Method to get an operator Id from a map of operator strings vs. Id.
uint32_t GetOperatorId(const GraphTile* tile,
                       uint32_t routeid,
                       std::unordered_map<std::string, uint32_t>& operators) {
  const TransitRoute* transit_route = tile->GetTransitRoute(routeid);

  // Test if the transit operator changed
  if (transit_route && transit_route->op_by_onestop_id_offset()) {
    // Get the operator name and look up in the operators map
    std::string operator_name = tile->GetName(transit_route->op_by_onestop_id_offset());
    auto operator_itr = operators.find(operator_name);
    if (operator_itr == operators.end()) {
      // Operator not found - add to the map
      uint32_t id = operators.size() + 1;
      operators[operator_name] = id;
      return id;
    } else {
      return operator_itr->second;
    }
  }
  return 0;
}

} // namespace

namespace valhalla {
namespace thor {

constexpr uint32_t kBucketCount = 20000;
constexpr uint32_t kInitialEdgeLabelCount = 500000;

// Default constructor
Isochrone::Isochrone() : shape_interval_(50.0f) {
  Dijkstras();
}

// Destructor
Isochrone::~Isochrone() {
  Clear();
}

// Clear the temporary information generated during path construction.
void Isochrone::Clear() {
  // Clear the edge labels, edge status flags, and adjacency list
  // TODO - clear only the edge label set that was used?
  bdedgelabels_.clear();
  mmedgelabels_.clear();
  adjacencylist_.reset();
  edgestatus_.clear();
}

// Construct the isotile. Use a fixed grid size. Convert time in minutes to
// a max distance in meters based on an estimate of max average speed for
// the travel mode.
void Isochrone::ConstructIsoTile(
    const bool multimodal,
    const unsigned int max_minutes,
    google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations) {
  float max_distance;
  max_seconds_ = max_minutes * 60;
  if (multimodal) {
    max_distance = max_seconds_ * 70.0f * kMPHtoMetersPerSec;
  } else if (mode_ == TravelMode::kPedestrian) {
    max_distance = max_seconds_ * 5.0f * kMPHtoMetersPerSec;
  } else if (mode_ == TravelMode::kBicycle) {
    max_distance = max_seconds_ * 20.0f * kMPHtoMetersPerSec;
  } else {
    // A driving mode
    max_distance = max_seconds_ * 70.0f * kMPHtoMetersPerSec;
  }

  // Form bounding box that's just big enough to surround all of the locations.
  // Convert to PointLL
  PointLL center_ll(origin_locations.Get(0).ll().lng(), origin_locations.Get(0).ll().lat());
  AABB2<PointLL> loc_bounds(center_ll.lng(), center_ll.lat(), center_ll.lng(), center_ll.lat());

  for (const auto& origin : origin_locations) {
    PointLL loc(origin.ll().lng(), origin.ll().lat());
    loc_bounds.Expand(loc);
  }
  // Find the location closest to the center.
  PointLL bounds_center = loc_bounds.Center();
  float shortest_dist = center_ll.Distance(bounds_center);
  for (const auto& origin : origin_locations) {
    PointLL loc(origin.ll().lng(), origin.ll().lat());
    float current_dist = loc.Distance(bounds_center);
    if (current_dist < shortest_dist) {
      shortest_dist = current_dist;
      center_ll = loc;
    }
  }

  // Range of grids in latitude space
  float dlat = max_distance / kMetersPerDegreeLat;
  // Range of grids in longitude space
  float dlon = max_distance / DistanceApproximator::MetersPerLngDegree(center_ll.lat());

  // Optimize for 600 cells in latitude (slightly larger for multimodal).
  // Round off to nearest 0.001 degree. TODO - revisit min and max grid sizes
  float grid_size = (multimodal) ? dlat / 500.0f : dlat / 300.0f;
  if (grid_size < 0.001f) {
    grid_size = 0.001f;
  } else if (grid_size > 0.005f) {
    grid_size = 0.005f;
  } else {
    // Round to nearest 0.001
    int r = std::round(grid_size * 1000.0f);
    grid_size = static_cast<float>(r) * 0.001f;
  }

  // Set the shape interval in meters
  shape_interval_ = grid_size * kMetersPerDegreeLat * 0.25f;

  // Create expanded bounds from the bounded box around the locations.
  AABB2<PointLL> bounds(loc_bounds.minx() - dlon, loc_bounds.miny() - dlat, loc_bounds.maxx() + dlon,
                        loc_bounds.maxy() + dlat);

  // Create isotile (gridded data)
  isotile_.reset(new GriddedData<PointLL>(bounds, grid_size, max_minutes));

  // Find the center of the grid that the location lies within. Shift the
  // tilebounds so the location lies in the center of a tile.
  PointLL grid_center = isotile_->Center(isotile_->TileId(center_ll));
  PointLL shift(grid_center.lng() - center_ll.lng(), grid_center.lat() - center_ll.lat());
  isotile_->ShiftTileBounds(shift);

  // Test that the shift worked...TODO - remove later
  grid_center = isotile_->Center(isotile_->TileId(center_ll));
  if (std::abs(center_ll.lat() - grid_center.lat()) > 0.0001f ||
      std::abs(center_ll.lng() - grid_center.lng()) > 0.0001f) {
    LOG_INFO("Isochrone center location is not centered within a tile. Off by: " +
             std::to_string(center_ll.lat() - grid_center.lat()) + "," +
             std::to_string(center_ll.lng() - grid_center.lng()));
  }
}

// Initialize - create adjacency list, edgestatus support, and reserve
// edgelabels
template <typename label_container_t>
void Isochrone::Initialize(label_container_t& labels, const uint32_t bucketsize) {
  labels.reserve(kInitialEdgeLabelCount);

  // Set up lambda to get sort costs
  const auto edgecost = [&labels](const uint32_t label) { return labels[label].sortcost(); };

  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new DoubleBucketQueue(0.0f, range, bucketsize, edgecost));
  edgestatus_.clear();
}
template void
Isochrone::Initialize<decltype(Isochrone::bdedgelabels_)>(decltype(Isochrone::bdedgelabels_)&,
                                                          const uint32_t);
template void
Isochrone::Initialize<decltype(Isochrone::mmedgelabels_)>(decltype(Isochrone::mmedgelabels_)&,
                                                          const uint32_t);

// Initializes the time of the expansion if there is one
std::pair<uint64_t, uint32_t>
Isochrone::SetTime(google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                   const GraphId& node_id,
                   GraphReader& reader) {

  // No time for this expansion
  const auto& location = locations.Get(0);
  has_date_time_ = false;
  if (!location.has_date_time() || !node_id.Is_Valid())
    return {};

  // Set the timezone to be the timezone at the end node
  start_tz_index_ = GetTimezone(reader, node_id);
  if (start_tz_index_ == 0)
    LOG_ERROR("Could not get the timezone at the destination location");

  // Set route start time (seconds from epoch)
  auto start_time = DateTime::seconds_since_epoch(location.date_time(),
                                                  DateTime::get_tz_db().from_index(start_tz_index_));

  // Set seconds from beginning of the week
  auto start_seconds_of_week = DateTime::day_of_week(location.date_time()) * kSecondsPerDay +
                               DateTime::seconds_from_midnight(location.date_time());
  has_date_time_ = true;

  // loop over all locations setting the date time with timezone
  for (auto& location : locations) {
    // no time skip
    if (!location.has_date_time())
      continue;
    // find a node
    for (const auto& e : location.path_edges()) {
      // get the edge and then the end node
      GraphId edge_id(e.graph_id());
      const auto* tile = reader.GetGraphTile(edge_id);
      GraphId node_id = tile ? tile->directededge(edge_id)->endnode() : GraphId{};
      if (reader.GetGraphTile(node_id, tile)) {
        // if its current time  use that otherwise use the time provided
        const auto* node = tile->node(node_id);
        auto tz_index = DateTime::get_tz_db().from_index(node->timezone());
        auto date_time =
            location.date_time() == "current"
                ? DateTime::iso_date_time(tz_index)
                : DateTime::seconds_to_date(DateTime::seconds_since_epoch(location.date_time(),
                                                                          tz_index),
                                            tz_index);
        location.set_date_time(date_time);
        break;
      }
    }
  }

  // Hand back the start time and second of the week
  return {start_time, start_seconds_of_week};
}

// Compute iso-tile that we can use to generate isochrones.
std::shared_ptr<const GriddedData<PointLL>>
Isochrone::Compute(google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
                   const unsigned int max_minutes,
                   GraphReader& graphreader,
                   const std::shared_ptr<DynamicCost>* mode_costing,
                   const TravelMode mode) {

  // Initialize and create the isotile
  max_seconds_ = max_minutes * 60;
  ConstructIsoTile(false, max_minutes, origin_locations);
  uint32_t n = 0; // TODO What is/was this used for

  Dijkstras::Compute(origin_locations, graphreader, mode_costing, mode);

  return isotile_; // Should never get here
}

// Compute iso-tile that we can use to generate isochrones.
std::shared_ptr<const GriddedData<PointLL>>
Isochrone::ComputeReverse(google::protobuf::RepeatedPtrField<valhalla::Location>& dest_locations,
                          const unsigned int max_minutes,
                          GraphReader& graphreader,
                          const std::shared_ptr<DynamicCost>* mode_costing,
                          const TravelMode mode) {

  // Initialize and create the isotile
  max_seconds_ = max_minutes * 60;
  ConstructIsoTile(false, max_minutes, dest_locations);

  Dijkstras::ComputeReverse(dest_locations, graphreader, mode_costing, mode);

  return isotile_;
}

// Compute isochrone for mulit-modal route.
std::shared_ptr<const GriddedData<PointLL>>
Isochrone::ComputeMultiModal(google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
                             const unsigned int max_minutes,
                             GraphReader& graphreader,
                             const std::shared_ptr<DynamicCost>* mode_costing,
                             const TravelMode mode) {
  // Initialize and create the isotile
  max_seconds_ = max_minutes * 60;
  ConstructIsoTile(true, max_minutes, origin_locations);

  Dijkstras::ComputeMultiModal(origin_locations, graphreader, mode_costing, mode);

  return isotile_;
}

// Update the isotile
void Isochrone::UpdateIsoTile(const EdgeLabel& pred,
                              GraphReader& graphreader,
                              const PointLL& ll,
                              float secs0) {
  // Skip if the opposing edge has already been settled.
  const GraphTile* t2;
  GraphId opp = graphreader.GetOpposingEdgeId(pred.edgeid(), t2);
  EdgeStatusInfo es = edgestatus_.Get(opp);
  if (es.set() == EdgeSet::kPermanent) {
    return;
  }

  // Get the DirectedEdge because we'll need its shape
  const GraphTile* tile = graphreader.GetGraphTile(pred.edgeid().Tile_Base());
  const DirectedEdge* edge = tile->directededge(pred.edgeid());

  // Transit lines and ferries can't really be "reached" you really just
  // pass through those cells.
  if (edge->IsTransitLine() || edge->use() == Use::kFerry) {
    return;
  }

  // Get the time at the end node of the predecessor
  // TODO - do we need partial shape from origin location to end of edge?
  float secs1 = pred.cost().secs;

  // For short edges just mark the segment between the 2 nodes of the edge. This
  // avoid getting the shape for short edges.
  if (edge->length() < shape_interval_ * 1.5f) {
    // Mark tiles that intersect the segment. Optimize this to avoid calling the Intersect
    // method unless more than 2 tiles are crossed by the segment.
    PointLL ll0 = tile->get_node_ll(t2->directededge(opp)->endnode());
    auto tile1 = isotile_->TileId(ll0);
    auto tile2 = isotile_->TileId(ll);
    if (tile1 == tile2) {
      isotile_->SetIfLessThan(tile1, secs1 * kMinPerSec);
    } else if (isotile_->AreNeighbors(tile1, tile2)) {
      // If tile 2 is directly east, west, north, or south of tile 1 then the
      // segment will not intersect any other tiles other than tile1 and tile2.
      isotile_->SetIfLessThan(tile1, secs1 * kMinPerSec);
      isotile_->SetIfLessThan(tile2, secs1 * kMinPerSec);
    } else {
      // Find intersecting tiles (using a Bresenham method)
      auto tiles = isotile_->Intersect(std::list<PointLL>{ll0, ll});
      for (auto t : tiles) {
        isotile_->SetIfLessThan(t.first, secs1 * kMinPerSec);
      }
    }
    return;
  }

  // Get the shape and make sure shape is forward direction. Resample it to
  // the shape interval to get regular spacing. Use the faster resample method.
  // This does not use spherical interpolation - so it is not as accurate but
  // interpolation is over short distances so accuracy should be fine.
  auto shape = tile->edgeinfo(edge->edgeinfo_offset()).shape();
  auto resampled = resample_polyline(shape, edge->length(), shape_interval_);
  if (!edge->forward()) {
    std::reverse(resampled.begin(), resampled.end());
  }

  // Mark grid cells along the shape if time is less than what is
  // already populated. Get intersection of tiles along each segment
  // (just use a bounding box around the segment) so this doesn't miss
  // shape that crosses tile corners
  float minutes = secs0 * kMinPerSec;
  float delta = ((secs1 - secs0) / (resampled.size() - 1)) * kMinPerSec;
  auto itr1 = resampled.begin();
  for (auto itr2 = itr1 + 1; itr2 < resampled.end(); itr1++, itr2++) {
    minutes += delta;

    // Mark tiles that intersect the segment. Optimize this to avoid calling the Intersect
    // method unless more than 2 tiles are crossed by the segment.
    auto tile1 = isotile_->TileId(*itr1);
    auto tile2 = isotile_->TileId(*itr2);
    if (tile1 == tile2) {
      isotile_->SetIfLessThan(tile1, minutes);
    } else if (isotile_->AreNeighbors(tile1, tile2)) {
      // If tile 2 is directly east, west, north, or south of tile 1 then the
      // segment will not intersect any other tiles other than tile1 and tile2.
      isotile_->SetIfLessThan(tile1, minutes);
      isotile_->SetIfLessThan(tile2, minutes);
    } else {
      // Find intersecting tiles (using a Bresenham method)
      auto tiles = isotile_->Intersect(std::list<PointLL>{*itr1, *itr2});
      for (auto t : tiles) {
        isotile_->SetIfLessThan(t.first, minutes);
      }
    }
  }
}

// Add edge(s) at each origin to the adjacency list
void Isochrone::SetOriginLocations(GraphReader& graphreader,
                                   google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
                                   const std::shared_ptr<DynamicCost>& costing) {
  // Add edges for each location to the adjacency list
  for (auto& location : locations) {
    // NOTE ISOCHRONE SPECIFIC
    PointLL ll(location.ll().lng(), location.ll().lat());
    // Set time at the lat, lon grid to 0
    isotile_->Set(ll, 0);
    // ENDNOTE

    // Only skip inbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(location.path_edges().begin(), location.path_edges().end(),
                  [&has_other_edges](const valhalla::Location::PathEdge& e) {
                    has_other_edges = has_other_edges || !e.end_node();
                  });

    // Iterate through edges and add to adjacency list
    for (const auto& edge : (location.path_edges())) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (has_other_edges && edge.end_node()) {
        continue;
      }

      // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge
      const GraphTile* tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Get the opposing directed edge, continue if we cannot get it
      const GraphTile* opp_tile = nullptr;
      GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid, opp_tile);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }
      const DirectedEdge* opp_dir_edge = opp_tile->directededge(edgeid);

      // Get cost
      Cost cost = costing->EdgeCost(directededge, tile) * (1.0f - edge.percent_along());

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: high edge scores cause issues as there is code to limit cost so
      // that large penalties (e.g., ferries) are excluded.
      cost.cost += edge.distance() * 0.005f;

      // Construct the edge label. Set the predecessor edge index to invalid
      // to indicate the origin of the path.
      uint32_t idx = bdedgelabels_.size();
      uint32_t path_distance =
          static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()));
      // TODO Do we care about time restrictions on origin edges?
      const bool has_time_restrictions = false;
      bdedgelabels_.emplace_back(kInvalidLabel, edgeid, opp_edge_id, directededge, cost, cost.cost,
                                 0., mode_, Cost{}, path_distance, false, has_time_restrictions);
      // Set the origin flag
      bdedgelabels_.back().set_origin();

      // Add EdgeLabel to the adjacency list
      adjacencylist_->add(idx);
      edgestatus_.Set(edgeid, EdgeSet::kTemporary, idx, tile);
    }
  }
}

// Add destination edges to the reverse path adjacency list.
void Isochrone::SetDestinationLocations(
    GraphReader& graphreader,
    google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    const std::shared_ptr<DynamicCost>& costing) {
  // Add edges for each location to the adjacency list
  for (auto& location : locations) {
    // NOTE ISOCHRONE SPECIFIC
    PointLL ll(location.ll().lng(), location.ll().lat());
    // Set time at the lat, lon grid to 0
    isotile_->Set(ll, 0);
    // ENDNOTE

    // Only skip outbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(location.path_edges().begin(), location.path_edges().end(),
                  [&has_other_edges](const valhalla::Location::PathEdge& e) {
                    has_other_edges = has_other_edges || !e.begin_node();
                  });

    // Iterate through edges and add to adjacency list
    for (const auto& edge : (location.path_edges())) {
      // If the destination is at a node, skip any outbound edges (so any
      // opposing inbound edges are not considered)
      if (has_other_edges && edge.begin_node()) {
        continue;
      }

      // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsDestinationEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge
      const GraphTile* tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Get the opposing directed edge, continue if we cannot get it
      const GraphTile* opp_tile = nullptr;
      GraphId opp_edge_id = graphreader.GetOpposingEdgeId(edgeid, opp_tile);
      if (!opp_edge_id.Is_Valid()) {
        continue;
      }
      const DirectedEdge* opp_dir_edge = opp_tile->directededge(edgeid);

      // Get the cost
      Cost cost = costing->EdgeCost(directededge, tile) * edge.percent_along();

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: high edge scores cause issues as there is code to limit cost so
      // that large penalties (e.g., ferries) are excluded.
      cost.cost += edge.distance() * 0.005f;

      // Add EdgeLabel to the adjacency list. Set the predecessor edge index
      // to invalid to indicate the origin of the path. Make sure the opposing
      // edge (edgeid) is set.
      uint32_t idx = bdedgelabels_.size();
      uint32_t path_distance = static_cast<uint32_t>(directededge->length() * edge.percent_along());
      // TODO Do we care about time restrictions at destination edges?
      const bool has_time_restrictions = false;
      bdedgelabels_.emplace_back(kInvalidLabel, opp_edge_id, edgeid, opp_dir_edge, cost, cost.cost,
                                 0., mode_, Cost{}, path_distance, false, has_time_restrictions);
      adjacencylist_->add(idx);
      edgestatus_.Set(opp_edge_id, EdgeSet::kTemporary, idx, graphreader.GetGraphTile(opp_edge_id));
    }
  }
}

// Add edge(s) at each origin to the adjacency list
void Isochrone::SetOriginLocationsMM(
    GraphReader& graphreader,
    google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
    const std::shared_ptr<DynamicCost>& costing) {
  // Add edges for each location to the adjacency list
  for (auto& origin : origin_locations) {
    // NOTE ISOCHRONE SPECIFIC
    PointLL ll(origin.ll().lng(), origin.ll().lat());
    // Set time at the origin lat, lon grid to 0
    isotile_->Set(ll, 0);
    // ENDNOTE

    // Only skip inbound edges if we have other options
    bool has_other_edges = false;
    std::for_each(origin.path_edges().begin(), origin.path_edges().end(),
                  [&has_other_edges](const valhalla::Location::PathEdge& e) {
                    has_other_edges = has_other_edges || !e.end_node();
                  });

    // Iterate through edges and add to adjacency list
    for (const auto& edge : (origin.path_edges())) {
      // If origin is at a node - skip any inbound edge (dist = 1)
      if (has_other_edges && edge.end_node()) {
        continue;
      }

      // Disallow any user avoid edges if the avoid location is ahead of the origin along the edge
      GraphId edgeid(edge.graph_id());
      if (costing_->AvoidAsOriginEdge(edgeid, edge.percent_along())) {
        continue;
      }

      // Get the directed edge
      const GraphTile* tile = graphreader.GetGraphTile(edgeid);
      const DirectedEdge* directededge = tile->directededge(edgeid);

      // Get the tile at the end node. Skip if tile not found as we won't be
      // able to expand from this origin edge.
      const GraphTile* endtile = graphreader.GetGraphTile(directededge->endnode());
      if (endtile == nullptr) {
        continue;
      }

      // Get cost
      Cost cost = costing->EdgeCost(directededge, endtile) * (1.0f - edge.percent_along());

      // We need to penalize this location based on its score (distance in meters from input)
      // We assume the slowest speed you could travel to cover that distance to start/end the route
      // TODO: high edge scores cause issues as there is code to limit cost so
      // that large penalties (e.g., ferries) are excluded.
      cost.cost += edge.distance() * 0.005f;

      // Add EdgeLabel to the adjacency list (but do not set its status).
      // Set the predecessor edge index to invalid to indicate the origin
      // of the path.
      uint32_t idx = mmedgelabels_.size();
      uint32_t d = static_cast<uint32_t>(directededge->length() * (1.0f - edge.percent_along()));
      // TODO Do we care about time restrictions at origin edges?
      bool has_time_restrictions = false;
      MMEdgeLabel edge_label(kInvalidLabel, edgeid, directededge, cost, cost.cost, 0.0f, mode_, d, 0,
                             GraphId(), 0, 0, false, has_time_restrictions);

      // Set the origin flag
      edge_label.set_origin();

      // Add EdgeLabel to the adjacency list
      mmedgelabels_.push_back(std::move(edge_label));
      adjacencylist_->add(idx);
    }
  }
}

// Virtual function called when expanding a node
//
// Children can implement this to customize behaviour
void Isochrone::ExpandingNode(baldr::GraphReader& graphreader,
                              const sif::EdgeLabel& pred,
                              const ExpandingNodeMiscInfo& info) {

  const GraphTile* tile = graphreader.GetGraphTile(pred.endnode());
  if (tile == nullptr) {
    return;
  }
  const PointLL ll = tile->get_node_ll(pred.endnode());

  if (info.edge_type == InfoEdgeType::origin || info.edge_type == InfoEdgeType::destination) {
    // Use pred to get endnode and it's location

    float time = 0.; // Set time at the lat, lon grid to 0
    isotile_->Set(ll, time);
  } else {
    // Update the isotile
    uint32_t idx = pred.predecessor();

    float secs0;

    // TODO this case is ugly, can it be simplified?
    if (info.routing_type == InfoRoutingType::multi_modal) {
      secs0 = (idx == kInvalidLabel) ? 0 : mmedgelabels_[idx].cost().secs;
    } else {
      secs0 = (idx == kInvalidLabel) ? 0 : bdedgelabels_[idx].cost().secs;
    }
    UpdateIsoTile(pred, graphreader, ll, secs0);
  }
};

RouteCallbackRecommendedAction
Isochrone::RouteCallbackDecideAction(baldr::GraphReader& graphreader,
                                     const sif::EdgeLabel& pred,
                                     const InfoRoutingType route_type) {
  if (route_type == InfoRoutingType::multi_modal) {
    // Skip edges with large penalties (e.g. ferries?)
    if (pred.cost().cost > max_seconds_ * 2) {
      return RouteCallbackRecommendedAction::skip_expansion;
    }
  }
  if (pred.cost().secs > max_seconds_ || pred.cost().cost > max_seconds_ * 4) {
    LOG_DEBUG("Exceed time interval: n = " + std::to_string(n));
    return RouteCallbackRecommendedAction::stop_expansion;
  }
  return RouteCallbackRecommendedAction::no_action;
};

} // namespace thor
} // namespace valhalla
