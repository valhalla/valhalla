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
uint32_t GetOperatorId(const std::shared_ptr<const GraphTile>& tile,
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

constexpr uint32_t kInitialEdgeLabelCount = 500000;

// Default constructor
Isochrone::Isochrone() : Dijkstras(), shape_interval_(50.0f) {
}

// Construct the isotile. Use a fixed grid size. Convert time in minutes to
// a max distance in meters based on an estimate of max average speed for
// the travel mode.
void Isochrone::ConstructIsoTile(
    const bool multimodal,
    const unsigned int max_minutes,
    const google::protobuf::RepeatedPtrField<valhalla::Location>& locations,
    const sif::TravelMode mode) {
  float max_distance;
  max_seconds_ = max_minutes * 60;
  if (multimodal) {
    max_distance = max_seconds_ * 70.0f * kMPHtoMetersPerSec;
  } else if (mode == TravelMode::kPedestrian) {
    max_distance = max_seconds_ * 5.0f * kMPHtoMetersPerSec;
  } else if (mode == TravelMode::kBicycle) {
    max_distance = max_seconds_ * 20.0f * kMPHtoMetersPerSec;
  } else {
    // A driving mode
    max_distance = max_seconds_ * 70.0f * kMPHtoMetersPerSec;
  }

  // Form bounding box that's just big enough to surround all of the locations.
  // Convert to PointLL
  PointLL center_ll(locations.Get(0).ll().lng(), locations.Get(0).ll().lat());
  AABB2<PointLL> loc_bounds(center_ll.lng(), center_ll.lat(), center_ll.lng(), center_ll.lat());

  for (const auto& location : locations) {
    PointLL ll(location.ll().lng(), location.ll().lat());
    loc_bounds.Expand(ll);
  }
  // Find the location closest to the center.
  PointLL bounds_center = loc_bounds.Center();
  float shortest_dist = center_ll.Distance(bounds_center);
  for (const auto& location : locations) {
    PointLL ll(location.ll().lng(), location.ll().lat());
    float current_dist = ll.Distance(bounds_center);
    if (current_dist < shortest_dist) {
      shortest_dist = current_dist;
      center_ll = ll;
    }
  }

  // Range of grids in latitude space
  float dlat = max_distance / kMetersPerDegreeLat;
  // Range of grids in longitude space
  float dlon = max_distance / DistanceApproximator<PointLL>::MetersPerLngDegree(center_ll.lat());

  // Optimize for 600 cells in latitude (slightly larger for multimodal).
  // Round off to nearest 0.001 degree. TODO - revisit min and max grid sizes
  float grid_size = multimodal ? dlat / 500.0f : dlat / 300.0f;
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

  // initialize the time at these locations
  for (const auto& location : locations) {
    PointLL ll(location.ll().lng(), location.ll().lat());
    isotile_->Set(ll, 0);
  }
}

// Compute iso-tile that we can use to generate isochrones.
std::shared_ptr<const GriddedData<PointLL>>
Isochrone::Compute(google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
                   const unsigned int max_minutes,
                   GraphReader& graphreader,
                   const sif::mode_costing_t& mode_costing,
                   const TravelMode mode) {
  // Initialize and create the isotile
  ConstructIsoTile(false, max_minutes, origin_locations, mode);
  // Compute the expansion
  Dijkstras::Compute(origin_locations, graphreader, mode_costing, mode);
  return isotile_;
}

// Compute iso-tile that we can use to generate isochrones.
std::shared_ptr<const GriddedData<PointLL>>
Isochrone::ComputeReverse(google::protobuf::RepeatedPtrField<valhalla::Location>& dest_locations,
                          const unsigned int max_minutes,
                          GraphReader& graphreader,
                          const sif::mode_costing_t& mode_costing,
                          const TravelMode mode) {

  // Initialize and create the isotile
  ConstructIsoTile(false, max_minutes, dest_locations, mode);
  // Compute the expansion
  Dijkstras::ComputeReverse(dest_locations, graphreader, mode_costing, mode);
  return isotile_;
}

// Compute isochrone for mulit-modal route.
std::shared_ptr<const GriddedData<PointLL>>
Isochrone::ComputeMultiModal(google::protobuf::RepeatedPtrField<valhalla::Location>& origin_locations,
                             const unsigned int max_minutes,
                             GraphReader& graphreader,
                             const sif::mode_costing_t& mode_costing,
                             const TravelMode mode) {
  // Initialize and create the isotile
  ConstructIsoTile(true, max_minutes, origin_locations, mode);
  // Compute the expansion
  Dijkstras::ComputeMultiModal(origin_locations, graphreader, mode_costing, mode);
  return isotile_;
}

// Update the isotile
void Isochrone::UpdateIsoTile(const EdgeLabel& pred,
                              GraphReader& graphreader,
                              const PointLL& ll,
                              float secs0) {
  // Skip if the opposing edge has already been settled.
  std::shared_ptr<const GraphTile> t2;
  GraphId opp = graphreader.GetOpposingEdgeId(pred.edgeid(), t2);
  EdgeStatusInfo es = edgestatus_.Get(opp);
  if (es.set() == EdgeSet::kPermanent) {
    return;
  }

  // Get the DirectedEdge because we'll need its shape
  std::shared_ptr<const GraphTile> tile = graphreader.GetGraphTile(pred.edgeid().Tile_Base());
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
      for (const auto& t : tiles) {
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
      for (const auto& t : tiles) {
        isotile_->SetIfLessThan(t.first, minutes);
      }
    }
  }
}

// here we mark the cells of the isochrone along the edge we just reached up to its end node
void Isochrone::ExpandingNode(baldr::GraphReader& graphreader,
                              std::shared_ptr<const baldr::GraphTile> tile,
                              const baldr::NodeInfo* node,
                              const sif::EdgeLabel& current,
                              const sif::EdgeLabel* previous) {
  // Update the isotile
  float secs0 = previous ? previous->cost().secs : 0;
  UpdateIsoTile(current, graphreader, node->latlng(tile->header()->base_ll()), secs0);
}

ExpansionRecommendation Isochrone::ShouldExpand(baldr::GraphReader& /*graphreader*/,
                                                const sif::EdgeLabel& pred,
                                                const InfoRoutingType route_type) {
  if (route_type == InfoRoutingType::multi_modal) {
    // Skip edges with large penalties (e.g. ferries?), MMCompute function will skip expanding this
    // label
    if (pred.cost().cost > max_seconds_ * 2) {
      return ExpansionRecommendation::prune_expansion;
    }
  }
  // Continue if the time interval has been met. This bus or rail line goes beyond the max
  // but need to consider others so we just continue here. Tells MMExpand function to skip
  // updating or pushing the label back
  if (pred.cost().secs > max_seconds_ || pred.cost().cost > max_seconds_ * 4) {
    return ExpansionRecommendation::stop_expansion;
  }
  return ExpansionRecommendation::continue_expansion;
};

void Isochrone::GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const {
  bucket_count = 20000;
  edge_label_reservation = 500000;
}

} // namespace thor
} // namespace valhalla
