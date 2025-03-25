#include "thor/isochrone.h"
#include "baldr/datetime.h"
#include "midgard/distanceapproximator.h"
#include "midgard/logging.h"
#include <algorithm>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

constexpr float METRIC_PADDING = 10.f;

template <typename PrecisionT>
std::vector<GeoPoint<PrecisionT>> OriginEdgeShape(const std::vector<GeoPoint<PrecisionT>>& pts,
                                                  double distance_along) {
  // just the endpoint really
  if (distance_along == 0)
    return {pts.back(), pts.back()};

  // consume shape until we reach the desired distance
  double suffix_len = 0;
  for (auto from = std::next(pts.rbegin()), to = pts.rbegin(); from != pts.rend(); ++from, ++to) {
    // add whatever this segment of shape contributes to the overall distance
    PrecisionT len = from->Distance(*to);
    suffix_len += len;

    // we have enough distance now, lets find the exact stopping point along the geom
    if (suffix_len >= distance_along) {
      auto interpolated = from->PointAlongSegment(*to, (suffix_len - distance_along) / len);
      std::vector<GeoPoint<PrecisionT>> res(pts.rbegin(), from);
      res.push_back(interpolated);
      std::reverse(res.begin(), res.end());
      return res;
    }
  }

  // we got through the whole shape didnt reach the distance, floating point noise probably
  return pts;
}

} // namespace

namespace valhalla {
namespace thor {

// Default constructor
Isochrone::Isochrone(const boost::property_tree::ptree& config)
    : Dijkstras(config), shape_interval_(50.0f) {
}

// Construct the isotile. Use a fixed grid size. Convert time in minutes to
// a max distance in meters based on an estimate of max average speed for
// the travel mode.
void Isochrone::ConstructIsoTile(const bool multimodal,
                                 const valhalla::Api& api,
                                 const sif::travel_mode_t mode) {

  // Extend the times in the 2-D grid to be 10 minutes beyond the highest contour time.
  // Cost (including penalties) is used when adding to the adjacency list but the elapsed
  // time in seconds is used when terminating the search. The + 10 minutes adds a buffer for edges
  // where there has been a higher cost that might still be marked in the isochrone
  auto max_time_itr =
      std::max_element(api.options().contours().begin(), api.options().contours().end(),
                       [](const auto& a, const auto& b) {
                         return (!a.has_time_case() && b.has_time_case()) ||
                                (a.has_time_case() && b.has_time_case() && a.time() < b.time());
                       });
  bool has_time = max_time_itr->has_time_case();
  auto max_minutes =
      has_time ? max_time_itr->time() + METRIC_PADDING : std::numeric_limits<float>::min();
  auto max_dist_itr =
      std::max_element(api.options().contours().begin(), api.options().contours().end(),
                       [](const auto& a, const auto& b) {
                         return (!a.has_distance_case() && b.has_distance_case()) ||
                                (a.has_distance_case() && b.has_distance_case() &&
                                 a.distance() < b.distance());
                       });
  bool has_distance = max_dist_itr->has_distance_case();
  auto max_km =
      has_distance ? max_dist_itr->distance() + METRIC_PADDING : std::numeric_limits<float>::min();

  max_seconds_ = has_time ? max_minutes * kSecPerMinute : max_minutes;
  max_meters_ = has_distance ? max_km * kMetersPerKm : max_km;
  float max_distance;
  if (multimodal) {
    max_distance = max_seconds_ * 70.0f * kMPHtoMetersPerSec;
  } else if (mode == travel_mode_t::kPedestrian) {
    max_distance = max_seconds_ * 5.0f * kMPHtoMetersPerSec;
  } else if (mode == travel_mode_t::kBicycle) {
    max_distance = max_seconds_ * 20.0f * kMPHtoMetersPerSec;
  } else {
    // A driving mode
    max_distance = max_seconds_ * 70.0f * kMPHtoMetersPerSec;
  }
  // Either the user-specified or estimated max distance
  max_distance = std::max(max_distance, max_meters_);

  // Form bounding box that's just big enough to surround all of the locations.
  // Convert to PointLL
  PointLL center_ll(api.options().locations(0).ll().lng(), api.options().locations(0).ll().lat());
  AABB2<PointLL> loc_bounds(center_ll.lng(), center_ll.lat(), center_ll.lng(), center_ll.lat());

  for (const auto& location : api.options().locations()) {
    PointLL ll(location.ll().lng(), location.ll().lat());
    loc_bounds.Expand(ll);
  }
  // Find the location closest to the center.
  PointLL bounds_center = loc_bounds.Center();
  float shortest_dist = center_ll.Distance(bounds_center);
  for (const auto& location : api.options().locations()) {
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
  isotile_.reset(new GriddedData<2>(bounds, grid_size, {max_minutes, max_km}));

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
  for (const auto& location : api.options().locations()) {
    auto tile_id = isotile_->TileId({location.ll().lng(), location.ll().lat()});
    isotile_->SetIfLessThan(tile_id, {has_time ? 0.0f : max_minutes, has_distance ? 0.0f : max_km});
  }
}

// Compute iso-tile that we can use to generate isochrones.
std::shared_ptr<const GriddedData<2>> Isochrone::Expand(const ExpansionType& expansion_type,
                                                        Api& api,
                                                        GraphReader& reader,
                                                        const sif::mode_costing_t& mode_costing,
                                                        const travel_mode_t mode) {
  // Initialize and create the isotile
  ConstructIsoTile(expansion_type == ExpansionType::multimodal, api, mode);
  // Compute the expansion
  Dijkstras::Expand(expansion_type, api, reader, mode_costing, mode);
  return isotile_;
}

void Isochrone::UpdateIsoTileAlongSegment(const midgard::PointLL& from,
                                          const midgard::PointLL& to,
                                          float seconds,
                                          float meters) {
  float minutes = seconds * kMinPerSec;
  float km = meters * kKmPerMeter;
  // Mark tiles that intersect the segment. Optimize this to avoid calling the Intersect
  // method unless more than 2 tiles are crossed by the segment.
  auto tile1 = isotile_->TileId(from);
  auto tile2 = isotile_->TileId(to);
  if (tile1 == tile2) {
    isotile_->SetIfLessThan(tile1, {minutes, km});
  } else if (isotile_->AreNeighbors(tile1, tile2)) {
    // If tile 2 is directly east, west, north, or south of tile 1 then the
    // segment will not intersect any other tiles other than tile1 and tile2.
    isotile_->SetIfLessThan(tile1, {minutes, km});
    isotile_->SetIfLessThan(tile2, {minutes, km});
  } else {
    // Find intersecting tiles (using a Bresenham method)
    auto tiles = isotile_->Intersect(std::list<PointLL>{from, to});
    for (const auto& t : tiles) {
      isotile_->SetIfLessThan(t.first, {minutes, km});
    }
  }
}

// Update the isotile
void Isochrone::UpdateIsoTile(const EdgeLabel& pred,
                              GraphReader& graphreader,
                              const PointLL& ll,
                              float secs0,
                              float dist0) {
  // Skip if the opposing edge has already been settled.
  graph_tile_ptr t2;
  GraphId opp = graphreader.GetOpposingEdgeId(pred.edgeid(), t2);
  EdgeStatusInfo es = edgestatus_.Get(opp);
  // process origin edge even if its opposite edge is permanent
  if (es.set() == EdgeSet::kPermanent && !pred.origin()) {
    return;
  }

  // Get the DirectedEdge because we'll need its shape
  graph_tile_ptr tile = graphreader.GetGraphTile(pred.edgeid().Tile_Base());
  const DirectedEdge* edge = tile->directededge(pred.edgeid());

  // Transit lines and ferries can't really be "reached" you really just
  // pass through those cells.
  if (edge->IsTransitLine() || edge->use() == Use::kFerry) {
    return;
  }

  // Get the time and distance at the end node of the predecessor
  float secs1 = pred.cost().secs;
  float dist1 = static_cast<float>(pred.path_distance());

  // For short edges just mark the segment between the 2 nodes of the edge. This
  // avoid getting the shape for short edges.
  auto len = pred.origin() ? pred.path_distance() : edge->length();
  if (len < shape_interval_ * 1.5f) {
    PointLL ll0 = tile->get_node_ll(t2->directededge(opp)->endnode());
    if (pred.origin()) {
      // interpolate ll0 for origin edge using edge_label.path_distance()
      auto edge_info = tile->edgeinfo(edge);
      const auto& shape = edge_info.shape();
      const auto& ordered_shape =
          edge->forward() ? shape : std::vector<midgard::PointLL>(shape.rbegin(), shape.rend());
      auto origin_edge_shape = OriginEdgeShape(ordered_shape, pred.path_distance());
      ll0 = origin_edge_shape.front();
    }
    UpdateIsoTileAlongSegment(ll0, ll, secs1, dist1);
    return;
  }

  // Get the shape and make sure shape is forward direction. Resample it to
  // the shape interval to get regular spacing. Use the faster resample method.
  // This does not use spherical interpolation - so it is not as accurate but
  // interpolation is over short distances so accuracy should be fine.
  auto edge_info = tile->edgeinfo(edge);
  const auto& shape = edge_info.shape();
  auto resampled = resample_polyline(shape, edge->length(), shape_interval_);
  if (!edge->forward()) {
    std::reverse(resampled.begin(), resampled.end());
  }
  if (pred.origin()) {
    resampled = OriginEdgeShape(resampled, pred.path_distance());
  }

  // Mark grid cells along the shape if time is less than what is
  // already populated. Get intersection of tiles along each segment
  // (just use a bounding box around the segment) so this doesn't miss
  // shape that crosses tile corners
  float seconds = secs0;
  float meters = dist0;
  float delta_seconds = ((secs1 - secs0) / (resampled.size() - 1));
  float delta_meters = ((dist1 - dist0) / (resampled.size() - 1));
  auto itr1 = resampled.begin();
  for (auto itr2 = itr1 + 1; itr2 < resampled.end(); itr1++, itr2++) {
    seconds += delta_seconds;
    meters += delta_meters;
    UpdateIsoTileAlongSegment(*itr1, *itr2, seconds, meters);
  }
}

// here we mark the cells of the isochrone along the edge we just reached up to its end node
void Isochrone::ExpandingNode(baldr::GraphReader& graphreader,
                              graph_tile_ptr tile,
                              const baldr::NodeInfo* node,
                              const sif::EdgeLabel& current,
                              const sif::EdgeLabel* previous) {
  // Update the isotile
  float secs0 = previous ? previous->cost().secs : 0.0f;
  float dist0 = previous ? static_cast<float>(previous->path_distance()) : 0.0f;
  UpdateIsoTile(current, graphreader, node->latlng(tile->header()->base_ll()), secs0, dist0);
}

ExpansionRecommendation Isochrone::ShouldExpand(baldr::GraphReader& /*graphreader*/,
                                                const sif::EdgeLabel& pred,
                                                const ExpansionType route_type) {
  float time;
  uint32_t dist;
  if (route_type == ExpansionType::multimodal) {
    // Skip edges with large penalties (e.g. ferries?), MMCompute function will skip expanding this
    // label
    if (pred.cost().cost > max_seconds_ * 2) {
      return ExpansionRecommendation::prune_expansion;
    }
    time = pred.predecessor() == kInvalidLabel ? 0.f : mmedgelabels_[pred.predecessor()].cost().secs;
    dist =
        pred.predecessor() == kInvalidLabel ? 0 : mmedgelabels_[pred.predecessor()].path_distance();
  } else {
    time = pred.predecessor() == kInvalidLabel ? 0.f : bdedgelabels_[pred.predecessor()].cost().secs;
    dist =
        pred.predecessor() == kInvalidLabel ? 0 : bdedgelabels_[pred.predecessor()].path_distance();
  }

  // Continue if the time and distance intervals have been met. This bus or rail line goes beyond the
  // max but need to consider others so we just continue here. Tells MMExpand function to skip
  // updating or pushing the label back
  // prune the edge if its start is above max contour

  ExpansionRecommendation recommendation = (time > max_seconds_ && dist > max_meters_)
                                               ? ExpansionRecommendation::prune_expansion
                                               : ExpansionRecommendation::continue_expansion;

  // track expansion
  if (inner_expansion_callback_ && (time <= (max_seconds_ - METRIC_PADDING * kSecondsPerMinute) ||
                                    dist <= (max_meters_ - METRIC_PADDING * kMetersPerKm))) {
    if (!expansion_callback_) {
      expansion_callback_ = inner_expansion_callback_;
    }
  } else if (expansion_callback_) {
    expansion_callback_ = nullptr;
  }
  return recommendation;
};

void Isochrone::GetExpansionHints(uint32_t& bucket_count, uint32_t& edge_label_reservation) const {
  bucket_count = 20000;
  edge_label_reservation = kInitialEdgeLabelCountDijkstras;
}

} // namespace thor
} // namespace valhalla
