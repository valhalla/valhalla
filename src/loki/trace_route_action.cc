#include "loki/search.h"
#include "loki/worker.h"

#include "baldr/rapidjson_utils.h"
#include "midgard/encoded.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "tyr/actor.h"

#include <cmath>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::loki;

namespace {
PointLL to_ll(const valhalla::Location& l) {
  return PointLL{l.ll().lng(), l.ll().lat()};
}

void check_shape(const google::protobuf::RepeatedPtrField<valhalla::Location>& shape,
                 unsigned int max_shape,
                 float max_factor = 1.0f) {
  // Adjust max - this enables max edge_walk shape count to be larger
  max_shape *= max_factor;

  // Must have at least two points
  if (shape.size() < 2) {
    throw valhalla_exception_t{123};
    // Validate shape is not larger than the configured max
  } else if (shape.size() > max_shape) {
    throw valhalla_exception_t{153, "(" + std::to_string(shape.size()) + "). The limit is " +
                                        std::to_string(max_shape)};
  };
}

void check_distance(const google::protobuf::RepeatedPtrField<valhalla::Location>& shape,
                    float max_distance,
                    float max_breakage_distance,
                    float max_factor = 1.0f) {

  // Adjust max - this enables max edge_walk distance to be larger
  max_distance *= max_factor;

  bool can_be_matched = false;
  float crow_distance = 0.f;
  for (auto iter = shape.begin(); iter < shape.end() - 1; ++iter) {
    // We bail when the distance between two locations exceeds the threshold
    PointLL curr_point = to_ll(*iter);
    PointLL next_point = to_ll(*std::next(iter));
    float point_to_point_distance = curr_point.Distance(next_point);
    crow_distance += point_to_point_distance;

    if (point_to_point_distance <= max_breakage_distance) {
      can_be_matched = true;
    }

    if (crow_distance > max_distance) {
      throw valhalla_exception_t{154, std::to_string(static_cast<size_t>(max_distance)) + " meters"};
    }
  }

  if (!can_be_matched) {
    throw valhalla_exception_t{172, std::to_string(static_cast<size_t>(max_breakage_distance)) +
                                        " meters"};
  }
}

void check_best_paths(unsigned int best_paths, unsigned int max_best_paths) {

  // Validate the best paths count is not less than 1
  if (best_paths < 1) {
    throw valhalla_exception_t{158, "(" + std::to_string(best_paths) +
                                        "). The best_paths lower limit is 1"};
  };

  // Validate the best paths count is not larger than the configured best paths max
  if (best_paths > max_best_paths) {
    throw valhalla_exception_t{158, "(" + std::to_string(best_paths) +
                                        "). The best_paths upper limit is " +
                                        std::to_string(max_best_paths)};
  }
}

void check_alternates_shape(unsigned int alternates,
                            const google::protobuf::RepeatedPtrField<valhalla::Location>& shape,
                            size_t max_alternates_shape) {

  // Validate shape is not larger than the configured best paths shape max
  if ((alternates > 1) && (shape.size() > max_alternates_shape)) {
    throw valhalla_exception_t{153, "(" + std::to_string(shape.size()) +
                                        "). The best paths shape limit is " +
                                        std::to_string(max_alternates_shape)};
  }
}

void check_gps_accuracy(const float input_gps_accuracy, const float max_gps_accuracy) {
  if (input_gps_accuracy > max_gps_accuracy || input_gps_accuracy < 0.f) {
    throw valhalla_exception_t{158};
  }
}

void check_search_radius(const float input_search_radius, const float max_search_radius) {
  if (input_search_radius > max_search_radius || input_search_radius < 0.f) {
    throw valhalla_exception_t{158};
  }
}

void check_turn_penalty_factor(const float input_turn_penalty_factor) {
  if (input_turn_penalty_factor < 0.f) {
    throw valhalla_exception_t{158};
  }
}
} // namespace

namespace valhalla {
namespace loki {

void loki_worker_t::init_trace(Api& request) {
  parse_costing(request);
  auto& options = *request.mutable_options();

  // we require shape or encoded polyline but we dont know which at first
  if (!options.shape_size()) {
    throw valhalla_exception_t{114};
  }

  // Determine max factor, defaults to 1. This factor is used to increase
  // the max value when an edge_walk shape match is requested
  float max_factor = 1.0f;
  if (options.shape_match() == ShapeMatch::edge_walk) {
    max_factor = 5.0f;
  }

  // Validate shape count and distance (for now, just send max_factor for distance)
  check_shape(options.shape(), max_trace_shape);
  float breakage_distance =
      options.has_breakage_distance_case() ? options.breakage_distance() : default_breakage_distance;
  check_distance(options.shape(), max_distance.find("trace")->second, breakage_distance, max_factor);

  // Validate best paths and best paths shape for `map_snap` requests
  if (options.shape_match() == ShapeMatch::map_snap) {
    check_alternates_shape(options.alternates() + 1, options.shape(), max_trace_alternates_shape);
  }

  // Validate optional trace options
  if (options.has_gps_accuracy_case()) {
    check_gps_accuracy(options.gps_accuracy(), max_gps_accuracy);
  }
  if (options.has_search_radius_case()) {
    check_search_radius(options.search_radius(), max_search_radius);
  }
  if (options.has_turn_penalty_factor_case()) {
    check_turn_penalty_factor(options.turn_penalty_factor());
  }

  // Set locations after parsing the shape
  locations_from_shape(request);
}

void loki_worker_t::trace(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  init_trace(request);
  if (request.options().costing_type() == Costing::multimodal) {
    throw valhalla_exception_t{140, Options_Action_Enum_Name(request.options().action())};
  };
}

// TODO: remove this, it was a hack to support display_ll for map matching, what we can do is
// actually use the display_ll now as its  used in loki::Search
void loki_worker_t::locations_from_shape(Api& request) {
  auto& options = *request.mutable_options();
  std::vector<baldr::Location> locations{PathLocation::fromPBF(*options.shape().begin()),
                                         PathLocation::fromPBF(*options.shape().rbegin())};
  locations.front().node_snap_tolerance_ = 0.f;
  locations.front().radius_ = 10;
  locations.back().node_snap_tolerance_ = 0.f;
  locations.back().radius_ = 10;

  // Add first and last correlated locations to request
  try {
    // If trace route has 2 locations get the lat,lng of the first and last so we can support
    // side of street.
    bool has_locations = false;
    PointLL orig_ll, dest_ll;
    if (options.locations_size() == 2) {
      has_locations = true;
      orig_ll = {options.locations(0).ll().lng(), options.locations(0).ll().lat()};
      dest_ll = {options.locations(1).ll().lng(), options.locations(1).ll().lat()};
    }

    // Project first and last shape point onto nearest edge(s). Clear current locations list
    // and set the path locations
    auto projections = loki::Search(locations, *reader, costing);
    options.clear_locations();
    PathLocation::toPBF(projections.at(locations.front()), options.mutable_locations()->Add(),
                        *reader);
    PathLocation::toPBF(projections.at(locations.back()), options.mutable_locations()->Add(),
                        *reader);

    // If locations were provided, backfill the origin and dest lat,lon and update
    // side of street on associated edges. TODO - create a constant for side of street
    // distance
    if (has_locations) {
      // Update the location lat,lon and fill in new side of street to each edge
      auto orig = options.mutable_locations(0);
      orig->mutable_ll()->set_lng(orig_ll.lng());
      orig->mutable_ll()->set_lat(orig_ll.lat());
      for (auto& e : *orig->mutable_correlation()->mutable_edges()) {
        GraphId edgeid(e.graph_id());
        graph_tile_ptr tile = reader->GetGraphTile(edgeid);
        const DirectedEdge* de = tile->directededge(edgeid);
        auto shape = tile->edgeinfo(de).shape();
        auto closest = orig_ll.ClosestPoint(shape);

        // TODO - consider consolidating side of street logic into a common method
        // (perhaps in baldr that can be called here and in search.cc get_side)

        // Close to centerline - set to no side of street
        if (std::get<1>(closest) < 5.0f) {
          e.set_side_of_street(valhalla::Location::kNone);
          continue;
        }

        // Check side of street of the origin location
        int idx = std::get<2>(closest);
        LineSegment2<PointLL> segment(shape[idx], shape[idx + 1]);
        bool is_left = segment.IsLeft(orig_ll) > 0;
        e.set_side_of_street(is_left == de->forward() ? valhalla::Location::kLeft
                                                      : valhalla::Location::kRight);
      }

      auto dest = options.mutable_locations(1);
      dest->mutable_ll()->set_lng(dest_ll.lng());
      dest->mutable_ll()->set_lat(dest_ll.lat());
      for (auto& e : *dest->mutable_correlation()->mutable_edges()) {
        GraphId edgeid(e.graph_id());
        graph_tile_ptr tile = reader->GetGraphTile(edgeid);
        const DirectedEdge* de = tile->directededge(edgeid);
        auto shape = tile->edgeinfo(de).shape();
        auto closest = dest_ll.ClosestPoint(shape);

        // TODO - consider consolidating side of street logic into a common method
        // (perhaps in baldr that can be called here and in search.cc get_side)

        // Close to centerline - set to no side of street
        if (std::get<1>(closest) < 5.0f) {
          e.set_side_of_street(valhalla::Location::kNone);
          continue;
        }

        // Check side of street of the destination location
        int idx = std::get<2>(closest);
        LineSegment2<PointLL> segment(shape[idx], shape[idx + 1]);
        bool is_left = segment.IsLeft(dest_ll) > 0;
        e.set_side_of_street(is_left == de->forward() ? valhalla::Location::kLeft
                                                      : valhalla::Location::kRight);
      }
    }
  } catch (const std::exception&) { throw valhalla_exception_t{171}; }
}

} // namespace loki
} // namespace valhalla
