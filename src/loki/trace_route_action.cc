#include "loki/worker.h"
#include "loki/search.h"

#include "midgard/pointll.h"
#include "midgard/logging.h"
#include "midgard/encoded.h"
#include "baldr/rapidjson_utils.h"
#include "tyr/actor.h"

#include <cmath>

using namespace valhalla;
using namespace valhalla::tyr;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::loki;

namespace {
PointLL to_ll(const odin::Location& l) {
  return PointLL{l.ll().lng(), l.ll().lat()};
}

void check_shape(const google::protobuf::RepeatedPtrField<odin::Location>& shape, unsigned int max_shape,
                 float max_factor = 1.0f) {
  // Adjust max - this enables max edge_walk shape count to be larger
  max_shape *= max_factor;

  // Must have at least two points
  if (shape.size() < 2)
    throw valhalla_exception_t{123};
  // Validate shape is not larger than the configured max
  else if (shape.size() > max_shape)
    throw valhalla_exception_t{153, "(" + std::to_string(shape.size()) +"). The limit is " + std::to_string(max_shape)};

  valhalla::midgard::logging::Log(
      "trace_size::" + std::to_string(shape.size()), " [ANALYTICS] ");

}

void check_distance(const google::protobuf::RepeatedPtrField<odin::Location>& shape, float max_distance,
                    float max_factor = 1.0f) {
  // Adjust max - this enables max edge_walk distance to be larger
  max_distance *= max_factor;

  // Calculate "crow distance" of shape
  auto crow_distance = to_ll(*shape.begin()).Distance(to_ll(*shape.rbegin()));

  if (crow_distance > max_distance)
    throw valhalla_exception_t { 154 };

  valhalla::midgard::logging::Log(
      "location_distance::" + std::to_string(crow_distance * kKmPerMeter) + "km", " [ANALYTICS] ");
}

void check_best_paths(unsigned int best_paths, unsigned int max_best_paths) {

  // Validate the best paths count is not less than 1
  if (best_paths < 1)
    throw valhalla_exception_t { 158, "(" + std::to_string(best_paths) + "). The best_paths lower limit is 1" };

  // Validate the best paths count is not larger than the configured best paths max
  if (best_paths > max_best_paths)
    throw valhalla_exception_t { 158, "(" + std::to_string(best_paths) + "). The best_paths upper limit is " + std::to_string(max_best_paths) };
}

void check_best_paths_shape(unsigned int best_paths,
    const google::protobuf::RepeatedPtrField<odin::Location>& shape, size_t max_best_paths_shape) {

  // Validate shape is not larger than the configured best paths shape max
  if ((best_paths > 1) && (shape.size() > max_best_paths_shape))
    throw valhalla_exception_t { 153, "(" + std::to_string(shape.size()) + "). The best paths shape limit is " + std::to_string(max_best_paths_shape) };
}

void check_gps_accuracy(const float input_gps_accuracy, const float max_gps_accuracy) {
  if (input_gps_accuracy > max_gps_accuracy || input_gps_accuracy < 0.f)
    throw valhalla_exception_t { 158 };

  valhalla::midgard::logging::Log(
      "gps_accuracy::" + std::to_string(input_gps_accuracy) + "meters", " [ANALYTICS] ");
}

void check_search_radius(const float input_search_radius, const float max_search_radius) {
  if (input_search_radius > max_search_radius || input_search_radius < 0.f)
    throw valhalla_exception_t { 158 };

  valhalla::midgard::logging::Log(
      "search_radius::" + std::to_string(input_search_radius) + "meters", " [ANALYTICS] ");
}

void check_turn_penalty_factor(const float input_turn_penalty_factor) {
  if (input_turn_penalty_factor < 0.f)
    throw valhalla_exception_t { 158 };
}
}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_trace(valhalla_request_t& request) {
      parse_costing(request);

      //we require shape or encoded polyline but we dont know which at first
      if(!request.options.shape_size())
        throw valhalla_exception_t{114};

      // Determine max factor, defaults to 1. This factor is used to increase
      // the max value when an edge_walk shape match is requested
      float max_factor = 1.0f;
      std::string shape_match = rapidjson::GetValueByPointerWithDefault(request.document, "/shape_match", "walk_or_snap").GetString();
      if (shape_match == "edge_walk")
        max_factor = 5.0f;

      // Validate shape count and distance (for now, just send max_factor for distance)
      check_shape(request.options.shape(), max_trace_shape);
      check_distance(request.options.shape(), max_distance.find("trace")->second, max_factor);

      // Validate best paths and best paths shape for `map_snap` requests
      if  (shape_match == "map_snap") {
        unsigned int best_paths = rapidjson::GetValueByPointerWithDefault(request.document, "/best_paths", 1).GetUint();
        check_best_paths(best_paths, max_best_paths);
        check_best_paths_shape(best_paths, request.options.shape(), max_best_paths_shape);
      }

      // Validate optional trace options
      auto input_gps_accuracy = rapidjson::get_optional<float>(request.document, "/trace_options/gps_accuracy");
      auto input_search_radius = rapidjson::get_optional<float>(request.document, "/trace_options/search_radius");
      auto input_turn_penalty_factor = rapidjson::get_optional<float>(request.document, "/trace_options/turn_penalty_factor");
      if (input_gps_accuracy)
        check_gps_accuracy(*input_gps_accuracy, max_gps_accuracy);
      if (input_search_radius)
        check_search_radius(*input_search_radius, max_search_radius);
      if (input_turn_penalty_factor)
        check_turn_penalty_factor(*input_turn_penalty_factor);

      // Set locations after parsing the shape
      locations_from_shape(request);
    }

    void loki_worker_t::trace(valhalla_request_t& request) {
      init_trace(request);
      std::string costing = request.document["costing"].GetString();
      if (costing == "multimodal")
        throw valhalla_exception_t{140, odin::DirectionsOptions::Action_Name(request.options.action())};
    }

    void loki_worker_t::locations_from_shape(valhalla_request_t& request) {
      std::vector<Location> locations{PathLocation::fromPBF(*request.options.shape().begin()),
        PathLocation::fromPBF(*request.options.shape().rbegin())};
      locations.front().node_snap_tolerance_ = 0.f;
      locations.front().radius_ = 10;
      locations.back().node_snap_tolerance_ = 0.f;
      locations.back().radius_ = 10;

      // Add first and last correlated locations to request
      try{
        auto projections = loki::Search(locations, reader, edge_filter, node_filter);
        request.options.clear_locations();
        PathLocation::toPBF(projections.at(locations.front()), request.options.mutable_locations()->Add(), reader);
        PathLocation::toPBF(projections.at(locations.back()), request.options.mutable_locations()->Add(), reader);
      }
      catch(const std::exception&) {
        throw valhalla_exception_t{171};
      }
    }

  }
}
