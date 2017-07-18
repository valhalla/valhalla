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
void check_shape(const std::vector<PointLL>& shape, unsigned int max_shape,
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

void check_distance(const std::vector<PointLL>& shape, float max_distance,
                    float max_factor = 1.0f) {
  // Adjust max - this enables max edge_walk distance to be larger
  max_distance *= max_factor;

  // Calculate "crow distance" of shape
  auto crow_distance = shape.front().Distance(shape.back());

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
    const std::vector<PointLL>& shape, size_t max_best_paths_shape) {

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

    void loki_worker_t::init_trace(rapidjson::Document& request) {
      parse_costing(request);
      parse_trace(request);

      // Determine max factor, defaults to 1. This factor is used to increase
      // the max value when an edge_walk shape match is requested
      float max_factor = 1.0f;
      std::string shape_match = rapidjson::GetValueByPointerWithDefault(request, "/shape_match", "walk_or_snap").GetString();
      if (shape_match == "edge_walk")
        max_factor = 5.0f;

      // Validate shape count and distance (for now, just send max_factor for distance)
      check_shape(shape, max_shape);
      check_distance(shape, max_distance.find("trace")->second, max_factor);

      // Validate best paths and best paths shape for `map_snap` requests
      if  (shape_match == "map_snap") {
        unsigned int best_paths = rapidjson::GetValueByPointerWithDefault(request, "/best_paths", 1).GetUint();
        check_best_paths(best_paths, max_best_paths);
        check_best_paths_shape(best_paths, shape, max_best_paths_shape);
      }

      // Validate optional trace options
      auto input_gps_accuracy = GetOptionalFromRapidJson<float>(request, "/trace_options/gps_accuracy");
      auto input_search_radius = GetOptionalFromRapidJson<float>(request, "/trace_options/search_radius");
      auto input_turn_penalty_factor = GetOptionalFromRapidJson<float>(request, "/trace_options/turn_penalty_factor");
      if (input_gps_accuracy)
        check_gps_accuracy(*input_gps_accuracy, max_gps_accuracy);
      if (input_search_radius)
        check_search_radius(*input_search_radius, max_search_radius);
      if (input_turn_penalty_factor)
        check_turn_penalty_factor(*input_turn_penalty_factor);

      // Set locations after parsing the shape
      locations_from_shape(request);
    }

    void loki_worker_t::trace(ACTION_TYPE action, rapidjson::Document& request) {
      init_trace(request);
      std::string costing = request["costing"].GetString();
      if (costing == "multimodal")
        throw valhalla_exception_t{140, ACTION_TO_STRING.find(action)->second};
    }

    void loki_worker_t::parse_trace(rapidjson::Document& request) {
      auto& allocator = request.GetAllocator();
      //we require uncompressed shape or encoded polyline
      auto input_shape = GetOptionalFromRapidJson<rapidjson::Value::Array>(request, "/shape");
      auto encoded_polyline = GetOptionalFromRapidJson<std::string>(request, "/encoded_polyline");
      //we require shape or encoded polyline but we dont know which at first
      try {
        //uncompressed shape
        if (input_shape) {
          for (const auto& latlng : *input_shape) {
            shape.push_back(Location::FromRapidJson(latlng).latlng_);
          }
        }//compressed shape
        //if we receive as encoded then we need to add as shape to request
        else if (encoded_polyline) {
          shape = midgard::decode<std::vector<midgard::PointLL> >(*encoded_polyline);
          rapidjson::Value shape_array{rapidjson::kArrayType};
          for(const auto& pt : shape) {
            rapidjson::Value point_child{rapidjson::kObjectType};
            point_child.AddMember("lon", static_cast<double>(pt.first), allocator).
                AddMember("lat", static_cast<double>(pt.second), allocator);
            shape_array.PushBack(point_child, allocator);
          }
          request.AddMember("shape", shape_array, allocator);
        }/* else if (gpx) {
          //TODO:Add support
        } else if (geojson){
          //TODO:Add support
        }*/
        else
          throw valhalla_exception_t{126};
      }
      catch (const std::exception& e) {
        //TODO: pass on e.what() to generic exception
        throw valhalla_exception_t{114};
      }
    }

    void loki_worker_t::locations_from_shape(rapidjson::Document& request) {
      std::vector<Location> locations{shape.front(), shape.back()};
      locations.front().heading_ = std::round(PointLL::HeadingAlongPolyline(shape, 30.f));
      locations.back().heading_ = std::round(PointLL::HeadingAtEndOfPolyline(shape, 30.f));

      // Add first and last locations to request
      auto& allocator = request.GetAllocator();
      rapidjson::Value locations_child{rapidjson::kArrayType};
      locations_child.PushBack(locations.front().ToRapidJson(allocator), allocator);
      locations_child.PushBack(locations.back().ToRapidJson(allocator), allocator);
      rapidjson::Pointer("/locations").Set(request, locations_child);

      // Add first and last correlated locations to request
      try{
        auto projections = loki::Search(locations, reader, edge_filter, node_filter);
        rapidjson::Pointer("/correlated_0").Set(request, projections.at(locations.front()).ToRapidJson(0, allocator));
        rapidjson::Pointer("/correlated_1").Set(request, projections.at(locations.back()).ToRapidJson(1, allocator));
      }
      catch(const std::exception&) {
        throw valhalla_exception_t{171};
      }
    }

  }
}
