#include "loki/service.h"
#include "loki/search.h"

#include "midgard/pointll.h"
#include "midgard/logging.h"
#include "midgard/encoded.h"
#include "baldr/rapidjson_utils.h"

#include <boost/property_tree/json_parser.hpp>
#include <math.h>

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

void check_shape(const std::vector<PointLL>& shape, unsigned int max_shape,
                 float max_factor = 1.0f) {
  // Adjust max - this enables max edge_walk shape count to be larger
  max_shape *= max_factor;

  // Must have at least two points
  if (shape.size() < 2)
    throw valhalla_exception_t{400, 123};
  // Validate shape is not larger than the configured max
  else if (shape.size() > max_shape)
    throw valhalla_exception_t{400, 153, "(" + std::to_string(shape.size()) +"). The limit is " + std::to_string(max_shape)};

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
    throw valhalla_exception_t { 400, 154 };

  valhalla::midgard::logging::Log(
      "location_distance::" + std::to_string(crow_distance * kKmPerMeter) + "km", " [ANALYTICS] ");
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

      // Set locations after parsing the shape
      locations_from_shape(request);
    }

    worker_t::result_t loki_worker_t::trace_route(rapidjson::Document& request, http_request_info_t& request_info) {
      init_trace(request);

      //pass it on to thor
      worker_t::result_t result{true};
      result.messages.emplace_back(rapidjson::to_string(request));
      return result;
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
          throw valhalla_exception_t{400, 126};
      }
      catch (const std::exception& e) {
        //TODO: pass on e.what() to generic exception
        throw valhalla_exception_t{400, 114};
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
        throw valhalla_exception_t{400, 171};
      }
    }

  }
}
