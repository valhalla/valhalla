#include "loki/service.h"
#include "loki/search.h"

#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/encoded.h>

#include <boost/property_tree/json_parser.hpp>
#include <math.h>

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {

void check_shape(const std::vector<PointLL>& shape, unsigned int max_shape,
                 float max_factor) {
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
                    float max_factor) {
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

    void loki_worker_t::init_trace(boost::property_tree::ptree& request) {
      parse_costing(request);
      parse_trace(request);

      // Determine max factor, defaults to 1. This factor is used to increase
      // the max value when an edge_walk shape match is requested
      float max_factor = 1.0f;
      auto shape_match = request.get<std::string>("shape_match", "walk_or_snap");
      if (shape_match == "edge_walk")
        max_factor = 8.0f;

      // Validate shape count and distance
      check_shape(shape, max_shape, max_factor);
      check_distance(shape, max_distance.find("trace")->second, max_factor);

      // Set locations after parsing the shape
      locations_from_shape(request);
    }

    worker_t::result_t loki_worker_t::trace_route(boost::property_tree::ptree& request, http_request_info_t& request_info) {
      init_trace(request);

      //pass it on to thor
      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());
      return result;
    }

    void loki_worker_t::parse_trace(boost::property_tree::ptree& request) {
      //we require uncompressed shape or encoded polyline
      auto input_shape = request.get_child_optional("shape");
      auto encoded_polyline = request.get_optional<std::string>("encoded_polyline");

      //we require shape or encoded polyline but we dont know which at first
      try {
        //uncompressed shape
        if (input_shape) {
          for (const auto& latlng : *input_shape) {
            shape.push_back(Location::FromPtree(latlng.second).latlng_);
          }
        }//compressed shape
        //if we receive as encoded then we need to add as shape to request
        else if (encoded_polyline) {
          shape = midgard::decode<std::vector<midgard::PointLL> >(*encoded_polyline);
          boost::property_tree::ptree shape_child;
          for(const auto& pt : shape) {
            boost::property_tree::ptree point_child;
            point_child.put("lon", static_cast<double>(pt.first));
            point_child.put("lat", static_cast<double>(pt.second));
            shape_child.push_back(std::make_pair("", point_child));
          }
          request.add_child("shape", shape_child);
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

    void loki_worker_t::locations_from_shape(boost::property_tree::ptree& request) {
      std::vector<Location> locations{shape.front(), shape.back()};
      locations.front().heading_ = std::round(PointLL::HeadingAlongPolyline(shape, 30.f));
      locations.back().heading_ = std::round(PointLL::HeadingAtEndOfPolyline(shape, 30.f));

      // Add first and last locations to request
      boost::property_tree::ptree locations_child;
      locations_child.push_back(std::make_pair("", locations.front().ToPtree()));
      locations_child.push_back(std::make_pair("", locations.back().ToPtree()));
      request.put_child("locations", locations_child);

      // Add first and last correlated locations to request
      auto projections = loki::Search(locations, reader, edge_filter, node_filter);
      request.put_child("correlated_0", projections.at(locations.front()).ToPtree(0));
      request.put_child("correlated_1", projections.at(locations.back()).ToPtree(1));

    }

  }
}
