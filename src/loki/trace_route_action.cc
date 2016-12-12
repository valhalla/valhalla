#include "loki/service.h"
#include "loki/search.h"

#include <valhalla/midgard/pointll.h>

#include <boost/property_tree/json_parser.hpp>
#include <math.h>

using namespace prime_server;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {


}

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_trace(boost::property_tree::ptree& request) {
      parse_costing(request);
      parse_trace(request);
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
