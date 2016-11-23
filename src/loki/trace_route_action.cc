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

    worker_t::result_t loki_worker_t::trace_route(boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      init_trace(request);

      //pass it on to thor
      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());
      return result;
    }

    void loki_worker_t::locations_from_shape(boost::property_tree::ptree& request) {
      auto shape = request.get_child("shape");

      // TODO should we process more than two points?
      auto first = Location::FromPtree(shape.begin()->second);
      auto second = Location::FromPtree(std::next(shape.begin())->second);
      auto next_to_last = Location::FromPtree(std::prev(shape.end(), 2)->second);
      auto last = Location::FromPtree(std::prev(shape.end())->second);

      // Set heading for first
      first.heading_ = std::round(
          PointLL::HeadingAlongPolyline(
              { { first.latlng_.lng(), first.latlng_.lat() },
                { second.latlng_.lng(), second.latlng_.lat() } },
                30.f));

      // Set heading for last
      last.heading_ = std::round(
          PointLL::HeadingAtEndOfPolyline(
              { { next_to_last.latlng_.lng(), next_to_last.latlng_.lat() },
                { last.latlng_.lng(), last.latlng_.lat() } },
                30.f));

      // Add first and last locations to request
      boost::property_tree::ptree locations_child;
      locations_child.push_back(std::make_pair("", first.ToPtree()));
      locations_child.push_back(std::make_pair("", last.ToPtree()));

      request.put_child("locations", locations_child);

      request.put_child("correlated_0", loki::Search(first, reader, edge_filter, node_filter).ToPtree(0));
      request.put_child("correlated_1", loki::Search(last, reader, edge_filter, node_filter).ToPtree(1));

    }

  }
}
