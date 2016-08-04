#include "loki/service.h"
#include "loki/search.h"

#include <boost/property_tree/info_parser.hpp>

#include <valhalla/baldr/json.h>
#include <valhalla/baldr/datetime.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>

using namespace prime_server;
using namespace valhalla::baldr;

namespace valhalla {
  namespace loki {

    worker_t::result_t loki_worker_t::isochrones(boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      //check that location size does not exceed max
      if (locations.size() > max_locations.find("isochrone")->second)
        throw std::runtime_error("Exceeded max locations of " + std::to_string(max_locations.find("isochrone")->second) + ".");

      //correlate the various locations to the underlying graph
      for(size_t i = 0; i < locations.size(); ++i) {
        auto correlated = loki::Search(locations[i], reader, edge_filter, node_filter);
        request.put_child("correlated_" + std::to_string(i), correlated.ToPtree(i));
      }

      //let thor know this is isolines
      request.put("isochrone", 1);
      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());

      return result;
    }

  }
}
