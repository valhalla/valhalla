#include "loki/service.h"
#include "loki/search.h"

#include <boost/property_tree/json_parser.hpp>

using namespace prime_server;
using namespace valhalla::baldr;

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_isochrones(const boost::property_tree::ptree& request) {
      parse_locations(request);
      if(locations.size() < 1)
        throw std::runtime_error("Insufficient number of locations provided");
      //make sure the isoline definitions are valid
      auto contours = request.get_child_optional("contours");
      if(!contours)
        throw std::runtime_error("Insufficiently specified required parameter 'contours'");
      //check that the number of contours is ok
      if(contours->size() > max_contours)
        throw std::runtime_error("Exceeded max contours of " + std::to_string(max_contours) + ".");
      size_t prev = 0;
      for(const auto& contour : *contours) {
        auto c = contour.second.get<size_t>("time", -1);
        if(c < prev || c == -1)
          throw std::runtime_error("Insufficiently specified required parameter 'time'");
        if(c > max_time)
          throw std::runtime_error("Exceeded max time of " + std::to_string(max_time) + ".");
        prev = c;
      }
      parse_costing(request);
    }

    worker_t::result_t loki_worker_t::isochrones(boost::property_tree::ptree& request, http_request_t::info_t& request_info) {
      init_isochrones(request);
      //check that location size does not exceed max
      if (locations.size() > max_locations.find("isochrone")->second)
        throw std::runtime_error("Exceeded max locations of " + std::to_string(max_locations.find("isochrone")->second) + ".");

      //correlate the various locations to the underlying graph
      for(size_t i = 0; i < locations.size(); ++i) {
        auto correlated = loki::Search(locations[i], reader, edge_filter, node_filter);
        request.put_child("correlated_" + std::to_string(i), correlated.ToPtree(i));
      }

      //pass it on
      std::stringstream stream;
      boost::property_tree::write_json(stream, request, false);
      worker_t::result_t result{true};
      result.messages.emplace_back(stream.str());

      return result;
    }

  }
}
