#include "loki/service.h"
#include "loki/search.h"

#include <boost/property_tree/json_parser.hpp>

using namespace prime_server;
using namespace valhalla::baldr;

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_trace(boost::property_tree::ptree& request) {
      parse_costing(request);
      parse_trace(request);
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

  }
}
