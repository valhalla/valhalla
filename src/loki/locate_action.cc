#include "loki/worker.h"
#include "loki/search.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace valhalla {
  namespace loki {

    void loki_worker_t::init_locate(valhalla_request_t& request) {
      locations = parse_locations(request, "locations");
      if(locations.size() < 1)
        throw valhalla_exception_t{120};
      if(request.document.HasMember("costing"))
        parse_costing(request);
      else {
        edge_filter = loki::PassThroughEdgeFilter;
        node_filter = loki::PassThroughNodeFilter;
      }
    }

    std::string loki_worker_t::locate(valhalla_request_t& request) {
      //correlate the various locations to the underlying graph
      init_locate(request);
      auto projections = loki::Search(locations, reader, edge_filter, node_filter);
      return tyr::serializeLocate(request, locations, projections, reader);
    }

  }
}
