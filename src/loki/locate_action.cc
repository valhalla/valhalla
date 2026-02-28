#include "loki/search.h"
#include "loki/worker.h"
#include "tyr/serializers.h"

using namespace valhalla;
using namespace valhalla::baldr;

namespace valhalla {
namespace loki {

void loki_worker_t::init_locate(Api& request) {
  parse_locations(request.mutable_options()->mutable_locations(), request);
  if (request.options().locations_size() < 1)
    throw valhalla_exception_t{120};

  parse_costing(request, true);
}

std::string loki_worker_t::locate(Api& request) {
  // time this whole method and save that statistic
  auto _ = measure_scope_time(request);

  // correlate the various locations to the underlying graph
  init_locate(request);
  search_.search(*request.mutable_options()->mutable_locations(), costing);
  size_t i = 0;
  for (const auto& loc : request.options().locations()) {
    LOG_ERROR("Correlated edges for location {}: {}", i, loc.correlation().edges_size());
    ++i;
  }
  return tyr::serializeLocate(request, *reader);
}

} // namespace loki
} // namespace valhalla
